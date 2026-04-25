from __future__ import annotations

import atexit
from dataclasses import dataclass
import math
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
from typing import Literal
from xml.sax.saxutils import escape

import numpy as np

from .common_types import ActionPrimitive, SimulationTrajectory, VehicleParams

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.msg import Contacts
    from ros_gz_interfaces.srv import DeleteEntity
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.srv import SpawnEntity
except ImportError:
    rclpy = None
    Twist = None
    Odometry = None
    Entity = None
    Contacts = None
    DeleteEntity = None
    SetEntityPose = None
    SpawnEntity = None


@dataclass(frozen=True)
class SimulationContext:
    heightmap: np.ndarray
    heading_rad: float
    vehicle: VehicleParams
    action: ActionPrimitive
    friction_mu: float
    motion_model: Literal["skid", "ackermann"]
    sample_rate_hz: int
    duration_sec: float
    settle_time_sec: float = 0.5


class SimulationRunner:
    def run(self, context: SimulationContext, rng: np.random.Generator) -> SimulationTrajectory:
        raise NotImplementedError


@dataclass(frozen=True)
class RosGzRuntimeConfig:
    world_sdf_file: str = "worlds/ml25d_empty.sdf"
    world_name: str = "ml25d"
    model_name: str = "ml25d_vehicle"
    startup_timeout_sec: float = 20.0
    service_timeout_sec: float = 5.0
    auto_start_processes: bool = True
    fallback_to_mock_on_error: bool = False


@dataclass(frozen=True)
class _OdomSnapshot:
    position_x: float
    position_y: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    linear_x: float
    linear_y: float
    angular_z: float


class _OdomBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: _OdomSnapshot | None = None

    def update_from_msg(self, msg: Odometry) -> None:
        snapshot = _OdomSnapshot(
            position_x=float(msg.pose.pose.position.x),
            position_y=float(msg.pose.pose.position.y),
            orientation_x=float(msg.pose.pose.orientation.x),
            orientation_y=float(msg.pose.pose.orientation.y),
            orientation_z=float(msg.pose.pose.orientation.z),
            orientation_w=float(msg.pose.pose.orientation.w),
            linear_x=float(msg.twist.twist.linear.x),
            linear_y=float(msg.twist.twist.linear.y),
            angular_z=float(msg.twist.twist.angular.z),
        )
        with self._lock:
            self._latest = snapshot

    def latest(self) -> _OdomSnapshot | None:
        with self._lock:
            return self._latest

    def clear(self) -> None:
        with self._lock:
            self._latest = None


class _ContactBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._counts: dict[str, int] = {}
        self._force_sums: dict[str, float] = {}

    def make_callback(self, key: str):
        def _callback(msg: Contacts) -> None:
            force_sum = 0.0
            for contact in msg.contacts:
                force_sum += self._contact_force_magnitude(contact)

            with self._lock:
                self._counts[key] = self._counts.get(key, 0) + len(msg.contacts)
                self._force_sums[key] = self._force_sums.get(key, 0.0) + force_sum

        return _callback

    def snapshot(self) -> dict[str, tuple[int, float]]:
        with self._lock:
            keys = set(self._counts) | set(self._force_sums)
            return {key: (self._counts.get(key, 0), self._force_sums.get(key, 0.0)) for key in keys}

    def clear(self) -> None:
        with self._lock:
            self._counts.clear()
            self._force_sums.clear()

    @staticmethod
    def _contact_force_magnitude(contact) -> float:
        max_force = 0.0
        for wrench in getattr(contact, "wrenches", []):
            for force in [wrench.body_1_wrench.force, wrench.body_2_wrench.force]:
                norm = math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z)
                max_force = max(max_force, float(norm))
        return max_force


class RosGzSimulationRunner(SimulationRunner):
    def __init__(self, runtime_cfg: RosGzRuntimeConfig | None = None, resolution_m: float = 0.1) -> None:
        self.runtime_cfg = runtime_cfg or RosGzRuntimeConfig()
        self.resolution_m = resolution_m
        self._fallback = MockSimulationRunner(resolution_m=resolution_m)

        self._node = None
        self._cmd_pub = None
        self._set_pose_client = None
        self._spawn_client = None
        self._delete_client = None
        self._odom_buffer = _OdomBuffer()
        self._contact_buffer = _ContactBuffer()
        self._spawned_entities: set[str] = set()
        self._last_spawn_z = 0.3
        self._terrain_counter = 0

        self._gzserver_proc: subprocess.Popen | None = None
        self._bridge_proc: subprocess.Popen | None = None
        self._log_dir = Path("/tmp/ml25d_ros_gz_logs")
        self._gzserver_log_fp = None
        self._bridge_log_fp = None
        self._runtime_ready = False
        self._runtime_failure_logged = False

        atexit.register(self.shutdown)

    def run(self, context: SimulationContext, rng: np.random.Generator) -> SimulationTrajectory:
        try:
            self._ensure_runtime()
            return self._run_once(context=context, rng=rng)
        except Exception as exc:
            if not self._runtime_failure_logged:
                print(f"[ml25d][ros_gz] runtime failure: {exc}")
                self._runtime_failure_logged = True
            if self.runtime_cfg.fallback_to_mock_on_error:
                return self._fallback.run(context, rng)
            raise

    def shutdown(self) -> None:
        self._runtime_ready = False

        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

        for proc in [self._bridge_proc, self._gzserver_proc]:
            if proc is not None and proc.poll() is None:
                try:
                    proc.terminate()
                    proc.wait(timeout=3.0)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass

        self._bridge_proc = None
        self._gzserver_proc = None

        for fp in [self._bridge_log_fp, self._gzserver_log_fp]:
            if fp is not None:
                try:
                    fp.close()
                except Exception:
                    pass
        self._bridge_log_fp = None
        self._gzserver_log_fp = None

        if rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _ensure_runtime(self) -> None:
        if self._runtime_ready:
            self._raise_if_process_exited()
            return

        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
            self._cmd_pub = None
            self._set_pose_client = None
            self._spawn_client = None
            self._delete_client = None

        if (
            rclpy is None
            or Twist is None
            or Odometry is None
            or SetEntityPose is None
            or SpawnEntity is None
            or DeleteEntity is None
            or Contacts is None
            or Entity is None
        ):
            raise RuntimeError("ROS Gazebo Python interfaces are unavailable in this environment")

        for exe in ["ros2"]:
            if shutil.which(exe) is None:
                raise RuntimeError(f"required executable not found: {exe}")

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = rclpy.create_node("ml25d_ros_gz_runner")
        model_name = self.runtime_cfg.model_name
        world_name = self.runtime_cfg.world_name

        self._cmd_pub = self._node.create_publisher(Twist, f"/model/{model_name}/cmd_vel", 10)
        self._node.create_subscription(Odometry, f"/model/{model_name}/odometry", self._odom_buffer.update_from_msg, 10)
        self._set_pose_client = self._node.create_client(SetEntityPose, f"/world/{world_name}/set_pose")
        self._spawn_client = self._node.create_client(SpawnEntity, f"/world/{world_name}/create")
        self._delete_client = self._node.create_client(DeleteEntity, f"/world/{world_name}/remove")
        for key in ["chassis", "front_left", "front_right", "rear_left", "rear_right"]:
            self._node.create_subscription(
                Contacts,
                f"/ml25d/{model_name}/{key}_contacts",
                self._contact_buffer.make_callback(key),
                10,
            )

        if self.runtime_cfg.auto_start_processes:
            self._start_processes()

        deadline = time.time() + self.runtime_cfg.startup_timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            self._raise_if_process_exited()
            services_ready = (
                self._set_pose_client.wait_for_service(timeout_sec=0.05)
                and self._spawn_client.wait_for_service(timeout_sec=0.05)
                and self._delete_client.wait_for_service(timeout_sec=0.05)
            )
            if services_ready:
                self._runtime_ready = True
                return

        raise RuntimeError("timed out waiting for /world/<name>/set_pose bridge service")

    def _start_processes(self) -> None:
        self._log_dir.mkdir(parents=True, exist_ok=True)
        env = os.environ.copy()
        env.setdefault("ROS_LOG_DIR", str(self._log_dir / "ros"))
        env.setdefault("GZ_LOG_PATH", str(self._log_dir / "gz"))
        Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
        Path(env["GZ_LOG_PATH"]).mkdir(parents=True, exist_ok=True)

        world_sdf_file = self._resolve_world_sdf_file(self.runtime_cfg.world_sdf_file)
        if self._gzserver_proc is None or self._gzserver_proc.poll() is not None:
            if self._gzserver_log_fp is not None:
                self._gzserver_log_fp.close()
            self._gzserver_log_fp = (self._log_dir / "gzserver.log").open("a", encoding="utf-8")
            self._gzserver_log_fp.write(f"\n--- start gzserver world={world_sdf_file} ---\n")
            self._gzserver_log_fp.flush()
            self._gzserver_proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "ros_gz_sim",
                    "gzserver",
                    "--ros-args",
                    "-p",
                    f"world_sdf_file:={world_sdf_file}",
                ],
                stdout=self._gzserver_log_fp,
                stderr=subprocess.STDOUT,
                env=env,
            )

        if self._bridge_proc is None or self._bridge_proc.poll() is not None:
            if self._bridge_log_fp is not None:
                self._bridge_log_fp.close()
            self._bridge_log_fp = (self._log_dir / "bridge.log").open("a", encoding="utf-8")
            self._bridge_log_fp.write("\n--- start ros_gz_bridge parameter_bridge ---\n")
            self._bridge_log_fp.flush()
            self._bridge_proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    f"/model/{self.runtime_cfg.model_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    f"/model/{self.runtime_cfg.model_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                    f"/world/{self.runtime_cfg.world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                    f"/world/{self.runtime_cfg.world_name}/create@ros_gz_interfaces/srv/SpawnEntity",
                    f"/world/{self.runtime_cfg.world_name}/remove@ros_gz_interfaces/srv/DeleteEntity",
                    f"/ml25d/{self.runtime_cfg.model_name}/chassis_contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
                    f"/ml25d/{self.runtime_cfg.model_name}/front_left_contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
                    f"/ml25d/{self.runtime_cfg.model_name}/front_right_contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
                    f"/ml25d/{self.runtime_cfg.model_name}/rear_left_contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
                    f"/ml25d/{self.runtime_cfg.model_name}/rear_right_contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
                ],
                stdout=self._bridge_log_fp,
                stderr=subprocess.STDOUT,
                env=env,
            )

    def _raise_if_process_exited(self) -> None:
        failures = []
        if self._gzserver_proc is not None and self._gzserver_proc.poll() is not None:
            failures.append(("gzserver", self._log_dir / "gzserver.log", self._gzserver_proc.returncode))
        if self._bridge_proc is not None and self._bridge_proc.poll() is not None:
            failures.append(("ros_gz_bridge", self._log_dir / "bridge.log", self._bridge_proc.returncode))
        if not failures:
            return

        messages = []
        for name, log_path, returncode in failures:
            messages.append(f"{name} exited with code {returncode}. log_tail:\n{self._tail_log(log_path)}")
        self._runtime_ready = False
        raise RuntimeError("\n".join(messages))

    @staticmethod
    def _tail_log(path: Path, max_chars: int = 4000) -> str:
        if not path.exists():
            return "<log file missing>"
        text = path.read_text(encoding="utf-8", errors="replace")
        if len(text) <= max_chars:
            return text
        return text[-max_chars:]

    @staticmethod
    def _resolve_world_sdf_file(path_text: str) -> str:
        path = Path(path_text).expanduser()
        if path.is_absolute() and path.exists():
            return str(path)
        candidates = [
            Path.cwd() / path,
            Path(__file__).resolve().parents[2] / path,
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        return str(path)

    def _run_once(self, context: SimulationContext, rng: np.random.Generator) -> SimulationTrajectory:
        assert self._node is not None
        assert self._cmd_pub is not None
        assert self._set_pose_client is not None
        assert self._spawn_client is not None
        assert self._delete_client is not None

        self._prepare_scene(context)
        self._odom_buffer.clear()
        self._contact_buffer.clear()
        self._reset_vehicle_pose(context.heading_rad, context.vehicle)
        self._publish_cmd(0.0, 0.0)
        self._wait_for_reset_pose(context.heading_rad)
        self._odom_buffer.clear()
        self._contact_buffer.clear()

        settle_steps = max(int(context.settle_time_sec * context.sample_rate_hz), 1)
        for _ in range(settle_steps):
            self._publish_cmd(0.0, 0.0)
            rclpy.spin_once(self._node, timeout_sec=1.0 / max(context.sample_rate_hz, 1))

        num_steps = max(int(context.duration_sec * context.sample_rate_hz), 2)
        dt = 1.0 / max(context.sample_rate_hz, 1)

        timestamps = np.arange(num_steps, dtype=np.float32) * dt
        positions = np.zeros((num_steps, 2), dtype=np.float32)
        yaw = np.zeros(num_steps, dtype=np.float32)
        roll = np.zeros(num_steps, dtype=np.float32)
        pitch = np.zeros(num_steps, dtype=np.float32)
        actual_linear = np.zeros(num_steps, dtype=np.float32)
        actual_angular = np.zeros(num_steps, dtype=np.float32)
        wheel_contact_forces = np.zeros((num_steps, 4), dtype=np.float32)
        chassis_contacts = np.zeros(num_steps, dtype=np.uint8)

        delta_psi_rad = float(np.deg2rad(context.action.delta_psi_deg))
        nominal_v = float(context.action.delta_s_m / max(context.duration_sec, 1e-6))
        nominal_w = float(delta_psi_rad / max(context.duration_sec, 1e-6))

        if context.motion_model == "skid":
            if context.action.delta_s_m > 1e-6:
                v_cmd = float(np.clip(1.0 * nominal_v, 0.06, 0.22))
            else:
                v_cmd = 0.0

            if abs(delta_psi_rad) > 1e-6:
                w_cmd = float(np.clip(nominal_w, -0.8, 0.8))
            else:
                w_cmd = float(context.action.skid_cmd["omega_cmd_rps"])
        else:
            if context.action.delta_s_m > 1e-6:
                v_cmd = float(np.clip(1.0 * nominal_v, 0.06, 0.22))
            else:
                v_cmd = 0.0

            if abs(delta_psi_rad) > 1e-6:
                w_cmd = float(np.clip(nominal_w, -0.8, 0.8))
            else:
                steer = float(np.deg2rad(context.action.ackermann_cmd["steering_angle_deg"]))
                w_cmd = float(v_cmd * np.tan(steer) / max(context.vehicle.l, 1e-6))

        commanded_linear = np.full(num_steps, v_cmd, dtype=np.float32)
        commanded_angular = np.full(num_steps, w_cmd, dtype=np.float32)

        odom_seen = 0
        prev_contact_counts = self._contact_buffer.snapshot()
        for idx in range(num_steps):
            step_start = time.monotonic()
            self._publish_cmd(v_cmd, w_cmd)
            rclpy.spin_once(self._node, timeout_sec=dt)

            elapsed = time.monotonic() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

            # Drain one more callback cycle after pacing to capture freshest odom.
            rclpy.spin_once(self._node, timeout_sec=0.0)
            prev_contact_counts = self._fill_contact_sample(
                idx=idx,
                vehicle=context.vehicle,
                forces=wheel_contact_forces,
                chassis_contacts=chassis_contacts,
                previous=prev_contact_counts,
            )

            sample = self._odom_buffer.latest()
            if sample is None:
                if idx > 0:
                    positions[idx] = positions[idx - 1]
                    yaw[idx] = yaw[idx - 1]
                    roll[idx] = roll[idx - 1]
                    pitch[idx] = pitch[idx - 1]
                    actual_linear[idx] = actual_linear[idx - 1]
                    actual_angular[idx] = actual_angular[idx - 1]
                continue

            odom_seen += 1
            positions[idx, 0] = sample.position_x
            positions[idx, 1] = sample.position_y

            roll_i, pitch_i, yaw_i = self._quat_to_rpy(
                sample.orientation_x,
                sample.orientation_y,
                sample.orientation_z,
                sample.orientation_w,
            )
            roll[idx] = roll_i
            pitch[idx] = pitch_i
            yaw[idx] = yaw_i

            actual_linear[idx] = sample.linear_x
            actual_angular[idx] = sample.angular_z

        self._publish_cmd(0.0, 0.0)
        for _ in range(3):
            rclpy.spin_once(self._node, timeout_sec=dt)

        if odom_seen < max(2, int(0.2 * num_steps)):
            raise RuntimeError("insufficient odometry samples received from ros_gz bridge")

        if self._has_odometry_discontinuity(positions):
            raise RuntimeError("odometry discontinuity detected after pose reset")

        unwrapped_yaw = np.unwrap(yaw).astype(np.float32)
        actual_linear = self._estimate_forward_speed(positions, unwrapped_yaw, dt)
        actual_angular = np.gradient(unwrapped_yaw, dt).astype(np.float32)
        if np.max(wheel_contact_forces) <= 0.0:
            raise RuntimeError("no wheel contact sensor samples received from Gazebo")

        completed_displacement = float(np.linalg.norm(positions[-1] - positions[0]))
        completed_heading = float(abs(unwrapped_yaw[-1] - unwrapped_yaw[0]))

        return SimulationTrajectory(
            timestamps=timestamps,
            positions_xy=positions,
            yaw_rad=yaw,
            roll_rad=roll,
            pitch_rad=pitch,
            commanded_linear_speed=commanded_linear,
            actual_linear_speed=actual_linear,
            commanded_angular_speed=commanded_angular,
            actual_angular_speed=actual_angular,
            wheel_contact_forces=wheel_contact_forces,
            chassis_contacts=chassis_contacts,
            completed_displacement_m=completed_displacement,
            completed_heading_change_rad=completed_heading,
        )

    def _prepare_scene(self, context: SimulationContext) -> None:
        terrain_name = "ml25d_terrain"
        model_name = self.runtime_cfg.model_name
        for name in [model_name, terrain_name]:
            self._delete_entity(name, require_success=False)

        self._spawn_entity(
            name=terrain_name,
            sdf=self._terrain_sdf(terrain_name, context.heightmap, context.friction_mu),
            z=0.0,
        )
        self._last_spawn_z = self._vehicle_start_z(context.heightmap, context.vehicle)
        self._spawn_entity(
            name=model_name,
            sdf=self._vehicle_sdf(model_name, context.vehicle),
            z=self._last_spawn_z,
        )

        assert self._node is not None
        deadline = time.time() + min(self.runtime_cfg.service_timeout_sec, 3.0)
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def _spawn_entity(self, name: str, sdf: str, z: float) -> None:
        assert self._spawn_client is not None
        assert self._node is not None

        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = sdf
        req.entity_factory.allow_renaming = False
        req.entity_factory.pose.position.z = float(z)

        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self.runtime_cfg.service_timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"spawn service call timed out for {name}")
        if not bool(future.result().success):
            raise RuntimeError(f"spawn service returned success=False for {name}")
        self._spawned_entities.add(name)

    def _delete_entity(self, name: str, require_success: bool) -> None:
        assert self._delete_client is not None
        assert self._node is not None

        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        future = self._delete_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self.runtime_cfg.service_timeout_sec)
        if require_success and (not future.done() or future.result() is None or not bool(future.result().success)):
            raise RuntimeError(f"delete service failed for {name}")
        self._spawned_entities.discard(name)

    def _fill_contact_sample(
        self,
        idx: int,
        vehicle: VehicleParams,
        forces: np.ndarray,
        chassis_contacts: np.ndarray,
        previous: dict[str, tuple[int, float]],
    ) -> dict[str, tuple[int, float]]:
        current = self._contact_buffer.snapshot()
        keys = ["chassis", "front_left", "front_right", "rear_left", "rear_right"]
        delta_counts = {key: current.get(key, (0, 0.0))[0] - previous.get(key, (0, 0.0))[0] for key in keys}
        delta_forces = {key: current.get(key, (0, 0.0))[1] - previous.get(key, (0, 0.0))[1] for key in keys}
        base_load = float(vehicle.m * 9.81 / 4.0)

        def force_or_fallback(key: str) -> float:
            force = float(delta_forces[key])
            if force > 0.0:
                return force
            if delta_counts[key] > 0:
                return base_load
            return 0.0

        forces[idx] = np.array(
            [
                force_or_fallback("front_left"),
                force_or_fallback("front_right"),
                force_or_fallback("rear_left"),
                force_or_fallback("rear_right"),
            ],
            dtype=np.float32,
        )
        chassis_contacts[idx] = 1 if delta_counts["chassis"] > 0 else 0
        return current

    def _vehicle_start_z(self, heightmap: np.ndarray, vehicle: VehicleParams) -> float:
        if heightmap.size == 0:
            return 0.03
        center = heightmap.shape[0] // 2
        half_span_m = 0.5 * max(float(vehicle.L), float(vehicle.W)) + 0.15
        half_span_cells = max(1, int(math.ceil(half_span_m / max(self.resolution_m, 1e-6))))
        lo = max(center - half_span_cells, 0)
        hi = min(center + half_span_cells + 1, heightmap.shape[0])
        local_max = float(np.max(heightmap[lo:hi, lo:hi]))
        return local_max + 0.03

    def _terrain_sdf(self, model_name: str, heightmap: np.ndarray, friction_mu: float) -> str:
        friction = float(np.clip(friction_mu, 0.05, 3.0))
        mesh_uri = self._write_terrain_mesh(heightmap)
        return f"""
<sdf version='1.8'>
  <model name='{escape(model_name)}'>
    <static>true</static>
    <link name='terrain_link'>
      <collision name='terrain_collision'>
        <geometry><mesh><uri>{escape(mesh_uri)}</uri></mesh></geometry>
        <surface>
          <friction><ode><mu>{friction:.4f}</mu><mu2>{friction:.4f}</mu2></ode></friction>
          <contact><ode><kp>1000000</kp><kd>1</kd><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name='terrain_visual'>
        <geometry><mesh><uri>{escape(mesh_uri)}</uri></mesh></geometry>
        <material><ambient>0.35 0.28 0.18 1</ambient><diffuse>0.45 0.36 0.22 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def _write_terrain_mesh(self, heightmap: np.ndarray) -> str:
        self._terrain_counter += 1
        mesh_dir = Path("/tmp/ml25d_ros_gz_meshes")
        mesh_dir.mkdir(parents=True, exist_ok=True)
        mesh_path = mesh_dir / f"terrain_{id(self)}_{self._terrain_counter}.obj"

        n, m = heightmap.shape
        center_i = (n - 1) / 2.0
        center_j = (m - 1) / 2.0
        base_z = float(np.min(heightmap) - 0.20)
        vertices: list[tuple[float, float, float]] = []
        for level in ["top", "bottom"]:
            for i in range(n):
                for j in range(m):
                    x = (i - center_i) * self.resolution_m
                    y = (j - center_j) * self.resolution_m
                    z = float(heightmap[i, j]) if level == "top" else base_z
                    vertices.append((x, y, z))

        def top_idx(i: int, j: int) -> int:
            return i * m + j + 1

        def bottom_idx(i: int, j: int) -> int:
            return n * m + i * m + j + 1

        faces: list[tuple[int, int, int]] = []
        for i in range(n - 1):
            for j in range(m - 1):
                faces.append((top_idx(i, j), top_idx(i + 1, j), top_idx(i, j + 1)))
                faces.append((top_idx(i + 1, j), top_idx(i + 1, j + 1), top_idx(i, j + 1)))
                faces.append((bottom_idx(i, j), bottom_idx(i, j + 1), bottom_idx(i + 1, j)))
                faces.append((bottom_idx(i + 1, j), bottom_idx(i, j + 1), bottom_idx(i + 1, j + 1)))

        for i in range(n - 1):
            faces.extend(
                [
                    (top_idx(i, 0), bottom_idx(i, 0), top_idx(i + 1, 0)),
                    (top_idx(i + 1, 0), bottom_idx(i, 0), bottom_idx(i + 1, 0)),
                    (top_idx(i, m - 1), top_idx(i + 1, m - 1), bottom_idx(i, m - 1)),
                    (top_idx(i + 1, m - 1), bottom_idx(i + 1, m - 1), bottom_idx(i, m - 1)),
                ]
            )
        for j in range(m - 1):
            faces.extend(
                [
                    (top_idx(0, j), top_idx(0, j + 1), bottom_idx(0, j)),
                    (top_idx(0, j + 1), bottom_idx(0, j + 1), bottom_idx(0, j)),
                    (top_idx(n - 1, j), bottom_idx(n - 1, j), top_idx(n - 1, j + 1)),
                    (top_idx(n - 1, j + 1), bottom_idx(n - 1, j), bottom_idx(n - 1, j + 1)),
                ]
            )

        lines = ["# ml25d generated terrain mesh"]
        lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices)
        lines.extend(f"f {a} {b} {c}" for a, b, c in faces)
        mesh_path.write_text("\n".join(lines) + "\n", encoding="ascii")
        return mesh_path.as_uri()

    def _vehicle_sdf(self, model_name: str, vehicle: VehicleParams) -> str:
        name = escape(model_name)
        length = float(vehicle.L)
        width = float(vehicle.W)
        radius = float(vehicle.r_w)
        wheel_sep = float(vehicle.b)
        chassis_h = max(float(vehicle.z_c), 0.08)
        chassis_z = radius + float(vehicle.c_g) + 0.5 * chassis_h
        wheel_x = min(float(vehicle.l) * 0.5, length * 0.42)
        wheel_y = max(wheel_sep * 0.5, width * 0.45)
        wheel_width = float(np.clip(width * 0.18, 0.04, 0.12))
        mass = max(float(vehicle.m), 1.0)
        chassis_mass = mass * 0.65
        wheel_mass = max((mass - chassis_mass) / 4.0, 0.25)
        ixx = chassis_mass * (width * width + chassis_h * chassis_h) / 12.0
        iyy = chassis_mass * (length * length + chassis_h * chassis_h) / 12.0
        izz = chassis_mass * (length * length + width * width) / 12.0
        topic_prefix = f"/ml25d/{name}"
        max_v = max(float(vehicle.F_max) / max(mass * 9.81, 1e-6), 0.2)
        max_v = float(np.clip(max_v, 0.2, 1.2))
        return f"""
<sdf version='1.8'>
  <model name='{name}'>
    <self_collide>false</self_collide>
    <link name='chassis'>
      <pose>0 0 {chassis_z:.4f} 0 0 0</pose>
      <inertial><mass>{chassis_mass:.4f}</mass><inertia><ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy:.6f}</iyy><iyz>0</iyz><izz>{izz:.6f}</izz></inertia></inertial>
      <collision name='collision_chassis'>
        <geometry><box><size>{length:.4f} {width:.4f} {chassis_h:.4f}</size></box></geometry>
      </collision>
      <visual name='visual_chassis'><geometry><box><size>{length:.4f} {width:.4f} {chassis_h:.4f}</size></box></geometry></visual>
      <sensor name='chassis_contact' type='contact'>
        <contact><collision>collision_chassis</collision><topic>{topic_prefix}/chassis_contacts</topic></contact>
        <always_on>1</always_on><update_rate>100</update_rate>
      </sensor>
    </link>
    {self._wheel_link_sdf('front_left', wheel_x, wheel_y, radius, wheel_width, wheel_mass, f'{topic_prefix}/front_left_contacts')}
    {self._wheel_link_sdf('front_right', wheel_x, -wheel_y, radius, wheel_width, wheel_mass, f'{topic_prefix}/front_right_contacts')}
    {self._wheel_link_sdf('rear_left', -wheel_x, wheel_y, radius, wheel_width, wheel_mass, f'{topic_prefix}/rear_left_contacts')}
    {self._wheel_link_sdf('rear_right', -wheel_x, -wheel_y, radius, wheel_width, wheel_mass, f'{topic_prefix}/rear_right_contacts')}
    <joint name='front_left_wheel_joint' type='revolute'><parent>chassis</parent><child>front_left</child><axis><xyz>0 0 1</xyz></axis></joint>
    <joint name='front_right_wheel_joint' type='revolute'><parent>chassis</parent><child>front_right</child><axis><xyz>0 0 1</xyz></axis></joint>
    <joint name='rear_left_wheel_joint' type='revolute'><parent>chassis</parent><child>rear_left</child><axis><xyz>0 0 1</xyz></axis></joint>
    <joint name='rear_right_wheel_joint' type='revolute'><parent>chassis</parent><child>rear_right</child><axis><xyz>0 0 1</xyz></axis></joint>
    <plugin filename='gz-sim-diff-drive-system' name='gz::sim::systems::DiffDrive'>
      <left_joint>front_left_wheel_joint</left_joint>
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <wheel_separation>{2.0 * wheel_y:.4f}</wheel_separation>
      <wheel_radius>{radius:.4f}</wheel_radius>
      <odom_publish_frequency>50</odom_publish_frequency>
      <max_linear_velocity>{max_v:.4f}</max_linear_velocity>
      <max_angular_velocity>1.2</max_angular_velocity>
    </plugin>
    <plugin filename='gz-sim-odometry-publisher-system' name='gz::sim::systems::OdometryPublisher'>
      <odom_frame>{name}/odom</odom_frame>
      <robot_base_frame>{name}</robot_base_frame>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
"""

    @staticmethod
    def _wheel_link_sdf(
        link_name: str,
        x: float,
        y: float,
        radius: float,
        width: float,
        mass: float,
        contact_topic: str,
    ) -> str:
        i_axis = 0.5 * mass * radius * radius
        i_radial = mass * (3.0 * radius * radius + width * width) / 12.0
        return f"""
    <link name='{escape(link_name)}'>
      <pose>{x:.4f} {y:.4f} {radius:.4f} -1.5707 0 0</pose>
      <inertial><mass>{mass:.4f}</mass><inertia><ixx>{i_radial:.6f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{i_radial:.6f}</iyy><iyz>0</iyz><izz>{i_axis:.6f}</izz></inertia></inertial>
      <collision name='collision_{escape(link_name)}'>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{width:.4f}</length></cylinder></geometry>
        <surface>
          <friction>
            <ode><mu>1.2</mu><mu2>1.2</mu2><slip1>0.02</slip1><slip2>0.02</slip2><fdir1>0 0 1</fdir1></ode>
            <bullet><friction>1.2</friction><friction2>1.2</friction2><rolling_friction>0.08</rolling_friction></bullet>
          </friction>
        </surface>
      </collision>
      <visual name='visual_{escape(link_name)}'><geometry><cylinder><radius>{radius:.4f}</radius><length>{width:.4f}</length></cylinder></geometry></visual>
      <sensor name='{escape(link_name)}_contact' type='contact'>
        <contact><collision>collision_{escape(link_name)}</collision><topic>{escape(contact_topic)}</topic></contact>
        <always_on>1</always_on><update_rate>100</update_rate>
      </sensor>
    </link>"""

    @staticmethod
    def _estimate_forward_speed(positions: np.ndarray, yaw_rad: np.ndarray, dt: float) -> np.ndarray:
        if positions.shape[0] == 0:
            return np.zeros(0, dtype=np.float32)

        deltas = np.zeros_like(positions, dtype=np.float32)
        if positions.shape[0] > 1:
            deltas[1:] = positions[1:] - positions[:-1]
            deltas[0] = deltas[1]

        heading = np.column_stack([np.cos(yaw_rad), np.sin(yaw_rad)]).astype(np.float32)
        forward = np.sum(deltas * heading, axis=1) / max(dt, 1e-6)
        return forward.astype(np.float32)

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        assert self._cmd_pub is not None
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _reset_vehicle_pose(self, heading_rad: float, vehicle: VehicleParams) -> None:
        assert self._set_pose_client is not None
        assert self._node is not None

        req = SetEntityPose.Request()
        req.entity.name = self.runtime_cfg.model_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = self._last_spawn_z

        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = float(np.sin(heading_rad / 2.0))
        req.pose.orientation.w = float(np.cos(heading_rad / 2.0))

        future = self._set_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self.runtime_cfg.service_timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError("set_pose service call timed out")

        resp = future.result()
        if not bool(resp.success):
            raise RuntimeError("set_pose service returned success=False")

    def _wait_for_reset_pose(self, heading_rad: float) -> None:
        assert self._node is not None

        deadline = time.time() + min(self.runtime_cfg.service_timeout_sec, 3.0)
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            sample = self._odom_buffer.latest()
            if sample is None:
                continue

            _, _, yaw = self._quat_to_rpy(
                sample.orientation_x,
                sample.orientation_y,
                sample.orientation_z,
                sample.orientation_w,
            )
            dist = math.hypot(sample.position_x, sample.position_y)
            yaw_err = abs(math.atan2(math.sin(yaw - heading_rad), math.cos(yaw - heading_rad)))
            if dist < 0.25 and yaw_err < 0.35:
                return

        raise RuntimeError("timed out waiting for fresh odometry after pose reset")

    @staticmethod
    def _has_odometry_discontinuity(positions: np.ndarray) -> bool:
        if positions.shape[0] < 2:
            return False
        jumps = np.linalg.norm(positions[1:] - positions[:-1], axis=1)
        return bool(np.max(jumps) > 1.0)

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return float(roll), float(pitch), float(yaw)

class MockSimulationRunner(SimulationRunner):
    def __init__(self, resolution_m: float = 0.1) -> None:
        self.resolution_m = resolution_m

    def run(self, context: SimulationContext, rng: np.random.Generator) -> SimulationTrajectory:
        num_steps = int(max(context.duration_sec * context.sample_rate_hz, 2))
        dt = 1.0 / max(context.sample_rate_hz, 1)

        timestamps = np.arange(num_steps, dtype=np.float32) * dt
        center = context.heightmap.shape[0] // 2
        grad_u, grad_v = np.gradient(context.heightmap, self.resolution_m, edge_order=1)

        slope_forward = float(
            grad_u[center, center] * np.cos(context.heading_rad) + grad_v[center, center] * np.sin(context.heading_rad)
        )
        slope_lateral = float(
            -grad_u[center, center] * np.sin(context.heading_rad) + grad_v[center, center] * np.cos(context.heading_rad)
        )

        local_window = context.heightmap[max(center - 2, 0) : center + 3, max(center - 2, 0) : center + 3]
        roughness = float(np.std(local_window))
        span = float(np.max(local_window) - np.min(local_window))

        delta_psi_rad = float(np.deg2rad(context.action.delta_psi_deg))
        v_cmd = float(context.action.delta_s_m / max(context.duration_sec, 1e-6))
        w_cmd = float(delta_psi_rad / max(context.duration_sec, 1e-6))

        alpha_max = np.deg2rad(context.vehicle.alpha_max_deg)
        capability = np.tan(max(alpha_max, 1e-4))
        traction = context.friction_mu * context.vehicle.F_max / max(context.vehicle.m * 9.81, 1e-6)

        slip_base = max(0.0, abs(slope_forward) - capability) / max(capability, 1e-4)
        slip_base += max(0.0, 0.55 - traction)
        slip_base += roughness * 8.0
        slip_base += span / max(context.vehicle.r_w * 4.0, 1e-4)
        slip_base = float(np.clip(slip_base, 0.0, 1.4))

        lin_factor = float(np.clip(1.0 - slip_base + rng.normal(0.0, 0.03), 0.0, 1.2))
        ang_factor = float(np.clip(1.0 - 0.8 * slip_base + rng.normal(0.0, 0.03), 0.0, 1.2))

        commanded_linear = np.full(num_steps, v_cmd, dtype=np.float32)
        commanded_angular = np.full(num_steps, w_cmd, dtype=np.float32)
        actual_linear = (commanded_linear * lin_factor).astype(np.float32)
        actual_angular = (commanded_angular * ang_factor).astype(np.float32)

        base_roll = np.arctan(abs(slope_lateral) + roughness * 2.0)
        base_pitch = np.arctan(abs(slope_forward) + roughness * 2.0)
        roll_sign = -1.0 if slope_lateral < 0.0 else 1.0
        pitch_sign = -1.0 if slope_forward < 0.0 else 1.0

        roll = np.zeros(num_steps, dtype=np.float32)
        pitch = np.zeros(num_steps, dtype=np.float32)
        yaw = np.zeros(num_steps, dtype=np.float32)
        positions = np.zeros((num_steps, 2), dtype=np.float32)

        for t in range(1, num_steps):
            phase = 2.0 * np.pi * t / max(num_steps - 1, 1)
            roll[t] = float(
                roll_sign * base_roll * (1.0 + 0.12 * np.sin(phase)) + rng.normal(0.0, 0.01 + roughness * 0.5)
            )
            pitch[t] = float(
                pitch_sign * base_pitch * (1.0 + 0.12 * np.cos(phase)) + rng.normal(0.0, 0.01 + roughness * 0.5)
            )
            yaw[t] = yaw[t - 1] + actual_angular[t - 1] * dt
            positions[t, 0] = positions[t - 1, 0] + actual_linear[t - 1] * dt * np.cos(yaw[t - 1])
            positions[t, 1] = positions[t - 1, 1] + actual_linear[t - 1] * dt * np.sin(yaw[t - 1])

        base_load = context.vehicle.m * 9.81 / 4.0
        forces = np.zeros((num_steps, 4), dtype=np.float32)
        lift_prob = float(np.clip(roughness * 12.0 + abs(slope_lateral) * 1.5 + span / max(context.vehicle.r_w, 1e-3), 0.0, 0.9))

        for t in range(num_steps):
            tr_roll = base_load * np.tan(roll[t]) * 0.6
            tr_pitch = base_load * np.tan(pitch[t]) * 0.4

            fl = base_load - tr_roll - tr_pitch
            fr = base_load + tr_roll - tr_pitch
            rl = base_load - tr_roll + tr_pitch
            rr = base_load + tr_roll + tr_pitch
            row = np.array([fl, fr, rl, rr], dtype=np.float32)

            if rng.random() < lift_prob * 0.1:
                row[rng.integers(0, 4)] *= 0.02

            forces[t, :] = np.clip(row, 0.0, None)

        bottom_severity = float(
            np.clip((span - 0.75 * context.vehicle.c_g) / max(2.0 * context.vehicle.c_g, 1e-3), 0.0, 1.0)
        )
        bottom_rate = float(np.clip(0.35 * bottom_severity + 0.05 * min(roughness / max(context.vehicle.c_g, 1e-3), 1.0), 0.0, 0.4))
        chassis_contacts = _deterministic_contact_series(num_steps, bottom_rate)

        completed_displacement = float(np.linalg.norm(positions[-1] - positions[0]))
        completed_heading = float(abs(yaw[-1] - yaw[0]))

        return SimulationTrajectory(
            timestamps=timestamps,
            positions_xy=positions,
            yaw_rad=yaw,
            roll_rad=roll,
            pitch_rad=pitch,
            commanded_linear_speed=commanded_linear,
            actual_linear_speed=actual_linear,
            commanded_angular_speed=commanded_angular,
            actual_angular_speed=actual_angular,
            wheel_contact_forces=forces,
            chassis_contacts=chassis_contacts,
            completed_displacement_m=completed_displacement,
            completed_heading_change_rad=completed_heading,
        )


def _deterministic_contact_series(num_steps: int, contact_rate: float) -> np.ndarray:
    contacts = np.zeros(num_steps, dtype=np.uint8)
    if num_steps <= 0:
        return contacts

    count = int(round(float(np.clip(contact_rate, 0.0, 1.0)) * num_steps))
    if count <= 0:
        return contacts

    indices = np.linspace(0, num_steps - 1, count, dtype=int)
    contacts[indices] = 1
    return contacts


def make_runner(backend: str, sim_cfg: dict | None = None) -> SimulationRunner:
    if backend in {"mock", "surrogate"}:
        return MockSimulationRunner()
    if backend == "ros_gz":
        ros_cfg = (sim_cfg or {}).get("ros_gz", {}) if sim_cfg is not None else {}
        runtime_cfg = RosGzRuntimeConfig(
            world_sdf_file=str(ros_cfg.get("world_sdf_file", RosGzRuntimeConfig.world_sdf_file)),
            world_name=str(ros_cfg.get("world_name", RosGzRuntimeConfig.world_name)),
            model_name=str(ros_cfg.get("model_name", RosGzRuntimeConfig.model_name)),
            startup_timeout_sec=float(ros_cfg.get("startup_timeout_sec", RosGzRuntimeConfig.startup_timeout_sec)),
            service_timeout_sec=float(ros_cfg.get("service_timeout_sec", RosGzRuntimeConfig.service_timeout_sec)),
            auto_start_processes=bool(ros_cfg.get("auto_start_processes", RosGzRuntimeConfig.auto_start_processes)),
            fallback_to_mock_on_error=bool(
                ros_cfg.get("fallback_to_mock_on_error", RosGzRuntimeConfig.fallback_to_mock_on_error)
            ),
        )
        return RosGzSimulationRunner(runtime_cfg=runtime_cfg)
    raise ValueError(f"unsupported backend: {backend}")
