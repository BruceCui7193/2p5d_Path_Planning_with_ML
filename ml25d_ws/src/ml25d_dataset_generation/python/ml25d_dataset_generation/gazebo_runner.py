from __future__ import annotations

import atexit
from dataclasses import dataclass
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from typing import Literal
from xml.sax.saxutils import escape

import numpy as np
from PIL import Image

from .common_types import ActionPrimitive, SimulationTrajectory, VehicleParams

_ROS_IMPORT_ERROR: Exception | None = None

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.srv import ControlWorld
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.msg import Contacts
    from ros_gz_interfaces.srv import DeleteEntity
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.srv import SpawnEntity
except ImportError as exc:
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Twist = None
    Odometry = None
    ControlWorld = None
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
    cmd_ramp_sec: float = 0.0
    extra_obstacles: list[dict[str, float]] | None = None
    scene_id: str | None = None


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
    log_dir: str = "/tmp/ml25d_ros_gz_logs"
    headless: bool = True


@dataclass(frozen=True)
class _OdomSnapshot:
    position_x: float
    position_y: float
    position_z: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    linear_x: float
    linear_y: float
    angular_z: float
    msg_time_sec: float
    recv_time_sec: float


class StartGateError(RuntimeError):
    def __init__(self, diag: dict[str, float]) -> None:
        self.diag = dict(diag)
        super().__init__(
            "start stability gate failed: " + ", ".join(f"{k}={v:.4f}" for k, v in sorted(self.diag.items()))
        )


class _OdomBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: _OdomSnapshot | None = None

    def update_from_msg(self, msg: Odometry) -> None:
        recv_time = time.monotonic()
        msg_time_sec = float(getattr(msg.header.stamp, "sec", 0)) + 1e-9 * float(getattr(msg.header.stamp, "nanosec", 0))
        if not math.isfinite(msg_time_sec):
            msg_time_sec = 0.0

        pos_x = float(msg.pose.pose.position.x)
        pos_y = float(msg.pose.pose.position.y)
        pos_z = float(msg.pose.pose.position.z)
        ori_x = float(msg.pose.pose.orientation.x)
        ori_y = float(msg.pose.pose.orientation.y)
        ori_z = float(msg.pose.pose.orientation.z)
        ori_w = float(msg.pose.pose.orientation.w)
        if not all(math.isfinite(v) for v in [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]):
            return
        q_norm = math.sqrt(ori_x * ori_x + ori_y * ori_y + ori_z * ori_z + ori_w * ori_w)
        if q_norm <= 1e-8:
            return
        ori_x /= q_norm
        ori_y /= q_norm
        ori_z /= q_norm
        ori_w /= q_norm

        lin_x = float(msg.twist.twist.linear.x)
        lin_y = float(msg.twist.twist.linear.y)
        ang_z = float(msg.twist.twist.angular.z)
        if not math.isfinite(lin_x):
            lin_x = 0.0
        if not math.isfinite(lin_y):
            lin_y = 0.0
        if not math.isfinite(ang_z):
            ang_z = 0.0

        snapshot = _OdomSnapshot(
            position_x=pos_x,
            position_y=pos_y,
            position_z=pos_z,
            orientation_x=ori_x,
            orientation_y=ori_y,
            orientation_z=ori_z,
            orientation_w=ori_w,
            linear_x=lin_x,
            linear_y=lin_y,
            angular_z=ang_z,
            msg_time_sec=msg_time_sec,
            recv_time_sec=recv_time,
        )
        with self._lock:
            self._latest = snapshot

    def latest(
        self,
        *,
        min_recv_time_sec: float | None = None,
        min_msg_time_sec: float | None = None,
    ) -> _OdomSnapshot | None:
        with self._lock:
            sample = self._latest
        if sample is None:
            return None
        if min_recv_time_sec is not None and sample.recv_time_sec < float(min_recv_time_sec):
            return None
        if min_msg_time_sec is not None and sample.msg_time_sec < float(min_msg_time_sec):
            return None
        return sample

    def clear(self) -> None:
        with self._lock:
            self._latest = None


class _ContactBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._counts: dict[str, int] = {}
        self._force_sums: dict[str, float] = {}
        self._latest_counts: dict[str, int] = {}
        self._latest_forces: dict[str, float] = {}
        self._latest_recv_stamp: dict[str, float] = {}
        self._latest_msg_stamp: dict[str, float] = {}
        self._latest_nonzero_counts: dict[str, int] = {}
        self._latest_nonzero_forces: dict[str, float] = {}
        self._latest_nonzero_recv_stamp: dict[str, float] = {}
        self._latest_nonzero_msg_stamp: dict[str, float] = {}

    def make_callback(self, key: str):
        def _callback(msg: Contacts) -> None:
            contact_count = len(msg.contacts)
            force_sum = 0.0
            for contact in msg.contacts:
                force_sum += self._contact_force_magnitude(contact)
            recv_time = time.monotonic()
            msg_time_sec = float(getattr(msg.header.stamp, "sec", 0)) + 1e-9 * float(getattr(msg.header.stamp, "nanosec", 0))

            with self._lock:
                self._counts[key] = self._counts.get(key, 0) + contact_count
                self._force_sums[key] = self._force_sums.get(key, 0.0) + force_sum
                self._latest_counts[key] = contact_count
                self._latest_forces[key] = force_sum
                self._latest_recv_stamp[key] = recv_time
                self._latest_msg_stamp[key] = msg_time_sec
                if contact_count > 0:
                    self._latest_nonzero_counts[key] = contact_count
                    self._latest_nonzero_forces[key] = force_sum
                    self._latest_nonzero_recv_stamp[key] = recv_time
                    self._latest_nonzero_msg_stamp[key] = msg_time_sec

        return _callback

    def snapshot(self) -> dict[str, dict[str, float | int]]:
        with self._lock:
            keys = (
                set(self._counts)
                | set(self._force_sums)
                | set(self._latest_counts)
                | set(self._latest_forces)
                | set(self._latest_recv_stamp)
                | set(self._latest_msg_stamp)
                | set(self._latest_nonzero_counts)
                | set(self._latest_nonzero_forces)
                | set(self._latest_nonzero_recv_stamp)
                | set(self._latest_nonzero_msg_stamp)
            )
            return {
                key: {
                    "count_total": int(self._counts.get(key, 0)),
                    "force_total": float(self._force_sums.get(key, 0.0)),
                    "latest_count": int(self._latest_counts.get(key, 0)),
                    "latest_force": float(self._latest_forces.get(key, 0.0)),
                    "latest_recv_time": float(self._latest_recv_stamp.get(key, 0.0)),
                    "latest_msg_time": float(self._latest_msg_stamp.get(key, 0.0)),
                    "latest_nonzero_count": int(self._latest_nonzero_counts.get(key, 0)),
                    "latest_nonzero_force": float(self._latest_nonzero_forces.get(key, 0.0)),
                    "latest_nonzero_recv_time": float(self._latest_nonzero_recv_stamp.get(key, 0.0)),
                    "latest_nonzero_msg_time": float(self._latest_nonzero_msg_stamp.get(key, 0.0)),
                }
                for key in keys
            }

    def clear(self) -> None:
        with self._lock:
            self._counts.clear()
            self._force_sums.clear()
            self._latest_counts.clear()
            self._latest_forces.clear()
            self._latest_recv_stamp.clear()
            self._latest_msg_stamp.clear()
            self._latest_nonzero_counts.clear()
            self._latest_nonzero_forces.clear()
            self._latest_nonzero_recv_stamp.clear()
            self._latest_nonzero_msg_stamp.clear()

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
        self._control_world_client = None
        self._odom_buffer = _OdomBuffer()
        self._contact_buffer = _ContactBuffer()
        self._spawned_entities: set[str] = set()
        self._active_obstacle_names: set[str] = set()
        self._active_terrain_name: str | None = None
        self._last_spawn_z = 0.3
        self._last_spawn_roll = 0.0
        self._last_spawn_pitch = 0.0
        self._terrain_counter = 0
        self._last_sample_start_msg_time = 0.0
        self._sample_buffer_reset_total = 0
        self._last_run_debug: dict[str, float] = {}

        self._gzserver_proc: subprocess.Popen | None = None
        self._bridge_proc: subprocess.Popen | None = None
        self._log_dir = Path(self.runtime_cfg.log_dir)
        self._gzserver_log_fp = None
        self._bridge_log_fp = None
        self._runtime_ready = False
        self._runtime_failure_logged = False
        self._pause_control_available = False

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

    def get_last_run_debug(self) -> dict[str, float]:
        return {k: float(v) for k, v in self._last_run_debug.items()}

    def audit_start_gate(self, context: SimulationContext) -> dict[str, float | bool | str]:
        self._ensure_runtime()
        assert self._node is not None

        self._set_paused(True)
        self._prepare_scene(context)
        self._reset_sample_buffers()
        self._reset_vehicle_pose(
            heading_rad=context.heading_rad,
            roll_rad=self._last_spawn_roll,
            pitch_rad=self._last_spawn_pitch,
            z_pos=self._last_spawn_z,
        )
        self._publish_cmd(0.0, 0.0)
        self._set_paused(False)
        self._wait_for_reset_pose(context.heading_rad)
        self._reset_sample_buffers()
        sample_epoch_recv_time = time.monotonic()

        settle_sec = float(np.clip(context.settle_time_sec, 0.5, 1.0))
        settle_steps = max(int(settle_sec * context.sample_rate_hz), 1)
        settle_t0 = time.monotonic()
        for _ in range(settle_steps):
            self._publish_cmd(0.0, 0.0)
            rclpy.spin_once(self._node, timeout_sec=1.0 / max(context.sample_rate_hz, 1))
        settle_elapsed = float(time.monotonic() - settle_t0)

        gate_pass = False
        diag: dict[str, float] = {}
        error = ""
        try:
            gate = self._wait_start_stability_gate(
                context=context,
                sample_epoch_recv_time=sample_epoch_recv_time,
            )
            gate_pass = True
            diag = {k: float(v) for k, v in gate.items() if isinstance(v, (int, float))}
        except StartGateError as exc:
            diag = {k: float(v) for k, v in exc.diag.items()}
            error = str(exc)
        except Exception as exc:  # pragma: no cover - defensive fallback
            error = str(exc)
        finally:
            self._publish_cmd(0.0, 0.0)
            for _ in range(2):
                rclpy.spin_once(self._node, timeout_sec=0.0)

        bottom_before_action = bool(float(diag.get("bottom_flag", 0.0)) > 0.5)
        lift_before_action = bool(float(diag.get("lift_before_action", 0.0)) > 0.5)
        message_time_valid = bool(
            float(diag.get("contact_fresh", 0.0)) > 0.5 and float(diag.get("odom_msg_fresh", 0.0)) > 0.5
        )
        out: dict[str, float | bool | str] = {
            "gate_pass": bool(gate_pass),
            "initialization_invalid": bool(not gate_pass),
            "settle_time_sec": settle_elapsed,
            "bottom_before_action": bottom_before_action,
            "lift_before_action": lift_before_action,
            "message_time_valid": message_time_valid,
            "error": error,
        }
        for key, value in diag.items():
            out[key] = float(value)
        return out

    def shutdown(self) -> None:
        self._runtime_ready = False

        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
            self._control_world_client = None
            self._pause_control_available = False

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
            self._control_world_client = None
            self._pause_control_available = False

        if (
            rclpy is None
            or Twist is None
            or Odometry is None
            or ControlWorld is None
            or SetEntityPose is None
            or SpawnEntity is None
            or DeleteEntity is None
            or Contacts is None
            or Entity is None
        ):
            py_path = os.environ.get("PYTHONPATH", "")
            import_err = repr(_ROS_IMPORT_ERROR) if _ROS_IMPORT_ERROR is not None else "unknown"
            hint = (
                "Ensure `source /opt/ros/jazzy/setup.bash` is executed and do not overwrite PYTHONPATH. "
                "Use `PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH` (append) instead of assignment."
            )
            raise RuntimeError(
                "ROS Gazebo Python interfaces are unavailable in this environment. "
                f"import_error={import_err}; python={sys.executable}; PYTHONPATH={py_path!r}. {hint}"
            )

        for exe in ["ros2"]:
            if shutil.which(exe) is None:
                raise RuntimeError(f"required executable not found: {exe}")
        if self.runtime_cfg.auto_start_processes and shutil.which("gz") is None:
            raise RuntimeError("required executable not found: gz")

        if not rclpy.ok():
            rclpy.init(args=None)

        safe_name = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in self.runtime_cfg.model_name)
        self._node = rclpy.create_node(f"ml25d_ros_gz_runner_{safe_name}")
        model_name = self.runtime_cfg.model_name
        world_name = self.runtime_cfg.world_name

        self._cmd_pub = self._node.create_publisher(Twist, f"/model/{model_name}/cmd_vel", 10)
        self._node.create_subscription(Odometry, f"/model/{model_name}/odometry", self._odom_buffer.update_from_msg, 10)
        self._set_pose_client = self._node.create_client(SetEntityPose, f"/world/{world_name}/set_pose")
        self._spawn_client = self._node.create_client(SpawnEntity, f"/world/{world_name}/create")
        self._delete_client = self._node.create_client(DeleteEntity, f"/world/{world_name}/remove")
        self._control_world_client = self._node.create_client(ControlWorld, f"/world/{world_name}/control")
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
                self._pause_control_available = bool(self._control_world_client.wait_for_service(timeout_sec=0.05))
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
            mode = "gzserver (headless)" if self.runtime_cfg.headless else "gz sim (GUI)"
            self._gzserver_log_fp.write(f"\n--- start {mode} world={world_sdf_file} ---\n")
            self._gzserver_log_fp.flush()
            gz_cmd = [
                "gz",
                "sim",
                "-r",
                "-v",
                "4",
                "--physics-engine",
                "gz-physics-bullet-featherstone-plugin",
            ]
            if self.runtime_cfg.headless:
                gz_cmd.insert(2, "-s")
            gz_cmd.append(world_sdf_file)
            self._gzserver_proc = subprocess.Popen(
                gz_cmd,
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
                    f"/model/{self.runtime_cfg.model_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    f"/model/{self.runtime_cfg.model_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    f"/world/{self.runtime_cfg.world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                    f"/world/{self.runtime_cfg.world_name}/create@ros_gz_interfaces/srv/SpawnEntity",
                    f"/world/{self.runtime_cfg.world_name}/remove@ros_gz_interfaces/srv/DeleteEntity",
                    f"/world/{self.runtime_cfg.world_name}/control@ros_gz_interfaces/srv/ControlWorld",
                    f"/ml25d/{self.runtime_cfg.model_name}/chassis_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    f"/ml25d/{self.runtime_cfg.model_name}/front_left_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    f"/ml25d/{self.runtime_cfg.model_name}/front_right_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    f"/ml25d/{self.runtime_cfg.model_name}/rear_left_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    f"/ml25d/{self.runtime_cfg.model_name}/rear_right_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
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

    def _set_paused(self, paused: bool) -> None:
        assert self._node is not None
        if not self._pause_control_available or self._control_world_client is None:
            return
        req = ControlWorld.Request()
        req.world_control.pause = bool(paused)
        future = self._control_world_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self.runtime_cfg.service_timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError("world control service call timed out while toggling pause")
        if not bool(future.result().success):
            raise RuntimeError("world control service returned success=False while toggling pause")

    def _reset_sample_buffers(self) -> None:
        self._odom_buffer.clear()
        self._contact_buffer.clear()
        self._sample_buffer_reset_total += 1

    def _wait_start_stability_gate(
        self,
        *,
        context: SimulationContext,
        sample_epoch_recv_time: float,
    ) -> dict[str, float]:
        assert self._node is not None
        max_wait_sec = max(6.0, float(context.settle_time_sec) + 3.0)
        deadline = time.monotonic() + max_wait_sec
        last_diag: dict[str, float] = {}

        while time.monotonic() < deadline:
            self._publish_cmd(0.0, 0.0)
            rclpy.spin_once(self._node, timeout_sec=1.0 / max(context.sample_rate_hz, 1))

            sample = self._odom_buffer.latest(min_recv_time_sec=sample_epoch_recv_time)
            if sample is None:
                continue
            # Guard against rare stale / corrupted odometry right after respawn.
            # We only evaluate the gate on samples close to the intended reset pose.
            if (
                abs(float(sample.position_x)) > 2.0
                or abs(float(sample.position_y)) > 2.0
                or abs(float(sample.position_z) - float(self._last_spawn_z)) > 1.5
            ):
                continue

            roll_i, pitch_i, yaw_i = self._quat_to_rpy(
                sample.orientation_x,
                sample.orientation_y,
                sample.orientation_z,
                sample.orientation_w,
            )
            roll_expected, pitch_expected, _ = self._estimate_support_plane_rpy(
                heightmap=context.heightmap,
                vehicle=context.vehicle,
                center_x=float(sample.position_x),
                center_y=float(sample.position_y),
                heading_rad=float(yaw_i),
            )
            roll_actual_deg = float(np.rad2deg(roll_i))
            pitch_actual_deg = float(np.rad2deg(pitch_i))
            roll_expected_deg = float(np.rad2deg(roll_expected))
            pitch_expected_deg = float(np.rad2deg(pitch_expected))

            odom_planar_fallback = bool(
                abs(roll_actual_deg) < 0.25
                and abs(pitch_actual_deg) < 0.25
                and (abs(roll_expected_deg) > 5.0 or abs(pitch_expected_deg) > 5.0)
            )
            if odom_planar_fallback:
                roll_gate = float(roll_expected)
                pitch_gate = float(pitch_expected)
            else:
                roll_gate = float(roll_i)
                pitch_gate = float(pitch_i)

            roll_abs_deg = float(abs(np.rad2deg(roll_gate)))
            pitch_abs_deg = float(abs(np.rad2deg(pitch_gate)))
            roll_error_gate_deg = float(
                abs(np.rad2deg(math.atan2(math.sin(roll_gate - roll_expected), math.cos(roll_gate - roll_expected))))
            )
            pitch_error_gate_deg = float(
                abs(np.rad2deg(math.atan2(math.sin(pitch_gate - pitch_expected), math.cos(pitch_gate - pitch_expected))))
            )
            roll_error_world_deg = float(
                abs(np.rad2deg(math.atan2(math.sin(roll_i - roll_expected), math.cos(roll_i - roll_expected))))
            )
            pitch_error_world_deg = float(
                abs(np.rad2deg(math.atan2(math.sin(pitch_i - pitch_expected), math.cos(pitch_i - pitch_expected))))
            )
            lin_x = float(sample.linear_x) if math.isfinite(sample.linear_x) else 0.0
            lin_y = float(sample.linear_y) if math.isfinite(sample.linear_y) else 0.0
            ang_z = float(sample.angular_z) if math.isfinite(sample.angular_z) else 0.0
            lin_speed = float(math.hypot(lin_x, lin_y))
            ang_speed = float(abs(ang_z))

            one_pos = np.array([[sample.position_x, sample.position_y]], dtype=np.float32)
            one_z = np.array([sample.position_z], dtype=np.float32)
            one_roll = np.array([roll_i], dtype=np.float32)
            one_pitch = np.array([pitch_i], dtype=np.float32)
            one_yaw = np.array([yaw_i], dtype=np.float32)
            wheel_clear = self._compute_wheel_clearance_center_sample(
                heightmap=context.heightmap,
                vehicle=context.vehicle,
                positions_xy=one_pos,
                positions_z=one_z,
                roll=np.array([roll_gate], dtype=np.float32),
                pitch=np.array([pitch_gate], dtype=np.float32),
                yaw=one_yaw,
            )[0]
            chassis_clear = float(
                self._compute_chassis_min_clearance(
                    heightmap=context.heightmap,
                    vehicle=context.vehicle,
                    positions_xy=one_pos,
                    positions_z=one_z,
                    roll=one_roll,
                    pitch=one_pitch,
                    yaw=one_yaw,
                )[0]
            )

            snapshot = self._contact_buffer.snapshot()
            required_wheels = ["front_left", "front_right", "rear_left", "rear_right"]
            fresh_wheel_count = 0
            fresh_wheel_nonzero_count = 0
            contact_msg_time_floor = float(sample.msg_time_sec)
            if float(sample.msg_time_sec) > 0.0 and float(self._last_sample_start_msg_time) > 0.0:
                odom_msg_fresh = float(sample.msg_time_sec) > float(self._last_sample_start_msg_time)
            else:
                odom_msg_fresh = float(sample.recv_time_sec) >= float(sample_epoch_recv_time)
            for key in required_wheels:
                entry = snapshot.get(
                    key,
                    {
                        "latest_recv_time": 0.0,
                        "latest_msg_time": 0.0,
                        "latest_nonzero_recv_time": 0.0,
                        "latest_nonzero_msg_time": 0.0,
                    },
                )
                latest_recv = float(entry.get("latest_recv_time", 0.0))
                latest_msg = float(entry.get("latest_msg_time", 0.0))
                latest_nz_recv = float(entry.get("latest_nonzero_recv_time", 0.0))
                latest_nz_msg = float(entry.get("latest_nonzero_msg_time", 0.0))
                wheel_fresh = max(latest_recv, latest_nz_recv) >= sample_epoch_recv_time
                if wheel_fresh:
                    fresh_wheel_count += 1
                if latest_nz_recv >= sample_epoch_recv_time:
                    fresh_wheel_nonzero_count += 1
                latest_msg_any = max(latest_msg, latest_nz_msg)
                if latest_msg_any > 0.0:
                    if latest_msg_any <= float(self._last_sample_start_msg_time):
                        wheel_fresh = False
                    contact_msg_time_floor = max(contact_msg_time_floor, latest_msg_any)
                    if not wheel_fresh and fresh_wheel_count > 0:
                        fresh_wheel_count -= 1

            contact_fresh = bool(fresh_wheel_count >= 2 and fresh_wheel_nonzero_count >= 1)

            chassis_entry = snapshot.get("chassis", {})
            chassis_recent = float(chassis_entry.get("latest_count", 0.0)) > 0 and (
                time.monotonic() - float(chassis_entry.get("latest_recv_time", 0.0))
            ) <= 0.25
            bottom_flag = bool(chassis_recent or chassis_clear < 0.005)
            wheel_clear_ok = bool(
                np.all(np.isfinite(wheel_clear))
                and float(np.min(wheel_clear)) > -0.03
                and float(np.max(wheel_clear)) < 0.30
            )
            contact_geom_ready = bool(
                np.all(np.isfinite(wheel_clear))
                and float(np.min(wheel_clear)) > -0.03
                and float(np.max(wheel_clear)) < 0.03
                and float(chassis_clear) > -0.01
            )
            lift_before_action = bool(float(np.max(wheel_clear)) > 0.05)

            last_diag = {
                "quat_x": float(sample.orientation_x),
                "quat_y": float(sample.orientation_y),
                "quat_z": float(sample.orientation_z),
                "quat_w": float(sample.orientation_w),
                "roll_actual_deg": roll_actual_deg,
                "pitch_actual_deg": pitch_actual_deg,
                "roll_gate_deg": float(np.rad2deg(roll_gate)),
                "pitch_gate_deg": float(np.rad2deg(pitch_gate)),
                "roll_abs_deg": roll_abs_deg,
                "pitch_abs_deg": pitch_abs_deg,
                "roll_error_gate_deg": roll_error_gate_deg,
                "pitch_error_gate_deg": pitch_error_gate_deg,
                "roll_error_world_deg": roll_error_world_deg,
                "pitch_error_world_deg": pitch_error_world_deg,
                "roll_dev_deg": roll_error_gate_deg,
                "pitch_dev_deg": pitch_error_gate_deg,
                "roll_expected_deg": roll_expected_deg,
                "pitch_expected_deg": pitch_expected_deg,
                "odom_planar_fallback": 1.0 if odom_planar_fallback else 0.0,
                "linear_speed": lin_speed,
                "angular_speed": ang_speed,
                "chassis_min_clearance": chassis_clear,
                "wheel_clearance_min": float(np.min(wheel_clear)),
                "wheel_clearance_max": float(np.max(wheel_clear)),
                "contact_fresh": float(contact_fresh),
                "contact_fresh_wheels": float(fresh_wheel_count),
                "contact_fresh_nonzero_wheels": float(fresh_wheel_nonzero_count),
                "contact_geom_ready": 1.0 if contact_geom_ready else 0.0,
                "odom_msg_fresh": float(odom_msg_fresh),
                "bottom_flag": float(bottom_flag),
                "lift_before_action": 1.0 if lift_before_action else 0.0,
                "odom_recv_time": float(sample.recv_time_sec),
                "odom_msg_time": float(sample.msg_time_sec),
                "contact_msg_time_floor": float(contact_msg_time_floor),
            }
            # When contact topics are transiently stale after respawn, use the
            # geometric groundedness check as a conservative fallback.
            if (not contact_fresh) and contact_geom_ready:
                contact_fresh = True
                last_diag["contact_fresh"] = 1.0
                last_diag["contact_fallback_geom"] = 1.0
            else:
                last_diag["contact_fallback_geom"] = 0.0

            if (
                roll_error_gate_deg < 4.0
                and pitch_error_gate_deg < 4.0
                and roll_abs_deg < 35.0
                and pitch_abs_deg < 35.0
                and lin_speed < 0.02
                and ang_speed < 0.05
                and (not bottom_flag)
                and (not lift_before_action)
                and contact_fresh
                and odom_msg_fresh
                and wheel_clear_ok
            ):
                return {
                    "sample_start_msg_time": float(max(sample.msg_time_sec, contact_msg_time_floor)),
                    "sample_start_recv_time": float(time.monotonic()),
                    **last_diag,
                }

        raise StartGateError(last_diag)

    def _run_once(self, context: SimulationContext, rng: np.random.Generator) -> SimulationTrajectory:
        assert self._node is not None
        assert self._cmd_pub is not None
        assert self._set_pose_client is not None
        assert self._spawn_client is not None
        assert self._delete_client is not None

        # A/B/C/D/E sequence: reset -> spawn -> settle -> gate -> start
        reset_before = int(self._sample_buffer_reset_total)
        debug_info: dict[str, float] = {
            "buffer_reset_before": float(reset_before),
            "sample_start_msg_time": float("nan"),
            "sample_start_recv_time": float("nan"),
            "odom_msg_time_min": float("nan"),
            "odom_msg_time_max": float("nan"),
            "contact_msg_time_min": float("nan"),
            "contact_msg_time_max": float("nan"),
        }
        self._set_paused(True)
        self._prepare_scene(context)
        self._reset_sample_buffers()
        self._reset_vehicle_pose(
            heading_rad=context.heading_rad,
            roll_rad=self._last_spawn_roll,
            pitch_rad=self._last_spawn_pitch,
            z_pos=self._last_spawn_z,
        )
        self._publish_cmd(0.0, 0.0)
        self._set_paused(False)
        self._wait_for_reset_pose(context.heading_rad)
        self._reset_sample_buffers()
        sample_epoch_recv_time = time.monotonic()

        settle_sec = float(np.clip(context.settle_time_sec, 0.5, 1.0))
        settle_steps = max(int(settle_sec * context.sample_rate_hz), 1)
        for _ in range(settle_steps):
            self._publish_cmd(0.0, 0.0)
            rclpy.spin_once(self._node, timeout_sec=1.0 / max(context.sample_rate_hz, 1))

        gate = self._wait_start_stability_gate(
            context=context,
            sample_epoch_recv_time=sample_epoch_recv_time,
        )
        sample_start_msg_time = float(gate["sample_start_msg_time"])
        if sample_start_msg_time <= 0.0:
            sample_start_msg_time = 0.0
        self._last_sample_start_msg_time = float(sample_start_msg_time)
        # Keep current-sample settle/gate sensor history and gate by per-sample time floor
        # so sparse contact streams still have valid continuity without leaking previous sample data.
        sample_start_recv_time = float(gate.get("sample_start_recv_time", sample_epoch_recv_time))
        if not math.isfinite(sample_start_recv_time):
            sample_start_recv_time = float(sample_epoch_recv_time)
        debug_info["sample_start_msg_time"] = float(sample_start_msg_time)
        debug_info["sample_start_recv_time"] = float(sample_start_recv_time)

        num_steps = max(int(context.duration_sec * context.sample_rate_hz), 2)
        dt = 1.0 / max(context.sample_rate_hz, 1)

        timestamps = np.arange(num_steps, dtype=np.float32) * dt
        positions = np.zeros((num_steps, 2), dtype=np.float32)
        positions_z = np.zeros(num_steps, dtype=np.float32)
        yaw = np.zeros(num_steps, dtype=np.float32)
        roll = np.zeros(num_steps, dtype=np.float32)
        pitch = np.zeros(num_steps, dtype=np.float32)
        actual_linear = np.zeros(num_steps, dtype=np.float32)
        actual_angular = np.zeros(num_steps, dtype=np.float32)
        wheel_contact_forces = np.zeros((num_steps, 4), dtype=np.float32)
        wheel_contact_observed = np.zeros((num_steps, 4), dtype=bool)
        wheel_contact_latched = np.zeros((num_steps, 4), dtype=bool)
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

        commanded_linear = np.zeros(num_steps, dtype=np.float32)
        commanded_angular = np.zeros(num_steps, dtype=np.float32)
        ramp_sec = max(float(context.cmd_ramp_sec), 0.0)

        odom_seen = 0
        odom_msg_min = float("inf")
        odom_msg_max = float("-inf")
        contact_msg_min = float("inf")
        contact_msg_max = float("-inf")
        prev_contact_counts = self._contact_buffer.snapshot()
        for idx in range(num_steps):
            step_start = time.monotonic()
            if ramp_sec > 1e-6:
                t = float(idx) * dt
                scale = float(np.clip(t / ramp_sec, 0.0, 1.0))
            else:
                scale = 1.0
            v_step = float(v_cmd * scale)
            w_step = float(w_cmd * scale)
            commanded_linear[idx] = v_step
            commanded_angular[idx] = w_step

            self._publish_cmd(v_step, w_step)
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
                wheel_contact_observed=wheel_contact_observed,
                wheel_contact_latched=wheel_contact_latched,
                chassis_contacts=chassis_contacts,
                previous=prev_contact_counts,
                min_recv_time_sec=sample_start_recv_time,
                min_msg_time_sec=sample_start_msg_time,
            )
            for key in ["front_left", "front_right", "rear_left", "rear_right", "chassis"]:
                entry = prev_contact_counts.get(key, {})
                for fld in ["latest_msg_time", "latest_nonzero_msg_time"]:
                    v = float(entry.get(fld, 0.0))
                    if v > 0.0 and (sample_start_msg_time <= 0.0 or v >= sample_start_msg_time):
                        contact_msg_min = min(contact_msg_min, v)
                        contact_msg_max = max(contact_msg_max, v)

            sample = self._odom_buffer.latest(
                min_recv_time_sec=sample_start_recv_time,
                min_msg_time_sec=(sample_start_msg_time if sample_start_msg_time > 0.0 else None),
            )
            if sample is None:
                if idx > 0:
                    positions[idx] = positions[idx - 1]
                    positions_z[idx] = positions_z[idx - 1]
                    yaw[idx] = yaw[idx - 1]
                    roll[idx] = roll[idx - 1]
                    pitch[idx] = pitch[idx - 1]
                    actual_linear[idx] = actual_linear[idx - 1]
                    actual_angular[idx] = actual_angular[idx - 1]
                continue

            # ros_gz occasionally emits stale/teleported odometry frames during reset/spawn churn.
            # Treat large per-step jumps as outliers instead of hard-failing the sample.
            if idx > 0:
                jump = math.hypot(sample.position_x - float(positions[idx - 1, 0]), sample.position_y - float(positions[idx - 1, 1]))
                if jump > 0.40:
                    positions[idx] = positions[idx - 1]
                    positions_z[idx] = positions_z[idx - 1]
                    yaw[idx] = yaw[idx - 1]
                    roll[idx] = roll[idx - 1]
                    pitch[idx] = pitch[idx - 1]
                    actual_linear[idx] = actual_linear[idx - 1]
                    actual_angular[idx] = actual_angular[idx - 1]
                    continue

            odom_seen += 1
            if float(sample.msg_time_sec) > 0.0 and (sample_start_msg_time <= 0.0 or float(sample.msg_time_sec) >= sample_start_msg_time):
                odom_msg_min = min(odom_msg_min, float(sample.msg_time_sec))
                odom_msg_max = max(odom_msg_max, float(sample.msg_time_sec))
            positions[idx, 0] = sample.position_x
            positions[idx, 1] = sample.position_y
            positions_z[idx] = sample.position_z

            roll_i, pitch_i, yaw_i = self._quat_to_rpy(
                sample.orientation_x,
                sample.orientation_y,
                sample.orientation_z,
                sample.orientation_w,
            )
            if idx > 0:
                # ros_gz occasionally reports orientation outliers (e.g. near 180 deg roll spikes)
                # even when pose remains continuous. Treat large single-step jumps as stale odom.
                prev_roll = float(roll[idx - 1])
                prev_pitch = float(pitch[idx - 1])
                prev_yaw = float(yaw[idx - 1])
                d_roll = abs(math.atan2(math.sin(roll_i - prev_roll), math.cos(roll_i - prev_roll)))
                d_pitch = abs(math.atan2(math.sin(pitch_i - prev_pitch), math.cos(pitch_i - prev_pitch)))
                d_yaw = abs(math.atan2(math.sin(yaw_i - prev_yaw), math.cos(yaw_i - prev_yaw)))
                extreme_tilt = abs(roll_i) > 1.20 or abs(pitch_i) > 1.20
                if (d_roll > 0.45 or d_pitch > 0.45 or d_yaw > 0.80) or (
                    extreme_tilt and (d_roll > 0.25 or d_pitch > 0.25)
                ):
                    positions[idx] = positions[idx - 1]
                    positions_z[idx] = positions_z[idx - 1]
                    yaw[idx] = yaw[idx - 1]
                    roll[idx] = roll[idx - 1]
                    pitch[idx] = pitch[idx - 1]
                    actual_linear[idx] = actual_linear[idx - 1]
                    actual_angular[idx] = actual_angular[idx - 1]
                    continue
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
        odom_linear = actual_linear.copy()
        odom_angular = actual_angular.copy()
        actual_linear_from_pose = self._estimate_forward_speed(positions, unwrapped_yaw, dt)
        actual_angular_from_pose = np.gradient(unwrapped_yaw, dt).astype(np.float32)
        debug_info["odom_linear_abs_max"] = float(np.max(np.abs(odom_linear)))
        debug_info["pose_forward_abs_max"] = float(np.max(np.abs(actual_linear_from_pose)))
        debug_info["odom_pose_forward_mae"] = float(np.mean(np.abs(odom_linear - actual_linear_from_pose)))
        debug_info["odom_angular_abs_max"] = float(np.max(np.abs(odom_angular)))
        debug_info["pose_angular_abs_max"] = float(np.max(np.abs(actual_angular_from_pose)))
        debug_info["odom_pose_angular_mae"] = float(np.mean(np.abs(odom_angular - actual_angular_from_pose)))
        # In ros_gz DiffDrive odom.twist.linear.x is body-forward speed and tends to be
        # more stable than finite-difference pose estimates for slip labels.
        # Keep pose-derived speeds as deterministic fallback when odom twist is unavailable.
        if float(np.max(np.abs(odom_linear))) <= 1e-3:
            actual_linear = actual_linear_from_pose.astype(np.float32)
        else:
            actual_linear = odom_linear.astype(np.float32)
        if float(np.max(np.abs(odom_angular))) <= 1e-3:
            actual_angular = actual_angular_from_pose.astype(np.float32)
        else:
            actual_angular = odom_angular.astype(np.float32)
        contact_stream_missing = False
        if np.max(wheel_contact_forces) <= 0.0:
            contact_snapshot = self._contact_buffer.snapshot()
            wheel_keys = ["front_left", "front_right", "rear_left", "rear_right"]
            compact = {
                key: {
                    "total_contacts": int(val.get("count_total", 0)),
                    "total_force": float(val.get("force_total", 0.0)),
                    "latest_contacts": int(val.get("latest_count", 0)),
                    "latest_force": float(val.get("latest_force", 0.0)),
                }
                for key, val in contact_snapshot.items()
            }
            total_contact_events = 0
            fresh_contact_topics = 0
            base_load = float(context.vehicle.m * 9.81 / 4.0)
            for w, key in enumerate(wheel_keys):
                entry = contact_snapshot.get(key, {})
                total_contact_events += int(entry.get("count_total", 0))
                latest_recv = float(entry.get("latest_recv_time", 0.0))
                if latest_recv >= float(sample_start_recv_time):
                    fresh_contact_topics += 1
                    # Preserve observability for sparse/zero-force contact streams.
                    wheel_contact_observed[:, w] = True
                    wheel_contact_latched[:, w] = True
                    wheel_contact_forces[:, w] = np.maximum(wheel_contact_forces[:, w], base_load).astype(np.float32)

            if total_contact_events <= 0 and fresh_contact_topics <= 0:
                contact_stream_missing = True
        wheel_clearance = self._compute_wheel_clearance(
            heightmap=context.heightmap,
            vehicle=context.vehicle,
            positions_xy=positions,
            positions_z=positions_z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
        chassis_min_clearance = self._compute_chassis_min_clearance(
            heightmap=context.heightmap,
            vehicle=context.vehicle,
            positions_xy=positions,
            positions_z=positions_z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
        if contact_stream_missing:
            # ros_gz contact topics can be absent for some episodes. Use geometric
            # groundedness to rebuild minimal contact observability.
            base_load = float(context.vehicle.m * 9.81 / 4.0)
            grounded_mask = np.isfinite(wheel_clearance) & (wheel_clearance < 0.03)
            if np.any(grounded_mask):
                wheel_contact_observed = np.logical_or(wheel_contact_observed, grounded_mask)
                wheel_contact_latched = np.logical_or(wheel_contact_latched, grounded_mask)
                wheel_contact_forces = np.where(
                    grounded_mask,
                    np.maximum(wheel_contact_forces, base_load),
                    wheel_contact_forces,
                ).astype(np.float32)

        observed_force = np.any(wheel_contact_observed, axis=1)
        contact_observability = float(np.mean(observed_force))
        if contact_observability < 0.08:
            reason = "missing contact topics + geometric fallback unavailable" if contact_stream_missing else "insufficient contact topic coverage"
            raise RuntimeError(
                "insufficient wheel contact observability from Gazebo "
                f"(coverage={contact_observability:.3f}, threshold=0.080, reason={reason})"
            )
        lift_clearance_threshold = max(0.04, 0.25 * float(context.vehicle.r_w))
        grounded_clearance_threshold = 0.012
        wheel_lift_valid_mask = np.zeros((num_steps, 4), dtype=bool)
        wheel_lift_state = np.zeros((num_steps, 4), dtype=bool)
        for t in range(num_steps):
            for w in range(4):
                if wheel_contact_latched[t, w]:
                    wheel_lift_valid_mask[t, w] = True
                    wheel_lift_state[t, w] = False
                    continue
                clearance = float(wheel_clearance[t, w])
                if clearance > lift_clearance_threshold:
                    wheel_lift_valid_mask[t, w] = True
                    wheel_lift_state[t, w] = True
                elif clearance < grounded_clearance_threshold:
                    wheel_lift_valid_mask[t, w] = True
                    wheel_lift_state[t, w] = False

        completed_displacement = float(np.linalg.norm(positions[-1] - positions[0]))
        completed_heading = float(abs(unwrapped_yaw[-1] - unwrapped_yaw[0]))
        first_window = min(num_steps, max(int(round(0.5 / max(dt, 1e-6))), 1))
        debug_info["roll_abs_max_deg"] = float(np.rad2deg(np.max(np.abs(roll))))
        debug_info["pitch_abs_max_deg"] = float(np.rad2deg(np.max(np.abs(pitch))))
        debug_info["roll_abs_max_first_0p5s_deg"] = float(np.rad2deg(np.max(np.abs(roll[:first_window]))))
        debug_info["pitch_abs_max_first_0p5s_deg"] = float(np.rad2deg(np.max(np.abs(pitch[:first_window]))))
        debug_info["roll_abs_max_after_0p5s_deg"] = float(
            np.rad2deg(np.max(np.abs(roll[first_window:]))) if first_window < num_steps else 0.0
        )
        debug_info["pitch_abs_max_after_0p5s_deg"] = float(
            np.rad2deg(np.max(np.abs(pitch[first_window:]))) if first_window < num_steps else 0.0
        )
        debug_info["yaw_abs_max_deg"] = float(np.rad2deg(np.max(np.abs(unwrapped_yaw))))
        debug_info["completed_displacement_m"] = completed_displacement
        debug_info["completed_heading_change_deg"] = float(np.rad2deg(completed_heading))

        debug_info["odom_msg_time_min"] = float(odom_msg_min) if np.isfinite(odom_msg_min) else float("nan")
        debug_info["odom_msg_time_max"] = float(odom_msg_max) if np.isfinite(odom_msg_max) else float("nan")
        debug_info["contact_msg_time_min"] = float(contact_msg_min) if np.isfinite(contact_msg_min) else float("nan")
        debug_info["contact_msg_time_max"] = float(contact_msg_max) if np.isfinite(contact_msg_max) else float("nan")
        reset_after = int(self._sample_buffer_reset_total)
        debug_info["buffer_reset_after"] = float(reset_after)
        debug_info["buffer_reset_delta"] = float(reset_after - reset_before)
        self._last_run_debug = debug_info

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
            wheel_contact_observed=wheel_contact_observed,
            wheel_contact_latched=wheel_contact_latched,
            wheel_clearance_m=wheel_clearance,
            wheel_lift_valid_mask=wheel_lift_valid_mask,
            wheel_lift_state=wheel_lift_state,
            chassis_min_clearance_m=chassis_min_clearance,
        )

    def _prepare_scene(self, context: SimulationContext) -> None:
        scene_suffix = ""
        if context.scene_id:
            scene_suffix = "_" + "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in str(context.scene_id))
        terrain_name = f"ml25d_terrain{scene_suffix}"
        model_name = self.runtime_cfg.model_name
        # Critical ordering: delete vehicle first, then wait for odom stream to go quiet
        # before touching terrain. This avoids stale old-vehicle odom frames (often from
        # brief "fall-through" windows) leaking into the next sample.
        self._delete_entity(model_name, require_success=False)
        self._wait_for_odom_quiet(quiet_sec=0.15, timeout_sec=1.5)

        names_to_delete = sorted(self._active_obstacle_names)
        if self._active_terrain_name is not None:
            names_to_delete.append(self._active_terrain_name)
        if terrain_name not in names_to_delete:
            names_to_delete.append(terrain_name)
        for name in names_to_delete:
            self._delete_entity(name, require_success=False)
        self._active_obstacle_names.clear()
        self._active_terrain_name = None
        if self._node is not None:
            for _ in range(6):
                rclpy.spin_once(self._node, timeout_sec=0.02)

        self._spawn_entity(
            name=terrain_name,
            sdf=self._terrain_sdf(terrain_name, context.heightmap, context.friction_mu),
            z=0.0,
        )
        self._active_terrain_name = terrain_name
        self._last_spawn_z, self._last_spawn_roll, self._last_spawn_pitch = self._estimate_initial_vehicle_pose(
            heightmap=context.heightmap,
            vehicle=context.vehicle,
            heading_rad=context.heading_rad,
        )

        if context.extra_obstacles:
            for idx, obs in enumerate(context.extra_obstacles):
                obs_name = f"ml25d_obstacle{scene_suffix}_{idx}"
                length = float(obs.get("length_m", 0.2))
                width = float(obs.get("width_m", 0.6))
                height = float(obs.get("height_m", 0.1))
                x = float(obs.get("x_m", 0.25))
                y = float(obs.get("y_m", 0.0))
                base_h = self._support_height_under_wheel(context.heightmap, x=x, y=y, wheel_radius=max(length, width) * 0.5)
                self._spawn_entity(
                    name=obs_name,
                    sdf=self._box_obstacle_sdf(obs_name, length_m=length, width_m=width, height_m=height),
                    z=base_h + 0.5 * height,
                    x=x,
                    y=y,
                )
                self._active_obstacle_names.add(obs_name)

        self._spawn_entity(
            name=model_name,
            sdf=self._vehicle_sdf(model_name, context.vehicle),
            z=self._last_spawn_z,
            roll=self._last_spawn_roll,
            pitch=self._last_spawn_pitch,
            yaw=context.heading_rad,
        )

        assert self._node is not None
        deadline = time.time() + min(self.runtime_cfg.service_timeout_sec, 3.0)
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def _wait_for_odom_quiet(self, quiet_sec: float, timeout_sec: float) -> None:
        assert self._node is not None
        quiet_sec = max(float(quiet_sec), 0.05)
        timeout_sec = max(float(timeout_sec), quiet_sec + 0.2)
        start = time.monotonic()
        last_activity = start
        sample = self._odom_buffer.latest()
        last_recv = float(sample.recv_time_sec) if sample is not None else 0.0

        while (time.monotonic() - start) < timeout_sec:
            rclpy.spin_once(self._node, timeout_sec=0.02)
            sample = self._odom_buffer.latest()
            recv = float(sample.recv_time_sec) if sample is not None else last_recv
            if recv > (last_recv + 1e-6):
                last_recv = recv
                last_activity = time.monotonic()
                continue
            if (time.monotonic() - last_activity) >= quiet_sec:
                return

        raise RuntimeError("odometry stream did not become quiet after vehicle deletion")

    def _spawn_entity(
        self,
        name: str,
        sdf: str,
        z: float,
        x: float = 0.0,
        y: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        assert self._spawn_client is not None
        assert self._node is not None

        qx, qy, qz, qw = self._rpy_to_quat(roll, pitch, yaw)
        for attempt in range(2):
            req = SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.sdf = sdf
            req.entity_factory.allow_renaming = False
            req.entity_factory.pose.position.x = float(x)
            req.entity_factory.pose.position.y = float(y)
            req.entity_factory.pose.position.z = float(z)
            req.entity_factory.pose.orientation.x = float(qx)
            req.entity_factory.pose.orientation.y = float(qy)
            req.entity_factory.pose.orientation.z = float(qz)
            req.entity_factory.pose.orientation.w = float(qw)

            future = self._spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=self.runtime_cfg.service_timeout_sec)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"spawn service call timed out for {name}")
            resp = future.result()
            if bool(resp.success):
                self._spawned_entities.add(name)
                return

            status_message = str(getattr(resp, "status_message", ""))
            if attempt == 0 and "already exists" in status_message.lower():
                self._delete_entity(name, require_success=False)
                for _ in range(6):
                    rclpy.spin_once(self._node, timeout_sec=0.02)
                continue
            raise RuntimeError(f"spawn service returned success=False for {name}: {status_message}")

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
        wheel_contact_observed: np.ndarray,
        wheel_contact_latched: np.ndarray,
        chassis_contacts: np.ndarray,
        previous: dict[str, dict[str, float | int]],
        min_recv_time_sec: float,
        min_msg_time_sec: float,
    ) -> dict[str, dict[str, float | int]]:
        current = self._contact_buffer.snapshot()
        keys = ["chassis", "front_left", "front_right", "rear_left", "rear_right"]
        zero = {
            "count_total": 0,
            "force_total": 0.0,
            "latest_count": 0,
            "latest_force": 0.0,
            "latest_recv_time": 0.0,
            "latest_msg_time": 0.0,
            "latest_nonzero_count": 0,
            "latest_nonzero_force": 0.0,
            "latest_nonzero_recv_time": 0.0,
            "latest_nonzero_msg_time": 0.0,
        }
        delta_counts = {
            key: int(current.get(key, zero).get("count_total", 0)) - int(previous.get(key, zero).get("count_total", 0))
            for key in keys
        }
        delta_forces = {
            key: float(current.get(key, zero).get("force_total", 0.0)) - float(previous.get(key, zero).get("force_total", 0.0))
            for key in keys
        }
        base_load = float(vehicle.m * 9.81 / 4.0)
        now = time.monotonic()
        hold_sec = 0.25

        def _msg_is_fresh(entry: dict[str, float | int], key_recv: str, key_msg: str) -> bool:
            recv_time = float(entry.get(key_recv, 0.0))
            msg_time = float(entry.get(key_msg, 0.0))
            recv_ok = recv_time >= float(min_recv_time_sec)
            if min_msg_time_sec > 0.0:
                # Some ros_gz contact headers can carry zero/invalid sim time.
                # In that case, fall back to recv-time freshness instead of dropping all contacts.
                msg_ok = True if msg_time <= 0.0 else (msg_time >= float(min_msg_time_sec))
            else:
                msg_ok = True
            return recv_ok and msg_ok

        def force_or_fallback(key: str, wheel_idx: int) -> float:
            entry = current.get(key, zero)
            force = float(delta_forces[key])
            if force > 0.0 and _msg_is_fresh(entry, "latest_recv_time", "latest_msg_time"):
                wheel_contact_observed[idx, wheel_idx] = True
                wheel_contact_latched[idx, wheel_idx] = True
                return force
            if delta_counts[key] > 0 and _msg_is_fresh(entry, "latest_recv_time", "latest_msg_time"):
                wheel_contact_observed[idx, wheel_idx] = True
                wheel_contact_latched[idx, wheel_idx] = True
                return base_load
            latest_count = int(entry.get("latest_count", 0))
            latest_force = float(entry.get("latest_force", 0.0))
            latest_stamp = float(entry.get("latest_recv_time", 0.0))
            if latest_count > 0 and (now - latest_stamp) <= hold_sec and _msg_is_fresh(entry, "latest_recv_time", "latest_msg_time"):
                wheel_contact_observed[idx, wheel_idx] = True
                wheel_contact_latched[idx, wheel_idx] = True
                return latest_force if latest_force > 0.0 else base_load
            latest_nonzero_count = int(entry.get("latest_nonzero_count", 0))
            latest_nonzero_force = float(entry.get("latest_nonzero_force", 0.0))
            latest_nonzero_stamp = float(entry.get("latest_nonzero_recv_time", 0.0))
            if (
                latest_nonzero_count > 0
                and (now - latest_nonzero_stamp) <= hold_sec
                and _msg_is_fresh(entry, "latest_nonzero_recv_time", "latest_nonzero_msg_time")
            ):
                wheel_contact_observed[idx, wheel_idx] = False
                wheel_contact_latched[idx, wheel_idx] = True
                return latest_nonzero_force if latest_nonzero_force > 0.0 else base_load
            wheel_contact_observed[idx, wheel_idx] = False
            wheel_contact_latched[idx, wheel_idx] = False
            return 0.0

        forces[idx] = np.array(
            [
                force_or_fallback("front_left", 0),
                force_or_fallback("front_right", 1),
                force_or_fallback("rear_left", 2),
                force_or_fallback("rear_right", 3),
            ],
            dtype=np.float32,
        )
        chassis_entry = current.get("chassis", zero)
        latest_chassis_count = int(chassis_entry.get("latest_count", 0))
        latest_chassis_stamp = float(chassis_entry.get("latest_recv_time", 0.0))
        chassis_recent = (
            latest_chassis_count > 0
            and (now - latest_chassis_stamp) <= hold_sec
            and _msg_is_fresh(chassis_entry, "latest_recv_time", "latest_msg_time")
        )
        chassis_contacts[idx] = 1 if (delta_counts["chassis"] > 0 or chassis_recent) else 0
        return current

    def _estimate_initial_vehicle_pose(
        self,
        heightmap: np.ndarray,
        vehicle: VehicleParams,
        heading_rad: float,
    ) -> tuple[float, float, float]:
        if heightmap.size == 0:
            return 0.03, 0.0, 0.0
        roll, pitch, plane_c = self._estimate_support_plane_rpy(
            heightmap=heightmap,
            vehicle=vehicle,
            center_x=0.0,
            center_y=0.0,
            heading_rad=heading_rad,
        )

        rot = self._rpy_to_rot(roll, pitch, heading_rad)
        offsets = self._wheel_offsets(vehicle)
        required_z: list[float] = []

        # Wheel-ground support constraints.
        for w in range(4):
            off = offsets[w]
            wheel_center_offset = rot @ off
            support_h = self._support_height_under_wheel(
                heightmap=heightmap,
                x=float(wheel_center_offset[0]),
                y=float(wheel_center_offset[1]),
                wheel_radius=float(vehicle.r_w),
            )
            world_z_off = float(wheel_center_offset[2])
            required_z.append(support_h + float(vehicle.r_w) - world_z_off + 0.028)

        # Chassis underside clearance constraints to avoid immediate bottom penetration.
        sample_points = self._chassis_underside_sample_points(vehicle=vehicle, nx=5, ny=3)
        for sp in sample_points:
            world_off = rot @ sp
            terrain_h = self._sample_height_bilinear(heightmap=heightmap, x=float(world_off[0]), y=float(world_off[1]))
            required_z.append(float(terrain_h) - float(world_off[2]) + 0.020)

        # Fallback plane height around origin.
        required_z.append(plane_c + 0.028)
        return float(max(required_z)), roll, pitch

    def _estimate_support_plane_rpy(
        self,
        *,
        heightmap: np.ndarray,
        vehicle: VehicleParams,
        center_x: float,
        center_y: float,
        heading_rad: float,
    ) -> tuple[float, float, float]:
        offsets = self._wheel_offsets(vehicle)
        cy = math.cos(heading_rad)
        sy = math.sin(heading_rad)
        xw: list[float] = []
        yw: list[float] = []
        zw: list[float] = []
        for off in offsets:
            wx = float(center_x) + cy * float(off[0]) - sy * float(off[1])
            wy = float(center_y) + sy * float(off[0]) + cy * float(off[1])
            support_h = self._support_height_under_wheel(
                heightmap=heightmap,
                x=wx,
                y=wy,
                wheel_radius=float(vehicle.r_w),
            )
            xw.append(wx)
            yw.append(wy)
            zw.append(float(support_h))

        a = b = c = 0.0
        if len(zw) >= 3:
            A = np.column_stack(
                [
                    np.asarray(xw, dtype=np.float64),
                    np.asarray(yw, dtype=np.float64),
                    np.ones(len(zw), dtype=np.float64),
                ]
            )
            z_arr = np.asarray(zw, dtype=np.float64)
            coef, *_ = np.linalg.lstsq(A, z_arr, rcond=None)
            a, b, c = float(coef[0]), float(coef[1]), float(coef[2])

        slope_forward = a * math.cos(heading_rad) + b * math.sin(heading_rad)
        slope_lateral = -a * math.sin(heading_rad) + b * math.cos(heading_rad)
        pitch = float(np.clip(-math.atan(slope_forward), -np.deg2rad(35.0), np.deg2rad(35.0)))
        roll = float(np.clip(math.atan(slope_lateral), -np.deg2rad(35.0), np.deg2rad(35.0)))
        return roll, pitch, c

    def _compute_wheel_clearance(
        self,
        heightmap: np.ndarray,
        vehicle: VehicleParams,
        positions_xy: np.ndarray,
        positions_z: np.ndarray,
        roll: np.ndarray,
        pitch: np.ndarray,
        yaw: np.ndarray,
    ) -> np.ndarray:
        wheel_offsets = self._wheel_offsets(vehicle)
        clearances = np.zeros((positions_xy.shape[0], 4), dtype=np.float32)
        for t in range(positions_xy.shape[0]):
            rot = self._rpy_to_rot(float(roll[t]), float(pitch[t]), float(yaw[t]))
            model_pos = np.array([positions_xy[t, 0], positions_xy[t, 1], positions_z[t]], dtype=np.float64)
            for w in range(4):
                wheel_center = model_pos + rot @ wheel_offsets[w]
                support_h = self._support_height_under_wheel(
                    heightmap=heightmap,
                    x=float(wheel_center[0]),
                    y=float(wheel_center[1]),
                    wheel_radius=float(vehicle.r_w),
                )
                clearances[t, w] = float(wheel_center[2] - float(vehicle.r_w) - support_h)
        return clearances

    def _compute_wheel_clearance_center_sample(
        self,
        heightmap: np.ndarray,
        vehicle: VehicleParams,
        positions_xy: np.ndarray,
        positions_z: np.ndarray,
        roll: np.ndarray,
        pitch: np.ndarray,
        yaw: np.ndarray,
    ) -> np.ndarray:
        wheel_offsets = self._wheel_offsets(vehicle)
        clearances = np.zeros((positions_xy.shape[0], 4), dtype=np.float32)
        for t in range(positions_xy.shape[0]):
            rot = self._rpy_to_rot(float(roll[t]), float(pitch[t]), float(yaw[t]))
            model_pos = np.array([positions_xy[t, 0], positions_xy[t, 1], positions_z[t]], dtype=np.float64)
            for w in range(4):
                wheel_center = model_pos + rot @ wheel_offsets[w]
                terrain_h = self._sample_height_bilinear(heightmap=heightmap, x=float(wheel_center[0]), y=float(wheel_center[1]))
                clearances[t, w] = float(wheel_center[2] - float(vehicle.r_w) - terrain_h)
        return clearances

    def _compute_chassis_min_clearance(
        self,
        heightmap: np.ndarray,
        vehicle: VehicleParams,
        positions_xy: np.ndarray,
        positions_z: np.ndarray,
        roll: np.ndarray,
        pitch: np.ndarray,
        yaw: np.ndarray,
    ) -> np.ndarray:
        sample_points = self._chassis_underside_sample_points(vehicle=vehicle, nx=7, ny=5)
        clearances = np.zeros(positions_xy.shape[0], dtype=np.float32)
        for t in range(positions_xy.shape[0]):
            rot = self._rpy_to_rot(float(roll[t]), float(pitch[t]), float(yaw[t]))
            model_pos = np.array([positions_xy[t, 0], positions_xy[t, 1], positions_z[t]], dtype=np.float64)
            min_clearance = 1e9
            for sp in sample_points:
                p = model_pos + rot @ sp
                terrain_h = self._sample_height_bilinear(heightmap=heightmap, x=float(p[0]), y=float(p[1]))
                min_clearance = min(min_clearance, float(p[2] - terrain_h))
            clearances[t] = float(min_clearance)
        return clearances

    @staticmethod
    def _chassis_underside_sample_points(vehicle: VehicleParams, nx: int = 7, ny: int = 5) -> np.ndarray:
        # Sample the lower face of chassis collision geometry (base_link frame).
        x_extent = 0.48 * float(vehicle.L)
        y_extent = 0.48 * float(vehicle.W)
        xs = np.linspace(-x_extent, x_extent, max(nx, 2), dtype=np.float64)
        ys = np.linspace(-y_extent, y_extent, max(ny, 2), dtype=np.float64)
        bottom_offset = float(vehicle.r_w + vehicle.c_g)
        pts = [[float(x), float(y), bottom_offset] for x in xs for y in ys]
        return np.asarray(pts, dtype=np.float64)

    def _sample_height_bilinear(self, heightmap: np.ndarray, x: float, y: float) -> float:
        if heightmap.size == 0:
            return 0.0
        n, m = heightmap.shape
        ci = (n - 1) / 2.0
        cj = (m - 1) / 2.0
        i = x / max(self.resolution_m, 1e-6) + ci
        j = y / max(self.resolution_m, 1e-6) + cj
        i = float(np.clip(i, 0.0, n - 1.0))
        j = float(np.clip(j, 0.0, m - 1.0))

        i0 = int(math.floor(i))
        j0 = int(math.floor(j))
        i1 = min(i0 + 1, n - 1)
        j1 = min(j0 + 1, m - 1)
        ti = float(i - i0)
        tj = float(j - j0)

        h00 = float(heightmap[i0, j0])
        h10 = float(heightmap[i1, j0])
        h01 = float(heightmap[i0, j1])
        h11 = float(heightmap[i1, j1])

        h0 = h00 * (1.0 - ti) + h10 * ti
        h1 = h01 * (1.0 - ti) + h11 * ti
        return float(h0 * (1.0 - tj) + h1 * tj)

    @staticmethod
    def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return float(qx), float(qy), float(qz), float(qw)

    @staticmethod
    def _wheel_offsets(vehicle: VehicleParams) -> np.ndarray:
        wheel_x = min(float(vehicle.l) * 0.5, float(vehicle.L) * 0.42)
        wheel_y = max(float(vehicle.b) * 0.5, float(vehicle.W) * 0.45)
        radius = float(vehicle.r_w)
        return np.array(
            [
                [wheel_x, wheel_y, radius],
                [wheel_x, -wheel_y, radius],
                [-wheel_x, wheel_y, radius],
                [-wheel_x, -wheel_y, radius],
            ],
            dtype=np.float64,
        )

    def _support_height_under_wheel(self, heightmap: np.ndarray, x: float, y: float, wheel_radius: float) -> float:
        if heightmap.size == 0:
            return 0.0
        n, m = heightmap.shape
        center_i = (n - 1) / 2.0
        center_j = (m - 1) / 2.0
        i = x / max(self.resolution_m, 1e-6) + center_i
        j = y / max(self.resolution_m, 1e-6) + center_j
        i_idx = int(round(i))
        j_idx = int(round(j))
        window_cells = max(1, int(math.ceil((wheel_radius + 0.02) / max(self.resolution_m, 1e-6))))
        lo_i = max(i_idx - window_cells, 0)
        hi_i = min(i_idx + window_cells + 1, n)
        lo_j = max(j_idx - window_cells, 0)
        hi_j = min(j_idx + window_cells + 1, m)
        if lo_i >= hi_i or lo_j >= hi_j:
            return float(np.max(heightmap))
        return float(np.max(heightmap[lo_i:hi_i, lo_j:hi_j]))

    @staticmethod
    def _box_obstacle_sdf(model_name: str, length_m: float, width_m: float, height_m: float) -> str:
        return f"""
<sdf version='1.8'>
  <model name='{escape(model_name)}'>
    <static>true</static>
    <link name='obstacle_link'>
      <collision name='obstacle_collision'>
        <geometry><box><size>{length_m:.4f} {width_m:.4f} {height_m:.4f}</size></box></geometry>
      </collision>
      <visual name='obstacle_visual'>
        <geometry><box><size>{length_m:.4f} {width_m:.4f} {height_m:.4f}</size></box></geometry>
        <material><ambient>0.45 0.42 0.36 1</ambient><diffuse>0.55 0.50 0.40 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

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
        height_span = float(np.max(heightmap) - np.min(heightmap)) if heightmap.size > 0 else 0.0
        is_flat_plane = bool(height_span <= 1e-6)
        if is_flat_plane:
            size_x = max(float((heightmap.shape[0] - 1) * self.resolution_m), 0.5)
            size_y = max(float((heightmap.shape[1] - 1) * self.resolution_m), 0.5)
            collision_geom = (
                f"<geometry><plane><normal>0 0 1</normal><size>{size_x:.4f} {size_y:.4f}</size></plane></geometry>"
            )
            visual_geom = (
                f"<geometry><plane><normal>0 0 1</normal><size>{size_x:.4f} {size_y:.4f}</size></plane></geometry>"
            )
        else:
            mesh_uri = self._write_terrain_mesh(heightmap)
            collision_geom = f"<geometry><mesh><uri>{escape(mesh_uri)}</uri></mesh></geometry>"
            visual_geom = f"<geometry><mesh><uri>{escape(mesh_uri)}</uri></mesh></geometry>"
        return f"""
<sdf version='1.8'>
  <model name='{escape(model_name)}'>
    <static>true</static>
    <link name='terrain_link'>
      <collision name='terrain_collision'>
        {collision_geom}
        <surface>
          <friction>
            <ode><mu>{friction:.4f}</mu><mu2>{friction:.4f}</mu2></ode>
            <bullet><friction>{friction:.4f}</friction><friction2>{friction:.4f}</friction2><rolling_friction>0.03</rolling_friction></bullet>
          </friction>
          <contact><ode><kp>1000000</kp><kd>1</kd><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name='terrain_visual'>
        {visual_geom}
        <material><ambient>0.35 0.28 0.18 1</ambient><diffuse>0.45 0.36 0.22 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def _write_terrain_heightmap(self, heightmap: np.ndarray) -> tuple[str, float, float, float, float]:
        self._terrain_counter += 1
        terrain_dir = Path("/tmp/ml25d_ros_gz_heightmaps")
        terrain_dir.mkdir(parents=True, exist_ok=True)
        image_path = terrain_dir / f"terrain_{id(self)}_{self._terrain_counter}.png"

        h = heightmap.astype(np.float64)
        z_min = float(np.min(h))
        z_max = float(np.max(h))
        z_span = max(z_max - z_min, 0.01)
        normalized = np.clip((h - z_min) / z_span, 0.0, 1.0)
        # Gazebo heightmaps are most stable with power-of-two-plus-one images.
        image = Image.fromarray(np.round(normalized * 255.0).astype(np.uint8), mode="L")
        image = image.resize((129, 129), resample=Image.Resampling.BILINEAR)
        image.save(image_path)

        size_x = float((heightmap.shape[0] - 1) * self.resolution_m)
        size_y = float((heightmap.shape[1] - 1) * self.resolution_m)
        return image_path.as_uri(), size_x, size_y, z_span, z_min

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
        normals: list[tuple[float, float, float]] = []
        grad_x, grad_y = np.gradient(heightmap.astype(np.float64), self.resolution_m, edge_order=1)
        for level in ["top", "bottom"]:
            for i in range(n):
                for j in range(m):
                    x = (i - center_i) * self.resolution_m
                    y = (j - center_j) * self.resolution_m
                    z = float(heightmap[i, j]) if level == "top" else base_z
                    vertices.append((x, y, z))
                    if level == "top":
                        nx = -float(grad_x[i, j])
                        ny = -float(grad_y[i, j])
                        nz = 1.0
                        norm = max(math.sqrt(nx * nx + ny * ny + nz * nz), 1e-9)
                        normals.append((nx / norm, ny / norm, nz / norm))
                    else:
                        normals.append((0.0, 0.0, -1.0))

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
        lines.extend(f"vn {x:.6f} {y:.6f} {z:.6f}" for x, y, z in normals)
        lines.extend(f"f {a}//{a} {b}//{b} {c}//{c}" for a, b, c in faces)
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
    <joint name='front_left_wheel_joint' type='revolute'><parent>chassis</parent><child>front_left</child><axis><xyz>0 1 0</xyz></axis></joint>
    <joint name='front_right_wheel_joint' type='revolute'><parent>chassis</parent><child>front_right</child><axis><xyz>0 1 0</xyz></axis></joint>
    <joint name='rear_left_wheel_joint' type='revolute'><parent>chassis</parent><child>rear_left</child><axis><xyz>0 1 0</xyz></axis></joint>
    <joint name='rear_right_wheel_joint' type='revolute'><parent>chassis</parent><child>rear_right</child><axis><xyz>0 1 0</xyz></axis></joint>
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
      <pose>{x:.4f} {y:.4f} {radius:.4f} 0 0 0</pose>
      <inertial><mass>{mass:.4f}</mass><inertia><ixx>{i_radial:.6f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{i_axis:.6f}</iyy><iyz>0</iyz><izz>{i_radial:.6f}</izz></inertia></inertial>
      <collision name='collision_{escape(link_name)}'>
        <pose>0 0 0 1.5707 0 0</pose>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{width:.4f}</length></cylinder></geometry>
        <surface>
          <friction>
            <ode><mu>1.2</mu><mu2>1.2</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode>
            <bullet><friction>1.2</friction><friction2>1.2</friction2><rolling_friction>0.03</rolling_friction></bullet>
          </friction>
          <contact>
            <ode><kp>50000</kp><kd>1000</kd><min_depth>0.003</min_depth></ode>
          </contact>
        </surface>
      </collision>
      <visual name='visual_{escape(link_name)}'>
        <pose>0 0 0 1.5707 0 0</pose>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{width:.4f}</length></cylinder></geometry>
      </visual>
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

    def _reset_vehicle_pose(self, heading_rad: float, roll_rad: float, pitch_rad: float, z_pos: float) -> None:
        assert self._set_pose_client is not None
        assert self._node is not None

        req = SetEntityPose.Request()
        req.entity.name = self.runtime_cfg.model_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = float(z_pos)

        qx, qy, qz, qw = self._rpy_to_quat(roll_rad, pitch_rad, heading_rad)
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw

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
        return bool(np.max(jumps) > 1.5)

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

        commanded_linear = np.zeros(num_steps, dtype=np.float32)
        commanded_angular = np.zeros(num_steps, dtype=np.float32)
        ramp_sec = max(float(context.cmd_ramp_sec), 0.0)
        for t in range(num_steps):
            if ramp_sec > 1e-6:
                scale = float(np.clip((t * dt) / ramp_sec, 0.0, 1.0))
            else:
                scale = 1.0
            commanded_linear[t] = float(v_cmd * scale)
            commanded_angular[t] = float(w_cmd * scale)
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

        observed = forces > 1e-3
        latched = observed.copy()
        f_min = 0.05 * (context.vehicle.m * 9.81 / 4.0)
        lift_state = forces < f_min
        valid_mask = np.ones_like(lift_state, dtype=bool)
        clearance = np.clip((f_min - forces) / max(f_min, 1e-6) * context.vehicle.r_w, -0.03, 0.10).astype(np.float32)

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
            wheel_contact_observed=observed,
            wheel_contact_latched=latched,
            wheel_clearance_m=clearance,
            wheel_lift_valid_mask=valid_mask,
            wheel_lift_state=lift_state,
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
            log_dir=str(ros_cfg.get("log_dir", RosGzRuntimeConfig.log_dir)),
            headless=bool(ros_cfg.get("headless", RosGzRuntimeConfig.headless)),
        )
        return RosGzSimulationRunner(runtime_cfg=runtime_cfg)
    raise ValueError(f"unsupported backend: {backend}")
