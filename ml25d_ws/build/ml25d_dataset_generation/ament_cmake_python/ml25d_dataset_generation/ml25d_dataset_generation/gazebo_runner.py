from __future__ import annotations

import atexit
from dataclasses import dataclass
import math
import shutil
import subprocess
import threading
import time
from typing import Literal

import numpy as np

from .common_types import ActionPrimitive, SimulationTrajectory, VehicleParams

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
except ImportError:
    rclpy = None
    Twist = None
    Odometry = None
    Entity = None
    SetEntityPose = None


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
    world_sdf_file: str = "/opt/ros/jazzy/share/ros_gz_sim_demos/worlds/vehicle.sdf"
    world_name: str = "demo"
    model_name: str = "vehicle"
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


class RosGzSimulationRunner(SimulationRunner):
    def __init__(self, runtime_cfg: RosGzRuntimeConfig | None = None, resolution_m: float = 0.1) -> None:
        self.runtime_cfg = runtime_cfg or RosGzRuntimeConfig()
        self.resolution_m = resolution_m
        self._fallback = MockSimulationRunner(resolution_m=resolution_m)

        self._node = None
        self._cmd_pub = None
        self._set_pose_client = None
        self._odom_buffer = _OdomBuffer()

        self._gzserver_proc: subprocess.Popen | None = None
        self._bridge_proc: subprocess.Popen | None = None
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

        if rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _ensure_runtime(self) -> None:
        if self._runtime_ready:
            if self._gzserver_proc is not None and self._gzserver_proc.poll() is not None:
                raise RuntimeError("gzserver process exited unexpectedly")
            if self._bridge_proc is not None and self._bridge_proc.poll() is not None:
                raise RuntimeError("ros_gz_bridge process exited unexpectedly")
            return

        if rclpy is None or Twist is None or Odometry is None or SetEntityPose is None or Entity is None:
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

        if self.runtime_cfg.auto_start_processes:
            self._start_processes()

        deadline = time.time() + self.runtime_cfg.startup_timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._set_pose_client.wait_for_service(timeout_sec=0.1):
                self._runtime_ready = True
                return

        raise RuntimeError("timed out waiting for /world/<name>/set_pose bridge service")

    def _start_processes(self) -> None:
        if self._gzserver_proc is None or self._gzserver_proc.poll() is not None:
            self._gzserver_proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "ros_gz_sim",
                    "gzserver",
                    "--ros-args",
                    "-p",
                    f"world_sdf_file:={self.runtime_cfg.world_sdf_file}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        if self._bridge_proc is None or self._bridge_proc.poll() is not None:
            self._bridge_proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    f"/model/{self.runtime_cfg.model_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    f"/model/{self.runtime_cfg.model_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                    f"/world/{self.runtime_cfg.world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    def _run_once(self, context: SimulationContext, rng: np.random.Generator) -> SimulationTrajectory:
        assert self._node is not None
        assert self._cmd_pub is not None
        assert self._set_pose_client is not None

        self._odom_buffer.clear()
        self._reset_vehicle_pose(context.heading_rad, context.vehicle)
        self._publish_cmd(0.0, 0.0)
        self._wait_for_reset_pose(context.heading_rad)
        self._odom_buffer.clear()

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
        for idx in range(num_steps):
            step_start = time.monotonic()
            self._publish_cmd(v_cmd, w_cmd)
            rclpy.spin_once(self._node, timeout_sec=dt)

            elapsed = time.monotonic() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

            # Drain one more callback cycle after pacing to capture freshest odom.
            rclpy.spin_once(self._node, timeout_sec=0.0)

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

        roll, pitch = self._inject_terrain_attitude_bias(roll, pitch, context, rng)
        wheel_contact_forces, chassis_contacts = self._estimate_contacts(context, roll, pitch, rng)

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
        req.pose.position.z = max(0.2, vehicle.r_w + 0.05)

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

    def _inject_terrain_attitude_bias(
        self,
        roll: np.ndarray,
        pitch: np.ndarray,
        context: SimulationContext,
        rng: np.random.Generator,
    ) -> tuple[np.ndarray, np.ndarray]:
        center = context.heightmap.shape[0] // 2
        grad_u, grad_v = np.gradient(context.heightmap, self.resolution_m, edge_order=1)

        slope_forward = float(
            grad_u[center, center] * np.cos(context.heading_rad) + grad_v[center, center] * np.sin(context.heading_rad)
        )
        slope_lateral = float(
            -grad_u[center, center] * np.sin(context.heading_rad) + grad_v[center, center] * np.cos(context.heading_rad)
        )

        roughness = float(np.std(context.heightmap[max(0, center - 2) : center + 3, max(0, center - 2) : center + 3]))

        roll_bias = np.arctan(slope_lateral) * (0.8 + 0.6 * roughness)
        pitch_bias = np.arctan(slope_forward) * (0.8 + 0.6 * roughness)

        out_roll = np.copy(roll)
        out_pitch = np.copy(pitch)
        for idx in range(out_roll.shape[0]):
            phase = 2.0 * np.pi * idx / max(out_roll.shape[0] - 1, 1)
            out_roll[idx] += float(roll_bias * (0.8 + 0.2 * np.sin(phase)) + rng.normal(0.0, 0.003 + roughness * 0.03))
            out_pitch[idx] += float(
                pitch_bias * (0.8 + 0.2 * np.cos(phase)) + rng.normal(0.0, 0.003 + roughness * 0.03)
            )

        return out_roll.astype(np.float32), out_pitch.astype(np.float32)

    def _estimate_contacts(
        self,
        context: SimulationContext,
        roll: np.ndarray,
        pitch: np.ndarray,
        rng: np.random.Generator,
    ) -> tuple[np.ndarray, np.ndarray]:
        n = roll.shape[0]
        center = context.heightmap.shape[0] // 2
        local = context.heightmap[max(0, center - 2) : center + 3, max(0, center - 2) : center + 3]
        roughness = float(np.std(local))
        span = float(np.max(local) - np.min(local))

        base_load = context.vehicle.m * 9.81 / 4.0
        forces = np.zeros((n, 4), dtype=np.float32)

        lift_prob = float(np.clip(roughness * 10.0 + span / max(context.vehicle.r_w, 1e-3), 0.0, 0.85))
        for idx in range(n):
            tr_roll = base_load * np.tan(float(roll[idx])) * 0.6
            tr_pitch = base_load * np.tan(float(pitch[idx])) * 0.4
            row = np.array(
                [
                    base_load - tr_roll - tr_pitch,
                    base_load + tr_roll - tr_pitch,
                    base_load - tr_roll + tr_pitch,
                    base_load + tr_roll + tr_pitch,
                ],
                dtype=np.float32,
            )
            if rng.random() < lift_prob * 0.1:
                row[int(rng.integers(0, 4))] *= 0.03
            forces[idx] = np.clip(row, 0.0, None)

        # Use detrended terrain residual for bottom-contact estimate so global
        # slope does not dominate as a false "bottoming" trigger.
        grid = context.heightmap
        h, w = grid.shape
        xs, ys = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
        A = np.stack([xs.reshape(-1), ys.reshape(-1), np.ones(h * w)], axis=1)
        b = grid.reshape(-1)
        coeff, *_ = np.linalg.lstsq(A, b, rcond=None)
        plane = (coeff[0] * xs + coeff[1] * ys + coeff[2]).astype(np.float32)
        residual = grid - plane

        obs_amp = float(np.percentile(np.abs(residual), 95))
        clearance = max(context.vehicle.c_g, 1e-3)
        ratio = obs_amp / clearance
        contact_prob = float(np.clip((ratio - 0.9) / 2.5, 0.0, 0.35))
        contact_prob += float(np.clip(roughness / clearance * 0.03, 0.0, 0.08))
        contact_prob = float(np.clip(contact_prob, 0.0, 0.4))

        chassis_contacts = _deterministic_contact_series(n, contact_prob)
        return forces, chassis_contacts


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
