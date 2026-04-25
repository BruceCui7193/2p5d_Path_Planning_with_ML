# ros_gz Dataset Generation

The final dataset path should use the real `ros_gz` backend, not the surrogate backend.

## What The Backend Does

- Starts `ros_gz_sim` with `worlds/ml25d_empty.sdf`.
- Dynamically spawns a generated 2.5D terrain mesh for each sample.
- Dynamically spawns a parameterized diff-drive vehicle SDF for each sample.
- Executes the local action in Gazebo physics.
- Reads roll, pitch, position, yaw, and speed from Gazebo odometry.
- Reads wheel/chassis contacts and contact wrench magnitudes from Gazebo contact sensors.
- Retries failed samples and records `failed_attempts` in `dataset_manifest.json`.
- Writes auto-started `gzserver` and `ros_gz_bridge` logs to `/tmp/ml25d_ros_gz_logs`.

## Run Smoke Test

Run this from a normal terminal, not from a restricted sandbox:

```bash
cd ~/文档/Machine_Learning_25D/ml25d_ws
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR=/tmp/ml25d_ros_logs
export GZ_LOG_PATH=/tmp/ml25d_gz_logs
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/smoke_ros_gz.py \
  --num-samples 3 \
  --output-dir data/generated_ros_gz_smoke
```

Validate:

```bash
PYTHONPATH=src/ml25d_dataset_generation/python python3 \
  src/ml25d_dataset_generation/scripts/validate_samples.py \
  --pattern "data/generated_ros_gz_smoke/samples_batch_*.h5" \
  --report data/generated_ros_gz_smoke/validation_report.json
```

## Generate Candidate Dataset

Start with a small physical dataset before scaling:

```bash
cd ~/文档/Machine_Learning_25D/ml25d_ws
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR=/tmp/ml25d_ros_logs
export GZ_LOG_PATH=/tmp/ml25d_gz_logs
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/generate_dataset.py \
  --backend ros_gz \
  --num-samples 100 \
  --seed 20260425 \
  --output-dir data/generated_ros_gz_v1
```

If the smoke set has reasonable label diversity and low `failed_attempts`, increase to 300-1000 samples.

If startup fails, inspect:

```bash
tail -200 /tmp/ml25d_ros_gz_logs/gzserver.log
tail -200 /tmp/ml25d_ros_gz_logs/bridge.log
```

## Important

Use `PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH`. Do not overwrite ROS's existing `PYTHONPATH`, otherwise `rclpy` and `ros_gz_interfaces` will not import.

The old `data/generated_hq_v1` dataset was produced by the surrogate backend. It is useful only as a warm-start / ablation dataset and should not be presented as Gazebo physics data.
