# ml25d_dataset_generation

ROS2 package for 2.5D terrain local-action dataset generation.

## Features

- Config-driven terrain, vehicle, friction, and action sampling
- 6-channel feature patch construction with motion swept mask
- 7-label risk target extraction pipeline
- HDF5 batch serialization with manifest output
- Real `ros_gz` backend with dynamic 2.5D terrain mesh, parameterized vehicle SDF, Gazebo odometry, and contact sensors
- Surrogate backend kept only as a deterministic fallback / ablation source

## Workspace layout

- Package root: ml25d_ws/src/ml25d_dataset_generation
- Default output dir: ml25d_ws/data/generated

## Build

```bash
cd ml25d_ws
colcon build --packages-select ml25d_dataset_generation
source install/setup.bash
```

If ROS runtime reports missing Python modules on Ubuntu 24 (PEP 668), install them for the ROS interpreter:

```bash
/usr/bin/python3 -m pip install --user --break-system-packages h5py PyYAML numpy
```

## Run dataset generation

Always source ROS before running the real Gazebo backend:

```bash
source /opt/ros/jazzy/setup.bash
```

Small real-physics smoke test:

```bash
cd ml25d_ws
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/smoke_ros_gz.py \
  --num-samples 3 \
  --output-dir data/generated_ros_gz_smoke
```

Generate the real Gazebo dataset:

```bash
cd ml25d_ws
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/generate_dataset.py \
  --num-samples 300 \
  --seed 20260425 \
  --output-dir data/generated_ros_gz_v1 \
  --backend ros_gz
```

Notes for ros_gz backend:

- It auto-starts `ros2 run ros_gz_sim gzserver` and `ros2 run ros_gz_bridge parameter_bridge`.
- Default empty world is [worlds/ml25d_empty.sdf](worlds/ml25d_empty.sdf); each sample dynamically spawns a generated terrain mesh and a parameterized vehicle.
- Wheel lift and chassis-bottom labels use Gazebo contact sensors; wheel contacts use wrench magnitudes when available, not only a binary contact flag.
- Auto-start logs are written to `/tmp/ml25d_ros_gz_logs/gzserver.log` and `/tmp/ml25d_ros_gz_logs/bridge.log`.
- Detailed run notes are in [docs/ros_gz_dataset_generation.md](docs/ros_gz_dataset_generation.md).
- Preserve ROS Python paths when running from source: use `PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH`, not just `PYTHONPATH=src/...`.
- Fallback to surrogate is disabled by default so a requested `ros_gz` dataset cannot silently become synthetic.
- Samples with invalid Gazebo runtime behavior are retried and counted as `failed_attempts` in `dataset_manifest.json`.
- The surrogate backend remains available explicitly with `--backend surrogate` for ablations only.

## Validate generated files

```bash
ros2 run ml25d_dataset_generation validate_samples.py \
  --pattern "data/generated_ros_gz_v1/samples_batch_*.h5" \
  --report data/generated_ros_gz_v1/validation_report.json
```

## Stats report

```bash
ros2 run ml25d_dataset_generation stats_report.py \
  --pattern "data/generated_ros_gz_v1/samples_batch_*.h5" \
  --output data/generated_ros_gz_v1/stats_report.json
```

## Train CNN+MLP Risk Model With PSO

Install PyTorch in the project virtual environment first. For an RTX 4060 machine, CUDA wheels are recommended:

```bash
.venv/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
.venv/bin/python -m pip install scikit-learn
```

Run training from the workspace root:

```bash
cd ml25d_ws
PYTHONPATH=src/ml25d_dataset_generation/python ../.venv/bin/python \
  src/ml25d_dataset_generation/scripts/train_risk_model.py \
  --pattern "data/generated_ros_gz_v1/samples_batch_*.h5" \
  --output-dir data/training_runs/cnn_pso_rosgz_v1 \
  --device auto \
  --pso-particles 6 \
  --pso-iters 4 \
  --pso-epochs 5 \
  --final-epochs 30
```

Outputs:

- `best_model.pt`: PyTorch checkpoint containing model weights, normalization stats, fusion weights, and A* thresholds.
- `training_report.json`: PSO history, validation metrics, final test metrics, and proxy risk-constrained A* evaluation.

Calibrate the fail decision threshold after training. Use a higher calibration recall target for safety-critical planning, then report the held-out test metrics from the generated JSON:

```bash
PYTHONPATH=src/ml25d_dataset_generation/python ../.venv/bin/python \
  src/ml25d_dataset_generation/scripts/calibrate_risk_model.py \
  --checkpoint data/training_runs/cnn_pso_rosgz_v1/best_model.pt \
  --pattern "data/generated_ros_gz_v1/samples_batch_*.h5" \
  --output data/training_runs/cnn_pso_rosgz_v1/calibration_report_target095.json \
  --device auto \
  --target-recall 0.95
```

Training note: `p_bottom` is a continuous probability label and is supervised as a soft target, not thresholded into a binary class.

## Single sample debug

```bash
ros2 run ml25d_dataset_generation run_single_sample.py \
  --backend ros_gz \
  --seed 20260424 \
  --save-npz data/generated/debug_sample.npz
```

## Launch

```bash
ros2 launch ml25d_dataset_generation dataset_gen_pipeline.launch.py \
  backend:=ros_gz num_samples:=100 seed:=42
```

Visual debug in Gazebo (vehicle + bridge):

```bash
ros2 launch ml25d_dataset_generation ros_gz_vehicle_debug.launch.py
```

Then publish commands from another terminal:

```bash
ros2 topic pub /model/vehicle/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" -r 10
```

Failure-mode visual analysis for generated dataset:

```bash
ros2 run ml25d_dataset_generation analyze_failure_modes.py \
  --pattern "data/generated_ros_gz_v1/samples_batch_*.h5" \
  --output-dir data/generated_ros_gz_v1/analysis
```

## Notes

- `backend=mock` remains as a backward-compatible alias for older commands.
- `backend=surrogate` should not be used as the final physics source; it is only useful for quick debugging and ablation.
