# ml25d_dataset_generation

ROS2 package for 2.5D terrain local-action dataset generation.

## Features

- Config-driven terrain, vehicle, friction, and action sampling
- 6-channel feature patch construction with motion swept mask
- 7-label risk target extraction pipeline
- HDF5 batch serialization with manifest output
- Surrogate backend for deterministic 2.5D terrain-risk generation
- RosGz backend scaffold with strict failure handling

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

```bash
ros2 run ml25d_dataset_generation generate_dataset.py \
  --num-samples 3000 \
  --seed 42 \
  --backend surrogate
```

For real Gazebo runtime (ros_gz):

```bash
ros2 run ml25d_dataset_generation generate_dataset.py \\
  --num-samples 100 \\
  --seed 42 \\
  --backend ros_gz
```

Notes for ros_gz backend:

- It auto-starts `ros2 run ros_gz_sim gzserver` and `ros2 run ros_gz_bridge parameter_bridge`.
- Default world and model are configured in [config/dataset_config.yaml](config/dataset_config.yaml).
- Fallback to surrogate is disabled by default so a requested `ros_gz` dataset cannot silently become synthetic.
- The bundled ROS demo vehicle is suitable for connectivity smoke tests, not final terrain-risk data generation.

## Validate generated files

```bash
ros2 run ml25d_dataset_generation validate_samples.py \
  --pattern "data/generated/samples_batch_*.h5" \
  --report data/generated/validation_report.json
```

## Stats report

```bash
ros2 run ml25d_dataset_generation stats_report.py \
  --pattern "data/generated/samples_batch_*.h5" \
  --output data/generated/stats_report.json
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
  --pattern "data/generated_hq_v1/samples_batch_*.h5" \
  --output-dir data/training_runs/cnn_pso_v1 \
  --device auto \
  --pso-particles 6 \
  --pso-iters 4 \
  --pso-epochs 5 \
  --final-epochs 30
```

Outputs:

- `best_model.pt`: PyTorch checkpoint containing model weights, normalization stats, fusion weights, and A* thresholds.
- `training_report.json`: PSO history, validation metrics, final test metrics, and proxy risk-constrained A* evaluation.

## Single sample debug

```bash
ros2 run ml25d_dataset_generation run_single_sample.py \
  --seed 20260424 \
  --save-npz data/generated/debug_sample.npz
```

## Launch

```bash
ros2 launch ml25d_dataset_generation dataset_gen_pipeline.launch.py \
  backend:=surrogate num_samples:=100 seed:=42
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
  --pattern "data/generated_hq_v1/samples_batch_*.h5" \
  --output-dir data/generated_hq_v1/analysis
```

## Notes

- Use `backend=surrogate` for the current trainable dataset.
- `backend=mock` remains as a backward-compatible alias for older commands.
- RosGz runtime binding should be treated as a smoke-test backend until a project-specific terrain world and stable odometry source are added.
