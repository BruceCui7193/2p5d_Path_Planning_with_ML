# generated_hq_v1 Quality Summary

Generated at: 2026-04-24 23:58 Asia/Shanghai

## Dataset

- Backend: `surrogate`
- Samples: 3000
- Seed: `20260424`
- Schema version: `0.2.0`
- Files: `samples_batch_0001.h5` to `samples_batch_0015.h5`
- Manifest: `dataset_manifest.json`
- Validation report: `validation_report.json`
- Statistics report: `stats_report.json`
- Failure analysis: `analysis/failure_analysis_summary.json`

## Class Balance

- Safe: 900 / 3000 = 0.30
- Fail: 900 / 3000 = 0.30
- Critical: 1200 / 3000 = 0.40

## Coverage

- Terrain classes: 10 / 10 covered
- Vehicle types: `urban_small`, `standard_offroad`, `mountain_large`
- Motion models: `skid` 1523, `ackermann` 1477
- Action primitives: `a0` 730, `a1` 822, `a2` 770, `a3` 340, `a4` 338

Ackermann samples intentionally exclude in-place rotation primitives because that action is not physically feasible for Ackermann kinematics.

## Validation

`validate_samples.py` passed on all 15 HDF5 batches:

- Missing keys: none
- Shape errors: none
- NaN/Inf: none
- Label range violations: none

## Known RosGz Status

The previous `ros_gz` data quality issue was caused mainly by code/runtime problems rather than terrain thresholds:

- The runner compared `cmd_vel` with an unreliable odometry component, causing false slip labels.
- Pose reset did not wait for fresh odometry, so old Gazebo messages could enter the sample window.
- `fallback_to_mock_on_error` was enabled, allowing requested `ros_gz` runs to silently become synthetic.
- The bundled ROS demo vehicle/world does not provide a project-specific 2.5D terrain interaction source and produced unstable odometry in smoke tests.

The current trainable dataset therefore uses the deterministic 2.5D surrogate backend. The `ros_gz` backend is kept for smoke tests only until a custom terrain world and stable odometry/contact bridge are implemented.
