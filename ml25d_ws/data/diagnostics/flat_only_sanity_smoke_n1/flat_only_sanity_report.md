## Flat-only Sanity Report

| workers | target_samples | accepted | invalid | invalid_rate | fail_rate |
| --- | --- | --- | --- | --- | --- |
| 1 | 24 | 24 | 0 | 0.0000 | 0.2917 |

| metric | mean | p95 | max |
| --- | --- | --- | --- |
| q_roll | 0.135809 | 1.000000 | 1.000000 |
| q_pitch | 0.323975 | 1.000000 | 1.000000 |
| q_slip | 0.212601 | 0.558382 | 0.631217 |
| q_lift | 0.047917 | 0.230417 | 0.275000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.083333 | 0.850000 | 1.000000 |

- q_slip_vs_abs_cos_heading_corr: -0.124550
- odom_pose_forward_mae mean/p95/max: 1.460840 / 2.841221 / 3.204504
- slip_fail_rate: 0.000000
- bottom_fail_rate: 0.000000
- stuck_fail_rate: 0.083333

### Fail Reasons
| reason | count |
| --- | --- |
| pitch | 7 |
| roll | 3 |
| stuck | 2 |
| lift | 1 |

### Coverage
- vehicle_counts: {"urban_small": 8, "standard_offroad": 9, "mountain_large": 7}
- action_counts: {"a1": 8, "a2": 8, "a0": 8}
- motion_model_counts: {"ackermann": 15, "skid": 9}
