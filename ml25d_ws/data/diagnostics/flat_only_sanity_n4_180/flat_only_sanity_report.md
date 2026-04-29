## Flat-only Sanity Report

| workers | target_samples | accepted | invalid | invalid_rate | fail_rate |
| --- | --- | --- | --- | --- | --- |
| 4 | 180 | 180 | 0 | 0.0000 | 0.0000 |

| metric | mean | p95 | max |
| --- | --- | --- | --- |
| q_roll | 0.000017 | 0.000052 | 0.000367 |
| q_pitch | 0.000017 | 0.000096 | 0.000188 |
| q_slip | 0.251764 | 0.646650 | 0.647982 |
| q_lift | 0.000000 | 0.000000 | 0.000000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.000000 | 0.000000 | 0.000000 |

- q_slip_vs_abs_cos_heading_corr: 0.112338
- odom_pose_forward_mae mean/p95/max: 0.216606 / 0.613929 / 1.018569
- slip_fail_rate: 0.000000
- bottom_fail_rate: 0.000000
- stuck_fail_rate: 0.000000

### Fail Reasons
| reason | count |
| --- | --- |
| (none) | 0 |

### Coverage
- vehicle_counts: {"urban_small": 60, "standard_offroad": 60, "mountain_large": 60}
- action_counts: {"a1": 60, "a2": 60, "a0": 60}
- motion_model_counts: {"ackermann": 83, "skid": 97}
