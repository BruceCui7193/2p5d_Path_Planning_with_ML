## Flat-only Sanity Report

| workers | target_samples | accepted | invalid | invalid_rate | fail_rate |
| --- | --- | --- | --- | --- | --- |
| 1 | 60 | 60 | 0 | 0.0000 | 0.0000 |

| metric | mean | p95 | max |
| --- | --- | --- | --- |
| q_roll | 0.000015 | 0.000054 | 0.000089 |
| q_pitch | 0.000013 | 0.000065 | 0.000109 |
| q_slip | 0.210629 | 0.645050 | 0.647942 |
| q_lift | 0.000000 | 0.000000 | 0.000000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.000000 | 0.000000 | 0.000000 |

- q_slip_vs_abs_cos_heading_corr: 0.099972
- odom_pose_forward_mae mean/p95/max: 0.191674 / 0.784108 / 0.867115
- slip_fail_rate: 0.000000
- bottom_fail_rate: 0.000000
- stuck_fail_rate: 0.000000

### Fail Reasons
| reason | count |
| --- | --- |
| (none) | 0 |

### Coverage
- vehicle_counts: {"urban_small": 20, "standard_offroad": 21, "mountain_large": 19}
- action_counts: {"a1": 20, "a2": 20, "a0": 20}
- motion_model_counts: {"ackermann": 30, "skid": 30}
