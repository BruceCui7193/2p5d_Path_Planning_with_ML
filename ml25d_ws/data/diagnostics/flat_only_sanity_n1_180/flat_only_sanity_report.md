## Flat-only Sanity Report

| workers | target_samples | accepted | invalid | invalid_rate | fail_rate |
| --- | --- | --- | --- | --- | --- |
| 1 | 180 | 180 | 0 | 0.0000 | 0.0056 |

| metric | mean | p95 | max |
| --- | --- | --- | --- |
| q_roll | 0.000013 | 0.000046 | 0.000089 |
| q_pitch | 0.000012 | 0.000063 | 0.000153 |
| q_slip | 0.233677 | 0.646696 | 0.813119 |
| q_lift | 0.000000 | 0.000000 | 0.000000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.000000 | 0.000000 | 0.000000 |

- q_slip_vs_abs_cos_heading_corr: 0.162991
- odom_pose_forward_mae mean/p95/max: 0.196407 / 0.556962 / 0.675439
- slip_fail_rate: 0.005556
- bottom_fail_rate: 0.000000
- stuck_fail_rate: 0.000000

### Fail Reasons
| reason | count |
| --- | --- |
| slip | 1 |

### Coverage
- vehicle_counts: {"urban_small": 60, "standard_offroad": 60, "mountain_large": 60}
- action_counts: {"a1": 60, "a2": 60, "a0": 60}
- motion_model_counts: {"ackermann": 95, "skid": 85}
