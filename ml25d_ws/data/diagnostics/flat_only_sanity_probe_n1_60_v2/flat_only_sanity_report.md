## Flat-only Sanity Report

| workers | target_samples | accepted | invalid | invalid_rate | fail_rate |
| --- | --- | --- | --- | --- | --- |
| 1 | 60 | 60 | 0 | 0.0000 | 0.0833 |

| metric | mean | p95 | max |
| --- | --- | --- | --- |
| q_roll | 0.005803 | 0.024734 | 0.047804 |
| q_pitch | 0.101386 | 1.000000 | 1.000000 |
| q_slip | 0.166848 | 0.624799 | 0.648045 |
| q_lift | 0.018403 | 0.105250 | 0.370000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.033333 | 0.000000 | 1.000000 |

- q_slip_vs_abs_cos_heading_corr: 0.147990
- odom_pose_forward_mae mean/p95/max: 0.167084 / 0.481886 / 0.743247
- slip_fail_rate: 0.000000
- bottom_fail_rate: 0.000000
- stuck_fail_rate: 0.033333

### Fail Reasons
| reason | count |
| --- | --- |
| pitch | 5 |
| lift | 2 |
| stuck | 2 |

### Coverage
- vehicle_counts: {"urban_small": 20, "standard_offroad": 21, "mountain_large": 19}
- action_counts: {"a1": 20, "a2": 20, "a0": 20}
- motion_model_counts: {"ackermann": 35, "skid": 25}
