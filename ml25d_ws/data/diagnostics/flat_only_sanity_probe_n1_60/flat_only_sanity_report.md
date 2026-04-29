## Flat-only Sanity Report

| workers | target_samples | accepted | invalid | invalid_rate | fail_rate |
| --- | --- | --- | --- | --- | --- |
| 1 | 60 | 60 | 0 | 0.0000 | 0.1000 |

| metric | mean | p95 | max |
| --- | --- | --- | --- |
| q_roll | 0.057418 | 0.095565 | 1.000000 |
| q_pitch | 0.144588 | 1.000000 | 1.000000 |
| q_slip | 0.281851 | 0.637607 | 1.000000 |
| q_lift | 0.028514 | 0.213667 | 0.405000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.033333 | 0.000000 | 1.000000 |

- q_slip_vs_abs_cos_heading_corr: -0.013651
- odom_pose_forward_mae mean/p95/max: 1.351904 / 2.995797 / 3.103161
- slip_fail_rate: 0.033333
- bottom_fail_rate: 0.000000
- stuck_fail_rate: 0.033333

### Fail Reasons
| reason | count |
| --- | --- |
| pitch | 6 |
| roll | 3 |
| lift | 2 |
| slip | 2 |
| stuck | 2 |

### Coverage
- vehicle_counts: {"urban_small": 20, "standard_offroad": 21, "mountain_large": 19}
- action_counts: {"a1": 20, "a2": 20, "a0": 20}
- motion_model_counts: {"ackermann": 35, "skid": 25}
