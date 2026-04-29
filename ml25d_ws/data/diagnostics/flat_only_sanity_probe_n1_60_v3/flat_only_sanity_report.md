## Flat-only Sanity Report

| workers | target_samples | accepted | invalid | invalid_rate | fail_rate |
| --- | --- | --- | --- | --- | --- |
| 1 | 60 | 60 | 0 | 0.0000 | 0.1333 |

| metric | mean | p95 | max |
| --- | --- | --- | --- |
| q_roll | 0.018067 | 0.025589 | 0.684960 |
| q_pitch | 0.147657 | 1.000000 | 1.000000 |
| q_slip | 0.246145 | 0.977847 | 1.000000 |
| q_lift | 0.054861 | 0.426917 | 0.680000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.116667 | 1.000000 | 1.000000 |

- q_slip_vs_abs_cos_heading_corr: 0.014549
- odom_pose_forward_mae mean/p95/max: 0.159783 / 0.347290 / 0.589340
- slip_fail_rate: 0.066667
- bottom_fail_rate: 0.000000
- stuck_fail_rate: 0.116667

### Fail Reasons
| reason | count |
| --- | --- |
| pitch | 8 |
| lift | 7 |
| stuck | 7 |
| slip | 4 |

### Coverage
- vehicle_counts: {"urban_small": 20, "standard_offroad": 21, "mountain_large": 19}
- action_counts: {"a1": 20, "a2": 20, "a0": 20}
- motion_model_counts: {"skid": 27, "ackermann": 33}
