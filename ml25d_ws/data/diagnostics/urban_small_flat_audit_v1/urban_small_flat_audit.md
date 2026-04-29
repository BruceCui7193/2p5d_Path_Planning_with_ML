## Flat Slice Report
| flat_total_count | flat_fail_count | flat_fail_rate |
| --- | --- | --- |
| 196 | 30 | 0.1531 |

### vehicle × action × mu_bin fail_rate
| vehicle_id | action_id | mu_bin | count | fail_rate |
| --- | --- | --- | --- | --- |
| mountain_large | a0 | [0.20,0.35) | 5 | 0.2000 |
| mountain_large | a0 | [0.35,0.50) | 6 | 0.0000 |
| mountain_large | a0 | [0.50,0.65) | 3 | 0.0000 |
| mountain_large | a0 | [0.65,0.80) | 5 | 0.0000 |
| mountain_large | a0 | [0.80,0.95) | 7 | 0.0000 |
| mountain_large | a1 | [0.35,0.50) | 9 | 0.1111 |
| mountain_large | a1 | [0.50,0.65) | 4 | 0.0000 |
| mountain_large | a1 | [0.65,0.80) | 7 | 0.0000 |
| mountain_large | a1 | [0.80,0.95) | 5 | 0.0000 |
| mountain_large | a2 | [0.20,0.35) | 4 | 0.0000 |
| mountain_large | a2 | [0.35,0.50) | 6 | 0.0000 |
| mountain_large | a2 | [0.50,0.65) | 2 | 0.0000 |
| mountain_large | a2 | [0.65,0.80) | 2 | 0.0000 |
| mountain_large | a2 | [0.80,0.95) | 1 | 0.0000 |
| standard_offroad | a0 | [0.20,0.35) | 3 | 0.0000 |
| standard_offroad | a0 | [0.35,0.50) | 2 | 0.0000 |
| standard_offroad | a0 | [0.50,0.65) | 6 | 0.0000 |
| standard_offroad | a0 | [0.65,0.80) | 6 | 0.0000 |
| standard_offroad | a0 | [0.80,0.95) | 2 | 0.5000 |
| standard_offroad | a1 | [0.20,0.35) | 5 | 0.0000 |
| standard_offroad | a1 | [0.35,0.50) | 3 | 0.0000 |
| standard_offroad | a1 | [0.50,0.65) | 4 | 0.0000 |
| standard_offroad | a1 | [0.65,0.80) | 5 | 0.0000 |
| standard_offroad | a1 | [0.80,0.95) | 3 | 0.0000 |
| standard_offroad | a2 | [0.20,0.35) | 4 | 0.0000 |
| standard_offroad | a2 | [0.35,0.50) | 5 | 0.2000 |
| standard_offroad | a2 | [0.50,0.65) | 6 | 0.0000 |
| standard_offroad | a2 | [0.65,0.80) | 2 | 0.0000 |
| standard_offroad | a2 | [0.80,0.95) | 4 | 0.0000 |
| urban_small | a0 | [0.20,0.35) | 5 | 0.4000 |
| urban_small | a0 | [0.35,0.50) | 4 | 0.5000 |
| urban_small | a0 | [0.50,0.65) | 6 | 0.1667 |
| urban_small | a0 | [0.65,0.80) | 5 | 0.6000 |
| urban_small | a0 | [0.80,0.95) | 5 | 0.2000 |
| urban_small | a1 | [0.20,0.35) | 6 | 0.3333 |
| urban_small | a1 | [0.35,0.50) | 8 | 0.5000 |
| urban_small | a1 | [0.50,0.65) | 3 | 0.3333 |
| urban_small | a1 | [0.65,0.80) | 4 | 0.2500 |
| urban_small | a2 | [0.20,0.35) | 3 | 0.6667 |
| urban_small | a2 | [0.35,0.50) | 10 | 0.4000 |
| urban_small | a2 | [0.50,0.65) | 5 | 0.4000 |
| urban_small | a2 | [0.65,0.80) | 3 | 0.0000 |
| urban_small | a2 | [0.80,0.95) | 3 | 0.3333 |

### vehicle × fail_reason
| vehicle_id | fail_reason | count | ratio_in_vehicle_fail_reasons |
| --- | --- | --- | --- |
| mountain_large | lift | 1 | 0.3333 |
| mountain_large | pitch | 2 | 0.6667 |
| standard_offroad | lift | 2 | 0.2857 |
| standard_offroad | pitch | 2 | 0.2857 |
| standard_offroad | roll | 2 | 0.2857 |
| standard_offroad | stuck | 1 | 0.1429 |
| urban_small | lift | 12 | 0.1765 |
| urban_small | pitch | 26 | 0.3824 |
| urban_small | roll | 14 | 0.2059 |
| urban_small | slip | 4 | 0.0588 |
| urban_small | stuck | 12 | 0.1765 |

### urban_small parameter fail/pass compare
| param | fail_mean | fail_p50 | pass_mean | pass_p50 | delta_fail_minus_pass |
| --- | --- | --- | --- | --- | --- |
| wheel_radius | 0.0620 | 0.0601 | 0.0624 | 0.0605 | -0.0004 |
| track_width | 0.3101 | 0.3000 | 0.3158 | 0.3055 | -0.0057 |
| wheelbase | 0.3340 | 0.3203 | 0.3330 | 0.3225 | 0.0010 |
| clearance | 0.0416 | 0.0402 | 0.0415 | 0.0400 | 0.0001 |
| z_com | 0.1637 | 0.1607 | 0.1638 | 0.1601 | -0.0001 |
| roll_limit | 19.3891 | 19.6431 | 19.8773 | 19.5729 | -0.4882 |
| pitch_limit | 22.5522 | 22.2386 | 22.3506 | 21.9802 | 0.2016 |
| drive_force | 84.7398 | 84.6409 | 80.8311 | 82.0601 | 3.9087 |

### urban_small fail action breakdown
| total_fail | a0_fail_count | a1_fail_count | a2_fail_count | a1_a2_fail_count | a0_ratio | a1_a2_ratio | dominant |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 26 | 9 | 8 | 9 | 17 | 0.3462 | 0.6538 | a1_a2 |

## Replay Reproducibility
| selected_fail_cases | selected_pass_cases | repeats_per_case | fail_case_mean_replay_fail_rate | pass_case_mean_replay_fail_rate |
| --- | --- | --- | --- | --- |
| 10 | 5 | 5 | 0.0000 | 0.0000 |

### per-case replay summary
| sample_id | case_type | action_id | original_fail | ok_repeats | runtime_fail_repeats | replay_fail_rate | replay_pass_rate | original_fail_reasons | replay_fail_reason_distribution |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 83 | fail | a0 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['roll', 'pitch', 'lift', 'stuck'] | {} |
| 329 | fail | a1 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['roll', 'pitch', 'stuck'] | {} |
| 51 | fail | a2 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['pitch'] | {} |
| 94 | fail | a0 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['roll', 'pitch', 'slip', 'lift'] | {} |
| 454 | fail | a1 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['pitch'] | {} |
| 222 | fail | a2 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['roll', 'pitch', 'lift', 'stuck'] | {} |
| 230 | fail | a0 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['pitch'] | {} |
| 664 | fail | a1 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['roll', 'pitch', 'stuck'] | {} |
| 252 | fail | a2 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['pitch', 'lift'] | {} |
| 332 | fail | a0 | 1.0000 | 5 | 0 | 0.0000 | 1.0000 | ['pitch', 'stuck'] | {} |
| 104 | pass | a0 | 0.0000 | 5 | 0 | 0.0000 | 1.0000 | [] | {} |
| 15 | pass | a1 | 0.0000 | 5 | 0 | 0.0000 | 1.0000 | [] | {} |
| 21 | pass | a2 | 0.0000 | 5 | 0 | 0.0000 | 1.0000 | [] | {} |
| 131 | pass | a0 | 0.0000 | 5 | 0 | 0.0000 | 1.0000 | [] | {} |
| 35 | pass | a1 | 0.0000 | 5 | 0 | 0.0000 | 1.0000 | [] | {} |
