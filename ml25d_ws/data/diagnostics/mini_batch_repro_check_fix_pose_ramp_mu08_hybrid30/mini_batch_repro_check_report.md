## Mini Batch Repro Check

### stage1_n1
| first_run_fail_rate | replay_fail_rate | class_mismatch_rate | metric_mismatch_rate | terrain_H_std | terrain_H_range |
| --- | --- | --- | --- | --- | --- |
| 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000e+00 | 0.000000e+00 |

| ok_rows | runtime_failure_rows | sample_start_time_min | sample_start_time_max | message_time_min | message_time_max |
| --- | --- | --- | --- | --- | --- |
| 30 | 0 | 0.144000 | 130.608000 | 0.144000 | 132.592000 |

| metric | diff_mean | diff_p50 | diff_p95 | diff_max |
| --- | --- | --- | --- | --- |
| q_roll | 0.001537 | 0.001076 | 0.004044 | 0.004638 |
| q_pitch | 0.001776 | 0.001405 | 0.004083 | 0.005623 |
| q_slip | 0.049267 | 0.018212 | 0.164613 | 0.167612 |
| q_lift | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.000000 | 0.000000 | 0.000000 | 0.000000 |

No mismatches.

### stage2_n4
| first_run_fail_rate | replay_fail_rate | class_mismatch_rate | metric_mismatch_rate | terrain_H_std | terrain_H_range |
| --- | --- | --- | --- | --- | --- |
| 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000e+00 | 0.000000e+00 |

| ok_rows | runtime_failure_rows | sample_start_time_min | sample_start_time_max | message_time_min | message_time_max |
| --- | --- | --- | --- | --- | --- |
| 30 | 0 | 0.144000 | 33.388000 | 0.144000 | 35.372000 |

| metric | diff_mean | diff_p50 | diff_p95 | diff_max |
| --- | --- | --- | --- | --- |
| q_roll | 0.001228 | 0.000912 | 0.003253 | 0.003341 |
| q_pitch | 0.001581 | 0.001448 | 0.004273 | 0.004628 |
| q_slip | 0.033431 | 0.029473 | 0.075772 | 0.090742 |
| q_lift | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.000000 | 0.000000 | 0.000000 | 0.000000 |

No mismatches.
