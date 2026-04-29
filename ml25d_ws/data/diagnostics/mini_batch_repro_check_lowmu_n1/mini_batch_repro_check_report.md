## Mini Batch Repro Check

### stage1_n1
| first_run_fail_rate | replay_fail_rate | class_mismatch_rate | metric_mismatch_rate | terrain_H_std | terrain_H_range |
| --- | --- | --- | --- | --- | --- |
| 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000e+00 | 0.000000e+00 |

| ok_rows | runtime_failure_rows | sample_start_time_min | sample_start_time_max | message_time_min | message_time_max |
| --- | --- | --- | --- | --- | --- |
| 30 | 0 | 0.144000 | 130.892000 | 0.144000 | 132.876000 |

| metric | diff_mean | diff_p50 | diff_p95 | diff_max |
| --- | --- | --- | --- | --- |
| q_roll | 0.002081 | 0.001572 | 0.005134 | 0.005596 |
| q_pitch | 0.001011 | 0.000554 | 0.003826 | 0.004765 |
| q_slip | 0.030635 | 0.018215 | 0.080427 | 0.108647 |
| q_lift | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| p_bottom | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| p_stuck | 0.000000 | 0.000000 | 0.000000 | 0.000000 |

No mismatches.
