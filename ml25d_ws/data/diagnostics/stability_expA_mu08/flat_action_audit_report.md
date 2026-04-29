## Flat Action Audit Summary
| groups_total | groups_completed | valid_samples | invalid_attempts | flat_fail_rate | accept_flat_fail_lt_5pct | accept_a3a4_no_progress_gt3 | accept_a3a4_stuck_semantics | recommend_remove_a3a4 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 9 | 9 | 90 | 2 | 0.0000 | True | True | True | False |

### Notes
- flat 总 fail_rate 已低于 5%。
- a3/a4 未出现显著非 slip 失败放大，可保留。

## Per Group Metrics
| vehicle_type | action_id | friction_class | target_count | valid_count | fail_rate | stuck_fail_rate | q_roll_mean | q_pitch_mean | q_lift_mean | p_bottom_mean | translation_progress_mean | angular_progress_mean | translation_drift_mean | fail_reason_distribution |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| city_small | a0 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0050 | 0.0060 | 0.0000 | 0.0000 | 0.9804 | NaN | 0.2941 | {} |
| city_small | a1 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0063 | 0.0074 | 0.0000 | 0.0000 | 0.9625 | NaN | 0.2887 | {} |
| city_small | a2 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0065 | 0.0059 | 0.0000 | 0.0000 | 0.9850 | NaN | 0.2955 | {} |
| mountain_large | a0 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0023 | 0.0011 | 0.0000 | 0.0000 | 0.9571 | NaN | 0.2871 | {} |
| mountain_large | a1 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0027 | 0.0021 | 0.0000 | 0.0000 | 1.0555 | NaN | 0.3166 | {} |
| mountain_large | a2 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0031 | 0.0014 | 0.0000 | 0.0000 | 1.1209 | NaN | 0.3363 | {} |
| offroad_medium | a0 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0011 | 0.0023 | 0.0000 | 0.0000 | 0.9888 | NaN | 0.2966 | {} |
| offroad_medium | a1 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0016 | 0.0043 | 0.0000 | 0.0000 | 0.9523 | NaN | 0.2857 | {} |
| offroad_medium | a2 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0014 | 0.0019 | 0.0000 | 0.0000 | 1.0528 | NaN | 0.3158 | {} |

json_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/stability_expA_mu08/flat_action_audit_report.json`
samples_jsonl: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/stability_expA_mu08/flat_action_samples.jsonl`
