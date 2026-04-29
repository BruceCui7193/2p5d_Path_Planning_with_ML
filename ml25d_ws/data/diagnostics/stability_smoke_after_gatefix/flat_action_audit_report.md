## Flat Action Audit Summary
| groups_total | groups_completed | valid_samples | invalid_attempts | flat_fail_rate | accept_flat_fail_lt_5pct | accept_a3a4_no_progress_gt3 | accept_a3a4_stuck_semantics | recommend_remove_a3a4 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 1 | 10 | 0 | 0.0000 | True | True | True | False |

### Notes
- flat 总 fail_rate 已低于 5%。
- a3/a4 未出现显著非 slip 失败放大，可保留。

## Per Group Metrics
| vehicle_type | action_id | friction_class | target_count | valid_count | fail_rate | stuck_fail_rate | q_roll_mean | q_pitch_mean | q_lift_mean | p_bottom_mean | translation_progress_mean | angular_progress_mean | translation_drift_mean | fail_reason_distribution |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| city_small | a0 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0055 | 0.0068 | 0.0000 | 0.0000 | 0.9820 | NaN | 0.2946 | {} |

json_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/stability_smoke_after_gatefix/flat_action_audit_report.json`
samples_jsonl: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/stability_smoke_after_gatefix/flat_action_samples.jsonl`
