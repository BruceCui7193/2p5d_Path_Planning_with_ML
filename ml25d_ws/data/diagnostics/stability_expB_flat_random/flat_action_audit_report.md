## Flat Action Audit Summary
| groups_total | groups_completed | valid_samples | invalid_attempts | flat_fail_rate | accept_flat_fail_lt_5pct | accept_a3a4_no_progress_gt3 | accept_a3a4_stuck_semantics | recommend_remove_a3a4 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 36 | 36 | 360 | 8 | 0.0000 | True | True | True | False |

### Notes
- flat 总 fail_rate 已低于 5%。
- a3/a4 未出现显著非 slip 失败放大，可保留。

## Per Group Metrics
| vehicle_type | action_id | friction_class | target_count | valid_count | fail_rate | stuck_fail_rate | q_roll_mean | q_pitch_mean | q_lift_mean | p_bottom_mean | translation_progress_mean | angular_progress_mean | translation_drift_mean | fail_reason_distribution |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| city_small | a0 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0053 | 0.0066 | 0.0000 | 0.0000 | 0.9842 | NaN | 0.2953 | {} |
| city_small | a0 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0053 | 0.0069 | 0.0000 | 0.0000 | 0.9781 | NaN | 0.2934 | {} |
| city_small | a0 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0052 | 0.0066 | 0.0000 | 0.0000 | 0.9804 | NaN | 0.2941 | {} |
| city_small | a0 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0054 | 0.0065 | 0.0000 | 0.0000 | 0.9807 | NaN | 0.2942 | {} |
| city_small | a1 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0061 | 0.0076 | 0.0000 | 0.0000 | 0.9423 | NaN | 0.2827 | {} |
| city_small | a1 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0060 | 0.0075 | 0.0000 | 0.0000 | 0.9289 | NaN | 0.2787 | {} |
| city_small | a1 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0061 | 0.0076 | 0.0000 | 0.0000 | 0.9690 | NaN | 0.2907 | {} |
| city_small | a1 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0061 | 0.0076 | 0.0000 | 0.0000 | 0.9289 | NaN | 0.2787 | {} |
| city_small | a2 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0066 | 0.0062 | 0.0000 | 0.0000 | 0.9841 | NaN | 0.2952 | {} |
| city_small | a2 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0064 | 0.0063 | 0.0000 | 0.0000 | 0.9850 | NaN | 0.2955 | {} |
| city_small | a2 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0063 | 0.0062 | 0.0000 | 0.0000 | 0.9841 | NaN | 0.2952 | {} |
| city_small | a2 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0064 | 0.0061 | 0.0000 | 0.0000 | 0.9836 | NaN | 0.2951 | {} |
| mountain_large | a0 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0022 | 0.0011 | 0.0000 | 0.0000 | 0.9512 | NaN | 0.2853 | {} |
| mountain_large | a0 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0023 | 0.0012 | 0.0000 | 0.0000 | 0.9631 | NaN | 0.2889 | {} |
| mountain_large | a0 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0022 | 0.0011 | 0.0000 | 0.0000 | 0.9512 | NaN | 0.2853 | {} |
| mountain_large | a0 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0022 | 0.0011 | 0.0000 | 0.0000 | 0.9392 | NaN | 0.2818 | {} |
| mountain_large | a1 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0027 | 0.0021 | 0.0000 | 0.0000 | 1.0675 | NaN | 0.3202 | {} |
| mountain_large | a1 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0027 | 0.0021 | 0.0000 | 0.0000 | 1.0675 | NaN | 0.3202 | {} |
| mountain_large | a1 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0028 | 0.0021 | 0.0000 | 0.0000 | 1.0555 | NaN | 0.3166 | {} |
| mountain_large | a1 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0027 | 0.0021 | 0.0000 | 0.0000 | 1.0794 | NaN | 0.3238 | {} |
| mountain_large | a2 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0032 | 0.0014 | 0.0000 | 0.0000 | 1.0865 | NaN | 0.3259 | {} |
| mountain_large | a2 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0033 | 0.0014 | 0.0000 | 0.0000 | 1.0349 | NaN | 0.3105 | {} |
| mountain_large | a2 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0032 | 0.0015 | 0.0000 | 0.0000 | 1.1381 | NaN | 0.3414 | {} |
| mountain_large | a2 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0033 | 0.0015 | 0.0000 | 0.0000 | 1.0521 | NaN | 0.3156 | {} |
| offroad_medium | a0 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0011 | 0.0024 | 0.0000 | 0.0000 | 0.9855 | NaN | 0.2956 | {} |
| offroad_medium | a0 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0011 | 0.0022 | 0.0000 | 0.0000 | 0.9873 | NaN | 0.2962 | {} |
| offroad_medium | a0 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0011 | 0.0027 | 0.0000 | 0.0000 | 0.9859 | NaN | 0.2958 | {} |
| offroad_medium | a0 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0012 | 0.0023 | 0.0000 | 0.0000 | 0.9856 | NaN | 0.2957 | {} |
| offroad_medium | a1 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0016 | 0.0049 | 0.0000 | 0.0000 | 0.9255 | NaN | 0.2777 | {} |
| offroad_medium | a1 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0016 | 0.0047 | 0.0000 | 0.0000 | 0.9299 | NaN | 0.2790 | {} |
| offroad_medium | a1 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0016 | 0.0046 | 0.0000 | 0.0000 | 0.9424 | NaN | 0.2827 | {} |
| offroad_medium | a1 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0017 | 0.0046 | 0.0000 | 0.0000 | 0.9549 | NaN | 0.2865 | {} |
| offroad_medium | a2 | dry_hard | 10 | 10 | 0.0000 | 0.0000 | 0.0014 | 0.0019 | 0.0000 | 0.0000 | 1.0776 | NaN | 0.3233 | {} |
| offroad_medium | a2 | grass_soft | 10 | 10 | 0.0000 | 0.0000 | 0.0014 | 0.0019 | 0.0000 | 0.0000 | 1.0549 | NaN | 0.3165 | {} |
| offroad_medium | a2 | mixed | 10 | 10 | 0.0000 | 0.0000 | 0.0014 | 0.0025 | 0.0000 | 0.0000 | 1.0583 | NaN | 0.3175 | {} |
| offroad_medium | a2 | wet_muddy | 10 | 10 | 0.0000 | 0.0000 | 0.0016 | 0.0019 | 0.0000 | 0.0000 | 1.0560 | NaN | 0.3168 | {} |

json_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/stability_expB_flat_random/flat_action_audit_report.json`
samples_jsonl: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/stability_expB_flat_random/flat_action_samples.jsonl`
