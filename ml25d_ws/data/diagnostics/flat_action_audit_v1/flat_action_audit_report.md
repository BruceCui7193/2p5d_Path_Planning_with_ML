## Flat Action Audit Summary
| groups_total | groups_completed | valid_samples | invalid_attempts | flat_fail_rate | accept_flat_fail_lt_5pct | accept_a3a4_no_progress_gt3 | accept_a3a4_stuck_semantics | recommend_remove_a3a4 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 60 | 60 | 1200 | 300 | 0.3900 | False | True | True | True |

### Notes
- flat 总 fail_rate 仍高于 5%，不满足验收。
- a3/a4 在 flat 上仍有较高 roll/pitch/lift 非 slip 失败，建议从正式数据集与主动作空间移除。

## Per Group Metrics
| vehicle_type | action_id | friction_class | target_count | valid_count | fail_rate | stuck_fail_rate | q_roll_mean | q_pitch_mean | q_lift_mean | translation_progress_mean | angular_progress_mean | translation_drift_mean | fail_reason_distribution |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| city_small | a0 | dry_hard | 20 | 20 | 0.0000 | 0.0000 | 0.0101 | 0.0114 | 0.0000 | 0.9909 | NaN | 0.2973 | {} |
| city_small | a0 | grass_soft | 20 | 20 | 0.0500 | 0.0500 | 0.0585 | 0.0601 | 0.0000 | 0.9337 | NaN | 0.2801 | {"bottom": 1, "pitch": 1, "roll": 1, "slip": 1, "stuck": 1} |
| city_small | a0 | mixed | 20 | 20 | 1.0000 | 0.3000 | 0.6028 | 0.9094 | 0.0000 | 0.6150 | NaN | 0.1845 | {"bottom": 18, "pitch": 17, "roll": 12, "slip": 13, "stuck": 6} |
| city_small | a0 | wet_muddy | 20 | 20 | 0.2000 | 0.1000 | 0.1063 | 0.1549 | 0.0035 | 0.8462 | NaN | 0.2539 | {"bottom": 3, "pitch": 3, "roll": 2, "slip": 2, "stuck": 2} |
| city_small | a1 | dry_hard | 20 | 20 | 0.0000 | 0.0000 | 0.0103 | 0.0098 | 0.0000 | 0.9725 | NaN | 0.2918 | {} |
| city_small | a1 | grass_soft | 20 | 20 | 0.7500 | 0.1000 | 0.4026 | 0.6911 | 0.0000 | 0.8067 | NaN | 0.2420 | {"bottom": 15, "pitch": 11, "roll": 8, "slip": 11, "stuck": 2} |
| city_small | a1 | mixed | 20 | 20 | 0.1000 | 0.0500 | 0.1073 | 0.1059 | 0.0000 | 0.8928 | NaN | 0.2678 | {"bottom": 2, "pitch": 2, "roll": 2, "slip": 2, "stuck": 1} |
| city_small | a1 | wet_muddy | 20 | 20 | 0.0500 | 0.0000 | 0.0550 | 0.0559 | 0.0000 | 0.9593 | NaN | 0.2878 | {"bottom": 1, "pitch": 1, "roll": 1, "slip": 1} |
| city_small | a2 | dry_hard | 20 | 20 | 0.0000 | 0.0000 | 0.0106 | 0.0097 | 0.0000 | 0.9900 | NaN | 0.2970 | {} |
| city_small | a2 | grass_soft | 20 | 20 | 0.8500 | 0.3000 | 0.4061 | 0.7966 | 0.0000 | 0.5915 | NaN | 0.1774 | {"bottom": 17, "pitch": 15, "roll": 8, "slip": 14, "stuck": 6} |
| city_small | a2 | mixed | 20 | 20 | 0.0000 | 0.0000 | 0.0097 | 0.0097 | 0.0000 | 0.9910 | NaN | 0.2973 | {} |
| city_small | a2 | wet_muddy | 20 | 20 | 0.1500 | 0.0000 | 0.1575 | 0.1563 | 0.0000 | 0.9353 | NaN | 0.2806 | {"bottom": 3, "pitch": 3, "roll": 3, "slip": 3} |
| city_small | a3 | dry_hard | 20 | 20 | 0.4000 | 0.3500 | 0.0561 | 0.0556 | 0.0000 | NaN | 0.6464 | 0.0145 | {"bottom": 1, "pitch": 1, "roll": 1, "stuck": 7} |
| city_small | a3 | grass_soft | 20 | 20 | 0.8000 | 0.0500 | 0.8000 | 0.6188 | 0.0000 | NaN | 0.9380 | 0.0118 | {"bottom": 15, "roll": 16, "stuck": 1} |
| city_small | a3 | mixed | 20 | 20 | 0.5500 | 0.2000 | 0.3526 | 0.3534 | 0.0000 | NaN | 0.7240 | 0.0795 | {"bottom": 7, "pitch": 7, "roll": 7, "stuck": 4} |
| city_small | a3 | wet_muddy | 20 | 20 | 0.3500 | 0.3500 | 0.0094 | 0.0086 | 0.0000 | NaN | 0.6415 | 0.0047 | {"stuck": 7} |
| city_small | a4 | dry_hard | 20 | 20 | 0.9500 | 0.2500 | 0.9500 | 0.6953 | 0.0000 | NaN | 0.7493 | 0.0162 | {"bottom": 17, "roll": 19, "stuck": 5} |
| city_small | a4 | grass_soft | 20 | 20 | 0.4000 | 0.2000 | 0.2032 | 0.2046 | 0.0000 | NaN | 0.7867 | 0.0259 | {"bottom": 4, "pitch": 4, "roll": 4, "stuck": 4} |
| city_small | a4 | mixed | 20 | 20 | 0.9000 | 0.2000 | 0.9000 | 0.6051 | 0.0000 | NaN | 0.7952 | 0.0114 | {"bottom": 17, "roll": 18, "stuck": 4} |
| city_small | a4 | wet_muddy | 20 | 20 | 0.5000 | 0.5000 | 0.0057 | 0.0063 | 0.0000 | NaN | 0.4916 | 0.0047 | {"stuck": 10} |
| mountain_large | a0 | dry_hard | 20 | 20 | 0.1500 | 0.0000 | 0.1509 | 0.1514 | 0.0000 | 1.7061 | NaN | 0.5118 | {"bottom": 3, "pitch": 3, "roll": 3, "slip": 3} |
| mountain_large | a0 | grass_soft | 20 | 20 | 0.8000 | 0.3000 | 0.2515 | 0.6998 | 0.0000 | 0.6547 | NaN | 0.1964 | {"bottom": 12, "pitch": 11, "roll": 4, "slip": 7, "stuck": 6} |
| mountain_large | a0 | mixed | 20 | 20 | 0.0000 | 0.0000 | 0.0011 | 0.0013 | 0.0000 | 0.9227 | NaN | 0.2768 | {} |
| mountain_large | a0 | wet_muddy | 20 | 20 | 0.0000 | 0.0000 | 0.0026 | 0.0028 | 0.0000 | 0.9717 | NaN | 0.2915 | {} |
| mountain_large | a1 | dry_hard | 20 | 20 | 0.9000 | 0.1000 | 0.8646 | 0.7614 | 0.0000 | 0.8842 | NaN | 0.2653 | {"bottom": 16, "pitch": 13, "roll": 17, "slip": 12, "stuck": 2} |
| mountain_large | a1 | grass_soft | 20 | 20 | 0.1500 | 0.0000 | 0.1512 | 0.1514 | 0.0000 | 2.0568 | NaN | 0.6170 | {"bottom": 3, "pitch": 3, "roll": 3, "slip": 3} |
| mountain_large | a1 | mixed | 20 | 20 | 0.1000 | 0.0000 | 0.1019 | 0.1077 | 0.0000 | 0.9232 | NaN | 0.2770 | {"bottom": 2, "pitch": 2, "roll": 2, "slip": 2} |
| mountain_large | a1 | wet_muddy | 20 | 20 | 0.0000 | 0.0000 | 0.0034 | 0.0067 | 0.0000 | 1.0278 | NaN | 0.3083 | {} |
| mountain_large | a2 | dry_hard | 20 | 20 | 0.7000 | 0.0500 | 0.0204 | 0.6498 | 0.0000 | 0.9434 | NaN | 0.2830 | {"bottom": 12, "pitch": 11, "slip": 6, "stuck": 1} |
| mountain_large | a2 | grass_soft | 20 | 20 | 0.1000 | 0.0000 | 0.1028 | 0.1018 | 0.0000 | 1.6777 | NaN | 0.5033 | {"bottom": 2, "pitch": 2, "roll": 2, "slip": 2} |
| mountain_large | a2 | mixed | 20 | 20 | 0.0500 | 0.0000 | 0.0533 | 0.0519 | 0.0000 | 1.1475 | NaN | 0.3442 | {"bottom": 1, "pitch": 1, "roll": 1, "slip": 1} |
| mountain_large | a2 | wet_muddy | 20 | 20 | 0.0000 | 0.0000 | 0.0043 | 0.0033 | 0.0000 | 1.0305 | NaN | 0.3091 | {} |
| mountain_large | a3 | dry_hard | 20 | 20 | 0.9500 | 0.6500 | 0.0423 | 0.9500 | 0.0000 | NaN | 0.4086 | 0.0354 | {"bottom": 19, "pitch": 19, "stuck": 13} |
| mountain_large | a3 | grass_soft | 20 | 20 | 0.6000 | 0.6000 | 0.0014 | 0.0019 | 0.0000 | NaN | 0.4021 | 0.0080 | {"stuck": 12} |
| mountain_large | a3 | mixed | 20 | 20 | 0.5000 | 0.5000 | 0.0509 | 0.0514 | 0.0000 | NaN | 0.4951 | 0.0708 | {"bottom": 1, "pitch": 1, "roll": 1, "stuck": 10} |
| mountain_large | a3 | wet_muddy | 20 | 20 | 0.3500 | 0.3000 | 0.0515 | 0.0519 | 0.0000 | NaN | 0.6825 | 0.0942 | {"bottom": 1, "pitch": 1, "roll": 1, "stuck": 6} |
| mountain_large | a4 | dry_hard | 20 | 20 | 0.9000 | 0.7500 | 0.0337 | 0.9000 | 0.0000 | NaN | 0.3162 | 0.0400 | {"bottom": 16, "pitch": 18, "stuck": 15} |
| mountain_large | a4 | grass_soft | 20 | 20 | 0.4000 | 0.4000 | 0.0017 | 0.0083 | 0.0000 | NaN | 0.5699 | 0.0064 | {"stuck": 8} |
| mountain_large | a4 | mixed | 20 | 20 | 0.7000 | 0.6000 | 0.0229 | 0.6506 | 0.0000 | NaN | 0.4348 | 0.0372 | {"bottom": 12, "pitch": 13, "stuck": 12} |
| mountain_large | a4 | wet_muddy | 20 | 20 | 0.4500 | 0.3500 | 0.1008 | 0.1047 | 0.0000 | NaN | 0.5749 | 0.3338 | {"bottom": 2, "pitch": 2, "roll": 2, "stuck": 7} |
| offroad_medium | a0 | dry_hard | 20 | 20 | 0.0000 | 0.0000 | 0.0029 | 0.0032 | 0.0000 | 0.9964 | NaN | 0.2989 | {} |
| offroad_medium | a0 | grass_soft | 20 | 20 | 0.3000 | 0.1000 | 0.3014 | 0.3018 | 0.0000 | 0.8792 | NaN | 0.2638 | {"bottom": 6, "pitch": 6, "roll": 6, "slip": 6, "stuck": 2} |
| offroad_medium | a0 | mixed | 20 | 20 | 0.8500 | 0.1000 | 0.6055 | 0.7487 | 0.0000 | 0.8559 | NaN | 0.2568 | {"bottom": 15, "pitch": 14, "roll": 12, "slip": 10, "stuck": 2} |
| offroad_medium | a0 | wet_muddy | 20 | 20 | 0.1000 | 0.0500 | 0.1016 | 0.1031 | 0.0000 | 0.8994 | NaN | 0.2698 | {"bottom": 2, "pitch": 2, "roll": 2, "slip": 2, "stuck": 1} |
| offroad_medium | a1 | dry_hard | 20 | 20 | 0.0000 | 0.0000 | 0.0036 | 0.0046 | 0.0000 | 0.9899 | NaN | 0.2970 | {} |
| offroad_medium | a1 | grass_soft | 20 | 20 | 0.0000 | 0.0000 | 0.0035 | 0.0050 | 0.0000 | 0.9713 | NaN | 0.2914 | {} |
| offroad_medium | a1 | mixed | 20 | 20 | 0.3500 | 0.0500 | 0.3508 | 0.3514 | 0.0000 | 0.9370 | NaN | 0.2811 | {"bottom": 7, "pitch": 7, "roll": 7, "slip": 7, "stuck": 1} |
| offroad_medium | a1 | wet_muddy | 20 | 20 | 1.0000 | 0.3500 | 0.3159 | 0.8750 | 0.0000 | 0.5685 | NaN | 0.1706 | {"bottom": 19, "pitch": 15, "roll": 6, "slip": 14, "stuck": 7} |
| offroad_medium | a2 | dry_hard | 20 | 20 | 0.1000 | 0.0000 | 0.1018 | 0.1018 | 0.0000 | 0.9521 | NaN | 0.2856 | {"bottom": 2, "pitch": 2, "roll": 2, "slip": 2} |
| offroad_medium | a2 | grass_soft | 20 | 20 | 0.0000 | 0.0000 | 0.0029 | 0.0032 | 0.0000 | 1.0572 | NaN | 0.3172 | {} |
| offroad_medium | a2 | mixed | 20 | 20 | 0.0500 | 0.0000 | 0.0513 | 0.0512 | 0.0000 | 0.9480 | NaN | 0.2844 | {"bottom": 1, "pitch": 1, "roll": 1, "slip": 1} |
| offroad_medium | a2 | wet_muddy | 20 | 20 | 0.7500 | 0.2500 | 0.5659 | 0.6511 | 0.0000 | 0.7480 | NaN | 0.2244 | {"bottom": 14, "pitch": 11, "roll": 11, "slip": 10, "stuck": 5} |
| offroad_medium | a3 | dry_hard | 20 | 20 | 0.1500 | 0.1500 | 0.0029 | 0.0035 | 0.0000 | NaN | 0.8484 | 0.0005 | {"stuck": 3} |
| offroad_medium | a3 | grass_soft | 20 | 20 | 0.6000 | 0.3500 | 0.2514 | 0.2518 | 0.0000 | NaN | 0.6246 | 0.0431 | {"bottom": 5, "pitch": 5, "roll": 5, "stuck": 7} |
| offroad_medium | a3 | mixed | 20 | 20 | 0.4500 | 0.2500 | 0.2013 | 0.2021 | 0.0000 | NaN | 0.7327 | 0.1060 | {"bottom": 4, "pitch": 4, "roll": 4, "stuck": 5} |
| offroad_medium | a3 | wet_muddy | 20 | 20 | 0.7500 | 0.6000 | 0.0168 | 0.7500 | 0.0000 | NaN | 0.3991 | 0.0692 | {"bottom": 15, "pitch": 15, "stuck": 12} |
| offroad_medium | a4 | dry_hard | 20 | 20 | 0.3500 | 0.3500 | 0.0034 | 0.0023 | 0.0000 | NaN | 0.6268 | 0.0103 | {"stuck": 7} |
| offroad_medium | a4 | grass_soft | 20 | 20 | 0.8500 | 0.7000 | 0.0251 | 0.8500 | 0.0000 | NaN | 0.3076 | 0.0818 | {"bottom": 15, "pitch": 17, "stuck": 14} |
| offroad_medium | a4 | mixed | 20 | 20 | 0.6000 | 0.6000 | 0.0037 | 0.0026 | 0.0000 | NaN | 0.3903 | 0.0095 | {"stuck": 12} |
| offroad_medium | a4 | wet_muddy | 20 | 20 | 0.3500 | 0.3000 | 0.1021 | 0.1015 | 0.0000 | NaN | 0.6693 | 0.0121 | {"bottom": 2, "pitch": 2, "roll": 2, "stuck": 6} |

json_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/flat_action_audit_v1/flat_action_audit_report.json`
samples_jsonl: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/flat_action_audit_v1/flat_action_samples.jsonl`
