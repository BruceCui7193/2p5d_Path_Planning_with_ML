## Experiment 1: Existing Flat Sample Offline Audit
| sample_count | fail_count | fail_rate |
| --- | --- | --- |
| 1200 | 468 | 0.3900 |

### Fail Reason Distribution
| fail_reason | count |
| --- | --- |
| bottom | 330 |
| pitch | 266 |
| roll | 196 |
| slip | 135 |
| stuck | 236 |

### q Distribution (Fail vs Pass)
| metric | fail_mean | fail_p50 | fail_p95 | fail_max | fail_frac_eq_1 | pass_mean | pass_p50 | pass_p95 | pass_max | pass_frac_eq_1 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| q_roll | 0.4314 | 0.0381 | 1.0000 | 1.0000 | 0.4188 | 0.0040 | 0.0030 | 0.0122 | 0.0247 | 0.0000 |
| q_pitch | 0.6851 | 1.0000 | 1.0000 | 1.0000 | 0.5556 | 0.0071 | 0.0030 | 0.0118 | 0.7695 | 0.0000 |
| q_lift | 0.0001 | 0.0000 | 0.0000 | 0.0700 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| p_stuck | 0.5043 | 1.0000 | 1.0000 | 1.0000 | 0.5043 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |

### Fail Reason Co-occurrence
| reason | roll | pitch | slip | lift | bottom | stuck |
| --- | --- | --- | --- | --- | --- | --- |
| roll | 196 | 127 | 91 | 0 | 189 | 36 |
| pitch | 127 | 266 | 109 | 0 | 249 | 98 |
| slip | 91 | 109 | 135 | 0 | 135 | 37 |
| lift | 0 | 0 | 0 | 0 | 0 | 0 |
| bottom | 189 | 249 | 135 | 0 | 330 | 117 |
| stuck | 36 | 98 | 37 | 0 | 117 | 236 |

## Experiment 2: Replay 12 Fail + Pass Controls with Time Series
| selected_case_count | ok_count | runtime_failure_count | first_trigger_in_0p3s_count |
| --- | --- | --- | --- |
| 18 | 18 | 0 | 0 |

### First Trigger Reason Distribution
| trigger_reason | count |
| --- | --- |
| none | 18 |

### Replay Case Summary
| case_id | case_type | vehicle_id | action_id | friction_class | friction_mu | original_fail_reasons | replay_fail_reasons | first_fail_trigger_reason | first_fail_trigger_s | first_trigger_in_0p3s | roll_max_deg | roll_p95_deg | pitch_max_deg | pitch_p95_deg | roll_over_thr_duration_s | pitch_over_thr_duration_s | running_bottom_ratio_final | running_q_lift_final | p_stuck | translation_progress | angular_progress | translation_drift |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| a0_fail_01 | fail | urban_small | a0 | mixed | 0.8186 | ['bottom'] | [] | none | NaN | False | 0.1578 | 0.1216 | 0.2179 | 0.1423 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9888 | NaN | 0.2966 |
| a0_fail_02 | fail | urban_small | a0 | mixed | 0.5675 | ['pitch', 'slip', 'bottom'] | [] | none | NaN | False | 0.1661 | 0.1147 | 0.1992 | 0.1169 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9729 | NaN | 0.2919 |
| a0_fail_03 | fail | urban_small | a0 | mixed | 0.4734 | ['pitch'] | [] | none | NaN | False | 0.2237 | 0.1384 | 0.1773 | 0.1406 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9750 | NaN | 0.2925 |
| a1_fail_04 | fail | urban_small | a1 | grass_soft | 0.5981 | ['roll', 'slip', 'bottom', 'stuck'] | [] | none | NaN | False | 0.1632 | 0.1462 | 0.2176 | 0.1632 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9330 | 0.0003 | 0.2799 |
| a1_fail_05 | fail | urban_small | a1 | grass_soft | 0.5576 | ['roll', 'slip', 'bottom'] | [] | none | NaN | False | 0.1961 | 0.1294 | 0.1657 | 0.1470 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9330 | 0.0003 | 0.2799 |
| a1_fail_06 | fail | urban_small | a1 | grass_soft | 0.5308 | ['roll', 'pitch', 'bottom'] | [] | none | NaN | False | 0.2793 | 0.2208 | 0.2204 | 0.1699 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9151 | 0.0203 | 0.2745 |
| a2_fail_07 | fail | urban_small | a2 | grass_soft | 0.5151 | ['slip', 'bottom'] | [] | none | NaN | False | 0.2097 | 0.1133 | 0.2305 | 0.1705 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9769 | 0.0023 | 0.2931 |
| a2_fail_08 | fail | urban_small | a2 | grass_soft | 0.5892 | ['slip', 'bottom', 'stuck'] | [] | none | NaN | False | 0.1426 | 0.0918 | 0.1757 | 0.1334 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9907 | 0.0026 | 0.2972 |
| a2_fail_09 | fail | urban_small | a2 | grass_soft | 0.4518 | ['pitch', 'bottom'] | [] | none | NaN | False | 0.1687 | 0.1077 | 0.2305 | 0.1725 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9851 | 0.9768 | 0.2955 |
| a34_fail_10 | fail | urban_small | a3 | grass_soft | 0.4866 | ['roll', 'bottom'] | ['stuck'] | none | NaN | False | 0.1426 | 0.1414 | 0.0940 | 0.0670 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | NaN | 0.0017 | 0.0071 |
| a34_fail_11 | fail | urban_small | a3 | grass_soft | 0.4832 | ['roll', 'bottom'] | [] | none | NaN | False | 0.0792 | 0.0765 | 0.2234 | 0.1973 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | NaN | 0.9023 | 0.0045 |
| a34_fail_12 | fail | urban_small | a3 | grass_soft | 0.5751 | ['roll', 'bottom'] | ['stuck'] | none | NaN | False | 0.0945 | 0.0922 | 0.2271 | 0.2176 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | NaN | 0.0110 | 0.0074 |
| a0_pass_01 | pass | urban_small | a0 | dry_hard | 0.8754 | [] | [] | none | NaN | False | 0.1331 | 0.0878 | 0.2532 | 0.1355 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9751 | NaN | 0.2925 |
| a1_pass_02 | pass | urban_small | a1 | dry_hard | 0.8131 | [] | [] | none | NaN | False | 0.1772 | 0.1434 | 0.2014 | 0.1484 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9438 | 0.0004 | 0.2831 |
| a2_pass_03 | pass | urban_small | a2 | dry_hard | 0.7479 | [] | [] | none | NaN | False | 0.2174 | 0.1684 | 0.2648 | 0.1796 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9837 | 0.9819 | 0.2951 |
| a3_pass_04 | pass | urban_small | a3 | grass_soft | 0.5090 | [] | [] | none | NaN | False | 0.2624 | 0.1697 | 0.1877 | 0.0965 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | NaN | 1.0173 | 0.0010 |
| a4_pass_05 | pass | urban_small | a4 | dry_hard | 0.7210 | [] | ['stuck'] | none | NaN | False | 0.1267 | 0.1255 | 0.1540 | 0.1462 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | NaN | 0.0196 | 0.0009 |
| pass_extra_06 | pass | urban_small | a0 | dry_hard | 0.7567 | [] | [] | none | NaN | False | 0.1661 | 0.0934 | 0.1800 | 0.1064 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9728 | NaN | 0.2918 |

## Experiment 3: Golden Flat 45 Samples A/B
### Version Summary
| version | valid_count | runtime_failure_count | fail_rate | q_roll_mean | q_pitch_mean | q_slip_mean | p_bottom_mean | p_stuck_mean | fail_reason_distribution |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A_current_cmd | 45 | 0 | 0.0000 | 0.0047 | 0.0055 | 0.1880 | 0.0000 | 0.0000 | {} |
| B_ramp0p3_label_after_ramp | 45 | 0 | 0.0222 | 0.0057 | 0.0285 | 0.2165 | 0.0000 | 0.0000 | {'lift': 1, 'pitch': 1} |

### By Action
| version | action_id | count | fail_rate | q_roll_mean | q_pitch_mean | q_slip_mean | p_bottom_mean | p_stuck_mean |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A_current_cmd | a0 | 15 | 0.0000 | 0.0043 | 0.0057 | 0.0247 | 0.0000 | 0.0000 |
| A_current_cmd | a1 | 15 | 0.0000 | 0.0049 | 0.0060 | 0.2880 | 0.0000 | 0.0000 |
| A_current_cmd | a2 | 15 | 0.0000 | 0.0050 | 0.0047 | 0.2514 | 0.0000 | 0.0000 |
| B_ramp0p3_label_after_ramp | a0 | 15 | 0.0000 | 0.0043 | 0.0071 | 0.0203 | 0.0000 | 0.0000 |
| B_ramp0p3_label_after_ramp | a1 | 15 | 0.0667 | 0.0076 | 0.0732 | 0.2925 | 0.0000 | 0.0000 |
| B_ramp0p3_label_after_ramp | a2 | 15 | 0.0000 | 0.0051 | 0.0054 | 0.3367 | 0.0000 | 0.0000 |

### By Vehicle
| version | vehicle_type | count | fail_rate | q_roll_mean | q_pitch_mean | q_slip_mean | p_bottom_mean | p_stuck_mean |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A_current_cmd | city_small | 15 | 0.0000 | 0.0081 | 0.0093 | 0.0896 | 0.0000 | 0.0000 |
| A_current_cmd | offroad_medium | 15 | 0.0000 | 0.0026 | 0.0039 | 0.1404 | 0.0000 | 0.0000 |
| A_current_cmd | mountain_large | 15 | 0.0000 | 0.0035 | 0.0031 | 0.3341 | 0.0000 | 0.0000 |
| B_ramp0p3_label_after_ramp | city_small | 15 | 0.0000 | 0.0088 | 0.0088 | 0.0880 | 0.0000 | 0.0000 |
| B_ramp0p3_label_after_ramp | offroad_medium | 15 | 0.0000 | 0.0023 | 0.0042 | 0.2193 | 0.0000 | 0.0000 |
| B_ramp0p3_label_after_ramp | mountain_large | 15 | 0.0667 | 0.0060 | 0.0726 | 0.3423 | 0.0000 | 0.0000 |

json_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/flat_root_cause_v1/flat_root_cause_diagnostic.json`
md_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/diagnostics/flat_root_cause_v1/flat_root_cause_diagnostic.md`
