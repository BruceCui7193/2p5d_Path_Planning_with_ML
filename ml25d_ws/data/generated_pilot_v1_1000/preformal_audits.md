## Flat Fail Audit
| flat_sample_count | flat_fail_count | flat_fail_rate | assessment_flag |
| --- | --- | --- | --- |
| 102 | 17 | 0.1667 | needs_investigation |

assessment: flat_fail 含 roll/pitch/bottom/lift 主因，说明平地仍有结构性非 slip 失败，需要继续排查。

### Flat Fail Reason Distribution
| fail_reason | count |
| --- | --- |
| lift | 4 |
| pitch | 12 |
| roll | 7 |
| slip | 2 |
| stuck | 7 |

### Flat By Vehicle
| vehicle_type | count | fail_count | fail_rate |
| --- | --- | --- | --- |
| mountain_large | 37 | 2 | 0.0541 |
| standard_offroad | 34 | 4 | 0.1176 |
| urban_small | 31 | 11 | 0.3548 |

### Flat By Action
| action_id | count | fail_count | fail_rate |
| --- | --- | --- | --- |
| a0 | 33 | 5 | 0.1515 |
| a1 | 29 | 6 | 0.2069 |
| a2 | 22 | 1 | 0.0455 |
| a3 | 7 | 1 | 0.1429 |
| a4 | 11 | 4 | 0.3636 |

### Flat By Friction Class
| friction_class | count | fail_count | fail_rate |
| --- | --- | --- | --- |
| dry_hard | 34 | 2 | 0.0588 |
| grass_soft | 31 | 9 | 0.2903 |
| mixed | 13 | 2 | 0.1538 |
| wet_muddy | 24 | 4 | 0.1667 |

### Flat By Friction Mu Bin
| mu_bin | count | fail_count | fail_rate |
| --- | --- | --- | --- |
| [0.0,0.4) | 25 | 4 | 0.1600 |
| [0.4,0.6) | 35 | 10 | 0.2857 |
| [0.6,0.8) | 18 | 2 | 0.1111 |
| [0.8,1.0) | 24 | 1 | 0.0417 |

### Flat Fail Details
| sample_id | vehicle_type | action_id | friction_class | friction_mu | q_roll | q_pitch | q_slip | q_lift | p_bottom | p_stuck | progress_ratio | fail_reasons |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 29 | standard_offroad | a1 | wet_muddy | 0.2532 | 1.0000 | 1.0000 | 0.4392 | 0.2150 | 0.0000 | 0.0000 | 0.7608 | ['roll', 'pitch'] |
| 32 | urban_small | a1 | grass_soft | 0.4123 | 0.9386 | 1.0000 | 0.2045 | 0.5683 | 0.0000 | 1.0000 | 0.1683 | ['roll', 'pitch', 'lift', 'stuck'] |
| 44 | urban_small | a4 | mixed | 0.4514 | 0.0586 | 0.0523 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | 0.0045 | ['stuck'] |
| 45 | urban_small | a4 | grass_soft | 0.5259 | 0.0779 | 0.0268 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | 0.0168 | ['stuck'] |
| 57 | urban_small | a1 | wet_muddy | 0.3777 | 0.2750 | 1.0000 | 0.4745 | 0.1367 | 0.0000 | 0.0000 | 0.8072 | ['pitch'] |
| 95 | mountain_large | a4 | grass_soft | 0.4819 | 0.0559 | 0.0740 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | 0.1426 | ['stuck'] |
| 97 | urban_small | a3 | grass_soft | 0.5114 | 0.0442 | 0.0509 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | 0.0161 | ['stuck'] |
| 118 | urban_small | a1 | grass_soft | 0.4724 | 1.0000 | 1.0000 | 1.0000 | 0.3500 | 0.0000 | 1.0000 | 0.1952 | ['roll', 'pitch', 'slip', 'lift', 'stuck'] |
| 149 | standard_offroad | a1 | grass_soft | 0.4529 | 1.0000 | 1.0000 | 0.0036 | 0.1200 | 0.0000 | 0.0000 | 0.9828 | ['roll', 'pitch'] |
| 243 | urban_small | a0 | wet_muddy | 0.2728 | 1.0000 | 1.0000 | 1.0000 | 0.3983 | 0.0000 | 0.0000 | 0.9967 | ['roll', 'pitch', 'slip', 'lift'] |
| 306 | standard_offroad | a0 | wet_muddy | 0.3174 | 0.0807 | 1.0000 | 0.2996 | 0.1850 | 0.0000 | 0.0000 | 0.4962 | ['pitch'] |
| 410 | urban_small | a0 | mixed | 0.8594 | 1.0000 | 1.0000 | 0.1956 | 0.5125 | 0.0000 | 0.0000 | 1.0148 | ['roll', 'pitch', 'lift'] |
| 432 | mountain_large | a4 | grass_soft | 0.4105 | 0.0601 | 0.0388 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | 0.0014 | ['stuck'] |
| 474 | urban_small | a0 | dry_hard | 0.7693 | 0.1452 | 1.0000 | 0.1412 | 0.0700 | 0.0000 | 0.0000 | 0.5800 | ['pitch'] |
| 500 | urban_small | a0 | grass_soft | 0.4569 | 0.1732 | 1.0000 | 0.4449 | 0.1975 | 0.0000 | 0.0000 | 0.9436 | ['pitch'] |
| 544 | standard_offroad | a2 | dry_hard | 0.7022 | 1.0000 | 1.0000 | 0.3908 | 0.2000 | 0.0000 | 0.0000 | 0.9756 | ['roll', 'pitch'] |
| 944 | urban_small | a1 | grass_soft | 0.5144 | 0.0699 | 0.9019 | 0.2309 | 0.0150 | 0.0000 | 0.0000 | 0.8994 | ['pitch'] |

## Progress Outlier Audit
| progress_threshold | sample_count | outlier_count | outlier_ratio | conclusion_flag |
| --- | --- | --- | --- | --- |
| 3.0000 | 1000 | 89 | 0.0890 | needs_investigation |

conclusion: 存在疑似异常滑走/弹飞样本，建议进入 invalid 或收紧 progress 定义。

### Outlier Heuristic Distribution
| heuristic | count |
| --- | --- |
| downhill_or_low_mu_sliding | 4 |
| likely_valid_outlier | 2 |
| possible_collision_rebound_or_bounce | 2 |
| turn_overshoot_progress_definition | 81 |

### Outlier By Terrain
| terrain_type | count |
| --- | --- |
| bumps | 12 |
| flat | 9 |
| lateral_pits | 10 |
| lateral_slope | 15 |
| mixed_random | 7 |
| pits | 6 |
| slope_bumps | 9 |
| steps | 3 |
| uniform_slope | 9 |
| waves | 9 |

### Outlier By Vehicle
| vehicle_type | count |
| --- | --- |
| mountain_large | 30 |
| standard_offroad | 35 |
| urban_small | 24 |

### Outlier By Action
| action_id | count |
| --- | --- |
| a0 | 2 |
| a1 | 3 |
| a2 | 3 |
| a3 | 33 |
| a4 | 48 |

### Outlier Fail Reason Distribution
| fail_reason | count |
| --- | --- |
| bottom | 14 |
| lift | 9 |
| pitch | 17 |
| roll | 18 |
| slip | 8 |

### Outlier Details
| sample_id | terrain_type | vehicle_type | action_id | friction_class | friction_mu | progress_ratio | estimated_displacement_m | estimated_mean_speed_mps | commanded_speed_mps | estimated_heading_change_deg | fail_reasons | heuristic_judgement |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 647 | uniform_slope | standard_offroad | a4 | dry_hard | 0.7684 | 15.9960 | NaN | NaN | 0.0000 | 359.9106 | [] | turn_overshoot_progress_definition |
| 908 | lateral_slope | standard_offroad | a3 | grass_soft | 0.4972 | 15.9079 | NaN | NaN | 0.0000 | 357.9280 | [] | turn_overshoot_progress_definition |
| 249 | slope_bumps | standard_offroad | a4 | grass_soft | 0.5101 | 15.5004 | NaN | NaN | 0.0000 | 348.7580 | ['roll'] | turn_overshoot_progress_definition |
| 876 | bumps | urban_small | a4 | dry_hard | 0.7762 | 15.2180 | NaN | NaN | 0.0000 | 342.4051 | ['pitch', 'bottom'] | turn_overshoot_progress_definition |
| 814 | waves | mountain_large | a4 | wet_muddy | 0.2326 | 15.0567 | NaN | NaN | 0.0000 | 338.7761 | [] | turn_overshoot_progress_definition |
| 993 | lateral_slope | standard_offroad | a4 | grass_soft | 0.4294 | 15.0288 | NaN | NaN | 0.0000 | 338.1476 | ['roll'] | turn_overshoot_progress_definition |
| 829 | lateral_slope | urban_small | a4 | wet_muddy | 0.2288 | 15.0004 | NaN | NaN | 0.0000 | 337.5093 | [] | turn_overshoot_progress_definition |
| 63 | uniform_slope | mountain_large | a0 | mixed | 0.4884 | 8.1133 | 2.4340 | 1.2170 | 0.1500 | 0.0000 | ['roll', 'pitch', 'slip', 'lift', 'bottom'] | downhill_or_low_mu_sliding |
| 894 | lateral_slope | urban_small | a3 | wet_muddy | 0.2367 | 7.8273 | NaN | NaN | 0.0000 | 176.1144 | [] | turn_overshoot_progress_definition |
| 377 | waves | urban_small | a3 | dry_hard | 0.7631 | 7.7986 | NaN | NaN | 0.0000 | 175.4692 | ['lift'] | turn_overshoot_progress_definition |
| 793 | mixed_random | urban_small | a4 | wet_muddy | 0.3630 | 7.7719 | NaN | NaN | 0.0000 | 174.8684 | ['bottom'] | turn_overshoot_progress_definition |
| 878 | bumps | mountain_large | a4 | grass_soft | 0.5823 | 7.7457 | NaN | NaN | 0.0000 | 174.2791 | [] | turn_overshoot_progress_definition |
| 453 | mixed_random | urban_small | a3 | wet_muddy | 0.2397 | 7.7254 | NaN | NaN | 0.0000 | 173.8205 | [] | turn_overshoot_progress_definition |
| 830 | lateral_pits | urban_small | a3 | grass_soft | 0.5171 | 7.7075 | NaN | NaN | 0.0000 | 173.4191 | ['bottom'] | turn_overshoot_progress_definition |
| 25 | waves | mountain_large | a3 | grass_soft | 0.4256 | 7.4425 | NaN | NaN | 0.0000 | 167.4565 | [] | turn_overshoot_progress_definition |
| 206 | bumps | standard_offroad | a4 | grass_soft | 0.4744 | 7.3908 | NaN | NaN | 0.0000 | 166.2938 | [] | turn_overshoot_progress_definition |
| 730 | bumps | urban_small | a4 | dry_hard | 0.7514 | 7.2899 | NaN | NaN | 0.0000 | 164.0233 | ['bottom'] | turn_overshoot_progress_definition |
| 458 | flat | standard_offroad | a3 | dry_hard | 0.8815 | 7.1586 | NaN | NaN | 0.0000 | 161.0683 | [] | turn_overshoot_progress_definition |
| 212 | lateral_slope | mountain_large | a3 | wet_muddy | 0.3849 | 7.0286 | NaN | NaN | 0.0000 | 158.1442 | [] | turn_overshoot_progress_definition |
| 639 | flat | urban_small | a4 | grass_soft | 0.5330 | 7.0073 | NaN | NaN | 0.0000 | 157.6633 | [] | turn_overshoot_progress_definition |
| 946 | lateral_pits | urban_small | a3 | wet_muddy | 0.3882 | 6.8777 | NaN | NaN | 0.0000 | 154.7489 | ['bottom'] | turn_overshoot_progress_definition |
| 536 | pits | standard_offroad | a3 | mixed | 0.8065 | 6.8134 | NaN | NaN | 0.0000 | 153.3011 | [] | turn_overshoot_progress_definition |
| 873 | waves | mountain_large | a3 | grass_soft | 0.4449 | 6.7417 | NaN | NaN | 0.0000 | 151.6872 | ['roll'] | turn_overshoot_progress_definition |
| 375 | uniform_slope | urban_small | a3 | grass_soft | 0.4553 | 6.7011 | NaN | NaN | 0.0000 | 150.7746 | ['pitch', 'bottom'] | turn_overshoot_progress_definition |
| 86 | flat | urban_small | a3 | mixed | 0.7937 | 6.6815 | NaN | NaN | 0.0000 | 150.3338 | [] | turn_overshoot_progress_definition |
| 581 | lateral_pits | urban_small | a4 | wet_muddy | 0.2852 | 6.5995 | NaN | NaN | 0.0000 | 148.4893 | ['bottom'] | turn_overshoot_progress_definition |
| 729 | flat | mountain_large | a3 | wet_muddy | 0.2358 | 6.5093 | NaN | NaN | 0.0000 | 146.4588 | [] | turn_overshoot_progress_definition |
| 461 | lateral_slope | mountain_large | a4 | dry_hard | 0.7825 | 6.4951 | NaN | NaN | 0.0000 | 146.1403 | [] | turn_overshoot_progress_definition |
| 593 | waves | mountain_large | a4 | grass_soft | 0.4953 | 6.4480 | NaN | NaN | 0.0000 | 145.0798 | [] | turn_overshoot_progress_definition |
| 193 | uniform_slope | mountain_large | a1 | dry_hard | 0.7140 | 6.4139 | 1.9242 | 0.9621 | 0.1200 | 144.3118 | ['roll', 'pitch', 'slip', 'lift', 'bottom'] | downhill_or_low_mu_sliding |
| 882 | lateral_slope | mountain_large | a4 | grass_soft | 0.4610 | 6.2752 | NaN | NaN | 0.0000 | 141.1921 | [] | turn_overshoot_progress_definition |
| 834 | slope_bumps | mountain_large | a4 | dry_hard | 0.8408 | 6.2428 | NaN | NaN | 0.0000 | 140.4639 | [] | turn_overshoot_progress_definition |
| 233 | mixed_random | standard_offroad | a2 | dry_hard | 0.8668 | 6.2080 | 1.8624 | 0.9312 | 0.1200 | 139.6802 | ['roll', 'pitch', 'slip', 'lift', 'bottom'] | possible_collision_rebound_or_bounce |
| 868 | bumps | urban_small | a4 | grass_soft | 0.4917 | 6.1672 | NaN | NaN | 0.0000 | 138.7617 | [] | turn_overshoot_progress_definition |
| 921 | slope_bumps | mountain_large | a4 | grass_soft | 0.5259 | 6.1050 | NaN | NaN | 0.0000 | 137.3619 | [] | turn_overshoot_progress_definition |
| 612 | flat | mountain_large | a3 | dry_hard | 0.7276 | 6.0790 | NaN | NaN | 0.0000 | 136.7781 | [] | turn_overshoot_progress_definition |
| 996 | uniform_slope | standard_offroad | a3 | grass_soft | 0.4153 | 6.0686 | NaN | NaN | 0.0000 | 136.5445 | [] | turn_overshoot_progress_definition |
| 696 | bumps | standard_offroad | a4 | wet_muddy | 0.2144 | 6.0408 | NaN | NaN | 0.0000 | 135.9170 | [] | turn_overshoot_progress_definition |
| 294 | mixed_random | standard_offroad | a1 | dry_hard | 0.7687 | 6.0321 | 1.8096 | 0.9048 | 0.1200 | 135.7222 | ['roll', 'pitch', 'slip', 'lift', 'bottom'] | possible_collision_rebound_or_bounce |
| 282 | slope_bumps | mountain_large | a3 | grass_soft | 0.5961 | 5.9722 | NaN | NaN | 0.0000 | 134.3751 | ['roll', 'pitch'] | turn_overshoot_progress_definition |
| 15 | lateral_slope | standard_offroad | a4 | grass_soft | 0.5010 | 5.8861 | NaN | NaN | 0.0000 | 132.4380 | [] | turn_overshoot_progress_definition |
| 641 | lateral_pits | urban_small | a4 | grass_soft | 0.4594 | 5.8141 | NaN | NaN | 0.0000 | 130.8174 | [] | turn_overshoot_progress_definition |
| 914 | lateral_slope | standard_offroad | a3 | grass_soft | 0.4837 | 5.7956 | NaN | NaN | 0.0000 | 130.4010 | ['pitch'] | turn_overshoot_progress_definition |
| 331 | flat | standard_offroad | a4 | dry_hard | 0.8580 | 5.7712 | NaN | NaN | 0.0000 | 129.8524 | [] | turn_overshoot_progress_definition |
| 341 | bumps | standard_offroad | a3 | dry_hard | 0.8272 | 5.7616 | NaN | NaN | 0.0000 | 129.6356 | [] | turn_overshoot_progress_definition |
| 454 | steps | mountain_large | a3 | grass_soft | 0.5606 | 5.5782 | NaN | NaN | 0.0000 | 125.5103 | [] | turn_overshoot_progress_definition |
| 677 | lateral_slope | standard_offroad | a3 | grass_soft | 0.4183 | 5.5426 | NaN | NaN | 0.0000 | 124.7083 | [] | turn_overshoot_progress_definition |
| 387 | pits | urban_small | a4 | grass_soft | 0.5510 | 5.5039 | NaN | NaN | 0.0000 | 123.8375 | [] | turn_overshoot_progress_definition |
| 446 | flat | standard_offroad | a3 | mixed | 0.5607 | 5.3381 | NaN | NaN | 0.0000 | 120.1083 | [] | turn_overshoot_progress_definition |
| 559 | uniform_slope | urban_small | a4 | dry_hard | 0.7512 | 5.3215 | NaN | NaN | 0.0000 | 119.7339 | ['roll', 'bottom'] | turn_overshoot_progress_definition |
| 247 | bumps | standard_offroad | a3 | wet_muddy | 0.2440 | 5.2589 | NaN | NaN | 0.0000 | 118.3241 | [] | turn_overshoot_progress_definition |
| 448 | uniform_slope | mountain_large | a4 | dry_hard | 0.8197 | 5.2118 | NaN | NaN | 0.0000 | 117.2645 | ['roll'] | turn_overshoot_progress_definition |
| 354 | steps | urban_small | a3 | dry_hard | 0.7703 | 5.1660 | NaN | NaN | 0.0000 | 116.2345 | [] | turn_overshoot_progress_definition |
| 674 | lateral_slope | urban_small | a4 | grass_soft | 0.4216 | 5.1125 | NaN | NaN | 0.0000 | 115.0311 | [] | turn_overshoot_progress_definition |
| 991 | lateral_pits | standard_offroad | a3 | dry_hard | 0.8603 | 5.0813 | NaN | NaN | 0.0000 | 114.3288 | [] | turn_overshoot_progress_definition |
| 205 | bumps | mountain_large | a4 | mixed | 0.4869 | 5.0306 | NaN | NaN | 0.0000 | 113.1874 | [] | turn_overshoot_progress_definition |
| 290 | waves | standard_offroad | a4 | grass_soft | 0.5085 | 5.0026 | NaN | NaN | 0.0000 | 112.5582 | [] | turn_overshoot_progress_definition |
| 30 | bumps | urban_small | a3 | grass_soft | 0.5241 | 5.0013 | NaN | NaN | 0.0000 | 112.5287 | [] | turn_overshoot_progress_definition |
| 344 | lateral_pits | standard_offroad | a4 | dry_hard | 0.7100 | 4.9418 | NaN | NaN | 0.0000 | 111.1897 | [] | turn_overshoot_progress_definition |
| 129 | waves | urban_small | a4 | grass_soft | 0.5157 | 4.9402 | NaN | NaN | 0.0000 | 111.1547 | [] | turn_overshoot_progress_definition |
| 339 | slope_bumps | mountain_large | a0 | wet_muddy | 0.3299 | 4.9281 | 1.4784 | 0.7392 | 0.1500 | 0.0000 | ['roll', 'pitch', 'slip', 'lift'] | downhill_or_low_mu_sliding |
| 773 | pits | standard_offroad | a4 | dry_hard | 0.8962 | 4.9099 | NaN | NaN | 0.0000 | 110.4737 | [] | turn_overshoot_progress_definition |
| 870 | lateral_slope | standard_offroad | a4 | grass_soft | 0.5346 | 4.8764 | NaN | NaN | 0.0000 | 109.7185 | ['roll'] | turn_overshoot_progress_definition |
| 382 | lateral_slope | mountain_large | a4 | mixed | 0.5817 | 4.6583 | NaN | NaN | 0.0000 | 104.8125 | [] | turn_overshoot_progress_definition |
| 69 | waves | mountain_large | a4 | dry_hard | 0.8202 | 4.5778 | NaN | NaN | 0.0000 | 103.0009 | ['pitch'] | turn_overshoot_progress_definition |
| 160 | lateral_pits | mountain_large | a4 | grass_soft | 0.5250 | 4.5167 | NaN | NaN | 0.0000 | 101.6251 | ['roll'] | turn_overshoot_progress_definition |
| 600 | lateral_slope | mountain_large | a4 | dry_hard | 0.7424 | 4.3801 | NaN | NaN | 0.0000 | 98.5517 | [] | turn_overshoot_progress_definition |
| 658 | lateral_pits | standard_offroad | a3 | grass_soft | 0.5618 | 4.2356 | NaN | NaN | 0.0000 | 95.3007 | [] | turn_overshoot_progress_definition |
| 136 | mixed_random | mountain_large | a4 | mixed | 0.2977 | 4.2060 | NaN | NaN | 0.0000 | 94.6360 | ['pitch'] | turn_overshoot_progress_definition |
| 413 | steps | standard_offroad | a4 | wet_muddy | 0.3575 | 4.1872 | NaN | NaN | 0.0000 | 94.2128 | [] | turn_overshoot_progress_definition |
| 704 | waves | standard_offroad | a3 | wet_muddy | 0.3686 | 4.0018 | NaN | NaN | 0.0000 | 90.0406 | [] | turn_overshoot_progress_definition |
| 236 | slope_bumps | mountain_large | a3 | dry_hard | 0.8467 | 3.9747 | NaN | NaN | 0.0000 | 89.4306 | [] | turn_overshoot_progress_definition |
| 31 | lateral_slope | standard_offroad | a3 | wet_muddy | 0.2118 | 3.9655 | NaN | NaN | 0.0000 | 89.2246 | ['pitch'] | turn_overshoot_progress_definition |
| 225 | flat | mountain_large | a4 | wet_muddy | 0.3709 | 3.8882 | NaN | NaN | 0.0000 | 87.4848 | [] | turn_overshoot_progress_definition |
| 345 | flat | urban_small | a4 | grass_soft | 0.4687 | 3.8840 | NaN | NaN | 0.0000 | 87.3890 | [] | turn_overshoot_progress_definition |
| 298 | mixed_random | mountain_large | a2 | mixed | 0.4701 | 3.8120 | 1.1436 | 0.5718 | 0.1200 | 85.7709 | ['roll', 'pitch', 'slip', 'lift'] | likely_valid_outlier |
| 800 | lateral_pits | mountain_large | a4 | grass_soft | 0.4437 | 3.7429 | NaN | NaN | 0.0000 | 84.2151 | [] | turn_overshoot_progress_definition |
| 706 | bumps | mountain_large | a4 | wet_muddy | 0.2049 | 3.5176 | NaN | NaN | 0.0000 | 79.1453 | [] | turn_overshoot_progress_definition |
| 720 | slope_bumps | standard_offroad | a4 | grass_soft | 0.4965 | 3.4812 | NaN | NaN | 0.0000 | 78.3260 | ['pitch', 'bottom'] | turn_overshoot_progress_definition |
| 529 | slope_bumps | mountain_large | a4 | wet_muddy | 0.3995 | 3.4760 | NaN | NaN | 0.0000 | 78.2108 | ['roll'] | turn_overshoot_progress_definition |
| 857 | lateral_pits | urban_small | a4 | wet_muddy | 0.2654 | 3.3817 | NaN | NaN | 0.0000 | 76.0886 | [] | turn_overshoot_progress_definition |
| 107 | uniform_slope | standard_offroad | a3 | dry_hard | 0.7018 | 3.3730 | NaN | NaN | 0.0000 | 75.8920 | [] | turn_overshoot_progress_definition |
| 302 | pits | standard_offroad | a4 | dry_hard | 0.8197 | 3.3622 | NaN | NaN | 0.0000 | 75.6505 | [] | turn_overshoot_progress_definition |
| 406 | uniform_slope | standard_offroad | a4 | mixed | 0.8022 | 3.3502 | NaN | NaN | 0.0000 | 75.3791 | [] | turn_overshoot_progress_definition |
| 426 | mixed_random | urban_small | a4 | wet_muddy | 0.2469 | 3.3111 | NaN | NaN | 0.0000 | 74.4991 | ['roll', 'pitch', 'bottom'] | turn_overshoot_progress_definition |
| 775 | bumps | standard_offroad | a3 | grass_soft | 0.4091 | 3.2793 | NaN | NaN | 0.0000 | 73.7837 | [] | turn_overshoot_progress_definition |
| 498 | slope_bumps | standard_offroad | a1 | grass_soft | 0.4460 | 3.1943 | 0.9583 | 0.4791 | 0.1200 | 71.8721 | ['roll', 'pitch', 'slip', 'lift'] | downhill_or_low_mu_sliding |
| 519 | pits | standard_offroad | a2 | dry_hard | 0.7518 | 3.0852 | 0.9256 | 0.4628 | 0.1200 | 69.4170 | ['roll', 'pitch', 'slip', 'lift'] | likely_valid_outlier |
| 716 | pits | standard_offroad | a3 | grass_soft | 0.4569 | 3.0593 | NaN | NaN | 0.0000 | 68.8332 | [] | turn_overshoot_progress_definition |
