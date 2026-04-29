## Pilot Dataset Statistics

| total_samples | valid_samples | hdf5_samples | invalid_attempts | invalid_sample_rate | total_fail_rate |
| --- | --- | --- | --- | --- | --- |
| 1000 | 1000 | 1000 | 4 | 0.0040 | 0.4080 |

### By Vehicle Fail Rate
| vehicle_type | count | fail_rate |
| --- | --- | --- |
| mountain_large | 359 | 0.3064 |
| standard_offroad | 363 | 0.3719 |
| urban_small | 278 | 0.5863 |

### By Terrain Fail Rate
| terrain_type | count | fail_rate |
| --- | --- | --- |
| bumps | 115 | 0.2174 |
| flat | 102 | 0.1667 |
| lateral_pits | 81 | 0.4321 |
| lateral_slope | 135 | 0.4074 |
| mixed_random | 66 | 0.8030 |
| pits | 106 | 0.2264 |
| slope_bumps | 75 | 0.6533 |
| steps | 89 | 0.4270 |
| uniform_slope | 112 | 0.5536 |
| waves | 119 | 0.4202 |

### Fail Reason Distribution
| fail_reason | count | per_fail_sample |
| --- | --- | --- |
| bottom | 160 | 0.3922 |
| lift | 94 | 0.2304 |
| pitch | 243 | 0.5956 |
| roll | 224 | 0.5490 |
| slip | 49 | 0.1201 |
| stuck | 64 | 0.1569 |

### q Metrics
| metric | mean | p50 | p95 |
| --- | --- | --- | --- |
| q_roll | 0.5397 | 0.5218 | 1.0000 |
| q_pitch | 0.5416 | 0.5116 | 1.0000 |
| q_slip | 0.2828 | 0.2464 | 0.7963 |
| q_lift | 0.0877 | 0.0400 | 0.3117 |

### Bottom / Stuck Distribution
| bottom_fail_rate | stuck_fail_rate | p_bottom_mean | p_bottom_p50 | p_bottom_p95 | p_stuck_mean | p_stuck_p50 | p_stuck_p95 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.1600 | 0.0640 | 0.0522 | 0.0000 | 0.4000 | 0.0640 | 0.0000 | 1.0000 |

### Risk Heatmap (Fail Rate)
| vehicle_type | bumps | flat | lateral_pits | lateral_slope | mixed_random | pits | slope_bumps | steps | uniform_slope | waves |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| mountain_large | 0.0769 | 0.0541 | 0.2903 | 0.3404 | 0.7200 | 0.1842 | 0.4444 | 0.1852 | 0.5556 | 0.3023 |
| standard_offroad | 0.2000 | 0.1176 | 0.4583 | 0.3600 | 0.8095 | 0.1951 | 0.7143 | 0.3784 | 0.4048 | 0.4146 |
| urban_small | 0.4194 | 0.3548 | 0.5769 | 0.5526 | 0.9000 | 0.3333 | 0.8500 | 0.7600 | 0.8000 | 0.5714 |

### Progress Distribution (Pass/Fail)
| group | count | mean | p50 | p95 | min | max |
| --- | --- | --- | --- | --- | --- | --- |
| pass | 592 | 1.5173 | 0.9848 | 5.5213 | 0.5135 | 15.9960 |
| fail | 408 | 1.2641 | 0.9694 | 4.7719 | 0.0014 | 15.5004 |

### Progress Histogram (Pass)
| bin | count | ratio |
| --- | --- | --- |
| [0.00, 0.25) | 0 | 0.0000 |
| [0.25, 0.50) | 0 | 0.0000 |
| [0.50, 0.75) | 12 | 0.0203 |
| [0.75, 1.00) | 366 | 0.6182 |
| [1.00, 1.25) | 125 | 0.2111 |
| [1.25, 1.50) | 8 | 0.0135 |
| [1.50, inf) | 81 | 0.1368 |

### Progress Histogram (Fail)
| bin | count | ratio |
| --- | --- | --- |
| [0.00, 0.25) | 52 | 0.1275 |
| [0.25, 0.50) | 31 | 0.0760 |
| [0.50, 0.75) | 29 | 0.0711 |
| [0.75, 1.00) | 176 | 0.4314 |
| [1.00, 1.25) | 65 | 0.1593 |
| [1.25, 1.50) | 9 | 0.0221 |
| [1.50, inf) | 46 | 0.1127 |

### Invalid Reasons (from manifest)
| invalid_reason | count |
| --- | --- |
| no_wheel_contact_samples | 3 |
| runtime_failure | 1 |

json_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/generated_pilot_v1_1000/pilot_stats_report.json`
md_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/generated_pilot_v1_1000/pilot_stats_report.md`
