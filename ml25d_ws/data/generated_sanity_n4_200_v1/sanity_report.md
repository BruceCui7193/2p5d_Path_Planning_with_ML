## Pilot Dataset Statistics

| total_samples | valid_samples | hdf5_samples | invalid_attempts | invalid_sample_rate | total_fail_rate |
| --- | --- | --- | --- | --- | --- |
| 200 | 200 | 200 | 0 | 0.0000 | 0.4600 |

### By Vehicle Fail Rate
| vehicle_type | count | fail_rate |
| --- | --- | --- |
| mountain_large | 13 | 0.1538 |
| standard_offroad | 49 | 0.2653 |
| urban_small | 138 | 0.5580 |

### By Terrain Fail Rate
| terrain_type | count | fail_rate |
| --- | --- | --- |
| bumps | 23 | 0.4348 |
| flat | 28 | 0.2143 |
| lateral_pits | 15 | 0.6667 |
| lateral_slope | 32 | 0.5938 |
| mixed_random | 13 | 0.9231 |
| pits | 18 | 0.3889 |
| slope_bumps | 12 | 0.5833 |
| steps | 23 | 0.1739 |
| uniform_slope | 19 | 0.4737 |
| waves | 17 | 0.4706 |

### Fail Reason Distribution
| fail_reason | count | per_fail_sample |
| --- | --- | --- |
| bottom | 35 | 0.3804 |
| lift | 27 | 0.2935 |
| pitch | 59 | 0.6413 |
| roll | 48 | 0.5217 |
| slip | 17 | 0.1848 |
| stuck | 7 | 0.0761 |

### q Metrics
| metric | mean | p50 | p95 |
| --- | --- | --- | --- |
| q_roll | 0.5090 | 0.5158 | 1.0000 |
| q_pitch | 0.5633 | 0.6122 | 1.0000 |
| q_slip | 0.3049 | 0.2752 | 1.0000 |
| q_lift | 0.0952 | 0.0287 | 0.3588 |

### Bottom / Stuck Distribution
| bottom_fail_rate | stuck_fail_rate | p_bottom_mean | p_bottom_p50 | p_bottom_p95 | p_stuck_mean | p_stuck_p50 | p_stuck_p95 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.1750 | 0.0350 | 0.0495 | 0.0000 | 0.3120 | 0.0350 | 0.0000 | 0.0000 |

### Risk Heatmap (Fail Rate)
| vehicle_type | bumps | flat | lateral_pits | lateral_slope | mixed_random | pits | slope_bumps | steps | uniform_slope | waves |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| mountain_large | 0.3333 | 0.0000 | NaN | NaN | NaN | 0.0000 | NaN | 1.0000 | NaN | 0.0000 |
| standard_offroad | 0.5000 | 0.1429 | 0.2000 | 0.3333 | NaN | 0.2500 | 1.0000 | 0.1111 | 0.0000 | 0.2857 |
| urban_small | 0.4167 | 0.3571 | 0.9000 | 0.6538 | 0.9231 | 0.4615 | 0.5455 | 0.1538 | 0.5294 | 0.6667 |

### Translation Progress Distribution (Pass/Fail)
| group | count | mean | p50 | p95 | min | max |
| --- | --- | --- | --- | --- | --- | --- |
| pass | 108 | 0.9599 | 0.9802 | 1.0861 | 0.7304 | 1.2927 |
| fail | 92 | 0.9277 | 0.9728 | 1.4384 | 0.0440 | 5.1630 |

### Translation Progress Histogram (Pass)
| bin | count | ratio |
| --- | --- | --- |
| [0.00, 0.25) | 0 | 0.0000 |
| [0.25, 0.50) | 0 | 0.0000 |
| [0.50, 0.75) | 2 | 0.0185 |
| [0.75, 1.00) | 92 | 0.8519 |
| [1.00, 1.25) | 13 | 0.1204 |
| [1.25, 1.50) | 1 | 0.0093 |
| [1.50, inf) | 0 | 0.0000 |

### Translation Progress Histogram (Fail)
| bin | count | ratio |
| --- | --- | --- |
| [0.00, 0.25) | 5 | 0.0543 |
| [0.25, 0.50) | 8 | 0.0870 |
| [0.50, 0.75) | 8 | 0.0870 |
| [0.75, 1.00) | 54 | 0.5870 |
| [1.00, 1.25) | 11 | 0.1196 |
| [1.25, 1.50) | 2 | 0.0217 |
| [1.50, inf) | 4 | 0.0435 |

### Angular Progress Distribution (Pass/Fail)
| group | count | mean | p50 | p95 | min | max |
| --- | --- | --- | --- | --- | --- | --- |
| pass | 0 | NaN | NaN | NaN | NaN | NaN |
| fail | 0 | NaN | NaN | NaN | NaN | NaN |

### Angular Progress Histogram (Pass)
| bin | count | ratio |
| --- | --- | --- |

### Angular Progress Histogram (Fail)
| bin | count | ratio |
| --- | --- | --- |

### Translation Drift Distribution (Pass/Fail)
| group | count | mean | p50 | p95 | min | max |
| --- | --- | --- | --- | --- | --- | --- |
| pass | 108 | 0.2880 | 0.2941 | 0.3258 | 0.2191 | 0.3878 |
| fail | 92 | 0.2783 | 0.2918 | 0.4315 | 0.0132 | 1.5489 |

### Invalid Reasons (from manifest)
| invalid_reason | count |
| --- | --- |

json_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/generated_sanity_n4_200_v1/sanity_report.json`
md_path: `/home/crh/文档/Machine_Learning_25D/ml25d_ws/data/generated_sanity_n4_200_v1/sanity_report.md`
