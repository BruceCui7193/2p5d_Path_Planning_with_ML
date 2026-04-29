# pilot_v2_1000 统计报告

## 1) 样本规模
- 总样本数(target): 1000
- 有效样本数(valid): 1000
- invalid_sample_rate: 0.0000
- 总 fail_rate: 0.4450

## 2) 按车型 fail_rate
| vehicle | n | fail_rate |
|---|---:|---:|
| mountain_large | 103 | 0.0680 |
| standard_offroad | 332 | 0.2169 |
| urban_small | 565 | 0.6478 |

## 3) 按地形 fail_rate
| terrain | n | fail_rate |
|---|---:|---:|
| bumps | 139 | 0.4604 |
| flat | 196 | 0.1531 |
| lateral_pits | 64 | 0.6719 |
| lateral_slope | 112 | 0.5357 |
| mixed_random | 26 | 0.8846 |
| pits | 127 | 0.3937 |
| slope_bumps | 55 | 0.8545 |
| steps | 121 | 0.3058 |
| uniform_slope | 85 | 0.5882 |
| waves | 75 | 0.5467 |

## 4) fail_reason 分布
| reason | count |
|---|---:|
| pitch | 367 |
| roll | 258 |
| lift | 149 |
| bottom | 118 |
| slip | 81 |
| stuck | 71 |

## 5) q_roll/q_pitch/q_slip/q_lift
| metric | mean | p50 | p95 |
|---|---:|---:|---:|
| q_roll | 0.4643 | 0.3770 | 1.0000 |
| q_pitch | 0.5362 | 0.4583 | 1.0000 |
| q_slip | 0.3244 | 0.2833 | 1.0000 |
| q_lift | 0.0992 | 0.0300 | 0.3750 |

## 6) bottom/stuck 分布
- bottom_fail_rate: 0.1180
- stuck_fail_rate: 0.0710
- both_rate: 0.0090
- none_rate: 0.8200

## 7) flat 样本 fail_rate
- flat n=196, fail_rate=0.1531

## 8) 车型×地形风险热力表（fail_rate）
| vehicle | bumps | flat | lateral_pits | lateral_slope | mixed_random | pits | slope_bumps | steps | uniform_slope | waves |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mountain_large | 0.182 | 0.030 | NA | NA | NA | 0.273 | NA | 0.000 | 0.000 | 0.000 |
| standard_offroad | 0.262 | 0.033 | 0.286 | 0.275 | 0.500 | 0.200 | 0.500 | 0.196 | 0.200 | 0.500 |
| urban_small | 0.687 | 0.371 | 0.780 | 0.681 | 0.917 | 0.590 | 0.882 | 0.450 | 0.776 | 0.596 |

## 9) pass/fail progress 分布
| split | n | mean | p50 | p95 |
|---|---:|---:|---:|---:|
| pass | 555 | 0.9651 | 0.9800 | 1.0899 |
| fail | 445 | 0.8449 | 0.9685 | 1.8486 |