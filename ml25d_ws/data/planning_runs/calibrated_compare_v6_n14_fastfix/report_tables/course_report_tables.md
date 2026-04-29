## 课程报告关键表格汇总

### 模型指标
| model_tag | auc_fail | recall_fail | f1_fail | mae_risk | infer_ms | proxy_plan_sr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| pso | 0.8684 | 0.7298 | 0.7328 | 0.1666 | 0.1033 | 0.7875 |
| baseline | 0.8658 | 0.7645 | 0.7389 | 0.1683 | 0.0912 | 0.9500 |

### 规划指标（方法汇总）
| method | model_tag | success_rate | path_length_mean | risk_max_mean | risk_avg_mean | expanded_mean | time_ms_mean |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline1_2p5d_astar | none | 1.0000 | 10.1331 | 0.0000 | 0.0000 | 258.2593 | 7.5521 |
| baseline2_manual_risk_weighted_astar | none | 0.5185 | 10.2927 | 0.4103 | 0.2358 | 7614.6190 | 6882.0483 |
| baseline3_ml_risk_weighted_astar | compare | 0.8519 | 10.2496 | 0.3071 | 0.1772 | 5903.2174 | 19773.2332 |
| baseline3_ml_risk_weighted_astar | main | 0.8519 | 10.2412 | 0.3064 | 0.1854 | 6063.2754 | 24137.9010 |
| proposed_ml_risk_constrained_astar | compare | 0.8272 | 10.1311 | 0.3316 | 0.2255 | 663.4776 | 836.8721 |
| proposed_ml_risk_constrained_astar | main | 0.8642 | 10.1402 | 0.3375 | 0.2330 | 632.2571 | 943.1503 |

### PSO 对比
`pso_compare_table.csv` 已生成。
