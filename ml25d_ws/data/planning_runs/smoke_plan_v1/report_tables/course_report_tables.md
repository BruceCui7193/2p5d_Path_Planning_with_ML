## 课程报告关键表格汇总

### 模型指标
| model_tag | auc_fail | recall_fail | f1_fail | mae_risk | infer_ms | proxy_plan_sr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| pso | 0.9590 | 0.9493 | 0.8530 | 0.0734 | 0.0392 | 1.0000 |
| baseline | 0.7639 | 1.0000 | 0.6429 | 0.4145 | 0.0847 | 0.0000 |

### 规划指标（方法汇总）
| method | model_tag | success_rate | path_length_mean | risk_max_mean | risk_avg_mean | expanded_mean | time_ms_mean |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline1_2p5d_astar | none | 1.0000 | 4.0000 | 0.0000 | 0.0000 | 10.0000 | 0.6039 |
| baseline2_manual_risk_weighted_astar | none | 1.0000 | 4.0000 | 0.0000 | 0.0000 | 10.0000 | 481.5666 |
| baseline3_ml_risk_weighted_astar | main | 1.0000 | 4.0000 | 0.3995 | 0.3995 | 356.0000 | 16667.6906 |
| proposed_ml_risk_constrained_astar | main | 1.0000 | 4.0000 | 0.3995 | 0.3995 | 10.0000 | 467.0604 |

### PSO 对比
`pso_compare_table.csv` 已生成。
