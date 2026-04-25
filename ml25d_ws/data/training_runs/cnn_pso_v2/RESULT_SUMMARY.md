# CNN+PSO v2 Training Summary

Dataset: `data/generated_hq_v1/samples_batch_*.h5`

Main fix in v2: `p_bottom` is trained as a continuous soft probability target instead of being thresholded into a binary label.

## Test Metrics

| Run | Threshold policy | Threshold | Precision | Recall | F1 | Risk MAE | Proxy A* success |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `cnn_pso_v1` | default | 0.500 | 0.9557 | 0.6959 | 0.8053 | 0.0871 | 1.000 |
| `cnn_pso_v2` | default | 0.500 | 0.7744 | 0.9493 | 0.8530 | 0.0734 | 1.000 |
| `cnn_pso_v2` | val-calibrated, target recall 0.95 | 0.740 | 0.8700 | 0.8940 | 0.8818 | 0.0734 | 1.000 |

Recommended reporting point for the current dataset: `cnn_pso_v2` with fail threshold `0.74`, because it keeps recall high while avoiding the excessive false positives of the default `0.5` threshold.

For a more conservative safety-first setting, use the F2-oriented threshold from `calibration_report_target095.json`: threshold `0.67`, precision `0.8333`, recall `0.9217`, F1 `0.8753`.
