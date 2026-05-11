# Machine_Learning_25D 项目交付说明（最终版）

本 README 只保留当前可复现主链路：  
`Gazebo 数据生成 -> PSO 引导训练 -> 路径规划评测 -> 2D/3D 可视化 -> 报告表格`

## 1. 环境与入口

工作目录：

```bash
cd /home/crh/文档/Machine_Learning_25D/ml25d_ws
```

ROS 环境：

```bash
source /opt/ros/jazzy/setup.bash
```

Python 路径：

```bash
export PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH
```

## 2. 主流程脚本（正式）

仅以下脚本属于正式流程：

- `src/ml25d_dataset_generation/scripts/generate_dataset.py`（单目录生成入口，按需）
- `src/ml25d_dataset_generation/scripts/generate_dataset_shards.py`
- `src/ml25d_dataset_generation/scripts/generate_pilot_parallel.py`（分片生成器的并行执行内核）
- `src/ml25d_dataset_generation/scripts/train_risk_model.py`
- `src/ml25d_dataset_generation/scripts/run_planning_benchmark.py`
- `src/ml25d_dataset_generation/scripts/run_custom_scattered_benchmark.py`
- `src/ml25d_dataset_generation/scripts/render_planning_figures.py`
- `src/ml25d_dataset_generation/scripts/render_proposed_only_figures.py`
- `src/ml25d_dataset_generation/scripts/build_course_report_tables.py`

仓库内原有诊断/冒烟/专项审计脚本已清理，不再保留。

## 3. 数据生成（20k 分片）

已使用命令（20 shards × 1000）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/crh/文档/Machine_Learning_25D/ml25d_ws
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/generate_dataset_shards.py \
  --output-root data/generated_dataset_v1_20k_n6_comp \
  --num-shards 20 \
  --samples-per-shard 1000 \
  --workers 20 \
  --backend ros_gz \
  --disable-balance \
  --base-domain-start 220 \
  --seed-start 20260510 \
  --run-tag-prefix datasetv1 \
  --terrain-compensation \
  --terrain-comp-strength 1.8 \
  --terrain-comp-warmup 40 \
  --terrain-comp-min-mult 0.30 \
  --terrain-comp-max-mult 5.00 \
  --result-queue-mult 16 \
  --worker-startup-stagger-sec 0.4 \
  --ros-startup-timeout-sec 40 \
  --ros-service-timeout-sec 14 \
  --resume
```

产物目录：

- `data/generated_dataset_v1_20k_n6_comp/`
- `data/generated_dataset_v1_20k_n6_comp/shards_summary.json`

## 4. 模型训练（PSO + baseline）

已使用命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/crh/文档/Machine_Learning_25D/ml25d_ws
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH ../.venv/bin/python \
  src/ml25d_dataset_generation/scripts/train_risk_model.py \
  --pattern "data/generated_dataset_v1_20k_n6_comp/shard_*/samples_batch_*.h5" \
  --output-dir data/training_runs/cnn_pso_dataset_v1_20k \
  --mode both \
  --device cuda \
  --batch-size 128 \
  --loader-workers 6 \
  --pso-particles 8 \
  --pso-iters 6 \
  --pso-epochs 6 \
  --final-epochs 90 \
  --final-min-epochs 25 \
  --final-patience 12 \
  --final-min-delta 5e-4 \
  --final-lr-patience 4 \
  --final-lr-factor 0.5 \
  --max-pso-train-samples 4096 \
  --proxy-tasks-pso 32 \
  --proxy-tasks-final 80
```

关键产物：

- `data/training_runs/cnn_pso_dataset_v1_20k/pso/best_model_calibrated.pt`
- `data/training_runs/cnn_pso_dataset_v1_20k/baseline/best_model_calibrated.pt`
- `data/training_runs/cnn_pso_dataset_v1_20k/compare_report.json`

## 5. 路径规划与可视化

### 5.1 标准地形评测（课程表格来源）

目录：

- `data/planning_runs/calibrated_compare_v6_n14_fastfix/`

关键文件：

- `planning_metrics.csv`
- `planning_paths.jsonl`
- `planning_summary.json`
- `report_tables/*.csv`
- `report_tables/course_report_tables.md`

### 5.2 自定义散布坑包场景（最终图）

目录：

- `data/planning_runs/custom_random_scattered_obstacles_v2_guard_tight/`

关键文件：

- `planning_metrics.csv`
- `planning_paths.jsonl`
- `planning_summary.json`
- `scenes/*.npz`
- `figures_compare_2d_final/`（2D方法对比 + 三车型对比）
- `figures_compare_3d_final/`（3D方法对比 + 三车型对比）
- `figures_proposed_only_2d_final/`（Proposed 单独图）
- `figures_proposed_only_3d_final/`（Proposed 单独图）

