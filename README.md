# 面向不同车型的2.5D野外地形风险感知路径规划

**基于机器学习与搜索算法的双向嵌套设计**

东北大学自动化强基班 《人工智能》与《机器学习与模式识别》综合课程作业

---

## 项目概述

本方案构建了 **PSO 引导的内层模型训练 → ML 风险模型 → 硬约束 A\* 外层规划** 的双向嵌套协同架构。内层粒子群优化对 CNN+MLP 多任务风险预测模型的网络结构、损失权重及规划阈值进行联合寻优；外层将优化后的模型作为启发函数，通过 **ML+Geometric Guard** 融合机制驱动力约束 A\* 搜索，实现不同车型在复杂 2.5D 地形中的自适应低风险路径规划。

### 核心自主实现模块

| 模块 | 文件 | 说明 |
|:---|:---|:---|
| 风险约束 A\*（外层搜索） | `python/ml25d_dataset_generation/risk_astar.py` | 16向状态空间、多维硬约束剪枝、Pareto 支配路径选择、ML+Guard 融合 |
| 多任务风险模型（内层 ML） | `python/ml25d_dataset_generation/risk_model.py` | CNN+MLP 双分支架构，6通道局部特征输入，7维物理风险输出 |
| PSO 训练引擎（内层搜索） | `python/ml25d_dataset_generation/pso_training.py` | 20维粒子编码，复合适应度（AUC+Recall+MAE+PlanSR），代理规划反馈 |
| 物理仿真数据生成 | `scripts/generate_dataset_shards.py` | Gazebo+ros_gz 并行数据生成，20 shard × parallel workers |
| 特征构建器 | `python/ml25d_dataset_generation/feature_builder.py` | 31×31×6 局部特征图，车身掩码与扫掠掩码 |
| 规划评测与可视化 | `scripts/run_planning_benchmark.py`, `scripts/render_planning_figures.py` | 多基线消融实验，2D/3D 路径对比图 |

---

## 1. 环境要求

| 组件 | 版本/说明 |
|:---|:---|
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| Gazebo Sim | ≥ 8.0（Bullet-Featherstone 物理引擎） |
| Python | ≥ 3.12 |
| CUDA | 推荐 12.x / 13.x（模型训练） |
| GPU | NVIDIA RTX 4060 或更高（训练约需 8 GB 显存） |

### 安装依赖

```bash
# ROS 2 Jazzy + Gazebo
# 参考 https://docs.ros.org/en/jazzy/Installation.html
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

# Python 虚拟环境
python3 -m venv .venv
source .venv/bin/activate
pip install torch numpy h5py pyyaml matplotlib pyvista scipy

# 构建 colcon 工作空间
cd ml25d_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ml25d_dataset_generation
source install/setup.bash
```

### 环境变量（每次新终端）

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash                    # colcon workspace
export PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH
```

---

## 2. 数据生成（Gazebo 物理仿真）

生成 20,000 条局部动作交互样本（20 shards × 1000 samples，20 workers 并行）：

```bash
cd ml25d_ws

PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/generate_dataset_shards.py \
  --output-root data/generated_dataset_v1_20k \
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

**产物**：`data/generated_dataset_v1_20k/shard_*/samples_batch_*.h5`

> **注意**：完整数据生成需大量计算资源（约 20 进程并行，每样本含 Gazebo 物理仿真）。数据集不在 Git 仓库中（已 gitignore），需本地生成。

---

## 3. 模型训练（PSO + Baseline）

在内层，PSO 对 20 维超参数进行联合寻优，代理规划模块在小型验证地图上实测 A\* 成功率作为适应度反馈：

```bash
cd ml25d_ws

PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH .venv/bin/python \
  src/ml25d_dataset_generation/scripts/train_risk_model.py \
  --pattern "data/generated_dataset_v1_20k/shard_*/samples_batch_*.h5" \
  --output-dir data/training_runs/cnn_pso_v1 \
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

**关键产物**：
- `data/training_runs/cnn_pso_v1/pso/best_model_calibrated.pt` — PSO 最优模型
- `data/training_runs/cnn_pso_v1/baseline/best_model_calibrated.pt` — 对照基线模型
- `data/training_runs/cnn_pso_v1/compare_report.json` — 对比评估报告

---

## 4. 路径规划评测

### 4.1 标准地形消融实验

在 27 张未知测试地图上对 3 种车型（共 81 个规划任务）运行 5 种方法对比：

```bash
cd ml25d_ws

PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/run_planning_benchmark.py \
  --dataset-pattern "data/generated_dataset_v1_20k/shard_*/samples_batch_*.h5" \
  --model-root data/training_runs/cnn_pso_v1 \
  --output-dir data/planning_runs/benchmark_v1 \
  --num-scenes 27 \
  --workers 6
```

### 4.2 自定义散落障碍场景

```bash
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/run_custom_scattered_benchmark.py \
  --model-root data/training_runs/cnn_pso_v1 \
  --output-dir data/planning_runs/custom_scattered_v1 \
  --num-scenes 12
```

### 4.3 可视化

```bash
# 多方法对比图（2D + 3D）
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/render_planning_figures.py \
  --results-dir data/planning_runs/benchmark_v1 \
  --output-dir data/planning_runs/benchmark_v1/figures

# Proposed 单独图
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/render_proposed_only_figures.py \
  --results-dir data/planning_runs/custom_scattered_v1 \
  --output-dir data/planning_runs/custom_scattered_v1/figures

# 生成课程报告表格
PYTHONPATH=src/ml25d_dataset_generation/python:$PYTHONPATH python3 \
  src/ml25d_dataset_generation/scripts/build_course_report_tables.py \
  --results-dir data/planning_runs/benchmark_v1
```

---

## 5. 算法架构

### 外层：风险约束 A\*

状态空间 $(i, j, k)$ 包含 16 个离散航向。A\* 扩展节点时将候选边 $(s, a)$ 下发给内层 ML 模型，获取 7 维物理风险预测。通过 ML+Geometric Guard 融合机制：

$$r_{plan} = (1-\lambda_g) \cdot r_{ml} + \lambda_g \cdot \max(r_{ml}, r_{manual})$$

确保 ML 分布外失效时仍有无偏几何保底。三条硬约束（$T_{edge}, T_{max}, T_{avg}$）将风险从软代价变为可行域边界，配合 Pareto 支配进行多标签路径选择。

### 内层：PSO 超参数联合寻优

20 维粒子编码涵盖网络结构（CNN 通道数、MLP 隐藏层）、损失权重（多任务平衡）、规划阈值。复合适应度函数：

$$Fitness = AUC_{fail} + \alpha_1 \cdot Recall_{fail} - \alpha_2 \cdot MAE_{risk} + \beta_1 \cdot PlanSR$$

其中 **PlanSR** 为代理 A\* 在验证地图上实测的规划成功率，实现了"搜索结果反馈至模型结构"的不可微闭环优化。

### 数据流

```
Gazebo 物理仿真 → 20k 交互数据集 → [内层] PSO 寻优 CNN+MLP 模型
                                              ↓
                    [外层] 风险约束 A* ← 最优模型 + Guard 保底
```

---

## 6. 预期结果

消融实验在 81 项测试任务上的结果：

| 方法 | 成功率 | 最大风险均值 |
|:---|:---:|:---:|
| 原始 2.5D A\* (Gu & Cao 2011) | 0.0% | 0.885 |
| 手工代价 A\* | 22.2% | 0.542 |
| ML 风险 A\*（软惩罚） | 38.5% | 0.410 |
| 纯几何规则硬约束 A\* | 40.7% | 0.408 |
| **本方案 ML+Guard** | **55.6%** | **0.358** |

---

## 7. 仓库结构

```
.
├── 报告/                    # LaTeX 报告源码与 PDF
├── ml25d_ws/                # ROS 2 + colcon 工作空间
│   └── src/ml25d_dataset_generation/
│       ├── config/          # YAML 配置文件（地形/车辆/动作/标签阈值）
│       ├── launch/          # ROS 2 launch 文件
│       ├── worlds/          # Gazebo SDF 世界文件
│       ├── python/          # 核心算法库
│       │   └── ml25d_dataset_generation/
│       │       ├── risk_astar.py          # 外层风险约束 A*
│       │       ├── risk_model.py          # 内层 CNN+MLP 模型
│       │       ├── pso_training.py        # PSO 训练引擎
│       │       ├── feature_builder.py     # 特征工程
│       │       ├── gazebo_runner.py       # Gazebo 仿真控制
│       │       ├── label_extractor.py     # 物理标签提取
│       │       └── ...
│       └── scripts/         # 入口脚本
│           ├── generate_dataset_shards.py
│           ├── train_risk_model.py
│           ├── run_planning_benchmark.py
│           ├── render_planning_figures.py
│           └── ...
└── README.md
```

---

## 参考文献

- J. Gu and Q. Cao. *Path planning for mobile robot in a 2.5-dimensional grid-based map.* Industrial Robot, 38(2):126–134, 2011. DOI: 10.1108/01439911111122815.
