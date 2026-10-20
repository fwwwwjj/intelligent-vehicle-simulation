# 交接说明：基线对比与消融实验（未完成）

> 状态：**代码已实现、单元/闭环冒烟通过；全量实验和文档尚未完成**。
> 交接时间：2026-10-20。原机器因长时间仿真发热，任务暂停。
> 当前状态：本轮基线/消融代码已提交（commit 见 `git log -1`）。全量实验尚未跑。

---

## 1. 要回应的问题

评审意见：**“实验结果无法区分算法优劣（缺乏对比度）：系统仅包含一套决策规划方案（FSM + Lattice + LQR），没有 baseline 对比或 ablation study。”**

结论：**问题真实存在**。当前主结果 `results/benchmark*.csv/json` 只有 `fsm + lattice + lqr + 运动学` 一套配置；Game/EM/MPC 被标为 experimental 并从主结果排除，因此没有可信的算法对比或消融。

本轮已实现的解决方式：新增**经典基线栈** + **单组件消融矩阵** + 配对统计报告，不修改原主结果口径。

---

## 2. 已完成的工作（已提交）

### 2.1 新增基线算法
- `src/decision/idm_decision.py`：`IDMDecisionMaker`
  - IDM 自适应巡航；`speed_command_horizon=1.0`；
  - 经典 TTC/AEB 兜底（`a_idm < -3.0` 或 `ttc < 1.5` 时 `target_speed=0`）；
  - **不做换道**，所以 highway_merge 必然失败（这是有意的能力对比）。
- `src/planning/centerline_planner.py`：`CenterlinePlanner`
  - 只跟踪目标车道中心线，不采样、不做代价评估、不检查障碍物（无优化基线）；
  - 横向五次多项式初速度 `l_d0 = clip(0.8*(target_l-l), ±1.2)`，修复了递推规划横向收敛停滞。
- `src/control/pure_pursuit_controller.py`：`PurePursuitController`
  - 几何纯跟踪 + 纵向 PID，经典控制基线。

### 2.2 集成与消融开关
- `src/runner.py`：`build_engine` 现在支持
  - decision：`fsm` / `game` / `idm` / `fsm_no_lane_change`
  - planner：`lattice` / `em` / `centerline`
  - controller：`lqr` / `mpc` / `pure_pursuit`
- `src/decision/rule_based_fsm.py`：新增 `enable_lane_change`，用于“去掉战术换道”的消融。
- `src/evaluation/benchmark.py`：resolved planner/controller 名称支持新增类。
- `scripts/run_benchmark.py`、`scripts/run_simulation.py`：CLI choices 已同步。
- `.gitignore`：允许提交 `results/contrast/`（含 CSV）。

### 2.3 对比实验与统计
- `src/evaluation/contrast.py`：Condition 矩阵、Wilson 区间、exact McNemar、paired bootstrap、Wilcoxon、Cliff's delta、Holm-Bonferroni、Markdown 报告生成。
- `scripts/run_contrast.py`：按 condition 逐个跑 `BenchmarkSuite`，输出
  `contrast.json` / `contrast_runs.csv` / `contrast_summary.csv` / `contrast_report.md`。
- `scripts/analyze_contrast.py`：从 JSON 重算统计并生成 3 张图（需 matplotlib，已可用）。
- `tests/test_baseline_algorithms.py`（15 tests）、`tests/test_contrast.py`（37 tests）。

### 2.4 已验证
- 全量测试：**148 tests OK**（约 10 s，单线程 BLAS）。
- 11 个 condition × 3 场景 × 1 seed 冒烟通过，输出在 `/tmp/siac_contrast_all_smoke`（临时目录，未提交）。
- 关键冒烟结果（seed 42，20 s）：
  - proposed：3/3 成功；cut-in min gap 5.36 m，emergency min gap 13.99 m，jerk ≈1.80–2.01。
  - baseline IDM+centerline+PP：cut-in/emergency 成功，**highway_merge 失败**（无换道意图）。
  - EM planner：3/3 成功；cut-in gap 9.88 m、emergency gap 18.60 m，但规划时延 ≈6–13.5 ms（Lattice ≈1–1.6 ms）。
  - Game：highway_merge 失败（汇入死锁），cut-in/emergency 成功。
  - MPC：3/3 失败（cut-in/emergency 碰撞，min gap 0）。
  - 2-DOF 线性 / Pacejka：highway_merge、cut-in 成功；**emergency_avoidance 失败**（min gap 3.20 m，最终速度未 ≤0.5）。
  - Pure pursuit：3/3 成功，但 jerk 更高（highway_merge 2.73 vs 2.01）。
  - Centerline planner：3/3 成功（修复后），时延 ≈0.9 ms，但无避障能力。
  - no-lane-change：仅 highway_merge 失败。

> 重要：以上只是 1 seed 冒烟，**不能写进论文/答辩**。必须跑 30 seeds 并用正式脚本生成结果。

---

## 3. 还没做的工作

1. **确认代码已提交**（本轮代码已在最新 commit 中；全量实验要求 `git_dirty=false`）。
2. 跑全量对比实验：11 conditions × 3 scenarios × 30 seeds（42–71），20 s。
3. 运行分析脚本，检查结果和负结果是否合理。
4. 写文档并更新：
   - 新增 `docs/11_baseline_and_ablation.md`（条件矩阵、结果表、配对统计、负结果、结论）；
   - 更新 `README.md`、`docs/03_results_and_demo.md`、`docs/07_scope_and_future_work.md`、`docs/08_paper_draft.md`、`CHANGELOG_AI.md`；
   - 新增 `results/contrast/README.md`（provenance、复现命令、产物说明）。
5. 独立复核：测试、provenance、文档数字与 JSON/CSV 一致；不要夸大统计显著性。

---

## 4. 全量实验命令（低功耗，务必单线程）

```bash
cd <repo-root>

# 1) 先确认测试全绿
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B -m unittest discover tests

# 2) 全量对比（11 × 3 × 30 = 990 runs，预计 40–60 min）
mkdir -p results/contrast
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nohup nice -n 19 python3 -B scripts/run_contrast.py \
    --scenarios highway_merge cut_in emergency_avoidance \
    --seeds 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 \
    --duration 20.0 --allow-experimental \
    --output-dir results/contrast \
    > /tmp/siac_contrast_full.log 2>&1 &

# 3) 分析 + 图表
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B scripts/analyze_contrast.py \
    --input results/contrast/contrast.json --output-dir results/contrast
```

> 如果目标机器也很热：先跑 5–10 seeds 出初步表，最后补到 30；不要并行跑多个 benchmark。
> 运行前必须先提交代码，否则 `contrast.json.metadata.git_dirty=true`。

---

## 5. 条件矩阵（`src/evaluation/contrast.py`）

| condition | family | 替换的组件 | 预期作用 |
| :--- | :--- | :--- | :--- |
| `proposed_fsm_lattice_lqr` | proposed | 完整栈 | 参照 |
| `baseline_idm_centerline_pure_pursuit` | baseline | 决策/规划/控制全换经典方法 | 经典基线 |
| `ablation_decision_game` | ablation | FSM→Game | 交互决策对比（当前汇入失败） |
| `ablation_decision_idm` | ablation | FSM→IDM ACC | 去掉战术决策 |
| `ablation_decision_no_lane_change` | ablation | 关闭 FSM 换道 | 量化换道决策贡献 |
| `ablation_planner_em` | ablation | Lattice→EM/QP | 优化规划对比 |
| `ablation_planner_centerline` | ablation | Lattice→中心线跟踪 | 去掉搜索/优化 |
| `ablation_controller_pure_pursuit` | ablation | LQR→纯跟踪 | 经典控制对比 |
| `ablation_controller_mpc` | ablation | LQR→LTV-MPC | 先进控制对比（当前会撞） |
| `ablation_model_dynamic_2dof` | ablation | 运动学→2-DOF 线性轮胎 | 模型保真度消融 |
| `ablation_model_pacejka` | ablation | 2-DOF 线性→Pacejka | 轮胎模型消融 |

---

## 6. 写作口径（必须诚实）

- **不要**写“本文算法全面优于所有基线”。实际结果是：
  - 经典基线/IDM 在 cut-in、emergency 上也能成功，但**没有换道能力**，highway_merge 失败；
  - EM 在安全间距/舒适性上更强，但规划时延约 5–10 倍；
  - LQR 比 pure pursuit 更舒适（jerk 更低）；
  - Game 与 MPC 当前实现是**负结果**，必须如实报告失败原因；
  - 2-DOF/Pacejka 在 emergency 停车判定上失败，说明运动学简化对停车精度有影响。
- 推荐结论：**proposed 是“任务成功 + 舒适性 + 实时性”的折中，不是每个指标都最优**；这正是需要对比/消融才能得到的结论。
- 统计注意：当前 30 seeds 只扰动传感器噪声，场景参数是确定的，组内方差很小；**不要**把“p<0.05”写成对场景分布统计显著。报告效应量、配对差值和置信区间，并在局限中说明 seeds 未做场景参数扰动。
- 原主结果 `results/benchmark*.csv/json` 不要改；对比实验放 `results/contrast/`。

---

## 7. 已提交的改动

已修改：
- `.gitignore`
- `scripts/run_benchmark.py`
- `scripts/run_simulation.py`
- `src/decision/rule_based_fsm.py`
- `src/evaluation/benchmark.py`
- `src/runner.py`

新增未跟踪：
- `src/decision/idm_decision.py`
- `src/planning/centerline_planner.py`
- `src/control/pure_pursuit_controller.py`
- `src/evaluation/contrast.py`
- `scripts/run_contrast.py`
- `scripts/analyze_contrast.py`
- `tests/test_baseline_algorithms.py`
- `tests/test_contrast.py`
- 本文件 `HANDOFF_BASELINE_ABLATION.md`

**换电脑前**：本 commit 已推到 GitHub 的 `main`；在另一台电脑 `git clone`/`git pull` 即可。若推送失败，则整目录拷贝（含 `.git`）。
