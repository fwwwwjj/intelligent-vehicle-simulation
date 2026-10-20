# 30-seed 基线与消融正式产物

本目录是独立补充实验；原 `results/benchmark*.csv/json` 保持字节/版本库blob不变。
正式数据由脚本运行生成，没有使用交接文档中的单seed冒烟，也没有修改算法使proposed获胜。
结果说明及完整表见 [docs/11](../../docs/11_baseline_and_ablation.md)。

## 来源与实验规模

- 仿真源提交：`e0d7643a0941a651fb2ec8ffa712c77f0f3d26ff`
  (`feat(contrast): add baseline/ablation experiment suite`)。
- 11个条件，`highway_merge` / `cut_in` / `emergency_avoidance`，seeds42–71，
  每次20.0 s、dt0.05 s、400步，共990次、33组。`duration_s=19.95`是最后一帧的时间戳。
- `contrast.json.metadata.git_dirty=false`按原脚本排除results目录后采集；算法与配置在整个
  仿真期间保持干净。之后的报告/文档交付提交与仿真源提交不同，不能改写元数据使它指向交付提交。
- `scenario_sha256`记录实际YAML文件指纹；`contrast_parameters.json`另外记录全局默认配置
  SHA-256、装配后的参数、控制器构造默认值。`environment.json`保存运行前环境、单线程验证、
  原主结果SHA-256和工作树状态。`validation.json`保存组合、CSV、统计及原主结果blob校验。
- 执行环境为Windows11、Intel i7-12700H、Python3.13.1、NumPy2.2.2、SciPy1.15.1、
  Matplotlib3.10.0、PyYAML6.0.3；单进程串行运行、Windows空闲优先级，四项BLAS线程变量为1。
  这些库高于原requirements上限，本次148项测试通过；没有冒充原Python3.9/Mac环境。
  PyYAML来自工作树外的隔离依赖目录，不修改全局Python环境或requirements。

## 产物说明

| 文件 | 内容 |
| :--- | :--- |
| `contrast.json` | 原始990次KPI、条件矩阵、仿真来源、33组汇总、相对proposed的合并配对统计 |
| `contrast_runs.csv` | 990行原始运行，包含成功/碰撞/出界/规划与控制时延等诊断 |
| `contrast_summary.csv` | 33行条件×场景汇总，样本标准差ddof=1、成功率Wilson95%区间 |
| `contrast_report.md` | 原分析脚本生成的英文完整合并统计报告 |
| `contrast_success_rate.png` | 按场景/条件的成功率与Wilson区间 |
| `contrast_safety_comfort.png` | 固定三场景聚合安全—舒适散点，编号图例避免标签重叠，不用于总体排名 |
| `contrast_planning_latency.png` | 本机规划均值及逐次p95均值，不能作硬实时保证 |
| `contrast_supplement.json` | 使用既有函数逐场景重算的统计，以及航向误差/控制时延等额外汇总 |
| `contrast_tables.md` / `contrast_success_table.md` | 由JSON生成的中文数值表与成功数简表 |
| `contrast_findings.md` | 由JSON生成的结果要点与失败诊断 |
| `contrast_parameters.json` | YAML/装配参数及算法默认值快照，无仿真重跑 |
| `environment.json` / `validation.json` | 实际环境和一致性复核证据 |
| `execution.log` / `tests_before.log` / `tests_after.log` | 完整正式实验进度/耗时及前后148项测试日志 |
| `run_lowpower_windows.py` | 本次实际使用的外部启动器副本，只设置线程与进程优先级，不改算法 |

`contrast.json`原始运行及元数据保持不变；分析不重新仿真。TTC汇总过滤999及≥50 s；
完成时间过滤999。控制/规划时延分开记录；表中的p95均值不是全部帧合并后的p95。
统计差值方向均为条件−proposed。Holm仅限每个条件内的既定指标组，不是整个研究全局校正。

## 只重建报告、图及文档表（无需仿真）

Linux/macOS：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B scripts/analyze_contrast.py \
  --input results/contrast/contrast.json --output-dir results/contrast

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B scripts/export_contrast_study.py --write-study-doc
```

Windows PowerShell（选择已安装numpy/matplotlib/PyYAML的Python，均从仓库根执行）：

```powershell
python -u -B results/contrast/run_lowpower_windows.py scripts/analyze_contrast.py `
  --input results/contrast/contrast.json --output-dir results/contrast
python -u -B results/contrast/run_lowpower_windows.py scripts/export_contrast_study.py --write-study-doc
python -u -B results/contrast/run_lowpower_windows.py -m unittest discover tests -v
git diff --check
```

若PyYAML安装在隔离目录，先将该目录加入本次进程的`PYTHONPATH`。
导出器校验模拟src/configs/tests/requirements与源提交一致；不更改原JSON或原主结果。
跨平台检出可能由Git转换LF/CRLF：校验仅接受这种换行符等价，不改写原始指纹；
原主结果仍比较Git规范化blob，并另报当前字节是否与本机运行前完全一致。

## 从源版本复现实验（另建工作树，避免覆盖归档）

本任务已经完成正式实验；以下仅供独立复现，不应在当前产物目录重复运行：

```bash
git worktree add --detach ../siac-contrast-reproduce e0d7643a0941a651fb2ec8ffa712c77f0f3d26ff
cd ../siac-contrast-reproduce
git status --short
# 必须干净；安装本机兼容的依赖，记录实际版本；本次精确版本见environment.json。

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B -m unittest discover tests -v

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -u -B scripts/run_contrast.py \
  --scenarios highway_merge cut_in emergency_avoidance \
  --seeds 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 \
  --duration 20.0 --allow-experimental --output-dir results/contrast

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
  nice -n 19 python3 -B scripts/analyze_contrast.py \
  --input results/contrast/contrast.json --output-dir results/contrast
```

Windows复现时，将归档的`run_lowpower_windows.py`复制到独立工作树**之外**，
使用 `python -u -B <外部启动器> scripts/run_contrast.py ...`（同上完整参数）替代nice命令。
不要在旧源工作树内新增未跟踪启动器后再宣称`git_dirty=false`。
全程单个benchmark，禁止并行启动多个；记录硬件/负载与实际运行时间。

## 解释边界

30个seeds只扰动感知噪声，场景参数固定；p值与窄置信区间不能证明道路场景分布上的优势。
经典基线不做换道，因此汇入结果包含任务能力差异。低jerk也可能来自未完成任务，
需先读成功/碰撞/出界诊断。所有负结果保留，不能为使proposed获胜而调参或删条件。
运动学与2-DOF存在后轴/质心原点混杂；Pacejka相对proposed同时改变模型和轮胎，
不能把该差值解释为纯轮胎效应。当前仍无真实轮胎标定、真实道路分布或硬实时保证。
