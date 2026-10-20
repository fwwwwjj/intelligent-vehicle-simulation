#!/usr/bin/env python3
"""Validate the archived 30-seed study and export tables, without simulation.

Uses the existing contrast statistics unchanged. Scenario-specific comparisons
are supplementary; the original pooled comparisons and raw JSON stay intact.
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.evaluation.contrast import (  # noqa: E402
    CONDITION_NAMES, CORE_SCENARIOS, DEFAULT_SEEDS, PROPOSED_CONDITION,
    compare_conditions, summarize_runs, write_runs_csv,
    write_summary_csv,
)

BASELINE = 'e0d7643a0941a651fb2ec8ffa712c77f0f3d26ff'
EXTRA_METRICS = (
    'mean_heading_error_rad', 'avg_control_latency_ms',
    'p95_control_latency_ms', 'avg_decision_latency_ms',
    'road_departure', 'duration_s',
)

STUDY_HEADER = """# 基线对比与组件消融：30-seed 闭环实验

本节为补充对比实验，原 `results/benchmark*.csv/json` 主结果保持原样。
当前算法、场景、默认参数及成功判据均冻结于
`e0d7643a0941a651fb2ec8ffa712c77f0f3d26ff`；没有为了结果调整算法或删去失败条件。
原始逐次数据、CSV 和统计见 [results/contrast](../results/contrast/README.md)。

## 实验设置与口径

- 11 conditions × 三个固定场景 × seeds 42–71 = **990 次运行**；每次 20 s，dt=0.05 s（400 步）。
  KPI 的 `duration_s` 为最后一帧时间戳 19.95 s，表示完整 400 步，并非少跑了一个周期。
- 同一 `(scenario, seed)` 在所有条件间配对。seed 只扰动感知位置/速度噪声，标准差分别
  0.05 m / 0.10 m/s；道路几何、车流初态、切入时刻等没有做场景参数随机化。
- 执行环境：Windows 11、Intel Core i7-12700H、Python 3.13.1、NumPy 2.2.2、
  SciPy 1.15.1、Matplotlib 3.10.0、PyYAML 6.0.3；单进程串行执行所有条件，
  四个 BLAS 环境变量均为 1，Windows `IDLE_PRIORITY_CLASS` 对应低优先级运行约定。
  依赖版本高于仓库原 `requirements.txt` 上限；本次 148 项测试全绿，不能冒称原 Python 3.9/Mac 环境。
  完整环境记录见 `results/contrast/environment.json`。
- 所有算法复用同一装配工厂和安全监督。中心线规划器不感知障碍物，但决策层的减速/
  AEB 与引擎安全监督保留，因此它并非“删除所有安全保护”的条件。
- 成功由场景任务、安全、道路偏离共同判定：汇入须在观察窗内进入主线；cut-in 须无碰撞、
  净距≥2 m、不出界且观察≥5 s；紧急制动还须终速≤0.5 m/s。失败运行全部保留。
- 最小净距为 OBB 边界距离；TTC 只统计前方横向重叠且有接近的目标；999 为哨兵。
  TTC 汇总保留有限且<50 s 的值；完成时间只保留<999 s，缺失不表示 999 秒完成。
  jerk 来自实际加速度差分；横向/航向误差相对当前规划轨迹，不是对地图车道的绝对跟踪精度。
- 车辆/控制参数实际快照见 `contrast_parameters.json`：轴距2.85 m、质量1600 kg、
  最大前轮角0.65 rad、转角速率0.50 rad/s、加速/制动上限3.5/7.0 m/s²、jerk上限4.0 m/s³；
  LQR 的 Q 对角为(2,0.1,1.5,0.1)、R=1、基础预瞄0.6 s；MPC Np/Nc=20/10，
  Q横向/航向/速度=10/6/1，R加速/转向=0.08/0.8，增量权重0.6/6，横向软限0.9 m；
  纯跟踪预瞄0.8 s、最小前视2 m。配置与构造默认值均未针对实验条件重新调参。

"""

STUDY_FOOTER = """
## 统计解释与局限

1. Wilson 区间来自每个固定场景的二值结果。30/30 对应约[0.886,1.000]，
   0/30 对应约[0.000,0.114]；这不是实际道路分布上的成功率保证。
2. Wilcoxon 使用仓库既有正态近似及并列/零差处理；bootstrap 是配对差值百分位区间，
   不因检验显著而自动成为场景泛化证据。近零组内方差、窄区间或很小 p 值可能只反映
   确定性场景下算法行为稳定。Holm 的校正范围见表注，不是跨整个研究的全局校正。
3. 合并三场景的指标均值不能抹平任务差异。未完成任务、碰撞后继续仿真等条件中的
   jerk/净距/时延不可当作完成同一任务的效能优势；应先结合成功和失败诊断阅读。
   合并检验/重采样沿用原实现，没有按相同seed跨场景聚类；同seed也不保证不同闭环
   行为下的每个感知噪声样本完全一致。相关合并p值仅作当前固定实验的描述性结果。
4. 模型消融有明确混杂：运动学位姿以**后轴**为原点，2-DOF 以**质心**为原点，当前
   引擎没有做原点变换。因此跨模型结果是当前实现的敏感性诊断，不能全部归因为轮胎/
   动力学保真度。Pacejka 相对 proposed 还同时改变动力学与轮胎；与线性2-DOF对照才
   单独隔离轮胎模型。Pacejka 尚无真实轮胎试验标定。
5. 条件按固定顺序串行运行，没有交错计时；Windows 空闲优先级、背景负载与热状态会
   影响时延。表中均值和逐次 p95 只用于本机观察，不保证50 Hz硬实时；不得与历史Mac
   主结果的时延直接作算法优劣对比。安全—舒适散点图按场景聚合，不能据此进行总体排名。
6. 该实验没有覆盖场景参数分布、真实highD闭环、城市路口、不同附着/载荷或真实硬件延迟。
   增加同一固定场景的噪声seed数量，不能替代场景多样性。Game/MPC失败和模型停车差异
   均如实保留；未修改算法以使 proposed 获胜。

## 产物与复现

运行与分析命令见 [结果目录说明](../results/contrast/README.md)。
`run_contrast.py` 生成原始JSON/CSV及合并统计；`analyze_contrast.py` 从原始数据重建
报告和三张图；`export_contrast_study.py --write-study-doc` 复核数据并生成本节表格、
逐场景补充统计及参数快照，不重新仿真。三个入口都应使用单线程BLAS和低优先级。
本次正式运行前后均执行完整148项测试及 `git diff --check`；原主结果Git blob保持一致。
"""


def table(headers, rows):
    lines = ['| ' + ' | '.join(headers) + ' |',
             '| ' + ' | '.join([':---'] * len(headers)) + ' |']
    lines.extend('| ' + ' | '.join(map(str, row)) + ' |' for row in rows)
    return '\n'.join(lines)


def number(value, digits=3):
    return '—' if value is None else f'{value:.{digits}f}'


def mean_std(payload, digits=3):
    if payload['mean'] is None:
        return '—'
    return f"{payload['mean']:.{digits}f} ± {payload['std']:.{digits}f}"


def interval(payload, digits=3):
    if payload['mean_diff'] is None:
        return '—'
    return (f"{payload['mean_diff']:+.{digits}f} "
            f"[{payload['ci95_low']:+.{digits}f}, {payload['ci95_high']:+.{digits}f}]")


def short_name(name):
    return name.replace('ablation_', '').replace('proposed_fsm_lattice_lqr', 'proposed').replace(
        'baseline_idm_centerline_pure_pursuit', 'classical_baseline')


def text_hashes(blob):
    """Checkout EOL variants only; recorded simulation fingerprints stay intact."""
    lf = blob.replace(b'\r\n', b'\n')
    return {hashlib.sha256(value).hexdigest()
            for value in (blob, lf, lf.replace(b'\n', b'\r\n'))}


def validate(data, directory):
    metadata, runs = data['metadata'], data['runs']
    assert metadata['git_commit'] == BASELINE
    assert metadata['git_dirty'] is False
    assert metadata['duration_s'] == 20.0
    assert metadata['seeds'] == list(DEFAULT_SEEDS)
    assert metadata['condition_names'] == list(CONDITION_NAMES)
    assert metadata['scenario_configs'] == list(CORE_SCENARIOS)
    expected = {(c, s, seed) for c in CONDITION_NAMES
                for s in CORE_SCENARIOS for seed in DEFAULT_SEEDS}
    actual = [(r['condition'], r['scenario'], r['seed']) for r in runs.values()]
    assert len(actual) == len(set(actual)) == 990
    assert set(actual) == expected
    for scenario in CORE_SCENARIOS:
        path = ROOT / 'configs' / 'scenarios' / (scenario + '.yaml')
        assert metadata['scenario_sha256'][scenario] in text_hashes(path.read_bytes())
    for row in runs.values():
        condition = data['conditions'][row['condition']]
        for key in ('decision', 'planner', 'controller', 'tire_model', 'dynamic_model'):
            assert row[key] == condition[key], (row['run_key'], key)
        assert row['success_rate'] in (0.0, 1.0)
        assert row['task_success'] == row['success_rate']
        for key in ('min_obstacle_distance_m', 'min_ttc_s', 'rms_jerk_m_s3',
                    'max_lateral_accel_m_s2', 'max_lateral_error_m',
                    'mean_heading_error_rad', 'avg_planning_latency_ms',
                    'p95_planning_latency_ms', 'avg_control_latency_ms',
                    'p95_control_latency_ms', 'collision_frames'):
            assert math.isfinite(row[key]) and row[key] >= 0, (row['run_key'], key, row[key])
        if row['success_rate']:
            assert not row['collision_frames'] and not row['road_departure'], row['run_key']
        assert row['duration_s'] >= 19.9, (row['run_key'], row['duration_s'])
        assert row['duration_s'] <= 20.1
    # Check deterministic CSV serialization against the JSON, without touching
    # either original CSV. Also covers optional/raw diagnostic columns.
    with tempfile.TemporaryDirectory(prefix='siac_contrast_verify_') as tmp:
        raw = Path(tmp) / 'runs.csv'
        summary = Path(tmp) / 'summary.csv'
        write_runs_csv(raw, runs)
        write_summary_csv(summary, data['summary'])
        assert raw.read_bytes().replace(b'\r\n', b'\n') == (directory / 'contrast_runs.csv').read_bytes().replace(b'\r\n', b'\n')
        assert summary.read_bytes().replace(b'\r\n', b'\n') == (directory / 'contrast_summary.csv').read_bytes().replace(b'\r\n', b'\n')
    assert summarize_runs(runs) == data['summary']
    assert compare_conditions(runs, n_boot=metadata['n_boot'], seed=metadata['bootstrap_seed']) == data['comparisons']
    unchanged = subprocess.check_output(
        ['git', 'diff', '--name-only', BASELINE, '--', 'src', 'configs', 'tests', 'requirements.txt'],
        cwd=ROOT, text=True,
    ).strip()
    assert not unchanged, unchanged
    # The baseline's main-result blobs must remain byte-identical in Git's
    # canonical text representation (handles checkout CRLF transparently).
    original_files = subprocess.check_output(
        ['git', 'ls-tree', '-r', '--name-only', BASELINE, '--', 'results'],
        cwd=ROOT, text=True,
    ).splitlines()
    main_hashes = {}
    for name in original_files:
        if not Path(name).name.startswith('benchmark'):
            continue
        old = subprocess.check_output(['git', 'rev-parse', BASELINE + ':' + name], cwd=ROOT, text=True).strip()
        current = subprocess.check_output(['git', 'hash-object', '--path=' + name, name], cwd=ROOT, text=True).strip()
        assert old == current, name
        main_hashes[name] = old
    environment_path = directory / 'environment.json'
    raw_main_hashes = {}
    if environment_path.is_file():
        before = json.loads(environment_path.read_text(encoding='utf-8'))
        for name, expected_hash in before['original_benchmark_sha256'].items():
            actual_hash = hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            assert expected_hash in text_hashes((ROOT / name).read_bytes()), name
            raw_main_hashes[name] = {'actual_sha256': actual_hash, 'recorded_sha256': expected_hash,
                                     'exact_original_bytes': actual_hash == expected_hash}
    original_tables = {}
    for name in ('README.md', 'docs/03_results_and_demo.md', 'docs/08_paper_draft.md'):
        old = subprocess.check_output(['git', 'show', BASELINE + ':' + name], cwd=ROOT).decode('utf-8')
        current = (ROOT / name).read_text(encoding='utf-8')
        rows = [line for line in old.splitlines() if line.startswith('| ') and '1.000 [0.886, 1.000]' in line]
        assert len(rows) == 3, (name, len(rows))
        assert all(line in current for line in rows), name
        original_tables[name] = hashlib.sha256('\n'.join(rows).encode('utf-8')).hexdigest()
    test_evidence = {}
    for name in ('tests_before.log', 'tests_after.log'):
        path = directory / name
        if not path.is_file():
            continue
        log = path.read_text(encoding='utf-8')
        result = re.search(r'Ran 148 tests in ([0-9.]+)s', log)
        assert result and log.strip().endswith('OK'), name
        test_evidence[name] = {'n_tests': 148, 'status': 'OK',
                               'duration_s': float(result.group(1)),
                               'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    return {'passed': True, 'n_runs': 990, 'n_groups': 33,
            'source_commit': BASELINE, 'main_result_git_blobs': main_hashes,
            'main_result_raw_sha256_verified': raw_main_hashes,
            'original_main_table_rows_sha256': original_tables,
            'archived_test_evidence': test_evidence,
            'checks': ['complete unique Cartesian product', '20-second observations',
                       'finite nonnegative KPIs and success implies no collision/departure',
                       'resolved stack matches condition', 'metadata clean and scene hashes valid',
                       'raw and summary CSV equality (checkout LF/CRLF only ignored)',
                       'summary and pooled statistics reproduced',
                       'simulation source/config/tests unchanged', 'original main result blobs unchanged',
                       'original README/docs core result rows preserved']}


def export(data, directory):
    groups = data['summary']['by_condition_scenario']
    extra = summarize_runs(data['runs'], metrics=EXTRA_METRICS)
    stratified = {
        s: compare_conditions(
            {key: row for key, row in data['runs'].items() if row['scenario'] == s},
            n_boot=data['metadata']['n_boot'], seed=data['metadata']['bootstrap_seed'],
        ) for s in CORE_SCENARIOS
    }
    supplement = {'source_commit': BASELINE,
                  'definition': 'Existing compare_conditions applied separately per scenario; no new simulations.',
                  'by_scenario': stratified, 'additional_summary': extra}
    (directory / 'contrast_supplement.json').write_text(
        json.dumps(supplement, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    lines = ['<!-- Generated by scripts/export_contrast_study.py; do not hand-edit numeric tables. -->',
             '## 条件矩阵', '',
             'P 为运动学 proposed；D 为 2-DOF 线性模型。Pacejka 与 D 比较才单独隔离轮胎模型；',
             '标准统计仍统一对 P，因此 Pacejka 对 P 同时包含动力学和轮胎两项变化。', '',
             table(['condition', '决策', '规划', '控制', '车辆模型', 'experimental'], [
                 [name, c['decision'], c['planner'], c['controller'],
                  ('2-DOF/' + c['tire_model']) if c['dynamic_model'] else '运动学', str(c['experimental'])]
                 for name, c in data['conditions'].items()]), '',
             '以下简写去掉 `ablation_`；`proposed` 与 `classical_baseline` 对应上表完整名称。', '',
             '## 逐场景 30-seed 结果', '',
             '连续指标为逐次 KPI 的均值 ± 样本标准差；`规划 p95 均值` 是每次运行逐帧 p95 的均值，',
             '并非合并所有帧后再取 p95。成功区间采用 Wilson 95%；0.000 可能是显示舍入，完整精度保留在 JSON。', '']
    for scenario in CORE_SCENARIOS:
        rows = []
        for name in CONDITION_NAMES:
            g = groups[name + '::' + scenario]
            metrics = g['metrics']
            rows.append([short_name(name),
                         f"{g['success_count']}/30 [{g['success_ci95_low']:.3f}, {g['success_ci95_high']:.3f}]",
                         mean_std(metrics['min_obstacle_distance_m']),
                         mean_std(metrics['min_ttc_s']), mean_std(metrics['rms_jerk_m_s3']),
                         mean_std(metrics['avg_planning_latency_ms']),
                         number(metrics['p95_planning_latency_ms']['mean'])])
        lines += ['### ' + scenario, '', table(
            ['condition', '成功数 [95% CI]', '净距/m', 'TTC/s', 'jerk/(m/s³)', '规划均值/ms', '规划 p95 均值/ms'], rows), '']
    lines += ['## 跟踪、控制时延与失败诊断', '',
              '碰撞/驶出道路为发生该事件的运行数；控制回退是逐次回退帧比例的均值，不能单独作为安全判据。', '']
    diagnostic_rows = []
    for scenario in CORE_SCENARIOS:
        for name in CONDITION_NAMES:
            group_key = name + '::' + scenario
            metrics = groups[group_key]['metrics']
            more = extra['by_condition_scenario'][group_key]['metrics']
            raw = [r for r in data['runs'].values() if r['scenario'] == scenario and r['condition'] == name]
            diagnostic_rows.append([
                scenario, short_name(name), mean_std(metrics['max_lateral_error_m']),
                mean_std(more['mean_heading_error_rad'], 6), mean_std(metrics['max_lateral_accel_m_s2']),
                mean_std(more['avg_control_latency_ms']), number(more['p95_control_latency_ms']['mean']),
                str(sum(r['collision_frames'] > 0 for r in raw)),
                str(sum(bool(r['road_departure']) for r in raw)),
                number(metrics['controller_fallback_rate']['mean'], 6),
            ])
    lines += [table(['场景', 'condition', '最大横向误差/m', '平均绝对航向误差/rad',
                     '最大横向加速度/(m/s²)', '控制均值/ms', '控制 p95 均值/ms',
                     '碰撞运行数', '出界运行数', '控制回退率'], diagnostic_rows), '',
              '## 配对统计：合并三场景', '',
              '每个条件有 90 个 `(scenario, seed)` 配对。合并均值/检验是固定三个场景的描述性汇总，',
              '不能当作 90 个独立抽取的道路场景；逐场景结果见下一节。', '']
    success_rows = []
    for comparison in data['comparisons']['comparisons']:
        s = comparison['success']
        success_rows.append([short_name(comparison['condition']), str(s['n_pairs']),
                             f"{s['success_count_condition']}/{s['n_pairs']}",
                             number(s['risk_difference']), str(s['discordant_baseline_only']),
                             str(s['discordant_condition_only']), f"{s['mcnemar_p_value']:.6g}",
                             f"{s['mcnemar_p_holm']:.6g}"])
    lines += [table(['condition', '配对数', '条件成功数', '成功率差', '仅 P 成功 b',
                     '仅条件成功 c', 'McNemar p', 'Holm p'], success_rows), '',
              '## 配对统计：逐场景净距与 jerk', '',
              '差值为条件 − proposed；方括号为 2,000 次配对 bootstrap 的百分位 95% CI，RNG seed=0。',
              'p 列为 Wilcoxon 正态近似经每个条件内部（成功 + 既定指标组）Holm 校正后的值；',
              '没有做跨 10 个条件或跨三个场景的全局校正。Cliff’s δ 比较边际分布，不是配对效应量。', '']
    paired_rows = []
    for scenario, comparison_set in stratified.items():
        for comparison in comparison_set['comparisons']:
            m = comparison['metrics']
            gap, jerk = m['min_obstacle_distance_m'], m['rms_jerk_m_s3']
            paired_rows.append([scenario, short_name(comparison['condition']),
                                interval(gap), f"{gap['wilcoxon_p_holm']:.6g}", number(gap['cliffs_delta']),
                                interval(jerk), f"{jerk['wilcoxon_p_holm']:.6g}", number(jerk['cliffs_delta'])])
    lines += [table(['场景', 'condition', '净距差/m [95% CI]', 'p(Holm)', 'δ',
                     'jerk 差/(m/s³) [95% CI]', 'p(Holm)', 'δ'], paired_rows), '',
              '完整逐场景成功检验及所有既定指标在 `contrast_supplement.json`；',
              '原有合并统计保持在 `contrast.json.comparisons`，未覆盖或重新定义。', '']
    tables = '\n'.join(lines)
    (directory / 'contrast_tables.md').write_text(tables, encoding='utf-8')
    conclusions(data, directory)
    capture_parameters(directory)
    return tables


def capture_parameters(directory):
    """Archive loader-resolved config and constructor defaults, without stepping."""
    from src.control.lqr_controller import LQRController
    from src.control.mpc_controller import MPCController
    from src.control.pure_pursuit_controller import PurePursuitController
    from src.environment.scenario_loader import load_named_scenario
    from src.runner import build_controller
    resolved = {}
    for name in CORE_SCENARIOS:
        scene = load_named_scenario(name)
        controllers = {}
        for controller_name in ('lqr', 'mpc', 'pure_pursuit'):
            controller = build_controller(scene, controller_name)
            values = {}
            for key, value in vars(controller).items():
                if key.startswith('_'):
                    continue
                if value is None or isinstance(value, (str, int, float, bool)):
                    values[key] = value
                elif hasattr(value, 'tolist'):
                    values[key] = value.tolist()
            controllers[controller_name] = values
        resolved[name] = {'dt_s': scene.dt, 'vehicle_params': scene.vehicle_params,
                          'sensor_params': scene.sensor_params,
                          'planning_params': scene.planning_params,
                          'control_params': scene.control_params,
                          'resolved_controller_attributes_before_first_step': controllers}
    defaults = {}
    for cls in (LQRController, MPCController, PurePursuitController):
        defaults[cls.__name__] = {
            key: p.default for key, p in inspect.signature(cls).parameters.items()
            if p.default is not inspect.Parameter.empty}
    config = ROOT / 'configs/default_config.yaml'
    payload = {'simulation_source_commit': BASELINE,
               'note': 'Read after simulation; source/config identity checked against the recorded baseline. No stepping.',
               'default_config_sha256': hashlib.sha256(config.read_bytes()).hexdigest(),
               'constructor_defaults': defaults, 'resolved_by_scenario': resolved}
    (directory / 'contrast_parameters.json').write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def conclusions(data, directory):
    """Emit factual prose and a compact comparison table from the stored runs."""
    groups = data['summary']['by_condition_scenario']
    def g(name, scene):
        return groups[name + '::' + scene]
    def avg(name, scene, metric):
        return g(name, scene)['metrics'][metric]['mean']
    rows = []
    for name in CONDITION_NAMES:
        entries = [g(name, s) for s in CORE_SCENARIOS]
        rows.append([short_name(name)] + [str(e['success_count']) + '/30' for e in entries]
                    + [str(sum(e['success_count'] for e in entries)) + '/90'])
    success_table = table(['condition', 'highway_merge', 'cut_in', 'emergency_avoidance', '合计'], rows)
    (directory / 'contrast_success_table.md').write_text(success_table + '\n', encoding='utf-8')
    lines = ['## 结果要点（从正式 JSON 生成）', '', success_table, '',
             '以下结论只涉及这三个固定合成场景及当前实现，不表示某类算法的一般优劣。', '']
    p = PROPOSED_CONDITION
    baseline = 'baseline_idm_centerline_pure_pursuit'
    lines += [f"- proposed 共 {sum(g(p, s)['success_count'] for s in CORE_SCENARIOS)}/90 次任务成功；"
              f"经典基线共 {sum(g(baseline, s)['success_count'] for s in CORE_SCENARIOS)}/90。"
              '经典基线不产生换道意图，汇入任务包含能力差异，不能把汇入差距仅归因于低层跟踪。',
              '- Game 的高速汇入成功数为 '
              f"{g('ablation_decision_game', 'highway_merge')['success_count']}/30；"
              '任务失败时的低 jerk 不构成完成同一任务后的舒适性优势。']
    for scene in CORE_SCENARIOS:
        name = 'ablation_controller_mpc'
        raw = [r for r in data['runs'].values() if r['condition'] == name and r['scenario'] == scene]
        lines += [f"- MPC / {scene}：成功 {g(name, scene)['success_count']}/30，"
                  f"发生碰撞的运行 {sum(r['collision_frames'] > 0 for r in raw)}/30，"
                  f"出界 {sum(bool(r['road_departure']) for r in raw)}/30，"
                  f"最小净距均值 {avg(name, scene, 'min_obstacle_distance_m'):.3f} m。"
                  '失败运行同样纳入全部统计；不能用优化器名称替代闭环安全验证。']
    for scene in CORE_SCENARIOS:
        em = 'ablation_planner_em'
        pp = 'ablation_controller_pure_pursuit'
        ratio = avg(em, scene, 'avg_planning_latency_ms') / avg(p, scene, 'avg_planning_latency_ms')
        lines += [f"- {scene}：EM 净距 {avg(em, scene, 'min_obstacle_distance_m'):.3f} m，"
                  f"proposed {avg(p, scene, 'min_obstacle_distance_m'):.3f} m；"
                  f"jerk {avg(em, scene, 'rms_jerk_m_s3'):.3f} / {avg(p, scene, 'rms_jerk_m_s3'):.3f} m/s³；"
                  f"规划均值时延为 proposed 的 {ratio:.2f} 倍。"
                  f"纯跟踪控制 jerk {avg(pp, scene, 'rms_jerk_m_s3'):.3f}，"
                  f"LQR {avg(p, scene, 'rms_jerk_m_s3'):.3f} m/s³。"]
    for name in ('ablation_model_dynamic_2dof', 'ablation_model_pacejka'):
        scene = 'emergency_avoidance'
        raw = [r for r in data['runs'].values() if r['condition'] == name and r['scenario'] == scene]
        stop_only_failures = [r for r in raw if not r['task_success'] and not r['collision_frames']
                              and not r['road_departure'] and r['min_obstacle_distance_m'] >= 2]
        lines += [f"- {short_name(name)} / emergency：成功 {g(name, scene)['success_count']}/30，"
                  f"最小净距 {avg(name, scene, 'min_obstacle_distance_m'):.3f} m；"
                  f"其中 {len(stop_only_failures)} 次无碰撞、无出界、净距≥2 m 却任务失败。"
                  '按当前任务判据可推断这些运行未达到终速≤0.5 m/s；原始 KPI 未保存最终速度，'
                  '故不报告具体终速或唯一物理根因。']
    lines += ['- 中心线规划器即使通过当前场景，也不具有障碍物搜索/碰撞优化能力；'
              '决策层减速与统一安全监督仍在运行，因此不能据此判定避障规划无用。',
              '- proposed 可作为任务成功、舒适性与计算时延之间的工程折中。'
              '当前结果不支持“全面优于所有基线”，也不支持提升至真实车辆或复杂场景分布的主张。', '']
    (directory / 'contrast_findings.md').write_text('\n'.join(lines), encoding='utf-8')


def update_generated_block(path, content):
    """Update only explicitly marked numeric blocks in authored documents."""
    start, end = '<!-- BEGIN CONTRAST RESULTS -->', '<!-- END CONTRAST RESULTS -->'
    text = path.read_text(encoding='utf-8')
    if start not in text and end not in text:
        return
    assert text.count(start) == text.count(end) == 1, path
    before, tail = text.split(start)
    _, after = tail.split(end)
    path.write_text(before + start + '\n\n' + content.strip() + '\n\n' + end + after, encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT / 'results/contrast/contrast.json')
    parser.add_argument('--write-study-doc', action='store_true',
                        help='Generate docs/11_baseline_and_ablation.md from the same raw data.')
    args = parser.parse_args()
    data = json.loads(args.input.read_text(encoding='utf-8'))
    validation = validate(data, args.input.parent)
    tables = export(data, args.input.parent)
    if args.write_study_doc:
        findings = (args.input.parent / 'contrast_findings.md').read_text(encoding='utf-8')
        (ROOT / 'docs/11_baseline_and_ablation.md').write_text(
            STUDY_HEADER + findings + '\n' + tables + STUDY_FOOTER, encoding='utf-8')
        success_table = (args.input.parent / 'contrast_success_table.md').read_text(encoding='utf-8')
        update_generated_block(ROOT / 'README.md', success_table)
        update_generated_block(ROOT / 'docs/08_paper_draft.md', findings.replace(
            '## 结果要点（从正式 JSON 生成）\n\n', ''))
    (args.input.parent / 'validation.json').write_text(
        json.dumps(validation, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('Validated 990 runs / 33 groups; original algorithms and main results unchanged.')


if __name__ == '__main__':
    main()
