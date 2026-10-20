# 竞赛材料包（v0.4-core）

当前目录保留**核心链路**的 KPI 表与 5 张图：

- 范围：HFSM + Frenet Lattice + LQR + 运动学模型；
- 场景：highway_merge、cut_in、emergency_avoidance；
- 样本：30 seeds（轻量基准）；
- 成功判据：任务完成 + 无碰撞 + 不驶出道路。

## 已生成

| 文件 | 内容 |
| :--- | :--- |
| `kpi_table.md` | 30-seed 核心 KPI 表 |
| `figures/fig1_highway_merge_bev.png` | 高速汇入 BEV（首次进入主车道附近） |
| `figures/fig2_cut_in_bev.png` | cut-in 最小净距帧 |
| `figures/fig3_emergency_avoidance_bev.png` | 紧急制动停车帧 |
| `figures/fig4_core_telemetry.png` | 三场景速度/加速度/Jerk/TTC 遥测 |
| `figures/fig5_core_kpi_distribution.png` | 30-seed KPI 箱线图 |

生成命令：

```bash
python3 scripts/export_core_figures.py --seed 42 --duration 20 \
  --output-dir results/competition_pack/figures
```

## 待补

- [ ] 30–50 seeds 版本 KPI 表（论文最终数字）
- [ ] 系统架构图（矢量 PDF）
- [ ] 场景拓扑图（匝道/切入/紧急制动）
- [ ] 任务成功/失败案例对比图（Game 未汇入、交叉口驶出）
- [ ] 答辩 PPT 与 3–5 min 演示视频
- [ ] 论文/申报书按官方模板排版

## 旧版

旧版 `kpi_table.md` 与四张图来自范围收敛前的代码，已归档到：

```text
results/archive/2026-10-20_pre_scope/competition_pack/
```

**不要**在论文/答辩中引用旧图或旧表。
