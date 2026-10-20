"""
仿真结果 2D 鸟瞰图 (Bird's Eye View, BEV) 实时动态可视化器。
支持车辆 OBB 矩形包围盒、规划候选轨迹簇、最优时空轨迹及障碍物动态渲染。
满足竞赛现场答辩演示与录屏展示要求。
"""

from typing import List, Optional
from pathlib import Path
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from src.common.state import VehicleState, ObstacleState, Trajectory
from src.environment.road_network import RoadNetwork


class Visualizer2D:
    """
    基于 Matplotlib 的 2D 仿真交互可视化渲染引擎
    """
    def __init__(self, figsize=(14, 6)):
        self.figsize = figsize
        self.fig, self.ax = plt.subplots(figsize=self.figsize)
        self.fig.canvas.manager.set_window_title("智能车辆行为决策与路径规划仿真系统 - 中国仿真创新应用大赛")

    def render_frame(
        self,
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        road_network: Optional[RoadNetwork],
        planned_trajectory: Optional[Trajectory],
        current_time: float,
        step_metrics: Optional[dict] = None
    ) -> None:
        """
        绘制当前仿真帧
        """
        self.ax.clear()
        self.ax.set_title(f"Simulation Real-time Visualizer (Time: {current_time:.2f}s)", fontsize=12)
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True, linestyle="--", alpha=0.5)

        # 1. 绘制道路拓扑与车道线
        if road_network:
            for lane_id, lane in road_network.lanes.items():
                if lane.center_points:
                    pts = np.array(lane.center_points)
                    self.ax.plot(pts[:, 0], pts[:, 1], "k--", alpha=0.4, label="Lane Center" if lane_id == "lane_0" else "")

        # 2. 绘制规划轨迹
        if planned_trajectory and planned_trajectory.points:
            t_x = [pt.x for pt in planned_trajectory.points]
            t_y = [pt.y for pt in planned_trajectory.points]
            self.ax.plot(t_x, t_y, "g-", linewidth=2.5, label="Planned Trajectory")

        # 3. 绘制主车 (红色矩形)
        self._draw_vehicle_box(ego_state.x, ego_state.y, ego_state.heading, 4.8, 1.9, color="crimson", label="Ego Vehicle")

        # 4. 绘制所有动态/静态障碍物 (蓝色矩形)
        for obs in obstacles:
            self._draw_vehicle_box(obs.x, obs.y, obs.heading, obs.length, obs.width, color="royalblue", label="Traffic Vehicle")

        # 5. 视角跟随主车
        self.ax.set_xlim(ego_state.x - 20, ego_state.x + 80)
        self.ax.set_ylim(ego_state.y - 15, ego_state.y + 15)
        self.ax.set_aspect("equal")

        # 仪表板 HUD 文本
        hud_text = f"Ego Speed: {ego_state.velocity * 3.6:.1f} km/h\nSteer: {np.degrees(ego_state.steer_angle):.1f}°"
        self.ax.text(0.02, 0.95, hud_text, transform=self.ax.transAxes, verticalalignment="top",
                     bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

        plt.pause(0.001)

    @staticmethod
    def save_bev_snapshot(
        ego_state: VehicleState,
        obstacles: List[ObstacleState],
        road_network: Optional[RoadNetwork],
        planned_trajectory: Optional[Trajectory],
        current_time: float,
        save_path: str,
        dpi: int = 300,
    ) -> None:
        """Render and save a publication-quality BEV image without opening a window."""
        fig = Figure(figsize=(14, 6), dpi=dpi, constrained_layout=True)
        FigureCanvasAgg(fig)
        ax = fig.subplots()
        ax.set_title(f"Bird's-Eye View | t = {current_time:.2f} s", fontsize=14, pad=12)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.grid(True, linestyle="--", linewidth=0.6, alpha=0.45)

        if road_network:
            for index, (lane_id, lane) in enumerate(road_network.lanes.items()):
                if lane.center_points:
                    points = np.asarray(lane.center_points, dtype=float)
                    ax.plot(points[:, 0], points[:, 1], "--", color="0.45", linewidth=1.2,
                            alpha=0.75, label="Lane centerline" if index == 0 else None)

        if planned_trajectory and planned_trajectory.points:
            points = planned_trajectory.points
            tx = np.asarray([p.x for p in points], dtype=float)
            ty = np.asarray([p.y for p in points], dtype=float)
            # Smoothly sample the discrete planner output for a clean report curve.
            if len(points) >= 3 and np.all(np.isfinite(tx)) and np.all(np.isfinite(ty)):
                distance = np.r_[0.0, np.cumsum(np.hypot(np.diff(tx), np.diff(ty)))]
                unique = np.r_[True, np.diff(distance) > 1e-9]
                if np.count_nonzero(unique) >= 3:
                    sample = np.linspace(distance[unique][0], distance[unique][-1], max(100, len(points) * 8))
                    s = distance[unique]
                    x = tx[unique]
                    y = ty[unique]
                    segment = np.clip(np.searchsorted(s, sample, side="right") - 1, 0, len(s) - 2)
                    span = s[segment + 1] - s[segment]
                    u = (sample - s[segment]) / span
                    # Piecewise cubic Hermite polynomials interpolate the planner's path points.
                    def interpolate(values):
                        slopes = np.gradient(values, s)
                        return ((2*u**3 - 3*u**2 + 1) * values[segment]
                                + (u**3 - 2*u**2 + u) * span * slopes[segment]
                                + (-2*u**3 + 3*u**2) * values[segment + 1]
                                + (u**3 - u**2) * span * slopes[segment + 1])
                    ax.plot(interpolate(x), interpolate(y), color="forestgreen",
                            linewidth=2.4, label="Planned trajectory")
                else:
                    ax.plot(tx, ty, color="forestgreen", linewidth=2.4, label="Planned trajectory")
            else:
                ax.plot(tx, ty, color="forestgreen", linewidth=2.4, label="Planned trajectory")

        def draw_box(x, y, heading, length, width, color, label):
            center = np.array([x, y], dtype=float)
            rotation = np.array([[np.cos(heading), -np.sin(heading)],
                                 [np.sin(heading), np.cos(heading)]])
            local = np.array([[-length/2, -width/2], [length/2, -width/2],
                              [length/2, width/2], [-length/2, width/2]])
            corners = local @ rotation.T + center
            ax.add_patch(patches.Polygon(corners, closed=True, facecolor=color,
                                         edgecolor="black", linewidth=0.9, alpha=0.88,
                                         label=label))

        draw_box(ego_state.x, ego_state.y, ego_state.heading, 4.8, 1.9, "crimson", "Ego vehicle")
        for index, obs in enumerate(obstacles or []):
            draw_box(obs.x, obs.y, obs.heading, obs.length, obs.width,
                     "royalblue", "Obstacle vehicle" if index == 0 else None)

        ax.set_xlim(ego_state.x - 20, ego_state.x + 80)
        ax.set_ylim(ego_state.y - 15, ego_state.y + 15)
        ax.set_aspect("equal", adjustable="box")
        hud = (f"TIME  {current_time:.2f} s\n"
               f"SPEED  {ego_state.velocity * 3.6:.1f} km/h\n"
               f"ACCEL  {ego_state.acceleration:.2f} m/s²\n"
               f"STEER  {np.degrees(ego_state.steer_angle):.1f}°")
        ax.text(0.015, 0.98, hud, transform=ax.transAxes, va="top", ha="left",
                fontsize=10, family="monospace",
                bbox=dict(boxstyle="round,pad=0.55", facecolor="white", edgecolor="0.65", alpha=0.92))
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            unique = dict(zip(labels, handles))
            ax.legend(unique.values(), unique.keys(), loc="upper right", framealpha=0.95, fontsize=9)
        output = Path(save_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=max(300, int(dpi)), format="png", bbox_inches="tight")
        fig.clear()

    def _draw_vehicle_box(self, x: float, y: float, heading: float, length: float, width: float, color: str, label: str = ""):
        """根据位姿绘制旋转矩形"""
        corner_x = x - (length / 2.0) * np.cos(heading) + (width / 2.0) * np.sin(heading)
        corner_y = y - (length / 2.0) * np.sin(heading) - (width / 2.0) * np.cos(heading)
        rect = patches.Rectangle(
            (corner_x, corner_y),
            length,
            width,
            angle=np.degrees(heading),
            color=color,
            alpha=0.85
        )
        self.ax.add_patch(rect)

    def close(self):
        plt.close(self.fig)
