"""
仿真结果 2D 鸟瞰图 (Bird's Eye View, BEV) 实时动态可视化器。
支持车辆 OBB 矩形包围盒、规划候选轨迹簇、最优时空轨迹及障碍物动态渲染。
满足竞赛现场答辩演示与录屏展示要求。
"""

from typing import List, Optional
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
