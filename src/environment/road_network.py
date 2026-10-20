"""
数字道路网络与拓扑地图接口。
支持多车道定义、中心参考线生成、真实弧长/切向/曲率计算、拓扑连通关系维护
以及按车辆位置查询当前所在车道。

参考线生成不再使用 ``theta=0, kappa=0`` 的占位值，而是对车道中心折线做
累积弧长、切向角与曲率的一阶/二阶差分，使 Frenet 规划器可以直接使用。
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import numpy as np

from src.common.frenet_frame import ReferencePoint


@dataclass
class Lane:
    """车道数字模型"""
    lane_id: str                       # 车道唯一标识
    width: float = 3.75                # 车道宽 (m)
    speed_limit: float = 22.22         # 限速 (m/s)
    left_lane_id: Optional[str] = None # 左相邻车道ID (用于换道决策)
    right_lane_id: Optional[str] = None# 右相邻车道ID
    predecessors: List[str] = field(default_factory=list) # 前驱车道列表
    successors: List[str] = field(default_factory=list)   # 后继车道列表
    center_points: List[Tuple[float, float]] = field(default_factory=list) # 中心线几何采样点


class RoadNetwork:
    """
    高精度数字道路网络管理器
    管理车道几何、拓扑以及全局参考线平滑
    """
    def __init__(self):
        self.lanes: Dict[str, Lane] = {}
        self.reference_lines: Dict[str, List[ReferencePoint]] = {}

    def add_lane(self, lane: Lane) -> None:
        """注册新车道"""
        self.lanes[lane.lane_id] = lane

    # ------------------------------------------------------------------
    # 参考线生成
    # ------------------------------------------------------------------
    def generate_reference_line(self, lane_id: str, sample_step: float = 1.0) -> List[ReferencePoint]:
        """
        基于车道中心几何离散点生成带 (x, y, theta, kappa, dkappa, s) 的参考线。

        :param lane_id: 车道ID
        :param sample_step: 重采样步长 (m)；<=0 时保留原始点
        :return: 连续平滑参考线点序列
        """
        if lane_id not in self.lanes:
            raise KeyError(f"Lane {lane_id} not found in RoadNetwork")

        lane = self.lanes[lane_id]
        pts = np.asarray(lane.center_points, dtype=float)
        if len(pts) == 0:
            self.reference_lines[lane_id] = []
            return []
        if len(pts) == 1:
            ref = [ReferencePoint(x=float(pts[0, 0]), y=float(pts[0, 1]), theta=0.0, s=0.0)]
            self.reference_lines[lane_id] = ref
            return ref

        # 1. 累积弧长
        seg_vec = np.diff(pts, axis=0)
        seg_len = np.hypot(seg_vec[:, 0], seg_vec[:, 1])
        s = np.concatenate(([0.0], np.cumsum(seg_len)))
        total_s = float(s[-1])
        if total_s < 1e-9:
            self.reference_lines[lane_id] = [
                ReferencePoint(x=float(pts[0, 0]), y=float(pts[0, 1]), theta=0.0, s=0.0)
            ]
            return self.reference_lines[lane_id]

        # 2. 切向角：中心差分，首尾单边差分，并做 unwrap
        theta = np.zeros(len(pts), dtype=float)
        for i in range(len(pts)):
            if i == 0:
                dx, dy = pts[1] - pts[0]
            elif i == len(pts) - 1:
                dx, dy = pts[-1] - pts[-2]
            else:
                dx, dy = pts[i + 1] - pts[i - 1]
            theta[i] = np.arctan2(dy, dx)
        theta = np.unwrap(theta)

        # 3. 曲率 kappa = dtheta/ds，dkappa = dkappa/ds
        kappa = np.zeros(len(pts), dtype=float)
        kappa[0] = (theta[1] - theta[0]) / max(seg_len[0], 1e-9)
        kappa[-1] = (theta[-1] - theta[-2]) / max(seg_len[-1], 1e-9)
        for i in range(1, len(pts) - 1):
            ds = max(s[i + 1] - s[i - 1], 1e-9)
            kappa[i] = (theta[i + 1] - theta[i - 1]) / ds
        dkappa = np.gradient(kappa, s) if len(pts) > 2 else np.zeros_like(kappa)

        # 4. 可选重采样
        if sample_step is not None and sample_step > 0 and total_s > sample_step:
            s_new = np.arange(0.0, total_s, sample_step)
            s_new = np.append(s_new, total_s)
            x_new = np.interp(s_new, s, pts[:, 0])
            y_new = np.interp(s_new, s, pts[:, 1])
            theta_new = np.interp(s_new, s, theta)
            kappa_new = np.interp(s_new, s, kappa)
            dkappa_new = np.interp(s_new, s, dkappa)
            s, x, y = s_new, x_new, y_new
            theta, kappa, dkappa = theta_new, kappa_new, dkappa_new

        ref_line = [
            ReferencePoint(
                x=float(x[i]),
                y=float(y[i]),
                theta=float(theta[i]),
                kappa=float(kappa[i]),
                dkappa=float(dkappa[i]),
                s=float(s[i]),
            )
            for i in range(len(s))
        ]
        self.reference_lines[lane_id] = ref_line
        return ref_line

    def get_reference_line(self, lane_id: str) -> Optional[List[ReferencePoint]]:
        """获取指定车道参考线；若尚未生成则按需生成。"""
        if lane_id in self.reference_lines:
            return self.reference_lines[lane_id]
        if lane_id in self.lanes:
            return self.generate_reference_line(lane_id)
        return None

    def get_lane(self, lane_id: str) -> Optional[Lane]:
        return self.lanes.get(lane_id)

    # ------------------------------------------------------------------
    # 位置查询
    # ------------------------------------------------------------------
    @staticmethod
    def _point_to_polyline_distance(x: float, y: float, points: List[Tuple[float, float]]) -> float:
        """点到车道中心折线的最短距离 (m)。"""
        if not points:
            return float("inf")
        best = float("inf")
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            vx, vy = x1 - x0, y1 - y0
            seg_len_sq = vx * vx + vy * vy
            if seg_len_sq < 1e-12:
                dist = np.hypot(x - x0, y - y0)
            else:
                ratio = float(np.clip(((x - x0) * vx + (y - y0) * vy) / seg_len_sq, 0.0, 1.0))
                dist = float(np.hypot(x - (x0 + ratio * vx), y - (y0 + ratio * vy)))
            best = min(best, dist)
        return float(best)

    def lane_id_at(
        self,
        x: float,
        y: float,
        previous_lane_id: Optional[str] = None,
    ) -> Optional[str]:
        """
        根据车辆位置查询所在车道。带简单滞回：车辆仍明显处于上一车道时保持不变，
        避免换道过程中在车道边界来回抖动。
        """
        if not self.lanes:
            return None

        distances = {
            lane_id: self._point_to_polyline_distance(x, y, lane.center_points)
            for lane_id, lane in self.lanes.items()
        }
        nearest_id = min(distances, key=distances.get)
        nearest_dist = distances[nearest_id]

        if previous_lane_id in distances:
            prev_lane = self.lanes[previous_lane_id]
            prev_dist = distances[previous_lane_id]
            # 仍处于上一车道的 75% 宽度内，保持原车道
            if prev_dist <= prev_lane.width * 0.75:
                return previous_lane_id

        # 超出最近车道半宽 + 0.5m 裕度认为不在结构化车道内
        nearest_lane = self.lanes[nearest_id]
        if nearest_dist <= nearest_lane.width * 0.75:
            return nearest_id
        return None

    def distance_to_lane(self, lane_id: str, x: float, y: float) -> float:
        """车辆到指定车道中心折线的最短距离 (m)，用于车道偏离/驶出道路判定。"""
        lane = self.lanes.get(lane_id)
        if lane is None or not lane.center_points:
            return float("inf")
        return self._point_to_polyline_distance(x, y, lane.center_points)

    def get_lane_center_y(self, lane_id: str) -> Optional[float]:
        """
        返回车道中心线的名义 y 坐标（直线道路场景）。
        对曲线道路，建议使用参考线在 ego 附近的点做横向判断。
        """
        lane = self.lanes.get(lane_id)
        if lane is None or not lane.center_points:
            return None
        return float(np.median([p[1] for p in lane.center_points]))
