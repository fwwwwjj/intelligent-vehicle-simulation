"""
计算几何基础模块与高保真碰撞检测接口。
实现有向包围盒 (OBB, Oriented Bounding Box)、分离轴定理 (SAT) 及几何判定基础。
"""

from typing import List, Tuple
import numpy as np


class Point2D:
    """二维空间点"""
    def __init__(self, x: float, y: float):
        self.x = float(x)
        self.y = float(y)

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=np.float64)


class BoundingBox2D:
    """
    二维有向包围盒 (OBB, Oriented Bounding Box)
    广泛应用于自动驾驶车辆与障碍物的精准几何轮廓建模
    """
    def __init__(self, center_x: float, center_y: float, length: float, width: float, heading: float):
        """
        初始化包围盒
        :param center_x: 几何中心 X (m)
        :param center_y: 几何中心 Y (m)
        :param length: 沿车头航向方向长度 (m)
        :param width: 垂直航向方向宽度 (m)
        :param heading: 方位航向角 (rad)
        """
        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.length = float(length)
        self.width = float(width)
        self.heading = float(heading)

    def get_corners(self) -> List[Tuple[float, float]]:
        """
        计算包围盒在世界坐标系下的4个顶点坐标 (从前左逆时针排列)
        """
        dx = self.length / 2.0
        dy = self.width / 2.0
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)

        local_corners = [
            (dx, dy),    # 前左
            (-dx, dy),   # 后左
            (-dx, -dy),  # 后右
            (dx, -dy)    # 前右
        ]

        corners = []
        for lx, ly in local_corners:
            gx = self.center_x + lx * cos_h - ly * sin_h
            gy = self.center_y + lx * sin_h + ly * cos_h
            corners.append((float(gx), float(gy)))
        return corners

    def get_axes(self) -> List[np.ndarray]:
        """
        获取该矩形两条正交边的单位法向量（分离轴候选轴）
        """
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        # 车身朝向单位向量与垂直朝向单位向量
        axis_longitudinal = np.array([cos_h, sin_h], dtype=np.float64)
        axis_lateral = np.array([-sin_h, cos_h], dtype=np.float64)
        return [axis_longitudinal, axis_lateral]


class CollisionChecker:
    """
    高精度碰撞检测器接口。
    基于严格分离轴定理 (Separating Axis Theorem, SAT) 提供高保真多边形/车辆 OBB 碰撞检测。
    """

    @staticmethod
    def check_obb_collision(box1: BoundingBox2D, box2: BoundingBox2D, buffer: float = 0.1) -> bool:
        """
        严格应用分离轴定理 (SAT) 检测两个 OBB 包围盒是否相交或侵入安全缓冲走廊
        :param box1: 车辆/障碍物1
        :param box2: 车辆/障碍物2
        :param buffer: 安全冗余裕度 (m)
        :return: True 表示发生重叠或侵入缓冲带，False 表示完全解耦无碰撞
        """
        # 1. 粗筛：外接圆半径快速排斥
        dist = np.hypot(box1.center_x - box2.center_x, box1.center_y - box2.center_y)
        max_reach = 0.5 * (np.hypot(box1.length, box1.width) + np.hypot(box2.length, box2.width)) + buffer
        if dist > max_reach:
            return False

        # 2. 精筛：SAT 分离轴测试 (2D 矩形碰撞需测试 4 条分离轴)
        axes = box1.get_axes() + box2.get_axes()
        corners1 = np.array(box1.get_corners(), dtype=np.float64)
        corners2 = np.array(box2.get_corners(), dtype=np.float64)

        for axis in axes:
            # 投影4个顶点到当前分离轴
            proj1 = corners1 @ axis
            proj2 = corners2 @ axis

            min1, max1 = np.min(proj1), np.max(proj1)
            min2, max2 = np.min(proj2), np.max(proj2)

            # 判断两段一维投影线段是否存在分离轴空隙
            if (max1 + buffer < min2) or (max2 + buffer < min1):
                return False  # 找到至少一条分离轴，两物体必定无碰撞

        return True  # 所有分离轴上投影均重叠，判定为碰撞

    @staticmethod
    def _point_segment_distance(
        point: np.ndarray,
        seg_start: np.ndarray,
        seg_end: np.ndarray,
    ) -> float:
        """点到线段的最短欧氏距离。"""
        seg = seg_end - seg_start
        seg_len_sq = float(seg @ seg)
        if seg_len_sq < 1e-12:
            return float(np.linalg.norm(point - seg_start))
        ratio = float(np.clip((point - seg_start) @ seg / seg_len_sq, 0.0, 1.0))
        projection = seg_start + ratio * seg
        return float(np.linalg.norm(point - projection))

    @staticmethod
    def min_distance_between_boxes(box1: BoundingBox2D, box2: BoundingBox2D) -> float:
        """
        计算两个 OBB 包围盒之间的最短净距离 (m)。
        相交时返回 0.0；否则返回两条矩形边界之间的最小距离。
        """
        if CollisionChecker.check_obb_collision(box1, box2, buffer=0.0):
            return 0.0

        corners1 = np.array(box1.get_corners(), dtype=np.float64)
        corners2 = np.array(box2.get_corners(), dtype=np.float64)
        best = float("inf")
        for i in range(4):
            a0 = corners1[i]
            a1 = corners1[(i + 1) % 4]
            for j in range(4):
                b0 = corners2[j]
                b1 = corners2[(j + 1) % 4]
                best = min(
                    best,
                    CollisionChecker._point_segment_distance(a0, b0, b1),
                    CollisionChecker._point_segment_distance(a1, b0, b1),
                    CollisionChecker._point_segment_distance(b0, a0, a1),
                    CollisionChecker._point_segment_distance(b1, a0, a1),
                )
        return float(best)

    @staticmethod
    def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """计算两点欧氏距离"""
        return float(np.hypot(p1[0] - p2[0], p1[1] - p2[1]))
