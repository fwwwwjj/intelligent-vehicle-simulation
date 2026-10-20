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
        dx = box1.center_x - box2.center_x
        dy = box1.center_y - box2.center_y
        dist_sq = dx * dx + dy * dy
        r1 = 0.5 * np.hypot(box1.length, box1.width)
        r2 = 0.5 * np.hypot(box2.length, box2.width)
        max_reach = r1 + r2 + buffer
        if dist_sq > max_reach * max_reach:
            return False

        # 2. 精筛：SAT 分离轴测试 (以标量投影计算 4 条正交分离轴，规避 numpy 小数组开销)
        c1 = float(np.cos(box1.heading))
        s1 = float(np.sin(box1.heading))
        hl1 = 0.5 * box1.length
        hw1 = 0.5 * box1.width

        c2 = float(np.cos(box2.heading))
        s2 = float(np.sin(box2.heading))
        hl2 = 0.5 * box2.length
        hw2 = 0.5 * box2.width

        # 轴 1：box1 的纵轴 [c1, s1]
        proj1 = abs(dx * c1 + dy * s1)
        r2_proj = hl2 * abs(c2 * c1 + s2 * s1) + hw2 * abs(-s2 * c1 + c2 * s1)
        if proj1 > hl1 + r2_proj + buffer:
            return False

        # 轴 2：box1 的横轴 [-s1, c1]
        proj2 = abs(-dx * s1 + dy * c1)
        r2_proj = hl2 * abs(-c2 * s1 + s2 * c1) + hw2 * abs(s2 * s1 + c2 * c1)
        if proj2 > hw1 + r2_proj + buffer:
            return False

        # 轴 3：box2 的纵轴 [c2, s2]
        proj3 = abs(dx * c2 + dy * s2)
        r1_proj = hl1 * abs(c1 * c2 + s1 * s2) + hw1 * abs(-s1 * c2 + c1 * s2)
        if proj3 > hl2 + r1_proj + buffer:
            return False

        # 轴 4：box2 的横轴 [-s2, c2]
        proj4 = abs(-dx * s2 + dy * c2)
        r1_proj = hl1 * abs(-c1 * s2 + s1 * c2) + hw1 * abs(s1 * s2 + c1 * c2)
        if proj4 > hw2 + r1_proj + buffer:
            return False

        return True  # 所有分离轴上投影均重叠，判定为碰撞

    @staticmethod
    def _scalar_point_segment_dist_sq(
        px: float, py: float, x1: float, y1: float, x2: float, y2: float
    ) -> float:
        """标量数学计算点到线段最短距离平方。"""
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            return (px - x1) ** 2 + (py - y1) ** 2
        t = ((px - x1) * dx + (py - y1) * dy) / l2
        if t <= 0.0:
            return (px - x1) ** 2 + (py - y1) ** 2
        elif t >= 1.0:
            return (px - x2) ** 2 + (py - y2) ** 2
        else:
            nx = x1 + t * dx
            ny = y1 + t * dy
            return (px - nx) ** 2 + (py - ny) ** 2

    @staticmethod
    def _point_segment_distance(
        point: np.ndarray,
        seg_start: np.ndarray,
        seg_end: np.ndarray,
    ) -> float:
        """点到线段的最短欧氏距离（保持外部接口兼容）。"""
        px, py = float(point[0]), float(point[1])
        x1, y1 = float(seg_start[0]), float(seg_start[1])
        x2, y2 = float(seg_end[0]), float(seg_end[1])
        return float(np.sqrt(CollisionChecker._scalar_point_segment_dist_sq(px, py, x1, y1, x2, y2)))

    @staticmethod
    def min_distance_between_boxes(box1: BoundingBox2D, box2: BoundingBox2D) -> float:
        """
        计算两个 OBB 包围盒之间的最短净距离 (m)。
        相交时返回 0.0；否则返回两条矩形边界之间的最小距离。
        """
        if CollisionChecker.check_obb_collision(box1, box2, buffer=0.0):
            return 0.0

        c1 = box1.get_corners()
        c2 = box2.get_corners()
        min_d_sq = float("inf")

        # 顶点到边投影（32 次标量计算，代替逐段 numpy 向量化）
        for px, py in c1:
            for j in range(4):
                x1, y1 = c2[j]
                x2, y2 = c2[(j + 1) % 4]
                d_sq = CollisionChecker._scalar_point_segment_dist_sq(px, py, x1, y1, x2, y2)
                if d_sq < min_d_sq:
                    min_d_sq = d_sq

        for px, py in c2:
            for i in range(4):
                x1, y1 = c1[i]
                x2, y2 = c1[(i + 1) % 4]
                d_sq = CollisionChecker._scalar_point_segment_dist_sq(px, py, x1, y1, x2, y2)
                if d_sq < min_d_sq:
                    min_d_sq = d_sq

        return float(np.sqrt(min_d_sq))

    @staticmethod
    def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """计算两点欧氏距离"""
        return float(np.hypot(p1[0] - p2[0], p1[1] - p2[1]))
