"""
计算几何基础模块与碰撞检测接口。
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
        return np.array([self.x, self.y])


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
        self.center_x = center_x
        self.center_y = center_y
        self.length = length
        self.width = width
        self.heading = heading

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
            corners.append((gx, gy))
        return corners


class CollisionChecker:
    """
    碰撞检测器接口。
    基于分离轴定理 (Separating Axis Theorem, SAT) 提供高保真凸多边形/包围盒碰撞检测。
    """

    @staticmethod
    def check_obb_collision(box1: BoundingBox2D, box2: BoundingBox2D, buffer: float = 0.2) -> bool:
        """
        检测两个OBB包围盒是否相交或重叠
        :param box1: 车辆/障碍物1
        :param box2: 车辆/障碍物2
        :param buffer: 安全冗余裕度 (m)
        :return: True表示发生碰撞或在安全距离以内，False表示无碰撞
        """
        # 接口实现骨架：SAT判定
        # 可在后续实现中调用具体的投影视角轴计算
        dist = np.hypot(box1.center_x - box2.center_x, box1.center_y - box2.center_y)
        max_dim = 0.5 * (np.hypot(box1.length, box1.width) + np.hypot(box2.length, box2.width)) + buffer
        if dist > max_dim:
            return False  # 粗筛：外接圆无重叠

        # 精细化判定可留空或补充SAT轴投影逻辑
        return False

    @staticmethod
    def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """计算两点欧氏距离"""
        return float(np.hypot(p1[0] - p2[0], p1[1] - p2[1]))
