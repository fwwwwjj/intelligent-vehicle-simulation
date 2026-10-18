"""
通用数据结构与几何数学模块
"""

from src.common.state import VehicleState, TrajectoryPoint, Trajectory, ObstacleState, ObstacleType
from src.common.geometry import BoundingBox2D, Point2D, CollisionChecker
from src.common.frenet_frame import FrenetPoint, FrenetConverter, ReferencePoint

__all__ = [
    "VehicleState",
    "TrajectoryPoint",
    "Trajectory",
    "ObstacleState",
    "ObstacleType",
    "BoundingBox2D",
    "Point2D",
    "CollisionChecker",
    "FrenetPoint",
    "FrenetConverter",
    "ReferencePoint",
]
