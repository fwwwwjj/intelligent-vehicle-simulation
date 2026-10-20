"""
车辆状态、轨迹点与障碍物状态基础数据结构定义。
提供整车状态描述、离散轨迹时空节点及动态交通参与者的数据规范。
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class ObstacleType(Enum):
    """障碍物类别枚举"""
    UNKNOWN = 0
    PEDESTRIAN = 1
    BICYCLE = 2
    CAR = 3
    TRUCK = 4
    STATIC = 5


@dataclass
class VehicleState:
    """
    智能车辆瞬时物理状态 (Cartesian 笛卡尔坐标系下)
    包含运动学量与动力学量。
    """
    x: float = 0.0                     # 全局 X 坐标 (m)
    y: float = 0.0                     # 全局 Y 坐标 (m)
    heading: float = 0.0               # 航向角 yaw (rad), 沿X轴正方向为0, 逆时针为正
    velocity: float = 0.0              # 标量速度 (m/s)
    vx: float = 0.0                    # 车身纵向速度 (m/s)
    vy: float = 0.0                    # 车身横向速度 (m/s)
    acceleration: float = 0.0          # 标量加速度 (m/s^2)
    ax: float = 0.0                    # 纵向加速度 (m/s^2)
    ay: float = 0.0                    # 横向加速度 (m/s^2)
    yaw_rate: float = 0.0              # 横摆角速度 r / d(yaw)/dt (rad/s)
    steer_angle: float = 0.0           # 前轮偏角 delta (rad)
    timestamp: float = 0.0             # 状态对应时间戳 (s)


@dataclass
class TrajectoryPoint:
    """
    离散时空轨迹规划点定义
    包含几何属性、运动学属性以及高阶导数(曲率、加加速度)
    """
    x: float                           # 空间 X (m)
    y: float                           # 空间 Y (m)
    theta: float                       # 路径切线方位角 (rad)
    kappa: float = 0.0                 # 曲率 (1/m)
    dkappa: float = 0.0                # 曲率对弧长的导数 (1/m^2)
    v: float = 0.0                     # 目标线速度 (m/s)
    a: float = 0.0                     # 目标切向加速度 (m/s^2)
    jerk: float = 0.0                  # 目标加加速度 (m/s^3)
    t: float = 0.0                     # 到达时间 (s)
    s: float = 0.0                     # 沿参考路径的累积弧长 (m)


@dataclass
class Trajectory:
    """
    连续或离散规划轨迹序列
    """
    points: List[TrajectoryPoint] = field(default_factory=list)
    total_cost: float = 0.0            # 综合优化代价
    valid: bool = True                 # 是否通过碰撞与动力学约束检验


@dataclass
class ObstacleState:
    """
    动态交通参与者状态描述
    """
    id: str                            # 障碍物全局唯一ID
    obstacle_type: ObstacleType        # 类别
    x: float                           # 质心 X (m)
    y: float                           # 质心 Y (m)
    length: float = 4.8                # 几何长度 (m)
    width: float = 1.9                 # 几何宽度 (m)
    heading: float = 0.0               # 航向角 (rad)
    vx: float = 0.0                    # 速度 X 分量 (m/s)
    vy: float = 0.0                    # 速度 Y 分量 (m/s)
    predicted_trajectory: Optional[List[TrajectoryPoint]] = None # 未来时域预测轨迹
