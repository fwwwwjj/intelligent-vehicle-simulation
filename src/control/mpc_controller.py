"""
模型预测控制 (Model Predictive Control, MPC) 轨迹跟踪接口。
在滚动有限时域内显式处理非线性车辆执行机构极限与动力学不等式约束。
"""

from src.common.state import VehicleState, Trajectory
from src.control.base_controller import BaseController, ControlCommand


class MPCController(BaseController):
    """
    MPC 轨迹跟踪控制器
    构建带硬约束/软约束的二次凸优化 (QP) 跟踪命题
    """
    def __init__(self, prediction_horizon: int = 15, control_horizon: int = 10):
        self.Np = prediction_horizon
        self.Nc = control_horizon

    def compute_control(
        self,
        current_state: VehicleState,
        target_trajectory: Trajectory,
        dt: float
    ) -> ControlCommand:
        """
        滚动时域求解最优控制序列并提取首个控制量
        """
        # 接口骨架预留
        return ControlCommand(steering_angle=0.0, acceleration=0.0)
