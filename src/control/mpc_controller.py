"""
模型预测控制 (Model Predictive Control, MPC) 轨迹跟踪**接口骨架**。
v0.2 未实现 QP 求解，调用 ``compute_control`` 会抛出 NotImplementedError；
当前闭环请使用 LQRController。
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
        # 接口骨架预留：避免调用方误以为已经得到有效控制量
        raise NotImplementedError(
            "MPCController is an interface skeleton in v0.2; use LQRController."
        )
