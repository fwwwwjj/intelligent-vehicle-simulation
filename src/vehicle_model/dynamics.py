"""
智能车辆二自由度横侧向动力学数字建模 (Dynamic Vehicle Model)。

包含横向位移、横摆角速度 (yaw rate)、前后轮侧偏角与轮胎侧向力。
轮胎模型支持两种口径：

- ``tire_model="linear"``：``F_y = C_alpha * alpha``，保持与早期版本完全一致的
  线性轮胎行为，便于和旧实验做 A/B 对比；
- ``tire_model="pacejka"``：调用 :class:`src.vehicle_model.tire.PacejkaTireModel`
  的稳态魔术公式，并计入纵向载荷转移与简化摩擦椭圆组合滑移。

注意：Pacejka 分支是“稳态纯侧偏 + 简化组合滑移”的工程级实现，不含轮胎松弛
长度、温度/胎压与完整试验标定。文档中应据此表述，不能写成完整轮胎试验台模型。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from src.common.state import VehicleState
from src.vehicle_model.tire import PacejkaTireModel

_GRAVITY = 9.81


class DynamicBicycleModel:
    """
    智能车辆 2-DOF 动力学数字孪生模型。

    状态向量包含全局位姿与车身侧向速度/横摆角速度，控制量为纵向加速度与前轮转角。
    ``tire_model`` 决定轮胎力采用线性侧偏刚度还是 Pacejka 魔术公式。
    """

    def __init__(
        self,
        mass: float = 1600.0,
        iz: float = 2500.0,
        lf: float = 1.35,
        lr: float = 1.50,
        cf: float = 120000.0,
        cr: float = 120000.0,
        tire_model: str = "linear",
        pacejka_params: Optional[Dict[str, Any]] = None,
        friction_coefficient: float = 0.9,
        cg_height: float = 0.55,
    ):
        self.mass = mass            # 整车质量 m (kg)
        self.iz = iz                # 绕Z轴转动惯量 Iz (kg*m^2)
        self.lf = lf                # 质心至前轴距离 (m)
        self.lr = lr                # 质心至后轴距离 (m)
        self.cf = cf                # 前轮线性侧偏刚度 (N/rad)
        self.cr = cr                # 后轮线性侧偏刚度 (N/rad)
        self.friction_coefficient = float(friction_coefficient)
        self.cg_height = float(cg_height)
        self.wheelbase = float(lf) + float(lr)

        self.tire_model = str(tire_model).lower()
        if self.tire_model not in {"linear", "pacejka"}:
            raise ValueError(
                f"Unknown tire_model='{tire_model}'; expected 'linear' or 'pacejka'."
            )

        params = dict(pacejka_params or {})
        common = {
            "B": float(params.get("B", 10.0)),
            "C": float(params.get("C", 1.3)),
            "D_scale": float(params.get("D_scale", 0.95)),
            "E": float(params.get("E", -0.97)),
            "mu": float(params.get("mu", self.friction_coefficient)),
            "load_sensitivity": float(params.get("load_sensitivity", 0.0)),
            "fz_ref": float(params.get("fz_ref", 4000.0)),
        }
        front_cfg = dict(common)
        rear_cfg = dict(common)
        front_cfg.update(params.get("front", {}) or {})
        rear_cfg.update(params.get("rear", {}) or {})
        self.pacejka_front = PacejkaTireModel(**front_cfg)
        self.pacejka_rear = PacejkaTireModel(**rear_cfg)
        # 兼容早期接口：未显式给 mu 时仍按 D = 0.95 * Fz 的旧口径计算
        self._pacejka_reference = PacejkaTireModel()

    # ------------------------------------------------------------------
    # 轮胎模型
    # ------------------------------------------------------------------
    def set_tire_model(self, tire_model: str) -> None:
        """在仿真运行前切换轮胎模型。"""
        model = str(tire_model).lower()
        if model not in {"linear", "pacejka"}:
            raise ValueError(
                f"Unknown tire_model='{tire_model}'; expected 'linear' or 'pacejka'."
            )
        self.tire_model = model

    def pacejka_tire_model(self, slip_angle: float, fz: float = 4000.0) -> float:
        """
        Pacejka 魔术公式 (Magic Formula) 轮胎力学非线性模型（兼容旧接口）。

        Fy = D * sin(C * arctan(B * alpha - E * (B * alpha - arctan(B * alpha))))
        """
        return self._pacejka_reference.lateral_force(slip_angle, fz)

    def normal_loads(self, accel_cmd: float) -> tuple[float, float]:
        """
        计算前/后轴垂向载荷 (N)，含纵向载荷转移。

        ``a_x > 0``（加速）时前轴减载、后轴增载；制动时相反。
        """
        static_front = self.mass * _GRAVITY * self.lr / max(self.wheelbase, 1e-6)
        static_rear = self.mass * _GRAVITY * self.lf / max(self.wheelbase, 1e-6)
        transfer = self.mass * float(accel_cmd) * self.cg_height / max(self.wheelbase, 1e-6)
        front = max(0.05 * static_front, static_front - transfer)
        rear = max(0.05 * static_rear, static_rear + transfer)
        return float(front), float(rear)

    # ------------------------------------------------------------------
    # 动力学步进
    # ------------------------------------------------------------------
    def step(self, state: VehicleState, accel_cmd: float, steer_cmd: float, dt: float) -> VehicleState:
        """
        根据动力学微分方程更新下一时刻状态。

        :param state: 当前车辆状态
        :param accel_cmd: 期望纵向加速度 (m/s^2)
        :param steer_cmd: 前轮转角 (rad)
        :param dt: 仿真步长 (s)
        :return: 动力学积分后的新状态
        """
        vx = max(1.0, state.vx)
        vy = state.vy
        r = state.yaw_rate

        # 线性化轮胎侧偏角估计
        alpha_f = steer_cmd - (vy + self.lf * r) / vx
        alpha_r = -(vy - self.lr * r) / vx

        if self.tire_model == "pacejka":
            fz_front, fz_rear = self.normal_loads(accel_cmd)
            fx_total = self.mass * float(accel_cmd)
            load_sum = max(fz_front + fz_rear, 1e-6)
            fx_front = fx_total * fz_front / load_sum
            fx_rear = fx_total * fz_rear / load_sum
            fy_f = self.pacejka_front.combined_lateral_force(
                alpha_f, fz_front, fx_front
            )
            fy_r = self.pacejka_rear.combined_lateral_force(
                alpha_r, fz_rear, fx_rear
            )
        else:
            # 线性轮胎：保持与 v0.2 完全一致的公式与积分顺序
            fy_f = self.cf * alpha_f
            fy_r = self.cr * alpha_r

        # 二自由度动力学状态方程:
        # m * (vy_dot + vx * r) = Fy_f + Fy_r
        # Iz * r_dot = lf * Fy_f - lr * Fy_r
        vy_dot = (fy_f + fy_r) / self.mass - vx * r
        r_dot = (self.lf * fy_f - self.lr * fy_r) / self.iz

        # 状态递推
        new_vy = vy + vy_dot * dt
        new_r = r + r_dot * dt
        new_vx = max(0.0, vx + accel_cmd * dt)

        # 变换回全局笛卡尔坐标系
        heading = state.heading + new_r * dt
        heading = float(np.arctan2(np.sin(heading), np.cos(heading)))
        x = state.x + (new_vx * np.cos(heading) - new_vy * np.sin(heading)) * dt
        y = state.y + (new_vx * np.sin(heading) + new_vy * np.cos(heading)) * dt

        return VehicleState(
            x=x,
            y=y,
            heading=heading,
            velocity=float(np.hypot(new_vx, new_vy)),
            vx=new_vx,
            vy=new_vy,
            acceleration=accel_cmd,
            ax=accel_cmd,
            ay=vy_dot + vx * r,
            yaw_rate=new_r,
            steer_angle=steer_cmd,
            timestamp=state.timestamp + dt,
        )


__all__ = ["DynamicBicycleModel"]
