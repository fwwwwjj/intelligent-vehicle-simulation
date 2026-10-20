"""
Pacejka 魔术公式 (Magic Formula) 轮胎力学模型。

稳态侧向力公式（Pacejka, *Tire and Vehicle Dynamics*, 2005）：

    F_y = D * sin(C * arctan(B * alpha - E * (B * alpha - arctan(B * alpha))))

其中 ``alpha`` 为轮胎侧偏角 (rad)，``D`` 为峰值侧向力，通常取
``D = mu * F_z * D_scale``。本模块只实现稳态纯侧偏 + 简化摩擦椭圆组合滑移，
不包含瞬态松弛长度、温度/胎压变化与载荷敏感性的完整标定；因此文档中应表述为
“Pacejka 稳态魔术公式已进入 2-DOF 动力学主回路”，而不是“完整轮胎试验台模型”。

该模型由 :class:`src.vehicle_model.dynamics.DynamicBicycleModel` 在
``tire_model="pacejka"`` 时调用，默认 ``tire_model="linear"`` 保持原有线性轮胎
行为，便于 A/B 对比。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class PacejkaTireModel:
    """
    单轴稳态 Pacejka 侧向轮胎模型。

    参数
    ----
    B, C, D_scale, E:
        魔术公式形状/峰值/曲率参数。默认值与仓库早期
        ``DynamicBicycleModel.pacejka_tire_model`` 保持同一量级。
    mu:
        名义路面附着系数；``lateral_force`` 可通过 ``mu_override`` 覆盖。
    load_sensitivity:
        简化载荷敏感性：``D *= (1 - load_sensitivity * (Fz/Fz_ref - 1))``，
        默认 0 表示不启用，避免在未标定数据上引入伪精度。
    fz_ref:
        载荷敏感性参考垂向载荷 (N)。
    """

    B: float = 10.0
    C: float = 1.3
    D_scale: float = 0.95
    E: float = -0.97
    mu: float = 1.0
    load_sensitivity: float = 0.0
    fz_ref: float = 4000.0

    def peak_force(self, normal_load: float, mu_override: Optional[float] = None) -> float:
        """返回给定垂向载荷下的峰值侧向力 D (N)。"""
        mu = self.mu if mu_override is None else float(mu_override)
        fz = max(0.0, float(normal_load))
        d = self.D_scale * max(0.0, mu) * fz
        if self.load_sensitivity > 0.0 and self.fz_ref > 1e-6:
            d *= max(0.05, 1.0 - self.load_sensitivity * (fz / self.fz_ref - 1.0))
        return float(d)

    def lateral_force(
        self,
        slip_angle: float,
        normal_load: float,
        mu_override: Optional[float] = None,
    ) -> float:
        """
        纯侧偏工况下的稳态侧向力 (N)。

        :param slip_angle: 轮胎侧偏角 (rad)，左偏/逆时针为正
        :param normal_load: 轮胎垂向载荷 (N)
        :param mu_override: 可选路面附着系数覆盖值
        :return: 侧向力 (N)，方向与侧偏角同号
        """
        alpha = float(slip_angle)
        d = self.peak_force(normal_load, mu_override)
        if d <= 1e-9:
            return 0.0
        b_alpha = self.B * alpha
        inner = b_alpha - self.E * (b_alpha - np.arctan(b_alpha))
        return float(d * np.sin(self.C * np.arctan(inner)))

    def combined_lateral_force(
        self,
        slip_angle: float,
        normal_load: float,
        longitudinal_force: float,
        mu_override: Optional[float] = None,
    ) -> float:
        """
        简化摩擦椭圆：纵向力占用一部分附着后，剩余侧向力上限为
        ``sqrt((mu Fz)^2 - Fx^2)``。用于避免“急加速/急制动同时满侧向力”
        的非物理组合。
        """
        mu = self.mu if mu_override is None else float(mu_override)
        fz = max(0.0, float(normal_load))
        fy = self.lateral_force(slip_angle, fz, mu_override=mu_override)
        max_total = max(0.0, mu) * fz
        if max_total <= 1e-9:
            return 0.0
        fx = float(np.clip(longitudinal_force, -max_total, max_total))
        max_lat = float(np.sqrt(max(0.0, max_total * max_total - fx * fx)))
        return float(np.clip(fy, -max_lat, max_lat))

    def slip_angle_at_peak(self) -> float:
        """近似峰值侧偏角 (rad)，用于测试/可视化，不参与控制。"""
        # 对 Magic Formula 做一维扫描，避免写死与 B/C/E 不一致的经验值
        grid = np.linspace(0.0, 0.35, 701)
        forces = np.array([abs(self.lateral_force(a, self.fz_ref, mu_override=1.0)) for a in grid])
        if forces.size == 0:
            return 0.0
        return float(grid[int(np.argmax(forces))])


__all__ = ["PacejkaTireModel"]
