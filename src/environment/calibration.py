"""
微观交通流参数标定与系统辨识工具链 (Microscopic Traffic Flow Calibration Pipeline)。

支持从实车采集或数据集（如 highD / NGSIM）跟驰时空轨迹中，
反演辨识智能驾驶员模型 (IDM) 的关键机理参数 (v0, s0, T, a_max, b_desired)。
目标函数采用观测加速度与 IDM 预测加速度的均方根误差 (RMSE)。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from src.environment.obstacle_manager import IntelligentDriverModel

try:
    from scipy.optimize import minimize  # noqa: F401
    _HAS_SCIPY = True
except Exception:  # pragma: no cover
    _HAS_SCIPY = False


DEFAULT_IDM_BOUNDS: Dict[str, Tuple[float, float]] = {
    "v0": (10.0, 45.0),         # 期望速度 (m/s)
    "s0": (1.0, 10.0),          # 静止安全间距 (m)
    "T": (0.5, 3.5),            # 安全车头时距 (s)
    "a_max": (0.5, 4.0),        # 最大舒适加速度 (m/s^2)
    "b_desired": (0.5, 4.0),    # 期望舒适减速度 (m/s^2)
}


class IDMCalibrator:
    """
    IDM 跟驰模型参数辨识器。
    通过非线性最小二乘优化拟合目标车辆行为特征。
    """

    def __init__(
        self,
        bounds: Optional[Dict[str, Tuple[float, float]]] = None,
        loss_type: str = "rmse",
    ):
        self.bounds = bounds or dict(DEFAULT_IDM_BOUNDS)
        self.loss_type = loss_type

    @staticmethod
    def compute_rmse(
        idm: IntelligentDriverModel,
        samples: List[Dict[str, float]],
    ) -> float:
        """计算给定样本集下的加速度预测均方根误差 (RMSE)。"""
        if not samples:
            return 0.0
        sq_err_sum = 0.0
        for item in samples:
            pred_a = idm.compute_acceleration(
                current_speed=item["current_speed"],
                lead_speed=item["lead_speed"],
                distance=max(0.1, item["gap"]),
            )
            obs_a = item["observed_accel"]
            sq_err_sum += (pred_a - obs_a) ** 2
        return float(math.sqrt(sq_err_sum / len(samples)))

    def calibrate(
        self,
        samples: List[Dict[str, float]],
        initial_params: Optional[Dict[str, float]] = None,
    ) -> Dict[str, Any]:
        """
        根据给定的跟驰观测样本拟合最优 IDM 参数。

        :param samples: 观测样本列表，每个元素包含:
               'current_speed', 'lead_speed', 'gap', 'observed_accel'
        :param initial_params: 初始猜测参数字典
        :return: 包含标定结果、初始与最终 RMSE 的字典
        """
        if not samples:
            raise ValueError("Cannot calibrate IDM parameters on empty samples.")

        init = {
            "v0": 25.0,
            "s0": 3.0,
            "T": 1.5,
            "a_max": 2.0,
            "b_desired": 2.0,
        }
        if initial_params:
            init.update(initial_params)

        keys = ["v0", "s0", "T", "a_max", "b_desired"]
        x0 = [init[k] for k in keys]
        bnds = [self.bounds[k] for k in keys]

        initial_idm = IntelligentDriverModel(**init)
        initial_rmse = self.compute_rmse(initial_idm, samples)

        if _HAS_SCIPY:
            def objective(p: List[float]) -> float:
                model = IntelligentDriverModel(
                    v0=p[0], s0=p[1], T=p[2], a_max=p[3], b_desired=p[4]
                )
                return self.compute_rmse(model, samples)

            res = minimize(
                objective,
                x0=x0,
                bounds=bnds,
                method="L-BFGS-B",
                options={"maxiter": 100, "ftol": 1e-5},
            )
            success = bool(res.success)
            best_x = res.x
        else:
            # 无 scipy 时的零依赖回退：多轮坐标轮换 + 逐轮收缩搜索区间。
            # 单轮 9 点网格在参数强耦合时容易停在 RMSE≈0.7 的粗解，因此这里
            # 每轮围绕当前最优点做局部细化，直到不再改进或达到轮数上限。
            best_x = list(x0)
            best_rmse = initial_rmse
            for round_idx in range(6):
                improved = False
                for idx in range(len(keys)):
                    low, high = bnds[idx]
                    span = (high - low) / (2.0 ** min(round_idx, 4))
                    local_low = max(low, best_x[idx] - span)
                    local_high = min(high, best_x[idx] + span)
                    for val in np.linspace(local_low, local_high, 25):
                        cand = list(best_x)
                        cand[idx] = float(val)
                        m = IntelligentDriverModel(
                            v0=cand[0], s0=cand[1], T=cand[2], a_max=cand[3], b_desired=cand[4]
                        )
                        rmse = self.compute_rmse(m, samples)
                        if rmse < best_rmse - 1e-12:
                            best_rmse = rmse
                            best_x = cand
                            improved = True
                if not improved:
                    break
            success = True

        opt_params = {keys[i]: float(best_x[i]) for i in range(len(keys))}
        final_idm = IntelligentDriverModel(**opt_params)
        final_rmse = self.compute_rmse(final_idm, samples)

        return {
            "success": success,
            "calibrated_params": opt_params,
            "initial_rmse": initial_rmse,
            "final_rmse": final_rmse,
            "sample_count": len(samples),
        }


def extract_following_pairs_from_arrays(
    time_series_self_x: np.ndarray,
    time_series_self_v: np.ndarray,
    time_series_lead_x: np.ndarray,
    time_series_lead_v: np.ndarray,
    dt: float,
    veh_length_sum: float = 9.6,
) -> List[Dict[str, float]]:
    """
    从主车与前车的逐帧时序数组中提取规范跟驰样本。
    """
    n = min(len(time_series_self_x), len(time_series_lead_x))
    samples = []
    for k in range(n - 1):
        gap = float(time_series_lead_x[k] - time_series_self_x[k] - veh_length_sum)
        if gap <= 0.5:
            continue
        cur_v = float(time_series_self_v[k])
        lead_v = float(time_series_lead_v[k])
        obs_a = float((time_series_self_v[k + 1] - time_series_self_v[k]) / max(1e-4, dt))
        samples.append({
            "current_speed": cur_v,
            "lead_speed": lead_v,
            "gap": gap,
            "observed_accel": obs_a,
        })
    return samples
