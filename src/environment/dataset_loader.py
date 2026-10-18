"""
真实道路轨迹数据集与场景描述文件加载器的**接口骨架**。
定义 NGSIM、highD、INTERACTION 及 OpenSCENARIO 的抽象规范；
当前仓库未附带原始数据，loader 只返回元数据/空列表，不能据此声称已完成
“真实数据驱动标定”。
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Any, Optional
import pandas as pd
from src.common.state import ObstacleState, ObstacleType


class BaseDatasetLoader(ABC):
    """数据集加载器抽象基类"""

    @abstractmethod
    def load_scenario(self, file_path: str, scenario_id: Optional[str] = None) -> Dict[str, Any]:
        """
        加载场景并解析为仿真环境可执行的状态序列字典
        :param file_path: 数据集文件路径
        :param scenario_id: 场景切片标识
        :return: 包含车道线、动态背景车时序轨迹的元数据字典
        """
        pass

    @abstractmethod
    def get_frame_obstacles(self, timestamp: float) -> List[ObstacleState]:
        """按时间戳提取当前帧的所有障碍物真实物理量"""
        pass


class HighDDatasetLoader(BaseDatasetLoader):
    """
    德国高速公路 highD 数据集解析器**接口骨架**。

    当前仓库未附带 highD 原始数据，``load_scenario`` 只返回元数据，
    ``get_frame_obstacles`` 返回空列表。接入真实数据前，不得在文档中声称
    “已用 highD 标定 IDM”。
    """
    def __init__(self, data_root: str = "data/raw/highD"):
        self.data_root = data_root
        self.tracks_data: Optional[pd.DataFrame] = None

    def load_scenario(self, file_path: str, scenario_id: Optional[str] = None) -> Dict[str, Any]:
        # 预留 highD CSV 解析接口
        return {
            "dataset_name": "highD",
            "frame_rate": 25.0,
            "total_tracks": 0,
            "scenario_metadata": {}
        }

    def get_frame_obstacles(self, timestamp: float) -> List[ObstacleState]:
        # 接口实现骨架
        return []


class InteractionDatasetLoader(BaseDatasetLoader):
    """
    INTERACTION 数据集解析器**接口骨架**（未读取实际数据文件）。
    """
    def __init__(self, data_root: str = "data/raw/INTERACTION"):
        self.data_root = data_root

    def load_scenario(self, file_path: str, scenario_id: Optional[str] = None) -> Dict[str, Any]:
        return {
            "dataset_name": "INTERACTION",
            "type": "roundabout_or_intersection",
            "scenario_id": scenario_id
        }

    def get_frame_obstacles(self, timestamp: float) -> List[ObstacleState]:
        return []


class OpenScenarioParser:
    """
    符合 ASAM OpenSCENARIO 2.0 / 1.x 国际标准的仿真测试场景描述解析器
    """
    @staticmethod
    def parse_xosc(file_path: str) -> Dict[str, Any]:
        """解析 XML 场景逻辑文件（当前仅占位，未实现 XML 解析）。"""
        return {
            "format": "OpenSCENARIO",
            "path": file_path,
            "status": "interface_only",
        }
