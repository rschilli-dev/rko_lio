from pathlib import Path
from typing import List, Optional

import numpy as np
from pydantic import BaseModel, ConfigDict, Field
from pydantic_settings import BaseSettings
from rko_lio.config.dataloader_config import DataLoaderConfig
from rko_lio.util import quat_xyzw_xyz_to_transform
from rko_lio import rko_lio_pybind


class TimestampProcessingConfig(BaseModel):
    multiplier_to_seconds: float = 0.0
    force_absolute: bool = False
    force_relative: bool = False
    absolute_start_threshold: int = 1  # ms
    absolute_end_threshold: int = 1  # ms
    relative_start_threshold: int = 10  # ms
    relative_end_threshold: int = 10  # ms

    def to_cpp(self) -> rko_lio_pybind._TimestampProcessingConfig:
        cpp_cfg = rko_lio_pybind._TimestampProcessingConfig()
        for k, v in self.model_dump().items():
            setattr(cpp_cfg, k, v)
        return cpp_cfg


class LIOConfig(BaseModel):
    deskew: bool = True
    max_iterations: int = 100
    voxel_size: float = 1.0
    max_points_per_voxel: int = 20
    max_range: float = 100.0
    min_range: float = 1.0
    convergence_criterion: float = 1e-5
    max_correspondance_distance: float = 0.5
    max_num_threads: int = 0
    initialization_phase: bool = False
    max_expected_jerk: float = 3.0
    double_downsample: bool = True
    min_beta: float = 200.0

    def to_cpp(self) -> rko_lio_pybind._LIOConfig:
        cpp_cfg = rko_lio_pybind._LIOConfig()
        for k, v in self.model_dump().items():
            setattr(cpp_cfg, k, v)
        return cpp_cfg


class TransformConfig(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    extrinsic_imu2base_quat_xyzw_xyz: Optional[List] = None
    extrinsic_imu2base: Optional[np.ndarray] = Field(exclude=True, default=None)
    extrinsic_lidar2base_quat_xyzw_xyz: Optional[List] = None
    extrinsic_lidar2base: Optional[np.ndarray] = Field(exclude=True, default=None)
    tf_file: Optional[str] = None

    def init_tf(self):
        if self.extrinsic_imu2base_quat_xyzw_xyz is not None:
            self.extrinsic_imu2base = quat_xyzw_xyz_to_transform(
                np.asarray(self.extrinsic_imu2base_quat_xyzw_xyz)
            )
        if self.extrinsic_lidar2base_quat_xyzw_xyz is not None:
            self.extrinsic_lidar2base = quat_xyzw_xyz_to_transform(
                np.asarray(self.extrinsic_lidar2base_quat_xyzw_xyz)
            )


class LoggingConfig(BaseModel):
    log_results: bool = False
    log_dir: str = str(Path('results').resolve().as_posix())
    dump_deskewed_scans: Optional[bool] = False
    run_name: Optional[str] = None


class PipelineConfig(BaseSettings):
    data_loader_cfg: DataLoaderConfig = DataLoaderConfig()
    lio_cfg: LIOConfig = LIOConfig()
    tsp_cfg: TimestampProcessingConfig = TimestampProcessingConfig()
    tf_cfg: TransformConfig = TransformConfig()
    log_cfg: LoggingConfig = LoggingConfig()
    viz: bool = False
    reset_viz: bool = True
    viz_every_n_frames: int = 20
