from typing import Optional
from pydantic import BaseModel


class RosBagDataloaderConfig(BaseModel):
    imu_topic: Optional[str] = None
    imu_frame_id: Optional[str] = None
    lidar_topic: Optional[str] = None
    lidar_frame_id: Optional[str] = None
    base_frame_id: Optional[str] = None


class HeliPRDataloaderConfig(BaseModel):
    sequence: Optional[str] = None


class OusterPacketsDataLoaderConfig(BaseModel):
    metadata: Optional[str] = None


class DataLoaderConfig(BaseModel):
    data_path: Optional[str] = None
    name: Optional[str] = None
    rosbag_cfg: RosBagDataloaderConfig = RosBagDataloaderConfig()
    helipr_cfg: HeliPRDataloaderConfig = HeliPRDataloaderConfig()
    ouster_cfg: OusterPacketsDataLoaderConfig = OusterPacketsDataLoaderConfig()
