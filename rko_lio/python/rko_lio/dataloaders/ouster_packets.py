import logging
import math
from pathlib import Path
from typing import Optional
import numpy as np
from ..config import TimestampProcessingConfig
from ..util import error_and_exit, info

try:
    from rosbags.highlevel import AnyReader
except ModuleNotFoundError:
    error_and_exit(
        'rosbags library not installed for using rosbag dataloader, please install with "pip install -U rosbags"'
    )
from ..scoped_profiler import ScopedProfiler
from .utils.static_tf_tree import create_static_tf_tree, query_static_tf
from ouster.sdk.bag import BagPacketSource
from ouster.sdk.core import (
    PacketFormat,
    LidarPacket,
    ImuPacket,
    XYZLut,
    destagger,
)
from ouster.sdk.util.parsing import packets_to_scan


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mapping')

G_FORCE_ = 9.80665


class OusterPacketLoader:
    def __init__(
        self,
        data_path: Path,
        imu_frame_id: str | None,
        lidar_frame_id: str | None,
        base_frame_id: str | None,
        timestamp_processing_config: TimestampProcessingConfig,
        sensor_metadata: Optional[str] = None,
        *args,
        **kwargs,
    ):
        logger.info(f'Try loading Ouster record from: {data_path}')
        self.record_path = data_path
        self.num_lidar_scans = 0
        self.num_imu_msgs = 0
        # load bag/mcap with optional metadata
        data_source = BagPacketSource(
            bag_path=self.record_path, meta=sensor_metadata if sensor_metadata is not None else None
        )
        # create iterator
        self.packet_iter = iter(data_source)
        # read metadata from ouster sensor for further dynamic config
        self.ouster_metadata = data_source.sensor_info[0]
        self.packet_format = PacketFormat(self.ouster_metadata)
        self.lidar_packets_per_frame = int(
            self.ouster_metadata.format.columns_per_frame
            / self.ouster_metadata.format.columns_per_packet
        )
        # create XYZ converter to reduce overhead during iteration
        self.xyzlut = XYZLut(self.ouster_metadata)
        self.lidar_packet_buffer = []
        self.cur_lidar_frame_id = None
        self.T_imu_to_base = None
        self.T_lidar_to_base = None
        # TODO inject properly
        self.imu_frame_id = imu_frame_id
        self.lidar_frame_id = lidar_frame_id
        self.base_frame_id = base_frame_id or self.lidar_frame_id
        # additional bag reader, used for TF extraction
        self.bag = AnyReader([data_path])
        self.record_duration = 0

    def __iter__(self):
        return self

    def __next__(self):
        while True:
            with ScopedProfiler("Ouster Dataloader") as data_timer:

                idx, packet = next(self.packet_iter)
                if packet is None:
                    continue
                if isinstance(packet, ImuPacket):
                    return 'imu', self.read_imu_from_packet(packet)
                if isinstance(packet, LidarPacket):
                    cur_id = packet.frame_id()
                    if self.cur_lidar_frame_id is None:
                        self.cur_lidar_frame_id = cur_id
                    if cur_id == self.cur_lidar_frame_id:
                        self.lidar_packet_buffer.append(packet)
                    else:
                        # reset to new frameID
                        self.cur_lidar_frame_id = cur_id
                        return 'lidar', self.read_lidar_scan_from_packets()

    def __len__(self):
        return self.num_imu_msgs + self.num_lidar_scans

    @property
    def extrinsics(self):
        self.bag.open()
        if self.T_imu_to_base is None or self.T_lidar_to_base is None:
            info('Trying to obtain extrinsics from the data.')
            print('Building TF tree.')
            static_tf_tree = create_static_tf_tree(self.bag)
            if not static_tf_tree:
                error_and_exit(
                    "The rosbag doesn't contain a static tf tree, cannot query it for extrinsics. "
                    "Please specify the extrinsics manually in a config. "
                    "You can use 'rko_lio --dump_config' to dump a default config."
                )

            print('Querying TF tree for imu to base extrinsic.')
            self.T_imu_to_base = query_static_tf(
                static_tf_tree, self.imu_frame_id, self.base_frame_id
            )
            print('Querying TF tree for lidar to base extrinsic.')
            self.T_lidar_to_base = query_static_tf(
                static_tf_tree, self.lidar_frame_id, self.base_frame_id
            )
        # additionaly extract msg count from record
        self.num_imu_msgs = self.bag.topics['/ouster/imu_packets'].msgcount
        self.num_lidar_scans = int(
            self.bag.topics['/ouster/lidar_packets'].msgcount / self.lidar_packets_per_frame
        )
        self.record_duration = self.bag.duration
        self.bag.close()
        return self.T_imu_to_base, self.T_lidar_to_base

    def read_imu_from_packet(self, imu_packet: ImuPacket):
        # convert timestamp from nanosec to seconds
        timestamp = imu_packet.gyro_ts() / 1e9
        # convert linear acceleration from g to m/s²
        accel = [
            self.packet_format.imu_la_x(imu_packet.buf) * G_FORCE_,
            self.packet_format.imu_la_y(imu_packet.buf) * G_FORCE_,
            self.packet_format.imu_la_z(imu_packet.buf) * G_FORCE_,
        ]
        # convert angular velocity from deg/s to rad/s
        gyro = [
            math.radians(self.packet_format.imu_av_x(imu_packet.buf)),
            math.radians(self.packet_format.imu_av_y(imu_packet.buf)),
            math.radians(self.packet_format.imu_av_z(imu_packet.buf)),
        ]
        return timestamp, accel, gyro

    def read_lidar_scan_from_packets(self):
        # convert multiple lidar packets to single scan
        lidar_scan = packets_to_scan(self.lidar_packet_buffer, self.ouster_metadata)
        # project scan into 3D cartesian coordinates
        xyz_destaggered = destagger(self.ouster_metadata, self.xyzlut(lidar_scan))
        # TODO: filter by range

        # extract timestamps and convert from nanosec to seconds
        timestamps = lidar_scan.timestamp / 1e9
        # create "flatten" array
        points = xyz_destaggered.reshape(-1, 3)
        # adjust timestamps data to points
        timestamps_flat = (
            np.repeat(timestamps, self.ouster_metadata.format.pixels_per_column)
            .reshape(
                self.ouster_metadata.format.pixels_per_column,
                self.ouster_metadata.format.columns_per_frame,
            )
            .T.reshape(-1)
        )
        # clear buffer for new frame
        self.lidar_packet_buffer.clear()
        return points, timestamps_flat

    def __repr__(self):
        return f'Ouster UDP Packet Loader..'
