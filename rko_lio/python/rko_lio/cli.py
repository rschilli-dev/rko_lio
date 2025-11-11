# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Entrypoint typer application for the python wrapper.
"""

from pathlib import Path
from rko_lio.config.pipeline_config import PipelineConfig

import typer
import yaml
from .util import (
    error_and_exit,
    info,
    warning,
)


def version_callback(value: bool):
    if value:
        from importlib.metadata import version

        rko_lio_version = version("rko_lio")
        info("RKO_LIO Version:", rko_lio_version)
        raise typer.Exit(0)


def dump_config_callback():

    with open('config.yaml', 'w') as f:
        dumping_cfg = PipelineConfig().model_dump()
        yaml.dump(dumping_cfg, f, default_flow_style=False, sort_keys=False)
    info(
        "Default config dumped to config.yaml. Note that the extrinsics are left as an empty list. If you need them, you need to specify them as \[qx, qy, qz, qw, x, y, z]. Delete all the keys you don't need."
    )
    raise typer.Exit(0)


def dataloader_name_callback(value: str):
    from .dataloaders import available_dataloaders

    if not value:
        return value
    dl = available_dataloaders()
    if value.lower() not in [d.lower() for d in dl]:
        raise typer.BadParameter(f"Supported dataloaders are: {', '.join(dl)}")
    for d in dl:
        if value.lower() == d.lower():
            return d
    return value


app = typer.Typer()


@app.command(
    epilog='Please open an issue on https://github.com/PRBonn/rko_lio if the usage of any option is unclear or you need some help!'
)
def pipeline(
    ctx: typer.Context,
    data_path: Path = typer.Option(
        None,
        '--data_path',
        exists=True,
        help='Path to data folder',
        # file_okay=False,
        dir_okay=True,
        readable=True,
    ),
    dataloader_name: str | None = typer.Option(
        None,
        '--dataloader',
        '-d',
        help='Specify a dataloader: [rosbag, raw, helipr]. Leave empty to guess one',
        show_choices=True,
        callback=dataloader_name_callback,
        case_sensitive=False,
    ),
    viz: bool = typer.Option(
        False,
        '--viz',
        '-v',
        help='Enable Rerun visualization',
        rich_help_panel='Visualisation options',
    ),
    viz_every_n_frames: int = typer.Option(
        None,
        '--viz_frame_skip',
        help='Publish (rerun) LiDAR information after specified number of frames. A low value will slow down the entire pipeline as logging LiDAR data is expensive.',
        rich_help_panel='Visualisation options',
    ),
    rbl_path: Path | None = typer.Option(
        None,
        '--rbl',
        exists=True,
        help='Path to a rerun blueprint file (.rbl). Leave empty to use the default rerun configuration. Respects --no_reset_viz if set',
        file_okay=True,
        dir_okay=False,
        readable=True,
        rich_help_panel='Visualisation options',
    ),
    reset_viz: bool = typer.Option(
        None,
        ' /--no_reset_viz',
        help='Pass this option to disable resetting rerun viewer configuration as per the blueprint (default or with --rbl). Useful if you want to take advantage of rerun`s caching behaviour.',
        show_default=False,
        rich_help_panel='Visualisation options',
    ),
    log_results: bool = typer.Option(
        None,
        '--log',
        '-l',
        help='Log trajectory results to disk at `log_dir` on completion',
        rich_help_panel='Disk logging options',
    ),
    log_dir: Path | None = typer.Option(
        None,
        '--log_dir',
        '-o',
        help='Where to dump LIO results if logging. If unspecified, and logging is enabled, a folder `results` will be created in the current directory.',
        file_okay=False,
        dir_okay=True,
        writable=True,
        rich_help_panel='Disk logging options',
    ),
    run_name: str | None = typer.Option(
        None,
        '--run_name',
        '-n',
        help='Name prefix for output files if logging. Leave empty to take the name from the data_path argument',
        rich_help_panel='Disk logging options',
    ),
    dump_deskewed_scans: bool = typer.Option(
        None,
        '--dump_deskewed',
        help='Dump each deskewed/motion-undistorted scan as a .ply file under log_dir/run_name, only if logging with --log',
        rich_help_panel='Disk logging options',
    ),
    sequence: str | None = typer.Option(
        None,
        '--sequence',
        help='Extra dataloader argument: sensor sequence',
        rich_help_panel='HeLiPR dataloader options',
    ),
    imu_topic: str | None = typer.Option(
        None,
        '--imu',
        help='Extra dataloader argument: imu topic',
        rich_help_panel='Rosbag dataloader options',
    ),
    lidar_topic: str | None = typer.Option(
        None,
        '--lidar',
        help='Extra dataloader argument: lidar topic',
        rich_help_panel='Rosbag dataloader options',
    ),
    base_frame: str | None = typer.Option(
        None,
        '--base_frame',
        help='Extra dataloader argument: base_frame for odometry estimation, default is lidar frame',
        rich_help_panel='Rosbag dataloader options',
    ),
    imu_frame: str | None = typer.Option(
        None,
        '--imu_frame',
        help='Extra dataloader argument: imu frame overload',
        rich_help_panel='Rosbag dataloader options',
    ),
    lidar_frame: str | None = typer.Option(
        None,
        '--lidar_frame',
        help='Extra dataloader argument: lidar frame overload',
        rich_help_panel='Rosbag dataloader options',
    ),
):
    """
    Run RKO_LIO with the selected dataloader and parameters.
    """

    user_config: PipelineConfig = ctx.obj

    # Update config if CLI overwrite is given
    if data_path:
        user_config.data_loader_cfg.data_path = data_path
    if dataloader_name:
        user_config.data_loader_cfg.name = dataloader_name
    if viz:
        user_config.viz = viz
    if reset_viz:
        user_config.reset_viz = reset_viz
    if viz_every_n_frames:
        user_config.viz_every_n_frames = viz_every_n_frames
    if log_results:
        user_config.log_cfg.log_results = log_results
    if log_dir:
        user_config.log_cfg.log_dir = log_dir
    if run_name:
        user_config.log_cfg.run_name = run_name
    if dump_deskewed_scans:
        user_config.log_cfg.dump_deskewed_scans = dump_deskewed_scans
    if sequence:
        user_config.data_loader_cfg.helipr_cfg.sequence = sequence
    if imu_topic:
        user_config.data_loader_cfg.rosbag_cfg.imu_topic = imu_topic
    if lidar_topic:
        user_config.data_loader_cfg.rosbag_cfg.lidar_topic = lidar_topic
    if base_frame:
        user_config.data_loader_cfg.rosbag_cfg.base_frame_id = base_frame
    if imu_frame:
        user_config.data_loader_cfg.rosbag_cfg.imu_frame_id = imu_frame
    if lidar_frame:
        user_config.data_loader_cfg.rosbag_cfg.lidar_frame_id = lidar_frame

    if (
        user_config.data_loader_cfg.data_path is None
        or not Path(user_config.data_loader_cfg.data_path).exists()
    ):
        error_and_exit(f'Given data_path doesnt exist: {user_config.data_loader_cfg.data_path}')
    # apply additional modifications
    if not run_name:
        user_config.log_cfg.run_name = Path(user_config.data_loader_cfg.data_path).name

    if user_config.viz:
        try:
            import rerun as rr

            rr.init('rko_lio')
            rr.spawn(memory_limit='2GB')
            if user_config.reset_viz:
                rr.log_file_from_path(
                    Path(__file__).parent / 'rko_lio.rbl' if rbl_path is None else rbl_path
                )

        except ImportError:
            error_and_exit(
                "Please install rerun with `pip install rerun-sdk` to enable visualization."
            )

    # pipeline_config = PipelineConfig(**user_config)

    from .dataloaders import dataloader_factory

    dataloader = dataloader_factory(
        lio_cfg=user_config,
    )
    print('Loaded dataloader:', dataloader)

    # try setup TF from config
    user_config.tf_cfg.init_tf()
    if (
        user_config.tf_cfg.extrinsic_imu2base is None
        or user_config.tf_cfg.extrinsic_lidar2base is None
    ):
        info('Extrinsics missing or not fully specified in config.')
        dl_ext_imu2base, dl_ext_lidar2base = dataloader.extrinsics
        if dl_ext_imu2base is None or dl_ext_lidar2base is None:
            error_and_exit(
                'Fatal: Could not obtain required IMU/Lidar extrinsics. Please specify in a config or as part of your data.'
            )
        else:
            user_config.tf_cfg.extrinsic_imu2base = dl_ext_imu2base
            user_config.tf_cfg.extrinsic_lidar2base = dl_ext_lidar2base

    from .util import transform_to_quat_xyzw_xyz

    print('Resolved extrinsics:')
    print('  IMU to Base:', transform_to_quat_xyzw_xyz(user_config.tf_cfg.extrinsic_imu2base))
    print(
        '  Lidar to Base:',
        transform_to_quat_xyzw_xyz(user_config.tf_cfg.extrinsic_lidar2base),
    )

    from .lio_pipeline import LIOPipeline

    pipeline = LIOPipeline(user_config)

    from tqdm import tqdm

    for kind, data_tuple in tqdm(dataloader, total=len(dataloader), desc='Data'):
        if kind == 'imu':
            pipeline.add_imu(*data_tuple)
        elif kind == 'lidar':
            pipeline.add_lidar(*data_tuple)

    if user_config.log_cfg.log_results:
        pipeline.dump_results_to_disk()


@app.callback()
def cli_callback(
    ctx: typer.Context,
    config_file: Path | None = typer.Option(
        None,
        '--config',
        '-c',
        exists=True,
        help='Path to config.yaml',
        file_okay=True,
        dir_okay=False,
        readable=True,
    ),
    version: bool | None = typer.Option(
        None,
        '--version',
        help='Print the current version of RKO_LIO and exit',
        callback=version_callback,
        is_eager=True,
    ),
):
    if config_file and config_file.exists():
        # try loading config from yaml
        yaml_obj = yaml.safe_load(config_file.read_text())
        cfg_loaded = PipelineConfig.model_validate(yaml_obj)
    else:
        # otherwise initialize pipeline with default config
        info(f'No YAML config given, starting with default params..')
        cfg_loaded = PipelineConfig()
    ctx.obj = cfg_loaded


@app.command(help='Dump the default config to config.yaml and exit')
def dump_config(ctx: typer.Context):
    dump_config_callback()


if __name__ == '__main__':
    app()
