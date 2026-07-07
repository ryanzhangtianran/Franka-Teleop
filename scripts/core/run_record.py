import os
import yaml
import time
from pathlib import Path
from typing import Dict, Any
import numpy as np
from scripts.utils.dataset_utils import generate_dataset_name, update_dataset_info
from scripts.utils.dataset_schema_utils import (
    build_legacy_action_frame,
    build_legacy_dataset_features,
    build_legacy_observation_frame,
    load_dataset_schema_config,
    uses_legacy_dataset_schema,
)
from interface import FrankaConfig, Franka
from interface.franka import HOME_JOINT_POSITION
from teleoperation.config_teleop import SpacemouseTeleopConfig
from teleoperation.spacemouse_teleop import SpacemouseTeleop
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.orbbec.configuration_orbbec import OrbbecCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCameraConfig
from lerobot.scripts.lerobot_record import record_loop as standard_record_loop
from lerobot.processor import make_default_processors
from lerobot.utils.visualization_utils import init_rerun
from lerobot.utils.control_utils import init_keyboard_listener
from send2trash import send2trash
import termios, sys
from lerobot.utils.constants import HF_LEROBOT_HOME
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.utils.control_utils import sanity_check_dataset_robot_compatibility
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.processor.rename_processor import rename_stats
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.utils.visualization_utils import log_rerun_data
from lerobot.utils.robot_utils import busy_wait
from dataclasses import field

import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")


class RecordConfig:
    """Configuration class for recording sessions."""
    
    def __init__(self, cfg: Dict[str, Any]):
        storage = cfg["storage"]
        task = cfg["task"]
        time = cfg["time"]
        cam = cfg["cameras"]
        robot = cfg["robot"]
        policy = cfg["policy"]
        teleop = cfg["teleop"]
        
        # Global config
        self.repo_id: str = cfg["repo_id"]
        self.debug: bool = cfg.get("debug", True)
        self.fps: str = cfg.get("fps", 15)
        self.dataset_path: str = HF_LEROBOT_HOME / self.repo_id
        self.user_info: str = cfg.get("user_notes", None)
        self.run_mode: str = cfg.get("run_mode", "run_record")
        self.rename_map: dict[str, str] = field(default_factory=dict)
        self.dataset_schema_config: str | None = cfg.get("dataset_schema_config")
        
        # Teleop config - parse based on control mode
        self.control_mode = teleop.get("control_mode", "spacemouse")
        self._parse_teleop_config(teleop)
        
        # Policy config
        self._parse_policy_config(policy)
        
        # Robot config
        self.robot_ip: str = robot["ip"]
        self.use_gripper: bool = robot["use_gripper"]
        self.close_threshold = robot["close_threshold"]
        self.gripper_reverse: bool = robot["gripper_reverse"]
        self.gripper_bin_threshold: float = robot["gripper_bin_threshold"]
        self.gripper_max_open: float = robot.get("gripper_max_open", 0.08)

        # Task config
        self.num_episodes: int = task.get("num_episodes", 1)
        self.display: bool = task.get("display", True)
        self.task_description: str = task.get("description", "default task")
        self.resume: bool = task.get("resume", False)
        self.resume_dataset: str = task.get("resume_dataset", "")
        
        # Time config
        self.episode_time_sec: int = time.get("episode_time_sec", 60)
        self.reset_time_sec: int = time.get("reset_time_sec", 10)
        self.save_mera_period: int = time.get("save_mera_period", 1)
        # 推迟视频编码: 1=每条录完立即编码(默认,采集中会等); >1(或设很大)=录制全程不编码,
        # 录完后统一编码,使采集过程不被编码阻塞。环境变量 FRANKA_BATCH_ENCODING 可覆盖。
        self.batch_encoding_size: int = int(
            os.environ.get("FRANKA_BATCH_ENCODING", time.get("batch_encoding_size", 1))
        )

        # Cameras config
        self.camera_type: str = cam.get("camera_type", cam.get("type", "realsense")).lower()
        self.wrist_cam_id: str | None = cam.get("wrist_cam_id", cam.get("wrist_cam_serial"))
        self.exterior_cam_id: str | None = cam.get("exterior_cam_id", cam.get("exterior_cam_serial"))
        self.width: int = cam["width"]
        self.height: int = cam["height"]
        
        # Storage config
        self.push_to_hub: bool = storage.get("push_to_hub", False)
    
    def _parse_teleop_config(self, teleop: Dict[str, Any]) -> None:
        """Parse teleoperation configuration based on control mode."""
        if self.control_mode == "spacemouse":
            sm_cfg = teleop["spacemouse_config"]
            self.use_gripper = sm_cfg["use_gripper"]
            self.pose_scaler = sm_cfg["pose_scaler"]
            self.channel_signs = sm_cfg["channel_signs"]

        else:
            raise ValueError(f"Unsupported control mode: {self.control_mode}")
    
    def _parse_policy_config(self, policy: Dict[str, Any]) -> None:
        """Parse policy configuration."""
        policy_type = policy["type"]
        if policy_type == "act":
            from lerobot.policies import ACTConfig
            self.policy = ACTConfig(
                device=policy["device"],
                push_to_hub=policy["push_to_hub"],
            )
        elif policy_type == "diffusion":
            from lerobot.policies import DiffusionConfig
            self.policy = DiffusionConfig(
                device=policy["device"],
                push_to_hub=policy["push_to_hub"],
            )
        else:
            raise ValueError(f"No config for policy type: {policy_type}")
        
        if policy.get("pretrained_path"):
            self.policy.pretrained_path = policy["pretrained_path"]
    
    def create_teleop_config(self):
        """Create teleoperation configuration object."""
        if self.control_mode == "spacemouse":
            return SpacemouseTeleopConfig(
                use_gripper=self.use_gripper,
                pose_scaler=self.pose_scaler,
                channel_signs=self.channel_signs,
            )
        else:
            raise ValueError(f"Unsupported control mode: {self.control_mode}")

def handle_incomplete_dataset(dataset_path):
    if dataset_path.exists():
        print(f"====== [WARNING] Detected an incomplete dataset folder: {dataset_path} ======")
        print("  ▶ 程序异常/中断退出,留下了未完成的数据集")
        print("  ▶ 请在【启动终端】(不是物理键盘)输入: y=删除(可从回收站找回) | n=保留,稍后手动处理")
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        ans = input("Do you want to delete it? (y/n): ").strip().lower()
        if ans == "y":
            print(f"====== [DELETE] Removing folder: {dataset_path} ======")
            # Send to trash
            send2trash(dataset_path)
            print("====== [DONE] Incomplete dataset folder deleted successfully. ======")
        else:
            print("====== [KEEP] Incomplete dataset folder retained, please check manually. ======")


def create_camera_config(camera_type: str, camera_id: str, fps: int, width: int, height: int):
    camera_type = camera_type.lower()

    if camera_type in ("realsense", "intelrealsense"):
        return RealSenseCameraConfig(
            serial_number_or_name=str(camera_id),
            fps=fps,
            width=width,
            height=height,
            color_mode=ColorMode.RGB,
            use_depth=False,
            rotation=Cv2Rotation.NO_ROTATION,
        )

    if camera_type == "orbbec":
        return OrbbecCameraConfig(
            serial_number_or_name=str(camera_id),
            fps=fps,
            width=width,
            height=height,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
        )

    raise ValueError(f"Unsupported camera_type: {camera_type}. Use 'realsense' or 'orbbec'.")


def create_camera_configs(record_cfg: RecordConfig):
    if not record_cfg.exterior_cam_id:
        raise ValueError("exterior_cam_id is required for recording.")

    camera_config = {
        "exterior_image": create_camera_config(
            record_cfg.camera_type,
            record_cfg.exterior_cam_id,
            record_cfg.fps,
            record_cfg.width,
            record_cfg.height,
        )
    }

    if record_cfg.wrist_cam_id:
        camera_config["wrist_image"] = create_camera_config(
            record_cfg.camera_type,
            record_cfg.wrist_cam_id,
            record_cfg.fps,
            record_cfg.width,
            record_cfg.height,
        )

    return camera_config


@safe_stop_image_writer
def legacy_record_loop(
    robot,
    events: dict[str, bool],
    fps: int,
    teleop,
    teleop_action_processor,
    robot_action_processor,
    robot_observation_processor,
    control_time_s: int | None = None,
    single_task: str | None = None,
    display_data: bool = False,
    dataset: LeRobotDataset | None = None,
):
    if teleop is None:
        raise ValueError("Legacy dataset schema currently supports teleoperation recording only.")

    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    timestamp = 0
    start_episode_t = time.perf_counter()
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        obs = robot.get_observation()
        obs_processed = robot_observation_processor(obs)

        act = teleop.get_action()
        act_processed_teleop = teleop_action_processor((act, obs))
        robot_action_to_send = robot_action_processor((act_processed_teleop, obs))
        _sent_action = robot.send_action(robot_action_to_send)

        if dataset is not None:
            observation_frame = build_legacy_observation_frame(dataset.features, obs_processed)
            action_frame = build_legacy_action_frame(dataset.features, act_processed_teleop)
            frame = {**observation_frame, **action_frame, "task": single_task}
            dataset.add_frame(frame)

        if display_data:
            log_rerun_data(observation=obs_processed, action=act_processed_teleop)

        dt_s = time.perf_counter() - start_loop_t
        busy_wait(1 / fps - dt_s)
        timestamp = time.perf_counter() - start_episode_t


def reset_environment_loop(
    robot: Franka,
    events: dict[str, bool],
    fps: int,
    control_time_s: int | float,
    display_data: bool = False,
) -> None:
    start_reset_t = time.perf_counter()
    timestamp = 0.0
    home_joints = np.asarray(HOME_JOINT_POSITION, dtype=float)
    motion_time_s = min(float(control_time_s), 5.0)
    start_joints = None

    events["exit_early"] = False

    try:
        robot._robot.robot_start_joint_impedance_control()
    except Exception as e:
        logging.warning(f"[RESET] Failed to start controller before reset loop: {e}")

    if robot.config.use_gripper:
        try:
            robot._robot.gripper_goto(
                width=robot.config.gripper_max_open,
                speed=robot._gripper_speed,
                force=robot._gripper_force,
                blocking=False,
            )
            robot._last_gripper_position = 1.0
            robot._gripper_position = 1.0
        except Exception as e:
            logging.warning(f"[RESET] Failed to open gripper during reset loop: {e}")

    try:
        start_joints = np.asarray(robot._robot.robot_get_joint_positions(), dtype=float)
    except Exception as e:
        logging.warning(f"[RESET] Failed to read joint positions before reset loop: {e}")
        start_joints = home_joints.copy()

    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["stop_recording"]:
            break

        if events["exit_early"]:
            events["exit_early"] = False
            break

        alpha = 1.0
        if motion_time_s > 0:
            alpha = min(1.0, timestamp / motion_time_s)
        target_joints = start_joints + alpha * (home_joints - start_joints)

        try:
            robot._robot.robot_update_desired_joint_positions(target_joints)
        except Exception as e:
            logging.warning(f"[RESET] Failed to send home joint target: {e}")
            try:
                robot._robot.robot_start_joint_impedance_control()
            except Exception as restart_error:
                logging.warning(f"[RESET] Failed to restart controller during reset loop: {restart_error}")

        if display_data:
            try:
                obs = robot.get_observation()
                log_rerun_data(observation=obs, action=None)
            except Exception as e:
                logging.warning(f"[RESET] Failed to log reset observation: {e}")

        dt_s = time.perf_counter() - start_loop_t
        busy_wait(1 / fps - dt_s)
        timestamp = time.perf_counter() - start_reset_t


def run_record(record_cfg: RecordConfig):
    print("====== [START] Starting recording ======")
    try:
        dataset_name, data_version = generate_dataset_name(record_cfg)
        config_dir = Path(__file__).resolve().parent.parent / "config"
        dataset_schema_config = load_dataset_schema_config(record_cfg.dataset_schema_config, config_dir)

        # Create the robot and teleoperator configurations
        camera_config = create_camera_configs(record_cfg)
        
        # Create teleop config using the new method
        teleop_config = record_cfg.create_teleop_config()
        
        robot_config = FrankaConfig(
            robot_ip=record_cfg.robot_ip,
            cameras = camera_config,
            debug = record_cfg.debug,
            close_threshold = record_cfg.close_threshold,
            use_gripper = record_cfg.use_gripper,
            gripper_reverse = record_cfg.gripper_reverse,
            gripper_bin_threshold = record_cfg.gripper_bin_threshold,
            gripper_max_open = record_cfg.gripper_max_open,
            control_mode = record_cfg.control_mode,
        )
        # Initialize the robot
        robot = Franka(robot_config)

        # Configure the dataset features
        if uses_legacy_dataset_schema(dataset_schema_config):
            dataset_features = build_legacy_dataset_features(
                dataset_schema_config,
                robot.action_features,
                robot.observation_features,
            )
        else:
            action_features = hw_to_dataset_features(robot.action_features, "action")
            obs_features = hw_to_dataset_features(robot.observation_features, "observation", use_video=True)
            dataset_features = {**action_features, **obs_features}

        if record_cfg.resume:
            dataset = LeRobotDataset(
                dataset_name,
            )

            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer()
            sanity_check_dataset_robot_compatibility(dataset, robot, record_cfg.fps, dataset_features)
        else:
            # # Create the dataset
            dataset = LeRobotDataset.create(
                repo_id=dataset_name,
                fps=record_cfg.fps,
                features=dataset_features,
                robot_type=robot.name,
                use_videos=True,
                image_writer_threads=4,
                batch_encoding_size=record_cfg.batch_encoding_size,
            )
            if record_cfg.batch_encoding_size > 1:
                logging.info(
                    f"====== [ENCODE] 推迟编码已开启 (batch_encoding_size={record_cfg.batch_encoding_size}):"
                    f" 录制全程不编码,结束后统一编码 ======"
                )
        # Set the episode metadata buffer size to 1, so that each episode is saved immediately
        dataset.meta.metadata_buffer_size = record_cfg.save_mera_period

        # Initialize the keyboard listener and rerun visualization
        _, events = init_keyboard_listener()
        init_rerun(session_name="recording")

        # Create processor
        teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
        preprocessor = None
        postprocessor = None

        # configure the teleop and policy
        if record_cfg.run_mode == "run_record":
            logging.info("====== [INFO] Running in teleoperation mode ======")
            teleop = SpacemouseTeleop(teleop_config)
            policy = None
        elif record_cfg.run_mode == "run_policy":
            logging.info("====== [INFO] Running in policy mode ======")
            policy = make_policy(record_cfg.policy, ds_meta=dataset.meta)
            teleop = None
        elif record_cfg.run_mode == "run_mix":
            logging.info("====== [INFO] Running in mixed mode ======")
            policy = make_policy(record_cfg.policy, ds_meta=dataset.meta)
            teleop = SpacemouseTeleop(teleop_config)

        if uses_legacy_dataset_schema(dataset_schema_config) and record_cfg.run_mode != "run_record":
            raise ValueError("Legacy dataset schema currently supports run_record only.")
        
        if policy is not None:
            preprocessor, postprocessor = make_pre_post_processors(
                policy_cfg=record_cfg.policy,
                pretrained_path=record_cfg.policy.pretrained_path,
                dataset_stats=rename_stats(dataset.meta.stats, {}),  # 使用空字典作为rename_map
                preprocessor_overrides={
                    "device_processor": {"device": record_cfg.policy.device},
                    "rename_observations_processor": {"rename_map": {}},  # 使用空字典作为rename_map
                },
            )

        robot.connect()
        if teleop is not None:
            teleop.connect()

        episode_idx = 0

        while episode_idx < record_cfg.num_episodes and not events["stop_recording"]:
            logging.info("")
            logging.info(f"====== [RECORD] Recording episode {episode_idx + 1} of {record_cfg.num_episodes} ======")
            logging.info("  ▶ 录制中!你的所有操作和画面正在被记录")
            logging.info("  ▶ SpaceMouse: 推杆控臂(行程过半才响应) | 左键=开夹爪 | 右键=合夹爪")
            logging.info("  ▶ 按键(物理键盘): →=完成并保存本条 | ←=录废了,丢弃重录 | Esc=结束整个会话")
            if uses_legacy_dataset_schema(dataset_schema_config):
                legacy_record_loop(
                    robot=robot,
                    events=events,
                    fps=record_cfg.fps,
                    teleop=teleop,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    dataset=dataset,
                    control_time_s=record_cfg.episode_time_sec,
                    single_task=record_cfg.task_description,
                    display_data=record_cfg.display,
                )
            else:
                standard_record_loop(
                    robot=robot,
                    events=events,
                    fps=record_cfg.fps,
                    teleop=teleop,
                    policy=policy,
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    dataset=dataset,
                    control_time_s=record_cfg.episode_time_sec,
                    single_task=record_cfg.task_description,
                    display_data=record_cfg.display,
                )

            if events["rerecord_episode"]:
                logging.info("====== [RERECORD] 收到 ←:本条数据已丢弃 ======")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                # 重录前先复位回 Home(并重开夹爪、同步夹爪状态),避免从上一条结束的乱姿态直接开录
                if not events["stop_recording"]:
                    logging.info("====== [RESET] 机械臂正在回 Home 位,请把物体摆回初始位置 ======")
                    logging.info("  ▶ 摆好后按 → :结束复位并【立即开始重录这一条】,请做好操作准备!")
                    logging.info("  ▶ 按 Esc 结束会话")
                    reset_environment_loop(
                        robot=robot,
                        events=events,
                        fps=record_cfg.fps,
                        control_time_s=record_cfg.reset_time_sec,
                        display_data=record_cfg.display,
                    )
                continue

            logging.info("====== [SAVE] 正在保存并编码视频(下方 Svt[info] 滚动输出是编码器日志,属正常现象)... ======")
            dataset.save_episode()
            logging.info(f"====== [SAVE] ✅ 第 {episode_idx + 1} 条 episode 已保存 ======")

            # Reset the environment if not stopping or re-recording
            if not events["stop_recording"] and (episode_idx < record_cfg.num_episodes - 1 or events["rerecord_episode"]):
                # Wait for the right arrow key (same physical keyboard as episode control),
                # so the operator never has to switch back to the launch terminal.
                logging.info("")
                logging.info("====== [WAIT] 程序已暂停,等待你的指令(此时不在录制,可以休息)======")
                logging.info("  ▶ 按 → 进入复位:机械臂将自动移回 Home 位(注意:机械臂会运动!)")
                logging.info("  ▶ 按 Esc 结束会话:保存全部已录数据并退出")
                events["exit_early"] = False
                events["rerecord_episode"] = False
                while not events["exit_early"] and not events["stop_recording"]:
                    time.sleep(0.05)
                events["exit_early"] = False
                events["rerecord_episode"] = False

                if events["stop_recording"]:
                    logging.info("====== [STOP] 收到 Esc:结束录制会话,开始收尾保存... ======")
                    break

                logging.info("")
                logging.info("====== [RESET] 机械臂正在回 Home 位,请趁现在把物体摆回初始位置 ======")
                logging.info("  ▶ 场景摆好后按 → :结束复位并【立即开始录制下一条】,请做好操作准备!")
                logging.info("  ▶ 按 Esc 结束会话")
                reset_environment_loop(
                    robot=robot,
                    events=events,
                    fps=record_cfg.fps,
                    control_time_s=record_cfg.reset_time_sec,
                    display_data=record_cfg.display,
                )

            episode_idx += 1

        # Clean up
        logging.info("")
        logging.info("====== [FINALIZE] 录制结束,正在断开设备并整理数据集(可能需要几十秒,请勿 Ctrl+C)... ======")
        robot.disconnect()
        if teleop is not None:
            teleop.disconnect()

        # 推迟编码模式: 录制全程没编码,这里逐条编码尾批 (start_ep..num_episodes)。
        # 注意: 不能用 dataset._batch_save_episode_video —— 这版 lerobot 它对"从零编码未编码的 episode"
        #   会崩(meta.episodes 为 None, 且 _save_episode_video 的 ep0 分支会去读 episodes[-1] 的视频列 -> KeyError)。
        #   正确做法: meta.episodes 置 None 让 ep0 从 chunk0/file0 干净起步, 逐条调 _save_episode_video
        #   并自己维护 latest_episode(供后续 episode 拼接), 最后把视频元数据列写回 episodes parquet。
        # (本流程假设"全部推迟到末尾", 即 batch_encoding_size 大于本次录制条数 -> start_ep==0)
        if (
            record_cfg.batch_encoding_size > 1
            and len(dataset.meta.video_keys) > 0
            and getattr(dataset, "episodes_since_last_encoding", 0) > 0
        ):
            import glob as _glob
            import pandas as _pd
            n_pending = dataset.episodes_since_last_encoding
            start_ep = dataset.num_episodes - n_pending
            logging.info(
                f"====== [ENCODE] 统一编码剩余 {n_pending} 条 episode (ep {start_ep}~{dataset.num_episodes - 1}),"
                f" 这一步耗时取决于条数,请耐心等待、勿 Ctrl+C... ======"
            )
            if start_ep == 0:
                dataset.meta.episodes = None   # 让 ep0 走 chunk0/file0 干净起步
            dataset.meta.latest_episode = None
            _rows = {}
            for _ep in range(start_ep, dataset.num_episodes):
                _row = {}
                for _vk in dataset.meta.video_keys:
                    _row.update(dataset._save_episode_video(_vk, _ep))
                dataset.meta.latest_episode = {k: [v] for k, v in _row.items()}
                _row.pop("episode_index", None)
                _rows[_ep] = _row
                logging.info(f"  [ENCODE] ep {_ep} ✅")
            _ep_path = _glob.glob(str(dataset.root / "meta/episodes/**/*.parquet"), recursive=True)[0]
            _ep_df = _pd.read_parquet(_ep_path)
            _vid_df = _pd.DataFrame.from_dict(_rows, orient="index")
            _vid_df.index = _vid_df.index.astype(_ep_df.index.dtype)
            _ep_df = _ep_df.combine_first(_vid_df)
            _ep_df.to_parquet(_ep_path)
            dataset.episodes_since_last_encoding = 0
            logging.info("====== [ENCODE] ✅ 全部视频编码完成 ======")

        dataset.finalize()

        update_dataset_info(record_cfg, dataset_name, data_version)
        logging.info(f"====== [DONE] ✅ 全部完成!数据集保存在: {HF_LEROBOT_HOME / dataset_name} ======")
        if record_cfg.push_to_hub:
            dataset.push_to_hub()

    except Exception as e:
        logging.info(f"====== [ERROR] {e} ======")
        logging.exception("====== [TRACEBACK] Recording failed with exception ======")
        dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
        handle_incomplete_dataset(dataset_path)
        sys.exit(1)

    except KeyboardInterrupt:
        logging.info("\n====== [INFO] Ctrl+C detected, cleaning up incomplete dataset... ======")
        dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
        handle_incomplete_dataset(dataset_path)
        sys.exit(1)


def main():
    parent_path = Path(__file__).resolve().parent
    # 默认 record_cfg.yaml; 可用 FRANKA_RECORD_CFG 指定别的配置(如 test 配置)
    cfg_path = os.environ.get("FRANKA_RECORD_CFG") or (parent_path.parent / "config" / "record_cfg.yaml")
    print(f"[CFG] using {cfg_path}")
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # 允许用环境变量覆盖录制条数(start_record.sh 的 --single / --episodes N 会设置它)
    n_override = os.environ.get("FRANKA_NUM_EPISODES")
    if n_override:
        cfg["record"]["task"]["num_episodes"] = int(n_override)

    record_cfg = RecordConfig(cfg["record"])
    run_record(record_cfg)

if __name__ == "__main__":
    main()
