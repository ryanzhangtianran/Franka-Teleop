import argparse
import os
import socket
import subprocess
import yaml
import time
from pathlib import Path
from typing import Dict, Any
import numpy as np
from scripts.utils.dataset_utils import generate_dataset_name, update_dataset_info
from interface import FrankaConfig, Franka
from interface.franka import HOME_JOINT_POSITION
from teleoperation.config_teleop import SpacemouseTeleopConfig
from teleoperation.spacemouse_teleop import SpacemouseTeleop
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.orbbec.configuration_orbbec import OrbbecCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCameraConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.processor import make_default_processors
from lerobot.utils.visualization_utils import init_rerun
from lerobot.utils.keyboard_input import init_keyboard_listener
from send2trash import send2trash
import termios, sys
from lerobot.utils.constants import HF_LEROBOT_HOME
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import hw_to_dataset_features
from lerobot.common.control_utils import sanity_check_dataset_robot_compatibility
from lerobot.utils.visualization_utils import log_rerun_data
from lerobot.utils.robot_utils import precise_sleep

import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")

ZERORPC_PORT = 4242
ORBBEC_USB_ID = "2bc5:0807"
# rerun window + arrow-key listener must land on the :2 physical display
DEFAULT_DISPLAY = ":2"
DEFAULT_XAUTHORITY = "/run/user/1007/gdm/Xauthority"

# Robot-state probe runs in a subprocess so a hung zerorpc call can be killed on timeout.
_ROBOT_PROBE = """
from interface.client import FrankaInterfaceClient
import numpy as np
c = FrankaInterfaceClient(ip="{ip}", port={port})
jp = c.robot_get_joint_positions(); c.close()
print("OK" if np.isfinite(jp).all() and len(jp) == 7 else "BAD")
"""


def _port_open(ip: str, port: int, timeout: float = 3.0) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def _lsusb_output() -> str:
    try:
        return subprocess.run(["lsusb"], capture_output=True, text=True).stdout
    except OSError:
        return ""


def _x_display_ok() -> bool:
    try:
        return subprocess.run(
            ["xset", "q"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ).returncode == 0
    except OSError:
        return False


def _robot_state_ok(ip: str, port: int) -> bool:
    try:
        result = subprocess.run(
            [sys.executable, "-c", _ROBOT_PROBE.format(ip=ip, port=port)],
            capture_output=True, text=True, timeout=10,
        )
        return result.stdout.strip().endswith("OK")
    except (subprocess.TimeoutExpired, OSError):
        return False


def run_preflight(robot_ip: str) -> bool:
    ok = True
    print("── Preflight ─────────────────────────")

    if _port_open(robot_ip, ZERORPC_PORT):
        print(f"  ✅ NUC {robot_ip}:{ZERORPC_PORT} (Polymetis/zerorpc) online")
    else:
        print("  ❌ NUC service unreachable — start the Polymetis services + server.py on the NUC (franka-start)")
        ok = False

    usb = _lsusb_output().lower()
    if "3dconnexion" in usb:
        print("  ✅ SpaceMouse connected")
    else:
        print("  ❌ SpaceMouse not detected")
        ok = False

    cam_n = usb.count(ORBBEC_USB_ID)
    if cam_n >= 2:
        print(f"  ✅ Orbbec cameras ×{cam_n}")
    else:
        print(f"  ❌ Only {cam_n} Orbbec camera(s) detected (need 2)")
        ok = False

    if _x_display_ok():
        print(f"  ✅ X display {os.environ.get('DISPLAY', '?')} available")
    else:
        print(f"  ❌ Cannot access X display {os.environ.get('DISPLAY', '?')} (no graphical session?)")
        ok = False

    if _robot_state_ok(robot_ip, ZERORPC_PORT):
        print("  ✅ Robot state readable (FCI OK)")
    else:
        print("  ❌ Cannot read robot joint positions — check that FCI is activated in Franka Desk and the e-stop is released")
        ok = False

    print("──────────────────────────────────────")
    return ok


def print_config_summary(cfg: Dict[str, Any], n_override: int | None) -> None:
    c = cfg["record"]
    n = c["task"]["num_episodes"]
    n_show = f"{n_override} (command-line override; config file says {n})" if n_override else str(n)
    print("Current task config:")
    print(f"  repo_id     : {c['repo_id']}")
    print(f"  task        : {c['task']['description']}")
    print(f"  episodes    : {n_show}  | fps: {c['fps']}  | debug: {c['debug']}")
    print(f"  robot.ip    : {c['robot']['ip']}")


class RecordConfig:
    """Configuration class for recording sessions."""
    
    def __init__(self, cfg: Dict[str, Any]):
        storage = cfg["storage"]
        task = cfg["task"]
        time = cfg["time"]
        cam = cfg["cameras"]
        robot = cfg["robot"]
        teleop = cfg["teleop"]

        # Global config
        self.repo_id: str = cfg["repo_id"]
        self.debug: bool = cfg.get("debug", True)
        self.fps: str = cfg.get("fps", 15)
        self.dataset_path: str = HF_LEROBOT_HOME / self.repo_id
        self.user_info: str = cfg.get("user_notes", None)

        # Teleop config (SpaceMouse)
        self._parse_teleop_config(teleop)

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
        self.save_meta_period: int = time.get("save_meta_period", 1)

        # Cameras config
        self.camera_type: str = cam.get("camera_type", cam.get("type", "realsense")).lower()
        self.wrist_cam_id: str | None = cam.get("wrist_cam_id", cam.get("wrist_cam_serial"))
        self.exterior_cam_id: str | None = cam.get("exterior_cam_id", cam.get("exterior_cam_serial"))
        self.width: int = cam["width"]
        self.height: int = cam["height"]
        
        # Storage config
        self.push_to_hub: bool = storage.get("push_to_hub", False)
    
    def _parse_teleop_config(self, teleop: Dict[str, Any]) -> None:
        """Parse SpaceMouse teleoperation configuration."""
        sm_cfg = teleop["spacemouse_config"]
        self.use_gripper = sm_cfg["use_gripper"]
        self.pose_scaler = sm_cfg["pose_scaler"]
        self.channel_signs = sm_cfg["channel_signs"]

    def create_teleop_config(self):
        """Create teleoperation configuration object."""
        return SpacemouseTeleopConfig(
            use_gripper=self.use_gripper,
            pose_scaler=self.pose_scaler,
            channel_signs=self.channel_signs,
        )

def _set_terminal_echo(enabled: bool) -> None:
    """Toggle tty echo so arrow/Esc key presses do not spill ^[[C / ^[ into the terminal."""
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    attrs = termios.tcgetattr(fd)
    if enabled:
        attrs[3] |= termios.ECHO
    else:
        attrs[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, attrs)


def handle_incomplete_dataset(dataset_path):
    if dataset_path.exists():
        _set_terminal_echo(True)
        print(f"====== [WARNING] Detected an incomplete dataset folder: {dataset_path} ======")
        print("  ▶ The program exited abnormally and left an unfinished dataset behind.")
        print("  ▶ Answer in THIS terminal (not the physical keyboard): y = delete (recoverable from trash) | n = keep for manual handling")
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
        precise_sleep(max(1 / fps - dt_s, 0.0))
        timestamp = time.perf_counter() - start_reset_t


def run_record(record_cfg: RecordConfig):
    print("====== [START] Starting recording ======")
    # Episode control keys (→ ← Esc) are read globally via pynput but still land in
    # this terminal; disable echo so they don't spill ^[[C / ^[ between the prompts.
    _set_terminal_echo(False)
    try:
        dataset_name, data_version = generate_dataset_name(record_cfg)

        # Quiet the Orbbec SDK's USB-enumeration warnings (string-descriptor timeouts
        # are harmless startup noise); the logger setting is global to the process.
        if record_cfg.camera_type == "orbbec":
            try:
                import pyorbbecsdk as ob
                ob.Context().set_logger_level(ob.OBLogLevel.ERROR)
            except Exception as e:
                logging.warning(f"[CAM] Failed to set Orbbec SDK log level: {e}")

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
        )
        # Initialize the robot
        robot = Franka(robot_config)

        # Configure the dataset features
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
            dataset = LeRobotDataset.create(
                repo_id=dataset_name,
                fps=record_cfg.fps,
                features=dataset_features,
                robot_type=robot.name,
                use_videos=True,
                image_writer_threads=4,
                # buffer size 1 -> each episode's metadata is saved immediately
                metadata_buffer_size=record_cfg.save_meta_period,
            )

        # Initialize the keyboard listener and rerun visualization
        _, events = init_keyboard_listener()
        init_rerun(session_name="recording")

        # Create processor
        teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

        print("====== [INFO] Running in teleoperation mode ======")
        teleop = SpacemouseTeleop(teleop_config)

        robot.connect()
        teleop.connect()

        episode_idx = 0

        while episode_idx < record_cfg.num_episodes and not events["stop_recording"]:
            print("")
            print(f"====== [RECORD] Recording episode {episode_idx + 1} of {record_cfg.num_episodes} ======")
            print("  ▶ Recording! All your motions and camera frames are being captured.")
            print("  ▶ SpaceMouse: push the cap to move the arm (responds past half travel) | left button = open gripper | right button = close gripper")
            print("  ▶ Keys (physical keyboard): → = finish and save this episode | ← = discard and re-record | Esc = end the whole session", flush=True)
            record_loop(
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

            if events["rerecord_episode"]:
                print("====== [RERECORD] Got ←: this episode has been discarded ======")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                # 重录前先复位回 Home(并重开夹爪、同步夹爪状态),避免从上一条结束的乱姿态直接开录
                if not events["stop_recording"]:
                    print("====== [RESET] Arm is moving back to home; put the objects back to their initial positions ======")
                    print("  ▶ When the scene is ready press → : reset ends and RE-RECORDING STARTS IMMEDIATELY — be ready to operate!")
                    print("  ▶ Press Esc to end the session", flush=True)
                    reset_environment_loop(
                        robot=robot,
                        events=events,
                        fps=record_cfg.fps,
                        control_time_s=record_cfg.reset_time_sec,
                        display_data=record_cfg.display,
                    )
                continue

            print("====== [SAVE] Saving and encoding video (the scrolling Svt[info] lines below are normal encoder logs)... ======", flush=True)
            dataset.save_episode()
            print(f"====== [SAVE] ✅ Episode {episode_idx + 1} saved ======")

            # Reset the environment if not stopping or re-recording
            if not events["stop_recording"] and (episode_idx < record_cfg.num_episodes - 1 or events["rerecord_episode"]):
                # Wait for the right arrow key (same physical keyboard as episode control),
                # so the operator never has to switch back to the launch terminal.
                print("")
                print("====== [WAIT] Paused, waiting for your command (not recording now — take a break) ======")
                print("  ▶ Press → to start reset: the arm will MOVE back to home automatically!")
                print("  ▶ Press Esc to end the session: save everything recorded and exit", flush=True)
                events["exit_early"] = False
                events["rerecord_episode"] = False
                while not events["exit_early"] and not events["stop_recording"]:
                    time.sleep(0.05)
                events["exit_early"] = False
                events["rerecord_episode"] = False

                if events["stop_recording"]:
                    print("====== [STOP] Got Esc: ending the session, finalizing... ======")
                    break

                print("")
                print("====== [RESET] Arm is moving back to home; use this time to put the objects back ======")
                print("  ▶ When the scene is ready press → : reset ends and the NEXT EPISODE STARTS IMMEDIATELY — be ready to operate!")
                print("  ▶ Press Esc to end the session", flush=True)
                reset_environment_loop(
                    robot=robot,
                    events=events,
                    fps=record_cfg.fps,
                    control_time_s=record_cfg.reset_time_sec,
                    display_data=record_cfg.display,
                )

            episode_idx += 1

        # Clean up
        print("")
        print("====== [FINALIZE] Recording ended; disconnecting devices and finalizing the dataset (may take tens of seconds — do NOT Ctrl+C)... ======")
        robot.disconnect()
        teleop.disconnect()

        dataset.finalize()

        update_dataset_info(record_cfg, dataset_name, data_version)
        print(f"====== [DONE] ✅ All done! Dataset saved at: {HF_LEROBOT_HOME / dataset_name} ======")
        if record_cfg.push_to_hub:
            dataset.push_to_hub()

    except Exception as e:
        print(f"====== [ERROR] {e} ======")
        logging.exception("====== [TRACEBACK] Recording failed with exception ======")
        dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
        handle_incomplete_dataset(dataset_path)
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n====== [INFO] Ctrl+C detected, cleaning up incomplete dataset... ======")
        dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
        handle_incomplete_dataset(dataset_path)
        sys.exit(1)

    finally:
        # Drop any control keys still queued in the tty so they don't replay into the shell
        if sys.stdin.isatty():
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
        _set_terminal_echo(True)


def main():
    parser = argparse.ArgumentParser(
        description="Franka SpaceMouse data recording (with preflight checks)"
    )
    parser.add_argument("--check", action="store_true",
                        help="run preflight checks only, do not start recording")
    parser.add_argument("--single", "-1", action="store_true",
                        help="record a single long episode (end it with the → key)")
    parser.add_argument("--episodes", type=int, metavar="N",
                        help="record N episodes this run (overrides config, file unchanged)")
    args = parser.parse_args()

    n_override = None
    if args.single:
        n_override = 1
    elif args.episodes is not None:
        if args.episodes <= 0:
            parser.error(f"--episodes must be a positive integer, got: {args.episodes}")
        n_override = args.episodes
    elif os.environ.get("FRANKA_NUM_EPISODES"):
        n_override = int(os.environ["FRANKA_NUM_EPISODES"])

    # GUI session: put the rerun window + arrow-key listener on the physical display
    if not os.environ.get("DISPLAY"):
        os.environ["DISPLAY"] = DEFAULT_DISPLAY
        os.environ["XAUTHORITY"] = DEFAULT_XAUTHORITY

    parent_path = Path(__file__).resolve().parent
    # 默认 record_config.yaml; 可用 FRANKA_RECORD_CFG 指定别的配置(如 test 配置)
    cfg_path = os.environ.get("FRANKA_RECORD_CFG") or (parent_path.parent / "config" / "record_config.yaml")
    print(f"[CFG] using {cfg_path}")
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    if not run_preflight(cfg["record"]["robot"]["ip"]):
        print("Preflight failed, exiting.")
        sys.exit(1)

    print_config_summary(cfg, n_override)

    if args.check:
        print("(--check mode: preflight done, not starting recording)")
        return

    if n_override:
        cfg["record"]["task"]["num_episodes"] = n_override

    print()
    print("Controls (physical keyboard): →=save, reset, record next (keep pressing to advance)  ←=re-record current  Esc=stop and save all")
    record_cfg = RecordConfig(cfg["record"])
    run_record(record_cfg)

if __name__ == "__main__":
    main()
