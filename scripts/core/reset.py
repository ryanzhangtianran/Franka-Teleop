import socket
import sys
import yaml
from pathlib import Path
from typing import Dict, Any
from interface import FrankaConfig, Franka
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")

ZERORPC_PORT = 4242


def _check_nuc_online(ip: str, port: int = ZERORPC_PORT) -> None:
    try:
        with socket.create_connection((ip, port), timeout=3):
            pass
    except OSError:
        print(f"❌ NUC {ip}:{port} unreachable — make sure the Polymetis services are running (franka-start).")
        sys.exit(1)
    print(f"✅ NUC {ip}:{port} online")


def _confirm_reset() -> None:
    print()
    print("⚠️  The robot arm will MOVE AUTONOMOUSLY to the home position (~5 s of joint motion); the gripper will open.")
    print("    Make sure the workspace is clear and the emergency stop is within reach.")
    try:
        ans = input("Confirm reset? Type y to continue, anything else to cancel: ")
    except (EOFError, KeyboardInterrupt):
        ans = ""
    if ans.strip().lower() != "y":
        print("Cancelled.")
        sys.exit(0)


def main():
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "record_config.yaml"
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    _check_nuc_online(cfg["record"]["robot"]["ip"])
    _confirm_reset()

    # 创建机器人配置(zerorpc 服务地址从配置读,默认 NUC 192.168.50.10;
    # 写死 127.0.0.1 只在 NUC 本机能用,在 Franka-Server 上会连不上)
    robot_config = FrankaConfig(
        robot_ip=cfg["record"]["robot"]["ip"],
        use_gripper=cfg["record"]["robot"]["use_gripper"],
        close_threshold=cfg["record"]["robot"]["close_threshold"],
        gripper_bin_threshold=cfg["record"]["robot"]["gripper_bin_threshold"],
        gripper_reverse=cfg["record"]["robot"]["gripper_reverse"],
        gripper_max_open=cfg["record"]["robot"]["gripper_max_open"],
        debug=False
    )
    
    robot = Franka(robot_config)
    robot.connect()
    
    logging.info("Resetting robot to home position...")
    robot.reset()
    
    robot.disconnect()
    logging.info("Robot reset completed successfully.")

if __name__ == "__main__":
    main()
