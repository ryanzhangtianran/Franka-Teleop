import yaml
from pathlib import Path
from typing import Dict, Any
from interface import FrankaConfig, Franka
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")

def main():
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "record_cfg.yaml"
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

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
