from setuptools import setup, find_packages
from pathlib import Path

# ====== Project root ======
ROOT = Path(__file__).parent.resolve()

setup(
    name="franka_teleop",
    version="0.1.0",
    description="Franka teleoperation and dataset collection utilities",
    author="Ryan Zhang, Claude Code",
    author_email="ryanzhangtianran@gmail.com",
    python_requires=">=3.12",
    packages=find_packages(where=".", include=["scripts*", "scripts.*"]),
    include_package_data=True,
    install_requires=[
        "send2trash",
        f"interface @ {(ROOT / 'interface').as_uri()}",
        f"teleoperation @ {(ROOT / 'teleoperation').as_uri()}",
    ],
    scripts=[
        "scripts/tools/map_gripper.sh",
    ],
    entry_points={
        "console_scripts": [
            # core commands
            "franka-record = scripts.core.record:main",
            "franka-replay = scripts.core.replay:main",
            "franka-visualize = scripts.core.visualize:main",
            "franka-reset = scripts.core.reset:main",
            "franka-connect = scripts.tools.desk_tunnel:main_connect",
            "franka-disconnect = scripts.tools.desk_tunnel:main_disconnect",
            "franka-start = scripts.tools.nuc_service:main_start",
            "franka-stop = scripts.tools.nuc_service:main_stop",
            "franka-attach = scripts.tools.nuc_service:main_attach",
            "franka-status = scripts.tools.nuc_service:main_status",
            "franka-gripper-reinit = scripts.tools.nuc_service:main_gripper_reinit",

            # tools commands (helper tools)
            "tools-check-dataset = scripts.tools.check_dataset_info:main",
            "tools-check-rs = scripts.tools.rs_devices:main",

            # unified help command
            "franka-help = scripts.help.help:main",
        ]
    },
)
