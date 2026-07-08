"""Remote control of the Franka driver services running on the NUC.

`franka-start`        run ~/start_franka.sh on the NUC
`franka-stop`         run ~/stop_franka.sh on the NUC
`franka-attach`       attach to the `franka` tmux session on the NUC (interactive)
`franka-status`       list service processes and zerorpc port status on the NUC
`franka-gripper-reinit`  restart launch_gripper in the NUC gripper tmux window

Override the SSH host with FRANKA_NUC_SSH_HOST (default: NUC).
"""

import os
import subprocess
import sys

SSH_HOST = os.environ.get("FRANKA_NUC_SSH_HOST", "NUC")

# [l]aunch/[r]un character classes stop pgrep from matching the remote
# "bash -c <this command>" wrapper process itself.
STATUS_CMD = (
    'pgrep -af "[l]aunch_robot|[l]aunch_gripper|[l]aunch_server|[r]un_server"'
    ' || echo "(no services running)";'
    ' nc -z localhost 4242 2>/dev/null'
    ' && echo "zerorpc 4242: UP" || echo "zerorpc 4242: DOWN"'
)


def _ssh(command: str) -> None:
    sys.exit(subprocess.run(["ssh", SSH_HOST, command]).returncode)


def main_start() -> None:
    _ssh("bash ~/start_franka.sh")


def main_stop() -> None:
    _ssh("bash ~/stop_franka.sh")


def main_attach() -> None:
    # exec so the interactive tmux session takes over this terminal directly
    os.execvp("ssh", ["ssh", "-t", SSH_HOST, "tmux attach -t franka"])


def main_status() -> None:
    _ssh(STATUS_CMD)


def main_gripper_reinit() -> None:
    result = subprocess.run([
        "ssh", SSH_HOST,
        'tmux send-keys -t franka:gripper C-c; sleep 2;'
        ' tmux send-keys -t franka:gripper "python launch_gripper.py gripper=franka_hand" C-m',
    ])
    if result.returncode == 0:
        print("launch_gripper restarted — once the NUC gripper window prints 'Homing gripper', the gripper is usable again")
    sys.exit(result.returncode)
