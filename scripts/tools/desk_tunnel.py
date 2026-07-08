"""SSH tunnel helpers for accessing Franka Desk through the NUC.

`franka-connect` opens a background SSH tunnel (localhost:8443 -> 172.16.0.2:443
via the `NUC` SSH host) if one is not already running, then prints the Desk URL.
`franka-disconnect` closes the tunnel.

Override defaults with environment variables:
    FRANKA_DESK_SSH_HOST   SSH host to jump through (default: NUC)
    FRANKA_DESK_LOCAL_PORT Local forwarded port (default: 8443)
    FRANKA_DESK_TARGET     Remote target host:port (default: 172.16.0.2:443)
"""

import os
import subprocess
import sys

SSH_HOST = os.environ.get("FRANKA_DESK_SSH_HOST", "NUC")
LOCAL_PORT = os.environ.get("FRANKA_DESK_LOCAL_PORT", "8443")
TARGET = os.environ.get("FRANKA_DESK_TARGET", "172.16.0.2:443")

FORWARD_SPEC = f"{LOCAL_PORT}:{TARGET}"
# Anchored to processes whose command line starts with "ssh" so pkill cannot
# match unrelated processes that merely mention the forward spec in their args.
PGREP_PATTERN = f"^ssh .*{FORWARD_SPEC}"


def _tunnel_running() -> bool:
    return subprocess.run(
        ["pgrep", "-f", PGREP_PATTERN], stdout=subprocess.DEVNULL
    ).returncode == 0


def main_connect() -> None:
    if not _tunnel_running():
        result = subprocess.run(
            ["ssh", "-fN", "-L", FORWARD_SPEC, "-o", "ExitOnForwardFailure=yes", SSH_HOST]
        )
        if result.returncode != 0:
            print(f"Failed to open SSH tunnel to {SSH_HOST} (exit {result.returncode})")
            sys.exit(result.returncode)
    print(f"Franka Desk -> https://localhost:{LOCAL_PORT}/desk/")


def main_disconnect() -> None:
    if subprocess.run(["pkill", "-f", PGREP_PATTERN]).returncode == 0:
        print("Desk tunnel closed")
    else:
        print("No tunnel running")


if __name__ == "__main__":
    main_connect()
