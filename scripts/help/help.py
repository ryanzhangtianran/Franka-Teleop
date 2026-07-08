def main():
    print("""
==================================================
 Franka Teleoperation Utilities - Command Reference
==================================================

Core Commands:
  franka-record         Record teleoperation dataset (--check | --single | --episodes N)
  franka-replay         Replay a recorded dataset
  franka-visualize      Visualize recorded dataset
  franka-reset          Reset the robot to the home position (asks for confirmation)
  franka-connect        Open SSH tunnel to Franka Desk (https://localhost:8443/desk/)
  franka-disconnect     Close the Franka Desk SSH tunnel
  franka-start          Start Franka driver services on the NUC
  franka-stop           Stop Franka driver services on the NUC
  franka-attach         Attach to the 'franka' tmux session on the NUC
  franka-status         Show NUC service processes and zerorpc port status
  franka-gripper-reinit Restart launch_gripper on the NUC (fixes a stuck gripper)

Tool Commands:
  tools-check-dataset   Clean up dataset_info.txt (removes stale entries; keeps a backup)
  tools-check-rs        Retrieve connected RealSense/Orbbec camera serial numbers

Shell Tools:
  map_gripper.sh        Map gripper serial port to a stable /dev name

--------------------------------------------------
 Tip: Use 'franka-help' anytime to see this summary.
==================================================
""")
