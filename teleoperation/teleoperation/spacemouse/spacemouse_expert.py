import multiprocessing
import numpy as np
from . import pyspacemouse
from typing import Tuple


class SpaceMouseExpert:
    """
    This class provides an interface to the SpaceMouse.
    It continuously reads the SpaceMouse state and provides
    a "get_action" method to get the latest action and button state.
    """

    def __init__(self):
        pyspacemouse.open()

        # Manager to handle shared state between processes
        self.manager = multiprocessing.Manager()
        self.latest_data = self.manager.dict()
        # Single key so readers always get a consistent (action, buttons) snapshot
        self.latest_data["state"] = ([0.0] * 6, [0, 0, 0, 0])

        # Start a process to continuously read the SpaceMouse state
        self.process = multiprocessing.Process(target=self._read_spacemouse)
        self.process.daemon = True
        self.process.start()

    def _read_spacemouse(self):
        while True:
            try:
                state = pyspacemouse.read_all()
            except Exception:
                # Read failure (e.g. device unplugged): publish a zero action so the
                # arm stops instead of replaying the last delta forever, then exit.
                self.latest_data["state"] = ([0.0] * 6, [0, 0, 0, 0])
                break

            action = [0.0] * 6
            buttons = [0, 0, 0, 0]

            if len(state) == 2:
                action = [
                    -state[0].y, state[0].x, state[0].z,
                    -state[0].roll, -state[0].pitch, -state[0].yaw,
                    -state[1].y, state[1].x, state[1].z,
                    -state[1].roll, -state[1].pitch, -state[1].yaw
                ]
                buttons = state[0].buttons + state[1].buttons
            elif len(state) == 1:
                action = [
                    -state[0].y, state[0].x, state[0].z,
                    -state[0].roll, -state[0].pitch, -state[0].yaw
                ]
                buttons = state[0].buttons

            self.latest_data["state"] = (action, buttons)

    def get_action(self) -> Tuple[np.ndarray, list]:
        """Returns the latest action and button state of the SpaceMouse."""
        action, buttons = self.latest_data["state"]
        return np.array(action), buttons

    def close(self):
        self.process.terminate()
        self.process.join(timeout=1.0)
        # Release the HID device so a later reconnect in the same process works
        pyspacemouse.close()
        self.manager.shutdown()
