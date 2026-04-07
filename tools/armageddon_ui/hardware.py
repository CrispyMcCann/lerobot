"""Hardware abstraction layer for the ARMageddon control interface.

Provides a ControlState dataclass and abstract interface that can be backed
by either an in-memory mock (for UI testing) or a real Teensy 4.1 serial
connection (for hardware integration).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass

JOINT_AXES = ["J1", "J2", "J3", "J4", "J5", "J6"]
CARTESIAN_AXES = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
MODES = ["joint", "cartesian", "leader"]

# Discrete sensitivity percentage detent positions on the 180° pot
# ~18° of physical rotation per detent; denser at the low end for precision
SENSITIVITY_DETENTS = [1, 5, 10, 20, 30, 50, 70, 85, 100]

# Base jog step at 100% sensitivity (degrees per click of the jog encoder)
BASE_JOG_STEP = 5.0


@dataclass
class ControlState:
    power: bool = False
    mode: str = "joint"
    axis_index: int = 0
    sensitivity_index: int = 5  # default = 50%
    jog_position: float = 0.0

    @property
    def axes(self) -> list[str]:
        if self.mode == "joint":
            return JOINT_AXES
        elif self.mode == "cartesian":
            return CARTESIAN_AXES
        return []

    @property
    def active_axis(self) -> str:
        if self.mode == "leader":
            return "ALL (1:1)"
        axes = self.axes
        if axes and 0 <= self.axis_index < len(axes):
            return axes[self.axis_index]
        return axes[0] if axes else ""

    @property
    def sensitivity_pct(self) -> int:
        return SENSITIVITY_DETENTS[self.sensitivity_index]

    @property
    def jog_step_size(self) -> float:
        """Degrees per jog click = base_step * sensitivity%."""
        return BASE_JOG_STEP * (self.sensitivity_pct / 100.0)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["active_axis"] = self.active_axis
        d["axes"] = self.axes
        d["sensitivity_pct"] = self.sensitivity_pct
        d["jog_step_size"] = self.jog_step_size
        d["sensitivity_detents"] = SENSITIVITY_DETENTS
        d["sensitivity_count"] = len(SENSITIVITY_DETENTS)
        d["base_jog_step"] = BASE_JOG_STEP
        return d


class ControlInterface(ABC):
    @abstractmethod
    async def read_state(self) -> ControlState: ...

    @abstractmethod
    async def on_state_change(self, state: ControlState) -> None: ...

    @abstractmethod
    async def start(self) -> None: ...

    @abstractmethod
    async def stop(self) -> None: ...


class MockInterface(ControlInterface):
    """In-memory mock. State is driven entirely by WebSocket UI events."""

    def __init__(self) -> None:
        self.state = ControlState()

    async def read_state(self) -> ControlState:
        return self.state

    async def on_state_change(self, state: ControlState) -> None:
        self.state = state

    async def start(self) -> None:
        pass

    async def stop(self) -> None:
        pass


class TeensySerialInterface(ControlInterface):
    """Placeholder for future Teensy 4.1 USB serial communication.

    To use: pass --serial /dev/ttyACM0 (or your port) when starting the server.
    Protocol TBD — will read switch/knob states from Teensy and relay them.
    """

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        raise NotImplementedError(
            f"TeensySerialInterface is a stub. To implement:\n"
            f"  1. pip install pyserial\n"
            f"  2. Open serial port {port} at {baudrate} baud\n"
            f"  3. Define a packet protocol with the Teensy firmware\n"
            f"  4. Implement read_state() to parse incoming packets\n"
            f"  5. Implement on_state_change() to send commands back"
        )

    async def read_state(self) -> ControlState:
        raise NotImplementedError

    async def on_state_change(self, state: ControlState) -> None:
        raise NotImplementedError

    async def start(self) -> None:
        raise NotImplementedError

    async def stop(self) -> None:
        raise NotImplementedError
