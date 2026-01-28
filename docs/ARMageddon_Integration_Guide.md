# ARMageddon Integration Guide for LeRobot

**Ohio State University Capstone Project - Intuitive Leader Arm Controller for Industrial Robots**

This document provides a comprehensive guide for integrating the ARMageddon scaled leader arm system with the LeRobot codebase, including support for joint scaling from a 400mm reach leader arm to the full-scale Yaskawa GP7 industrial robot.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [System Dataflow](#system-dataflow)
3. [Adding a Custom Robot Configuration](#adding-a-custom-robot-configuration)
4. [Adding a Custom Teleoperator (Leader Arm)](#adding-a-custom-teleoperator-leader-arm)
5. [Adding a Custom Motor Bus (ODrive)](#adding-a-custom-motor-bus-odrive)
6. [Implementing Joint Scaling with D-H Parameters](#implementing-joint-scaling-with-d-h-parameters)
7. [Processor Pipeline Integration](#processor-pipeline-integration)
8. [Directory Structure](#directory-structure)
9. [Implementation Checklist](#implementation-checklist)

---

## Architecture Overview

LeRobot follows a modular architecture with clean separation between hardware abstraction and data processing:

```
                                    LeRobot Architecture
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │                                                                             │
    │   ┌──────────────────┐     ┌─────────────────────┐     ┌────────────────┐  │
    │   │   Teleoperator   │     │     Processor       │     │     Robot      │  │
    │   │   (Leader Arm)   │────▶│     Pipeline        │────▶│   (Follower)   │  │
    │   │                  │     │                     │     │                │  │
    │   │  - get_action()  │     │  - JointScaling     │     │ - send_action()│  │
    │   │  - calibrate()   │     │  - FK/IK Transform  │     │ - get_obs()    │  │
    │   │  - connect()     │     │  - Safety Bounds    │     │ - calibrate()  │  │
    │   └──────────────────┘     └─────────────────────┘     └────────────────┘  │
    │            │                         │                         │           │
    │            ▼                         ▼                         ▼           │
    │   ┌──────────────────┐     ┌─────────────────────┐     ┌────────────────┐  │
    │   │    MotorsBus     │     │  ProcessorStep      │     │   MotorsBus    │  │
    │   │   (ODrive CAN)   │     │  Registry           │     │  (Industrial)  │  │
    │   └──────────────────┘     └─────────────────────┘     └────────────────┘  │
    │                                                                             │
    └─────────────────────────────────────────────────────────────────────────────┘
```

### Key Components

| Component | Purpose | Base Class Location |
|-----------|---------|---------------------|
| **Robot** | Hardware interface for follower robots | `src/lerobot/robots/robot.py` |
| **Teleoperator** | Hardware interface for leader arms | `src/lerobot/teleoperators/teleoperator.py` |
| **MotorsBus** | Low-level motor communication | `src/lerobot/motors/motors_bus.py` |
| **ProcessorStep** | Data transformation pipeline | `src/lerobot/processor/pipeline.py` |
| **RobotKinematics** | FK/IK computations | `src/lerobot/model/kinematics.py` |

---

## System Dataflow

### Complete Teleoperation Flow

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                          ARMageddon Teleoperation Dataflow                          │
└─────────────────────────────────────────────────────────────────────────────────────┘

    ARMageddon Leader Arm                                      Yaskawa GP7 Robot
    (400mm reach, 0.38 scale)                                  (1052mm reach)
    ┌─────────────────────────┐                                ┌─────────────────────┐
    │                         │                                │                     │
    │   EaglePower 8308/5010  │                                │   Industrial        │
    │   Motors (ODrive)       │                                │   Servos            │
    │                         │                                │                     │
    └───────────┬─────────────┘                                └──────────┬──────────┘
                │                                                         │
                ▼                                                         ▼
    ┌───────────────────────────┐                              ┌────────────────────┐
    │   ODriveMotorsBus         │                              │   ROS2 Interface   │
    │   (CAN bus @ ≥200Hz)      │                              │   (Isaac Sim)      │
    └───────────┬───────────────┘                              └──────────▲─────────┘
                │                                                         │
                │  Raw joint positions                                    │  Scaled joint
                │  (degrees or normalized)                                │  positions
                ▼                                                         │
    ┌───────────────────────────┐                              ┌──────────┴─────────┐
    │   ARMageddonLeader        │                              │   GP7Follower      │
    │   (Teleoperator)          │                              │   (Robot)          │
    │                           │                              │                    │
    │   - get_action()          │                              │   - send_action()  │
    │   - action_features       │                              │   - action_features│
    └───────────┬───────────────┘                              └──────────▲─────────┘
                │                                                         │
                │  action = {"j1.pos": θ₁, "j2.pos": θ₂, ...}            │
                ▼                                                         │
    ┌─────────────────────────────────────────────────────────────────────┴─────────┐
    │                                                                               │
    │                        DataProcessorPipeline                                  │
    │                                                                               │
    │   ┌─────────────────────────────────────────────────────────────────────┐    │
    │   │                     ProcessorStep Chain                              │    │
    │   │                                                                      │    │
    │   │   ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │    │
    │   │   │   Calibrate  │───▶│  DH Scale    │───▶│  Safety/Bounds       │  │    │
    │   │   │   Normalize  │    │  Transform   │    │  Clipping            │  │    │
    │   │   └──────────────┘    └──────────────┘    └──────────────────────┘  │    │
    │   │                                                                      │    │
    │   │   Input: leader θ    Output: follower θ                             │    │
    │   │   (400mm workspace)   (1052mm workspace)                            │    │
    │   │                                                                      │    │
    │   └─────────────────────────────────────────────────────────────────────┘    │
    │                                                                               │
    └───────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────────────────┐
    │                          D-H Parameter Scaling                                   │
    │                                                                                  │
    │   For kinematically similar robots with scale factor k = 0.38:                  │
    │                                                                                  │
    │   Leader D-H:  a_leader, d_leader, α, θ                                         │
    │   GP7 D-H:     a_gp7 = a_leader/k,  d_gp7 = d_leader/k,  α_same,  θ_same       │
    │                                                                                  │
    │   Joint angles remain IDENTICAL (1:1 mapping):                                  │
    │   θ_gp7 = θ_leader  (no transformation needed for angles!)                      │
    │                                                                                  │
    │   Only link lengths (a, d) are scaled; joint angles pass through directly.     │
    └─────────────────────────────────────────────────────────────────────────────────┘
```

### Data Types at Each Stage

```python
# 1. Raw motor reading (normalized [-100, 100] or degrees)
raw_position: float  # e.g., 45.0 degrees

# 2. Teleoperator action dict
leader_action: RobotAction = {
    "shoulder_pan.pos": 45.0,
    "shoulder_lift.pos": -30.0,
    "elbow_flex.pos": 60.0,
    "wrist_flex.pos": 15.0,
    "wrist_roll.pos": 0.0,
    "gripper.pos": 50.0,  # 0-100% open
}

# 3. After processing (identical for scaled D-H)
follower_action: RobotAction = {
    "j1.pos": 45.0,   # Same angles!
    "j2.pos": -30.0,
    "j3.pos": 60.0,
    "j4.pos": 15.0,
    "j5.pos": 0.0,
    "gripper.pos": 50.0,
}
```

---

## Adding a Custom Robot Configuration

### Step 1: Create Directory Structure

```
src/lerobot/robots/gp7_follower/
├── __init__.py
├── config_gp7_follower.py
└── gp7_follower.py
```

### Step 2: Define the Configuration (`config_gp7_follower.py`)

```python
#!/usr/bin/env python
"""Configuration for Yaskawa GP7 follower robot."""

from dataclasses import dataclass, field
from lerobot.cameras import CameraConfig
from ..config import RobotConfig


@dataclass
class GP7FollowerConfig:
    """Base configuration for GP7 follower robot."""

    # ROS2 topic or communication endpoint
    ros_namespace: str = "/gp7"

    # Isaac Sim connection settings
    isaac_sim_host: str = "localhost"
    isaac_sim_port: int = 45670

    # Safety limits (degrees)
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=lambda: {
        "j1": (-170, 170),
        "j2": (-65, 145),
        "j3": (-70, 190),
        "j4": (-190, 190),
        "j5": (-135, 135),
        "j6": (-360, 360),
    })

    # Maximum velocity limit (deg/s) for safety
    max_joint_velocity: float = 180.0

    # Emergency stop threshold (deg/s)
    emergency_stop_threshold: float = 360.0

    # cameras (optional)
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


@RobotConfig.register_subclass("gp7_follower")
@dataclass
class GP7FollowerRobotConfig(RobotConfig, GP7FollowerConfig):
    """Registered config for GP7 follower."""
    pass
```

### Step 3: Implement the Robot Class (`gp7_follower.py`)

```python
#!/usr/bin/env python
"""Yaskawa GP7 follower robot implementation for LeRobot."""

import logging
from functools import cached_property

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_gp7_follower import GP7FollowerRobotConfig

logger = logging.getLogger(__name__)


class GP7Follower(Robot):
    """
    Yaskawa GP7 industrial robot follower.

    Communicates with the robot via ROS2 and NVIDIA Isaac Sim.
    Implements the LeRobot Robot interface for seamless integration.
    """

    config_class = GP7FollowerRobotConfig
    name = "gp7_follower"

    # Joint names matching Yaskawa convention
    JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

    def __init__(self, config: GP7FollowerRobotConfig):
        super().__init__(config)
        self.config = config
        self._connected = False

        # ROS2/Isaac Sim client (to be implemented)
        self._ros_client = None

        # Cameras
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        """Motor feature types."""
        return {f"{joint}.pos": float for joint in self.JOINT_NAMES}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Camera feature types."""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Features returned by get_observation()."""
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Features expected by send_action()."""
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Connect to GP7 via ROS2/Isaac Sim."""
        # TODO: Initialize ROS2 node and connect to Isaac Sim
        # self._ros_client = ROS2Client(self.config.ros_namespace)
        # self._ros_client.connect()

        for cam in self.cameras.values():
            cam.connect()

        self._connected = True
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        """GP7 has built-in encoder calibration."""
        return True

    def calibrate(self) -> None:
        """No calibration needed for industrial robot."""
        logger.info("GP7 uses built-in encoder calibration.")

    def configure(self) -> None:
        """Apply runtime configuration."""
        # TODO: Set velocity limits, enable servo, etc.
        pass

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        """Read current joint positions from GP7."""
        obs_dict = {}

        # TODO: Read joint positions from ROS2
        # joint_state = self._ros_client.get_joint_state()
        # for i, joint in enumerate(self.JOINT_NAMES):
        #     obs_dict[f"{joint}.pos"] = joint_state.position[i]

        # Placeholder
        for joint in self.JOINT_NAMES:
            obs_dict[f"{joint}.pos"] = 0.0

        # Capture camera images
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Send joint position commands to GP7."""
        # Extract joint positions
        goal_positions = {}
        for joint in self.JOINT_NAMES:
            key = f"{joint}.pos"
            if key in action:
                goal_positions[joint] = action[key]

        # TODO: Apply safety checks
        # TODO: Send via ROS2 to Isaac Sim / real robot
        # self._ros_client.send_joint_command(goal_positions)

        return action

    @check_if_not_connected
    def disconnect(self):
        """Disconnect from GP7."""
        # TODO: Clean shutdown
        # self._ros_client.disconnect()

        for cam in self.cameras.values():
            cam.disconnect()

        self._connected = False
        logger.info(f"{self} disconnected.")
```

### Step 4: Update `__init__.py`

```python
# src/lerobot/robots/gp7_follower/__init__.py
from .config_gp7_follower import GP7FollowerConfig, GP7FollowerRobotConfig
from .gp7_follower import GP7Follower
```

### Step 5: Register in Factory

Add to `src/lerobot/robots/utils.py`:

```python
def make_robot_from_config(config: RobotConfig) -> Robot:
    # ... existing code ...
    elif config.type == "gp7_follower":
        from .gp7_follower import GP7Follower
        return GP7Follower(config)
    # ... rest of function ...
```

---

## Adding a Custom Teleoperator (Leader Arm)

### Step 1: Create Directory Structure

```
src/lerobot/teleoperators/armageddon_leader/
├── __init__.py
├── config_armageddon_leader.py
└── armageddon_leader.py
```

### Step 2: Define Configuration (`config_armageddon_leader.py`)

```python
#!/usr/bin/env python
"""Configuration for ARMageddon leader arm teleoperator."""

from dataclasses import dataclass
from ..config import TeleoperatorConfig


@dataclass
class ARMageddonLeaderConfig:
    """Base configuration for ARMageddon leader arm."""

    # CAN bus interface (e.g., "can0" for SocketCAN)
    can_interface: str = "can0"

    # ODrive node IDs for each axis
    odrive_node_ids: dict[str, int] = None

    # D-H parameters for the scaled leader arm (400mm reach)
    # Format: {"joint": {"a": mm, "d": mm, "alpha": rad, "theta_offset": rad}}
    dh_params: dict[str, dict[str, float]] = None

    # Scale factor relative to GP7 (leader_reach / gp7_reach)
    scale_factor: float = 0.38  # 400mm / 1052mm

    # Use degrees for joint values (vs normalized [-100, 100])
    use_degrees: bool = True

    # Brake control GPIO pins
    brake_gpio_pins: dict[str, int] = None

    def __post_init__(self):
        if self.odrive_node_ids is None:
            self.odrive_node_ids = {
                "shoulder_pan": 0,
                "shoulder_lift": 1,
                "elbow_flex": 2,
                "wrist_flex": 3,
                "wrist_roll": 4,
                "gripper": 5,
            }
        if self.dh_params is None:
            # Default D-H parameters for ARMageddon (scaled from GP7)
            # GP7 link lengths scaled by 0.38
            self.dh_params = {
                "shoulder_pan":  {"a": 0,     "d": 113.4, "alpha": -1.5708, "theta_offset": 0},
                "shoulder_lift": {"a": 106.4, "d": 0,     "alpha": 0,       "theta_offset": -1.5708},
                "elbow_flex":    {"a": 106.4, "d": 0,     "alpha": 0,       "theta_offset": 0},
                "wrist_flex":    {"a": 0,     "d": 76.0,  "alpha": -1.5708, "theta_offset": 0},
                "wrist_roll":    {"a": 0,     "d": 0,     "alpha": 1.5708,  "theta_offset": 0},
            }


@TeleoperatorConfig.register_subclass("armageddon_leader")
@dataclass
class ARMageddonLeaderTeleopConfig(TeleoperatorConfig, ARMageddonLeaderConfig):
    """Registered config for ARMageddon leader."""
    pass
```

### Step 3: Implement Teleoperator Class (`armageddon_leader.py`)

```python
#!/usr/bin/env python
"""ARMageddon leader arm teleoperator for LeRobot."""

import logging
import time
from typing import Any

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from ..teleoperator import Teleoperator
from .config_armageddon_leader import ARMageddonLeaderTeleopConfig

logger = logging.getLogger(__name__)


class ARMageddonLeader(Teleoperator):
    """
    ARMageddon scaled leader arm teleoperator.

    Uses ODrive motor controllers via CAN bus for high-frequency
    position feedback (≥200Hz target).
    """

    config_class = ARMageddonLeaderTeleopConfig
    name = "armageddon_leader"

    # Motor names matching the config
    MOTOR_NAMES = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]

    def __init__(self, config: ARMageddonLeaderTeleopConfig):
        super().__init__(config)
        self.config = config
        self._connected = False

        # ODrive CAN bus interface (to be implemented)
        self._odrive_bus = None

    @property
    def action_features(self) -> dict[str, type]:
        """Features returned by get_action()."""
        return {f"{motor}.pos": float for motor in self.MOTOR_NAMES}

    @property
    def feedback_features(self) -> dict[str, type]:
        """Features for force feedback (future implementation)."""
        return {}  # No force feedback yet

    @property
    def is_connected(self) -> bool:
        return self._connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Connect to ODrive controllers via CAN bus."""
        logger.info(f"Connecting {self} to CAN interface: {self.config.can_interface}")

        # TODO: Initialize CAN bus connection
        # self._odrive_bus = ODriveCANBus(
        #     interface=self.config.can_interface,
        #     node_ids=self.config.odrive_node_ids,
        # )
        # self._odrive_bus.connect()

        if not self.is_calibrated and calibrate:
            self.calibrate()

        self.configure()
        self._connected = True
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        """Check if ODrive encoders are calibrated."""
        # TODO: Check ODrive calibration state
        return True

    def calibrate(self) -> None:
        """Run ODrive encoder calibration."""
        logger.info(f"Calibrating {self}...")

        # TODO: Run ODrive encoder calibration sequence
        # for motor in self.MOTOR_NAMES:
        #     self._odrive_bus.calibrate_encoder(motor)

        self._save_calibration()
        logger.info("Calibration complete.")

    def configure(self) -> None:
        """Configure ODrive for passive readout mode."""
        # TODO: Set ODrive to idle mode (no torque, just read encoders)
        # for motor in self.MOTOR_NAMES:
        #     self._odrive_bus.set_idle_mode(motor)
        pass

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        """
        Read current joint positions from ODrive encoders.

        Returns joint positions in degrees or normalized format
        based on configuration.
        """
        start = time.perf_counter()

        action = {}
        # TODO: Read from ODrive CAN bus
        # for motor in self.MOTOR_NAMES:
        #     raw_pos = self._odrive_bus.get_position(motor)
        #     action[f"{motor}.pos"] = self._normalize_position(motor, raw_pos)

        # Placeholder
        for motor in self.MOTOR_NAMES:
            action[f"{motor}.pos"] = 0.0

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")

        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Send force feedback (not implemented yet)."""
        raise NotImplementedError("Force feedback not yet implemented")

    @check_if_not_connected
    def disconnect(self) -> None:
        """Disconnect from ODrive controllers."""
        # TODO: Clean shutdown
        # self._odrive_bus.disconnect()

        self._connected = False
        logger.info(f"{self} disconnected.")
```

---

## Adding a Custom Motor Bus (ODrive)

For the ARMageddon leader arm, you'll need a custom motor bus implementation for ODrive controllers over CAN.

### ODrive Motor Bus Implementation

```python
# src/lerobot/motors/odrive/odrive.py
"""ODrive motor bus implementation using CAN bus."""

import logging
from dataclasses import dataclass
from typing import Any

from ..motors_bus import MotorsBusBase, Motor, MotorCalibration, MotorNormMode

logger = logging.getLogger(__name__)


@dataclass
class ODriveMotor(Motor):
    """Extended motor class for ODrive-specific parameters."""
    encoder_cpr: int = 8192  # Counts per revolution
    gear_ratio: float = 1.0


class ODriveCANBus(MotorsBusBase):
    """
    ODrive motor bus using CAN interface.

    Supports high-frequency position readout (≥200Hz) for teleoperation.
    Uses python-can library for SocketCAN interface.
    """

    def __init__(
        self,
        interface: str,  # e.g., "can0"
        motors: dict[str, ODriveMotor],
        calibration: dict[str, MotorCalibration] | None = None,
        bitrate: int = 1000000,  # 1Mbps CAN
    ):
        super().__init__(interface, motors, calibration)
        self.bitrate = bitrate
        self._can_bus = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, handshake: bool = True) -> None:
        """Initialize CAN bus connection."""
        import can  # python-can library

        self._can_bus = can.Bus(
            interface="socketcan",
            channel=self.port,  # "can0"
            bitrate=self.bitrate,
        )
        self._connected = True

        if handshake:
            self._verify_odrives()

        logger.info(f"ODrive CAN bus connected on {self.port}")

    def disconnect(self, disable_torque: bool = True) -> None:
        """Close CAN bus connection."""
        if disable_torque:
            self.disable_torque()

        if self._can_bus:
            self._can_bus.shutdown()

        self._connected = False
        logger.info("ODrive CAN bus disconnected")

    def _verify_odrives(self):
        """Verify all ODrives respond to heartbeat."""
        # TODO: Send heartbeat request and verify responses
        pass

    def read(self, data_name: str, motor: str) -> float:
        """Read a value from ODrive."""
        if data_name == "Present_Position":
            return self._read_encoder_position(motor)
        elif data_name == "Present_Velocity":
            return self._read_encoder_velocity(motor)
        else:
            raise ValueError(f"Unknown data_name: {data_name}")

    def write(self, data_name: str, motor: str, value: float) -> None:
        """Write a value to ODrive."""
        # For leader arm, we mostly read; writes are for configuration
        pass

    def sync_read(self, data_name: str, motors: list[str] | None = None) -> dict[str, float]:
        """Read from multiple motors."""
        if motors is None:
            motors = list(self.motors.keys())

        return {motor: self.read(data_name, motor) for motor in motors}

    def sync_write(self, data_name: str, values: dict[str, float]) -> None:
        """Write to multiple motors."""
        for motor, value in values.items():
            self.write(data_name, motor, value)

    def enable_torque(self, motors: list[str] | None = None, num_retry: int = 0) -> None:
        """Enable motor torque (closed-loop control)."""
        # TODO: Send ODrive state transition command
        pass

    def disable_torque(self, motors: list[str] | None = None, num_retry: int = 0) -> None:
        """Disable motor torque (idle mode)."""
        # TODO: Send ODrive state transition to IDLE
        pass

    def read_calibration(self) -> dict[str, MotorCalibration]:
        """Read calibration from ODrive."""
        # TODO: Read encoder offset from ODrive
        return {}

    def write_calibration(self, calibration: dict[str, MotorCalibration], cache: bool = True) -> None:
        """Write calibration to ODrive."""
        # TODO: Write encoder offset to ODrive
        if cache:
            self.calibration = calibration

    def _read_encoder_position(self, motor: str) -> float:
        """Read encoder position in degrees."""
        # TODO: Send CAN message and read response
        # Example ODrive CAN protocol:
        # node_id = self.motors[motor].id
        # msg = can.Message(arbitration_id=(node_id << 5) | 0x09, is_extended_id=False)
        # self._can_bus.send(msg)
        # response = self._can_bus.recv(timeout=0.01)
        # raw_counts = struct.unpack('<f', response.data[:4])[0]
        # return raw_counts * 360.0 / self.motors[motor].encoder_cpr
        return 0.0

    def _read_encoder_velocity(self, motor: str) -> float:
        """Read encoder velocity in degrees/second."""
        # TODO: Implement velocity readout
        return 0.0
```

---

## Implementing Joint Scaling with D-H Parameters

Since your leader arm uses **scaled D-H parameters** (kinematically similar to GP7), joint angles map **1:1** between leader and follower. This is a key advantage of your design!

### Joint Scaling Processor Step

```python
# src/lerobot/processor/armageddon_scale_processor.py
"""Joint scaling processor for ARMageddon leader arm."""

from dataclasses import dataclass
from typing import Any

import numpy as np

from lerobot.configs.types import PipelineFeatureType, PolicyFeature, FeatureType
from lerobot.processor import (
    RobotAction,
    RobotActionProcessorStep,
    ProcessorStepRegistry,
)


@ProcessorStepRegistry.register("armageddon_joint_scale")
@dataclass
class ARMageddonJointScale(RobotActionProcessorStep):
    """
    Transforms joint positions from ARMageddon leader arm to GP7 follower.

    For kinematically similar robots with scaled D-H parameters:
    - Joint angles remain IDENTICAL (1:1 mapping)
    - Only link lengths differ by scale factor
    - End-effector positions scale by the same factor

    This processor handles:
    1. Joint name mapping (leader names -> GP7 names)
    2. Optional joint limit verification
    3. Optional velocity limiting for safety
    """

    # Scale factor: leader_reach / follower_reach
    scale_factor: float = 0.38  # 400mm / 1052mm

    # Joint name mapping: leader_name -> follower_name
    joint_mapping: dict[str, str] = None

    # GP7 joint limits (degrees)
    follower_joint_limits: dict[str, tuple[float, float]] = None

    # Maximum joint velocity (degrees/second)
    max_velocity: float | None = None

    # Previous joint positions for velocity calculation
    _prev_positions: dict[str, float] | None = None
    _prev_time: float | None = None

    def __post_init__(self):
        if self.joint_mapping is None:
            self.joint_mapping = {
                "shoulder_pan": "j1",
                "shoulder_lift": "j2",
                "elbow_flex": "j3",
                "wrist_flex": "j4",
                "wrist_roll": "j5",
                "gripper": "gripper",
            }

        if self.follower_joint_limits is None:
            self.follower_joint_limits = {
                "j1": (-170, 170),
                "j2": (-65, 145),
                "j3": (-70, 190),
                "j4": (-190, 190),
                "j5": (-135, 135),
                "j6": (-360, 360),
            }

    def action(self, action: RobotAction) -> RobotAction:
        """
        Transform leader joint positions to follower joint positions.

        For scaled D-H parameters, joint angles are identical!
        Only need to map joint names and verify limits.
        """
        import time
        current_time = time.perf_counter()

        transformed_action = {}

        for leader_name, follower_name in self.joint_mapping.items():
            leader_key = f"{leader_name}.pos"
            follower_key = f"{follower_name}.pos"

            if leader_key not in action:
                continue

            # 1:1 joint angle mapping for scaled D-H
            joint_angle = float(action[leader_key])

            # Verify within follower limits
            if follower_name in self.follower_joint_limits:
                limit_min, limit_max = self.follower_joint_limits[follower_name]
                joint_angle = np.clip(joint_angle, limit_min, limit_max)

            # Optional velocity limiting
            if self.max_velocity is not None and self._prev_positions is not None:
                prev_angle = self._prev_positions.get(follower_key, joint_angle)
                dt = current_time - self._prev_time if self._prev_time else 0.005

                if dt > 0:
                    velocity = abs(joint_angle - prev_angle) / dt
                    if velocity > self.max_velocity:
                        # Limit the step size
                        max_step = self.max_velocity * dt
                        if joint_angle > prev_angle:
                            joint_angle = prev_angle + max_step
                        else:
                            joint_angle = prev_angle - max_step

            transformed_action[follower_key] = joint_angle

        # Update state for velocity calculation
        self._prev_positions = transformed_action.copy()
        self._prev_time = current_time

        return transformed_action

    def reset(self):
        """Reset velocity limiting state."""
        self._prev_positions = None
        self._prev_time = None

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        """Update feature names from leader to follower convention."""
        action_features = features.get(PipelineFeatureType.ACTION, {})

        # Remove leader names, add follower names
        for leader_name, follower_name in self.joint_mapping.items():
            leader_key = f"{leader_name}.pos"
            follower_key = f"{follower_name}.pos"

            if leader_key in action_features:
                action_features.pop(leader_key)

            action_features[follower_key] = PolicyFeature(
                type=FeatureType.ACTION, shape=(1,)
            )

        features[PipelineFeatureType.ACTION] = action_features
        return features
```

### Alternative: End-Effector Space Scaling (if needed)

If you need to operate in end-effector space (e.g., for different kinematics):

```python
@ProcessorStepRegistry.register("armageddon_ee_scale")
@dataclass
class ARMageddonEEScale(RobotActionProcessorStep):
    """
    Scale end-effector positions between workspaces.

    Use this if leader and follower have different kinematics
    and you need to map EE positions.
    """

    scale_factor: float = 0.38
    leader_workspace_center: np.ndarray = None  # (x, y, z) in mm
    follower_workspace_center: np.ndarray = None

    def action(self, action: RobotAction) -> RobotAction:
        """Scale EE position from leader to follower workspace."""
        # Extract leader EE position
        leader_pos = np.array([
            action.get("ee.x", 0.0),
            action.get("ee.y", 0.0),
            action.get("ee.z", 0.0),
        ])

        # Transform: scale position relative to workspace center
        if self.leader_workspace_center is not None:
            leader_pos = leader_pos - self.leader_workspace_center

        # Scale by inverse of scale factor (leader smaller than follower)
        follower_pos = leader_pos / self.scale_factor

        if self.follower_workspace_center is not None:
            follower_pos = follower_pos + self.follower_workspace_center

        # Update action
        action["ee.x"] = float(follower_pos[0])
        action["ee.y"] = float(follower_pos[1])
        action["ee.z"] = float(follower_pos[2])

        # Orientation stays the same for kinematically similar arms
        # (wx, wy, wz unchanged)

        return action
```

---

## Processor Pipeline Integration

### Creating a Teleoperation Pipeline

```python
# Example: teleoperate.py

from lerobot.processor import DataProcessorPipeline
from lerobot.processor.armageddon_scale_processor import ARMageddonJointScale
from lerobot.teleoperators.armageddon_leader import ARMageddonLeader
from lerobot.robots.gp7_follower import GP7Follower


def create_teleop_pipeline():
    """Create the processor pipeline for ARMageddon -> GP7 teleoperation."""

    return DataProcessorPipeline(
        name="armageddon_to_gp7",
        steps=[
            # Step 1: Map joint names and verify limits
            ARMageddonJointScale(
                scale_factor=0.38,
                max_velocity=180.0,  # deg/s safety limit
            ),

            # Step 2: Add any additional safety processing
            # ... (e.g., workspace bounds, collision checking)
        ],
    )


def main():
    # Create devices
    leader_config = ARMageddonLeaderTeleopConfig(
        can_interface="can0",
        use_degrees=True,
    )

    follower_config = GP7FollowerRobotConfig(
        ros_namespace="/gp7",
    )

    leader = ARMageddonLeader(leader_config)
    follower = GP7Follower(follower_config)
    pipeline = create_teleop_pipeline()

    # Connect
    leader.connect()
    follower.connect()

    try:
        # Teleoperation loop
        while True:
            # 1. Read leader position
            leader_action = leader.get_action()

            # 2. Transform through pipeline
            follower_action = pipeline.process_action(leader_action)

            # 3. Send to follower
            follower.send_action(follower_action)

    finally:
        leader.disconnect()
        follower.disconnect()


if __name__ == "__main__":
    main()
```

---

## Directory Structure

After implementing all components, your directory structure should look like:

```
src/lerobot/
├── motors/
│   ├── odrive/                          # NEW: ODrive motor bus
│   │   ├── __init__.py
│   │   ├── odrive.py
│   │   └── tables.py                    # Register addresses if needed
│   └── ...
│
├── teleoperators/
│   ├── armageddon_leader/               # NEW: ARMageddon leader arm
│   │   ├── __init__.py
│   │   ├── config_armageddon_leader.py
│   │   └── armageddon_leader.py
│   └── ...
│
├── robots/
│   ├── gp7_follower/                    # NEW: Yaskawa GP7 robot
│   │   ├── __init__.py
│   │   ├── config_gp7_follower.py
│   │   └── gp7_follower.py
│   └── ...
│
├── processor/
│   ├── armageddon_scale_processor.py    # NEW: Joint scaling processor
│   └── ...
│
└── model/
    └── kinematics.py                    # Can be used if needed
```

---

## Implementation Checklist

Use this checklist to track your progress:

### Phase 1: Basic Infrastructure
- [ ] Create `ODriveCANBus` motor bus class
- [ ] Implement basic CAN communication (python-can)
- [ ] Test encoder readout at ≥200Hz
- [ ] Create `ARMageddonLeader` teleoperator class
- [ ] Test teleoperator `get_action()` returns valid data

### Phase 2: Robot Interface
- [ ] Create `GP7Follower` robot class
- [ ] Implement ROS2 client for Isaac Sim communication
- [ ] Test `send_action()` moves simulated robot
- [ ] Verify action-observation loop works

### Phase 3: Joint Scaling Pipeline
- [ ] Implement `ARMageddonJointScale` processor
- [ ] Verify 1:1 joint angle mapping works
- [ ] Add joint limit checking
- [ ] Add velocity limiting for safety
- [ ] Create complete teleoperation pipeline

### Phase 4: Safety & Testing
- [ ] Implement emergency stop (< 100ms response)
- [ ] Add disc brake control integration
- [ ] Test end-to-end teleoperation in Isaac Sim
- [ ] Verify ±0.1 inch (±2.54mm) accuracy requirement
- [ ] Document calibration procedures

### Phase 5: Advanced Features (Optional)
- [ ] Add force feedback via `send_feedback()`
- [ ] Implement collision avoidance
- [ ] Add data recording for imitation learning
- [ ] Train policies on recorded demonstrations

---

## Key Technical Notes

### D-H Parameter Advantage

Your choice of scaled D-H parameters is excellent for teleoperation:

```
┌────────────────────────────────────────────────────────────────────┐
│                      Why Scaled D-H Works                          │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│  GP7 D-H Parameters:                                               │
│  a₁, d₁, α₁, θ₁ → T₁                                               │
│  a₂, d₂, α₂, θ₂ → T₂                                               │
│  ...                                                               │
│                                                                    │
│  Leader D-H Parameters (k = 0.38):                                 │
│  k·a₁, k·d₁, α₁, θ₁ → T₁'                                          │
│  k·a₂, k·d₂, α₂, θ₂ → T₂'                                          │
│  ...                                                               │
│                                                                    │
│  Since α and θ are unchanged:                                      │
│  • Joint angles map 1:1 (no computation needed!)                  │
│  • End-effector orientation is identical                           │
│  • EE position scales linearly: p_gp7 = p_leader / k               │
│                                                                    │
│  This means your control loop is SIMPLE:                           │
│  θ_gp7 = θ_leader (direct copy!)                                   │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

### Communication Timing

To meet your ≥200Hz requirement:

```python
# Target loop time: 5ms (200Hz)
# Budget breakdown:
#   - CAN read:  ~1ms (ODrive supports up to 1kHz)
#   - Processing: ~0.5ms (simple scaling)
#   - ROS2 send: ~2ms (depends on network)
#   - Buffer:    ~1.5ms

import time

LOOP_PERIOD = 0.005  # 5ms = 200Hz

while running:
    loop_start = time.perf_counter()

    # Your teleoperation code here
    action = leader.get_action()
    scaled_action = pipeline.process_action(action)
    follower.send_action(scaled_action)

    # Maintain consistent loop rate
    elapsed = time.perf_counter() - loop_start
    sleep_time = LOOP_PERIOD - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)
```

---

## Getting Help

- **LeRobot Documentation**: See `docs/` folder in this repository
- **LeRobot Issues**: https://github.com/huggingface/lerobot/issues
- **ODrive Documentation**: https://docs.odriverobotics.com/
- **python-can**: https://python-can.readthedocs.io/

Good luck with your capstone project! Your kinematically similar scaled design is a smart approach that simplifies the control challenge significantly.
