from abc import ABC, abstractmethod
from enum import Enum
import logging
import numpy as np


class MotorStatus(Enum):
    MOTOR_IDLE = 1
    MOTOR_DISCONNECTED = 2
    MOTOR_EXCEPTION = 3
    MOTOR_MOVING = 4


class Motor(ABC):
    def __init__(self, name: str, can_id: int, max_velocity: float, max_current: float):
        self._name = name
        self._can_id = can_id
        self._status = MotorStatus.MOTOR_DISCONNECTED
        self._max_velocity = max_velocity
        self._max_current = max_current
        self.logger = logging.getLogger(self._name)

    # Abstract methods only

    @abstractmethod
    def _connect(self):
        ...

    @abstractmethod
    def _disconnect(self):
        ...

    @abstractmethod
    def _set_velocity(self, velocity: float):
        ...

    @abstractmethod
    def _stop_motor(self):
        ...

    @abstractmethod
    def _read_position(self) -> np.ndarray:
        ...

    @abstractmethod
    def _read_velocity(self) -> float:
        ...

    @abstractmethod
    def _read_current(self) -> float:
        ...

    # Public methods

    @property
    def name(self):
        return self._name

    @property
    def can_id(self):
        return self._can_id

    @property
    def status(self):
        return self._status

    def connect(self):
        if self._status in (MotorStatus.MOTOR_IDLE, MotorStatus.MOTOR_MOVING):
            self.logger.warning(f"Motor {self.name} already connected")
        else:
            self._status = MotorStatus.MOTOR_IDLE
            self._connect()
            self.logger.debug(f"Motor {self._name} connected")

    def disconnect(self):
        if self._status == MotorStatus.MOTOR_DISCONNECTED:
            self.logger.warning(f"Motor {self._name} already diconnected")
        else:
            self._stop_motor()
            self._disconnect()
            self._status = MotorStatus.MOTOR_DISCONNECTED
            self.logger.debug(f"Motor {self._name} diconnected")

    @property
    def max_velocity(self) -> float:
        return self._max_velocity

    @property
    def max_current(self) -> float:
        return self._max_current

    def command_velocity(self, velocity: float) -> None:
        if abs(velocity) > self._max_velocity:
            raise ValueError(
                f"{self._name}: velocity {velocity} "
                f"exceeds max velocity: {self._max_velocity}"
            )
        try:
            self.logger.debug(f"Motor {self._name} being set to {velocity}")
            self._set_velocity(velocity)
            self._status = MotorStatus.MOTOR_MOVING
        except Exception as e:
            self.logger.error(f"Motor {self._name} being set to velocity raised {e}")
            self._status = MotorStatus.MOTOR_EXCEPTION
            raise

    # Context manager

    def __enter__(self) -> "Motor":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if exc_type is not None:
            self.logger.error(f"Motor {self._name} exited with {exc_type}, {exc_val}, {exc_tb}")
        self.disconnect()

    def __repr__(self) -> str:
        return f"Motor(name='{self._name}', can_id={self._can_id}, status={self._status.name})"
