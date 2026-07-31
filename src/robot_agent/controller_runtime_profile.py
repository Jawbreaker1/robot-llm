"""Small composition boundary for one physical controller runtime."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple


class ControllerRuntimeProfileError(ValueError):
    """A configured controller profile or deployment binding is invalid."""


def _identifier(name: str, value: object, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise ControllerRuntimeProfileError("{} is invalid".format(name))
    return value


@dataclass(frozen=True)
class ControllerRuntimeDescriptor:
    """Read-only identity and capabilities for one execution node."""

    profile_id: str
    robot_id: str
    controller_id: str
    display_name: str
    capabilities: Tuple[str, ...]

    def __post_init__(self) -> None:
        _identifier("profile_id", self.profile_id)
        _identifier("robot_id", self.robot_id)
        _identifier("controller_id", self.controller_id)
        _identifier("display_name", self.display_name, 200)
        if (
            not isinstance(self.capabilities, tuple)
            or not self.capabilities
            or len(set(self.capabilities)) != len(self.capabilities)
        ):
            raise ControllerRuntimeProfileError(
                "controller capabilities are invalid"
            )
        for capability in self.capabilities:
            _identifier("capability", capability, 100)


@dataclass(frozen=True)
class ControllerRuntimeBinding:
    """Deployment-specific binding for a selected controller profile."""

    profile_id: str

    def __post_init__(self) -> None:
        _identifier("profile_id", self.profile_id)


class ControllerRuntimeProfile(ABC):
    """Build the existing three-method dashboard adapter for one controller."""

    @property
    @abstractmethod
    def descriptor(self) -> ControllerRuntimeDescriptor:
        raise NotImplementedError

    @abstractmethod
    def build_adapter(self, binding, *, planner_factory):
        """Compose an adapter without connecting to or starting hardware."""

        raise NotImplementedError


__all__ = (
    "ControllerRuntimeBinding",
    "ControllerRuntimeDescriptor",
    "ControllerRuntimeProfile",
    "ControllerRuntimeProfileError",
)
