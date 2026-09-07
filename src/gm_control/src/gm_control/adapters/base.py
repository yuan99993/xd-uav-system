from dataclasses import dataclass
from typing import Optional


@dataclass
class GimbalCommandData:
    mode: str = "rate"
    valid: bool = False
    yaw_rate_deg_s: float = 0.0
    pitch_rate_deg_s: float = 0.0
    roll_rate_deg_s: float = 0.0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0


@dataclass
class GimbalStateData:
    valid: bool = False
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_rate_deg_s: float = 0.0
    pitch_rate_deg_s: float = 0.0
    roll_rate_deg_s: float = 0.0


class BaseGimbalAdapter:
    """Backend boundary for Gazebo, MAVLink, serial, UDP, or vendor SDK gimbals."""

    def send_command(self, command: GimbalCommandData) -> None:
        raise NotImplementedError

    def get_state(self) -> Optional[GimbalStateData]:
        return None
