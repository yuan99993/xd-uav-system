"""
Coordinate Transformer for gimbal and body-frame transformations.

Ported from PixEagle src/classes/coordinate_transformer.py

Handles conversions between:
- Gimbal coordinate systems
- Aircraft body frame
- NED (North-East-Down) frame
- Normalized screen coordinates
"""

import math
import numpy as np
import logging
from typing import Tuple, Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class FrameType(Enum):
    """Coordinate frame types."""
    GIMBAL_BODY = "gimbal_body"
    AIRCRAFT_BODY = "aircraft_body"
    NED = "ned"
    NORMALIZED = "normalized"


@dataclass
class CameraParameters:
    """Camera calibration and mounting parameters."""
    mount_offset_roll: float = 0.0
    mount_offset_pitch: float = 0.0
    mount_offset_yaw: float = 0.0
    fov_horizontal: float = 60.0
    fov_vertical: float = 45.0
    focal_length_x: float = 1.0
    focal_length_y: float = 1.0


class CoordinateTransformer:
    """
    Coordinate transformation utility for gimbal-based tracking.

    Provides transformations between gimbal angles, body frame vectors,
    NED coordinates, and normalized screen coordinates.
    """

    def __init__(self, camera_params: Optional[CameraParameters] = None):
        self.camera_params = camera_params or CameraParameters()
        logger.debug("[CoordinateTransformer] Initialized")

    # ── Rotation Helpers ─────────────────────────────────────────────────

    @staticmethod
    def rotation_matrix_x(angle_rad: float) -> np.ndarray:
        """Rotation matrix about X axis."""
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    @staticmethod
    def rotation_matrix_y(angle_rad: float) -> np.ndarray:
        """Rotation matrix about Y axis."""
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    @staticmethod
    def rotation_matrix_z(angle_rad: float) -> np.ndarray:
        """Rotation matrix about Z axis."""
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    # ── Core Transformations ─────────────────────────────────────────────

    def gimbal_angles_to_body_vector(
        self, yaw: float, pitch: float, roll: float = 0.0,
        include_mount_offset: bool = True,
    ) -> np.ndarray:
        """
        Convert gimbal angles to unit vector in aircraft body frame.

        Args:
            yaw: Gimbal yaw angle in degrees (+ = right)
            pitch: Gimbal pitch angle in degrees (+ = up)
            roll: Gimbal roll angle in degrees (+ = clockwise)
            include_mount_offset: Apply camera mount offset corrections

        Returns:
            Unit vector [x, y, z] in aircraft body frame
            (x=forward, y=right, z=down)
        """
        if include_mount_offset:
            total_yaw = yaw + self.camera_params.mount_offset_yaw
            total_pitch = pitch + self.camera_params.mount_offset_pitch
            total_roll = roll + self.camera_params.mount_offset_roll
        else:
            total_yaw = yaw
            total_pitch = pitch
            total_roll = roll

        yaw_rad = math.radians(total_yaw)
        pitch_rad = math.radians(total_pitch)
        roll_rad = math.radians(total_roll)

        # Initial forward-looking vector (x-forward)
        forward = np.array([1.0, 0.0, 0.0])

        # Apply pitch (rotation around Y)
        Ry = self.rotation_matrix_y(pitch_rad)
        # Apply yaw (rotation around Z)
        Rz = self.rotation_matrix_z(-yaw_rad)

        vector = Rz @ Ry @ forward

        # Normalize
        norm = np.linalg.norm(vector)
        if norm > 1e-10:
            vector = vector / norm

        return vector

    def body_to_ned_vector(self, body_vector: np.ndarray,
                           aircraft_yaw: float) -> np.ndarray:
        """
        Transform body-frame vector to NED frame.

        Args:
            body_vector: [x, y, z] in aircraft body frame
            aircraft_yaw: Aircraft yaw angle in degrees

        Returns:
            [north, east, down] vector in NED frame
        """
        yaw_rad = math.radians(aircraft_yaw)
        Rz = self.rotation_matrix_z(yaw_rad)
        return Rz @ body_vector

    def pixel_to_normalized(
        self, px: float, py: float,
        frame_width: int, frame_height: int,
    ) -> Tuple[float, float]:
        """
        Convert pixel coordinates to normalized [0, 1] coordinates.

        Args:
            px, py: Pixel coordinates
            frame_width, frame_height: Frame dimensions

        Returns:
            (nx, ny) normalized coordinates
        """
        if frame_width <= 0 or frame_height <= 0:
            return (0.0, 0.0)
        nx = max(0.0, min(1.0, px / frame_width))
        ny = max(0.0, min(1.0, py / frame_height))
        return (nx, ny)

    def normalized_to_pixel(
        self, nx: float, ny: float,
        frame_width: int, frame_height: int,
    ) -> Tuple[int, int]:
        """
        Convert normalized [0, 1] coordinates to pixel coordinates.

        Args:
            nx, ny: Normalized coordinates
            frame_width, frame_height: Frame dimensions

        Returns:
            (px, py) pixel coordinates
        """
        px = int(round(nx * frame_width))
        py = int(round(ny * frame_height))
        return (px, py)

    def pixel_to_angle_error(
        self, px: float, py: float,
        frame_width: int, frame_height: int,
        fov_horizontal: Optional[float] = None,
        fov_vertical: Optional[float] = None,
    ) -> Tuple[float, float]:
        """
        Convert pixel offset from center to angular error (degrees).

        Args:
            px, py: Pixel coordinates
            frame_width, frame_height: Frame dimensions
            fov_horizontal, fov_vertical: FOV in degrees

        Returns:
            (yaw_error_deg, pitch_error_deg)
        """
        fov_h = fov_horizontal or self.camera_params.fov_horizontal
        fov_v = fov_vertical or self.camera_params.fov_vertical

        cx = frame_width / 2.0
        cy = frame_height / 2.0

        # Angle per pixel
        deg_per_px_h = fov_h / frame_width
        deg_per_px_v = fov_v / frame_height

        yaw_error = (px - cx) * deg_per_px_h
        pitch_error = (py - cy) * deg_per_px_v

        return (yaw_error, pitch_error)

    def compute_normalized_error(
        self, px: float, py: float,
        frame_width: int, frame_height: int,
    ) -> Tuple[float, float]:
        """
        Compute normalized error [-1, 1] from pixel coordinates.

        Center of frame = (0, 0), edges = (±1, ±1).

        Args:
            px, py: Pixel coordinates
            frame_width, frame_height: Frame dimensions

        Returns:
            (error_x, error_y) in [-1, 1]
        """
        if frame_width <= 0 or frame_height <= 0:
            return (0.0, 0.0)

        error_x = (px - frame_width / 2.0) / (frame_width / 2.0)
        error_y = (py - frame_height / 2.0) / (frame_height / 2.0)

        return (
            max(-1.0, min(1.0, error_x)),
            max(-1.0, min(1.0, error_y)),
        )

    def compute_size_error(
        self, bbox_area: float, frame_area: float,
        target_size_ratio: float = 0.15,
    ) -> float:
        """
        Compute normalized size error.

        Args:
            bbox_area: Area of bounding box
            frame_area: Area of frame
            target_size_ratio: Desired ratio of bbox to frame

        Returns:
            Normalized size error [-1, 1]
            Negative = target too small (too far)
            Positive = target too large (too close)
        """
        if frame_area <= 0:
            return 0.0

        actual_ratio = bbox_area / frame_area
        error = (actual_ratio - target_size_ratio) / target_size_ratio

        return max(-1.0, min(1.0, error))
