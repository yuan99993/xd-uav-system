"""
TargetLossHandler — 目标丢失处理器。

Ported from PixEagle MCVelocityChaseFollower._handle_target_loss()

Detects target loss, manages timeout-based recovery,
and provides status for downstream control decisions.
"""

import time
import math
from typing import Tuple, Optional
import logging

logger = logging.getLogger(__name__)


class TargetLossHandler:
    """
    Handles target loss detection and recovery.

    Features:
    - Coordinate-based loss detection (threshold on normalized coords)
    - Timeout-based sustained loss tracking
    - Last valid coordinate storage for recovery
    - Configurable coordinate threshold
    """

    def __init__(
        self,
        coord_threshold: float = 1.5,     # Normalized coord threshold (default >1.0 = edge)
        loss_timeout: float = 3.0,         # Seconds before declaring sustained loss
    ):
        """
        Initialize target loss handler.

        Args:
            coord_threshold: Abs coordinate value threshold for loss detection
                             (normalized [-1,1], but can exceed 1 for prediction)
            loss_timeout: Time in seconds before sustained loss is confirmed
        """
        self.coord_threshold = coord_threshold
        self.loss_timeout = loss_timeout

        # State
        self.target_lost = False
        self.loss_start_time: Optional[float] = None
        self.last_valid_coords: Tuple[float, float] = (0.0, 0.0)
        self.loss_duration = 0.0

    def update(
        self, target_coords: Tuple[float, float],
        is_valid: bool = True,
        timestamp: Optional[float] = None,
    ) -> bool:
        """
        Update target loss state.

        Args:
            target_coords: (x, y) normalized target coordinates
            is_valid: Whether the input data is considered valid
            timestamp: Current timestamp

        Returns:
            True if target is considered valid (not lost)
        """
        if timestamp is None:
            timestamp = time.time()

        x, y = target_coords

        # Check validity
        coord_valid = (
            is_valid
            and not (math.isnan(x) or math.isnan(y))
            and not (abs(x) > self.coord_threshold or abs(y) > self.coord_threshold)
        )

        if coord_valid:
            # Target is valid
            if self.target_lost:
                logger.info("[TargetLossHandler] Target recovered after %.1fs loss",
                           self.loss_duration)
            self.target_lost = False
            self.loss_start_time = None
            self.loss_duration = 0.0
            self.last_valid_coords = (x, y)
            return True
        else:
            # Target appears lost
            if not self.target_lost:
                self.target_lost = True
                self.loss_start_time = timestamp
                logger.warning("[TargetLossHandler] Target lost at coords: (%.2f, %.2f)", x, y)
            else:
                self.loss_duration = timestamp - (self.loss_start_time or timestamp)
            return False

    def is_sustained_loss(self) -> bool:
        """Check if loss has persisted beyond timeout."""
        return self.target_lost and self.loss_duration > self.loss_timeout

    def get_loss_duration(self) -> float:
        """Get current loss duration in seconds."""
        return self.loss_duration

    def get_last_valid_coords(self) -> Tuple[float, float]:
        """Get last known valid coordinates."""
        return self.last_valid_coords

    def reset(self):
        """Reset state."""
        self.target_lost = False
        self.loss_start_time = None
        self.loss_duration = 0.0
        self.last_valid_coords = (0.0, 0.0)
