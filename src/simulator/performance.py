from decimal import Decimal
from typing import Optional

from dataclasses import dataclass


@dataclass
class PerformanceMetrics:
    p_collision: Decimal
    discomfort: Decimal
    average_velocity: Decimal
    average_collision_momentum: Decimal

@dataclass
class CollisionStats:
    momentum: Decimal


@dataclass
class OneSimPerformanceMetrics:
    collided: Optional[CollisionStats]
    average_velocity: Decimal
    control_effort: Decimal

