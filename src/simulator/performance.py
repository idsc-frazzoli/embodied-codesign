from decimal import Decimal
from typing import Optional

from dataclasses import dataclass

from vehicle.vehicle import State


@dataclass
class PerformanceMetrics:
    danger: Decimal
    discomfort: Decimal
    average_velocity: Decimal


@dataclass
class CollisionStats:
    momentum: Decimal


@dataclass
class OneSimPerformanceMetrics:
    collided: Optional[CollisionStats]
    average_velocity: Decimal
    control_effort: Decimal
