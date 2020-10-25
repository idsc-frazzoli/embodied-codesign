from dataclasses import dataclass
from decimal import Decimal
from typing import Optional


@dataclass
class Statistics:
    mean: Decimal
    var: Decimal
    u95: Decimal
    l95: Decimal


@dataclass
class PerformanceMetrics:
    danger: Statistics
    discomfort: Statistics
    average_velocity: Statistics
    stopped_too_slow: bool


@dataclass
class CollisionStats:
    momentum: Decimal


@dataclass
class OneSimPerformanceMetrics:
    collided: Optional[CollisionStats]
    average_velocity: Decimal
    control_effort: Decimal
    stopped_too_slow: bool
