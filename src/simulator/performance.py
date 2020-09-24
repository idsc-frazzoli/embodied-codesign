from decimal import Decimal
from typing import Optional

from dataclasses import dataclass

from vehicle.vehicle import State


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
class StoppedStats:
    wt: Decimal
    stop: bool
    d_stop: Decimal

    def stopped(self, s: State, dt: Decimal) -> None:
        if round(s.vstate.v, 2) == 0.0 and s.objects[0].d <= self.d_stop:
            self.wt += dt
            self.stop = True
        else:
            self.stop = False


@dataclass
class OneSimPerformanceMetrics:
    collided: Optional[CollisionStats]
    average_velocity: Decimal
    control_effort: Decimal
