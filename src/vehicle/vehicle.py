from decimal import Decimal
from typing import List

from dataclasses import dataclass


@dataclass
class VehicleStats:
    a_min: Decimal  # m / s²
    a_max: Decimal  # m / s²
    v_nominal: Decimal  # m / s
    mass: Decimal  # kg


@dataclass
class Object:
    # distance relative to car
    d: Decimal


@dataclass
class VehicleState:
    x: Decimal
    v: Decimal
    x_prev: Decimal
    v_prev: Decimal


@dataclass
class State:
    vstate: VehicleState
    objects: List[Object]


@dataclass
class DelayedStates:
    states: List[State]
    latency: Decimal
    l: int

    def update(self, state: State):
        self.states =  self.states[:-1] + [state]


@dataclass
class Detection:
    d_mean: Decimal
    d_std: Decimal
