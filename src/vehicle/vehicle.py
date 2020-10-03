import random
from decimal import Decimal
from typing import List

import scipy.stats
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


@dataclass
class State:
    # vehicle state

    vstate: VehicleState
    # list of obstacles with distance
    objects: List[Object]


@dataclass
class DelayedStates:
    states: List[State]
    latency: Decimal
    l: int

    def update(self, state: State):
        self.states[0:(self.l - 1)] = self.states[1:]
        self.states[-1] = state


@dataclass
class Detection:
    d_mean: Decimal
    d_std: Decimal

    def at_distance(self, x: Decimal, fn: Decimal, fp: Decimal) -> Decimal:
        """ Returns the likeilhood of something at x given this detection"""
        # return gaussian distribution density at  (x-d_mean)/d_std
        # std = fn + fp
        std = self.d_std
        p = Decimal(scipy.stats.norm(float(x), float(std)).pdf(float(self.d_mean)))

        return p
