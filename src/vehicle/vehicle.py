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
    def __post_init__(self):
        assert self.l >= 1
        assert len(self.states) == self.l
    def update(self, state: State):
        vstate = VehicleState(x=state.vstate.x, v=state.vstate.v, x_prev=state.vstate.x_prev, v_prev=state.vstate.x_prev)
        object_list = [Object(ob.d) for ob in state.objects]
        new_state = State(vstate=vstate, objects=object_list)
        self.states = self.states[1:] + [new_state]


@dataclass
class Detection:
    d_mean: Decimal
