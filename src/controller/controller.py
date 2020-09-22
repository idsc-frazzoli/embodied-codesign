from abc import ABC, abstractmethod

from vehicle.state_estimation import Belief, Action
from vehicle.vehicle import VehicleState


class Controller(ABC):
    @abstractmethod
    def get_action(self, vstate: VehicleState, belief: Belief) -> Action:
        ...

