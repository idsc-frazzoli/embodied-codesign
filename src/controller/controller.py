from abc import ABC, abstractmethod

from vehicle.state_estimation import Action, Inference
from vehicle.vehicle import VehicleState


class Controller(ABC):
    @abstractmethod
    def get_action(self, vstate: VehicleState, inference: Inference) -> Action:
        ...

