from abc import ABC, abstractmethod
from decimal import Decimal

from vehicle.state_estimation import Action, Inference
from vehicle.vehicle import VehicleState


class Controller(ABC):
    @abstractmethod
    def get_action(self, vstate: VehicleState, inference: Inference, ds: Decimal) -> Action:
        ...

