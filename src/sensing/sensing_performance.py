from dataclasses import dataclass
from decimal import Decimal
from typing import List
import numpy as np



@dataclass
class SensingParameters:
    ds: Decimal  # meters
    max_distance: Decimal  # meters
    n: int  # max_distance / ds
    list_of_ds: List[Decimal]
    frequency: Decimal
    latency: Decimal

    def __post_init__(self):
        assert self.frequency > 0, self.frequency
        assert self.n >= 0
        assert len(self.list_of_ds) == self.n, (len(self.list_of_ds), self.n)

@dataclass
class SensingPerformance:
    fp: List[Decimal]  # of length n
    fn: List[Decimal]  # of length n
    lsd: List[Decimal]  # in meters
    sp: SensingParameters

    def __init__(self, sp: SensingParameters) -> None:
        self.n = sp.n
        self.ds = sp.ds

    def false_negative_at(self, d: Decimal) -> Decimal:
        i = int(np.ceil(d / self.ds))

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.fn[i]

    def false_positive_at(self, d: Decimal) -> Decimal:
        i = int(np.ceil(d / self.ds))

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.fp[i]

    def lsd_at(self, d: Decimal) -> Decimal:
        i = int(np.ceil(d / self.ds))

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.lsd[i]

