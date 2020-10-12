from dataclasses import dataclass
from decimal import Decimal
from typing import List


@dataclass
class ConfLevel:
    conf_level: List[Decimal]

@dataclass
class ConfLevelList:
    list: List[ConfLevel]
    treshold_idx: int


@dataclass
class SensingParameters:
    ds: Decimal  # meters
    max_distance: Decimal  # meters
    n: int  # max_distance / ds
    list_of_ds: List[Decimal]
    frequency: Decimal
    latency: Decimal


@dataclass
class SensingPerformance:
    fp: List[Decimal]  # of length n
    fn: List[Decimal]  # of length n
    lsd: List[Decimal]  # in meters
    sp: SensingParameters
    cl_list: ConfLevelList

    def __init__(self, sp: SensingParameters) -> None:
        self.n = sp.n
        self.ds = sp.ds

    def false_negative_at(self, d: Decimal) -> Decimal:
        i = int(d / self.ds)

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.fn[i]

    def false_positive_at(self, d: Decimal) -> Decimal:
        i = int(d / self.ds)

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.fp[i]

    def lsd_at(self, d: Decimal) -> Decimal:
        i = int(d / self.ds)

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.lsd[i]

