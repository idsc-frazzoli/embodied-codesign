from dataclasses import dataclass
from decimal import Decimal
from typing import List


@dataclass
class SensingParameters:
    ds: Decimal  # meters
    max_distance: Decimal  # meters
    n: int  # max_distance / ds
    list_of_ds: List[Decimal]


@dataclass
class SensingPerformance:
    # false positives as a function of distance
    # array of length n
    fp: List[Decimal]  # of length n
    # false negatives
    fn: List[Decimal]  # of length n
    # localization standard deviation
    lsd: List[Decimal]  # in meters
    sp: SensingParameters

    def __init__(self, sp: SensingParameters) -> None:
        self.n = sp.n
        self.ds = sp.ds

    def false_negative_at(self, d: Decimal) -> Decimal:
        try:
            i = int(d / self.ds)
            return self.fn[i]
        except IndexError:
            print("Index out of bound.")

    def false_positive_at(self, d: Decimal) -> Decimal:
        try:
            i = int(d / self.ds)
            return self.fp[i]
        except IndexError:
            print("Index out of bound.")

    def lsd_at(self, d: Decimal) -> Decimal:
        try:
            i = int(d / self.ds)
            return self.lsd[i]
        except IndexError:
            print("Index out of bound.")

