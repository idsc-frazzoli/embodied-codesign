import numpy as np
from dataclasses import dataclass
from decimal import Decimal
from typing import List, Tuple
from scipy.optimize import curve_fit


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

