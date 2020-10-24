import math

from dataclasses import dataclass
from decimal import Decimal
from typing import List
import numpy as np

@dataclass
class ProbAccuracy:
    a: Decimal
    b: Decimal
    p: Decimal

@dataclass
class SensingParameters:
    ds: Decimal  # meters
    max_distance: Decimal  # meters
    n: int  # max_distance / ds
    list_of_ds: List[Decimal]
    sens_sampl_time_s: Decimal
    latency_s: Decimal

    def __post_init__(self):
        assert self.sens_sampl_time_s > 0, self.sens_sampl_time_s
        assert self.n >= 0
        assert len(self.list_of_ds) == self.n, (len(self.list_of_ds), self.n)

@dataclass
class SensingPerformance:
    fp: List[Decimal]  # of length n
    fn: List[Decimal]  # of length n
    lsd: List[Decimal]  # in meters
    sp: SensingParameters
    prob_accuracy: List[ProbAccuracy]

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

    def prob_acc_at(self, d: Decimal) -> ProbAccuracy:
        i = int(d / self.ds)

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.prob_accuracy[i]


def calc_unit_dist_a_b_prob(d: Decimal, ds: Decimal,  std: Decimal) -> ProbAccuracy:
    mean = d + Decimal('0.5') * ds
    a = mean - Decimal(str(math.sqrt(3))) * std
    b = mean + Decimal(str(math.sqrt(3))) * std
    if std == Decimal('0.0'):
        a = mean - Decimal('0.5')*ds
        b = mean + Decimal('0.5')*ds
    a = Decimal('0.0') if a < Decimal('0.0') else a
    prob = 1 / (b - a)

    return ProbAccuracy(Decimal(a), Decimal(b), Decimal(prob))

