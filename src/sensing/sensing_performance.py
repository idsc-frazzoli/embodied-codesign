import math
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
    cl_list: ConfLevelList
    prob_accuracy: List[ProbAccuracy]

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

    def prob_acc_at(self, d: Decimal) -> ProbAccuracy:
        i = int(d / self.ds)

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.prob_accuracy[i]


def calc_unit_dist_a_b_prob(d: Decimal, ds: Decimal,  std: Decimal) -> ProbAccuracy:
    mean = d + Decimal('0.5')*ds
    a = mean - Decimal(str(math.sqrt(3))) * std
    if std == Decimal('0.0'):
        a = a - Decimal('0.5')*ds
    a = Decimal('0.0') if a < Decimal('0.0') else a
    b = mean + Decimal(str(math.sqrt(3))) * std
    prob = 1 / (b - a)

    return ProbAccuracy(Decimal(a), Decimal(b), Decimal(prob))
