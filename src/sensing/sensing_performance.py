from dataclasses import dataclass
from decimal import Decimal
from typing import List
<<<<<<< HEAD
from numpy import np
=======
import numpy as np
>>>>>>> 70c8a8ff601a5d5f16175ef9b4bcd646e7da899d


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
<<<<<<< HEAD
        # This is optimistic, shouldn't we put ceil as well? E.g. at 1.9 it thinks it is at 1
=======
>>>>>>> 70c8a8ff601a5d5f16175ef9b4bcd646e7da899d
        i = int(np.ceil(d / self.ds))

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.fn[i]

    def false_positive_at(self, d: Decimal) -> Decimal:
<<<<<<< HEAD
        # This is optimistic, shouldn't we put ceil as well? E.g. at 1.9 it thinks it is at 1
=======
>>>>>>> 70c8a8ff601a5d5f16175ef9b4bcd646e7da899d
        i = int(np.ceil(d / self.ds))

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.fp[i]

    def lsd_at(self, d: Decimal) -> Decimal:
<<<<<<< HEAD
        # This is optimistic, shouldn't we put ceil as well? E.g. at 1.9 it thinks it is at 1
=======
>>>>>>> 70c8a8ff601a5d5f16175ef9b4bcd646e7da899d
        i = int(np.ceil(d / self.ds))

        if i > self.n:
            raise IndexError("Index out of bound.")
        elif i == self.n:
            i = i - 1

        return self.lsd[i]

