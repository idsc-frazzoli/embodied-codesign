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


@dataclass
class CameraSpecification:
    f: Decimal  # focal length in [mm]
    width: Decimal  # sensor chip width [mm]
    height: Decimal  # sensor chip height [mm]
    n_pixel_width: int  # width of image in pixels
    n_pixel_height: int  # height of image in pixels

    def __init__(self, f: Decimal, width: Decimal, height: Decimal, n_pixel_width: int, n_pixel_height: int):
        self.f = f
        self.width = width
        self.height = height
        self.n_pixel_height = n_pixel_height
        self.n_pixel_width = n_pixel_width

    def get_vertical_fov(self, list_of_ds: List[Decimal]) -> List[Decimal]:
        vfov = self.height / self.f * np.array(list_of_ds)

        return list(vfov)

    def get_vertical_resolution(self, fov: List[Decimal]) -> List[Decimal]:
        resolution_v = [Decimal(min(float(self.n_pixel_height), float(self.n_pixel_height/max(0.0001, float(fov[i])))))
                        for i in range(len(fov))]

        return resolution_v


def func_recall_precision(x, a, b, c):
    return a * x ** b + c


def get_recall_precision(name_algo: str, n: int, ds: Decimal) -> Tuple[List[Decimal], List[Decimal]]:
    if name_algo == "faster_rcnn96":
        x_data = [0, 20, 50, 100, 150, 200, 250, 300, 350, 1032]
        y_data_recall = [0, 0.875, 0.925, 0.96, 0.96, 0.96, 0.96, 0.97, 0.975, 0.999]
        popt_recall, pcov_recall = curve_fit(func_recall_precision, x_data, y_data_recall)
        recall = [Decimal(max(0, min(1, func_recall_precision(float(i), *popt_recall)))) for i in range(n)]
        y_data_precision = [0, 0.7, 0.8, 0.92, 0.94, 0.955, 0.96, 0.97, 0.975, 0.999]
        popt_precision, pcov_precision = curve_fit(func_recall_precision, x_data, y_data_precision)
        precision = [Decimal(max(0, min(1, func_recall_precision(float(i), *popt_precision)))) for i in range(n)]
    else:
        raise ValueError("No or wrong algorithm name.")

    return recall, precision
