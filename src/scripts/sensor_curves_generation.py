import json
from decimal import Decimal
from typing import List, Tuple

import numpy as np
import yaml
import matplotlib.pyplot as plt

from scipy.interpolate import interp1d
from scipy.optimize import curve_fit


def get_vertical_resolution(list_of_ds: List[Decimal], cam) -> List[Decimal]:
    height = Decimal(cam["size"][1])
    f = Decimal(cam["lens"])
    n_pixel_height = cam["resolution"][1]

    vfov = list(height / f * np.array(list_of_ds))

    resolution_v = [
        Decimal(min(float(n_pixel_height), float(n_pixel_height * 1.70 / max(0.0001, float(vfov[i])))))
        for i in range(len(vfov))]

    return resolution_v


def func_recall_precision(x, a, b, c):
    return a * x ** b + c


def get_recall_precision(alg, n) -> Tuple[List[Decimal], List[Decimal]]:
    x_data = alg["x_data"]
    y_data_recall = alg["y_data_recall"]
    y_data_precision = alg["y_data_precision"]
    popt_recall, pcov_recall = curve_fit(func_recall_precision, x_data, y_data_recall)
    recall = [Decimal(max(0, min(1, func_recall_precision(float(i), *popt_recall)))) for i in range(n)]
    popt_precision, pcov_precision = curve_fit(func_recall_precision, x_data, y_data_precision)
    precision = [Decimal(max(0, min(1, func_recall_precision(float(i), *popt_precision)))) for i in range(n)]

    return recall, precision


def get_accuracy(cam, list_of_ds: List[Decimal]):
    width = Decimal(cam["size"][0])
    f = Decimal(cam["lens"])
    n_pixel_width = cam["resolution"][0]

    focal_pixel = (f/width) * n_pixel_width

    accuracy = [Decimal(max(0, ds**2/(focal_pixel*Decimal('0.1'))*Decimal('0.4'))) for ds in list_of_ds]

    return accuracy

if __name__ == '__main__':
    ds = Decimal('0.05')
    max_distance = Decimal('50.0')
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]

    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/object_detection.yaml') as file:
        algorithms = yaml.load(file, Loader=yaml.FullLoader)

    cameras = sensors["camera"]
    obj_detect = algorithms["camera"]["mono"]
    sens_pef = {}

    for cam_key in cameras:
        cam = cameras[cam_key]
        resolution_v = get_vertical_resolution(list_of_ds, cam)
        height_px = int(cam["resolution"][1])
        accuracy = get_accuracy(cam, list_of_ds)
        for env_key, env in obj_detect.items():
            for alg_key in env:
                alg = env[alg_key]

                recall, precision = get_recall_precision(alg, height_px + 1)
                f_recall = interp1d(range(height_px + 1), recall)
                f_precision = interp1d(range(height_px + 1), precision)
                recall_d = [f_recall(float(resolution_v[i])) for i in range(len(resolution_v))]
                precision_d = [f_precision(float(resolution_v[i])) for i in range(len(resolution_v))]
                fn = list(1 - np.array(recall_d))
                fp = list(1 - np.array(precision_d))
                fn = [str(p) for p in fn]
                fp = [str(p) for p in fp]
                accuracy = [str(a) for a in accuracy]
                fn_fp = {"fn": fn, "fp": fp, "accuracy": accuracy, "ds": str(ds), "max_distance": str(max_distance)}
                sens_pef[cam_key + "_" + env_key + "_" + alg_key] = fn_fp

    with open('data/input/camera_curves.yaml', 'w') as file:
        documents = yaml.dump(sens_pef, file, default_flow_style=False)
    with open('data/input/camera_curves.json', 'w') as file:
        json.dump(sens_pef, file, indent=2)

    with open('data/input/camera_curves.yaml') as file:
        curves = yaml.load(file, Loader=yaml.FullLoader)

    # for c_key, c in curves.items():
        # data = c["accuracy"]
        # data = [Decimal(s) for s in data]
        # plt.plot(list_of_ds, data, label=c_key)
        # plt.ylabel('fn')
        # plt.xlabel('d in [m]')
        # plt.legend(loc="upper left")

    # plt.show()

    # plt.savefig('data/output/fn.png')
    # plt.close()
    #
    # for c_key, c in curves.items():
    #     if "day" in c_key:
    #         fp = c["fp"]
    #         fp = [Decimal(s) for s in fp]
    #         plt.plot(list_of_ds, fp, label=c_key)
    #         plt.ylabel('FPR')
    #         plt.xlabel('d in [m]')
    #         plt.legend(loc="upper left")
    #
    # plt.show()
    # plt.savefig('data/output/fp.png')
    # plt.close()

    for cam_key in cameras:
        list_id = []
        for c_key in curves:
            if cam_key in c_key:
                list_id.append(c_key)

        cameras[cam_key]["sens_perf"] = list_id

    sensors["camera"] = cameras

    with open('data/input/sensors.yaml', 'w') as file:
        yaml.dump(sensors, file, default_flow_style=False)

