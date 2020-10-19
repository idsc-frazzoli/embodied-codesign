import json
import math
import sys
import argparse
from decimal import Decimal
from typing import List, Tuple
import numpy as np

import yaml
from scipy.interpolate import interp1d
from scipy.optimize import curve_fit


def get_vertical_resolution(list_of_ds: List[Decimal], cam: dict, h_ped: Decimal) -> List[Decimal]:
    height = Decimal(cam["chip_size_m"][1])
    f = Decimal(cam["focal_length_m"])
    n_pixel_height = cam["resolution_pxl"][1]

    vfov = list(height / f * np.array(list_of_ds))

    resolution_v = [
        Decimal(min(float(n_pixel_height), float(n_pixel_height * float(h_ped) / max(0.0001, float(vfov[i])))))
        for i in range(len(vfov))]

    return resolution_v


def func_recall_precision(x, a, b, c):
    return a * x ** b + c


def get_recall_precision(alg, n) -> Tuple[List[Decimal], List[Decimal]]:
    x_data = alg["x_data_pxl_obj"]
    y_data_recall = alg["y_data_recall"]
    y_data_precision = alg["y_data_precision"]
    popt_recall, pcov_recall = curve_fit(func_recall_precision, x_data, y_data_recall)
    recall = [Decimal(max(0, min(1, func_recall_precision(float(i), *popt_recall)))) for i in range(n)]
    popt_precision, pcov_precision = curve_fit(func_recall_precision, x_data, y_data_precision)
    precision = [Decimal(max(0, min(1, func_recall_precision(float(i), *popt_precision)))) for i in range(n)]

    return recall, precision


def get_accuracy(cam, list_of_ds: List[Decimal]):
    width = Decimal(cam["chip_size_m"][0])
    f = Decimal(cam["focal_length_m"])
    n_pixel_width = cam["resolution_pxl"][0]

    focal_pixel = (f / width) * n_pixel_width

    accuracy = [float(max(0, ds ** 2 / (focal_pixel * Decimal('0.1')) * Decimal('0.4'))) for ds in list_of_ds]

    return accuracy


def get_number_points_vertical(lenght_object: Decimal, d: Decimal, resolution_angle: Decimal,
                               max_points: int) -> Decimal:
    if float(d) == 0.0:
        n = max_points
    else:
        n = max(0, min(lenght_object / (d * Decimal(math.tan(math.radians(resolution_angle)))), max_points))

    return n


def get_number_points_horizontal(lenght_object: Decimal, d: Decimal, resolution_angle: Decimal) -> Decimal:
    max_points = 20000
    if float(d) == 0.0:
        n = max_points
    else:
        n = max(0, min(lenght_object / (d * Decimal(math.tan(math.radians(resolution_angle)))), max_points))

    return n


def func_recall_precision_lidar(x, a, c):
    return -a * x ** 0.5 + c


def get_recall_precision_lidar(alg, list_of_ds, total_hdl64e_log):
    y_data_recall = alg["y_data_recall"]
    y_data_precision = alg["y_data_precision"]
    popt_recall, pcov_recall = curve_fit(func_recall_precision_lidar, total_hdl64e_log, y_data_recall)
    recall = [max(0, min(1, func_recall_precision_lidar(float(i), *popt_recall))) for i in list_of_ds]
    popt_precision, pcov_precision = curve_fit(func_recall_precision_lidar, total_hdl64e_log, y_data_precision)
    precision = [max(0, min(1, func_recall_precision_lidar(float(i), *popt_precision))) for i in list_of_ds]

    return recall, precision


def get_cam_curves(list_of_ds: List[Decimal], sens: dict, al_data: dict, h_ped: Decimal) -> Tuple[
    List[float], List[float], List[float]]:
    resolution_v = get_vertical_resolution(list_of_ds, sens, h_ped)
    height_px = int(sens["resolution_pxl"][1])
    accuracy = get_accuracy(sens, list_of_ds)
    recall, precision = get_recall_precision(al_data, height_px + 1)
    f_recall = interp1d(range(height_px + 1), recall)
    f_precision = interp1d(range(height_px + 1), precision)
    recall_d = [f_recall(float(resolution_v[i])) for i in range(len(resolution_v))]
    precision_d = [f_precision(float(resolution_v[i])) for i in range(len(resolution_v))]

    return recall_d, precision_d, accuracy


def get_lidar_curves(list_of_ds: List[Decimal], sens: dict, h_ped: Decimal, w_ped: Decimal, al_data: dict,
                     total_hdl64e_log, ds: Decimal) -> Tuple[List[float], List[float], List[float]]:
    max_tog_log = Decimal('85.0')
    acc = Decimal(sens["accuracy_m"][1])
    n_vertical = [get_number_points_vertical(h_ped, d, Decimal(sens["resolution_degree"][1]),
                                             int(sens["channels"])) for d in list_of_ds]
    n_horizontal = [get_number_points_horizontal(w_ped, d,
                                                 Decimal(sens["resolution_degree"][0])) for d in list_of_ds]
    total = np.array(n_horizontal) * np.array(n_vertical)
    total_log = [min(max(math.log(t, 1.2), 0), float(max_tog_log-ds)) for t in total]
    n85 = int(round(max_tog_log / ds))
    list_of_ds_85 = [float(ds * Decimal(i)) for i in range(n85)]
    recall_hdl64e, precision__hdl64e = get_recall_precision_lidar(al_data, list_of_ds_85, total_hdl64e_log)
    f_recall = interp1d(list_of_ds_85, recall_hdl64e)
    f_precision = interp1d(list_of_ds_85, precision__hdl64e)
    recall_d = [f_recall(float(d)) for d in total_log]
    precision_d = [f_precision(float(d)) for d in total_log]
    accuracy = [float(acc) for _ in list_of_ds]

    return recall_d, precision_d, accuracy


def get_hdl64e_reference_values(hdl64e: dict, alg: dict, h_ped: Decimal, w_ped: Decimal) -> List[float]:
    hdl64e_v_resolution = Decimal(hdl64e["resolution_degree"][1])
    hdl64e_h_resolution = Decimal(hdl64e["resolution_degree"][0])
    hdl64e_channels = int(hdl64e["channels"])
    x_data = alg["x_data_m"]
    n_vertical_hdl64e = [min(64, get_number_points_vertical(h_ped, d, hdl64e_v_resolution, hdl64e_channels))
                         for d in x_data]
    n_horizontal_hdl64e = [get_number_points_horizontal(w_ped, d, hdl64e_h_resolution) for d in x_data]
    total_hdl64e = np.array(n_horizontal_hdl64e) * np.array(n_vertical_hdl64e)
    total_hdl64e_log = [max(0, math.log(t, 1.2)) for t in total_hdl64e]

    return total_hdl64e_log


def generate_sens_curves(ds: Decimal, max_distance: Decimal, h_ped: Decimal, w_ped: Decimal):
    max_detect_range = Decimal('50')
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]
    n50 = int(round(max_detect_range / ds))
    list_of_ds_50 = [ds * Decimal(i) for i in range(n50)]

    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/object_detection.yaml') as file:
        algorithms = yaml.load(file, Loader=yaml.FullLoader)

    hdl64e = sensors["hdl64"]
    day_kde_based = algorithms["day_kde_based"]
    total_hdl64e_log = get_hdl64e_reference_values(hdl64e, day_kde_based, h_ped, w_ped)
    sens_pef = {}

    for sens_key, sens in sensors.items():
        env_keys = sens["algorithm"]
        for env_key in env_keys:
            algs = env_keys[env_key]
            for al_name in algs:
                if sens['type'] == "camera":
                    al_data = algorithms[al_name]
                    recall_d, precision_d, accuracy = get_cam_curves(list_of_ds_50, sens, al_data, h_ped)
                else:
                    al_data = algorithms[al_name]
                    recall_d, precision_d, accuracy = get_lidar_curves(list_of_ds_50, sens, h_ped, w_ped, al_data,
                                                                       total_hdl64e_log, ds)
                fn = list(1 - np.array(recall_d))
                fp = list(1 - np.array(precision_d))
                if len(fn) < len(list_of_ds):
                    diff = len(list_of_ds) - len(fn)
                    ones = [1.0 for _ in range(diff)]
                    ten_m = [10.0 for _ in range(diff)]
                    fn = fn + ones
                    fp = fp + ones
                    accuracy = accuracy + ten_m
                fn_fp_acc = {"fn": fn, "fp": fp, "accuracy": accuracy, "ds": str(ds), "max_distance": str(max_distance)}
                sens_pef[sens_key + "_" + al_name] = fn_fp_acc

    with open('data/input/camera_curves.json', 'w') as file:
        json.dump(sens_pef, file, indent=2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating sensing curves.')
    parser.add_argument('--ds', type=str, default='0.1', help='The discretization of space in meters.')
    parser.add_argument('--max_distance', type=str, default='50.0', help='The maximum range of the sensor curves')
    parser.add_argument('--hped', type=str, default='1.7', help='Average height of a pedestrian in meters.')
    parser.add_argument('--wped', type=str, default='0.5', help='Average width of a pedestrian in meters.')
    args = parser.parse_args()
    ds = Decimal(args.ds)
    max_distance = Decimal(args.max_distance)
    h_ped = Decimal(args.hped)
    w_ped = Decimal(args.wped)
    generate_sens_curves(ds=ds, max_distance=max_distance, h_ped=h_ped, w_ped=w_ped)
