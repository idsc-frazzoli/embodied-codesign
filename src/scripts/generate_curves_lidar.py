import json
import math
from decimal import Decimal
from typing import Tuple, List

import numpy as np
import yaml
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.optimize import curve_fit

def get_number_points_vertical(lenght_object: Decimal, d: Decimal, resolution_angle: Decimal, max_points: int) -> Decimal:
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

def func_recall_precision(x, a, c):
    return -a * x ** 0.5 + c


def get_recall_precision(alg, list_of_ds, total_hdl64e_log):
    y_data_recall = alg["y_data_recall"]
    y_data_precision = alg["y_data_precision"]
    popt_recall, pcov_recall = curve_fit(func_recall_precision, total_hdl64e_log, y_data_recall)
    recall = [max(0, min(1, func_recall_precision(float(i), *popt_recall))) for i in list_of_ds]
    popt_precision, pcov_precision = curve_fit(func_recall_precision, total_hdl64e_log, y_data_precision)
    precision = [max(0, min(1, func_recall_precision(float(i), *popt_precision))) for i in list_of_ds]

    return recall, precision

if __name__ == '__main__':
    ds = Decimal('0.05')
    max_distance = Decimal('50.0')
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]
    h_ped = Decimal(str(1.7))
    w_ped = Decimal(str(0.5))

    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/object_detection.yaml') as file:
        algorithms = yaml.load(file, Loader=yaml.FullLoader)

    lidar = sensors["lidar"]
    obj_detect = algorithms["lidar"]
    sens_pef = {}
    hdl64e = lidar["hdl64"]
    hdl64e_v_resolution = Decimal(hdl64e["resolution"][1])
    hdl64e_h_resolution = Decimal(hdl64e["resolution"][0])
    hdl64e_channels = int(hdl64e["channels"])
    x_data = obj_detect["kde_based"]["x_data"]
    n_vertical_hdl64e = [min(64, get_number_points_vertical(h_ped, d, hdl64e_v_resolution, hdl64e_channels))
                         for d in x_data]
    n_horizontal_hdl64e = [get_number_points_horizontal(w_ped, d, hdl64e_h_resolution) for d in x_data]
    total_hdl64e = np.array(n_horizontal_hdl64e) * np.array(n_vertical_hdl64e)
    total_hdl64e_log = [max(0, math.log(t, 1.2)) for t in total_hdl64e]

    for lidar_key in lidar:
        lid = lidar[lidar_key]
        accuracy = Decimal(lid["accuracy"][1])*Decimal('0.01')
        n_vertical = [get_number_points_vertical(h_ped, d, Decimal(lid["resolution"][1]),
                                                 int(lid["channels"])) for d in list_of_ds]
        n_horizontal = [get_number_points_horizontal(w_ped, d,
                                                     Decimal(lid["resolution"][0])) for d in list_of_ds]
        total = np.array(n_horizontal)*np.array(n_vertical)
        total_log = [max(math.log(t, 1.2),0) for t in total]
        for alg_key, alg in obj_detect.items():
            n85 = int(round(Decimal('85') / ds))
            list_of_ds_85 = [float(ds * Decimal(i)) for i in range(n85)]
            recall_hdl64e, precision__hdl64e = get_recall_precision(alg, list_of_ds_85, total_hdl64e_log)
            f_recall = interp1d(list_of_ds_85, recall_hdl64e)
            f_precision = interp1d(list_of_ds_85, precision__hdl64e)
            recall = [f_recall(float(d)) for d in total_log]
            precision = [f_precision(float(d)) for d in total_log]
            accuracy_list = [str(accuracy) for ds in list_of_ds]
            fn = list(1 - np.array(recall))
            fp = list(1 - np.array(precision))
            fn = [str(p) for p in fn]
            fp = [str(p) for p in fp]
            fn_fp = {"fn": fn, "fp": fp, "accuracy": accuracy_list, "ds": str(ds), "max_distance": str(max_distance)}
            sens_pef[lidar_key + "_day_" + alg_key] = fn_fp

    with open('data/input/lidar_curves.yaml', 'w') as file:
        documents = yaml.dump(sens_pef, file, default_flow_style=False)
    with open('data/input/lidar_curves.json', 'w') as file:
        json.dump(sens_pef, file, indent=2)

    with open('data/input/lidar_curves.yaml') as file:
        curves = yaml.load(file, Loader=yaml.FullLoader)

    # for c_key, c in curves.items():
    #     data = c["fn"]
    #     data = [Decimal(s) for s in data]
    #     plt.plot(list_of_ds, data, label=c_key)
    #     plt.ylabel('fn')
    #     plt.xlabel('d in [m]')
    #     plt.legend(loc="upper left")
    #
    # plt.show()
    # plt.close()
    # for c_key, c in curves.items():
    #     data = c["fp"]
    #     data = [Decimal(s) for s in data]
    #     plt.plot(list_of_ds, data, label=c_key)
    #     plt.ylabel('fp')
    #     plt.xlabel('d in [m]')
    #     plt.legend(loc="upper left")


    # plt.show()

    for lidar_key in lidar:
        list_id = []
        for c_key in curves:
            if lidar_key in c_key:
                list_id.append(c_key)

        lidar[lidar_key]["sens_perf"] = list_id

    sensors["lidar"] = lidar

    with open('data/input/sensors.yaml', 'w') as file:
        yaml.dump(sensors, file, default_flow_style=False)
