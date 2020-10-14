import math
from decimal import Decimal

import scipy
import yaml

from sensing.sensing_performance import ConfLevel
from simulator.simulator import SimParameters

if __name__ == '__main__':
    with open('data/input/camera_curves.yaml') as file:
        camera_curves = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    sp = SimParameters(nsims=1, road_length=Decimal('500.0'), dt=Decimal(str(0.01)),
                       seed=0, do_animation=True)

    camera = sensors["camera"]

    sens = camera["Ace13gm"]
    sens_perf = sens["sens_perf"][0]
    sens_curves = camera_curves[sens_perf]
    lsd = sens_curves["accuracy"]
    lsd = [Decimal(p) for p in lsd]

    ds = Decimal(sens_curves["ds"])
    max_distance = Decimal(sens_curves["max_distance"])
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]
    tresh_idx = 0
    for i in range(n):
        idx = int(lsd[i] / ds)
        if idx >= 1:
            tresh_idx = i
            break
    list_cl = []
    print(tresh_idx)
    for i in range(n):
        cl = 0.95
        sigma = lsd[i]
        df = n - 1
        mean = float(list_of_ds[i])
        standard_error = float(sigma) / math.sqrt(n)
        cL_level = scipy.stats.t.interval(cl, df, mean, standard_error)
        if i < tresh_idx:
            if math.isnan(cL_level[0]) or math.isnan(cL_level[1]):
                clist_cell = [Decimal(str(0.0)), Decimal(str(0.0))]
            else:
                clist_cell = [Decimal(str(cL_level[0])), Decimal(str(cL_level[1]))]

            print(clist_cell)
            confidence_level = ConfLevel(clist_cell)
        else:
            ucl_cell = int(min(n, (mean + cL_level[1]) / float(ds)))
            lcl_cell = int(min(n, max(0.0, (mean + cL_level[0]) / float(ds))))
            if ucl_cell == lcl_cell:
                clist_cell = [int(mean/float(ds))]
            else:
                clist_cell = [Decimal(str(i)) for i in range(lcl_cell, ucl_cell)]
            print(clist_cell)
            confidence_level = ConfLevel(clist_cell)

        list_cl.append(confidence_level)
