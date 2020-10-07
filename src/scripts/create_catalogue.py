from decimal import Decimal
import time

import yaml

from simulator.simulator import SimParameters, simulate

if __name__ == '__main__':

    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/dyn_perf.yaml') as file:
        vehicles = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/environment.yaml') as file:
        environment = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/speeds.yaml') as file:
        cruise_speeds = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/control_param.yaml') as file:
        control_param = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/curves.yaml') as file:
        curves = yaml.load(file, Loader=yaml.FullLoader)

    camera = sensors["camera"]
    speed = cruise_speeds["speeds"]

    e = 0.1 / 100
    p = 0.01 / 100
    z = 1.96

    nsims = int(z ** 2 * (1 - p) * p / e ** 2)

    sp = SimParameters(nsims=1, road_length=Decimal('500.0'), dt=Decimal(str(0.01)), seed=0,
                       wt=Decimal('1.0'), do_animation=False)

    id = 1
    timestr = time.strftime("%Y%m%d-%H%M%S")
    with open('data/output/ad-performance-' + timestr + '.yaml', 'w') as file:
        for veh_key, dyn_perf in vehicles.items():
            for cam_key, sens in camera.items():
                sens_perf_key = sens["sens_perf"]
                sens_curves = curves[sens_perf_key]
                for s in speed:
                    for env_key, env in environment.items():
                        for cont_key, cont in control_param.items():
                            performance = simulate(sp, dyn_perf, sens, sens_curves, s, env, cont)
                            ad_perf = {}
                            ad_perf["ad" + str(id)] = {"danger": str(performance.danger), "discomfort": str(performance.discomfort),
                                 "speed": s, "sensor": cam_key, "dyn_perf": veh_key, "environment": env_key,
                                 "controller": cont_key}
                            documents = yaml.dump(ad_perf, file, default_flow_style=False)
                            id += 1

