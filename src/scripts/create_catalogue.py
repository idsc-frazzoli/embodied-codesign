import os
from decimal import Decimal

import yaml

from simulator.simulator import SimParameters, simulate

if __name__ == '__main__':
    current_directory = os.path.dirname(__file__)
    parent_directory = os.path.split(current_directory)[0]
    parent_directory = os.path.split(parent_directory)[0]

    with open(parent_directory + '/data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    with open(parent_directory + '/data/input/dyn_perf.yaml') as file:
        vehicles = yaml.load(file, Loader=yaml.FullLoader)

    with open(parent_directory + '/data/input/environment.yaml') as file:
        environment = yaml.load(file, Loader=yaml.FullLoader)

    with open(parent_directory + '/data/input/speeds.yaml') as file:
        cruise_speeds = yaml.load(file, Loader=yaml.FullLoader)

    with open(parent_directory + '/data/input/control_param.yaml') as file:
        control_param = yaml.load(file, Loader=yaml.FullLoader)


    camera = cameras = sensors["camera"]
    speed = cruise_speeds["speeds"]

    sp = SimParameters(nsims=50, road_length=Decimal('500.0'), dt=Decimal(0.01), seed=0, wt=Decimal('1.0'))

    for veh_key, dyn_perf in vehicles.items():
        for cam_key, sens in camera.items():
            for s in speed:
                for env_key, env in environment.items():
                    for cont_key, cont in control_param.items():
                        performance = simulate(sp, dyn_perf, sens, s, env, cont)

