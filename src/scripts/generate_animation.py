import json
from decimal import Decimal

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

    with open('data/input/camera_curves.json') as file:
        camera_curves = json.load(file)

    with open('data/input/lidar_curves.json') as file:
        lidar_curves = json.load(file)


    camera = sensors["camera"]
    lidars = sensors['lidar']
    speed = cruise_speeds["speeds"]

    sp = SimParameters(nsims=1, road_length=Decimal('500.0'), dt=Decimal(str(0.1)),
                       seed=0, do_animation=True)

    dyn_perf = vehicles["sedan_s"]
    sens = camera["Ace13gm"]
    sens = lidars['OS2128']
    sens_perf = sens["sens_perf"][1]
    # sens_curves = camera_curves[sens_perf]
    sens_curves = lidar_curves[sens_perf]
    s = speed[6]
    env_key = list(environment)[0] #["07day_env"]
    env = environment[env_key]
    print(list(control_param))
    k = 'cont-th-0.5-ds-2.5-tr-0.1-f-5.0'
    cont = control_param[k]

    performance = simulate(sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key="test6")


