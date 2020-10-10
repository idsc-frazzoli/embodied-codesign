import os
from decimal import Decimal
from multiprocessing import Pool

import yaml

from simulator.simulator import SimParameters, simulate

def simulate_and_write(params):
    sp = params[0]
    dyn_perf = params[1]
    sens = params[2]
    sens_curves = params[3]
    s = params[4]
    env = params[5]
    cont = params[6]
    experiment_key = params[7]
    fn = params[8]
    sens_key = [9]
    s_perf = [10]
    veh_key = params[11]
    env_key = params[12]
    cont_key = params[13]
    performance = simulate(sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key)
    ad_perf = {
        "danger": str(performance.danger),
        "discomfort": str(performance.discomfort),
        "speed": s, "sensor": sens_key, "sens_perf": s_perf, "dyn_perf": veh_key,
        "environment": env_key,
        "controller": cont_key,
    }
    yaml.dump(ad_perf, fn, default_flow_style=False)

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

    with open('data/input/camera_curves.yaml') as file:
        camera_curves = yaml.load(file, Loader=yaml.FullLoader)

    sens_type_key = "camera"
    camera = sensors[sens_type_key]
    speed = cruise_speeds["speeds"]

    sp = SimParameters(nsims=100, road_length=Decimal('500.0'), dt=Decimal(str(0.01)),
                       seed=0, do_animation=True)

    veh_key = "suv_m"
    dyn_perf = vehicles[veh_key]
    env_key = "07day_env"
    sens_key = "Ace13gm"
    sens = camera[sens_key]
    s_perf = sens["sens_perf"][0]
    sens_curves = camera_curves[s_perf]
    s = speed[4]
    env = environment[env_key]
    basedir = 'output'
    to_run = []  # list of parameters
    for cont_key, cont in control_param.items():
        experiment_key = f'{veh_key}-{env_key}-{sens_type_key}-{sens_key}-{s_perf}-{s}-{cont_key}'
        fn = os.path.join(basedir, f'output/{experiment_key}.experiment.yaml')
        params = sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key, fn, sens_key, s_perf, veh_key, env_key, cont_key
        to_run.append(params)

    nprocesses = 1
    with Pool(processes=nprocesses) as pool:
        pool.map(simulate_and_write, to_run)



