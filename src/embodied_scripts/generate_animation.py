import argparse
import json
import os
from decimal import Decimal

import yaml

from simulator.simulator import SimParameters, simulate


def write_store_param_file(args, param_file_name, alg_sens_curv):
    params = vars(args)
    params["ds"] = alg_sens_curv["ds"]
    params["max_distance"] = alg_sens_curv["max_distance"]
    with open(param_file_name, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)


def generate_animation(args):
    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/dyn_perf.yaml') as file:
        vehicles = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/environment.yaml') as file:
        environment = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/sensing_performance_curves.json') as file:
        sens_curves = json.load(file)

    with open('data/input/control_param.yaml') as file:
        control_param = yaml.load(file, Loader=yaml.FullLoader)

    sens = sensors[args.sensor]
    dyn_perf = vehicles[args.vehicle]

    if args.environment == "none":
        env = {"density_ped_km": float(args.env_density), "scenario_day_night": args.env_day_night}
        env_key = f'{args.env_density}{env["scenario_day_night"]}_env'
    else:
        env = environment[args.environment]
        env_key = args.environment

    sens_perf_curv_key = f'{args.sensor}_{env["scenario_day_night"]}_{args.algorithm}'
    alg_sens_curv = sens_curves[sens_perf_curv_key]
    speed = Decimal(args.speed)

    if args.controller == "none":
        d_stop = Decimal('2.5')
        p_treshold = Decimal(args.control_treshold)
        cont_freq = Decimal(args.control_freq)
        controller = {"d_stop_m": float(d_stop), "frequency_hz": float(cont_freq), "prob_threshold": float(p_treshold)}
        cont_key = f'cont-th-{str(p_treshold)}-ds-{str(d_stop)}-f-{str(cont_freq)}'
    else:
        controller = control_param[args.controller]
        cont_key = args.controller

    sp = SimParameters(nsims=args.nsims, road_length=Decimal(args.road_length), dt=Decimal(args.dt),
                       seed=args.seed, do_animation=args.do_animation, add_object_at=args.add_object_at)
    experiment_key = f'{args.vehicle}-{env_key}-{args.sensor}-{sens_perf_curv_key}-{str(speed)}-{cont_key}'
    file_dir = os.path.join(args.basedir, str(args.vehicle), str(env_key), str(args.sensor),
                      str(sens_perf_curv_key), str(speed), str(cont_key))
    if not os.path.exists(file_dir):
        os.makedirs(file_dir)
    file_name = os.path.join(file_dir, f'{experiment_key}.experiment.yaml')
    param_file_name = os.path.join(file_dir, f'{experiment_key}.parameters.yaml')
    write_store_param_file(args, param_file_name, alg_sens_curv)

    performance = simulate(sp=sp, dyn_perf=dyn_perf, sens=sens, sens_curves=alg_sens_curv, s=speed,
             env=env, cont=controller, experiment_key=experiment_key, file_directory=file_dir)

    danger = {
        "mean": str(round(performance.danger.mean, 2)), "var": str(round(performance.danger.var, 2)),
        "u95": str(round(performance.danger.u95, 2)),
        "l95": str(round(performance.danger.l95, 2))
    }
    discomfort = {
        "mean": str(round(performance.discomfort.mean, 2)), "var": str(round(performance.discomfort.var, 2)),
        "u95": str(round(performance.discomfort.u95, 2)), "l95": str(round(performance.discomfort.l95, 2))
    }
    ad_perf = {
        "danger": danger,
        "discomfort": discomfort,
        "speed": float(speed),
    }

    with open(file_name, 'w') as f:
        yaml.dump(ad_perf, f, default_flow_style=False)
    print(f'Finished experiment: {experiment_key}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating sensing curves.')
    parser.add_argument('--nsims', type=int, default=10, help='Number of simulations per parameter set.')
    parser.add_argument('--dt', type=str, default='0.01', help='Simulation sampling time in seconds.')
    parser.add_argument('--road_length', type=str, default='500.0', help='Road length in meters.')
    parser.add_argument('--do_animation', type=bool, default=True, help='Flag for crating an animation.')
    parser.add_argument('--seed', type=int, default=0, help='Seed for simulation.')
    parser.add_argument('--sensor', type=str, default='Ace13gm', help='Sensor name from sensor.yaml file.')
    parser.add_argument('--vehicle', type=str, default='sedan_s', help='Dynamic performance of the vehicle from dyn_perf.yaml file.')
    parser.add_argument('--environment', type=str, default='10day_env', help='Environment from environment.yaml file.')
    parser.add_argument('--env_density', type=str, default='7', help='Environment density in pedestrian per kilometer.')
    parser.add_argument('--env_day_night', type=str, default='day', help='Environment day or night.')
    parser.add_argument('--algorithm', type=str, default='faster_rcnn1', help='Algorithm for object detection '
                                                                              'from sensors.yaml file. Check if lidar or cam!!.')
    parser.add_argument('--speed', type=str, default='10.0', help='Cruise speed in meter per seconds.')
    parser.add_argument('--control_freq', type=str, default='20.0', help='Controller frequency in Hz.')
    parser.add_argument('--control_treshold', type=str, default='0.1', help='Treshohld for controller.')
    parser.add_argument('--controller', type=str, default='cont_th_0.2_ds_2.5_f_20.0', help='Controller name.')
    parser.add_argument('--basedir', type=str, default='DB', help='Controller name.')
    parser.add_argument('--add_object_at', type=str, default='20.0', help='Add additional object at a specif '
                                                                          'distance for testing. If no additional write none.')
    args = parser.parse_args()

    generate_animation(args)



