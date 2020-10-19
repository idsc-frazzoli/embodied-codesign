import argparse
import json
from decimal import Decimal

import yaml

from simulator.simulator import SimParameters, simulate

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
                       seed=args.seed, do_animation=args.do_animation)
    experiment_key = f'{args.vehicle}-{env_key}-{args.sensor}-{sens_perf_curv_key}-{str(speed)}-{cont_key}'
    performance = simulate(sp=sp, dyn_perf=dyn_perf, sens=sens, sens_curves=alg_sens_curv, s=speed,
             env=env, cont=controller, experiment_key=experiment_key)


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
    parser.add_argument('--controller', type=str, default='cont-th-0.2-ds-2.5-f-20.0', help='Controller name.')
    args = parser.parse_args()

    generate_animation(args)



