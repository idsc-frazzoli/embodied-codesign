import argparse
import os

import yaml

def generate_sim_environment(args):
    density_list = args.env_density
    scen_day_night = args.env_scen_day_night

    env_param = {}
    for dens in density_list:
        for dn in scen_day_night:
            param = {"density_ped_km": dens, "scenario_day_night": dn}
            env_key = f'env_d_{dens}_{dn}'
            env_param[env_key] = param

    fn = os.path.join(f'data/input/environment.yaml')
    dn = os.path.dirname(fn)
    if not os.path.exists(dn):
        os.makedirs(dn)

    with open(fn, 'w') as file:
        yaml.dump(env_param, file, default_flow_style=False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating sensing curves.')
    parser.add_argument('--env_density', nargs='+', type=float, default=[1.0, 5.0, 10.0, 25.0, 50.0, 100.0],
                        help='Pedestrian desnsity per km.')
    parser.add_argument('--env_scen_day_night', nargs='+', type=str, default=['day', 'night'], help='Day or night time.')

    args = parser.parse_args()
    generate_sim_environment(args)
