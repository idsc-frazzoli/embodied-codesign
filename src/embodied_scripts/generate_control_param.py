import argparse

import yaml

def generate_control_param(args):
    p_treshold_list = args.control_treshold
    d_stop = args.control_d_stop
    frequency_list = args.control_freq

    control_param = {}
    for p_treshold in p_treshold_list:
        for freq in frequency_list:
            param = {"prob_threshold": p_treshold, "d_stop_m": d_stop, "frequency_hz": freq}
            cont_key = f'cont_th_{p_treshold}_ds_{d_stop}_f_{freq}'
            control_param[cont_key] = param

    with open('data/input/control_param.yaml', 'w') as file:
        documents = yaml.dump(control_param, file, default_flow_style=False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating sensing curves.')
    parser.add_argument('--control_freq', nargs='+', type=float, default=[20.0, 10.0, 5.0, 1.0, 0.5], help='Controller frequency in Hz.')
    parser.add_argument('--control_treshold', nargs='+', type=float, default=[0.1, 0.2, 0.3, 0.4, 0.5], help='Treshohld for controller.')
    parser.add_argument('--control_d_stop', type=float, default=3.0, help='Additional distance to stop in front of object.')

    args = parser.parse_args()
    generate_control_param(args)
