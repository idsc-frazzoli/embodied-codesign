import argparse
import os
from decimal import Decimal

import yaml

from simulator.simulator import SimParameters
from utils.yaml_file_generation import read_results, write_bc_dpc, read_results_from_single_exp

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating MCDP files.')
    parser.add_argument('--basedir', type=str, default='DB', help='Controller name.')
    parser.add_argument('--simversion', type=str, default='simulation_v.1.2_sensors_nsims_50', help='Controller name.')
    parser.add_argument('--discomfort_penalty', type=str, default='5.0', help='The discomfort penalty when collided.')
    args = parser.parse_args()


    fn_results = os.path.join(args.basedir, args.simversion, 'catalogue', f'{args.simversion}.results.yaml')
    dn = os.path.dirname(fn_results)
    if not os.path.exists(dn):
        os.makedirs(dn)

    fn_parameters = os.path.join(args.basedir, args.simversion, f'{args.simversion}.parameters.yaml')
    with open(fn_parameters) as f:
        param = yaml.load(f.read(), Loader=yaml.FullLoader)
    sp = SimParameters(nsims=param["nsims"], road_length=Decimal(param['road_length']), dt=Decimal(param['dt']),
                       seed=param['seed'], do_animation=False, add_object_at="none",
                       stop_time=Decimal(param['stop_time']), discomfort_penalty=Decimal(args.discomfort_penalty))

    if not os.path.exists(fn_results):
        read_results_from_single_exp(os.path.join(args.basedir, args.simversion), fn_results, sp=sp)
        print(f'Finished results file of {args.simversion}.')

    fn_dpc_models = os.path.join(args.basedir, args.simversion, 'catalogue', f'{args.simversion}.brake_control_models.yaml')

    if not os.path.exists(fn_dpc_models):
        write_bc_dpc(dn, fn_dpc_models)
        print(f'Finished catalogue DPC file of {args.simversion}.')

