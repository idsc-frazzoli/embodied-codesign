import argparse
import os

from utils.yaml_file_generation import read_results, write_bc_dpc

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating MCDP files.')
    parser.add_argument('--basedir', type=str, default='DB', help='Controller name.')
    parser.add_argument('--simversion', type=str, default='simulation_v.1.0', help='Controller name.')

    args = parser.parse_args()


    fn_results = os.path.join(args.basedir, args.simversion, 'catalogue', f'{args.simversion}.results.yaml')
    dn = os.path.dirname(fn_results)
    if not os.path.exists(dn):
        os.makedirs(dn)

    if not os.path.exists(fn_results):
        read_results(args.basedir, fn_results)
        print(f'Finished results file of {args.simversion}.')

    fn_dpc_models = os.path.join(args.basedir, args.simversion, 'catalogue', f'{args.simversion}.brake_control_models.yaml')

    if not os.path.exists(fn_dpc_models):
        write_bc_dpc(dn, fn_dpc_models)
        print(f'Finished catalogue DPC file of {args.simversion}.')

