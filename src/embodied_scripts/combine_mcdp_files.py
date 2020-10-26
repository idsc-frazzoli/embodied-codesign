import argparse
import os
from utils.yaml_file_generation import combine_mcdp

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating MCDP files.')
    parser.add_argument('--basedir', type=str, default='DB', help='base directory.')
    parser.add_argument('--mcdp_folder', type=str, default='mcdp', help='mcdp folder path in basedir.')
    parser.add_argument('--simversion', type=str, default='simulation_v.1.0', help='Simulator version.')


    args = parser.parse_args()


    fn_dpc_models = os.path.join(args.basedir, args.mcdp_folder, args.simversion, f'{args.simversion}.joint.brake_control_models.yaml')
    dn = os.path.dirname(fn_dpc_models)
    if not os.path.exists(dn):
        os.makedirs(dn)

    if not os.path.exists(fn_dpc_models):
        combine_mcdp(os.path.join(args.basedir, args.mcdp_folder), fn_dpc_models)
