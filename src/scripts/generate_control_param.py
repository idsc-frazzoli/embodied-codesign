import yaml

if __name__ == '__main__':
    p_treshold_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    d_stop_list = [1.5, 2.0, 2.5] #2.5, 3]
    t_react = 0.1
    frequency_list = [100.0, 50.0, 20.0, 10.0, 5.0, 2.0, 1.0]

    control_param = {}
    for p_treshold in p_treshold_list:
        for d_stop in d_stop_list:
            for freq in frequency_list:
                param = {"prob_threshold": p_treshold, "d_stop": d_stop, "t_react": t_react, "frequency": freq}
                cont_key = f'cont-th-{p_treshold}-ds-{d_stop}-tr-{t_react}-f-{freq}'
                control_param[cont_key] = param

    with open('data/input/control_param.yaml', 'w') as file:
        documents = yaml.dump(control_param, file, default_flow_style=False)
