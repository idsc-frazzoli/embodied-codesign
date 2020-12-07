
out=out
tr=$(out)/test-results

tag=embodied_codesign

test_packages=controller_tests,embodied_scripts_tests,sensing_tests,simulator_tests,vehicle_tests,utils_tests

parallel=--processes=8 --process-timeout=1000 --process-restartworker

extra=--rednose --immediate

# parameters for curve generation and plotting
DS=0.1
SENSE_RANGE=50.0
PLOTSENSCURVE=OS0128_day_kde_based

# parameters for controller generation
CONTROL_PARAM_FREQ=20.0 10.0 5.0 1.0 0.5
CONTROL_PARAM_TH=1 2 5 7
CONTROL_PARAM_DSTOP=3.0
CONTROL_PARAM_PAMAX=0.5

#parameters for environment generation
ENV_SIM_DENSITY=1.0 5.0 10.0 25.0 50.0 100.0
ENV_SIM_DAYNIGHT=day night

# parameters for single fixed simulation
NSIMS=100
DT=0.01
ROADLENGTH=500.0
DOANIMATION=--do_animation
SEEDSIM=0
SENSOR=Ace13gm
VEHICLE=sedan_s
ENVIRONMENTSIM=env_d_5.0_day
ENVDENSITY=7
ENVDAYNIGHT=day
ALGORITHM=faster_rcnn1
SPEED=10.0
CONTROLFREQ=20.0
CONTROLTHRESHOLD=5.0
CONTROLLER=none
BASEDIR=DB
ADDOBJECTAT=none
STOPTIME=400.0
DISCOMFORT_PENALTY=5.0

# create catalogue parameters
ALL=--not_all
SENSOR_KEY=none
VEHICLE_KEY=none
ENVIRONMENT_KEY=none
CONTROL_KEY=none
ALG_KEY=none
SPEED_LIST=none
DOANIMATION_CAT=--do_not_animation
NPROCESSES=4
SIMULATION_VERSION=simulation_v.1.0

all:
	@echo "You can try:"
	@echo
	@echo " make clean test "

clean:
	coverage erase
	rm -rf $(out) $(coverage_dir) $(tr)

test: clean
	mkdir -p  $(tr)
	DISABLE_CONTRACTS=1 nosetests $(extra)  src  -v --nologcapture


test-parallel: clean
	mkdir -p  $(tr)
	DISABLE_CONTRACTS=1 nosetests $(extra) $(coverage) src  -v --nologcapture $(parallel)


generate_sens_curves:
	PYTHONPATH=src python src/embodied_scripts/generate_sensing_curves.py --ds $(DS) --max_distance $(SENSE_RANGE)


plot_sens_curves:
	PYTHONPATH=src python src/embodied_scripts/plot_sensing_curves.py --sens_alg $(PLOTSENSCURVE)


generate_control_param:
	PYTHONPATH=src python src/embodied_scripts/generate_control_param.py --control_freq $(CONTROL_PARAM_FREQ) \
	--control_treshold $(CONTROL_PARAM_TH) --control_d_stop $(CONTROL_PARAM_DSTOP) --control_percentage_amax $(CONTROL_PARAM_PAMAX)


generate_sim_environment:
	PYTHONPATH=src python src/embodied_scripts/generate_sim_environment.py --env_density $(ENV_SIM_DENSITY) \
	--env_scen_day_night $(ENV_SIM_DAYNIGHT)


generate_animation:
	PYTHONPATH=src python src/embodied_scripts/generate_animation.py --nsims $(NSIMS) --dt $(DT) --road_length $(ROADLENGTH) \
	$(DOANIMATION) --seed $(SEEDSIM) --sensor $(SENSOR) --vehicle $(VEHICLE) --environment $(ENVIRONMENTSIM) \
	--env_density $(ENVDENSITY) --env_day_night $(ENVDAYNIGHT) --algorithm $(ALGORITHM) --speed $(SPEED) --control_freq $(CONTROLFREQ) \
	--control_treshold $(CONTROLTHRESHOLD) --controller $(CONTROLLER) --basedir $(BASEDIR) --add_object_at $(ADDOBJECTAT) \
	--control_d_stop $(CONTROL_PARAM_DSTOP) --control_percentage_amax $(CONTROL_PARAM_PAMAX) --stop_time $(STOPTIME) --discomfort_penalty $(DISCOMFORT_PENALTY)

create_catalogue:
	PYTHONPATH=src python src/embodied_scripts/create_catalogue.py $(ALL) --sensor_key $(SENSOR_KEY) \
	--vehicle_key $(VEHICLE_KEY) --environment_key $(ENVIRONMENT_KEY) --control_key $(CONTROL_KEY) \
	 --alg_key $(ALG_KEY) --speed_list $(SPEED_LIST) $(DOANIMATION_CAT) --seed $(SEEDSIM) \
	 --nsims $(NSIMS) --dt $(DT) --road_length $(ROADLENGTH) --basedir $(BASEDIR) --nprocesses $(NPROCESSES) \
	 --simversion $(SIMULATION_VERSION) --stop_time $(STOPTIME) --discomfort_penalty $(DISCOMFORT_PENALTY)

generate_mcdp_file:
	PYTHONPATH=src python src/embodied_scripts/generate_mcdp_file.py --basedir $(BASEDIR) \
	 --simversion $(SIMULATION_VERSION)

combine_mcdp_files:
	PYTHONPATH=src python src/embodied_scripts/combine_mcdp_files.py --basedir $(BASEDIR) \
	 --simversion $(SIMULATION_VERSION)



