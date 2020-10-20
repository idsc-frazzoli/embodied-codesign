
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

# parameters for curve generation and plotting
CONTROL_PARAM_FREQ=20.0 10.0 5.0 1.0 0.5
CONTROL_PARAM_TH=0.1 0.2 0.3 0.4 0.5
CONTROL_PARAM_DSTOP=3.0

# parameters for single fixed simulation
NSIMS=10
DT=0.01
ROADLENGTH=500.0
DOANIMATION=True
SEEDSIM=0
SENSOR=Ace13gm
VEHICLE=sedan_s
ENVIRONMENTSIM=10day_env
ENVDENSITY=7
ENVDAYNIGHT=day
ALGORITHM=faster_rcnn1
SPEED=10.0
CONTROLFREQ=20.0
CONTROLTHRESHOLD=0.1
CONTROLLER=cont_th_0.2_ds_2.5_f_20.0
BASEDIR=DB
ADDOBJECTAT=none


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
	PYTHONPATH=src python src/embodied_scripts/generate_control_param.py --control_freq $(CONTROL_PARAM_FREQ)


generate_animation:
	PYTHONPATH=src python src/embodied_scripts/generate_animation.py --nsims $(NSIMS) --dt $(DT) --road_length $(ROADLENGTH) \
	--do_animation $(DOANIMATION) --seed $(SEEDSIM) --sensor $(SENSOR) --vehicle $(VEHICLE) --environment $(ENVIRONMENTSIM) \
	--env_density $(ENVDENSITY) --env_day_night $(ENVDAYNIGHT) --algorithm $(ALGORITHM) --speed $(SPEED) --control_freq $(CONTROLFREQ) \
	--control_treshold $(CONTROLTHRESHOLD) --controller $(CONTROLLER) --basedir $(BASEDIR) --add_object_at $(ADDOBJECTAT)

