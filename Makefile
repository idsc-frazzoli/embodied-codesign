
out=out
tr=$(out)/test-results

tag=embodied_codesign

test_packages=controller_tests,embodied_scripts_tests,sensing_tests,simulator_tests,vehicle_tests,utils_tests

parallel=--processes=8 --process-timeout=1000 --process-restartworker

extra=--rednose --immediate

DS=0.1
SENSE_RANGE=50.0
PLOTSENSCURVE=OS0128_day_kde_based


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

# camera_curves_generation:
# 	PYTHONPATH=src python src/scripts/sensor_curves_generation.py
#
# lidar_curves_generation:
# 	PYTHONPATH=src python src/scripts/generate_curves_lidar.py
#
# generate_animation:
# 	PYTHONPATH=src python src/scripts/generate_animation.py
