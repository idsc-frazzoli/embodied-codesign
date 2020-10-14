

generate_cat:
	PYTHONPATH=src python src/scripts/create_catalogue.py

camera_curves_generation:
	PYTHONPATH=src python src/scripts/sensor_curves_generation.py

lidar_curves_generation:
	PYTHONPATH=src python src/scripts/generate_curves_lidar.py
generate_animation:
	PYTHONPATH=src python src/scripts/generate_animation.py
