# autonomous_formula_student
Created by Alistair Thorogood 2023 for the requirements of ENG4111 & ENG4112 Research Project.
This Repo is a ros2 workspace designed to work with the formula student driverless simulator.

This repo currently contains four packages and 6 launch files for autonomous vehicle control:

 - autonomous_example (a c++/ros2 copy of the python example provided with the simulator)
	- autonomous_example.launch.py
	- graph_autonomous_example.launch.py
	
 - basic_lap (an improved version of autonomous_example with path planning & proportional steering)
 	- basic_lap.launch.py
 	- graph_basic_lap.launch.py
 	
 - graphing (a package for graphing traffic cone locations)
 
 - perception (improved LiDAR perception)
 	- perception.launch.py
 	- graph_perception.launch.py

 
