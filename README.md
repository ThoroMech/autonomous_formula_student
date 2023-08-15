# autonomous_formula_student
Created by Alistair Thorogood 2023 for the requirements of ENG4111 & ENG4112 Research Project.
This Repo is a ros2 workspace designed to work with the formula student driverless simulator.

This repo currently contains four packages, 9 nodes and 6 launch files for autonomous vehicle control:

 - **autonomous_example** (a c++/ros2 copy of the python example provided with the simulator)
	- autonomous_example.launch.py
	- graph_autonomous_example.launch.py

	- car_controller.cpp
	- cone_id.cpp
	- steering_angle.cpp
	- throttle_pos.cpp
	
 - **basic_lap** (an improved version of autonomous_example with path planning & proportional steering)
 	- basic_lap.launch.py
 	- graph_basic_lap.launch.py

	- steering_angle.cpp
 	
 - **graphing** (a package for graphing traffic cone locations)
	- cone_graph.py
	- map_graph.py
	- point_graph.py
 
 - **perception** (improved LiDAR perception)
 	- perception.launch.py
 	- graph_perception.launch.py

	- cone_id.cpp

**python_race_path** was utilised for mapping the race track and may be utilised for future path planning development. 
It is not a part of the ros workspace
 
