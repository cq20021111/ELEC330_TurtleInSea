Before operating, place the package (Teri_bot_final) in the src directory of any workspace.

After placing the files in the correct location, open a terminal and navigate to the workspace directory
Then use the following command to launch the model in Gazebo:
	colcon build
	source install/setup.bash
	ros2 launch Teri_bot launch.py

You will see Gazebo, RViz, and a terminal opened for observing relevant information.
