# examples

This package contains some files (launch, yaml, ...) that exemplify how the control codes can be used. The examples use the simulators provided in the package robotsim. It also provides a useful node that publishes example paths to be followed by the robot.







## example_path.py


This node is used to publish a reference path. It has some previously defined equations that can beselected to compute the sequence of points that will be published. Another option is to publish an equation that represents a path. After the node publishes the path, it is kept alive pblishing a path to be visualized in rviz.

Everything can be configured via ROS parameters.


### Parameters


- `N_curve` (`int`): id of the previously defined curve to be published according to the following options

	0 &#8594; Generic parametric equation. Publishes an equation (PathEq). The remaining option publishes a sequence of points (Path). \
	1 &#8594; Ellipse. \
	2 &#8594; Eight-like curve. \
	3 &#8594; Square-like curve. \
	4 &#8594; Sine curve. \
	5 &#8594; Smooth eight-like curve. \
	6 &#8594; Amoeba-like curve.


- `N_points` (`int`): number of points to be sampled


- `closed_path_flag` (`bool`): flag to indicate if the path is closed or not

- `insert_n_points` (`int`): number of points to be inserted in between each pair of points of the received path

- `filter_path_n_average` (`int`): number of points to use in the average filter (it is forced to be an odd number) - if 0 the path is not filtered



The following parameters apply when `N_curve > 0`


- `a` (`float`): stretch factor of the curve in the x direction
- `b` (`float`): stretch factor of the curve in the y direction
- `phi` (`float`): rotation of the curve around the z axis (in dregrees)
- `cx` (`float`): displacement in the x direction (meters)
- `cy` (`float`): displacement in the y direction (meters)



The following parameters apply when `N_curve = 0`


- `u_i` (`float`): Inferior limit of the parameter `u` used in the path equation
- `u_f` (`float`): Superior limit of the parameter `u` used in the path equation
- `equation` (`string`): String representing the equation of a point of the curve at the parameter `u`



Check these parameters in the file [config/pub_path.yaml](config/pub_path.yaml).





### Topics

This package does not subscribe to any topic. All of its inputs come from ROS parameters. The published topics are:

- `/example_path`  (message type: `distancefield_msgs/Path`): Topic in which a path represented as a sequence of points is published.

- `/example_path_equation`  (message type: `distancefield_msgs/PathEq`): Topic in which a path represented as an equation is published.

- `/visualization_path`  (message type: `visualization_msgs/MarkerArray`): Topic in which markers are published to be visualized in rviz.





## Ready launch files

This package provides some launch files that can be called to run simple simulations that check the control codes in the other packages of this stack.

All the launch files run the following order of nodes:

- Node rviz to visualize the simulation.
- A simulator node from package robotsim together with the associated parameters.
- Path generator node that publishes a sequence of points or an equation.
- Control node, and associated parameters, according to the simulated robot

The available launch files are:

- `integrator.launch`: Simulation of an integrator robot. Uses only the package distancefield
- `differential.launch`: Simulation of a differential robot. Uses the packages distancefield and ground_robot.
- `skidsteer.launch`: Simulation of a skid steering robot. Uses the packages distancefield and ground_robot.
- `drone.launch`: Simulation of a drone robot. Uses the packages distancefield and quad_robot.








