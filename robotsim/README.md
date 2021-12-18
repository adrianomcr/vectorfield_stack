# robotsim

This package provides simple implementations of several robots. The objective of this package is to enable simple examples of the use of the control packages of this stack.

The following robot types are available:

- Holonomic 3D (used to simulate distancefield package alone)

- Differential drive (used to simulate ground_robot package)

- Skid-steering (used to simulate ground_robot package)

- Quadcopter (used to simulate quad_robot package)


Below, the nodes relative to each simulator are explained:


## integrator_sim.py

This node simulates a 3D integrator robot. The equation of the robot model is:

![formula](https://render.githubusercontent.com/render/math?math=\dot{x}=u)

where `x` is the robot position and `u` is the control input corresponding to linear velocity.

The simulation can be visualized in rviz. For that, use the configuration file `rviz/integrator.rviz`.


### Parameters


- `pos_0` (`float list`): array with the initial position of the robot
- `robot_radius` (`float`): radius of the spherical robot

- `obtscles_pos` (`float list of list`): list of 3D position of spherical obstacles
- `obtscles_r` (`float list`): list of the radius of the spherical obstacles

Check these parameters in the file [config/integrator_sim.yaml](config/integrator_sim.yaml).


### Topics

- `/integrator/pose`  (message type: `geometry_msgs/Pose`): Topic in which the robot's position is published.
- `/integrator/closest_point`  (message type: `geometry_msgs/Point`): Topic in which the closest obstacle point (in the <strong>world</strong> frame) is published.

- `/integrator/robot`  (message type: `visualization_msgs/Marker`): Topic in which the a marker that represents the robot is published. In can be seen in rviz.

- `/integrator/closest_marker`  (message type: `visualization_msgs/Marker`): Topic in which the a marker that represents the closest colidable point is published. In can be seen in rviz.

- `/integrator/obstacles`  (message type: `visualization_msgs/MarkerArray`): Topic in which the markers that represent the obstacles are published. In can be seen in rviz.

- `/integrator/history`  (message type: `visualization_msgs/MarkerArray`): Topic in which the past history of the robot's position is published. In can be seen in rviz.

- `/integrator/vel`  (message type: `geometry_msgs/Twist`): Topic that the node subscripes to in order to get a linear velocity command for the robot. This velocity is in the world frame.




## differential_sim.py

This node simulates a diferential drive robot. The equation of the robot model is:

<!-- ![formula](https://render.githubusercontent.com/render/math?math=\begin{equation}%20\begin{array}{l}%20\dot{x}%20=%20v\cos(\theta)%20\\%20\dot{y}%20=%20v\sin(\theta)%20\\%20\dot{\theta}%20=%20\omega%20\end{array}%20\end{equation}) -->
![formula](https://render.githubusercontent.com/render/math?math=\dot{x}=v\cos(\theta)) \
![formula](https://render.githubusercontent.com/render/math?math=\dot{y}=v\sin(\theta)) \
![formula](https://render.githubusercontent.com/render/math?math=\dot{\theta}=\omega)

where `[x, y]` is the robot's position, `θ` is the robot's yaw angle, `v` is the linear velocity and `ω` is the angular velocity.

The simulation can be visualized in rviz. For that, use the configuration file `rviz/differential.rviz`.



## skidsteer_sim.py



## drone_sim.py