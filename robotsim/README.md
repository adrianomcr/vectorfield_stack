# robotsim

This package provides simple implementations of several robots. The objective of this package is to enable simple examples of the use of the control packages of this stack. The simulators rely only on basic ROS features, do not requiring none complex installation. The simulation can be visualized in rviz by using provided configuration files.

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

![formula](https://render.githubusercontent.com/render/math?math=\dot{x}=v\cos(\theta)) \
![formula](https://render.githubusercontent.com/render/math?math=\dot{y}=v\sin(\theta)) \
![formula](https://render.githubusercontent.com/render/math?math=\dot{\theta}=\omega)

where `[x, y]` is the robot's position, `θ` is the robot's yaw angle, `v` is the linear velocity and `ω` is the angular velocity.

The simulation can be visualized in rviz. For that, use the configuration file `rviz/differential.rviz`.



## skidsteer_sim.py

This node simulates a skid steering robot with four wheels. The kinematic equation of the robot model is equal to the equation of the diferential drive robot. However, here the velocities `v` and `ω` are defined from the right (v<sub>r</sub>) and left (v<sub>l</sub>) wheels speeds according to:

![formula](https://render.githubusercontent.com/render/math?math=v=0.5\cdot(v_r%2Bv_l)) \
![formula](https://render.githubusercontent.com/render/math?math=\omega=[b/(2a^2%2B2b^2)]\cdot(v_r-v_l))

where `a` and `b` are the geometric parameters of the robot (see [here](../ground_robot))

The simulation can be visualized in rviz. For that, use the configuration file `rviz/skidsteer.rviz`.






## drone_sim.py


This node simulates a drone operating in the acro mode. As control inputs, it receives the total thrust force and the angular body rates. The equations that describe the robot's dynamics are:

![image](https://github.com/adrianomcr/vectorfield_stack/blob/main/robotsim/images/drone_eq.png)

<!-- ![formula](https://render.githubusercontent.com/render/math?math=\dot{x}=v) \ -->
<!-- ![formula](https://render.githubusercontent.com/render/math?math=\dot{v}=1) \ -->
<!-- ![formula](https://render.githubusercontent.com/render/math?math=\dot{q}=1) -->

where `x` is the drone position, `v` is the velocity in the world frame and `R` is a rotation matrix representing the drone's orientation. The unit vector `ẑ` points up and f<sub>d</sub> is a drag force. The control inputs arethe total thrust `τ` and the angular speeds `ω`, written in the body frame. Finally, `S(ω)` is the skew-stmetric matrix.

The simulation can be visualized in rviz. For that, use the configuration file `rviz/drone.rviz`.


