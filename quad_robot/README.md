# quad_robot


This package uses the distancefield package to support a controller for quadcopters. It assumes that the control commands are the thrust force and the angular body rates. It can be checked with the quadcopter available in the robotsim package. The main theory that supports this implementation is published in the following paper:

[2] Adriano M. C. Rezende, Vinicius M. Gonçalves, Arthur H. D. Nunes and Luciano C. A. Pimenta, **Robust quadcopter control with artificial vector fields,** 2020 IEEE International Conference on Robotics and Automation (ICRA), 2020, pp. 6381-6387, doi: 10.1109/ICRA40945.2020.9196605. Online: <https://ieeexplore.ieee.org/document/9196605>






## quadrobot_class (python)


This class is an implementation of a controller for a quadcopter robot. The controller is designed upon the vector field implemented in the distancefield package (check it [here](../distancefield)), and makes the drone follow the vector field. <strong>The quadrobot_class object has a member of the distancefield_class called `vec_field_obj`</strong>.

Currently, the quadrobot_class assumes a drone in the acrorate mode. It means that the control inputs are the total thrust force and the three body angular rates. The image below illustrates the robot model that the class considers:

![image](https://github.com/adrianomcr/vectorfield_stack/blob/main/quad_robot/images/acrorate.png)

The available methods are listed below.




### Methods



#### `__init__(self, vr, kf, reverse_direction, flag_follow_obstacle, epsilon, switch_dist, m, Kv, Kw):`

Constructor method. It receives the following parameters:

- `vr` (`float`): norm of the velocity of the field
- `kf` (`float`): convergence gain of the vector field
- `reverse_direction` (`bool`): flag to make the vector field follow the curve in the opposite direction
- `flag_follow_obstacle` (`bool`): flag to enable/disable the object follow feature
- `epsilon` (`float`): distance that a close obstacle will be followed
- `switch_dist` (`float`): distance from which an obstacle starts to be followed
- `m` (`float`): mass of the drone
- `Kv` (`float`): gain of the action proportional to the velocity error (field x drone velocity)
- `Kw` (`float`): gain of the action proportional to the orientation error (orientation_ref x drone orientation)





#### `compute_Jacobian(self, pos):`

Method to compute the Jacobian matrix of the vector field at the point `pos`.


#### `get_orientation_ref(self, a_r, psi_r):`

Method to compute a matrix that represents the reference for the drone's attitude from the acceleration vector `a_r` and the yaw reference `yaw_r`.


#### `get_acc_ref(self, pos, vel, out=[0], id=0):`

Method to compute the acceleration reference for the robot when it is at a point `pos` with a velocity `vel` in the world frame.


#### `quat2rotm(self,q):`

Method that returns a rotation matrix equivalent to a quaternion `q`.


#### `control_step_parallel(self):`

<em>This method is inactive</em>


#### `control_step(self):`

This method considers the current state of the drone and updates the internal control input variables `tau` and `omega`. To get these variables use the method `get_acrorate` described below.


#### `get_acrorate(self):`

Consider calling this method after the method `control_step` described above.


#### `set_state(self, state):`

Method that returns the current state that the control object is set.


#### `set_pos(self, pos):`

Method to set the position `pos` of the drone.


#### `set_quat(self, quat):`

Method to set the orientation `quat` of the drone. The for of the quaternion is q = [q<sub>w</sub>, q<sub>x</sub>, q<sub>y</sub>, q<sub>z</sub>].


#### `set_vel(self, vel):`

Method to set the velocity `vel` (in the world frame) of the drone.


#### `set_closest(self, point):`

Method to set the 3D position (`point`) of the closest point (in the world frame) that belongs to an obstacle.



#### `quat2axang(self, q):`

Method that returns a 3D axis and an angle that correspond to the axis-angle representation of the quaternion `q`.

#### `rotm2axang(self, R):`

Method that returns a 3D axis and an angle that correspond to the axis-angle representation of the rotation matrix `R`.


#### `rotm2quat(self, R):`

Method that returns a quaternion equivalent to the matrix `R`.












## quad_node.py

This ROS node has an implementation that can be used to control a quadcopter that responds to acrorate (total thrust and angular rates) commands. Basically, it subscribes to a topic to get the robot's state, computes the control inputs for the drone, and publishes the result in a command topic.

It can be tested with drone_sim node available in the [robotsim](../robotsim) package. See the package [examples](../examples) for instruction in how to launch this simulation.


### Parameters


- `vr` (`float`): norm of the velocity of the field
- `kf` (`float`): convergence gain of the vector field
- `reverse_direction` (`bool`): flag to make the vector field follow the curve in the opposite direction
- `m` (`float`): mass of the drone
- `kv` (`float`): gain of the action proportional to the velocity error
- `kw` (`float`): gain of the action proportional to the orientation error

- `flag_follow_obstacle` (`bool`): flag to enable/disable the object follow feature
- `epsilon` (`float`): distance that a close obstacle will be followed
- `switch_dist` (`float`): distance from which an obstacle starts to be followed
- `obstacle_point_topic_name` (`string`): name of the topic in which the closest point of the obstacles are published

- `pose_topic_name` (`string`): name of the topic in which the robot's pose is published
- `pose_topic_type` (`string`): type of the topic in which the robot's pose is published (`TFMessage`, `Pose`, `Odometry`)
- `acrorate_cmd_topic_name` (`string`): name of the topic in which acrorate commands are published
- `path_topic_name` (`string`): name of the topic in which the path (sequence of points) is published
- `path_equation_topic_name` (`string`): name of the topic in which the path (equation) is published


Check these parameters in the file [config/quad_params.yaml](config/quad_params.yaml).






### Topics


- `path_topic_name`  (message type: `distancefield_msgs/Path`): Subscribe to this topic to get a path represented as a sequence of points
- `path_equation_topic_name`  (message type: `distancefield_msgs/PathEq`): Subscribe to this topic to get a path represented by a parametric equation
- `obstacle_point_topic_name`  (message type: `std_msgs/Point`): Subscribe to this topic to get the closest collidable point written in the <strong>world</strong> reference frame.
- `pose_topic_type`  (message type: `nav_msgs/Odometry`): Subscribe to this topic to get the robot's states


- `acrorate_cmd_topic_name`  (message type: `geometry_msgs/Quaternion`): Topic in which the acrorate command is published. The real quaternion element q<sub>w</sub> corresponds to the total thrust, while the imaginary elements [q<sub>x</sub> q<sub>y</sub> q<sub>z</sub>] correspond to the angular body rates.







## TODO

- Add saturation in the class

- Add ROS message for the acrorate command