# ground_robot


This package uses the distancefield package to support a controller for ground robots. It can be used to control a diferential drive or a skid-steering robot. Two types of input controls are available, (i) linear and angular speeds, and (ii) wheels speeds. It can be checked with the differential and skid-steering robots available in the robotsim package.



## groundrobot_class (python)

This class is a implementation of controllers for wheeled ground robots. The controllers are disigned upon the vector field implemented in the distancefield package (check it [here](../distancefield)). <strong>The groundrobot_class object has a member of the distancefield_class called `vec_field_obj`</strong>.

Currently, the groundrobot_class supports two types of robots, the differential drive, and the skid-steer. See the image below:

![image](https://github.com/adrianomcr/vectorfield_stack/blob/main/ground_robot/images/groundrobots.png)

The available methods are listed below.



### Methods

#### `__init__(self, vr, kf, reverse_direction, flag_follow_obstacle, epsilon, switch_dist, d, move_backwards):`

Constructor method. It receives the following parameters:

- `vr` (`float`): norm of the velocity of the field
- `kf` (`float`): convergence gain of the vector field
- `reverse_direction` (`bool`): flag to make the vector field follow the curve in the opposite direction
- `flag_follow_obstacle` (`bool`): flag to enable/disable the obect follow feature
- `epsilon` (`float`): distance that a close obstacle will be followed
- `switch_dist` (`float`): distance from which the an obstacle start to be followed
- `d` (`float`): displacement of the control point used by the feedback-linearization controller
- `move_backwards` (`bool`): flag to set the robot to move backwards


#### `set_state(self, state):`

Method to set the state of the ground robot. The arument `state` is a list with the x position, y position, and yaw angle.



#### `set_closest(self, point):`

Method to set the 3D position (point) of the closest point that belongs to an obstacle. The third element can be set 0.



#### `get_vw(self):`

Method to get the x linear velocity (v<sub>x</sub>) and the z angular velocity (w<sub>z</sub>) of the robot. It assumes that the robot is in the state previously defined with the method `set_state`.


#### `feedback_linearization(self,f):`

Method that computes the x linear velocity and the z angular velocity for the robot given a reference velocity `f`. The velocity `f` is imposed to a point displaced at a distance `d` in front of the robot. It assumes that the robot is in the state previously defined with the method `set_state`.


#### `get_wheels_diferential(self,b):`

Method to compute the linear velocities of the right and left wheels of a differential drive robot with parameter `b` (see image above). It assumes that the robot is in the state previously defined with the method `set_state`. It returns a list with two elements, the right and left speeds.


#### `get_wheels_skidsteer(self, a, b):`

Method to compute the linear velocities of the right and left wheels of a skid-steer robot with parameters `a` and `b` (see image above). It assumes that the robot is in the state previously defined with the method `set_state`. It returns a list with two elements, the right and left speeds.











## differential_node

This ROS node has an implementation that can be used to control a differential drive robot. Basically, it subscribes to a topic to get the robot's pose, computes the vector field and the associate linear and angular speeds for the robot, and publishes the result in a command topic.

It can be tested with differential_sim node available in the [robotsim](../robotsim) package. See the package [examples](../examples) for instruction in how to launch this simulation.


### Parameters


- `vr` (`float`): norm of the velocity of the field
- `kf` (`float`): convergence gain of the vector field
- `reverse_direction` (`bool`): flag to make the vector field follow the curve in the opposite direction
- `d_feedback` (`float`): distance of the control point used in the feedback linearization
- `move_backwards` (`bool`): flag used to make the robot move backwards

- `flag_follow_obstacle` (`bool`): flag to enable/disable the obect follow feature
- `epsilon` (`float`): distance that a close obstacle will be followed
- `switch_dist` (`float`): distance from which the an obstacle start to be followed
- `obstacle_point_topic_name` (`string`): name of the topic in which the closest point of the obstacles are published

- `pose_topic_name` (`string`): name of the topic in which the robot's pose is published
- `pose_topic_type` (`string`): type of the topic in which the robot's pose is published (`TFMessage`, `Pose`, `Odometry`)
- `cmd_vel_topic_name` (`string`): name of the topic in which the node publishes command velocities
- `path_topic_name` (`string`): name of the topic in which the path (sequence of points) is published
- `path_equation_topic_name` (`string`): name of the topic in which the path (equation) is published


Check these parameters in the file [config/differential_params.yaml](config/differential_params.yaml).


### Topics


- `path_topic_name`  (message type: `distancefield_msgs/Path`): Subscribe to this topic to get a path represented as a sequence of points
- `path_equation_topic_name`  (message type: `distancefield_msgs/PathEq`): Subscribe to this topic to get a path represented by a parametric equation
- `obstacle_point_body_topic_name`  (message type: `std_msgs/Point`): Subscribe to this topic to get the closest colidable point written in the <strong>body</strong> reference frame.
- `pose_topic_type`  (message type: `tf2_msgs/TFMessage` or `geometry_msgs/Pose` or `nav_msgs/Odometry`): Subscribe to this topic to get the robot's pose


- `cmd_vel_topic_name`  (message type: `geometry_msgs/Twist`): Topic in which the velocity command is published (linear velocity)




## skidsteer_node

This ROS node has an implementation that can be used to control a skid-steering robot. Basically, it subscribes to a topic to get the robot's pose, computes the vector field, and publishes the right and left wheels speeds in a command topic. The difference between this node and the differential_node is that, now, the node actuates directli on the speed of the wheels.

It can be tested with skidsteer_sim node available in the [robotsim](../robotsim) package. See the package [examples](../examples) for instruction in how to launch this simulation.



### Parameters

The parameters of the skidsteer_node are the same as the paramaters of the differential_node above with the adition of the following two:

- `a` (`float`): horizontal distance from the robot's senter to the front (or back) wheels' shafts
- `b` (`float`): horizontal distance from the robot's senter to the right (or left) wheels' shafts

Check these parameters in the file [config/skidsteer_params.yaml](config/skidsteer_params.yaml).


### Topics

The topics of the skidsteer_node are almost the same as the topics of the differential_node above. The only change is the replacement of the topic `cmd_vel_topic_name` by the topic `cmd_wheels_topic_name`, which has the following structure:


- `cmd_wheels_topic_name`  (message type: `std_msgs/Float32MultiArray`): Topic in which the command speeds for the four wheels are published. The array has the order: front right, back right, back left, front left










