# distancefield

This package contains a vector field based navigation algorithm. The vector field is designed to make a robot follow a given curve. The method is currently implemented in 3 dimensions. The strategy is based on the minimum distance between the robot and the curve. The main theory that supports this implementation is published in the following paper:

[1] Adriano M. C. Rezende, Vinicius M. Goncalves and Luciano C. A. Pimenta, **Constructive Time-Varying Vector Fields for Robot Navigation,** in IEEE Transactions on Robotics, doi: 10.1109/TRO.2021.3093674. Online: <https://ieeexplore.ieee.org/document/9490231>

The implementation also incorporates the ability to deviate from detected obstacles.

The image below gives an idea of the vector field strategy that is implemented in this package. On each point, the field is computed with basis on the closest point on the curve (red dots). The field (blue arrows) is composed of two components. The red arrow is a convergent action while the green arrow is a circulation action.


![image](https://github.com/adrianomcr/vectorfield_stack/blob/main/distancefield/images/field_components.png)



## Additional ROS messages

This package has two additional ROS messages. They are used to represent paths in the form of a sequence of points or a parametric equation.

### distancefield_msgs/Path

This message represents a path in the form of a sequence of points. The message fields are below:

- `closed_path_flag` (`bool`):  Flag to indicate if the path is closed or not
- `insert_n_points` (`int64`): Number of points to be inserted in between each pair of points of the received path
- `filter_path_n_average` (`int64`): Number of points to use in the average filter (it is forced to be an odd number) - if 0 the path is not filtered
- `path` (`geometry_msgs/Polygon`): Points of the path. Uses a list of lists [[x<sub>0</sub>, y<sub>0</sub>, z<sub>0</sub>], [x<sub>1</sub>, y<sub>1</sub>, z<sub>1</sub>], [x<sub>2</sub>, y<sub>2</sub>, z<sub>2</sub>], ...]


### distancefield_msgs/PathEq

This message represents a path in the form of a parametric equation. The message fields are below:

- `closed_path_flag` (`bool`): Flag to indicate if the path is closed or not
- `u_i` (`float64`): Lower bound of the parameter set. Example: `0.0`
- `u_f` (`float64`): Upper bound of the parameter set. Example: `6.2832`
- `equation` (`string`): Python string with the path parametric equation. The equation is defined in the parameter `u` in the interval `[u_i, u_f]`. Example: `[[cos(u)],[sin(u)],[0.0]]`



## distancefield_class (python)

This class is an implementation of the Euclidean distance vector field.

The curve that will be followed can be defined in 2 ways; (i) by using a sequence of points; or (ii) by using a parametric equation. See the methods `set_path` and `set_equation`.

A simple way to use this class is to define an object (consctructor method `distancefield_class`), set the curve (`set_path` or `set_equation`) and compute the vector field at a given point of space (method `compute_field_at_p`).

The available methods are listed below.



### Methods

#### `__init__(self, v_r, k_f, reverse_direction, flag_follow_obstacle, epsilon, switch_dist):`

Constructor method. It receives the following parameters:

- `v_r` (`float`): norm of the velocity of the field
- `k_f` (`float`): convergence gain of the vector field
- `reverse_direction` (`bool`): flag to make the vector field follow the curve in the opposite direction
- `flag_follow_obstacle` (`bool`): flag to enable/disable an object contour feature
- `epsilon` (`float`): distance that a close obstacle will be followed
- `switch_dist` (`float`): distance from which an obstacle starts to be followed


#### `set_pos(self, pos):`

Method to set the 3D position (`pos`) of the robot.

#### `set_closest(self, point):`

Method to set the 3D position (point) of the closest point that belongs to an obstacle.

#### `is_ready(self):`

Method that returns true if the vector field is already set up.

#### `set_path(self, path, insert_n_points, filter_path_n_average, closed_path_flag):`

Method to set the path to be followed by using a sequence of points. It has the following arguments:

- `path[N][3]` (`float`): List of N 3D points representing the path to be followed.

- `insert_n_points (`int`)`: Number of points to be inserted in between two consecutive points.

- `filter_path_n_average (`int`)`: Number considered to run an average filter and make the path smoother.

- `closed_path_flag (`bool`)`: Flag to indicate if the path is closed or cyclic (`True`) or open (`False`).



#### `set_equation(self, equation_str, u_i, u_f, closed_path_flag, N):`

Method to set the path to be followed by using a parametric equation. It has the following arguments:

- `equation_str` (`string`): String with the parametric equation writtenaccording to python syntax. Example for an ellipse: `"[[2.0*cos(u)],[1.0*sin(u)],[0.0]]"`

- `u_i` (`float`): Lower baund of the parameter `u`. For the `equation_str` example: `u_i = 0`

- `u_f` (`float`): Lower baund of the parameter `u`. For the `equation_str` example: `u_f = 6.283185`

- `closed_path_flag` (`bool`): Flag to indicate if the path is closed or cyclic (`True`) or open (`False`).

- `N` (`int`): Number of samples of the curve that will be internally generated.


#### `sample_curve(self, u):`

Method that returns the value of the parametric equation of the curve in the parameter `u`.



#### `golden_search(self,pos,al,bl):`

Method that returns the point on the curve (represented as a parametric equation) that is the closest to the point `pos`. 
It implements a golden search method in the parametric interval `[al, bl]`.




<!-- #### `get_GH_follower(self, delta):`

To be removed -->



#### `field_from_equation(self, pos):`

Method that returns the vector field computed at the point `pos` by using a parametric equation.


#### `field_from_points(self, pos):`

Method that returns the vector field computed at the point `pos` by using a sequence of points.



#### `compute_field_at_p(self, pos):`

Method that returns the vector field computed at the point `pos`. It automatically detects the curve type (sequence of points or parametric equation) that was set.



#### `vec_field_path(self):`

Method that returns the vector field computed at the point previously defined with the method `set_pos`.




#### `@staticmethod get_norm(arr):`

Method to compute the norm of a vector represented as an array `arr`.


#### `@staticmethod insert_points(original_path, qty_to_insert, closed_path_flag):`

Method to returns a sequence of points `original_path` with `qty_to_insert` inserted points in between each pair of original points. The `closed_path_flag` is `True` if the path is closed.


#### `@staticmethod filter_path(original_path, filter_path_n_average, closed_path_flag):`

Method that returns the `original_path` smoothened with an average filter with parameter `filter_path_n_average`.




## simple_node.py

This ROS node has an implementation that can be used to control an integrator robot. It means that the robot is holonomic and responds to linear velocity commands. Basically, it subscribes to a topic to get the robot's position, computes the vector field, and publishes the velocity in a command topic.

It can be tested with integrator_sim node available in the [robotsim](../robotsim) package. See the package [examples](../examples) for instruction in how to launch this simulation.


### Parameters

Below is the list of ROS parameters that the node requires:

- `vr` (`float`): norm of the velocity of the field
- `kf` (`float`): convergence gain of the vector field
- `reverse_direction` (`bool`): flag to make the vector field follow the curve in the opposite direction

- `flag_follow_obstacle` (`bool`): flag to enable/disable the object follow feature
- `epsilon` (`float`): distance that a close obstacle will be followed
- `switch_dist` (`float`): distance from which the obstacle starts to be followed
- `obstacle_point_topic_name` (`string`): name of the topic in which the closest point of the obstacles are published

- `pose_topic_name` (`string`): name of the topic in which the robot's pose is published
- `pose_topic_type` (`string`): type of the topic in which the robot's pose is published (`TFMessage`, `Pose`, `Odometry`)
- `cmd_vel_topic_name` (`string`): name of the topic in which the node publishes command velocities
- `path_topic_name` (`string`): name of the topic in which the path (sequence of points) is published
- `path_equation_topic_name` (`string`): name of the topic in which the path (equation) is published

Check these parameters in the file [config/basic_params.yaml](config/basic_params.yaml).


### Topics

- `/path_topic_name`  (message type: `distancefield_msgs/Path`): Subscribe to this topic to get a path represented as a sequence of points
- `/path_equation_topic_name`  (message type: `distancefield_msgs/PathEq`): Subscribe to this topic to get a path represented by a parametric equation
- `/obstacle_point_topic_name`  (message type: `std_msgs/Point`): Subscribe to this topic to get the closest collidable point written in the <strong>world</strong> reference frame.
- `/pose_topic_type`  (message type: `tf2_msgs/TFMessage` or `geometry_msgs/Pose` or `nav_msgs/Odometry`): Subscribe to this topic to get the robot's position


- `cmd_vel_topic_name`  (message type: `geometry_msgs/Twist`): Topic in which the velocity command is published (linear velocity)



