# vectorfield_stack
Navigation control algorithms based on artificial vector fields


![image](https://github.com/adrianomcr/vectorfield_stack/blob/main/images/field_illustration.png)


This ROS stack provides packages to perform the navigation control of a robot by using an artificial vector field strategy. The image above illustrates the method. Given a desired reference path, the vector field guides the robot towards the curve, which is then followed. The core of the navigation control is a vector field based on the Euclidean distance function, specific packages import the field and perform the control of different robot types.



## Available packages


### distancefield

This package contains a vector field based navigation algorithm. The vector field is designed to make a robot follow a given curve. The method is currently implemented in 3 dimensions. The strategy is based on the minimum distance between the robot and the curve. The main theory that supports this implementation is published in the following paper:

[1] Adriano M. C. Rezende, Vinicius M. Goncalves and Luciano C. A. Pimenta, **Constructive Time-Varying Vector Fields for Robot Navigation,** in IEEE Transactions on Robotics, doi: 10.1109/TRO.2021.3093674. Online: <https://ieeexplore.ieee.org/document/9490231>

The implementation also incorporates the ability to deviate from detected obstacles.

More details on the distancefield package [here](distancefield).



### robotsim

This package provides simple implementations of several robots. The objective of this package is to enable simple examples of the use of the control packages of this stack.

The following robot types are available:

- Holonomic 3D (used to simulate distancefield package alone)

- Differential drive (used to simulate ground_robot package)

- Skid-steering (used to simulate ground_robot package)

- Quadcopter (used to simulate quad_robot package)


The image below shows some of the simulated robots:

![image](https://github.com/adrianomcr/vectorfield_stack/blob/main/images/sim_robots.png)

More details on the robotsim package [here](robotsim).



### ground_robot


This package uses the distancefield package to support a controller for ground robots. It can be used to control a diferential drive or a skid-steering robot. Two types of input controls are available, (i) linear and angular speeds, and (ii) wheels speeds. It can be checked with the differential and skid-steering robots available in the robotsim package.


More details on the ground_robot package [here](ground_robot).




### quad_robot


This package uses the distancefield package to support a controller for quadcopters. It assumes that the control commands are the thrust force and the angular body rates. It can be checked with the quadcopter available in the robotsim package. The main theory that supports this implementation is published in the following paper:

[2] Adriano M. C. Rezende, Vinicius M. Gonçalves, Arthur H. D. Nunes and Luciano C. A. Pimenta, **Robust quadcopter control with artificial vector fields,** 2020 IEEE International Conference on Robotics and Automation (ICRA), 2020, pp. 6381-6387, doi: 10.1109/ICRA40945.2020.9196605. Online: <https://ieeexplore.ieee.org/document/9196605>

More details on the ground_robot package [here](quad_robot).


### examples

This package contains some files (launch, yaml, ...) that exemplify how the control codes can be used. The examples use the simulators provided in the package robotsim.

More details on the examples package [here](examples).



<!-- ## External links -->