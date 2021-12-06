# quad_robot


This package uses the distancefield package to support a controller for quadcopters. It assumes that the control commands are the thrust force and the angular body rates. It can be checked with the quadcopter available in the robotsim package. The main theory that supports this implementation is published in the following paper:

[2] Adriano M. C. Rezende, Vinicius M. Gonçalves, Arthur H. D. Nunes and Luciano C. A. Pimenta, **Robust quadcopter control with artificial vector fields,** 2020 IEEE International Conference on Robotics and Automation (ICRA), 2020, pp. 6381-6387, doi: 10.1109/ICRA40945.2020.9196605. Online: <https://ieeexplore.ieee.org/document/9196605>

