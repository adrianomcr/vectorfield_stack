# PathEq message - represents a path with an equation

Header header #ROS header message

bool closed_path_flag #Flag to indicate if the path is closed or not
float64 u_i #Lower bound of the parameter set
float64 u_f #Upper bound of the parameter set
string equation #Python string with the path parametric equation. The equation is defined in the parameter u in the interval [u_i, u_f].


