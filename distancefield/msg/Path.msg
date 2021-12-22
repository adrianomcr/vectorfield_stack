# Path message - represents a path with a sequence of points

Header header #ROS header message

bool closed_path_flag #Flag to indicate if the path is closed or not
int64 insert_n_points #Number of points to be inserted in between each pais of points of the received path
int64 filter_path_n_average #Number of points to use in the average filter (it is forced to be an odd number) - if 0 the path is not filtered
geometry_msgs/Polygon path  # Points of the path. Uses a list of lists [[...],[...],[...]]
