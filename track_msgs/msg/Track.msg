# tracking id
uint64 id 

# object label
string label

# tracked position
geometry_msgs/Point position

# velocity
geometry_msgs/Vector3 velocity

# covariance of position and velocity
float64[] covariance

# appearance feature if exist
float64[] feature