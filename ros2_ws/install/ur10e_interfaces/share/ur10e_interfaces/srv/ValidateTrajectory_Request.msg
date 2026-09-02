# ValidateTrajectory.srv

# 1. REQUEST (Input sent to the service)
# Desired EE trajectory path (flattened matrix or list of poses)
float64[] ee_positions_x
float64[] ee_positions_y
float64[] ee_positions_z
float64[] ee_quat
float64[] sim_time

