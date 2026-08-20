# Livox custom point cloud message
std_msgs/Header header

uint64 timebase            # The time of first point
uint32 point_num           # Total number of points
uint8 lidar_id             # Lidar device id

uint8[3] rsvd              # Reserved

CustomPoint[] points       # Custom point array
