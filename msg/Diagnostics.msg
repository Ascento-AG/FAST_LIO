# Fast-LIO per-scan diagnostics
std_msgs/Header header

# Point pipeline
uint32 raw_points           # After blind + point_filter_num (feats_undistort)
uint32 downsampled_points   # After voxel filter (feats_down_size)
uint32 effective_points     # After plane fit + residual check (effct_feat_num)

# Scan matching quality
float64 mean_residual       # Average point-to-plane distance (res_mean_last)

# IEKF convergence
uint8 iekf_iterations       # Actual iterations used (1-based)
bool  iekf_converged        # True = converged, false = hit max_iteration

# Timing (seconds)
float64 time_imu_prop       # IMU propagation + undistortion
float64 time_downsample     # Voxel filter
float64 time_match          # KNN search + plane fitting
float64 time_solve          # IEKF solve (Jacobian + Kalman gain)
float64 time_map_update     # ikd-tree insertion
float64 time_total          # Full scan processing

# IMU health
uint32 imu_msgs_used        # Number of IMU messages consumed for this scan
float64 imu_dt_max          # Max gap between consecutive IMU messages (seconds)

# Map state
uint32 kdtree_size          # Current ikd-tree node count
uint32 map_points_added     # Points added to map this scan
