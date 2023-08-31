# Cloud Info
Header header 

int32[] startRingIndex # 某条ring上第一个可以计算曲率的点
int32[] endRingIndex # 某条ring上最后一个可以计算曲率的点

int32[]  pointColInd # point column index in range image 列序号
float32[] pointRange # point range 像素值

int64 imuAvailable #imu旋转测量是否对齐到lidar，对齐说明imu可以用
int64 odomAvailable #当前帧最近的两个相邻帧之间的位姿变换可用

# Attitude for LOAM initialization
float32 imuRollInit
float32 imuPitchInit
float32 imuYawInit

# Initial guess from imu pre-integration
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed 原始去畸变点云
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature

# 3rd party messages
sensor_msgs/PointCloud2 key_frame_cloud
sensor_msgs/PointCloud2 key_frame_color
sensor_msgs/PointCloud2 key_frame_poses
sensor_msgs/PointCloud2 key_frame_map
