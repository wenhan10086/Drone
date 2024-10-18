std_msgs/Header header

## 机载电脑是否连接上飞控，true已连接，false则不是
bool connected
## 是否解锁，true为已解锁，false则不是
bool armed
## PX4飞控当前飞行模式
string mode 
## GPS连接状态
int8 gps_status
#  STATUS_NO_FIX =  -1    # unable to fix position
#  STATUS_FIX =      0    # unaugmented fix
#  STATUS_SBAS_FIX = 1    # with satellite-based augmentation
#  STATUS_GBAS_FIX = 2    # with ground-based augmentation
## GPS测量的全局位置
float64 latitude          # Latitude [degrees]. Positive is north of equator; negative is south.
float64 longitude        # Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 altitude          # Altitude [m]. Positive is above the WGS 84 ellipsoid(quiet NaN if no altitude is available).

## 系统启动时间
float32 time_from_start             ## [s]

## 无人机状态量：位置、速度、姿态
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s] ENU Frame
float32[3] velocity_body            ## [m/s] body Frame
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数
float32[3] attitude_rate            ## [rad/s]