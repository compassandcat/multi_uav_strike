# 多机心跳(对应协议补充规范 §6)
# comm_node 按距离动态调频:基础帧率 / 5Hz (≤40m) / 20Hz (≤20m)
string  sn
uint8   work_mode
int32   skill_type
string  skill_id
uint64  timestamp_us

# GPS(MSL 高度,米)
float64 lat
float64 lon
float64 alt

# 偏航角(弧度,NED 顺时针,0=N)
float32 yaw
