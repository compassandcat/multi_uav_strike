# 替换 /detection/yolo_result 的 typed 结果
# 来源:detection_simulator_node 或真实感知模块
uint32  frame_seq
uint64  stamp_us
string  label

# 置信度 0.0~1.0
float32 confidence

# 归一化 bbox 坐标(0.0~1.0,屏幕坐标系)
# 当 is_in_fov=false 时填 0
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# 是否在相机矩形 FOV 内(关键过滤标志)
# 见 协议补充规范 §4.3 目标回传时机
bool    is_in_fov

# 原始 JPEG 字节流(由 gimbal_simulator_node 嵌入一张固定图,真实 gimbal 接入后
# 替换为相机抓拍帧;starling_bridge 据此打包 DEVICE_TARGETS 0x2001 上行 GS)
uint8[] img_data
