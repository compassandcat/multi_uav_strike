# 单个被检测目标(对应协议 0x2001 DEVICE_TARGETS body 中 DetectObject)
uint64   timestamp_us

# 目标类别 ID
uint32   label

# 置信度 0.0~1.0
float32  confidence

# 目标类型(见协议补充规范 §4.1 / §8.1)
# 1 = 被打击目标(永久保留)
# 2 = 暂存跟踪目标(永久保留)
# 3 = 普通搜索目标(支持时效自动清理)
uint8    target_type

# 无人机 / 目标 GPS(MSL 高度,米)
float64  dev_lat
float64  dev_lon
float64  dev_alt
float64  obj_lat
float64  obj_lon
float64  obj_alt

# 图像格式 0=none 1=JPEG(本版本仅做占位,length=0)
uint8    img_format
uint8[]  img_data
