# TRACKING_STATE(协议 43000)上报
# 仅在 SEARCH_TRACK 模式下,识别到目标进入跟踪状态时发
uint8   state
# state 编码:
#   0 = 未开始
#   1 = 跟踪中
#   2 = 已识别未跟踪(等待地面站确认)
#   3 = 完成(攻击完成或主动退出)
#   4 = 取消(忽略 / 暂存)
uint8   flag
float32 target_dist
float64 target_lat
float64 target_lon
float64 target_alt
