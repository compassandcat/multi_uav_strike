# TRACKING_STATE(协议 43000 / TZS §6)上报
# 协议只定义两个状态值:
#   0 = 未跟踪
#   1 = 跟踪中
# GCS 端要"连续接收"才会高亮 UI、闪烁提示 — 故机载端持续发:
#   跟踪中: 5Hz 发 state=1
#   未跟踪: 1Hz 发 state=0(保活 + 让 GCS 知道 UAV 在线)
uint8   state
uint8   flag
float32 target_dist
float64 target_lat
float64 target_lon
float64 target_alt
