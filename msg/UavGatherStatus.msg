# 集结(skill_type=101)等待多机状态
# arrived_state:
#   0 = 在途
#   1 = 已到集结区
#   2 = 超时完成(到点后等不到队友,自动推进)
string  sn
uint8   arrived_state
float64 lat
float64 lon
float64 alt
uint32  priority
