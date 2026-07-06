# 工作模式(typed,与 Skill.msg 独立)
# 取代 std_msgs/String 模式字符串
uint8 IDLE = 0
uint8 TAKEOFF = 1
uint8 GATHER = 2
uint8 SEARCH_ONLY = 3
uint8 SEARCH_TRACK = 4
uint8 SEARCH_STRIKE = 5
uint8 RETURN = 6
uint8 LAND = 7
uint8 mode
