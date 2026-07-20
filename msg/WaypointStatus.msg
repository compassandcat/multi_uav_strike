# waypoint_executor 状态(typed,取代 std_msgs/String status)
uint8 PHASE_IDLE = 0
uint8 PHASE_ARRIVE = 1
uint8 PHASE_SKILL_AREA = 2
uint8 PHASE_HOLD = 3
uint8 PHASE_COMPLETE = 4

uint8  phase
string flow_id
string skill_id

# arrive_path 进度
uint32 arrive_idx
uint32 arrive_total

# skill_area_path 进度
uint32 skill_idx
uint32 skill_total
