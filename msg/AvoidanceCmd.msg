# uav_avoidance_node → waypoint_executor 的避障指令
# 压航线策略下:横向严禁,纵向与高度可调配
bool   lateral_blocked   # true = 横向不允许任何越线
float32 lateral_max      # 横向位移上限(米),默认 0.0
float32 vertical_bias    # 高度调整 bias(米/秒)
float32 speed_scale      # <1.0 减速,>1.0 加速
bool   reverse_allowed   # true 时允许 v_along < 0(回退)
