# 单条 skill 定义 — 任务流中的原子单元
# 与 WorkMode 独立(WorkMode 由 GS 单独下发)
# skill_type 枚举(参考 TZS v4.0 §9.2.6):
#   100 = Takeoff(弹射 or 地面)
#   101 = Gather(集结)
#   102 = Search(搜索)
#   103 = Return(返航)
#   104 = Relay(中继,本版本不实现)
#   105 = Attack(多目标打击)
string   skill_id
uint8    skill_type

# 仅 skill_type=100 时有效
# 0 = 地面起飞,1 = 弹射起飞
# 协议补充规范 §1.1 要求弹射起飞后置必须有 skill(由 GS 端强校验)
uint8    takeoff_subtype

# 仅 skill_type=100 时有效
# 起飞目标高度(米，相对 PX4 home altitude,即目标 alt = ref_alt + takeoff_altitude)
# 0 表示使用 mission_manager 私有 param ~takeoff_altitude 兜底
# skill_type != 100 时此字段忽略
float32  takeoff_altitude

# 数值越大优先级越高,用于机间避让
uint8    priority

# 进场巡航速度(米/秒)
float32  cruise_speed

# 任务区执行速度(米/秒)
# skill_type=105 时,此值也是 WP1→WP2 的打击速度
float32  task_speed

# 多机协同模式
# 0 = 默认,1 = 顺序惯序,2 = 编队
uint8    formation

# 进场航点(沿此段飞行至任务区,不触发任务执行)
nav_msgs/Path arrive_path

# 任务区航点(完成首个点 = 准入门;末点 = 准出门)
nav_msgs/Path skill_area_path

# 兜底参数(预留 JSON 字符串,具体字段后续追加)
string   params_json
