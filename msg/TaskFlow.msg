# ============================================================================
# 任务流:从地面站下发到机载的 skill 序列
# 仅描述 ROS 内部路由格式,不替 starling_bridge 业务帧
# ============================================================================
# flow_id:GS 唯一编号,机载回传 MissionState 时携带,用于 GS 端 task_flow 追踪
# issued_at_us:GS 下发时间戳(微秒,Unix epoch),若有数值则可用于机间协同时判定 task_flow 新旧
# device_id:
#   0 = 广播(全队都要处理,适合协同场景)
#   其它 = 指定 UAV device_id(mission_manager 比对 self_device_id_ 决定是否采纳)
# skills:按顺序执行的 skill 列表,典型顺序:
#   [Takeoff, Gather?, Search?, Track?, Attack?, Return?]
# ============================================================================
string   flow_id
uint64   issued_at_us
uint8    device_id
Skill[]  skills

# ============================================================================
# 关于优先级(详见 WorkMode.msg):
#   task_flow 整体优先级由 UAV 当前 work_mode 决定(粗粒度),与本消息字段无关
#   Skill.priority (细粒度) 在单机内多 skill 顺序执行时暂不生效
#   跨 UAV 协同仲裁(uav_avoidance / target_competition)依据:
#     - work_mode 优先级(主):SEARCH_STRIKE > SEARCH_TRACK > SEARCH_ONLY ≈ DENIED
#     - Skill.priority(辅):当 work_mode 相同时作为额外仲裁依据
#   GS 设计 task_flow 时建议:
#     - 通过 SET_WORKMODE 单独下发 work_mode 表达粗粒度优先级
#     - Skill.priority 字段可填 0(交给 work_mode 决定)或按业务需求细调
# ============================================================================
