# ============================================================================
# 工作模式(typed) — 表达"在任务区域内做什么"
# 由 GS 通过 SET_WORKMODE 单独下发
#
# 设计原则(2026-07 协议精简):
#   work_mode 只表达当前任务类型,不再承载 TAKEOFF/GATHER/RETURN/LAND 等
#   阶段语义 — 这些由 MissionPhase 表达
#   本枚举当前 5 个值:
#     - IDLE                (保留,UAV 空闲,等待新指令)
#     - SEARCH_ONLY/TRACK/STRIKE  (3 个当前已实现的任务模式)
#     - DENIED_ENV_FLIGHT   (预留,GPS 拒止/强对抗场景,未实现)
# ============================================================================
uint8 IDLE = 0
uint8 TAKEOFF = 1
uint8 GATHER = 2
uint8 SEARCH_ONLY = 3
uint8 SEARCH_TRACK = 4
uint8 SEARCH_STRIKE = 5
uint8 RETURN = 6
uint8 LAND = 7
uint8 DENIED_ENV_FLIGHT = 8
uint8 mode

# ============================================================================
# 任务优先级 — 由 work_mode 决定(粗粒度)
#
# 总优先级(用于多机协同/uav_avoidance/target_competition 仲裁):
#   SEARCH_STRIKE      = 最高级 (时间敏感,锁定后其他 UAV 应让出射界)
#   SEARCH_TRACK       = 次高   (锁定阶段,目标已识别待命)
#   SEARCH_ONLY        = 低     (扫描,可主动避让)
#   DENIED_ENV_FLIGHT  = 低     (等同 SEARCH_ONLY,低优先级,后续实现)
#   IDLE               = N/A   (无活动任务,通常不参与协同仲裁)
#
# 与 MissionPhase 的协作:
#   - work_mode 决定 UAV 当前的"任务类型"和"粗粒度优先级"
#   - MissionPhase 决定 UAV 当前的"执行阶段"
#     (GROUND_IDLE / TAKING_OFF / HOVERING / WAYPOINT_FOLLOW /
#      GUIDANCE_TRACK / GUIDANCE_STRIKE / RETURNING / COMPLETE /
#      FAILED / HOLDING)
#   - 两者独立,GS 可单独下发 SET_WORKMODE 而不影响 phase
#   - 协同 UAV 看到的"高优先级 UAV"判定 =
#     (work_mode 优先级高) AND (phase 处于活动阶段)
#     PHASE_GROUND_IDLE/PHASE_HOLDING/PHASE_RETURNING 时 UAV 虽 idle,仍可能因
#     历史任务享有路权(避免冲突)
#
# 与 Skill.priority 的协作(细粒度):
#   - Skill.priority 主要供未来"多 UAV 共用一条 task_flow"时仲裁
#   - 当前 task_flow 是单 UAV 顺序执行,Skill.priority 在单机内无作用
#   - 跨 UAV 仲裁以 work_mode 优先级为准
#
# 多机协同典型规则:
#   - 同一空域多 UAV 路径冲突:
#       高优先级 (SEARCH_STRIKE / SEARCH_TRACK) UAV 保持原路径
#       低优先级 (SEARCH_ONLY / DENIED_ENV_FLIGHT) UAV 主动避让
#   - 目标竞争:多 UAV 同时识别同一目标时,work_mode 优先级高的 UAV 获得锁定权,
#     低优先级 UAV 切回 SEARCH_ONLY 继续扫描
# ============================================================================