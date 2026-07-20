# ============================================================================
# 目标聚类"新出现"事件 (target_estimator_node -> mission_manager_node)
#
# 发布时机:某次 (label + 2D 位置) 联合匹配未命中任何已有 cluster,
#          创建一个新 cluster 的瞬间(每个 cluster_id 仅 publish 一次)。
# 接收方 mission_manager 拿此事件后,按 work_mode / skill state 决定是否
#   - publish DetectTarget(SEARCH_ONLY / SEARCH_STRIKE / attack_cmd.action=2)
#   - 启动跟踪(SEARCH_TRACK)
#   - 触发打击(SEARCH_STRIKE / attack_cmd.action=0)
# ============================================================================
uint64   cluster_id        # target_estimator 内部递增 id,稳定唯一
uint32   label_hash        # YoloDetection.label 的 hash(便于快速查重)
string   label             # YoloDetection.label 原值,如 "person"/"car"

float32  confidence        # 第一次出现时的置信度(后续 EMA 不会被 cluster_event 覆盖)
uint64   first_seen_us     # 第一次出现时间戳(微秒)
float64  ned_x             # cluster 中心 NWU x (米)
float64  ned_y             # cluster 中心 NWU y (米)
float64  ned_alt           # cluster 中心 z (米;地平面目标 = 0)
