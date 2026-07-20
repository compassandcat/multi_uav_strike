# ============================================================================
# 单个 cluster 在某时刻的快照(由 ClusterState[] 持有)
#
# target_estimator_node 在 10Hz 发布所有 alive cluster 的状态,
# 用于可视化(RViz Marker)以及"丢了一次 cluster_event"的恢复场景。
# ============================================================================
uint64  cluster_id
string  label
uint32  label_hash
float64 ned_x
float64 ned_y
float64 ned_alt
float32 confidence
uint64  last_seen_us     # 上次刷新(微秒)
uint8[] img_data        # 最新一帧 YoloDetection.img_data(可选,空 = 本帧没图)
uint8   img_format      # 1 = JPEG(与 DetectTarget.img_format 对齐)
