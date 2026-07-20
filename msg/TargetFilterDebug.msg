# ============================================================================
# 跟踪目标位置/速度 滤波前后对比 (debug 用,2026-07-16)
#   同一时刻 publish:
#     - *_raw      = 滤波前的原始值
#     - *_filtered = One Euro Filter 后的值
#     - *_delta    = raw - filtered (≈ 噪声幅度,可直接看还在不在跳)
#   用法:
#     rostopic echo -n 5 /mission/target_filter_debug   # 位置
#     rostopic echo -n 5 /guidance/target_filter_debug  # 速度
# ============================================================================
std_msgs/Header header

# ---- 位置滤波对比 (NWU,米) ----
geometry_msgs/Point  pos_raw
geometry_msgs/Point  pos_filtered
geometry_msgs/Point  pos_delta

# ---- 速度指令滤波对比 (NWU,m/s;STRIKE 模式 vz≈0,TRACK 模式 yaw_rate 在 vel_z 中) ----
# 注:vel_filtered 是实际喂给 PX4 的速度;vel_raw 是 P 控制直出。
geometry_msgs/Vector3 vel_raw
geometry_msgs/Vector3 vel_filtered
geometry_msgs/Vector3 vel_delta

# ---- 元信息 ----
uint64  cluster_id    # 当前锁定的 cluster_id(变 → target_estimator 重分配了)
bool    is_locked     # 当前是否锁定(is_locked=false 时所有 filtered 值无意义)
string  source        # "position" / "velocity",用来区分两个 node 发的同类型 msg