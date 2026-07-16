// ============================================================================
// target_estimator_node — 多实例目标聚类 (label + 2D 位置, bbox 反投影)
//
// 设计变更 (2026-07 多目标改造):
//   - 订阅 detection/yolo_results (YoloDetections):每帧可能多条检测,
//     每条独立做 per-bbox 反投影 → 地面点。原先共享 LOS (current_los_) 的反投影
//     会把所有检测塌缩到同一地面点,必须改成 bbox 中心 + camera 几何的反投影。
//   - 实例聚类:删除 label→cluster_id 单索引(label_to_id_)。改为遍历 clusters_,
//     找 label 相同 + 2D 距离 ≤ merge_radius_m_ 的最近 cluster 合并;
//     无匹配 → 新建 cluster(同类别多实例可共存)。
//   - 主目标输出:订阅 mission/primary_target (UInt64)。statesTimerCb 里
//     target_estimated_pose / gps 选择 primary_cluster_id_ 对应 cluster;
//     0 或找不到 → 回退最高置信度 cluster。
//   - 相机几何参数 (image_width/height/fov_h/fov_v) 与 gimbal 同步,
//     订阅 gimbal_pose 拿当前 camera world quat,做射线-地面求交。
//
// 输入:
//   detection/yolo_results    (YoloDetections, batch)        ← 新增
//   gimbal_pose               (PoseStamped, NWU, body_cam quat) ← 新增
//   /<uav_pose_topic>         (PoseStamped, NWU)
//   /mavros/global_position/global (NavSatFix)
//   mission/primary_target    (UInt64, cluster_id)            ← 新增
//
// 输出:
//   cluster_event             (ClusterEvent)— 新 cluster 唯一事件
//   cluster_states            (ClusterState, 10Hz)— 所有 alive cluster 快照
//   target_estimated_marker   (MarkerArray, 10Hz)— RViz 可视化
//   target_estimated_gps      (NavSatFix, 10Hz)— primary cluster 的 GPS
//   target_estimated_pose     (PoseStamped, 10Hz)— primary cluster 的 NWU 坐标
// ============================================================================
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <multi_uav_strike/YoloDetection.h>
#include <multi_uav_strike/YoloDetections.h>
#include <multi_uav_strike/ClusterEvent.h>
#include <multi_uav_strike/ClusterTarget.h>
#include <multi_uav_strike/ClusterState.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>

class TargetClusterer {
public:
    // ===== 单个目标集群 =====
    struct TargetCluster {
        uint64_t  id              = 0;
        std::string label;                  // 原始 label,string equality 区分
        uint32_t  label_hash      = 0;
        double    ned_x           = 0.0;     // 中心 NWU x (米)
        double    ned_y           = 0.0;     // 中心 NWU y (米)
        double    ned_alt         = 0.0;     // 中心 z (米)
        float     best_confidence = 0.0f;
        uint64_t  first_seen_us   = 0;
        uint64_t  last_seen_us    = 0;
        // 最新一帧 YoloDetection.img_data(可为 JPEG 字节,空 = 当前没图)
        std::vector<uint8_t> img_data;
        uint8_t              img_format = 0;  // 1 = JPEG(与 DetectTarget.img_format 对齐)

        // === 2026-07-16: Plan A 像素空间 EMA (scale-invariant) ===
        //   关键改动:UAV pose 抖动 → bbox 像素抖动 → 投影回世界时被距离 10-30m 放大成
        //   米级跳动。改在像素空间先 EMA,再投影,1px 抖动不再被距离放大。
        //   raw_cx/cy 与 ema_cx/cy 都在 [0, image_width/height] 范围。
        double    ema_cx          = 0.0;     // 平滑后 bbox 中心 x (像素)
        double    ema_cy          = 0.0;     // 平滑后 bbox 中心 y (像素)
        bool      ema_pixel_valid = false;   // 首次检测时初始化为原始像素

        // === 2026-07-16: 多次命中才晋升为正式 cluster ===
        //   hit_count: 累计被 detection 命中的次数(含本次)
        //   is_promoted: 达到 min_hit_count_ 后才置 true,之后才:
        //                - 发 cluster_event(让 mission_manager 决定上报/跟踪)
        //                - 进 cluster_states 10Hz 快照(RViz 显示)
        //                - 用 cluster_ttl_sec_ 长 TTL(确认存在的目标)
        //   未晋升的 tentative 用更短的 tentative_ttl_sec_ 过期丢弃。
        uint32_t  hit_count       = 0;
        bool      is_promoted     = false;
    };

    TargetClusterer() {
        ros::NodeHandle p("~");
        // 仿真/真机切换
        p.param<bool>("use_sim", use_sim_, true);
        p.param<std::string>("uav_pose_topic", uav_pose_topic_,
                             use_sim_ ? "quad/pose" : "mavros/local_position/pose");
        // 聚类参数
        p.param<double>("merge_radius_m", merge_radius_m_, 10.0);
        p.param<double>("cluster_ttl_sec", cluster_ttl_sec_, 60.0);
        // === 2026-07-16: 多次命中门禁 (兼筛选) ===
        //   同一 cluster (label + merge_radius_m 范围内) 被检测到 hit_count_ ≥
        //   min_hit_count_ 才晋升为正式目标。同位置多次命中 = 真目标的强信号;
        //   单次或零星命中 = YOLO 误检 / 飞过飞回的残影,直接被门禁过滤掉。
        //   经验值:10Hz yolo → 20 hits ≈ 2s 持续命中,误检基本进不来;
        //          真目标在视野停留 3~5s 时稳定晋升。
        p.param<int>("min_hit_count", min_hit_count_, 20);
        p.param<double>("tentative_ttl_sec", tentative_ttl_sec_, 1.0);
        // === 2026-07-16: Plan A — ema_alpha 改为像素空间平滑 ===
        //   0.6 在 10Hz 下对低频噪声增益≈1,基本不滤波;0.2 ≈ 时间常数 0.4s,
        //   1Hz bbox 抖动衰减 ~80%。注意这是像素域的 EMA,与距离无关,所以
        //   0.2 对近/远目标的滞后是一致的(都是 0.4s 时间常数)。
        //   如果目标是高速车辆,可调回 0.4~0.5;如果还看到残影,降到 0.10。
        p.param<double>("ema_alpha",       ema_alpha_,       0.05);
        p.param<double>("target_z_prior",  target_z_prior_,  0.0);
        // 发布周期
        p.param<double>("states_publish_freq", states_publish_freq_, 10.0);
        p.param<double>("cleanup_freq",        cleanup_freq_,        1.0);
        // marker 颜色
        p.param<double>("est_marker_r", est_marker_r_, 1.0);
        p.param<double>("est_marker_g", est_marker_g_, 1.0);
        p.param<double>("est_marker_b", est_marker_b_, 0.0);
        p.param<double>("est_marker_a", est_marker_a_, 1.0);

        // 相机几何参数(必须与 gimbal_simulator 同步)
        p.param<int>("image_width",  image_width_,  1920);
        p.param<int>("image_height", image_height_, 1080);
        p.param<double>("fov_h_deg", fov_h_deg_, 66.0);
        p.param<double>("fov_v_deg", fov_v_deg_, 52.0);
        computeFocalLengths();

        // 订阅
        yolo_batch_sub_     = nh_.subscribe("detection/yolo_results", 10,
                                            &TargetClusterer::yoloBatchCallback, this);
        gimbal_pose_sub_    = nh_.subscribe("gimbal_pose", 10,
                                            &TargetClusterer::gimbalPoseCallback, this);
        // === 2026-07-16: 改为订阅稳像 camera/pose 作为反投影唯一真源 ===
        // gimbal_simulator_node.cpp::publishCameraPose 现在发布的是稳像世界 quat,
        // 与 forward projection 完全一致,这里直接拿来用。
        camera_pose_sub_    = nh_.subscribe("camera/pose", 10,
                                            &TargetClusterer::cameraPoseCallback, this);
        uav_pose_sub_       = nh_.subscribe(uav_pose_topic_, 10,
                                            &TargetClusterer::uavPoseCallback, this);
        self_gps_sub_       = nh_.subscribe("mavros/global_position/global", 10,
                                            &TargetClusterer::selfGpsCallback, this);
        primary_target_sub_ = nh_.subscribe("mission/primary_target", 10,
                                            &TargetClusterer::primaryTargetCallback, this);

        // 发布
        cluster_event_pub_      = nh_.advertise<multi_uav_strike::ClusterEvent>(
                                    "cluster_event", 10);
        cluster_states_pub_     = nh_.advertise<multi_uav_strike::ClusterState>(
                                    "cluster_states", 10);
        target_est_pose_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(
                                    "target_estimated_pose", 10);
        target_est_marker_pub_  = nh_.advertise<visualization_msgs::MarkerArray>(
                                    "target_estimated_marker", 10);
        target_est_gps_pub_     = nh_.advertise<sensor_msgs::NavSatFix>(
                                    "target_estimated_gps", 10);

        // 周期任务
        states_timer_ = nh_.createTimer(ros::Duration(1.0 / states_publish_freq_),
                                        &TargetClusterer::statesTimerCb, this);
        cleanup_timer_= nh_.createTimer(ros::Duration(1.0 / cleanup_freq_),
                                        &TargetClusterer::cleanupTimerCb, this);

        ROS_INFO("[TargetClusterer] initialized. merge_radius=%.1fm ttl=%.1fs ema_alpha=%.2f "
                 "z_prior=%.1f image=%dx%d fov=(%.1f,%.1f)deg focal=(%.1f,%.1f)",
                 merge_radius_m_, cluster_ttl_sec_, ema_alpha_, target_z_prior_,
                 image_width_, image_height_, fov_h_deg_,
                 (fov_v_deg_ > 0.0 ? fov_v_deg_ : fov_v_rad_ * 180.0 / M_PI),
                 focal_x_, focal_y_);
    }

    // ============ 回调 ============

    // 每帧 batch YoloDetections:每条 detection 独立反投影 + 聚类
    void yoloBatchCallback(const multi_uav_strike::YoloDetections::ConstPtr& msg) {
        // === 2026-07-16: 改用稳像 camera/pose 作为反投影真源 ===
        if (!is_uav_pose_received_ || !is_camera_pose_received_) return;
        for (const auto& det : msg->detections) {
            processSingleDetection(det);
        }
    }

    // 单条 detection → 反投影地面点 → 命中已有 cluster (label + 2D 距离) 或新建
    void processSingleDetection(const multi_uav_strike::YoloDetection& det) {
        if (!det.is_in_fov) return;
        if (!is_uav_pose_received_ || !is_camera_pose_received_) return;

        // === 2026-07-16: Plan A 像素空间 EMA ===
        //   先取原始 bbox 中心像素,后面再做 EMA → 投影
        const double raw_cx = (det.x_min + det.x_max) * 0.5 * image_width_;
        const double raw_cy = (det.y_min + det.y_max) * 0.5 * image_height_;

        std::lock_guard<std::mutex> g(clusters_mtx_);
        const std::string& label = det.label;

        // 实例聚类:把当前帧的 raw 像素先投影成世界点(临时),与已有 cluster 的
        //   上一次稳定位置 c.ned_x/ned_y 算 2D 距离 → 选最近且 <merge_radius_m_ 的。
        //   注:c.ned_x/ned_y 是上一帧像素 EMA 投影后的稳定值,用做匹配参考点不会
        //       跟随单帧噪声漂移。
        double cand_tx = 0.0, cand_ty = 0.0, cand_tz = 0.0;
        const bool cand_valid = projectPixelToGround(raw_cx, raw_cy, cand_tx, cand_ty, cand_tz);
        const double r2 = merge_radius_m_ * merge_radius_m_;
        uint64_t nearest_id = 0;
        double   best_dist_sq = r2;
        if (cand_valid) {
            for (const auto& kv : clusters_) {
                const TargetCluster& c = kv.second;
                if (c.label != label) continue;  // 不同 label 不合并
                double dx = cand_tx - c.ned_x;
                double dy = cand_ty - c.ned_y;
                double d2 = dx*dx + dy*dy;
                if (d2 < best_dist_sq) {
                    best_dist_sq = d2;
                    nearest_id = c.id;
                }
            }
        }

        if (nearest_id != 0) {
            // 命中已有 cluster:先在像素空间 EMA,再投影回世界
            TargetCluster& c = clusters_[nearest_id];
            if (!c.ema_pixel_valid) {
                // 首帧 EMA 初始化
                c.ema_cx = raw_cx;
                c.ema_cy = raw_cy;
                c.ema_pixel_valid = true;
            } else {
                c.ema_cx = ema_alpha_ * raw_cx + (1.0 - ema_alpha_) * c.ema_cx;
                c.ema_cy = ema_alpha_ * raw_cy + (1.0 - ema_alpha_) * c.ema_cy;
            }
            // 投影平滑后的像素 → 世界
            double tx = 0.0, ty = 0.0, tz = target_z_prior_;
            if (!projectPixelToGround(c.ema_cx, c.ema_cy, tx, ty, tz)) return;
            // 平滑后的像素投影已经稳定,直接赋值(不再做世界坐标 EMA,避免双重平滑引入滞后)
            c.ned_x = tx;
            c.ned_y = ty;
            c.ned_alt = tz;
            c.best_confidence = std::max(c.best_confidence, det.confidence);
            c.last_seen_us = det.stamp_us;
            // === 2026-07-16: 命中计数 + 晋升门禁 ===
            c.hit_count++;
            if (!c.is_promoted && static_cast<int>(c.hit_count) >= min_hit_count_) {
                c.is_promoted = true;
                emitClusterEvent(c);
            }
            if (!det.img_data.empty()) {
                c.img_data   = det.img_data;
                c.img_format = 1;  // JPEG(与 gimbal_simulator 输出一致)
            }
            return;
        }

        // 新建 cluster:首帧像素直接作为 EMA 初值,但暂不晋升、不发 event
        TargetCluster nc;
        nc.id              = ++next_cluster_id_;
        nc.label           = label;
        nc.label_hash      = hashLabel(label);
        nc.ema_cx          = raw_cx;
        nc.ema_cy          = raw_cy;
        nc.ema_pixel_valid = true;
        if (!projectPixelToGround(nc.ema_cx, nc.ema_cy, nc.ned_x, nc.ned_y, nc.ned_alt)) {
            return;  // 投影失败(射线水平或朝后)就不建 cluster
        }
        nc.best_confidence = det.confidence;
        nc.first_seen_us   = det.stamp_us;
        nc.last_seen_us    = det.stamp_us;
        nc.img_data        = det.img_data;
        nc.img_format      = det.img_data.empty() ? 0 : 1;
        // === 2026-07-16: 首次进入 tentative 态 ===
        nc.hit_count       = 1;
        nc.is_promoted     = false;
        clusters_[nc.id]   = nc;

        ROS_INFO_THROTTLE(1.0, "[TargetClusterer] NEW tentative id=%lu label=%s "
                              "(ned=%.1f,%.1f,%.1f conf=%.2f hit=%u/%d, total=%zu)",
                           nc.id, nc.label.c_str(), nc.ned_x, nc.ned_y, nc.ned_alt,
                           nc.best_confidence, nc.hit_count, min_hit_count_,
                           clusters_.size());

        // 若 min_hit_count_ ≤ 1 (关闭门禁),立即晋升并发 event
        if (min_hit_count_ <= 1) {
            clusters_[nc.id].is_promoted = true;
            emitClusterEvent(clusters_[nc.id]);
        }
    }

    // === 2026-07-16: 抽出 cluster_event 发送逻辑 ===
    //   同一 cluster 整个生命周期只发一次(首次晋升时),
    //   mission_manager 据此决定上报/跟踪/打击。
    void emitClusterEvent(const TargetCluster& c) {
        multi_uav_strike::ClusterEvent ev;
        ev.cluster_id    = c.id;
        ev.label_hash    = c.label_hash;
        ev.label         = c.label;
        ev.confidence    = c.best_confidence;
        ev.first_seen_us = c.first_seen_us;
        ev.ned_x         = c.ned_x;
        ev.ned_y         = c.ned_y;
        ev.ned_alt       = c.ned_alt;
        cluster_event_pub_.publish(ev);

        ROS_INFO("[TargetClusterer] PROMOTE cluster id=%lu label=%s "
                 "(ned=%.1f,%.1f,%.1f conf=%.2f hit=%u)",
                 c.id, c.label.c_str(), c.ned_x, c.ned_y, c.ned_alt,
                 c.best_confidence, c.hit_count);
    }

    // 从 gimbal_pose 提取当前云台俯仰(q = setRPY(0, pitch, 0))
    void gimbalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 注意: gimbal_pose 的 orientation 已经是 uav_quat * body_cam_quat 形式
        // (见 gimbal_simulator_node.cpp 中 publishGimbalPose 的实现: 直接用
        //  setRPY(0, current_gimbal_pitch_, 0) 写入 orientation, 与 publishCameraPose
        //  不同; 此处采用与 forward projection 一致的 +pitch 约定)。
        // 我们需要从 orientation 反解 pitch 用于构造 R_world_cam。
        tf::Quaternion q(msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z,
                         msg->pose.orientation.w);
        double r, p, y;
        tf::Matrix3x3(q).getRPY(r, p, y);
        current_gimbal_pitch_ = p;
        is_gimbal_pose_received_ = true;
    }

    // === 2026-07-16: 接收稳像 camera/pose 作为反投影唯一真源 ===
    // 不再做 uav_quat * setRPY(pitch) 重建,直接用 gimbal 算好的稳像世界 quat,
    // 保证前向/反向投影用同一个 R_world_cam,且完全不依赖 UAV roll/机体 pitch。
    void cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_camera_world_quat_ = Eigen::Quaterniond(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        current_camera_world_quat_.normalize();
        is_camera_pose_received_ = true;
    }

    void primaryTargetCallback(const std_msgs::UInt64::ConstPtr& msg) {
        primary_cluster_id_ = msg->data;
        ROS_INFO_THROTTLE(2.0, "[TargetClusterer] primary_target → cluster_id=%lu",
                          primary_cluster_id_);
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (use_sim_) {
            current_uav_pose_.pose.position.x =  msg->pose.position.x;
            current_uav_pose_.pose.position.y = -msg->pose.position.y;
            current_uav_pose_.pose.position.z = -msg->pose.position.z;
            current_uav_pose_.pose.orientation.w =  msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x =  msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        } else {
            // 位置: ENU → NWU
            current_uav_pose_.pose.position.x =  msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z =  msg->pose.position.z;
            // 四元数: mavros(ENU 世界 + FRD 机体) → 内部(NWU 世界 + FLU 机体)
            //   q_internal = q_W · q_mavros · q_B
            //   q_W = (√2/2, 0, 0, -√2/2)  ENU→NWU: 绕世界 Z 旋转 -90°
            //   q_B = (0, 1, 0, 0)          FRD→FLU: 绕机体 X 旋转 180°
            // 与 gimbal_simulator_node.cpp::uavPoseCallback 完全一致,确保 target_estimator
            // 反投影用的 R_world_cam 与 gimbal forward projection 用的 R_world_cam 一致。
            // 旧版简单取反 (x, y, z, w)→(x, -y, -z, w) 是绕世界 X 转 180°,不是 ENU→NWU,
            // 会让 projectBboxToGround 的射线方向错误,导致 cluster 永远建不出来。
            const double kSqrtHalf = 0.7071067811865475;
            const auto& qe = msg->pose.orientation;
            current_uav_pose_.pose.orientation.w = -kSqrtHalf * (qe.x + qe.y);
            current_uav_pose_.pose.orientation.x =  kSqrtHalf * (qe.w + qe.z);
            current_uav_pose_.pose.orientation.y =  kSqrtHalf * (qe.z - qe.w);
            current_uav_pose_.pose.orientation.z =  kSqrtHalf * (qe.x - qe.y);
        }
        current_uav_pose_.header.stamp = msg->header.stamp;
        is_uav_pose_received_ = true;
    }

    void selfGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
            current_gps_ = *msg;
            is_gps_received_ = true;
        }
    }

    // ============ 周期任务 ============

    // 10Hz:publish 所有 alive cluster 快照 + 可视化 + primary GPS / pose
    void statesTimerCb(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> g(clusters_mtx_);
        // === 2026-07-16: 仅展示已晋升的 cluster ===
        //   tentative 在晋升前不出现在 snapshot / marker 里,避免污染 RViz
        size_t promoted_count = 0;
        for (const auto& kv : clusters_) if (kv.second.is_promoted) ++promoted_count;
        if (promoted_count == 0) {
            publishEmptyGps();
            return;
        }

        multi_uav_strike::ClusterState state;
        visualization_msgs::MarkerArray markers;
        state.targets.reserve(promoted_count);
        markers.markers.reserve(promoted_count);

        for (const auto& kv : clusters_) {
            const TargetCluster& c = kv.second;
            if (!c.is_promoted) continue;  // 未晋升的 tentative 不外发
            multi_uav_strike::ClusterTarget t;
            t.cluster_id    = c.id;
            t.label         = c.label;
            t.label_hash    = c.label_hash;
            t.ned_x         = c.ned_x;
            t.ned_y         = c.ned_y;
            t.ned_alt       = c.ned_alt;
            t.confidence    = c.best_confidence;
            t.last_seen_us  = c.last_seen_us;
            t.img_data      = c.img_data;
            t.img_format    = c.img_format;
            state.targets.push_back(t);

            markers.markers.push_back(buildMarker(c));
        }
        cluster_states_pub_.publish(state);
        target_est_marker_pub_.publish(markers);

        // primary 选择: 优先 primary_cluster_id_, 否则最高置信度 (仅看 promoted)
        const TargetCluster* primary = nullptr;
        if (primary_cluster_id_ != 0) {
            auto it = clusters_.find(primary_cluster_id_);
            if (it != clusters_.end() && it->second.is_promoted) {
                primary = &it->second;
            }
        }
        if (primary == nullptr) {
            for (const auto& kv : clusters_) {
                const TargetCluster& c = kv.second;
                if (!c.is_promoted) continue;
                if (primary == nullptr || c.best_confidence > primary->best_confidence) {
                    primary = &c;
                }
            }
        }
        if (primary != nullptr) {
            publishPrimaryGps(*primary);
            publishPrimaryPose(*primary);
        }
    }

    // 1Hz:清理超 TTL 的 cluster
    void cleanupTimerCb(const ros::TimerEvent&) {
        ros::Time now = ros::Time::now();
        std::lock_guard<std::mutex> g(clusters_mtx_);
        for (auto it = clusters_.begin(); it != clusters_.end(); ) {
            const auto& c = it->second;
            ros::Time last_seen;
            last_seen.fromNSec(c.last_seen_us * 1000ULL);
            double age = (now - last_seen).toSec();
            // === 2026-07-16: tentative 用更短 TTL,promoted 用 cluster_ttl_sec_ ===
            double ttl = c.is_promoted ? cluster_ttl_sec_ : tentative_ttl_sec_;
            if (age > ttl) {
                if (c.is_promoted) {
                    ROS_INFO("[TargetClusterer] EXPIRED cluster id=%lu label=%s "
                             "(alive %.1fs)", c.id, c.label.c_str(), age);
                } else {
                    ROS_INFO("[TargetClusterer] DROP tentative id=%lu label=%s "
                             "(hit=%u/%d, age=%.2fs)", c.id, c.label.c_str(),
                             c.hit_count, min_hit_count_, age);
                }
                it = clusters_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    // ============ 数据成员 ============
    ros::NodeHandle nh_;
    bool use_sim_ = true;
    std::string uav_pose_topic_;

    double merge_radius_m_   = 10.0;
    double cluster_ttl_sec_  = 60.0;
    int    min_hit_count_    = 3;       // 同 cluster 被检测到这么多次才晋升
    double tentative_ttl_sec_ = 1.0;    // 未晋升的 tentative 在这么久没刷新就丢弃
    double ema_alpha_        = 0.6;
    double target_z_prior_   = 0.0;
    double states_publish_freq_ = 10.0;
    double cleanup_freq_     = 1.0;
    double est_marker_r_ = 1.0, est_marker_g_ = 1.0, est_marker_b_ = 0.0, est_marker_a_ = 1.0;

    // 相机几何
    int    image_width_  = 1920;
    int    image_height_ = 1080;
    double fov_h_deg_    = 66.0;
    double fov_v_deg_    = 52.0;
    double fov_h_rad_ = 0.0, fov_v_rad_ = 0.0;
    double focal_x_ = 0.0, focal_y_ = 0.0;

    ros::Subscriber yolo_batch_sub_;
    ros::Subscriber gimbal_pose_sub_;
    ros::Subscriber camera_pose_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Subscriber self_gps_sub_;
    ros::Subscriber primary_target_sub_;

    ros::Publisher cluster_event_pub_;
    ros::Publisher cluster_states_pub_;
    ros::Publisher target_est_pose_pub_;
    ros::Publisher target_est_marker_pub_;
    ros::Publisher target_est_gps_pub_;
    ros::Timer states_timer_;
    ros::Timer cleanup_timer_;

    geometry_msgs::PoseStamped  current_uav_pose_;
    sensor_msgs::NavSatFix      current_gps_;
    double                      current_gimbal_pitch_ = 0.0;
    // === 2026-07-16: 稳像世界 quat,直接来自 gimbal 的 camera/pose ===
    Eigen::Quaterniond          current_camera_world_quat_{1.0, 0.0, 0.0, 0.0};
    bool is_uav_pose_received_    = false;
    bool is_gps_received_         = false;
    bool is_gimbal_pose_received_ = false;
    bool is_camera_pose_received_ = false;

    uint64_t primary_cluster_id_ = 0;   // 0 = mission 未指定, 回退最高置信度

    std::mutex clusters_mtx_;
    std::unordered_map<uint64_t, TargetCluster> clusters_;
    uint64_t next_cluster_id_ = 0;

    // ============ 工具 ============

    void computeFocalLengths() {
        fov_h_rad_ = fov_h_deg_ * M_PI / 180.0;
        if (fov_v_deg_ > 0.0) {
            fov_v_rad_ = fov_v_deg_ * M_PI / 180.0;
        } else {
            double aspect = static_cast<double>(image_width_) / image_height_;
            fov_v_rad_ = 2.0 * std::atan(std::tan(fov_h_rad_ / 2.0) / aspect);
        }
        focal_x_ = (image_width_  / 2.0) / std::tan(fov_h_rad_ / 2.0);
        focal_y_ = (image_height_ / 2.0) / std::tan(fov_v_rad_ / 2.0);
    }

    static uint32_t hashLabel(const std::string& s) {
        // FNV-1a 32-bit:足够防撞,非密码学场景
        uint32_t h = 2166136261u;
        for (unsigned char c : s) { h ^= c; h *= 16777619u; }
        return h;
    }

    // Per-detection bbox 反投影: 与 gimbal 的 forward projection 完全互逆。
    //   1) bbox 中心像素 → (u_ndc, v_ndc)
    //   2) 相机射线方向 (cam frame, +z = 光学轴):  t_cam ∝ (-v_ndc, u_ndc, 1)
    //   3) world 射线 = R_world_cam * t_cam (R_world_cam 来自 gimbal 的稳像 camera/pose)
    //   4) 射线与 z = target_z_prior_ 平面求交 → 目标世界 (NWU) 坐标
    bool projectBboxToGround(const multi_uav_strike::YoloDetection& det,
                             double& tx, double& ty, double& tz) {
        double cx = (det.x_min + det.x_max) * 0.5 * image_width_;
        double cy = (det.y_min + det.y_max) * 0.5 * image_height_;
        return projectPixelToGround(cx, cy, tx, ty, tz);
    }

    // === 2026-07-16: 像素级 helper (Plan A 用) ===
    //   processSingleDetection 先在像素空间做 EMA,再调本函数投影到世界。
    //   几何部分与 projectBboxToGround 完全相同,只是输入换成已平滑的像素。
    bool projectPixelToGround(double cx, double cy,
                              double& tx, double& ty, double& tz) {
        double u_ndc = (cx - image_width_  / 2.0) / focal_x_;
        double v_ndc = (cy - image_height_ / 2.0) / focal_y_;

        Eigen::Vector3d cam_ray(-v_ndc, u_ndc, 1.0);
        cam_ray.normalize();

        // === 2026-07-16: 直接使用 gimbal 算好的稳像世界 quat (camera/pose) ===
        // 不再在 estimator 这边自己重建 R_world_cam,消除前/反投影用不同 uav_quat 时的漂移。
        Eigen::Matrix3d R_world_cam = current_camera_world_quat_.toRotationMatrix();
        Eigen::Vector3d world_ray  = R_world_cam * cam_ray;

        Eigen::Vector3d cam_pos(
            current_uav_pose_.pose.position.x,
            current_uav_pose_.pose.position.y,
            current_uav_pose_.pose.position.z);

        double dz = target_z_prior_ - cam_pos.z();
        if (std::fabs(world_ray.z()) < 1e-3) return false;  // 射线几乎水平
        double t = dz / world_ray.z();
        if (t < 0.0) return false;  // 射线指向远离地面 (目标在 UAV "背后")

        Eigen::Vector3d target_world = cam_pos + t * world_ray;
        tx = target_world.x();
        ty = target_world.y();
        tz = target_z_prior_;
        return true;
    }

    visualization_msgs::Marker buildMarker(const TargetCluster& c) {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp    = ros::Time::now();
        m.ns   = "clustered_targets";
        m.id   = static_cast<int>(c.id);
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.ned_x;
        m.pose.position.y = c.ned_y;
        m.pose.position.z = c.ned_alt;
        m.pose.orientation.w = 1.0;
        m.scale.z = 1.0;  // text height
        m.color.r = std::max(0.f, std::min(1.f, (float)est_marker_r_));
        m.color.g = std::max(0.f, std::min(1.f, (float)est_marker_g_));
        m.color.b = std::max(0.f, std::min(1.f, (float)est_marker_b_));
        m.color.a = std::max(0.f, std::min(1.f, (float)est_marker_a_));
        m.lifetime = ros::Duration(0.5);
        m.text = c.label;
        return m;
    }

    void publishEmptyGps() {
        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "wgs84";
        msg.status.status  = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        target_est_gps_pub_.publish(msg);
    }

    void publishPrimaryGps(const TargetCluster& c) {
        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "wgs84";

        if (!is_gps_received_ || !is_uav_pose_received_) {
            publishEmptyGps();
            return;
        }
        const double dx = c.ned_x - current_uav_pose_.pose.position.x;
        const double dy = c.ned_y - current_uav_pose_.pose.position.y;
        const double dz = c.ned_alt - current_uav_pose_.pose.position.z;
        const double north_m = dx;       // NWU x = 北
        const double east_m  = -dy;      // NWU y = 西 → 东 = -y
        const double up_m    = dz;
        const double DEG_PER_M_LAT = 1.0 / 111320.0;
        const double uav_lat_rad = current_gps_.latitude * M_PI / 180.0;
        const double DEG_PER_M_LON = 1.0 / (111320.0 * std::cos(uav_lat_rad));

        msg.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        msg.latitude  = current_gps_.latitude  + north_m * DEG_PER_M_LAT;
        msg.longitude = current_gps_.longitude + east_m  * DEG_PER_M_LON;
        msg.altitude  = current_gps_.altitude  + up_m;
        target_est_gps_pub_.publish(msg);
    }

    /**
     * Publish primary cluster 位置作为 geometry_msgs::PoseStamped (NWU, 米)
     * guidance_control_node 订阅 target_estimated_pose 用此驱动 UAV 飞向目标
     */
    void publishPrimaryPose(const TargetCluster& c) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";  // NWU 世界系
        msg.pose.position.x = c.ned_x;
        msg.pose.position.y = c.ned_y;
        msg.pose.position.z = c.ned_alt;
        msg.pose.orientation.w = 1.0;  // 单位四元数(目标无朝向)
        target_est_pose_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_estimator_node");
    TargetClusterer node;
    ros::spin();
    return 0;
}