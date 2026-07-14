// ============================================================================
// target_estimator_node — 目标聚类 (label + 2D 位置, LOS 几何反投影)
//
// 设计变更 (2026-07):
//   - 删除原粒子滤波 (PF):`bypass_pf=true` 已默认,而且 PF 单独 LOS 反投影收敛很差。
//   - 改成"聚类节点":每条 YoloDetection 在 FOV 内时,基于 LOS 几何反投影得到地面位置
//     (target_z_prior_ 假设地面),与已有 cluster 做 (label + 2D 距离 ≤ merge_radius_m_) 合并。
//   - 命中已有 cluster:EMA 平滑刷新中心 + last_seen_us,不发事件。
//   - 新建 cluster:仅在创建瞬间 publish 一次 cluster_event(交给 mission_manager 派发)。
//   - 周期 10Hz publish cluster_states(可视化/兜底) + target_estimated_marker / gps。
//
// 输入:
//   /detection/yolo_result  (typed YoloDetection,来自 gimbal_simulator_node)
//   /target_los_angle       (Point: x=yaw, y=pitch, z=tracking_accuracy,仅用于几何)
//   /<uav_pose_topic>       (PoseStamped,NWU 已经做 use_sim 切换)
//   /mavros/global_position/global (NavSatFix,UAV 真实 GPS)
//   /detection/target_in_view (Bool,云台 FOV)
//
// 输出:
//   cluster_event            (ClusterEvent)— 新 cluster 唯一事件
//   cluster_states           (ClusterState, 10Hz)— 所有 alive cluster 快照
//   target_estimated_marker  (MarkerArray, 10Hz)— RViz 可视化
//   target_estimated_gps     (NavSatFix, 10Hz)— 取 best cluster 的 GPS(兼容位)
// ============================================================================
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <multi_uav_strike/YoloDetection.h>
#include <multi_uav_strike/ClusterEvent.h>
#include <multi_uav_strike/ClusterTarget.h>
#include <multi_uav_strike/ClusterState.h>

#include <cmath>
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
        // 透传到 ClusterTarget.img_data,mission_manager 据此填 DetectTarget.img_data
        std::vector<uint8_t> img_data;
        uint8_t              img_format = 0;  // 1 = JPEG(与 DetectTarget.img_format 对齐)
    };

    TargetClusterer() {
        ros::NodeHandle p("~");
        // 仿真/真机切换:uav_pose_topic 选择
        p.param<bool>("use_sim", use_sim_, true);
        p.param<std::string>("uav_pose_topic", uav_pose_topic_,
                             use_sim_ ? "quad/pose" : "mavros/local_position/pose");
        // 聚类参数
        p.param<double>("merge_radius_m", merge_radius_m_, 10.0);     // 2D 距离 ≤ 此值 → 同一 cluster
        p.param<double>("cluster_ttl_sec", cluster_ttl_sec_, 60.0);   // 多久无新命中 → 释放
        p.param<double>("ema_alpha",       ema_alpha_,       0.6);    // 新观测权重
        p.param<double>("target_z_prior",  target_z_prior_,  0.0);    // 反投影 z 假设(地面 = 0)
        // 发布周期
        p.param<double>("states_publish_freq", states_publish_freq_, 10.0);  // Hz
        p.param<double>("cleanup_freq",        cleanup_freq_,        1.0);   // Hz
        // marker 颜色
        p.param<double>("est_marker_r", est_marker_r_, 1.0);
        p.param<double>("est_marker_g", est_marker_g_, 1.0);
        p.param<double>("est_marker_b", est_marker_b_, 0.0);
        p.param<double>("est_marker_a", est_marker_a_, 1.0);

        // 订阅
        yolo_sub_           = nh_.subscribe("detection/yolo_result", 10,
                                            &TargetClusterer::yoloCallback, this);
        los_sub_            = nh_.subscribe("target_los_angle", 10,
                                            &TargetClusterer::losCallback, this);
        uav_pose_sub_       = nh_.subscribe(uav_pose_topic_, 10,
                                            &TargetClusterer::uavPoseCallback, this);
        self_gps_sub_       = nh_.subscribe("mavros/global_position/global", 10,
                                            &TargetClusterer::selfGpsCallback, this);
        target_in_view_sub_ = nh_.subscribe("detection/target_in_view", 10,
                                            &TargetClusterer::targetInViewCallback, this);

        // 发布
        cluster_event_pub_      = nh_.advertise<multi_uav_strike::ClusterEvent>(
                                    "cluster_event", 10);
        cluster_states_pub_     = nh_.advertise<multi_uav_strike::ClusterState>(
                                    "cluster_states", 10);
        target_est_pose_pub_    = nh_.advertise<geometry_msgs::PoseStamped>(
                                    "target_estimated_pose", 10);
        // 注:聚类只产生 2D 位置、无速度,不发 target_estimated_twist。
        // guidance 的 INTERCEPT 为纯追踪、不需要 twist。将来做动目标提前量拦截时,
        // 在此对 cluster 中心做差分估速并发布 target_estimated_twist。
        target_est_marker_pub_  = nh_.advertise<visualization_msgs::MarkerArray>(
                                    "target_estimated_marker", 10);
        target_est_gps_pub_     = nh_.advertise<sensor_msgs::NavSatFix>(
                                    "target_estimated_gps", 10);

        // 周期任务
        states_timer_ = nh_.createTimer(ros::Duration(1.0 / states_publish_freq_),
                                        &TargetClusterer::statesTimerCb, this);
        cleanup_timer_= nh_.createTimer(ros::Duration(1.0 / cleanup_freq_),
                                        &TargetClusterer::cleanupTimerCb, this);

        ROS_INFO("[TargetClusterer] initialized. merge_radius=%.1fm ttl=%.1fs ema_alpha=%.2f z_prior=%.1f "
                 "(uav_pose_topic=%s, use_sim=%d)",
                 merge_radius_m_, cluster_ttl_sec_, ema_alpha_, target_z_prior_,
                 uav_pose_topic_.c_str(), (int)use_sim_);
    }

    // ============ 回调 ============

    void yoloCallback(const multi_uav_strike::YoloDetection::ConstPtr& msg) {
        if (!msg->is_in_fov) {
            // 与原 PF 中 "云台回退 default_pitch" 同样的语义:不再有合法目标
            return;
        }
        if (!is_uav_pose_received_) return;

        // 1. 几何反投影:已知 UAV pose + LOS (yaw,pitch),target on ground plane z=target_z_prior_
        double tx = 0.0, ty = 0.0, tz = target_z_prior_;
        if (!projectLosToGround(tx, ty, tz)) return;

        std::lock_guard<std::mutex> g(clusters_mtx_);
        std::string label = msg->label;
        auto it = label_to_id_.find(label);
        if (it != label_to_id_.end()) {
            TargetCluster& c = clusters_[it->second];
            // label 相等 + 2D 距离 ≤ 阈值 → 合并到已有 cluster
            double dx = tx - c.ned_x;
            double dy = ty - c.ned_y;
            if (c.label == label && (dx*dx + dy*dy) <= merge_radius_m_ * merge_radius_m_) {
                c.ned_x = ema_alpha_ * tx + (1.0 - ema_alpha_) * c.ned_x;
                c.ned_y = ema_alpha_ * ty + (1.0 - ema_alpha_) * c.ned_y;
                c.ned_alt = tz;
                c.best_confidence = std::max(c.best_confidence, msg->confidence);
                c.last_seen_us = msg->stamp_us;
                // 仅在新帧携带图时才覆盖(img_data 可能为 0)
                if (!msg->img_data.empty()) {
                    c.img_data   = msg->img_data;
                    c.img_format = 1;  // YoloDetection.img_data 总是 JPEG(见 gimbal_simulator publishYoloDetection)
                }
                return;
            }
        }
        // 2. 新建 cluster
        TargetCluster nc;
        nc.id              = ++next_cluster_id_;
        nc.label           = label;
        nc.label_hash      = hashLabel(label);
        nc.ned_x           = tx;
        nc.ned_y           = ty;
        nc.ned_alt         = tz;
        nc.best_confidence = msg->confidence;
        nc.first_seen_us   = msg->stamp_us;
        nc.last_seen_us    = msg->stamp_us;
        nc.img_data        = msg->img_data;
        nc.img_format      = msg->img_data.empty() ? 0 : 1;  // 1 = JPEG(与 gimbal_simulator 输出一致)

        clusters_[nc.id] = nc;
        label_to_id_[label] = nc.id;

        // 3. 推 cluster_event(仅一次,mission_manager 据此决定上报/跟踪/打击)
        multi_uav_strike::ClusterEvent ev;
        ev.cluster_id    = nc.id;
        ev.label_hash    = nc.label_hash;
        ev.label         = nc.label;
        ev.confidence    = nc.best_confidence;
        ev.first_seen_us = nc.first_seen_us;
        ev.ned_x         = nc.ned_x;
        ev.ned_y         = nc.ned_y;
        ev.ned_alt       = nc.ned_alt;
        cluster_event_pub_.publish(ev);

        ROS_INFO_THROTTLE(1.0, "[TargetClusterer] NEW cluster id=%lu label=%s "
                              "(ned=%.1f,%.1f,%.1f conf=%.2f, total_clusters=%zu)",
                           nc.id, nc.label.c_str(), nc.ned_x, nc.ned_y, nc.ned_alt,
                           nc.best_confidence, clusters_.size());
    }

    void losCallback(const geometry_msgs::Point::ConstPtr& msg) {
        current_los_ = *msg;
        is_los_received_ = true;
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
            // ENU → NWU: 与原代码完全一致
            current_uav_pose_.pose.position.x =  msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z =  msg->pose.position.z;
            current_uav_pose_.pose.orientation.w =  msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x =  msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
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

    void targetInViewCallback(const std_msgs::Bool::ConstPtr& msg) {
        target_in_view_ = msg->data;
        (void)target_in_view_;  // 当前实现不强制使用 — yolo.msg.is_in_fov 已等价过滤
    }

    // ============ 周期任务 ============

    // 10Hz:publish 所有 alive cluster 快照 + 可视化 + best GPS
    void statesTimerCb(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> g(clusters_mtx_);
        if (clusters_.empty()) {
            // 没 cluster 时也发,但 target_estimated_gps 用 NO_FIX 兜底
            publishEmptyGps();
            return;
        }

        multi_uav_strike::ClusterState state;
        visualization_msgs::MarkerArray markers;

        const TargetCluster* best = nullptr;
        for (const auto& kv : clusters_) {
            const TargetCluster& c = kv.second;
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

            if (best == nullptr || c.best_confidence > best->best_confidence) {
                best = &c;
            }
        }
        cluster_states_pub_.publish(state);
        target_est_marker_pub_.publish(markers);
        if (best != nullptr) {
            publishBestClusterGps(*best);
            publishBestClusterPose(*best);  // target_estimated_pose (NWU PoseStamped) — guidance_control_node 用
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
            if (age > cluster_ttl_sec_) {
                ROS_INFO_THROTTLE(2.0, "[TargetClusterer] EXPIRED cluster id=%lu label=%s "
                                       "(alive %.1fs)", c.id, c.label.c_str(), age);
                label_to_id_.erase(c.label);
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
    double ema_alpha_        = 0.6;
    double target_z_prior_   = 0.0;
    double states_publish_freq_ = 10.0;
    double cleanup_freq_     = 1.0;
    double est_marker_r_ = 1.0, est_marker_g_ = 1.0, est_marker_b_ = 0.0, est_marker_a_ = 1.0;

    ros::Subscriber yolo_sub_;
    ros::Subscriber los_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Subscriber self_gps_sub_;
    ros::Subscriber target_in_view_sub_;

    ros::Publisher cluster_event_pub_;
    ros::Publisher cluster_states_pub_;
    ros::Publisher target_est_pose_pub_;
    ros::Publisher target_est_marker_pub_;
    ros::Publisher target_est_gps_pub_;
    ros::Timer states_timer_;
    ros::Timer cleanup_timer_;

    geometry_msgs::Point        current_los_;
    geometry_msgs::PoseStamped  current_uav_pose_;
    sensor_msgs::NavSatFix      current_gps_;
    bool is_los_received_    = false;
    bool is_uav_pose_received_ = false;
    bool is_gps_received_    = false;

    bool target_in_view_ = false;  // 留作将来兼容;当前用 yolo.is_in_fov

    std::mutex clusters_mtx_;
    std::unordered_map<uint64_t, TargetCluster> clusters_;
    std::unordered_map<std::string, uint64_t>   label_to_id_;
    uint64_t next_cluster_id_ = 0;

    // ============ 工具 ============

    static uint32_t hashLabel(const std::string& s) {
        // FNV-1a 32-bit:足够防撞,非密码学场景
        uint32_t h = 2166136261u;
        for (unsigned char c : s) { h ^= c; h *= 16777619u; }
        return h;
    }

    // UAV 本体位姿 + LOS → 目标地面坐标 (NWU)
    //   init_dist = uav_z / |sin(pitch)|
    //   target_x = uav_x + dist * cos(pitch) * cos(yaw)
    //   target_y = uav_y + dist * cos(pitch) * sin(yaw)
    //   target_z = target_z_prior_
    // 注意:原 computeLosTargetPosition 用 -yaw,这里也保持一致(NWU 约定)
    bool projectLosToGround(double& tx, double& ty, double& tz) {
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;
        double pitch = current_los_.y;
        double yaw   = -current_los_.x;  // 与原 PF 一致(NWU 约定)

        if (std::fabs(std::sin(pitch)) < 0.01) return false;
        double dist = uav_z / std::fabs(std::sin(pitch));
        const double max_dist = 1000.0;
        if (dist > max_dist) dist = max_dist;
        tx = uav_x + dist * std::cos(pitch) * std::cos(yaw);
        ty = uav_y + dist * std::cos(pitch) * std::sin(yaw);
        tz = target_z_prior_;
        return true;
    }

    visualization_msgs::Marker buildMarker(const TargetCluster& c) {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp    = ros::Time::now();
        m.ns   = "clustered_targets";
        m.id   = static_cast<int>(c.id);
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.ned_x;
        m.pose.position.y = c.ned_y;
        m.pose.position.z = c.ned_alt;
        m.pose.orientation.w = 1.0;
        m.scale.x = 1.5;  m.scale.y = 1.5;  m.scale.z = 1.5;
        m.color.r = std::max(0.f, std::min(1.f, (float)est_marker_r_));
        m.color.g = std::max(0.f, std::min(1.f, (float)est_marker_g_));
        m.color.b = std::max(0.f, std::min(1.f, (float)est_marker_b_));
        m.color.a = std::max(0.f, std::min(1.f, (float)est_marker_a_));
        m.lifetime = ros::Duration(0.5);
        // text 形式 label
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.text = c.label;
        m.scale.z = 1.0;  // text height
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

    void publishBestClusterGps(const TargetCluster& c) {
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
        msg.header.frame_id = "wgs84";
        target_est_gps_pub_.publish(msg);
    }

    /**
     * Publish best cluster 位置作为 geometry_msgs::PoseStamped (NWU, 米)
     * 这是 guidance_control_node 订阅的 target_estimated_pose:
     *   - 当没 cluster 时不发(guidance 的 targetPoseCallback 只在收到时才置 is_target_pose_received_)
     *   - 当有 best cluster 时发,guidance 用此坐标驱动 UAV 飞向目标
     *
     * 注:NWU 坐标系与 guidance_control_node 内部一致(target_estimator 内部反投影就在 NWU)。
     */
    void publishBestClusterPose(const TargetCluster& c) {
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
