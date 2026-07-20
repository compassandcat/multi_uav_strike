/**
 * comm_node.cpp
 * 通信节点：机间通信(地面站通信改由 typed TaskFlow 直达 mission_manager,不再走本节点)
 *
 * 职责：
 * - 广播本机位置给其他无人机
 * - 汇总邻居无人机位置
 * - 共享目标发现信息
 *
 * 订阅：
 * - /mavros/local_position/pose    - 本机位置（来自飞控）
 * - /inter_uav/other_uav_poses     - 邻居无人机位置
 * - /inter_uav/uav_status          - 邻居 UAV 心跳(Phase 7)
 * - /detection/yolo_result         - YOLO检测结果（typed YoloDetection）
 * - /target_estimated_pose         - 目标估计位置（共享给队友）
 *
 * 发布：
 * - /inter_uav/self_pose           - 本机位置
 * - /inter_uav/other_uav_poses     - 邻居无人机位置列表
 * - /inter_uav/target_info         - 目标信息
 * - /inter_uav/uav_status          - 本机状态心跳
 */

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>

// 自定义消息：目标信息
// 后续可扩展为 custom_msgs/TargetInfo
#include <vector>
#include <string>
#include <cstdint>
#include <algorithm>

// === Phase 7: 类型化心跳 ===
#include "multi_uav_strike/InterUavStatus.h"
#include "multi_uav_strike/UavGatherStatus.h"
#include "multi_uav_strike/YoloDetection.h"
#include "multi_uav_strike/MissionState.h"   // 用于把当前 skill_type/skill_id 灌进 InterUavStatus
#include "multi_uav_strike/WorkMode.h"       // 用于把当前 work_mode 灌进 InterUavStatus

class CommNode {
private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ============== 订阅 ==============
    ros::Subscriber self_pose_sub_;          // 本机位置（来自飞控）
    ros::Subscriber other_uav_poses_sub_;    // 邻居无人机位置
    ros::Subscriber yolo_result_sub_;        // YOLO检测结果(typed YoloDetection)
    ros::Subscriber target_est_pose_sub_;    // 目标估计位置
    // === Phase 7: 其他 UAV 心跳 ===
    ros::Subscriber inter_uav_status_sub_;
    // === Phase 7: 本机 mission_state 订阅(把 skill_type/skill_id 灌进 InterUavStatus)===
    ros::Subscriber mission_state_sub_;
    ros::Subscriber work_mode_state_sub_;
    // === PX4 home 自动加载 ===
    ros::Subscriber home_position_sub_;

    // ============== 发布 ==============
    ros::Publisher self_pose_pub_;           // 本机位置(发给其他无人机)
    ros::Publisher other_uav_poses_pub_;     // 邻居无人机位置列表
    ros::Publisher target_info_pub_;         // 目标信息
    // === Phase 7: 类型化心跳 ===
    ros::Publisher inter_uav_status_pub_;

    // ============== 状态 ==============
    geometry_msgs::PoseStamped current_self_pose_;
    bool is_self_pose_received_ = false;

    // 邻居无人机列表
    struct NeighborUav {
        std::string name;
        geometry_msgs::PoseStamped pose;
        ros::Time last_update;
    };
    std::vector<NeighborUav> neighbors_;
    double neighbor_timeout_;  // 邻居超时时间（秒）

    // (legacy /gs/* 状态缓冲已移除,任务入口由 typed TaskFlow 直达 mission_manager)

    // 目标信息
    struct TargetInfo {
        geometry_msgs::PoseStamped pose;
        std::string target_type;  // "person", "vehicle"
        double confidence;
        ros::Time detection_time;
        bool is_shared;  // 是否已共享给队友
    };
    std::vector<TargetInfo> detected_targets_;
    double target_share_interval_;  // 目标共享间隔（秒）
    ros::Time last_target_share_time_;

    // 定时器
    ros::Timer neighbor_check_timer_; // 邻居检查定时器
    // === Phase 7: 心跳定时器(频率自适应 1/5/20Hz)===
    ros::Timer inter_uav_status_timer_;

    // === Phase 7: 心跳调频相关 ===
    double heartbeat_base_rate_;    // 基础频率 (Hz,默认 1)
    double heartbeat_near_rate_;    // 邻居 ≤40m 时频率 (Hz,默认 5)
    double heartbeat_close_rate_;   // 邻居 ≤20m 时频率 (Hz,默认 20)
    double heartbeat_near_thresh_;  // ≤此距离进入 near 频率 (m,默认 40)
    double heartbeat_close_thresh_; // ≤此距离进入 close 频率 (m,默认 20)
    double current_heartbeat_rate_; // 当前实际频率
    uint8_t current_typed_work_mode_;   // 来自 mission_manager 的 typed mode(占位)
    int32_t current_skill_type_;        // 当前 skill_type(占位)
    std::string current_skill_id_;      // 当前 skill_id(占位)

    // 邻居心跳缓存(用于调频 + 集结判断)
    struct NeighborHeartbeat {
        std::string sn;
        multi_uav_strike::InterUavStatus::ConstPtr latest;
        ros::Time last_update;
    };
    std::vector<NeighborHeartbeat> neighbor_heartbeats_;

    // 参数
    std::string uav_name_;
    // UAV 序列号(命名约定 "<group>-<plane>" 如 "1-1");心跳 sn 用这个值,避免暴露 ROS namespace
    // 默认回落到 uav_name,保持向后兼容
    std::string uav_device_sn_;
    bool use_sim_;        // 仿真/真机切换

    // GPS 参考点（NED <-> GPS 转换用）。默认初始化为 0/0/0:
    // 启动后由 homePositionCallback() 自动从 /<ns>/mavros/home_position/home 拉取 PX4 当前 home,
    // 不再依赖 yaml 硬编码 (用户最新指示: 不应该硬编码 ref_*)。
    double ref_lat_ = 0.0;      // 参考纬度（度）
    double ref_lon_ = 0.0;      // 参考经度（度）
    double ref_alt_ = 0.0;      // 参考高度（米）
    bool ref_initialized_ = false;  // 是否已收到 PX4 home

public:
    CommNode() : nh_private_("~"),
        neighbor_timeout_(5.0),
        target_share_interval_(1.0) {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();

        ROS_INFO("[Comm] CommNode initialized for UAV: %s (device_sn=%s)",
                 uav_name_.c_str(), uav_device_sn_.c_str());
    }

    void initParams() {
        nh_private_.param<std::string>("uav_name", uav_name_, "uav0");
        // device_sn 是心跳实际发的 sn;空 → 回落 uav_name,避免改了 launch 之后还得改所有地方
        nh_private_.param<std::string>("uav_device_sn", uav_device_sn_, uav_name_);
        nh_private_.param<double>("neighbor_timeout", neighbor_timeout_, 5.0);
        nh_private_.param<double>("target_share_interval", target_share_interval_, 1.0);
        nh_private_.param<bool>("use_sim", use_sim_, true);

        // === Phase 7: 心跳调频参数 ===
        nh_private_.param<double>("heartbeat_base_rate",  heartbeat_base_rate_,  1.0);
        nh_private_.param<double>("heartbeat_near_rate",  heartbeat_near_rate_,  5.0);
        nh_private_.param<double>("heartbeat_close_rate", heartbeat_close_rate_, 20.0);
        nh_private_.param<double>("heartbeat_near_thresh",  heartbeat_near_thresh_,  40.0);
        nh_private_.param<double>("heartbeat_close_thresh", heartbeat_close_thresh_, 20.0);
        current_heartbeat_rate_ = heartbeat_base_rate_;
        current_typed_work_mode_ = 0;
        current_skill_type_ = 0;
        // GPS 参考点不再从 yaml 硬编码,而由 PX4 /mavros/home_position/home 自动填充
        nh_private_.param<double>("ref_lat", ref_lat_, ref_lat_);
        nh_private_.param<double>("ref_lon", ref_lon_, ref_lon_);
        nh_private_.param<double>("ref_alt", ref_alt_, ref_alt_);
    }

    void initSubscribers() {
        // 本机位置
        // 仿真：订阅 quad/pose；实机：订阅 mavros/local_position/pose
        if (use_sim_) {
            self_pose_sub_ = nh_.subscribe(
                "quad/pose", 10,
                &CommNode::selfPoseCallback, this);
        } else {
            self_pose_sub_ = nh_.subscribe(
                "mavros/local_position/pose", 10,
                &CommNode::selfPoseCallback, this);
        }

        // 订阅所有 UAV 发布的自己的位置（全局 topic）
        other_uav_poses_sub_ = nh_.subscribe(
            "/inter_uav/self_pose", 10,
            &CommNode::otherUavPosesCallback, this);

        // YOLO 检测结果（目标发现）
        yolo_result_sub_ = nh_.subscribe(
            "detection/yolo_result", 10,
            &CommNode::yoloResultCallback, this);

        // 目标估计位置（用于共享给队友）
        target_est_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &CommNode::targetEstPoseCallback, this);

        // === Phase 7: 其他 UAV 心跳(全局 topic)===
        inter_uav_status_sub_ = nh_.subscribe(
            "/inter_uav/uav_status", 50,
            &CommNode::interUavStatusCallback, this);

        // === Phase 7: 本机 mission_state(1Hz,来自 mission_manager)===
        //   用来把 current_skill_type_/skill_id_/work_mode_ 灌进 InterUavStatus,
        //   否则心跳里 skill_type 永远是 0(占位),多机 gather 同步无法判别伙伴当前阶段。
        mission_state_sub_ = nh_.subscribe(
            "mission/mission_state", 10,
            &CommNode::missionStateCallback, this);
        work_mode_state_sub_ = nh_.subscribe(
            "mission/work_mode_state", 10,
            &CommNode::workModeStateCallback, this);

        // === PX4 home 自动加载(ref_lat/lon/alt 的唯一来源,不再硬编码)===
        home_position_sub_ = nh_.subscribe(
            "mavros/home_position/home", 10,
            &CommNode::homePositionCallback, this);
    }

    void initPublishers() {
        // 本机位置（发给其他无人机）- 用相对路径，自动加 namespace
        self_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/inter_uav/self_pose", 10);

        // 邻居无人机位置列表（供 waypoint_executor 订阅）
        other_uav_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
            "inter_uav/other_uav_poses", 10);

        // 目标信息（共享给队友）
        target_info_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "inter_uav/target_info", 10);

        // === Phase 7: 类型化心跳(全局 topic)===
        inter_uav_status_pub_ = nh_.advertise<multi_uav_strike::InterUavStatus>(
            "/inter_uav/uav_status", 50);
    }

    void initTimers() {
        // 邻居检查定时器（1Hz）
        neighbor_check_timer_ = nh_.createTimer(
            ros::Duration(1.0),
            &CommNode::neighborCheckTimerCallback, this);

        // === Phase 7: 心跳定时器(频率自适应)===
        inter_uav_status_timer_ = nh_.createTimer(
            ros::Duration(1.0 / current_heartbeat_rate_),
            &CommNode::interUavStatusTimerCallback, this);
    }

    // ============== 回调函数 ==============

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 先把输入转为统一 NED（本地坐标系）
        double ned_x, ned_y, ned_z;
        if (use_sim_) {
            // 仿真输入是 NED 坐标系
            ned_x = msg->pose.position.x;
            ned_y = msg->pose.position.y;
            ned_z = msg->pose.position.z;
        } else {
            // Mavros 输入是 ENU 坐标系
            // ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            ned_x = msg->pose.position.y;
            ned_y = msg->pose.position.x;
            ned_z = -msg->pose.position.z;
        }

        // 转换 NED -> GPS（WGS84）用于机间共享
        double lat, lon, alt;
        nedToGps(ned_x, ned_y, ned_z, lat, lon, alt);

        // 存为 GPS pose: position.x=lat, position.y=lon, position.z=alt
        current_self_pose_.pose.position.x = lat;
        current_self_pose_.pose.position.y = lon;
        current_self_pose_.pose.position.z = alt;
        current_self_pose_.pose.orientation = msg->pose.orientation;
        current_self_pose_.header = msg->header;
        is_self_pose_received_ = true;

        // 设置 frame_id 为本机名，订阅方通过此过滤自己的消息
        current_self_pose_.header.frame_id = uav_name_;

        // 广播本机 GPS 位置给其他无人机（全局 topic）
        self_pose_pub_.publish(current_self_pose_);
    }

    /**
     * NED -> GPS 转换（使用参考点的小范围近似）
     * 输入：ned_x (北), ned_y (东), ned_z (下)
     * 输出：lat (度), lon (度), alt (米)
     */
    void nedToGps(double ned_x, double ned_y, double ned_z,
                  double& lat, double& lon, double& alt) {
        const double EARTH_R = 6378137.0;
        double d_lat = ned_x / EARTH_R * 180.0 / M_PI;
        double d_lon = ned_y / (EARTH_R * cos(ref_lat_ * M_PI / 180.0)) * 180.0 / M_PI;
        lat = ref_lat_ + d_lat;
        lon = ref_lon_ + d_lon;
        alt = ref_alt_ - ned_z;  // NED: z 向下 -> alt: 海拔向上
    }

    /**
     * PX4 home 回调 — 锁一次
     * PX4 的 LOCAL_POSITION_NED 参考系由 EKF2 在启动时锁定,
     * 后续 home 更新(MAV_CMD_DO_SET_HOME / disarm)不会重置 EKF2 origin,
     * 也不让 local_position 跳变。如果跟着 home 更新去重算 NED,
     * 反而让 setpoint 与 UAV 当前 local_position 不在同一 frame → 偏飞。
     */
    void homePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg) {
        if (ref_initialized_) return;  // 锁一次
        if (msg->geo.latitude == 0.0 && msg->geo.longitude == 0.0) {
            // PX4 在 home 未稳定前可能发 0/0,忽略
            return;
        }
        ROS_WARN("[Comm] >>>> PX4 home locked: (%.7f, %.7f, %.2f)",
                 msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
        ref_lat_ = msg->geo.latitude;
        ref_lon_ = msg->geo.longitude;
        ref_alt_ = msg->geo.altitude;
        ref_initialized_ = true;
    }

    void otherUavPosesCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 忽略自己的消息
        if (msg->header.frame_id == uav_name_) {
            return;
        }

        // 更新或添加邻居
        bool found = false;
        for (auto& neighbor : neighbors_) {
            if (neighbor.name == msg->header.frame_id) {
                neighbor.pose = *msg;
                neighbor.last_update = ros::Time::now();
                found = true;
                break;
            }
        }

        if (!found) {
            NeighborUav new_neighbor;
            new_neighbor.name = msg->header.frame_id;
            new_neighbor.pose = *msg;
            new_neighbor.last_update = ros::Time::now();
            neighbors_.push_back(new_neighbor);
            ROS_INFO("[Comm] New neighbor discovered: %s", msg->header.frame_id.c_str());
        }

        // 发布邻居列表（汇总）
        publishOtherUavPoses();
    }

    void yoloResultCallback(const multi_uav_strike::YoloDetection::ConstPtr& msg) {
        // 收到 typed YoloDetection(multi_uav_strike/YoloDetection)
        // comm_node 当前仅做日志/转发占位,具体目标信息上报逻辑后续再做
        if (!msg->is_in_fov) {
            return;  // 矩形 FOV 外不处理
        }
        ROS_DEBUG("[Comm] YOLO detection: %s conf=%.2f bbox=[%.2f,%.2f,%.2f,%.2f]",
                  msg->label.c_str(), msg->confidence,
                  msg->x_min, msg->y_min, msg->x_max, msg->y_max);
    }

    void targetEstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 收到目标估计位置，共享给队友
        ros::Time now = ros::Time::now();

        // 限制目标共享频率
        if ((now - last_target_share_time_).toSec() < target_share_interval_) {
            return;
        }

        // 共享目标信息
        geometry_msgs::PoseStamped target_info = *msg;
        target_info.header.frame_id = uav_name_;  // 标记是哪个无人机发现的目标
        target_info_pub_.publish(target_info);

        last_target_share_time_ = now;
    }

    // ============== 定时器回调 ==============

    void neighborCheckTimerCallback(const ros::TimerEvent&) {
        // 清理超时的邻居
        ros::Time now = ros::Time::now();
        auto it = neighbors_.begin();
        while (it != neighbors_.end()) {
            if ((now - it->last_update).toSec() > neighbor_timeout_) {
                ROS_WARN("[Comm] Neighbor %s timeout, removing", it->name.c_str());
                it = neighbors_.erase(it);
            } else {
                ++it;
            }
        }

        // 更新邻居列表发布
        publishOtherUavPoses();
    }

    // ============== 辅助函数 ==============

    void publishOtherUavPoses() {
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "map";

        for (const auto& neighbor : neighbors_) {
            pose_array.poses.push_back(neighbor.pose.pose);
        }

        other_uav_poses_pub_.publish(pose_array);
    }

    // ============== Phase 7: 类型化心跳 ==============

    /**
     * 收到其他 UAV 的心跳 — 缓存 + 用于自适应调频
     */
    void interUavStatusCallback(const multi_uav_strike::InterUavStatus::ConstPtr& msg) {
        // 自环过滤:心跳 sn 已切到 device_sn("1-1"格式),不能再用 ROS namespace 比
        if (msg->sn == uav_device_sn_) {
            return;  // 忽略自己的
        }
        bool found = false;
        for (auto& nh : neighbor_heartbeats_) {
            if (nh.sn == msg->sn) {
                nh.latest = msg;
                nh.last_update = ros::Time::now();
                found = true;
                break;
            }
        }
        if (!found) {
            NeighborHeartbeat n;
            n.sn = msg->sn;
            n.latest = msg;
            n.last_update = ros::Time::now();
            neighbor_heartbeats_.push_back(n);
            ROS_INFO("[Comm] New heartbeat neighbor: %s", msg->sn.c_str());
        }
    }

    /**
     * 心跳定时器回调:发布本机 InterUavStatus + 自适应调频
     */
    void interUavStatusTimerCallback(const ros::TimerEvent&) {
        if (!is_self_pose_received_) return;

        // 1. 发本机心跳
        multi_uav_strike::InterUavStatus hs;
        // 心跳 sn 用 device_sn("1-1" 格式)而不是 ROS namespace,保证 uav_avoidance
        // 在仿真/实机下拿到的都是同一种 id,uav_avoidance 可以靠字典序仲裁优先级。
        hs.sn            = uav_device_sn_;
        hs.work_mode     = current_typed_work_mode_;
        hs.skill_type    = current_skill_type_;
        hs.skill_id      = current_skill_id_;
        hs.timestamp_us  = ros::Time::now().toNSec() / 1000;
        hs.lat           = current_self_pose_.pose.position.x;
        hs.lon           = current_self_pose_.pose.position.y;
        hs.alt           = current_self_pose_.pose.position.z;
        hs.yaw           = 0.0;  // 简化:暂不发 yaw
        inter_uav_status_pub_.publish(hs);

        // 2. 清理超时心跳
        ros::Time now = ros::Time::now();
        neighbor_heartbeats_.erase(
            std::remove_if(neighbor_heartbeats_.begin(), neighbor_heartbeats_.end(),
                           [&](const NeighborHeartbeat& n) {
                               return (now - n.last_update).toSec() > neighbor_timeout_;
                           }),
            neighbor_heartbeats_.end());

        // 3. 自适应调频:取最近邻居距离,据此选频率
        double min_dist = computeClosestNeighborDistance();
        double new_rate = heartbeat_base_rate_;
        if (min_dist < heartbeat_close_thresh_) {
            new_rate = heartbeat_close_rate_;
        } else if (min_dist < heartbeat_near_thresh_) {
            new_rate = heartbeat_near_rate_;
        }

        // 频率变化时才 setPeriod(避免每个 tick 重建 timer)
        if (std::fabs(new_rate - current_heartbeat_rate_) > 0.01) {
            current_heartbeat_rate_ = new_rate;
            inter_uav_status_timer_.setPeriod(ros::Duration(1.0 / current_heartbeat_rate_));
            ROS_INFO("[Comm] >>>> Heartbeat rate adjusted to %.1f Hz (closest UAV: %.1f m)",
                     current_heartbeat_rate_, min_dist);
        }
    }

    /**
     * mission_state 回调(mission_manager 1Hz 上报) — 把当前 skill_type/skill_id
     * 缓存下来,InterUavStatus 发布时携带。
     *   -1 表示无 active skill,心跳里照样发出 -1(下游应忽略)。
     */
    void missionStateCallback(const multi_uav_strike::MissionState::ConstPtr& msg) {
        current_skill_type_ = msg->skill_type;  // 可能是 -1(无 active skill)
        current_skill_id_   = msg->skill_id;
    }

    /**
     * work_mode_state 回调(mission_manager 1Hz 上报) — 把当前 work_mode 灌进心跳
     */
    void workModeStateCallback(const multi_uav_strike::WorkMode::ConstPtr& msg) {
        current_typed_work_mode_ = msg->mode;
    }

    /**
     * 计算最近邻居距离(米),无邻居返回 1e9
     * 用心跳缓存的 lat/lon/alt 做粗算(同 uav_avoidance_node 算法)
     */
    double computeClosestNeighborDistance() {
        if (neighbor_heartbeats_.empty()) return 1e9;

        double self_lat = current_self_pose_.pose.position.x;
        double self_lon = current_self_pose_.pose.position.y;
        double self_alt = current_self_pose_.pose.position.z;

        double min_dist = 1e9;
        for (const auto& n : neighbor_heartbeats_) {
            if (!n.latest) continue;
            double dlat = n.latest->lat - self_lat;
            double dlon = n.latest->lon - self_lon;
            double dalt = n.latest->alt - self_alt;
            double cos_lat = std::max(0.1, std::cos(self_lat * M_PI / 180.0));
            double dx = dlat * 111000.0;
            double dy = dlon * 111000.0 * cos_lat;
            double dist = std::sqrt(dx*dx + dy*dy + dalt*dalt);
            if (dist < min_dist) min_dist = dist;
        }
        return min_dist;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "comm_node");
    CommNode comm_node;
    ros::spin();
    return 0;
}
