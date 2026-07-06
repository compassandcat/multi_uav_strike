/**
 * uav_avoidance_node.cpp
 *
 * 多机机间避障决策节点(纯 ROS 内部,不动 starling_bridge)
 *
 * 职责:
 *   - 接收邻居无人机 GPS 位姿(/<ns>/inter_uav/other_uav_poses)
 *   - 接收自身当前航段终点(/<ns>/waypoint_executor/current_target)
 *   - 基于 task_priority + uav_id 优先级判定"谁让谁"
 *   - 发布避障指令(/<ns>/avoidance/cmd,multi_uav_strike/AvoidanceCmd)
 *
 * 策略:
 *   - 横向(lateral)严禁越线 — 压航线核心约束
 *   - 高度差 < vertical_safe 时让下位优先调高度
 *   - 调不开 → 减速 / 停车
 *   - 仍要撞 → 回退(reverse_allowed=true)
 *   - 优先依据:skill.priority 大的优先,同优则 uav_id 小的优先
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>

#include "multi_uav_strike/AvoidanceCmd.h"
#include "multi_uav_strike/InterUavStatus.h"

class UavAvoidance {
public:
    UavAvoidance() : nh_(), nh_private_("~") {
        nh_private_.param<bool>("use_sim", use_sim_, true);
        nh_private_.param<std::string>("uav_pose_topic", uav_pose_topic_, "");
        if (uav_pose_topic_.empty()) {
            uav_pose_topic_ = use_sim_ ? "quad/pose" : "mavros/local_position/pose";
        }
        nh_private_.param<std::string>("uav_id", uav_id_, "uav0");
        nh_private_.param<int>("uav_sn_priority_offset", uav_sn_priority_offset_, 0);
        nh_private_.param<int>("self_task_priority", self_task_priority_, 100);
        nh_private_.param<double>("collision_threshold", collision_threshold_, 8.0);
        nh_private_.param<double>("proximity_threshold", proximity_threshold_, 15.0);
        nh_private_.param<double>("vertical_safe", vertical_safe_, 5.0);
        nh_private_.param<double>("loop_freq", loop_freq_, 20.0);

        // 从 uav_id 提取数字,数值小优先(同优时排前)
        self_uav_num_ = extractUavNumber(uav_id_);

        ROS_INFO("[UavAvoidance] self_id=%s num=%d priority_offset=%d self_task_prio=%d",
                 uav_id_.c_str(), self_uav_num_, uav_sn_priority_offset_, self_task_priority_);

        // ========== 订阅 ==========
        self_pose_sub_ = nh_.subscribe(uav_pose_topic_, 10,
                                       &UavAvoidance::selfPoseCb, this);

        std::string neighbors_t = "inter_uav/other_uav_poses";
        neighbor_sub_ = nh_.subscribe(neighbors_t, 10,
                                      &UavAvoidance::neighborCb, this);

        std::string current_target_t = "waypoint_executor/current_target";
        current_target_sub_ = nh_.subscribe(current_target_t, 10,
                                            &UavAvoidance::currentTargetCb, this);

        std::string priority_t = "inter_uav/priority_map";
        priority_sub_ = nh_.subscribe(priority_t, 10,
                                      &UavAvoidance::priorityMapCb, this);

        std::string heartbeat_t = "inter_uav/uav_status";
        heartbeat_sub_ = nh_.subscribe(heartbeat_t, 50,
                                       &UavAvoidance::heartbeatCb, this);

        // ========== 发布 ==========
        std::string out_t = "avoidance/cmd";
        avoidance_pub_ = nh_.advertise<multi_uav_strike::AvoidanceCmd>(out_t, 10);

        // ========== 定时器 ==========
        timer_ = nh_.createTimer(ros::Duration(1.0 / loop_freq_),
                                 &UavAvoidance::tickCb, this);

        have_self_pose_ = false;
        have_current_target_ = false;
    }

private:
    ros::NodeHandle nh_, nh_private_;

    ros::Subscriber self_pose_sub_;
    ros::Subscriber neighbor_sub_;
    ros::Subscriber current_target_sub_;
    ros::Subscriber priority_sub_;
    ros::Subscriber heartbeat_sub_;
    ros::Publisher  avoidance_pub_;
    ros::Timer      timer_;

    bool use_sim_;
    std::string uav_pose_topic_;
    std::string uav_id_;
    int uav_sn_priority_offset_;
    int self_task_priority_;
    double collision_threshold_;
    double proximity_threshold_;
    double vertical_safe_;
    double loop_freq_;
    int self_uav_num_;

    // 自位置
    bool have_self_pose_;
    geometry_msgs::PoseStamped self_pose_;

    // 当前航段终点
    bool have_current_target_;
    geometry_msgs::PoseStamped current_target_;

    // 邻居位姿(从 PoseArray 缓存)
    std::vector<geometry_msgs::Pose> neighbors_;

    // 邻居 priority + sn 表(简化存全部,找自己需要的)
    std::vector<multi_uav_strike::InterUavStatus::ConstPtr> heartbeats_;
    ros::Time last_heartbeat_purge_;

    int extractUavNumber(const std::string& id) {
        // "uav0" → 0, "uav12" → 12
        size_t pos = id.find_last_not_of("0123456789");
        if (pos == std::string::npos) return 0;
        std::string num = id.substr(pos + 1);
        return num.empty() ? 0 : std::stoi(num);
    }

    void selfPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        self_pose_ = *msg;
        have_self_pose_ = true;
    }

    void currentTargetCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_target_ = *msg;
        have_current_target_ = true;
    }

    void neighborCb(const geometry_msgs::PoseArray::ConstPtr& msg) {
        // PoseArray: position.x=lat, .y=lon, .z=alt(同 comm_node 约定)
        neighbors_.clear();
        neighbors_.reserve(msg->poses.size());
        for (const auto& p : msg->poses) {
            neighbors_.push_back(p);
        }
    }

    void priorityCb(const std_msgs::String::ConstPtr& msg) {
        // 暂未使用(留接口,完整 priority 由 heartbeat 提供)
    }

    void priorityMapCb(const std_msgs::String::ConstPtr& msg) {
        // 同上
        (void)msg;
    }

    void heartbeatCb(const multi_uav_strike::InterUavStatus::ConstPtr& msg) {
        // 缓存心跳用于查询 neighbor skill priority 与 sn
        // 清理过期
        ros::Time now = ros::Time::now();
        if ((now - last_heartbeat_purge_).toSec() > 1.0) {
            heartbeats_.erase(
                std::remove_if(heartbeats_.begin(), heartbeats_.end(),
                               [&](const multi_uav_strike::InterUavStatus::ConstPtr& h) {
                                   // h->timestamp_us 是微秒,转 ros::Time
                                   double age_sec = now.toSec() - static_cast<double>(h->timestamp_us) * 1e-6;
                                   return age_sec > 5.0;
                               }),
                heartbeats_.end());
            last_heartbeat_purge_ = now;
        }
        heartbeats_.push_back(msg);
    }

    // 邻居 SN 解析
    int neighborUavNumberFromSn(const std::string& sn) {
        return extractUavNumber(sn);
    }

    int neighborTaskPriorityFromSn(const std::string& sn) {
        for (const auto& h : heartbeats_) {
            if (h->sn == sn) {
                // 用 skill_type 作为粗优先级(100 系列 skill 优先 > 100 等)
                // 实际上 priority 由 skill.msg 提供,此版本暂用 skill_type
                return static_cast<int>(h->skill_type);
            }
        }
        return 0;  // 未知,默认低优
    }

    void tickCb(const ros::TimerEvent&) {
        multi_uav_strike::AvoidanceCmd cmd;
        cmd.lateral_blocked = true;
        cmd.lateral_max = 0.0;
        cmd.vertical_bias = 0.0;
        cmd.speed_scale = 1.0;
        cmd.reverse_allowed = false;

        if (!have_self_pose_ || neighbors_.empty()) {
            avoidance_pub_.publish(cmd);
            return;
        }

        // 取离我最近的一个邻居
        double min_dist = 1e9;
        size_t min_idx = 0;
        bool found = false;
        for (size_t i = 0; i < neighbors_.size(); ++i) {
            // neighbor: position.{x,y,z} = {lat,lon,alt} (与 comm_node 约定)
            // 距离粗算 (米): 1° lat ≈ 111000m, 1° lon ≈ 111000*cos(lat) m
            double dlat = (neighbors_[i].position.x - self_pose_.pose.position.x);
            double dlon = (neighbors_[i].position.y - self_pose_.pose.position.y);
            double dalt = (neighbors_[i].position.z - self_pose_.pose.position.z);
            // 注意: 自位置通常 NED(z=上为正),comm_node 转发也是同一坐标系
            // 距离 = sqrt((dlat*111000)^2 + (dlon*111000*cos(lat))^2 + dalt^2)
            double cos_lat = std::max(0.1, std::cos(self_pose_.pose.position.x * M_PI / 180.0));
            double dx = dlat * 111000.0;
            double dy = dlon * 111000.0 * cos_lat;
            double dist = std::sqrt(dx * dx + dy * dy + dalt * dalt);
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = i;
                found = true;
            }
        }

        if (!found || min_dist > 50.0) {
            // 50m 外:不需避障
            avoidance_pub_.publish(cmd);
            return;
        }

        // ========== 优先级判定 ==========
        // 简化:仅用 uav_id 数值排序(数值小优先)
        // TODO: 接 heartbeat 后改用 skill priority
        int other_num = min_idx;  // 没 sn 的占位;真实系统由 heartbeat 提供

        // 收集此邻居的 sn / task_priority (默认 100)
        std::string other_sn = std::to_string(other_num);
        int other_task_prio = neighborTaskPriorityFromSn(other_sn);
        int self_task_prio = self_task_priority_;

        bool i_am_lower_priority = false;
        if (self_task_prio < other_task_prio) {
            i_am_lower_priority = true;
        } else if (self_task_prio == other_task_prio) {
            if (self_uav_num_ > other_num) i_am_lower_priority = true;
        }

        if (!i_am_lower_priority) {
            // 我优先级 ≥ 对方,我直行,不需要任何减速
            avoidance_pub_.publish(cmd);
            return;
        }

        // ========== 我需避让 ==========
        double dalt_with_other = neighbors_[min_idx].position.z - self_pose_.pose.position.z;

        if (min_dist < collision_threshold_) {
            // 极近距离:优先调高度
            if (std::fabs(dalt_with_other) < vertical_safe_) {
                // 调不开高度 → 减速 / 停车 / 回退
                if (min_dist < collision_threshold_ * 0.6) {
                    cmd.speed_scale = 0.0;  // 完全停车
                    cmd.reverse_allowed = true;  // 极端情况允许回退
                } else {
                    cmd.speed_scale = 0.3;  // 慢行
                }
            } else {
                // 高度拉开,加速通过
                cmd.vertical_bias = (dalt_with_other > 0 ? -1.0 : 1.0) * 0.5;
                // 负高度差 → 对方在我下 → 我该往下压一点 → vertical_bias 负
            }
        } else if (min_dist < proximity_threshold_) {
            // 警告距离:减速
            cmd.speed_scale = 0.5;
            // 高度差如已经较大,微调一下确保不撞
            if (std::fabs(dalt_with_other) < vertical_safe_ * 0.5) {
                cmd.vertical_bias = 0.3 * (dalt_with_other > 0 ? -1.0 : 1.0);
            }
        }

        // 横向严禁 — 已经在 cmd.lateral_blocked=true / lateral_max=0.0

        avoidance_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_avoidance_node");
    UavAvoidance node;
    ros::spin();
    return 0;
}
