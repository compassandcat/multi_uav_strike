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
#include "mavros_msgs/HomePosition.h"

class UavAvoidance {
public:
    UavAvoidance() : nh_(), nh_private_("~") {
        nh_private_.param<bool>("use_sim", use_sim_, true);
        nh_private_.param<std::string>("uav_pose_topic", uav_pose_topic_, "");
        if (uav_pose_topic_.empty()) {
            uav_pose_topic_ = use_sim_ ? "quad/pose" : "mavros/local_position/pose";
        }
        nh_private_.param<std::string>("uav_id", uav_id_, "uav0");
        // uav_device_sn:本机 device_sn;空 → 回落到 uav_id("uav0")
        // 与 comm_node 的 ~uav_device_sn 一致,这样心跳的 sn 和这里的 sn 是同一字符串
        nh_private_.param<std::string>("uav_device_sn", uav_device_sn_, uav_id_);
        nh_private_.param<int>("uav_sn_priority_offset", uav_sn_priority_offset_, 0);
        nh_private_.param<int>("self_task_priority", self_task_priority_, 100);
        nh_private_.param<double>("collision_threshold", collision_threshold_, 8.0);
        nh_private_.param<double>("proximity_threshold", proximity_threshold_, 15.0);
        nh_private_.param<double>("vertical_safe", vertical_safe_, 5.0);
        nh_private_.param<double>("loop_freq", loop_freq_, 20.0);

        // 从 uav_id 提取数字(旧 tiebreaker 路径,保留向后兼容)
        self_uav_num_ = extractUavNumber(uav_id_);
        // device_sn 用于 tiebreaker(字典序,"1-1" 优先于 "1-2")

        ROS_INFO("[UavAvoidance] self_id=%s device_sn=%s num=%d priority_offset=%d self_task_prio=%d",
                 uav_id_.c_str(), uav_device_sn_.c_str(),
                 self_uav_num_, uav_sn_priority_offset_, self_task_priority_);

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

        // 心跳是 comm_node 在全局 topic "/inter_uav/uav_status" 上发(绝对路径),
        // 这里是相对路径会落在 /<ns>/inter_uav/uav_status 上,收不到。
        std::string heartbeat_t = "/inter_uav/uav_status";
        heartbeat_sub_ = nh_.subscribe(heartbeat_t, 50,
                                       &UavAvoidance::heartbeatCb, this);

        // 订阅 PX4 home — self_pose 是 NED(米),neighbors 是 GPS(度),距离公式需要同一坐标空间。
        // 用 home 把 self_pose NED → GPS,后续 distance 用 lat/lon 公式才对。
        home_sub_ = nh_.subscribe("mavros/home_position/home", 1,
                                  &UavAvoidance::homeCb, this);
        have_home_ = false;

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
    ros::Subscriber home_sub_;
    ros::Publisher  avoidance_pub_;
    ros::Timer      timer_;

    bool use_sim_;
    std::string uav_pose_topic_;
    std::string uav_id_;
    // 自身 device_sn(命名约定 "1-1" / "1-2"...),默认回落到 uav_id
    // 用作心跳里 sn 的查找键,以及优先级相同时的 tiebreaker(字典序)
    std::string uav_device_sn_;
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

    // PX4 home (用于 NED→GPS 转换,见 tickCb)
    bool have_home_;
    double home_lat_, home_lon_, home_alt_;

    // 当前航段终点
    bool have_current_target_;
    geometry_msgs::PoseStamped current_target_;

    // 邻居位姿(从 PoseArray 缓存)+ 对应 device_sn(由 heartbeat cache 按 lat/lon 反查)
    struct NeighborInfo {
        geometry_msgs::Pose pose;  // position.{x,y,z} = {lat,lon,alt}
        std::string sn;            // 来自 heartbeat,空 = 未匹配上(对方还没推心跳)
    };
    std::vector<NeighborInfo> neighbors_;

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
        if (!have_self_pose_) {
            ROS_WARN("[UavAvoidance] first self_pose received (NED x=%.2f y=%.2f z=%.2f)",
                     msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
        have_self_pose_ = true;
    }

    void homeCb(const mavros_msgs::HomePosition::ConstPtr& msg) {
        // PX4 home 一次性锁定 — 用于把 self_pose NED(m) 投影到 GPS(deg),与邻居同坐标空间
        if (have_home_) return;
        const double lat = msg->geo.latitude;
        const double lon = msg->geo.longitude;
        if (std::fabs(lat) < 1e-6 && std::fabs(lon) < 1e-6) return;
        home_lat_ = lat;
        home_lon_ = lon;
        home_alt_ = msg->geo.altitude;
        have_home_ = true;
        ROS_WARN("[UavAvoidance] home locked: lat=%.7f lon=%.7f alt=%.2f",
                 home_lat_, home_lon_, home_alt_);
    }

    void currentTargetCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_target_ = *msg;
        have_current_target_ = true;
    }

    void neighborCb(const geometry_msgs::PoseArray::ConstPtr& msg) {
        // PoseArray: position.x=lat, .y=lon, .z=alt(同 comm_node 约定)
        if (neighbors_.empty() && !msg->poses.empty()) {
            ROS_WARN("[UavAvoidance] first neighbor batch: %lu poses (sample: lat=%.6f lon=%.6f alt=%.1f)",
                     msg->poses.size(),
                     msg->poses[0].position.x, msg->poses[0].position.y, msg->poses[0].position.z);
        }
        neighbors_.clear();
        neighbors_.reserve(msg->poses.size());
        for (const auto& p : msg->poses) {
            NeighborInfo n;
            n.pose = p;
            // 用 lat/lon 反查 heartbeat cache,拿到对方 device_sn
            // heartbeat 里 sn/lat/lon 是对方的真实标识和位置,与 PoseArray 里的 lat/lon 是同一个数
            n.sn = lookupSnByLatLon(p.position.x, p.position.y);
            neighbors_.push_back(n);
        }
    }

    /**
     * 用 lat/lon 在 heartbeat cache 里找匹配的 device_sn。
     * 找不到返回空串(对方心跳还没到,后续 tickCb 会走"sn 缺失"分支,保守不避让)。
     */
    std::string lookupSnByLatLon(double lat, double lon) {
        // 1° lat ≈ 111000 m → 10m 容差 ≈ 9e-5 deg,够区分相邻 UAV
        constexpr double TOL = 1e-4;
        for (const auto& h : heartbeats_) {
            if (std::fabs(h->lat - lat) < TOL && std::fabs(h->lon - lon) < TOL) {
                return h->sn;
            }
        }
        return "";
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

        // === 每 1s 打一次 tick 状态摘要,方便诊断哪一步没到位 ===
        ROS_INFO_THROTTLE(1.0,
            "[UavAvoidance] tick: self_pose=%d home=%d neighbors=%lu thresholds(coll=%.1f/prox=%.1f) self_prio=%d",
            have_self_pose_, have_home_, neighbors_.size(),
            collision_threshold_, proximity_threshold_, self_task_priority_);

        if (!have_self_pose_ || neighbors_.empty()) {
            avoidance_pub_.publish(cmd);
            return;
        }

        // ========== 把 self_pose NED(m) 投影到 GPS(deg) ==========
        // neighbors 是 GPS(deg),distance 公式需要两边同坐标空间。
        // home 还没到 → 距离算不出来 → 跳过避障(给默认 cmd)。
        if (!have_home_) {
            avoidance_pub_.publish(cmd);
            return;
        }
        const double cos_home_lat = std::max(0.1, std::cos(home_lat_ * M_PI / 180.0));
        const double self_lat = home_lat_ + self_pose_.pose.position.x / 111000.0;
        const double self_lon = home_lon_ + self_pose_.pose.position.y / (111000.0 * cos_home_lat);
        const double self_alt = home_alt_ + self_pose_.pose.position.z;  // NED: z=down → 反号

        // 取离我最近的一个邻居
        double min_dist = 1e9;
        size_t min_idx = 0;
        bool found = false;
        for (size_t i = 0; i < neighbors_.size(); ++i) {
            // neighbor: position.{x,y,z} = {lat,lon,alt} (与 comm_node 约定)
            // 自位置也已转换到 GPS(deg),距离 = sqrt((dlat*111000)^2 + (dlon*111000*cos(lat))^2 + dalt^2)
            double dlat = (neighbors_[i].pose.position.x - self_lat);
            double dlon = (neighbors_[i].pose.position.y - self_lon);
            double dalt = (neighbors_[i].pose.position.z - self_alt);
            double dx = dlat * 111000.0;
            double dy = dlon * 111000.0 * cos_home_lat;
            double dist = std::sqrt(dx * dx + dy * dy + dalt * dalt);
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = i;
                found = true;
            }
        }

        if (!found || min_dist > 50.0) {
            // 50m 外:不需避障
            // 调试:发现 UAV 离得很近但仍 > 50m 时,打出来 min_dist 方便排查公式 bug
            ROS_INFO_THROTTLE(2.0,
                "[UavAvoidance] no-avoid: min_dist=%.2fm (>50m, thresholds prox=%.1f/coll=%.1f)",
                min_dist, proximity_threshold_, collision_threshold_);
            avoidance_pub_.publish(cmd);
            return;
        }

        // ========== 优先级判定 ==========
        // 对方 device_sn 来自 heartbeat cache(neighborCb 时已按 lat/lon 反查,
        // 但心跳可能比 PoseArray 晚到,所以这里 sn 为空时再查一次)
        std::string other_sn = neighbors_[min_idx].sn;
        if (other_sn.empty()) {
            other_sn = lookupSnByLatLon(neighbors_[min_idx].pose.position.x,
                                        neighbors_[min_idx].pose.position.y);
            if (!other_sn.empty()) {
                // 回填:后续 tick 不必再查
                neighbors_[min_idx].sn = other_sn;
            } else {
                ROS_INFO_THROTTLE(2.0,
                    "[UavAvoidance] no-avoid (no sn yet): dist=%.2fm, neighbor heartbeat not matched",
                    min_dist);
                avoidance_pub_.publish(cmd);
                return;
            }
        }
        int other_task_prio = neighborTaskPriorityFromSn(other_sn);
        int self_task_prio = self_task_priority_;

        bool i_am_lower_priority = false;
        if (self_task_prio < other_task_prio) {
            i_am_lower_priority = true;
        } else if (self_task_prio == other_task_prio) {
            // tiebreaker: device_sn 字典序 — "1-1" < "1-2" < "2-1"
            // sn 大的让 sn 小的(我若 sn 更大 → 我让)
            if (uav_device_sn_ > other_sn) i_am_lower_priority = true;
        }

        if (!i_am_lower_priority) {
            // 我优先级 ≥ 对方,我直行,不需要任何减速
            // 调试:即便距离够近,我方优先级较高时不避让
            ROS_INFO_THROTTLE(2.0,
                "[UavAvoidance] no-avoid (priority): self_prio=%d >= other_prio=%d dist=%.2fm",
                self_task_priority_, other_task_prio, min_dist);
            avoidance_pub_.publish(cmd);
            return;
        }

        // ========== 我需避让 ==========
        // 用 GPS 高度差,与 neighbor.pose.position.z (alt) 单位一致(都是 m AMSL)
        double dalt_with_other = neighbors_[min_idx].pose.position.z - self_alt;

        if (min_dist < collision_threshold_) {
            // 极近距离:优先调高度
            if (std::fabs(dalt_with_other) < vertical_safe_) {
                // 高度差小 → 主动拉垂直分离(关键:不能让低优先级 UAV 变成静止障碍物)
                // 方向:本机 device_sn 哈希决定,保证双向避免同向漂移。
                // 高优先级 UAV 不调高度(我让它),他从原高度穿过去,我在 5s 内爬/降出 vertical_safe 米
                bool i_go_up = (std::hash<std::string>{}(uav_device_sn_) & 1) == 0;
                cmd.vertical_bias = i_go_up ? +1.5 : -1.5;

                if (min_dist < collision_threshold_ * 0.6) {
                    // 极近:停水平让垂直分离时间(给 5s 拉到 vertical_safe=5m 够用)
                    cmd.speed_scale = 0.0;
                    cmd.reverse_allowed = true;  // 兜底:若垂直来不及,允许回退(暂未实现)
                } else {
                    cmd.speed_scale = 0.2;  // 慢行(不停,留点前进降低相对速度)
                }
            } else {
                // 高度已拉开,顺方向加速通过
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

        // 只在避障真正触发时打印(默认全零的 cmd 不打,免得 20Hz 噪声)
        if (cmd.speed_scale < 1.0 || std::fabs(cmd.vertical_bias) > 1e-3) {
            ROS_WARN_THROTTLE(1.0,
                "[UavAvoidance] AVOID neighbor#%zu dist=%.1fm dalt=%.1fm speed_scale=%.2f vert_bias=%.2f reverse=%d (thresholds coll=%.1f prox=%.1f vert_safe=%.1f)",
                min_idx, min_dist, dalt_with_other, cmd.speed_scale, cmd.vertical_bias, cmd.reverse_allowed,
                collision_threshold_, proximity_threshold_, vertical_safe_);
        }

        avoidance_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_avoidance_node");
    UavAvoidance node;
    ros::spin();
    return 0;
}
