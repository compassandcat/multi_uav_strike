/**
 * comm_node.cpp
 * 通信节点：统一处理机间通信和地面站通信
 *
 * 职责：
 * - 接收地面站指令（工作模式、航点列表）
 * - 广播本机位置给其他无人机
 * - 汇总邻居无人机位置
 * - 共享目标发现信息
 *
 * 订阅：
 * - /mavros/local_position/pose    - 本机位置（来自飞控）
 * - /inter_uav/other_uav_poses     - 邻居无人机位置
 * - /gs/mode_cmd                   - 地面站工作模式指令
 * - /gs/waypoint_upload            - 地面站航点列表
 * - /detection/yolo_result         - YOLO检测结果（目标发现）
 * - /target_estimated_pose         - 目标估计位置（共享给队友）
 *
 * 发布：
 * - /inter_uav/self_pose           - 本机位置（发给其他无人机）
 * - /inter_uav/other_uav_poses    - 邻居无人机位置列表
 * - /inter_uav/target_info         - 目标信息（共享给队友）
 * - /gs/telemetry                  - 无人机状态回传
 * - /gs/target_report              - 目标检测/定位报告
 * - /mission/mode                   - 工作模式（转发地面站指令）
 * - /mission/waypoint_cmd           - 航点命令（转发地面站）
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

// 自定义消息：目标信息
// 后续可扩展为 custom_msgs/TargetInfo
#include <vector>
#include <string>

class CommNode {
private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ============== 订阅 ==============
    ros::Subscriber self_pose_sub_;          // 本机位置（来自飞控）
    ros::Subscriber other_uav_poses_sub_;    // 邻居无人机位置
    ros::Subscriber gs_mode_cmd_sub_;        // 地面站工作模式
    ros::Subscriber gs_waypoint_sub_;        // 地面站航点列表
    ros::Subscriber yolo_result_sub_;        // YOLO检测结果
    ros::Subscriber target_est_pose_sub_;    // 目标估计位置

    // ============== 发布 ==============
    ros::Publisher self_pose_pub_;           // 本机位置（发给其他无人机）
    ros::Publisher other_uav_poses_pub_;     // 邻居无人机位置列表
    ros::Publisher target_info_pub_;         // 目标信息
    ros::Publisher gs_telemetry_pub_;        // 无人机状态回传
    ros::Publisher gs_target_report_pub_;    // 目标报告
    ros::Publisher mission_mode_pub_;        // 工作模式（转发）
    ros::Publisher mission_waypoint_pub_;    // 航点命令（转发）

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

    // 地面站指令缓冲
    std::string current_work_mode_ = "SEARCH_ONLY";
    nav_msgs::Path current_waypoints_;
    bool is_waypoints_received_ = false;

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
    ros::Timer telemetry_timer_;     // 状态回传定时器
    ros::Timer neighbor_check_timer_; // 邻居检查定时器

    // 参数
    std::string uav_name_;
    int telemetry_rate_;  // Hz
    bool use_sim_;        // 仿真/真机切换

public:
    CommNode() : nh_private_("~"),
        neighbor_timeout_(5.0),
        target_share_interval_(1.0),
        telemetry_rate_(10) {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();

        ROS_INFO("[Comm] CommNode initialized for UAV: %s", uav_name_.c_str());
        ROS_INFO("[Comm] Telemetry rate: %d Hz", telemetry_rate_);
    }

    void initParams() {
        nh_private_.param<std::string>("uav_name", uav_name_, "uav0");
        nh_private_.param<double>("neighbor_timeout", neighbor_timeout_, 5.0);
        nh_private_.param<double>("target_share_interval", target_share_interval_, 1.0);
        nh_private_.param<int>("telemetry_rate", telemetry_rate_, 10);
        nh_private_.param<bool>("use_sim", use_sim_, true);
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

        // 地面站工作模式指令
        gs_mode_cmd_sub_ = nh_.subscribe(
            "/gs/mode_cmd", 10,
            &CommNode::gsModeCmdCallback, this);

        // 地面站航点列表
        gs_waypoint_sub_ = nh_.subscribe(
            "gs/waypoint_upload", 10,
            &CommNode::gsWaypointCallback, this);

        // YOLO 检测结果（目标发现）
        yolo_result_sub_ = nh_.subscribe(
            "detection/yolo_result", 10,
            &CommNode::yoloResultCallback, this);

        // 目标估计位置（用于共享给队友）
        target_est_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &CommNode::targetEstPoseCallback, this);
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

        // 无人机状态回传地面站
        gs_telemetry_pub_ = nh_.advertise<std_msgs::String>(
            "gs/telemetry", 10);

        // 目标检测/定位报告回传地面站
        gs_target_report_pub_ = nh_.advertise<std_msgs::String>(
            "gs/target_report", 10);

        // 工作模式（转发给 mission_manager）
        mission_mode_pub_ = nh_.advertise<std_msgs::String>(
            "mission/mode", 10);

        // 航点命令（转发给 waypoint_executor）
        mission_waypoint_pub_ = nh_.advertise<nav_msgs::Path>(
            "mission/waypoint_cmd", 10);
    }

    void initTimers() {
        // 状态回传定时器
        telemetry_timer_ = nh_.createTimer(
            ros::Duration(1.0 / telemetry_rate_),
            &CommNode::telemetryTimerCallback, this);

        // 邻居检查定时器（1Hz）
        neighbor_check_timer_ = nh_.createTimer(
            ros::Duration(1.0),
            &CommNode::neighborCheckTimerCallback, this);
    }

    // ============== 回调函数 ==============

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (use_sim_) {
            // 仿真输入是 NED 坐标系，直接使用
            current_self_pose_ = *msg;
        } else {
            // Mavros 输入是 ENU 坐标系，需要转换为 NED
            // ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            current_self_pose_.pose.position.x = msg->pose.position.y;
            current_self_pose_.pose.position.y = msg->pose.position.x;
            current_self_pose_.pose.position.z = -msg->pose.position.z;
            // 四元数 ENU->NED: w,x 不变, y,z 取反
            current_self_pose_.pose.orientation.w = msg->pose.orientation.w;
            current_self_pose_.pose.orientation.x = msg->pose.orientation.x;
            current_self_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_self_pose_.pose.orientation.z = -msg->pose.orientation.z;
            current_self_pose_.header = msg->header;
        }
        is_self_pose_received_ = true;

        // 设置 frame_id 为本机名，订阅方通过此过滤自己的消息
        current_self_pose_.header.frame_id = uav_name_;

        // 广播本机位置给其他无人机（全局 topic）
        self_pose_pub_.publish(current_self_pose_);
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

    void gsModeCmdCallback(const std_msgs::String::ConstPtr& msg) {
        current_work_mode_ = msg->data;
        ROS_INFO("[Comm] Received mode command from GS: %s", current_work_mode_.c_str());

        // 转发给 mission_manager
        std_msgs::String mode_msg;
        mode_msg.data = current_work_mode_;
        mission_mode_pub_.publish(mode_msg);
    }

    void gsWaypointCallback(const nav_msgs::Path::ConstPtr& msg) {
        current_waypoints_ = *msg;
        is_waypoints_received_ = true;
        ROS_INFO("[Comm] Received %lu waypoints from GS", msg->poses.size());

        // 转发给 waypoint_executor
        mission_waypoint_pub_.publish(current_waypoints_);
    }

    void yoloResultCallback(const std_msgs::String::ConstPtr& msg) {
        // 收到 YOLO 检测结果，记录目标信息
        // 格式： "class,confidence,x,y,z" 或简单处理
        // TODO: 定义具体的 YOLO 消息格式

        // ROS_INFO("[Comm] YOLO detection: %s", msg->data.c_str());

        // 这里可以解析并存储目标信息，后续共享
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

        // 回传地面站
        reportTargetToGs(*msg);
    }

    // ============== 定时器回调 ==============

    void telemetryTimerCallback(const ros::TimerEvent&) {
        if (!is_self_pose_received_) {
            return;
        }

        // 组装遥测数据
        std_msgs::String telemetry;
        std::ostringstream oss;
        oss << "uav:" << uav_name_ << ";"
            << "x:" << current_self_pose_.pose.position.x << ";"
            << "y:" << current_self_pose_.pose.position.y << ";"
            << "z:" << current_self_pose_.pose.position.z << ";"
            << "mode:" << current_work_mode_ << ";"
            << "neighbors:" << neighbors_.size();
        telemetry.data = oss.str();

        gs_telemetry_pub_.publish(telemetry);
    }

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

    void reportTargetToGs(const geometry_msgs::PoseStamped& target_pose) {
        std_msgs::String report;
        std::ostringstream oss;
        oss << "target_found:"
            << "x:" << target_pose.pose.position.x << ";"
            << "y:" << target_pose.pose.position.y << ";"
            << "z:" << target_pose.pose.position.z << ";"
            << "detected_by:" << uav_name_;
        report.data = oss.str();

        gs_target_report_pub_.publish(report);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "comm_node");
    CommNode comm_node;
    ros::spin();
    return 0;
}
