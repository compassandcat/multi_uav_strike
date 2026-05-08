/**
 * mission_manager_node.cpp
 * 任务管理器：核心状态机，协调已有模块执行三种工作模式
 *
 * 职责：
 * - 响应地面站工作模式指令
 * - 管理搜索、跟踪、打击流程
 * - 协调 gimbal_simulator + target_estimator + guidance_control
 * - 实现机间避障（搜索阶段全程）
 * - 触发毫米波雷达急停（低速时）
 *
 * 工作模式：
 * - MODE_SEARCH_ONLY: 全图搜索，识别目标只回传，不跟踪不打击
 * - MODE_SEARCH_TRACK: 搜索即跟踪，识别后锁定+螺旋定位，不打击
 * - MODE_SEARCH_STRIKE: 搜索即打击，定位完成→制导打击
 *
 * 重要说明：
 * - 工作模式由地面站预设下发，不是自动切换
 * - 无人机上电预加载航线，筒射完毕后开始执行
 *
 * 订阅：
 * - /mission/mode                    - 工作模式（来自 comm_node）
 * - /mission/waypoint_cmd           - 航点命令（来自 comm_node）
 * - /detection/yolo_result          - YOLO检测结果
 * - /target_los_angle               - 目标LOS角度（来自 gimbal_simulator）
 * - /target_estimated_pose           - 目标估计位置（来自 target_estimator）
 * - /target_estimated_twist         - 目标估计速度
 * - /inter_uav/other_uav_poses      - 邻居无人机位置
 * - /inter_uav/target_info           - 其他无人机发现的目标
 * - /mavros/local_position/pose     - 本机位置
 * - /detection/obstacle              - 毫米波雷达障碍检测
 *
 * 发布：
 * - /waypoint_executor/control      - 航点执行器控制命令
 * - /guidance/enable                - 制导使能
 * - /guidance/target_pose           - 制导目标
 * - /avoidance/vector               - 避障向量
 * - /emergency/stop                 - 急停命令
 * - /mission/status                 - 任务状态
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

#include <string>
#include <cmath>

// 工作模式枚举
enum class WorkMode {
    IDLE,
    SEARCH_ONLY,      // 全图搜索
    SEARCH_TRACK,     // 搜索即跟踪
    SEARCH_STRIKE     // 搜索即打击
};

// 任务状态
enum class TaskStatus {
    IDLE,
    EXECUTING_WAYPOINT,  // 执行航点
    TARGET_LOCKED,       // 目标锁定
    SPIRAL_APPROACH,     // 螺旋接近
    GUIDANCE_APPROACH,   // 制导接近
    STRIKE_EXECUTED,     // 打击完成
    EMERGENCY_STOP       // 急停
};

class MissionManager {
private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ============== 订阅 ==============
    ros::Subscriber mode_sub_;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber yolo_result_sub_;
    ros::Subscriber gimbal_los_sub_;
    ros::Subscriber target_est_pose_sub_;
    ros::Subscriber target_est_twist_sub_;
    ros::Subscriber other_uav_poses_sub_;
    ros::Subscriber inter_uav_target_sub_;
    ros::Subscriber self_pose_sub_;
    ros::Subscriber obstacle_sub_;

// ==============

    // ============== 发布 ==============
    ros::Publisher waypoint_control_pub_;
    ros::Publisher guidance_enable_pub_;
    ros::Publisher guidance_mode_pub_;       // 模式："strike" 或 "track"
    ros::Publisher guidance_target_pub_;
    ros::Publisher avoidance_vector_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher status_pub_;
    ros::Publisher uav_pose_nwu_pub_;     // NWU姿态发布(RViz用)

    // ============== 定时器 ==============
    ros::Timer mission_timer_;
    ros::Timer avoidance_timer_;

    // ============== 状态 ==============
    WorkMode current_work_mode_;
    TaskStatus current_task_status_;
    bool is_mission_started_;

    // 目标状态
    struct TargetState {
        bool is_detected;
        bool is_locked;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::TwistStamped twist;
        double lock_time;  // 锁定持续时间
        bool is_shared;    // 是否已共享给队友
    };
    TargetState current_target_;

    // 本机状态
    geometry_msgs::PoseStamped current_pose_;
    bool is_pose_received_;

    // 邻居无人机
    struct NeighborUav {
        std::string name;
        geometry_msgs::PoseStamped pose;
        ros::Time last_update;
    };
    std::vector<NeighborUav> neighbors_;

    // 避障参数
    double avoidance_safe_distance_;
    geometry_msgs::Point current_avoidance_vector_;

    // 毫米波避障参数
    double low_speed_threshold_;  // 12 m/s
    bool is_obstacle_detected_;
    double obstacle_distance_;

    // 螺旋接近参数
    double spiral_approach_radius_;
    double spiral_approach_angle_;
    bool is_spiral_active_;

    // 制导接近参数
    bool is_guidance_active_;
    double strike_distance_threshold_;  // 打击成功距离阈值

    // 参数
    double mission_loop_rate_;
    double target_lock_confidence_;  // 目标锁定所需置信度
    double spiral_approach_duration_; // 螺旋接近持续时间（秒）

public:
    MissionManager() : nh_private_("~"),
        current_work_mode_(WorkMode::IDLE),
        current_task_status_(TaskStatus::IDLE),
        is_mission_started_(false),
        is_pose_received_(false),
        avoidance_safe_distance_(10.0),
        is_obstacle_detected_(false),
        obstacle_distance_(100.0),
        low_speed_threshold_(12.0),
        is_spiral_active_(false),
        is_guidance_active_(false),
        strike_distance_threshold_(2.0),
        mission_loop_rate_(50.0),
        target_lock_confidence_(0.7),
        spiral_approach_duration_(10.0) {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();
        initTargetState();

        ROS_INFO("[MissionManager] Initialized. Low speed threshold: %.1f m/s", low_speed_threshold_);
    }

    void initParams() {
        nh_private_.param<double>("avoidance_safe_distance", avoidance_safe_distance_, 10.0);
        nh_private_.param<double>("low_speed_threshold", low_speed_threshold_, 12.0);
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);
        nh_private_.param<double>("mission_loop_rate", mission_loop_rate_, 50.0);
        nh_private_.param<double>("target_lock_confidence", target_lock_confidence_, 0.7);
        nh_private_.param<double>("spiral_approach_duration", spiral_approach_duration_, 10.0);
        nh_private_.param<double>("spiral_approach_radius", spiral_approach_radius_, 20.0);
    }

    void initSubscribers() {
        // 工作模式
        mode_sub_ = nh_.subscribe(
            "mission/mode", 10,
            &MissionManager::modeCallback, this);

        // 航点命令
        waypoint_sub_ = nh_.subscribe(
            "mission/waypoint_cmd", 10,
            &MissionManager::waypointCallback, this);

        // YOLO 检测结果
        yolo_result_sub_ = nh_.subscribe(
            "detection/yolo_result", 10,
            &MissionManager::yoloResultCallback, this);

        // 云台 LOS 角度
        gimbal_los_sub_ = nh_.subscribe(
            "target_los_angle", 10,
            &MissionManager::gimbalLosCallback, this);

        // 目标估计位置
        target_est_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &MissionManager::targetEstPoseCallback, this);

        // 目标估计速度
        target_est_twist_sub_ = nh_.subscribe(
            "target_estimated_twist", 10,
            &MissionManager::targetEstTwistCallback, this);

        // 邻居无人机位置
        other_uav_poses_sub_ = nh_.subscribe(
            "inter_uav/other_uav_poses", 10,
            &MissionManager::otherUavPosesCallback, this);

        // 其他无人机发现的目标
        inter_uav_target_sub_ = nh_.subscribe(
            "inter_uav/target_info", 10,
            &MissionManager::interUavTargetCallback, this);

        // 本机位置
        // TODO: 仿真阶段用 quad/pose，真机飞行时需改为 mavros/local_position/pose
        self_pose_sub_ = nh_.subscribe(
            "quad/pose", 10,
            &MissionManager::selfPoseCallback, this);

        // 毫米波雷达障碍检测
        obstacle_sub_ = nh_.subscribe(
            "detection/obstacle", 10,
            &MissionManager::obstacleCallback, this);
    }

    void initPublishers() {
        waypoint_control_pub_ = nh_.advertise<std_msgs::String>(
            "waypoint_executor/control", 10);

        guidance_enable_pub_ = nh_.advertise<std_msgs::Bool>(
            "guidance/enable", 10);

        guidance_mode_pub_ = nh_.advertise<std_msgs::String>(
            "guidance/mode", 10);

        guidance_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "guidance/target_pose", 10);

        avoidance_vector_pub_ = nh_.advertise<geometry_msgs::Point>(
            "avoidance/vector", 10);

        emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>(
            "emergency/stop", 10);

        status_pub_ = nh_.advertise<std_msgs::String>(
            "mission/status", 10);

        uav_pose_nwu_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "quad/pose_nwu", 10);  // NWU姿态发布(RViz用)
    }

    void initTimers() {
        mission_timer_ = nh_.createTimer(
            ros::Duration(1.0 / mission_loop_rate_),
            &MissionManager::missionTimerCallback, this);

        avoidance_timer_ = nh_.createTimer(
            ros::Duration(1.0 / mission_loop_rate_),
            &MissionManager::avoidanceTimerCallback, this);
    }

    void initTargetState() {
        current_target_.is_detected = false;
        current_target_.is_locked = true;
        current_target_.lock_time = 0.0;
        current_target_.is_shared = false;
    }

    // ============== 回调函数 ==============

    void modeCallback(const std_msgs::String::ConstPtr& msg) {
        std::string mode = msg->data;

        if (mode == "SEARCH_ONLY") {
            current_work_mode_ = WorkMode::SEARCH_ONLY;
            ROS_WARN("[MissionManager] Mode changed to SEARCH_ONLY - Guidance DISABLED");

            // SEARCH_ONLY 模式下禁用制导，只做航点飞行
            disableGuidance();

        } else if (mode == "SEARCH_TRACK") {
            current_work_mode_ = WorkMode::SEARCH_TRACK;
            ROS_WARN("[MissionManager] Mode changed to SEARCH_TRACK");
        } else if (mode == "SEARCH_STRIKE") {
            current_work_mode_ = WorkMode::SEARCH_STRIKE;
            ROS_WARN("[MissionManager] Mode changed to SEARCH_STRIKE");
        } else if (mode == "IDLE") {
            current_work_mode_ = WorkMode::IDLE;
            stopMission();
            disableGuidance();
        } else {
            ROS_WARN("[MissionManager] Unknown mode: %s", mode.c_str());
        }

        // 模式切换时重置目标状态
        initTargetState();
    }

    void waypointCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 航点由 waypoint_executor 处理，这里只做记录
        ROS_INFO("[MissionManager] Received %lu waypoints", msg->poses.size());
    }

    void yoloResultCallback(const std_msgs::String::ConstPtr& msg) {
        // 收到 YOLO 检测结果
        // 格式解析："class,confidence,x,y,z" 或自定义格式
        // 这里简化处理，实际应该解析具体的检测消息

        if (current_work_mode_ == WorkMode::IDLE) {
            return;
        }

        // 检测到目标
        if (!current_target_.is_detected) {
            current_target_.is_detected = true;
            ROS_WARN("[MissionManager] Target detected via YOLO");

            // 根据工作模式决定后续行为
            if (current_work_mode_ == WorkMode::SEARCH_ONLY) {
                // 只回传目标信息，不跟踪
                ROS_INFO("[MissionManager] SEARCH_ONLY: Target detected, reporting only");
                reportTargetToGs();
            } else if (current_work_mode_ == WorkMode::SEARCH_TRACK ||
                       current_work_mode_ == WorkMode::SEARCH_STRIKE) {
                // 锁定目标
                current_target_.is_locked = true;
                current_task_status_ = TaskStatus::TARGET_LOCKED;
                ROS_INFO("[MissionManager] Target locked, starting tracking");
            }
        }
    }

    void gimbalLosCallback(const geometry_msgs::Point::ConstPtr& msg) {
        // 云台 LOS 角度，用于判断跟踪精度
        // msg->z 是跟踪置信度
    }

    void targetEstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (current_target_.is_locked) {
            current_target_.pose = *msg;
        }
    }

    void targetEstTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if (current_target_.is_locked) {
            current_target_.twist = *msg;
        }
    }

    void otherUavPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        // 更新邻居无人机列表
        neighbors_.clear();
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            NeighborUav neighbor;
            neighbor.pose.header = msg->header;
            neighbor.pose.pose = msg->poses[i];
            neighbor.last_update = ros::Time::now();
            neighbors_.push_back(neighbor);
        }
    }

    void interUavTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 收到其他无人机发现的目标
        // 可以选择是否协同跟踪
        ROS_INFO("[MissionManager] Received target info from other UAV: %s",
                 msg->header.frame_id.c_str());
    }

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        is_pose_received_ = true;

        // 发布NWU姿态用于RViz显示
        geometry_msgs::PoseStamped uav_pose_nwu = *msg;
        // NED -> NWU 转换
        uav_pose_nwu.pose.position.y = -uav_pose_nwu.pose.position.y;
        uav_pose_nwu.pose.position.z = -uav_pose_nwu.pose.position.z;
        // 四元数: w,x 不变, y,z 取反
        uav_pose_nwu.pose.orientation.y = -uav_pose_nwu.pose.orientation.y;
        uav_pose_nwu.pose.orientation.z = -uav_pose_nwu.pose.orientation.z;
        uav_pose_nwu.header.stamp = ros::Time::now();
        uav_pose_nwu.header.frame_id = "map";  // RViz
        uav_pose_nwu_pub_.publish(uav_pose_nwu);
    }

    void obstacleCallback(const std_msgs::String::ConstPtr& msg) {
        // 毫米波雷达障碍检测
        // 格式："distance,angle" 或自定义
        // 简化处理
        if (msg->data.empty()) {
            is_obstacle_detected_ = false;
            return;
        }

        // 解析障碍物距离
        try {
            obstacle_distance_ = std::stod(msg->data);
            is_obstacle_detected_ = (obstacle_distance_ < 30.0);  // 30米内认为有障碍
        } catch (...) {
            is_obstacle_detected_ = false;
        }
    }

    // ============== 定时器回调 ==============

    void missionTimerCallback(const ros::TimerEvent&) {
        if (!is_pose_received_) {
            return;
        }

        // 检查急停条件（毫米波雷达）
        checkEmergencyStop();

        // 根据工作模式执行对应行为
        switch (current_work_mode_) {
            case WorkMode::IDLE:
                handleIdle();
                break;
            case WorkMode::SEARCH_ONLY:
                handleSearchOnly();
                break;
            case WorkMode::SEARCH_TRACK:
                handleSearchTrack();
                break;
            case WorkMode::SEARCH_STRIKE:
                handleSearchStrike();
                break;
        }

        // 发布状态
        publishStatus();
    }

    void avoidanceTimerCallback(const ros::TimerEvent&) {
        // 搜索阶段全程计算机间避障
        if (current_work_mode_ == WorkMode::SEARCH_ONLY ||
            current_work_mode_ == WorkMode::SEARCH_TRACK ||
            current_work_mode_ == WorkMode::SEARCH_STRIKE) {
            computeAndPublishAvoidanceVector();
        }
    }

    // ============== 任务处理 ==============

    void handleIdle() {
        current_task_status_ = TaskStatus::IDLE;
    }

    void handleSearchOnly() {
        // 全图搜索模式
        current_task_status_ = TaskStatus::EXECUTING_WAYPOINT;

        // 目标检测后只回传，不做其他处理
        if (current_target_.is_detected && !current_target_.is_shared) {
            reportTargetToGs();
            current_target_.is_shared = true;
        }
    }

    void handleSearchTrack() {
        // 搜索即跟踪模式
        // 如果已启动制导，不应再设置 waypoint 模式
        if (is_guidance_active_) {
            return;
        }

        if (!current_target_.is_locked) {
            // 目标未锁定，执行航点
            current_task_status_ = TaskStatus::EXECUTING_WAYPOINT;
        } else {
            // 目标已锁定，直接启动制导接近
            startGuidanceApproach();
        }
    }

    void handleSearchStrike() {
        // 搜索即打击模式
        // 如果已启动制导，不应再设置 waypoint 模式
        if (is_guidance_active_) {
            return;
        }

        if (!current_target_.is_locked) {
            // 目标未锁定，执行航点
            current_task_status_ = TaskStatus::EXECUTING_WAYPOINT;
        } else {
            // 目标已锁定，直接启动制导接近
            startGuidanceApproach();
        }
    }

    void startSpiralApproach() {
        if (!current_target_.is_locked) {
            return;
        }

        is_spiral_active_ = true;
        current_task_status_ = TaskStatus::SPIRAL_APPROACH;

        // 停止航点执行
        std_msgs::String cmd;
        cmd.data = "stop";
        waypoint_control_pub_.publish(cmd);

        // 向 waypoint_executor 发送螺旋接近命令
        cmd.data = "spiral:" +
                   std::to_string(current_target_.pose.pose.position.x) + "," +
                   std::to_string(current_target_.pose.pose.position.y) + "," +
                   std::to_string(current_target_.pose.pose.position.z) + "," +
                   std::to_string(spiral_approach_radius_);
        waypoint_control_pub_.publish(cmd);

        ROS_INFO("[MissionManager] Starting spiral approach to target");
    }

    bool is_spiral_complete_ = false;

    void startGuidanceApproach() {
        is_guidance_active_ = true;
        current_task_status_ = TaskStatus::GUIDANCE_APPROACH;

        // 停止航点执行，避免和 guidance 冲突
        ROS_WARN("[MissionManager] >>>>> stop command about to be published to waypoint_executor");
        std_msgs::String cmd;
        cmd.data = "stop";
        waypoint_control_pub_.publish(cmd);
        ROS_WARN("[MissionManager] >>>>> stop command published");

        // 确定模式：SEARCH_STRIKE -> "strike", SEARCH_TRACK -> "track"
        std::string guidance_mode = (current_work_mode_ == WorkMode::SEARCH_STRIKE) ? "strike" : "track";

        // 使能制导
        std_msgs::Bool enable;
        enable.data = true;
        guidance_enable_pub_.publish(enable);

        // 发送模式
        std_msgs::String mode_msg;
        mode_msg.data = guidance_mode;
        guidance_mode_pub_.publish(mode_msg);

        // 发送目标给制导
        guidance_target_pub_.publish(current_target_.pose);

        ROS_WARN_THROTTLE(5.0, "[MissionManager] Starting guidance approach to target (mode: %s)", guidance_mode.c_str());
    }

    void disableGuidance() {
        is_guidance_active_ = false;

        // 禁用制导
        std_msgs::Bool enable;
        enable.data = false;
        guidance_enable_pub_.publish(enable);

        ROS_INFO("[MissionManager] Guidance disabled for SEARCH_ONLY mode");
    }

    void checkEmergencyStop() {
        // 低速时检查毫米波雷达
        // TODO: 获取当前速度判断是否低于阈值

        if (is_obstacle_detected_ && obstacle_distance_ < 10.0) {
            ROS_WARN("[MissionManager] OBSTACLE DETECTED! Emergency stop!");

            // 发布急停命令
            std_msgs::Bool stop;
            stop.data = true;
            emergency_stop_pub_.publish(stop);

            current_task_status_ = TaskStatus::EMERGENCY_STOP;
            is_mission_started_ = false;
        }
    }

    void computeAndPublishAvoidanceVector() {
        geometry_msgs::Point avoidance_vec;
        avoidance_vec.x = 0.0;
        avoidance_vec.y = 0.0;
        avoidance_vec.z = 0.0;

        if (neighbors_.empty()) {
            avoidance_vector_pub_.publish(avoidance_vec);
            return;
        }

        // 计算机间避障向量（人工势场法）
        for (const auto& neighbor : neighbors_) {
            double dx = current_pose_.pose.position.x - neighbor.pose.pose.position.x;
            double dy = current_pose_.pose.position.y - neighbor.pose.pose.position.y;
            double dz = current_pose_.pose.position.z - neighbor.pose.pose.position.z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);

            if (dist < avoidance_safe_distance_ && dist > 0.1) {
                // 距离越近，斥力越大
                double force = (avoidance_safe_distance_ - dist) / dist;
                avoidance_vec.x += force * dx;
                avoidance_vec.y += force * dy;
                avoidance_vec.z += force * dz;
            }
        }

        // 限制最大避障向量
        double max_avoidance = 5.0;
        double mag = sqrt(avoidance_vec.x*avoidance_vec.x +
                          avoidance_vec.y*avoidance_vec.y +
                          avoidance_vec.z*avoidance_vec.z);
        if (mag > max_avoidance) {
            avoidance_vec.x *= max_avoidance / mag;
            avoidance_vec.y *= max_avoidance / mag;
            avoidance_vec.z *= max_avoidance / mag;
        }

        current_avoidance_vector_ = avoidance_vec;
        avoidance_vector_pub_.publish(avoidance_vec);
    }

    void reportTargetToGs() {
        // 通过 comm_node 回传目标信息（由 comm_node 处理）
        // 这里只是标记
        ROS_INFO("[MissionManager] Reporting target to GS");
    }

    void stopMission() {
        ROS_WARN("[MissionManager] Mission stopped");

        // 停止所有子模块
        is_mission_started_ = false;
        is_spiral_active_ = false;
        is_guidance_active_ = false;

        // 禁用制导
        std_msgs::Bool enable;
        enable.data = false;
        guidance_enable_pub_.publish(enable);

        // 重置目标状态
        initTargetState();

        current_task_status_ = TaskStatus::IDLE;
    }

    void publishStatus() {
        std_msgs::String status;

        std::ostringstream oss;
        oss << "mode:" << workModeToString() << ";"
            << "status:" << taskStatusToString() << ";"
            << "target_locked:" << (current_target_.is_locked ? "true" : "false") << ";"
            << "spiral_active:" << (is_spiral_active_ ? "true" : "false") << ";"
            << "guidance_active:" << (is_guidance_active_ ? "true" : "false") << ";"
            << "neighbors:" << neighbors_.size();

        status.data = oss.str();
        status_pub_.publish(status);
    }

    std::string workModeToString() {
        switch (current_work_mode_) {
            case WorkMode::IDLE: return "IDLE";
            case WorkMode::SEARCH_ONLY: return "SEARCH_ONLY";
            case WorkMode::SEARCH_TRACK: return "SEARCH_TRACK";
            case WorkMode::SEARCH_STRIKE: return "SEARCH_STRIKE";
            default: return "UNKNOWN";
        }
    }

    std::string taskStatusToString() {
        switch (current_task_status_) {
            case TaskStatus::IDLE: return "IDLE";
            case TaskStatus::EXECUTING_WAYPOINT: return "EXECUTING_WAYPOINT";
            case TaskStatus::TARGET_LOCKED: return "TARGET_LOCKED";
            case TaskStatus::SPIRAL_APPROACH: return "SPIRAL_APPROACH";
            case TaskStatus::GUIDANCE_APPROACH: return "GUIDANCE_APPROACH";
            case TaskStatus::STRIKE_EXECUTED: return "STRIKE_EXECUTED";
            case TaskStatus::EMERGENCY_STOP: return "EMERGENCY_STOP";
            default: return "UNKNOWN";
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_manager_node");
    MissionManager manager;
    ros::spin();
    return 0;
}
