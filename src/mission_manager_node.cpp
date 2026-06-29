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
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <string>
#include <cmath>
#include <thread>
#include <chrono>

// 工作模式枚举
enum class WorkMode {
    IDLE,             // 等待：只获取 UAV 信息，不发任何控制指令（PX4 SITL 不发 setpoint）
    TAKEOFF,          // 起飞：执行 PX4 SITL 起飞流程（OFFBOARD + ARM + 爬升），完成后保持悬停
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

// PX4 SITL 起飞状态
enum class TakeoffState {
    TAKEOFF_IDLE,           // 空闲状态，等待开始起飞
    TAKEOFF_WAITING_FCU,   // 等待 FCU 连接
    TAKEOFF_SETTING_OFFBOARD, // 正在切换 OFFBOARD 模式
    TAKEOFF_ARMING,        // 正在解锁
    TAKEOFF_TAKEOFF_EXEC,  // 执行起飞爬升
    TAKEOFF_HOVERING,      // 悬停等待
    TAKEOFF_COMPLETE,      // 起飞完成
    TAKEOFF_FAILED         // 起飞失败
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
    ros::Subscriber mavros_state_sub_;    // PX4 SITL: 飞控状态

    // ============== Service Client ==============
    ros::ServiceClient set_mode_client_; // PX4 SITL: 模式切换
    ros::ServiceClient arming_client_;  // PX4 SITL: 解锁

    // ============== PX4 起飞 ==============
    ros::Publisher takeoff_setpoint_pub_; // PX4 SITL: 位置 setpoint 发布器

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
        double ned_x, ned_y, ned_z;  // 本地 NED 坐标（机间避障用）
        ros::Time last_update;
    };
    std::vector<NeighborUav> neighbors_;

    // GPS 参考点（机间避障时将邻居 GPS 转本地 NED）
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;

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

    // 仿真/真机切换
    bool use_sim_;
    std::string pose_topic_;

    // PX4 SITL 起飞状态
    TakeoffState takeoff_state_;
    bool is_px4_connected_;       // FCU 是否连接
    bool is_offboard_mode_;        // 是否已切换到 OFFBOARD
    double takeoff_altitude_;      // 起飞目标高度 (NED: z 向下为正)
    double takeoff_check_interval_;
    double takeoff_stable_time_;   // 高度达标后稳定等待（秒）
    double takeoff_hover_time_;    // 悬停等待（秒）
    ros::Time takeoff_start_time_;
    mavros_msgs::State current_mavros_state_;
    geometry_msgs::PoseStamped takeoff_setpoint_; // 起飞位置 setpoint
    std::atomic<bool> setpoint_running_{false};
    std::thread setpoint_thread_;
    int takeoff_retry_count_;        // 起飞重试次数
    int max_takeoff_retries_;        // 最大重试次数
    double takeoff_retry_delay_;     // 重试延迟（秒）
    ros::Time takeoff_failed_time_;  // 进入 FAILED 状态的时间
    bool takeoff_failed_logged_;     // 是否已打印 FAILED 日志（避免重复刷屏）

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
        spiral_approach_duration_(10.0),
        use_sim_(true),
        is_px4_connected_(false),
        is_offboard_mode_(false),
        takeoff_state_(TakeoffState::TAKEOFF_IDLE),
        takeoff_altitude_(50.0),
        takeoff_check_interval_(0.5),
        takeoff_stable_time_(1.0),     // 高度达标后稳定 1 秒（原 2 秒）
        takeoff_hover_time_(0.5),      // 悬停 0.5 秒（原 2 秒）
        takeoff_retry_count_(0),
        max_takeoff_retries_(3),
        takeoff_retry_delay_(5.0),
        takeoff_failed_logged_(false) {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();
        initTargetState();

        ROS_INFO("[MissionManager] Initialized. Low speed threshold: %.1f m/s, use_sim: %s",
                 low_speed_threshold_, use_sim_ ? "true" : "false");
    }

    void initParams() {
        nh_private_.param<double>("avoidance_safe_distance", avoidance_safe_distance_, 10.0);
        nh_private_.param<double>("low_speed_threshold", low_speed_threshold_, 12.0);
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);
        nh_private_.param<double>("mission_loop_rate", mission_loop_rate_, 50.0);
        nh_private_.param<double>("target_lock_confidence", target_lock_confidence_, 0.7);
        nh_private_.param<double>("spiral_approach_duration", spiral_approach_duration_, 10.0);
        nh_private_.param<double>("spiral_approach_radius", spiral_approach_radius_, 20.0);

        // 仿真/真机切换
        nh_private_.param<bool>("use_sim", use_sim_, true);
        nh_private_.param<double>("takeoff_altitude", takeoff_altitude_, 50.0);
        nh_private_.param<int>("max_takeoff_retries", max_takeoff_retries_, 3);
        nh_private_.param<double>("takeoff_retry_delay", takeoff_retry_delay_, 5.0);
        nh_private_.param<double>("takeoff_stable_time", takeoff_stable_time_, 1.0);  // 高度达标后稳定时间
        nh_private_.param<double>("takeoff_hover_time", takeoff_hover_time_, 0.5);    // 悬停等待时间
        // GPS 参考点（仿真时所有 UAV 共享，PX4 SITL 时各 UAV 用各自的 home）
        nh_private_.param<double>("ref_lat", ref_lat_, 36.096);
        nh_private_.param<double>("ref_lon", ref_lon_, 114.392);
        nh_private_.param<double>("ref_alt", ref_alt_, 100.0);

        // 根据 use_sim 设置 topic
        if (use_sim_) {
            pose_topic_ = "quad/pose";
        } else {
            pose_topic_ = "mavros/local_position/pose";
        }
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

        // 本机位置（根据 use_sim 选择 topic）
        self_pose_sub_ = nh_.subscribe(
            pose_topic_, 10,
            &MissionManager::selfPoseCallback, this);

        // 毫米波雷达障碍检测
        obstacle_sub_ = nh_.subscribe(
            "detection/obstacle", 10,
            &MissionManager::obstacleCallback, this);

        // PX4 SITL: 飞控状态订阅
        if (!use_sim_) {
            mavros_state_sub_ = nh_.subscribe(
                "mavros/state", 10,
                &MissionManager::mavrosStateCallback, this);

            // PX4 SITL: 模式切换 service client
            set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
                "mavros/set_mode");

            // PX4 SITL: 解锁 service client
            arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
                "mavros/cmd/arming");

            ROS_INFO("[MissionManager] PX4 SITL mode enabled, subscribing to mavros/state");
        }
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

        // PX4 SITL: 起飞位置 setpoint 发布器
        if (!use_sim_) {
            takeoff_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                "mavros/setpoint_position/local", 10);
        }
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

        // 重复模式（gs_simulator 1Hz 持续 latched、或地面站重复点击）早返回，避免日志刷屏
        if (mode == workModeToString()) {
            return;
        }

        if (mode == "SEARCH_ONLY") {
            current_work_mode_ = WorkMode::SEARCH_ONLY;
            ROS_WARN("[MissionManager] Mode changed to SEARCH_ONLY - Guidance DISABLED");

            // SEARCH_ONLY 模式下禁用制导，只做航点飞行
            disableGuidance();

        } else if (mode == "SEARCH_TRACK") {
            current_work_mode_ = WorkMode::SEARCH_TRACK;
            ROS_WARN("[MissionManager] Mode changed to SEARCH_TRACK - enabling guidance");
            // 切到 SEARCH_TRACK 立刻使能 guidance（即使没目标也发零速度 hover，避免 PX4 OFFBOARD 失联）
            enableGuidance();
            // 关键：立刻下发 guidance mode = "track"，不要等 startGuidanceApproach() 触发。
            // guidance_control_node 默认 mode = "strike"，如果不先下发 mode，
            //   目标出现前 guidance 会按 strike 处理（虽然没目标时只是发零速度，但一旦
            //   startGuidanceApproach 触发前如果 subscriber 没收到 mode 消息，会沿用 strike）。
            std_msgs::String guidance_mode_msg;
            guidance_mode_msg.data = "track";
            guidance_mode_pub_.publish(guidance_mode_msg);
            // 停掉 waypoint_executor，避免和 guidance 抢 setpoint
            std_msgs::String wp_cmd;
            wp_cmd.data = "stop";
            waypoint_control_pub_.publish(wp_cmd);

        } else if (mode == "SEARCH_STRIKE") {
            current_work_mode_ = WorkMode::SEARCH_STRIKE;
            ROS_WARN("[MissionManager] Mode changed to SEARCH_STRIKE - enabling guidance");
            // 切到 SEARCH_STRIKE 立刻使能 guidance（同 SEARCH_TRACK 原因）
            enableGuidance();
            // 立刻下发 guidance mode = "strike"
            std_msgs::String guidance_mode_msg;
            guidance_mode_msg.data = "strike";
            guidance_mode_pub_.publish(guidance_mode_msg);
            std_msgs::String wp_cmd;
            wp_cmd.data = "stop";
            waypoint_control_pub_.publish(wp_cmd);

        } else if (mode == "TAKEOFF") {
            current_work_mode_ = WorkMode::TAKEOFF;
            ROS_WARN("[MissionManager] Mode changed to TAKEOFF - starting takeoff sequence...");
            // 重置起飞状态机（让 runPx4TakeoffSequence() 从 TAKEOFF_IDLE 开始推进）
            if (!use_sim_) {
                takeoff_state_ = TakeoffState::TAKEOFF_IDLE;
                takeoff_retry_count_ = 0;
                takeoff_failed_logged_ = false;
            }
            // 清空目标状态（起飞阶段不需要管目标）
            initTargetState();
            stopMission();
        } else if (mode == "IDLE") {
            current_work_mode_ = WorkMode::IDLE;
            ROS_WARN("[MissionManager] Mode changed to IDLE - stopping everything");
            stopMission();
            disableGuidance();
        } else {
            ROS_WARN("[MissionManager] Unknown mode: %s", mode.c_str());
            // 未知模式不做任何事，直接返回
            return;
        }

        // 离开 TAKEOFF 模式时统一停掉 setpoint 发布器（避免持续发送位置指令覆盖其他模块的速度指令）
        if (current_work_mode_ != WorkMode::TAKEOFF && !use_sim_ && setpoint_running_) {
            stopSetpointPublisher();
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
        // 接收的是 GPS: pose.position.x=lat, .y=lon, .z=alt
        neighbors_.clear();
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            NeighborUav neighbor;
            gpsToNed(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z,
                     neighbor.ned_x, neighbor.ned_y, neighbor.ned_z);
            neighbor.last_update = ros::Time::now();
            neighbors_.push_back(neighbor);
        }
    }

    /**
     * GPS (WGS84) -> NED 坐标转换
     */
    void gpsToNed(double lat, double lon, double alt,
                  double& ned_x, double& ned_y, double& ned_z) {
        const double EARTH_R = 6378137.0;
        double d_lat = lat - ref_lat_;
        ned_x = d_lat * M_PI / 180.0 * EARTH_R;  // 北向
        double d_lon = lon - ref_lon_;
        ned_y = d_lon * M_PI / 180.0 * EARTH_R * cos(ref_lat_ * M_PI / 180.0);  // 东向
        ned_z = -(alt - ref_alt_);  // 下向
    }

    void interUavTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 收到其他无人机发现的目标
        // 可以选择是否协同跟踪
        ROS_INFO("[MissionManager] Received target info from other UAV: %s",
                 msg->header.frame_id.c_str());
    }

    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_mavros_state_ = *msg;
        is_px4_connected_ = msg->connected;
        is_offboard_mode_ = (msg->mode == "OFFBOARD");

        // 检测 OFFBOARD 模式切换（throttle 避免 mavros 1Hz 状态推送时刷屏）
        if (is_offboard_mode_ && !isTakeoffComplete()) {
            ROS_WARN_THROTTLE(5.0, "[MissionManager] OFFBOARD mode detected, waiting for takeoff complete...");
        }
    }

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (use_sim_) {
            // 仿真输入是 NED 坐标系
            current_pose_ = *msg;
        } else {
            // Mavros 输入是 ENU 坐标系，需要转换为 NED
            // ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            current_pose_.pose.position.x = msg->pose.position.y;
            current_pose_.pose.position.y = msg->pose.position.x;
            current_pose_.pose.position.z = -msg->pose.position.z;
            // 四元数 ENU->NED: w,x 不变, y,z 取反
            current_pose_.pose.orientation.w = msg->pose.orientation.w;
            current_pose_.pose.orientation.x = msg->pose.orientation.x;
            current_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_pose_.pose.orientation.z = -msg->pose.orientation.z;
            current_pose_.header = msg->header;
        }
        is_pose_received_ = true;

        // 发布NWU姿态用于RViz显示
        // NED -> NWU: x不变, y取反, z取反
        geometry_msgs::PoseStamped uav_pose_nwu;
        uav_pose_nwu.pose.position.x = current_pose_.pose.position.x;
        uav_pose_nwu.pose.position.y = -current_pose_.pose.position.y;
        uav_pose_nwu.pose.position.z = -current_pose_.pose.position.z;
        // 四元数: w,x 不变, y,z 取反
        uav_pose_nwu.pose.orientation.w = current_pose_.pose.orientation.w;
        uav_pose_nwu.pose.orientation.x = current_pose_.pose.orientation.x;
        uav_pose_nwu.pose.orientation.y = -current_pose_.pose.orientation.y;
        uav_pose_nwu.pose.orientation.z = -current_pose_.pose.orientation.z;
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

        // PX4 SITL: 运行起飞状态机
        if (!use_sim_) {
            runPx4TakeoffSequence();
            if (!isTakeoffComplete()) {
                // 起飞未完成，跳过任务执行
                return;
            }
        }

        // 检查急停条件（毫米波雷达）
        checkEmergencyStop();

        // 根据工作模式执行对应行为
        switch (current_work_mode_) {
            case WorkMode::IDLE:
                handleIdle();
                break;
            case WorkMode::TAKEOFF:
                // 起飞流程由 runPx4TakeoffSequence() 推进；mission_loop 不做额外动作
                // TAKEOFF_COMPLETE 后 UAV 在 setpoint 控制下悬停，等待用户切换到 SEARCH_*
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

    // PX4 SITL: 执行起飞状态机
    void runPx4TakeoffSequence() {
        // 关键门控：只有 TAKEOFF 模式下才推进起飞流程。
        // 避免：用户在 IDLE 时 FCU 一连上就自动起飞。
        if (current_work_mode_ != WorkMode::TAKEOFF) {
            // 切出 TAKEOFF 模式时：停止 setpoint 发布器（避免持续发送位置指令覆盖其他模块）
            if (setpoint_running_) {
                stopSetpointPublisher();
            }
            // 重置到 IDLE，等用户再次切到 TAKEOFF 才会重启
            if (takeoff_state_ != TakeoffState::TAKEOFF_IDLE) {
                takeoff_state_ = TakeoffState::TAKEOFF_IDLE;
            }
            return;
        }

        switch (takeoff_state_) {
            case TakeoffState::TAKEOFF_IDLE:
                // 已在 TAKEOFF 模式：等待 FCU 连接后开始
                if (!use_sim_ && is_px4_connected_) {
                    takeoff_state_ = TakeoffState::TAKEOFF_WAITING_FCU;
                    ROS_WARN("[MissionManager] Starting PX4 takeoff sequence...");
                }
                break;

            case TakeoffState::TAKEOFF_WAITING_FCU:
                if (!is_px4_connected_) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] Waiting for PX4 FCU connection...");
                } else {
                    takeoff_state_ = TakeoffState::TAKEOFF_SETTING_OFFBOARD;
                    startSetpointPublisher();
                }
                break;

            case TakeoffState::TAKEOFF_SETTING_OFFBOARD:
                if (setMode("OFFBOARD")) {
                    takeoff_state_ = TakeoffState::TAKEOFF_ARMING;
                }
                break;

            case TakeoffState::TAKEOFF_ARMING:
                if (armVehicle(true)) {
                    takeoff_state_ = TakeoffState::TAKEOFF_TAKEOFF_EXEC;
                    takeoff_start_time_ = ros::Time::now();
                    takeoff_setpoint_.pose.position.z = takeoff_altitude_;
                    ROS_WARN("[MissionManager] Arming successful, starting takeoff climb...");
                }
                break;

            case TakeoffState::TAKEOFF_TAKEOFF_EXEC: {
                // 检查高度 (NED: z 向下为正)
                double current_alt = -current_pose_.pose.position.z;
                if (current_alt >= takeoff_altitude_) {
                    double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                    if (elapsed > takeoff_stable_time_) {  // 高度稳定时间（ROS 参数）
                        takeoff_state_ = TakeoffState::TAKEOFF_HOVERING;
                        takeoff_start_time_ = ros::Time::now();
                        ROS_WARN("[MissionManager] Takeoff altitude reached, hovering...");
                    }
                } else {
                    ROS_WARN_THROTTLE(1.0, "[MissionManager] Takeoff climbing: %.1f / %.1f m",
                                     current_alt, takeoff_altitude_);
                }

                // 超时检测
                double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                if (elapsed > 60.0) {
                    ROS_ERROR("[MissionManager] Takeoff timeout!");
                    takeoff_state_ = TakeoffState::TAKEOFF_FAILED;
                }
                break;
            }

            case TakeoffState::TAKEOFF_HOVERING: {
                // 悬停一段时间后进入任务
                double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                if (elapsed > takeoff_hover_time_) {  // 悬停等待时间（ROS 参数）
                    takeoff_state_ = TakeoffState::TAKEOFF_COMPLETE;
                    ROS_WARN("[MissionManager] ===== TAKEOFF COMPLETE! Ready for mission execution =====");
                    // 关键：不要停掉 setpoint 发布器！否则 PX4 OFFBOARD 收不到 setpoint 会触发
                    // failsafe 自动降落。让 setpoint 持续发起飞点位置 = UAV 原地悬停，
                    // 直到用户在地面站切到 SEARCH_ONLY/TRACK/STRIKE 才会停掉。
                    // （modeCallback 里离开 TAKEOFF 模式时会统一 stopSetpointPublisher()）
                }
                break;
            }

            case TakeoffState::TAKEOFF_COMPLETE:
                // 起飞完成，等待任务模式
                break;

            case TakeoffState::TAKEOFF_FAILED: {
                if (!takeoff_failed_logged_) {
                    ROS_ERROR("[MissionManager] Takeoff failed! Retry %d / %d in %.1f s",
                              takeoff_retry_count_, max_takeoff_retries_, takeoff_retry_delay_);
                    takeoff_failed_time_ = ros::Time::now();
                    takeoff_failed_logged_ = true;
                }

                // 等待重试延迟后复位
                double failed_elapsed = (ros::Time::now() - takeoff_failed_time_).toSec();
                if (failed_elapsed < takeoff_retry_delay_) {
                    break;
                }

                if (takeoff_retry_count_ < max_takeoff_retries_) {
                    // 重试：复位到 IDLE 重新跑整个起飞流程
                    takeoff_retry_count_++;
                    takeoff_failed_logged_ = false;
                    takeoff_state_ = TakeoffState::TAKEOFF_IDLE;
                    ROS_WARN("[MissionManager] Retrying takeoff sequence (attempt %d / %d)...",
                             takeoff_retry_count_, max_takeoff_retries_);
                } else {
                    ROS_ERROR_THROTTLE(5.0, "[MissionManager] Takeoff failed after %d retries. Giving up.",
                                       max_takeoff_retries_);
                }
                break;
            }
        }
    }

    // PX4 SITL: 启动 setpoint 发布线程
    void startSetpointPublisher() {
        if (setpoint_running_) {
            return;
        }
        // 等待首次位置数据（确保使用当前 UAV 位置作为起飞起点）
        int wait_count = 0;
        while (!is_pose_received_ && ros::ok() && wait_count < 200) {  // 最多等 4 秒
            ros::Duration(0.02).sleep();
            wait_count++;
        }
        if (!is_pose_received_) {
            ROS_WARN("[MissionManager] No pose received before setpoint publisher start, using default (0,0)");
            // mavros/setpoint_position/local 期望 ENU，0,0 直接发即可
            takeoff_setpoint_.pose.position.x = 0;
            takeoff_setpoint_.pose.position.y = 0;
        } else {
            // current_pose_ 是 NED（来自 selfPoseCallback 的 ENU→NED 转换）
            // mavros/setpoint_position/local 期望 ENU：x_east = y_ned, y_north = x_ned
            takeoff_setpoint_.pose.position.x = current_pose_.pose.position.y;  // y_ned → x_enu (east)
            takeoff_setpoint_.pose.position.y = current_pose_.pose.position.x;  // x_ned → y_enu (north)
            ROS_INFO("[MissionManager] Takeoff setpoint start: x=%.2f, y=%.2f (ENU, from NED pos=%.2f,%.2f)",
                     takeoff_setpoint_.pose.position.x, takeoff_setpoint_.pose.position.y,
                     current_pose_.pose.position.x, current_pose_.pose.position.y);
        }

        // 初始低高度（ENU z up=正，直接用正值即可）
        takeoff_setpoint_.pose.position.z = 0.5;
        takeoff_setpoint_.header.frame_id = "map";

        setpoint_running_ = true;
        setpoint_thread_ = std::thread([this]() {
            ROS_INFO("[MissionManager] Setpoint publisher thread started");
            ros::Rate rate(50);  // 50Hz
            while (ros::ok() && setpoint_running_) {
                takeoff_setpoint_.header.stamp = ros::Time::now();
                takeoff_setpoint_pub_.publish(takeoff_setpoint_);
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("[MissionManager] Setpoint publisher thread stopped");
        });
    }

    void stopSetpointPublisher() {
        setpoint_running_ = false;
        if (setpoint_thread_.joinable()) {
            setpoint_thread_.join();
        }
    }

    bool setMode(const std::string& mode) {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = mode;

        if (!set_mode_client_.call(set_mode) || !set_mode.response.mode_sent) {
            ROS_WARN_THROTTLE(2.0, "[MissionManager] Failed to send set mode command: %s", mode.c_str());
            return false;
        }

        // 等待确认
        ros::Rate rate(10);
        auto start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < 5.0) {
            ros::spinOnce();
            if (current_mavros_state_.mode == mode) {
                ROS_INFO("[MissionManager] Mode confirmed: %s", mode.c_str());
                return true;
            }
            rate.sleep();
        }

        ROS_WARN_THROTTLE(2.0, "[MissionManager] Mode change timeout - current mode: %s",
                         current_mavros_state_.mode.c_str());
        return false;
    }

    bool armVehicle(bool arm) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;

        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("[MissionManager] Vehicle %s", arm ? "ARMED" : "DISARMED");
            return true;
        }

        ROS_WARN_THROTTLE(2.0, "[MissionManager] Failed to %s vehicle", arm ? "arm" : "disarm");
        return false;
    }

    // PX4 SITL: 获取起飞是否完成
    bool isTakeoffComplete() const {
        return takeoff_state_ == TakeoffState::TAKEOFF_COMPLETE;
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

    void enableGuidance() {
        is_guidance_active_ = true;

        // 使能制导（guidance_control_node 在 enabled 但无目标时会持续发零速度，
        // 维持 PX4 OFFBOARD 心跳，避免 UAV 因 setpoint 断流触发 failsafe 降落）
        std_msgs::Bool enable;
        enable.data = true;
        guidance_enable_pub_.publish(enable);

        ROS_INFO("[MissionManager] Guidance enabled");
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
            double dx = current_pose_.pose.position.x - neighbor.ned_x;
            double dy = current_pose_.pose.position.y - neighbor.ned_y;
            double dz = current_pose_.pose.position.z - neighbor.ned_z;
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
            case WorkMode::TAKEOFF: return "TAKEOFF";
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
