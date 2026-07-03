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
#include <mavros_msgs/CommandLong.h>

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

// 内部任务执行阶段 (独立于 WorkMode, 由 mission_manager 自己推进)
// WorkMode 是 GS 的意图 (SEARCH_*), MissionPhase 是实际在执行什么
// 引入原因: 地面站把"起飞+航点"打包成一个任务, 期间 WorkMode 一直是 TAKEOFF,
// 但内部需要在 HOVERING 完成后自动切到航点跟踪; 手动切模式会卡死航点执行。
enum class MissionPhase {
    PHASE_GROUND_IDLE,       // 在地面无任务
    PHASE_TAKING_OFF,        // 正在执行 OFFBOARD+ARM+爬升
    PHASE_HOVERING,          // 起飞完成, 悬停等待自动交接
    PHASE_WAYPOINT_FOLLOW,   // 正在按航点飞
    PHASE_GUIDANCE_TRACK,    // 搜索跟踪 (guidance)
    PHASE_GUIDANCE_STRIKE,   // 搜索打击 (guidance)
    PHASE_RETURNING,         // 返航
    PHASE_COMPLETE,          // 任务完成
    PHASE_FAILED,            // 失败
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
    ros::ServiceClient command_long_client_; // PX4 SITL: COMMAND_LONG 备用通道（绕过 SET_MODE 静默拒绝问题）

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
    ros::Publisher mission_mode_pub_;     // 向 waypoint_executor 转发 WorkMode (auto-handoff 时用)

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
    // LOCAL_POSITION_NED 发送频率（Hz）。某些板子 PX4 重启后默认频率太低，
    // 必须上电后用 MAV_CMD_SET_MESSAGE_INTERVAL (511) 设置一次，否则位置数据延迟大、
    // OFFBOARD 控制发散。FCU 断连后 flag 会清掉，PX4 重启重连时自动重发。
    double local_position_rate_hz_;
    bool is_local_position_rate_set_;
    int set_message_rate_retry_count_;     // 当前 FCU 连接期间的累计失败次数
    int max_set_message_rate_retries_;     // 超过此值放弃 (放行到 SETTING_OFFBOARD)
    ros::Time set_message_rate_start_time_;// 第一次尝试的时间, 用于 max_wait 超时
    double set_message_rate_max_wait_sec_; // 累计尝试时间上限 (秒)
    ros::Time takeoff_start_time_;
    ros::Time setpoint_start_time_;   // setpoint publisher 线程真正开始发包的时刻（用于 OFFBOARD 请求前的稳定等待）
    int setpoint_publish_count_;      // setpoint publisher 已发包计数（调试用：>0 证明 mavros 在转）
    mavros_msgs::State current_mavros_state_;
    geometry_msgs::PoseStamped takeoff_setpoint_; // 起飞位置 setpoint
    std::atomic<bool> setpoint_running_{false};
    std::thread setpoint_thread_;
    int takeoff_retry_count_;        // 起飞重试次数
    int max_takeoff_retries_;        // 最大重试次数
    double takeoff_retry_delay_;     // 重试延迟（秒）
    ros::Time takeoff_failed_time_;  // 进入 FAILED 状态的时间
    bool takeoff_failed_logged_;     // 是否已打印 FAILED 日志（避免重复刷屏）

    // ============== MissionPhase 状态机 ==============
    MissionPhase current_phase_;         // 当前任务执行阶段
    bool is_waypoints_received_;         // 地面站是否已下发航点
    size_t current_waypoint_count_;      // 当前收到的航点数量
    bool is_takeoff_handoff_done_;       // 起飞后是否已经完成"自动交接" (TAKEOFF→SEARCH_ONLY+停setpoint)

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
        takeoff_failed_logged_(false),
        current_phase_(MissionPhase::PHASE_GROUND_IDLE),
        is_waypoints_received_(false),
        current_waypoint_count_(0),
        is_takeoff_handoff_done_(false),
        local_position_rate_hz_(20.0),       // 默认 20Hz (50ms), 板子一般够用
        is_local_position_rate_set_(false),
        set_message_rate_retry_count_(0),
        max_set_message_rate_retries_(3),
        set_message_rate_start_time_(ros::Time()),  // isZero() 表示还没开始尝试
        set_message_rate_max_wait_sec_(5.0) {

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
        nh_private_.param<double>("local_position_rate_hz", local_position_rate_hz_, 20.0);  // LOCAL_POSITION_NED 发送频率
        nh_private_.param<int>("max_set_message_rate_retries", max_set_message_rate_retries_, 3);
        nh_private_.param<double>("set_message_rate_max_wait_sec", set_message_rate_max_wait_sec_, 5.0);
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

            // PX4 SITL: COMMAND_LONG 备用通道（某些 commander 状态下 SET_MODE 会被静默拒收）
            command_long_client_ = nh_.serviceClient<mavros_msgs::CommandLong>(
                "mavros/cmd/command");

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

        // 向 waypoint_executor 转发 WorkMode (auto-handoff 时用, 把 TAKEOFF 变成 SEARCH_ONLY)
        // 直接发到 /mission/mode, 复用 comm_node 同一个 topic — 真实 GS 场景下 comm_node
        // 收到任务后只发一次 mode (不是 1Hz 持续 latched), 所以不会和这里冲突。
        // 保险: mission_manager 的 is_takeoff_handoff_done_ 标志保证只发一次。
        mission_mode_pub_ = nh_.advertise<std_msgs::String>(
            "mission/mode", 10);

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

        // === Auto-handoff 保护 ===
        // 如果已经完成了 takeoff→SEARCH_ONLY 自动交接, 任何再次收到的 TAKEOFF 都忽略,
        // 同时立刻 re-publish SEARCH_ONLY 把它"压回去"。
        //
        // 原因: 真实 GS 在任务执行期间会一直推 /gs/mode_cmd="TAKEOFF" (任务还未结束),
        //       comm_node 会持续转 /mission/mode="TAKEOFF"。如果只 return, waypoint_executor
        //       会收到 comm_node 的 TAKEOFF → 退出 SEARCH_ONLY → 不再发速度 → PX4 OFFBOARD
        //       500ms 后 failsafe 降落。
        // 解决方案: mission_manager 在这里立刻再发一次 SEARCH_ONLY, waypoint_executor 的
        //           "最新模式生效" 逻辑会把它切回去。ROS 同一 topic 多个 publisher 之间
        //           没有全局顺序保证, 但 localhost 上 mission_manager 的 callback 一定在
        //           comm_node 之后触发 (mission_manager 订阅的就是 /mission/mode), 所以
        //           mission_manager 的 re-publish 几乎总在 comm_node 的 TAKEOFF 之后到达
        //           waypoint_executor, 起到"压回去"的作用。
        // 真正的"重新起飞"只能由用户切 IDLE → TAKEOFF 显式触发 (is_takeoff_handoff_done_ 被重置)。
        if (mode == "TAKEOFF" && is_takeoff_handoff_done_) {
            std_msgs::String override_msg;
            override_msg.data = "SEARCH_ONLY";
            mission_mode_pub_.publish(override_msg);
            ROS_WARN_THROTTLE(2.0, "[MissionManager] Re-publishing SEARCH_ONLY to override TAKEOFF "
                                   "(phase=%s)", missionPhaseToString().c_str());
            return;
        }

        if (mode == "SEARCH_ONLY") {
            current_work_mode_ = WorkMode::SEARCH_ONLY;
            ROS_WARN("[MissionManager] Mode changed to SEARCH_ONLY - Guidance DISABLED");

            // 同步 MissionPhase (用户手动切的场景, 或 auto-handoff 后被这条分支处理)
            current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;

            // SEARCH_ONLY 模式下禁用制导，只做航点飞行
            disableGuidance();

        } else if (mode == "SEARCH_TRACK") {
            current_work_mode_ = WorkMode::SEARCH_TRACK;
            ROS_WARN("[MissionManager] Mode changed to SEARCH_TRACK - enabling guidance");
            current_phase_ = MissionPhase::PHASE_GUIDANCE_TRACK;
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
            current_phase_ = MissionPhase::PHASE_GUIDANCE_STRIKE;
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
            // 重置 auto-handoff 状态 — 全新的一次起飞, 上次的 handoff 标志清掉
            is_takeoff_handoff_done_ = false;
            // 注意: 不要在这里清 is_waypoints_received_ !
            // 场景: 地面站先把航点 latched 发下来 (comm_node 转发给 mission_manager,
            //       is_waypoints_received_=true, count=3), 然后才发 TAKEOFF。
            //       如果在这里清掉, HOVERING 到达时 flag=false, auto-handoff 永远不触发,
            //       UAV 永远悬停等永远不会到达的"新航点"。
            // 真正需要清的场景: 用户进 IDLE (已清) 或发空 path (waypointCallback 会清)。
            // 重新起飞时, waypoint 仍然有效, 复用即可。
            current_phase_ = MissionPhase::PHASE_TAKING_OFF;
            stopMission();
        } else if (mode == "IDLE") {
            current_work_mode_ = WorkMode::IDLE;
            ROS_WARN("[MissionManager] Mode changed to IDLE - stopping everything");
            // 全部状态复位 — 重新起飞前需要重新走完整流程
            is_takeoff_handoff_done_ = false;
            is_waypoints_received_ = false;
            current_phase_ = MissionPhase::PHASE_GROUND_IDLE;
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
        // 但需要跟踪"航点是否已下发", 给 auto-handoff 当门控条件
        current_waypoint_count_ = msg->poses.size();
        if (current_waypoint_count_ > 0) {
            is_waypoints_received_ = true;
            ROS_INFO("[MissionManager] Received %lu waypoints (ready for auto-handoff if in HOVERING)",
                     current_waypoint_count_);
        } else {
            // 空 path 视为"清空航点", 不当作"已收到"
            is_waypoints_received_ = false;
            ROS_WARN("[MissionManager] Received EMPTY waypoint path, clearing is_waypoints_received_");
        }
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

        // 检测 FCU 断连 (上次的 connected=true, 这次变 false) — 清掉 LOCAL_POSITION_NED
        // 频率标志, 下次重连时重新发 SET_MESSAGE_INTERVAL (PX4 重启会重置消息间隔)
        bool was_connected = is_px4_connected_;
        is_px4_connected_ = msg->connected;
        if (was_connected && !is_px4_connected_) {
            ROS_WARN("[MissionManager] >>>>> PX4 FCU DISCONNECTED — will re-set LOCAL_POSITION_NED "
                     "rate on reconnect");
            is_local_position_rate_set_ = false;
            set_message_rate_retry_count_ = 0;
            set_message_rate_start_time_ = ros::Time();  // isZero(), 下次重连时重新记录起始
        }

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
            // 例外: auto-handoff 后, current_work_mode_ 已经被切到 SEARCH_ONLY, 但
            //       takeoff_state_ 必须保留在 COMPLETE (让 isTakeoffComplete()=true,
            //       missionTimer 主循环才能继续跑航点跟踪)。
            if (takeoff_state_ != TakeoffState::TAKEOFF_IDLE && !is_takeoff_handoff_done_) {
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
                    break;
                }
                // FCU 已连, 但还没调过 LOCAL_POSITION_NED 频率 — 调一下
                if (!is_local_position_rate_set_) {
                    // 第一次进这分支时记录起始时间 (用 isZero 判定, 避免 set_now 重复覆盖)
                    if (set_message_rate_start_time_.isZero()) {
                        set_message_rate_start_time_ = ros::Time::now();
                    }
                    if (setMessageRate(32, local_position_rate_hz_)) {
                        // 成功 — 标记完成, 进入下一步
                        is_local_position_rate_set_ = true;
                        ROS_WARN("[MissionManager] >>>>> PX4 LOCAL_POSITION_NED rate set to %.1f Hz "
                                 "(attempt %d, elapsed %.2fs)",
                                 local_position_rate_hz_,
                                 set_message_rate_retry_count_ + 1,
                                 (ros::Time::now() - set_message_rate_start_time_).toSec());
                        set_message_rate_start_time_ = ros::Time();  // 清零, 下次 FCU 重连时复用
                        set_message_rate_retry_count_ = 0;
                        // fall through 进 SETTING_OFFBOARD
                    } else {
                        set_message_rate_retry_count_++;
                        double elapsed = (ros::Time::now() - set_message_rate_start_time_).toSec();
                        ROS_WARN_THROTTLE(1.0, "[MissionManager] >>>>> LOCAL_POSITION_NED rate set FAILED "
                                              "(attempt %d/%d, elapsed %.1fs/%.1fs)",
                                              set_message_rate_retry_count_, max_set_message_rate_retries_,
                                              elapsed, set_message_rate_max_wait_sec_);
                        // 判定放弃条件: 重试次数耗尽 OR 总等待时间超上限
                        bool retries_done = (set_message_rate_retry_count_ >= max_set_message_rate_retries_);
                        bool timeout = (elapsed > set_message_rate_max_wait_sec_);
                        if (retries_done || timeout) {
                            ROS_ERROR("[MissionManager] >>>>> LOCAL_POSITION_NED rate set GIVE UP "
                                      "(%s after %d attempts / %.1fs). Proceeding with PX4 default rate — "
                                      "UAV may oscillate in OFFBOARD. Check PX4 MAVLink config or reboot FCU.",
                                      retries_done ? "retries exhausted" : "timeout",
                                      set_message_rate_retry_count_, elapsed);
                            // 强制标记完成, 放行进 OFFBOARD; flag 重置交给 FCU 断连时处理
                            is_local_position_rate_set_ = true;
                            set_message_rate_start_time_ = ros::Time();
                            set_message_rate_retry_count_ = 0;
                            // fall through 进 SETTING_OFFBOARD
                        } else {
                            // 还没放弃 — 下个 tick 再试
                            break;
                        }
                    }
                }
                // 通过到这里说明已经成功 (或放弃), 进 SETTING_OFFBOARD
                takeoff_state_ = TakeoffState::TAKEOFF_SETTING_OFFBOARD;
                startSetpointPublisher();
                break;

            case TakeoffState::TAKEOFF_SETTING_OFFBOARD: {
                // ===== 关键: setpoint publisher 起来后稳定流 ≥ 150ms 再触发 OFFBOARD+ARM 序列 =====
                // PX4 要求最近 COM_OFFBOARD_LOSS_TIMEOUT (默认 0.5s) 内有连续 setpoint
                // 才接受 OFFBOARD 切换。150ms 已经足够安全。
                double sp_elapsed = (ros::Time::now() - setpoint_start_time_).toSec();
                if (sp_elapsed < 0.15) {
                    ROS_WARN_THROTTLE(0.5, "[MissionManager] Pre-warming setpoint stream: %.0f ms / 150 ms (count=%d)",
                                      sp_elapsed * 1000.0, setpoint_publish_count_);
                    break;
                }
                // 一次性快速序列: SET_MODE → COMMAND_LONG → ARM，不等 mode 确认。
                // 模仿 pymavlink 的 50ms 内连发 SET_MODE+ARM 的行为 — 这是它在同一架
                // FCU 上能跑通的关键时机 (PX4 commander 100~500ms 内会 revert OFFBOARD)。
                ROS_WARN("[MissionManager] >>>> OFFBOARD+ARM rapid sequence starting "
                         "(sp=%.0fms count=%d, PX4 mode=%s armed=%d sys_status=%d)",
                         sp_elapsed * 1000.0, setpoint_publish_count_,
                         current_mavros_state_.mode.c_str(),
                         current_mavros_state_.armed ? 1 : 0,
                         current_mavros_state_.system_status);
                if (triggerOffboardAndArm()) {
                    takeoff_state_ = TakeoffState::TAKEOFF_TAKEOFF_EXEC;
                    takeoff_start_time_ = ros::Time::now();
                    takeoff_setpoint_.pose.position.z = takeoff_altitude_;
                    ROS_WARN("[MissionManager] >>>> OFFBOARD + ARM SUCCESS, takeoff climb started (target alt=%.1f m)",
                             takeoff_altitude_);
                } else {
                    // 不进 FAILED 状态，下一 tick 会再走一遍 SETTING_OFFBOARD 重试整组动作
                    ROS_WARN("[MissionManager] >>>> OFFBOARD + ARM sequence FAILED (PX4 mode=%s armed=%d), "
                             "will retry next cycle",
                             current_mavros_state_.mode.c_str(),
                             current_mavros_state_.armed ? 1 : 0);
                }
                break;
            }

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
                    // === Auto-handoff 判定 ===
                    // 真实 GS 场景下: 任务 = TAKEOFF + 航点列表, 期间 WorkMode 一直是 TAKEOFF
                    // 不能等用户手动切 SEARCH_ONLY (会被地面站持续发的 TAKEOFF 覆盖),
                    // 必须在这里检测到航点已下发后自动交接给 waypoint_executor。
                    if (is_waypoints_received_ && !is_takeoff_handoff_done_) {
                        performTakeoffHandoff();
                        // performTakeoffHandoff 会:
                        //   1. 标记 is_takeoff_handoff_done_=true (防重复)
                        //   2. 内部 work_mode 切到 SEARCH_ONLY
                        //   3. 发 /mission/mode="SEARCH_ONLY" 给 waypoint_executor
                        //   4. 立刻停 setpoint_thread (gap ~20ms < 500ms PX4 timeout)
                        //   5. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE
                        //   6. current_phase_=PHASE_WAYPOINT_FOLLOW
                    } else if (!is_takeoff_handoff_done_) {
                        // 还没收到航点, 保持 HOVERING, 让 setpoint_thread 持续发悬停点
                        // 保险: 避免在没航点的情况下交接, 导致 waypoint_executor 看到空队列报错
                        ROS_WARN_THROTTLE(2.0, "[MissionManager] HOVERING waiting for waypoints "
                                              "(is_waypoints_received_=%d, waypoint_count=%zu)",
                                              is_waypoints_received_ ? 1 : 0, current_waypoint_count_);
                        // 重置 elapsed, 再多等一拍, 直到航点到达
                        takeoff_start_time_ = ros::Time::now();
                    } else {
                        // 已经交接过, 不应该再回到 HOVERING — 防御性, 不会发生
                        takeoff_state_ = TakeoffState::TAKEOFF_COMPLETE;
                    }
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

    // === Auto-handoff: TAKEOFF_HOVERING 完成后, 自动切到 PHASE_WAYPOINT_FOLLOW ===
    // 触发条件: 高度达标 + hover 计时到 + 航点已下发 + 还没交接过
    // 流程:
    //   1. 标记 is_takeoff_handoff_done_=true (防重复)
    //   2. 内部 work_mode 切到 SEARCH_ONLY (与 waypoint_executor 的门控保持一致)
    //   3. 发 /mission/mode="SEARCH_ONLY" 触发 waypoint_executor 开始执行
    //   4. 立即停 setpoint_thread (gap ~20ms, 远小于 PX4 COM_OFFBOARD_LOSS_TIMEOUT=500ms)
    //   5. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE (让 missionTimerCallback 继续跑)
    //   6. current_phase_=PHASE_WAYPOINT_FOLLOW
    //
    // 与"用户在 HOVERING 后手动发 SEARCH_ONLY"的区别:
    //   - 手动: modeCallback 收到 SEARCH_ONLY, 走 SEARCH_ONLY 分支
    //   - 自动: 这里直接发到 /mission/mode, 不经过 modeCallback (避免再次进入 if work_mode != TAKEOFF 的分支)
    //
    // 关于 dual publisher 风险:
    //   两者都发 SET_POSITION_TARGET_LOCAL_NED, PX4 用最新的。setpoint_thread_ 50Hz,
    //   waypoint_executor 也是 50Hz。我们先发 SEARCH_ONLY 再停 setpoint_thread,
    //   期间 (~20ms) 会有少量重叠包, 但 PX4 取最新, 不会出问题。
    void performTakeoffHandoff() {
        ROS_WARN("[MissionManager] ===== AUTO-HANDOFF: TAKEOFF → SEARCH_ONLY =====");
        ROS_WARN("[MissionManager]   - is_waypoints_received_=true, waypoint_count=%zu",
                 current_waypoint_count_);

        // 1. 标记 handoff 完成 (防重复)
        is_takeoff_handoff_done_ = true;

        // 2. 内部 work_mode 切到 SEARCH_ONLY (与 waypoint_executor 的门控保持一致)
        WorkMode prev_mode = current_work_mode_;
        current_work_mode_ = WorkMode::SEARCH_ONLY;
        disableGuidance();  // SEARCH_ONLY 模式不用制导

        // 3. 通知 waypoint_executor 开始执行航点
        std_msgs::String mode_msg;
        mode_msg.data = "SEARCH_ONLY";
        mission_mode_pub_.publish(mode_msg);
        ROS_WARN("[MissionManager]   - Published /mission/mode='SEARCH_ONLY' (was: %s)",
                 workModeToString().c_str());

        // 4. 立刻停 setpoint_thread (此时 waypoint_executor 即将接管)
        //    重要顺序: 先 publish 再 stop — 让 waypoint_executor 有时间被通知
        if (setpoint_running_) {
            stopSetpointPublisher();
            ROS_WARN("[MissionManager]   - Takeoff setpoint publisher stopped");
        }

        // 5. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE
        //    让 isTakeoffComplete()=true, missionTimerCallback 继续跑 mission 主循环
        takeoff_state_ = TakeoffState::TAKEOFF_COMPLETE;

        // 6. 推进 MissionPhase
        current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;

        ROS_WARN("[MissionManager] ===== AUTO-HANDOFF COMPLETE, UAV now waypoint-following =====");
    }

    // PX4 SITL: 启动 setpoint 发布线程
    void startSetpointPublisher() {
        if (setpoint_running_) {
            return;
        }
        // 等待首次位置数据（确保使用当前 UAV 位置作为起飞起点）
        // 3588 + 真机场景下 mavros 启动到首帧 local_position 可能 >4s，所以放宽到 10s
        int wait_count = 0;
        while (!is_pose_received_ && ros::ok() && wait_count < 500) {  // 最多等 10 秒
            ros::Duration(0.02).sleep();
            wait_count++;
        }
        if (!is_pose_received_) {
            ROS_WARN("[MissionManager] >>>> No pose received before setpoint publisher start "
                     "(waited %.1fs), using default (0,0). Will jump to actual position when "
                     "first pose arrives — may cause brief takeoff overshoot.",
                     wait_count * 0.02);
            // mavros/setpoint_position/local 期望 ENU，0,0 直接发即可
            takeoff_setpoint_.pose.position.x = 0;
            takeoff_setpoint_.pose.position.y = 0;
        } else {
            // current_pose_ 是 NED（来自 selfPoseCallback 的 ENU→NED 转换）
            // mavros/setpoint_position/local 期望 ENU：x_east = y_ned, y_north = x_ned
            takeoff_setpoint_.pose.position.x = current_pose_.pose.position.y;  // y_ned → x_enu (east)
            takeoff_setpoint_.pose.position.y = current_pose_.pose.position.x;  // x_ned → y_enu (north)
            ROS_INFO("[MissionManager] Takeoff setpoint start: x=%.2f, y=%.2f (ENU, from NED pos=%.2f,%.2f, pose received after %.1fs)",
                     takeoff_setpoint_.pose.position.x, takeoff_setpoint_.pose.position.y,
                     current_pose_.pose.position.x, current_pose_.pose.position.y,
                     wait_count * 0.02);
        }

        // 初始低高度（ENU z up=正，直接用正值即可）
        takeoff_setpoint_.pose.position.z = 0.5;

        setpoint_publish_count_ = 0;
        setpoint_start_time_ = ros::Time::now();   // 关键：thread 即将开始的时刻
        setpoint_running_ = true;
        setpoint_thread_ = std::thread([this]() {
            ROS_INFO("[MissionManager] Setpoint publisher thread started");
            ros::Rate rate(50);  // 50Hz
            ros::Time last_log = ros::Time::now();
            // 🛠️ 新增：创建一个基准时间，模拟板子时间是和飞控同步对齐的
            ros::Time fake_now = ros::Time::now();

            while (ros::ok() && setpoint_running_) {
                // 🛠️ 修正：不要用 Time(0)，也不要用本地错误的 Time::now()
                // 让我们手动构建一个稳定的单调递增时间戳流发送给 Mavros
                fake_now += ros::Duration(0.02); // 50Hz 每次递增 20ms
                takeoff_setpoint_.header.stamp = fake_now;
                takeoff_setpoint_.pose.orientation.w = 1.0;  // 保持水平
                takeoff_setpoint_pub_.publish(takeoff_setpoint_);
                setpoint_publish_count_++;
                //ros::spinOnce();
                rate.sleep();

                // 每 1s 打印一次发包状态 + PX4 当前 mode，方便对比两架 UAV 差异
                if ((ros::Time::now() - last_log).toSec() > 1.0) {
                    ROS_WARN("[MissionManager] setpoint stream: %d pkts, PX4 mode=%s armed=%d connected=%d system_status=%d",
                             setpoint_publish_count_,
                             current_mavros_state_.mode.c_str(),
                             current_mavros_state_.armed ? 1 : 0,
                             current_mavros_state_.connected ? 1 : 0,
                             current_mavros_state_.system_status);
                    last_log = ros::Time::now();
                }
            }
            ROS_INFO("[MissionManager] Setpoint publisher thread stopped (total pkts=%d)", setpoint_publish_count_);
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

        // 每次请求都打一行（不 throttle）：方便两架 UAV 对比看到底调了几次
        ROS_WARN("[MissionManager] >>>>> setMode(%s) called, PX4 current_mode=%s connected=%d armed=%d system_status=%d setpoint_pkts=%d",
                 mode.c_str(),
                 current_mavros_state_.mode.c_str(),
                 current_mavros_state_.connected ? 1 : 0,
                 current_mavros_state_.armed ? 1 : 0,
                 current_mavros_state_.system_status,
                 setpoint_publish_count_);

        // ===== Path 1: 标准 SET_MODE 服务（走 SET_MODE MAVLink 消息）=====
        bool set_mode_sent = false;
        if (set_mode_client_.call(set_mode) && set_mode.response.mode_sent) {
            set_mode_sent = true;
            ROS_WARN("[MissionManager] >>>>> SET_MODE service: command accepted by mavros (mode_sent=true)");
        } else {
            ROS_WARN("[MissionManager] >>>>> SET_MODE service failed, will try COMMAND_LONG fallback");
        }

        // ===== 等 5 秒看是否切成功（覆盖 SET_MODE 和 COMMAND_LONG 都试一遍）=====
        ros::Rate rate(10);
        auto start = ros::Time::now();
        int poll_count = 0;
        bool tried_command_long = false;
        while (ros::ok() && (ros::Time::now() - start).toSec() < 5.0) {
            ros::spinOnce();
            poll_count++;
            if (current_mavros_state_.mode == mode) {
                ROS_INFO("[MissionManager] >>>>> Mode confirmed: %s (took %.2fs, %d polls)",
                         mode.c_str(), (ros::Time::now() - start).toSec(), poll_count);
                return true;
            }
            // 1.5s 还没切成功,且没试过 COMMAND_LONG,就走第二条路径
            if (!tried_command_long && (ros::Time::now() - start).toSec() > 1.5) {
                tried_command_long = true;
                ROS_WARN("[MissionManager] >>>>> 1.5s elapsed, PX4 still in '%s'. Trying COMMAND_LONG fallback...",
                         current_mavros_state_.mode.c_str());
                sendSetModeCommandLong(mode);
            }
            // 每 1s 报一次当前 PX4 mode（即使 throttle 了这里也要刷）
            if (poll_count % 10 == 0) {
                ROS_WARN("[MissionManager] ... waiting for mode=%s, PX4 currently in '%s' (%.1fs elapsed)",
                         mode.c_str(), current_mavros_state_.mode.c_str(),
                         (ros::Time::now() - start).toSec());
            }
            rate.sleep();
        }

        ROS_WARN("[MissionManager] >>>>> Mode change TIMEOUT after 5s, PX4 mode='%s' (expected '%s'), setpoint_pkts=%d",
                 current_mavros_state_.mode.c_str(), mode.c_str(), setpoint_publish_count_);
        return false;
    }

    // ===== Path 2: COMMAND_LONG 备用通道 =====
    // 某些 PX4 commander 状态下 SET_MODE 会被静默忽略（result=ACCEPTED 但 mode 不变），
    // COMMAND_LONG 走 handle_command 这条路径，PX4 会打明确的拒绝原因，
    // 在我们的场景里它实际上更可靠。
    void sendSetModeCommandLong(const std::string& mode) {
        if (!command_long_client_.exists()) {
            ROS_WARN("[MissionManager] >>>>> COMMAND_LONG service not available");
            return;
        }

        // PX4 custom main mode 编号（v1.10+ 稳定）
        static const std::map<std::string, uint32_t> px4_modes = {
            {"MANUAL", 1}, {"ALTCTL", 2}, {"POSCTL", 3}, {"AUTO", 4},
            {"ACRO", 5}, {"OFFBOARD", 6}, {"STABILIZED", 7}, {"RATTITUDE", 8},
        };

        auto it = px4_modes.find(mode);
        if (it == px4_modes.end()) {
            ROS_WARN("[MissionManager] >>>>> Unknown mode '%s' for COMMAND_LONG", mode.c_str());
            return;
        }

        mavros_msgs::CommandLong cmd;
        cmd.request.command = 176;                       // MAV_CMD_DO_SET_MODE
        cmd.request.param1 = 0x8C;                       // base_mode: SAFETY_ARMED | CUSTOM_MODE_ENABLED | GUIDED | STABILIZE
        cmd.request.param2 = static_cast<float>(it->second); // PX4 custom main mode
        cmd.request.param3 = 0.0;                        // custom_sub_mode
        cmd.request.param4 = 0.0;
        cmd.request.param5 = 0.0;
        cmd.request.param6 = 0.0;
        cmd.request.param7 = 0.0;
        cmd.request.broadcast = false;
        cmd.request.confirmation = 0;

        ROS_WARN("[MissionManager] >>>>> COMMAND_LONG DO_SET_MODE: base=0x%X custom=%d (%s)",
                 (int)cmd.request.param1, (int)cmd.request.param2, mode.c_str());

        if (!command_long_client_.call(cmd)) {
            ROS_WARN("[MissionManager] >>>>> COMMAND_LONG service call FAILED (RPC error)");
            return;
        }

        ROS_WARN("[MissionManager] >>>>> COMMAND_LONG response: success=%d result=%d (result: 0=ACCEPTED 1=TEMP_REJ 2=DENIED 3=UNSUPPORTED)",
                 cmd.response.success, cmd.response.result);
    }

    /**
     * 设置 PX4 MAVLink 消息发送频率（MAV_CMD_SET_MESSAGE_INTERVAL=511）
     * 某些板子上电后默认 LOCAL_POSITION_NED 频率太低 (1Hz 或 5Hz),
     * 必须上电后调一次才能让 OFFBOARD 控制流畅。
     *
     * @param msg_id     MAVLink 消息 ID (e.g. 32 = LOCAL_POSITION_NED)
     * @param rate_hz    目标频率 (Hz)
     * @return true 表示调用成功 (success=true 且 result=0=ACCEPTED)
     */
    bool setMessageRate(uint32_t msg_id, double rate_hz) {
        if (!command_long_client_.exists()) {
            ROS_WARN("[MissionManager] >>>>> command_long service not available, "
                     "cannot set msg %u rate", msg_id);
            return false;
        }
        if (rate_hz <= 0.0) {
            ROS_WARN("[MissionManager] >>>>> Invalid rate %.1f Hz for msg %u, skip", rate_hz, msg_id);
            return false;
        }

        mavros_msgs::CommandLong cmd;
        cmd.request.command = 511;                                // MAV_CMD_SET_MESSAGE_INTERVAL
        cmd.request.param1 = static_cast<float>(msg_id);          // 消息 ID
        cmd.request.param2 = static_cast<float>(1000000.0 / rate_hz);  // interval (us)
        cmd.request.param3 = 0.0;
        cmd.request.param4 = 0.0;
        cmd.request.param5 = 0.0;
        cmd.request.param6 = 0.0;
        cmd.request.param7 = 0.0;
        cmd.request.broadcast = false;
        cmd.request.confirmation = 0;

        ROS_WARN("[MissionManager] >>>>> SET_MESSAGE_INTERVAL msg_id=%u rate=%.1fHz (interval=%.0fus)",
                 msg_id, rate_hz, cmd.request.param2);

        if (!command_long_client_.call(cmd)) {
            ROS_WARN("[MissionManager] >>>>> SET_MESSAGE_INTERVAL service call FAILED (RPC error)");
            return false;
        }

        // MAV_CMD result: 0=ACCEPTED, 1=TEMP_REJ, 2=DENIED, 3=UNSUPPORTED, 4=FAILED, 5=IN_PROGRESS
        bool ok = (cmd.response.success && cmd.response.result == 0);
        ROS_WARN("[MissionManager] >>>>> SET_MESSAGE_INTERVAL result: success=%d result=%d (%s)",
                 cmd.response.success, cmd.response.result, ok ? "ACCEPTED" : "REJECTED");
        return ok;
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

    /**
     * ============================================================
     * OFFBOARD + ARM 快速序列（核心修复）
     * ============================================================
     *
     * 为什么要用这个序列而不是 setMode() 等 5 秒？
     *
     * 1. PX4 commander 内部逻辑:
     *    当收到 SET_MODE(OFFBOARD) 后，commander 会切到 MAIN_STATE_OFFBOARD，
     *    但紧接着每 100~500ms 会重新检查 is_offboard_available()：
     *      if (main_state == OFFBOARD && !is_offboard_available())
     *          main_state = previous_main_state;   // 立刻 revert
     *    这个 is_offboard_available() 检查的是 offboard 模块是否最近收到过 setpoint，
     *    正常情况下 mavros 在持续发 setpoint，应该返回 true。但如果有任何原因导致
     *    offboard 模块的 update 滞后于 commander 检查（CPU 抖动、MAVLink 延迟、单串口
     *    互斥等），PX4 会瞬间 revert 回 AUTO.LOITER。
     *
     * 2. pymavlink 为什么能切成功:
     *    它用的就是 race-through — SET_MODE → 50ms → ARM。在 revert 窗口内 ARM
     *    到达，commander 进入 ARM transition 状态，OFFBOARD 模式在 transition 期间
     *    被"卡住"，PX4 不得不保留 OFFBOARD 直到 ARM 完成。
     *
     * 3. 我们之前的代码 (setMode 等 5 秒) 错就错在:
     *    等到 PX4 已经 revert 回 LOITER 才去 ARM，结果 ARM 是在 LOITER 模式下请求的，
     *    UAV 上了 LOITER 的锁，但 mode 还是 LOITER — setpoint 被忽略，UAV 不动。
     *
     * 4. 这个函数的策略:
     *    SET_MODE → 50ms → COMMAND_LONG → 50ms → ARM。
     *    完整序列 ≤ 200ms，远小于 commander 的 revert 窗口。
     *    ARM 之后再做最多 3 次重试（每次 50ms），命中 PX4 还在 transition 的窗口。
     *
     * 返回：true 表示 ARM 成功（最关键的指标，UAV 能否起飞就靠它），
     *       false 表示 ARM 失败；上层 state machine 会再次重试整组动作。
     */
    bool triggerOffboardAndArm() {
        ros::Time t0 = ros::Time::now();

        // -------- Step 1: SET_MODE service (MAVLink SET_MODE) --------
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = "OFFBOARD";
        bool set_mode_sent = false;
        if (set_mode_client_.call(set_mode)) {
            set_mode_sent = set_mode.response.mode_sent;
            ROS_WARN("[MissionManager] >>>> [%4.0fms] SET_MODE service: mode_sent=%d (PX4 mode=%s)",
                     (ros::Time::now() - t0).toSec() * 1000.0,
                     set_mode_sent ? 1 : 0,
                     current_mavros_state_.mode.c_str());
        } else {
            ROS_WARN("[MissionManager] >>>> [%4.0fms] SET_MODE service: RPC call FAILED",
                     (ros::Time::now() - t0).toSec() * 1000.0);
        }

        // -------- Step 2: 50ms 后 COMMAND_LONG DO_SET_MODE --------
        // 命中 PX4 commander handle_command 路径 — 某些固件版本对这条路径更稳定
        ros::Duration(0.05).sleep();
        ROS_WARN("[MissionManager] >>>> [%4.0fms] sending COMMAND_LONG DO_SET_MODE OFFBOARD (PX4 mode=%s)",
                 (ros::Time::now() - t0).toSec() * 1000.0,
                 current_mavros_state_.mode.c_str());
        sendSetModeCommandLong("OFFBOARD");

        // -------- Step 3: 50ms 后 直接 ARM，不等 mode 确认 --------
        // 这是关键时机 — ARM 必须在 commander 还没 revert 之前到达
        ros::Duration(0.05).sleep();
        ROS_WARN("[MissionManager] >>>> [%4.0fms] attempting ARM #1 (PX4 mode=%s)",
                 (ros::Time::now() - t0).toSec() * 1000.0,
                 current_mavros_state_.mode.c_str());
        bool armed = armVehicle(true);

        // -------- Step 4: ARM 失败重试最多 3 次（每次 50ms） --------
        // PX4 commander 可能在 OFFBOARD transition 中，ARM 会被暂缓一会儿
        for (int i = 0; i < 3 && !armed && ros::ok(); ++i) {
            ros::Duration(0.05).sleep();
            ROS_WARN("[MissionManager] >>>> [%4.0fms] ARM retry %d/3 (PX4 mode=%s armed=%d)",
                     (ros::Time::now() - t0).toSec() * 1000.0, i + 1,
                     current_mavros_state_.mode.c_str(),
                     current_mavros_state_.armed ? 1 : 0);
            armed = armVehicle(true);
        }

        ros::Duration(0.05).sleep();
        ROS_WARN("[MissionManager] >>>> [%4.0fms] triggerOffboardAndArm DONE: armed=%d, "
                 "PX4 mode=%s armed=%d sys_status=%d",
                 (ros::Time::now() - t0).toSec() * 1000.0, armed ? 1 : 0,
                 current_mavros_state_.mode.c_str(),
                 current_mavros_state_.armed ? 1 : 0,
                 current_mavros_state_.system_status);
        return armed;
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

    std::string missionPhaseToString() {
        switch (current_phase_) {
            case MissionPhase::PHASE_GROUND_IDLE:      return "GROUND_IDLE";
            case MissionPhase::PHASE_TAKING_OFF:       return "TAKING_OFF";
            case MissionPhase::PHASE_HOVERING:         return "HOVERING";
            case MissionPhase::PHASE_WAYPOINT_FOLLOW:  return "WAYPOINT_FOLLOW";
            case MissionPhase::PHASE_GUIDANCE_TRACK:   return "GUIDANCE_TRACK";
            case MissionPhase::PHASE_GUIDANCE_STRIKE:  return "GUIDANCE_STRIKE";
            case MissionPhase::PHASE_RETURNING:        return "RETURNING";
            case MissionPhase::PHASE_COMPLETE:         return "COMPLETE";
            case MissionPhase::PHASE_FAILED:           return "FAILED";
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
