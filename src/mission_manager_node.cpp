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

 using namespace std;

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/HomePosition.h>
#include <tf/transform_datatypes.h> // 用于RPY转四元数

#include <string>
#include <cmath>
#include <cstdint>
#include <thread>
#include <chrono>
#include <sstream>
#include <boost/make_shared.hpp>

// === Phase 1+ 类型化消息 ===
#include "multi_uav_strike/TaskFlow.h"
#include "multi_uav_strike/Skill.h"
#include "multi_uav_strike/WorkMode.h"
#include "multi_uav_strike/AttackCmd.h"
#include "multi_uav_strike/MissionState.h"
#include "multi_uav_strike/DetectTarget.h"
#include "multi_uav_strike/DetectTargets.h"
#include "multi_uav_strike/YoloDetection.h"
#include "multi_uav_strike/TrackingState.h"
#include "multi_uav_strike/WaypointStatus.h"
#include "multi_uav_strike/UavGatherStatus.h"

// 工作模式枚举
// WorkMode 只表达「在任务区域内做什么」(对应 TZS MAV_CMD_SET_WORKMODE 语义)。
// 起飞/落地/返航/集结等阶段不归 WorkMode 管,由 MissionPhase 表达。
// 与 WorkMode.msg 同步:
//   IDLE              = 0  (保留,UAV 空闲)
//   SEARCH_ONLY       = 1
//   SEARCH_TRACK      = 2
//   SEARCH_STRIKE     = 3
//   DENIED_ENV_FLIGHT = 4  (预留,未实现)
enum class WorkMode {
    IDLE,             // 等待:只获取 UAV 信息,不发任何控制指令(PX4 SITL 不发 setpoint)
    SEARCH_ONLY,      // 全图搜索
    SEARCH_TRACK,     // 搜索即跟踪
    SEARCH_STRIKE,    // 搜索即打击
    DENIED_ENV_FLIGHT // 拒止环境飞行(预留,GPS 拒止/强对抗场景,未实现)
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
    // ===== 弹射起飞特有状态(skill_type=100) =====
    TAKEOFF_CATAPULT_ARMED,     // 弹射模式就绪,等外部 trigger
    TAKEOFF_CATAPULT_TRIGGERED, // 收到 trigger,等 PX4 进 POSCTL
    TAKEOFF_CATAPULT_POSCTL,    // PX4 POSCTL 稳定,准备 OFFBOARD 接管
    // ===== 标准起飞流程(地面 + 弹射后段共用) =====
    TAKEOFF_SETTING_OFFBOARD, // 正在切换 OFFBOARD 模式
    TAKEOFF_TAKEOFF_EXEC,  // 执行起飞爬升
    TAKEOFF_HOVERING,      // 悬停等待
    TAKEOFF_COMPLETE,      // 起飞完成
    TAKEOFF_FAILED         // 起飞失败
};

// 内部任务执行阶段 (独立于 WorkMode, 由 mission_manager 自己推进)
// WorkMode 是 GS 的意图 (SEARCH_*), MissionPhase 是实际在执行什么
// 引入原因: 地面站把"起飞+航点"打包成一个任务, 期间 WorkMode 由 SET_WORKMODE 控制,
// 但内部需要在 HOVERING 完成后自动推进到航点跟踪; 推进过程由 phase 表达,不污染 work_mode。
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
    PHASE_HOLDING            // 所有 skill 完成后原地悬停, 等新 task_flow
};

// === Phase 2: Skill 生命周期 ===
// 单条 Skill 在机载内部的执行阶段,独立于 WorkMode(MissionPhase)
enum class SkillState {
    PENDING,         // 已入队,等待上一 skill 完成
    TRANSIT,         // fly arrive_path,等门控:executor 报 ARRIVE_DONE
    ENTRY_PENDING,   // 已到 arrive 末点,等门控:接近 skill_area_path[0]
    IN_TASK,         // 准入门触发,执行 skill 本体(搜索/集结等待/打击)
    EXIT_PENDING,    // executor 报 SKILL_AREA_DONE,等一帧发最终遥测
    COMPLETE,        // skill 已完成
    FAILED           // 超时或不容许状态转换
};

// === Phase 2: 当前 Skill 的运行时容器 ===
struct SkillRuntime {
    multi_uav_strike::Skill  msg;            // 输入 Skill 完整拷贝
    SkillState               state;          // 当前生命周期阶段
    ros::Time                state_enter_time;
    // 准入门计时(ENTRY_PENDING 等待超过此时间 → FAILED)
    ros::Time                entry_gate_enter_time;
    // 任务区耗时上限(IN_TASK 等待超过此时间 → FAILED,可配 0=无限)
    ros::Time                in_task_enter_time;
    // 集结超时计时(仅 skill_type=101 适用)
    ros::Time                gather_enter_time;
    // 暂停/失败原因,日志用
    std::string              last_event;
};

// === Phase 5: 目标锁定状态(供 attack_cmd 决��前置) ===
enum class TrackLockState {
    NOT_LOCKED,        // 未识别或识别后已退出
    LOCKED_WAIT_CONFIRM, // SEARCH_TRACK 模式:识别到目标,等地面站 attack_cmd
    LOCKED_AUTO          // SEARCH_STRIKE 模式:识别即上报,可直接触发 strike
};

struct TrackedTarget {
    bool                  is_valid;
    multi_uav_strike::DetectTarget::ConstPtr latest;
    TrackLockState        lock_state;
    ros::Time             locked_at;
    // 锁定瞬间 YOLO 的原始 label (string,如 "person"/"car"),
    // 用于 attack_cmd action=1/2 加黑名单时携带 label 信息
    // (DetectTarget.label 是 uint32 占位 =1,丢失了原始 label)
    std::string           yolo_label;
};

// === 目标级"已忽略"黑名单(label + TTL) ===
// 用于解决 SEARCH_TRACK 模式下的目标聚类缺失问题:
//   1. UAV 检测并锁定目标 A (label="person")
//   2. GS 发 attack_cmd action=1/2 (忽略/暂存)
//   3. 当前实现: tracked_target_ 清空 → 下一帧 YOLO 再命中同一目标 → 又重新锁
//   4. 加本黑名单后: attack_cmd 触发时记录 label+到期时间;
//      后续 checkYoloDrivenStrike 进门控时检查新命中 label 是否在黑名单 + 未过期
//      → 视为同一类目标(由 GS ignore 过),跳过锁定。
//
// 设计权衡(2026-07 改为 label-only):
//   - 原始方案带空间半径(20m)需要靠 UAV 当前位置做 proxy,
//     但 DetectTarget.obj_lat/lon 实际是 UAV pose 占位(不是真实目标位置),
//     UAV 一边扫描一边移动,3s 就走出 20m,空间匹配失效。
//   - 改 label-only:20s 内同 label 一律忽略,扫描/悬停都稳定;空间维度
//     留给未来云台+UAV pose 反推真实目标位置(C 方案)落地后再补。
//   - 不持久化(节点重启即清空),只防同一任务流内重复
//   - 与未来 reported_targets_ 上报去重列表解耦,语义独立
//   - 用户的语义("同位置忽略人后再看到车可追")由 label 区分已满足
struct IgnoredTarget {
    std::string  label;          // YoloDetection.label 原值,如 "person"/"car"
    ros::Time    ignore_until;   // 过期时间 (now + ~ignored_retention_sec_)
    ros::Time    ignore_set_time;// 用于日志/debug
    // 保留位置字段以便未来 C 方案落地时直接复用(暂不参与匹配)
    double       ned_x           = 0.0;
    double       ned_y           = 0.0;
    double       ned_alt         = 0.0;
};

class MissionManager {
private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ============== 订阅 ==============
    ros::Subscriber yolo_typed_sub_;  // typed YoloDetection(detection_simulator_node 发布,Phase 5 启用)
    ros::Subscriber gimbal_los_sub_;
    ros::Subscriber target_est_pose_sub_;
    ros::Subscriber target_est_twist_sub_;
    ros::Subscriber other_uav_poses_sub_;
    ros::Subscriber inter_uav_target_sub_;
    ros::Subscriber self_pose_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber mavros_state_sub_;    // PX4 SITL: 飞控状态
    // === Phase 2: 类型化订阅 ===
    ros::Subscriber task_flow_sub_;        // 任务流入口(multi_uav_strike/TaskFlow)
    ros::Subscriber work_mode_sub_;        // 工作模式 typed(multi_uav_strike/WorkMode)
    ros::Subscriber attack_cmd_sub_;       // MAV_CMD_ATTACK typed(multi_uav_strike/AttackCmd)
    // yolo_typed_sub_ 在上方已声明
    ros::Subscriber waypoint_status_sub_;  // 航段状态(Phase 4 启用)

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
    ros::Publisher guidance_speed_pub_;  // 拦截速度(下发到 guidance_control)
    ros::Publisher avoidance_vector_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher status_pub_;
    ros::Publisher uav_pose_nwu_pub_;     // NWU姿态发布(RViz用)
    // === Phase 2: 类型化发布 ===
    ros::Publisher mission_state_pub_;       // 1Hz 定频上报 multi_uav_strike/MissionState
    ros::Publisher detect_target_pub_;       // 单条 target:multi_uav_strike/DetectTarget
    ros::Publisher detect_targets_pub_;      // 批量:multi_uav_strike/DetectTargets
    ros::Publisher tracking_state_pub_;      // multi_uav_strike/TrackingState
    ros::Publisher waypoint_skill_pub_;      // 向 executor 下发 Skill(分段航点)
    ros::Publisher gather_status_pub_;       // 集结多机状态(Phase 7)

    // ============== 定时器 ==============
    ros::Timer mission_timer_;
    ros::Timer avoidance_timer_;
    ros::Timer mission_state_timer_;   // Phase 2: 1Hz MissionState 定频上报
    ros::Timer skill_advance_timer_;   // Phase 3: Skill 状态机推进(独立频率)

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

    // GPS 参考点(机间避障时将邻居 GPS 转本地 NED)。
    // 不再 yaml 硬编码 — 由 homePositionCallback() 从 PX4 /mavros/home_position/home 自动加载
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;
    bool ref_initialized_ = false;
    ros::Subscriber home_position_sub_;

    // 避障参数
    double avoidance_safe_distance_;
    geometry_msgs::Point current_avoidance_vector_;

    // 毫米波避障参数
    double low_speed_threshold_;  // 12 m/s
    bool is_obstacle_detected_;

    // 下发给 guidance_control_node 的拦截速度(InterceptGuidance 用)
    double guidance_speed_;
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
    // ===== 弹射起飞流程状态(skill_type=100) =====
    // TODO(skill_type_mapping): 协议升级后 skill_type=100 仅表示弹射,
    //   skill_type=106 表示地面起飞;此处所有判定同步更新
    bool is_catapult_takeoff_ = false;          // 当前 task_flow 是否为弹射起飞
    bool is_px4_catapult_mode_ = false;         // PX4 是否已被 comm_node 切到抛飞模式
    bool catapult_trigger_received_ = false;    // 是否收到 /mission/catapult_trigger
    ros::Time catapult_posctl_enter_time_;      // PX4 进入 POSCTL 的时刻(用于稳定等待)
    double catapult_posctl_stable_sec_ = 1.0;   // POSCTL 持续多久才算稳定
    ros::Subscriber catapult_trigger_sub_;
    ros::Subscriber px4_catapult_mode_sub_;
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

    // ============== PX4 返航降落 (Return skill 完成后) ==============
    // 当任务流的最后一个 skill 是 Return(skill_type=103)且其 COMPLETE 时,
    // mission_manager 切到 PHASE_RETURNING 并通过 setMode("AUTO.LAND") 让 PX4
    // 自带降落+着陆后自动 disarm。落地后保持 RETURNING,UAV 不再起 setpoint publisher。
    // 适用:
    //   - 任务流结束于返航 → 自动落地收尾
    //   - 任务流中段包含 Return → 仅落地,不结束任务(交给后续 skill)
    //     (但当前业务上 Return 一般作为最后一条 skill)
    // 多机协同:其他 UAV 不应把 PHASE_RETURNING 当成"可抢占",应继续执行各自任务。
    bool        is_landing_in_progress_;     // 是否正在执行 AUTO.LAND 流程
    ros::Time   landing_start_time_;         // 触发 AUTO.LAND 的时刻(用于安全超时)
    bool        landing_complete_logged_;    // 是否已打印"落地完成"日志(避免刷屏)
    double      landing_safety_timeout_sec_; // 着陆后仍未 disarm 的兜底超时(秒)

    // ============== MissionPhase 状态机 ==============
    MissionPhase current_phase_;         // 当前任务执行阶段
    bool is_waypoints_received_;         // 地面站是否已下发航点
    size_t current_waypoint_count_;      // 当前收到的航点数量
    // is_takeoff_handoff_done_ 已删除:phase 接管协调职责后,该 flag 不再需要
    // (原逻辑靠 work_mode=SEARCH_ONLY 标志判断"已交接",会污染 work_mode 语义,
    //  现在由 current_phase_==PHASE_WAYPOINT_FOLLOW 表达交接完成)
    MissionPhase phase_before_takeoff_;   // 进入 PHASE_TAKING_OFF 前缓存的 phase,performTakeoffHandoff 失败时恢复用
    WorkMode work_mode_before_takeoff_;  // 起飞 skill 到来时缓存的用户 work_mode(TAKEOFF skill 完成后恢复,默认 IDLE)
    WorkMode post_takeoff_work_mode_;     // 起飞完成后 work_mode 兜底值(~post_takeoff_work_mode,默认 SEARCH_ONLY, 兼容老 GS 不发 SET_WORKMODE 的场景)

    // === Phase 2+: Skill 流运行时数据 ===
    std::vector<SkillRuntime> skill_queue_;  // 多条 Skill 顺序排队
    size_t current_skill_index_;             // 当前推进中的 skill 下标(0-based)
    bool has_task_flow_;                     // 是否有有效任务流(true 时不接收入参覆盖)
    std::string current_flow_id_;            // 当前任务流的 flow_id(上报 MissionState 用)
    int self_device_id_;                    // 本机 device_id(任务流的 device_id,默认自身)
    // 各 skill_type 的超时设置(秒,0=无限)
    double entry_gate_timeout_;   // ENTRY_PENDING 默认 30s
    double in_task_timeout_;      // IN_TASK 默认 0(无限)
    double gather_timeout_;       // 101 集结合计超时默认 60s
    // 期望集结 UAV SN 列表(参数 --expected-sns "uav0,uav2"),101 用
    std::vector<std::string> expected_sns_;
    // task_flow 最新缓存(防止回调顺序与启动冲突)
    multi_uav_strike::TaskFlow::ConstPtr latest_task_flow_;
    // 来自 typed WorkMode 的 topic,优先级高于 String(过渡期双订阅)
    uint8_t typed_work_mode_;
    ros::Time typed_work_mode_set_time_;
    bool has_typed_work_mode_;
    // 来自 typed YoloDetection 的最新一帧
    multi_uav_strike::YoloDetection::ConstPtr latest_yolo_;
    bool has_latest_yolo_;
    ros::Time latest_yolo_time_;
    // 跟踪目标(typed)
    TrackedTarget tracked_target_;

    // === 已忽略目标黑名单 ===
    std::vector<IgnoredTarget> ignored_targets_;
    double ignored_retention_sec_;     // 黑名单保留时长 (秒)
    double ignored_match_radius_m_;    // 已弃用:label-only 方案不再做位置匹配;
                                       //   保留 param 仅为兼容 yaml 配置,代码内不读取
    ros::Time last_ignored_cleanup_;   // 上次清理过期项的时间(避免每帧遍历)
    // WaypointStatus(Phase 4)
    multi_uav_strike::WaypointStatus::ConstPtr latest_wp_status_;
    ros::Time latest_wp_status_time_;
    // 救生索:当 typed WorkMode 与 String 模式冲突时优先 typed
    bool prefer_typed_mode_ = true;  // 保留字段但固定 true(只走 typed WorkMode)
    // MissionState 1Hz 已发布标志(避免重复打印)
    ros::Time last_mission_state_pub_time_;

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
        guidance_speed_(10.0),    // 默认 10 m/s,可在 mission_manager.yaml 的 ~max_speed 覆盖
        strike_distance_threshold_(2.0),
        mission_loop_rate_(50.0),
        target_lock_confidence_(0.7),
        spiral_approach_duration_(10.0),
        use_sim_(false),
        self_device_id_(0),
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
        is_landing_in_progress_(false),
        landing_start_time_(ros::Time()),
        landing_complete_logged_(false),
        landing_safety_timeout_sec_(120.0),  // AUTO.LAND 兜底超时 120s
        current_phase_(MissionPhase::PHASE_GROUND_IDLE),
        is_waypoints_received_(false),
        current_waypoint_count_(0),
        phase_before_takeoff_(MissionPhase::PHASE_GROUND_IDLE),
        work_mode_before_takeoff_(WorkMode::IDLE),
        post_takeoff_work_mode_(WorkMode::SEARCH_ONLY),
        local_position_rate_hz_(20.0),       // 默认 20Hz (50ms), 板子一般够用
        is_local_position_rate_set_(false),
        set_message_rate_retry_count_(0),
        max_set_message_rate_retries_(3),
        set_message_rate_start_time_(ros::Time()),  // isZero() 表示还没开始尝试
        set_message_rate_max_wait_sec_(5.0),
        ignored_retention_sec_(20.0),       // 黑名单默认保留 20s
        ignored_match_radius_m_(20.0),      // 空间匹配默认 20m
        last_ignored_cleanup_(ros::Time()) {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();
        initTargetState();

        ROS_INFO("[MissionManager] Initialized. Low speed threshold: %.1f m/s, use_sim: %s",
                 low_speed_threshold_, use_sim_ ? "true" : "false");
    }

    void initParams() {
        nh_private_.param<int>("device_id", self_device_id_, 0);
        nh_private_.param<double>("avoidance_safe_distance", avoidance_safe_distance_, 10.0);
        nh_private_.param<double>("low_speed_threshold", low_speed_threshold_, 12.0);
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);
        nh_private_.param<double>("mission_loop_rate", mission_loop_rate_, 50.0);
        nh_private_.param<double>("target_lock_confidence", target_lock_confidence_, 0.7);
        nh_private_.param<double>("spiral_approach_duration", spiral_approach_duration_, 10.0);
        nh_private_.param<double>("spiral_approach_radius", spiral_approach_radius_, 20.0);

        // 仿真/真机切换
        nh_private_.param<bool>("use_sim", use_sim_, false);
        nh_private_.param<double>("takeoff_altitude", takeoff_altitude_, 50.0);
        nh_private_.param<int>("max_takeoff_retries", max_takeoff_retries_, 3);
        nh_private_.param<double>("takeoff_retry_delay", takeoff_retry_delay_, 5.0);
        nh_private_.param<double>("takeoff_stable_time", takeoff_stable_time_, 1.0);  // 高度达标后稳定时间
        nh_private_.param<double>("takeoff_hover_time", takeoff_hover_time_, 0.5);    // 悬停等待时间
        nh_private_.param<double>("local_position_rate_hz", local_position_rate_hz_, 20.0);  // LOCAL_POSITION_NED 发送频率
        nh_private_.param<int>("max_set_message_rate_retries", max_set_message_rate_retries_, 3);
        nh_private_.param<double>("set_message_rate_max_wait_sec", set_message_rate_max_wait_sec_, 5.0);
        // GPS 参考点: 由 PX4 home 自动填充,param() 仅作 fallback
        nh_private_.param<double>("ref_lat", ref_lat_, 36.096);
        nh_private_.param<double>("ref_lon", ref_lon_, 114.392);
        nh_private_.param<double>("ref_alt", ref_alt_, 100.0);

        // 起飞后 work_mode 兜底值(默认 SEARCH_ONLY,兼容老 GS 不发 SET_WORKMODE 的场景)
        // 用户已通过 SET_WORKMODE 设过 SEARCH_TRACK/STRIKE 时,起飞后保留用户的设置,不会落到这里
        {
            std::string post_takeoff_mode_str;
            nh_private_.param<std::string>("post_takeoff_work_mode", post_takeoff_mode_str, "SEARCH_ONLY");
            if      (post_takeoff_mode_str == "IDLE")              post_takeoff_work_mode_ = WorkMode::IDLE;
            else if (post_takeoff_mode_str == "SEARCH_ONLY")       post_takeoff_work_mode_ = WorkMode::SEARCH_ONLY;
            else if (post_takeoff_mode_str == "SEARCH_TRACK")      post_takeoff_work_mode_ = WorkMode::SEARCH_TRACK;
            else if (post_takeoff_mode_str == "SEARCH_STRIKE")     post_takeoff_work_mode_ = WorkMode::SEARCH_STRIKE;
            else if (post_takeoff_mode_str == "DENIED_ENV_FLIGHT") post_takeoff_work_mode_ = WorkMode::DENIED_ENV_FLIGHT;
            else {
                ROS_WARN_THROTTLE(5.0, "[MissionManager] Unknown ~post_takeoff_work_mode='%s', fallback to SEARCH_ONLY",
                                  post_takeoff_mode_str.c_str());
                post_takeoff_work_mode_ = WorkMode::SEARCH_ONLY;
            }
        }

        // 根据 use_sim 设置 topic
        if (use_sim_) {
            pose_topic_ = "quad/pose";
        } else {
            pose_topic_ = "mavros/local_position/pose";
        }

        // === Phase 2: Skill 流相关参数 ===
        nh_private_.param<double>("entry_gate_timeout", entry_gate_timeout_, 30.0); // ENTRY_PENDING 上限 30s
        nh_private_.param<double>("in_task_timeout",    in_task_timeout_, 0.0);    // IN_TASK 默认不限(0)
        nh_private_.param<double>("gather_timeout",     gather_timeout_, 60.0);    // 101 集结合计超时 60s

        // === 已忽略目标黑名单参数 (Phase 5.5: 目标聚类轻量替代) ===
        nh_private_.param<double>("ignored_retention_sec",  ignored_retention_sec_,  20.0);  // 默认 20s
        nh_private_.param<double>("ignored_match_radius_m", ignored_match_radius_m_, 20.0);  // 默认 20m

        // 期望集结 UAV SN 列表(逗号分隔字符串,如 "uav0,uav2")
        std::string sns_str;
        nh_private_.param<std::string>("expected_sns", sns_str, "");
        expected_sns_.clear();
        if (!sns_str.empty()) {
            std::stringstream ss(sns_str);
            std::string item;
            while (std::getline(ss, item, ',')) {
                if (!item.empty()) expected_sns_.push_back(item);
            }
        }
        if (!expected_sns_.empty()) {
            ROS_INFO("[MissionManager] Expected gather SNs: %zu", expected_sns_.size());
        }

    }

    void initSubscribers() {
        // YOLO 检测结果:由 typed YoloDetection 统一接管(multi_uav_strike/YoloDetection)
// 删除原 std_msgs/String legacy 订阅 yolo_result_sub_,避免与 detection_simulator_node
// 发布的 YoloDetection 类型冲突

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

        // === Phase 2: 类型化订阅 ===
        // TaskFlow 入口(本机 device_id 匹配的 flow 才会被采纳)
        task_flow_sub_ = nh_.subscribe("mission/task_flow", 10,
                                       &MissionManager::taskFlowCallback, this);

        // 类型化 WorkMode(GS 升级到 typed 时启用,与 String 模式并存双订阅)
        work_mode_sub_ = nh_.subscribe("mission/work_mode", 10,
                                       &MissionManager::typedWorkModeCallback, this);

        // MAV_CMD_ATTACK typed(地面站确认指令)
        attack_cmd_sub_ = nh_.subscribe("mission/attack_cmd", 10,
                                        &MissionManager::attackCmdCallback, this);

        // typed YoloDetection(detection_simulator_node 发布到同名 topic,但 msg 类型不同)
        yolo_typed_sub_ = nh_.subscribe("detection/yolo_result", 10,
                                        &MissionManager::typedYoloCallback, this);

        // waypoint_executor 段状态(Phase 4 启用)
        waypoint_status_sub_ = nh_.subscribe("waypoint_executor/status", 10,
                                              &MissionManager::waypointStatusCallback, this);

        // === PX4 home 自动加载(ref_lat/lon/alt 的唯一权威来源)===
        home_position_sub_ = nh_.subscribe("mavros/home_position/home", 10,
                                            &MissionManager::homePositionCallback, this);

        // ===== 弹射起飞流程(协议 TODO: skill_type=100 = 弹射, 106 = 地面) =====
        // 弹射指令触发(由外部弹射控制节点发,本节点不实现发端)
        catapult_trigger_sub_ = nh_.subscribe("mission/catapult_trigger", 10,
                                              &MissionManager::catapultTriggerCallback, this);
        // PX4 抛飞模式就绪(由 comm_node 切完 PX4 模式后发 true)
        px4_catapult_mode_sub_ = nh_.subscribe("px4/catapult_mode_ready", 10,
                                               &MissionManager::px4CatapultModeCallback, this);
    }

    void initPublishers() {
        waypoint_control_pub_ = nh_.advertise<std_msgs::String>(
            "waypoint_executor/control", 10);

        guidance_enable_pub_ = nh_.advertise<std_msgs::Bool>(
            "guidance/enable", 10);

        guidance_mode_pub_ = nh_.advertise<std_msgs::String>(
            "guidance/mode", 10);

        guidance_speed_pub_ = nh_.advertise<std_msgs::Float32>(
            "guidance/guidance_speed", 10);

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

        // 已移除 /mission/mode (String) 发布:auto-handoff 通过 typed Skill
        // (pushCurrentSkillToExecutor → waypoint_executor/skill) 通知 executor 起算

        // PX4 SITL: 起飞位置 setpoint 发布器
        if (!use_sim_) {
            takeoff_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                "mavros/setpoint_position/local", 10);
        }

        // === Phase 2: 类型化发布 ===
        // 1Hz MissionState(GS 关心的上报)
        mission_state_pub_ = nh_.advertise<multi_uav_strike::MissionState>(
            "mission/mission_state", 10);

        // 目标上报(单条)
        detect_target_pub_ = nh_.advertise<multi_uav_strike::DetectTarget>(
            "mission/detect_target", 10);

        // 目标上报(批量预留)
        detect_targets_pub_ = nh_.advertise<multi_uav_strike::DetectTargets>(
            "mission/detect_targets", 10);

        // 跟踪状态(协议 43000)
        tracking_state_pub_ = nh_.advertise<multi_uav_strike::TrackingState>(
            "mission/tracking_state", 10);

        // 向 waypoint_executor 下发 Skill(分段航点)
        waypoint_skill_pub_ = nh_.advertise<multi_uav_strike::Skill>(
            "waypoint_executor/skill", 10);

        // 集结多机状态(Phase 7)
        gather_status_pub_ = nh_.advertise<multi_uav_strike::UavGatherStatus>(
            "inter_uav/gather_status", 10);
    }

    void initTimers() {
        mission_timer_ = nh_.createTimer(
            ros::Duration(1.0 / mission_loop_rate_),
            &MissionManager::missionTimerCallback, this);

        avoidance_timer_ = nh_.createTimer(
            ros::Duration(1.0 / mission_loop_rate_),
            &MissionManager::avoidanceTimerCallback, this);

        // === Phase 2: MissionState 1Hz 定频上报 ===
        mission_state_timer_ = nh_.createTimer(
            ros::Duration(1.0),
            &MissionManager::missionStateTimerCallback, this);

        // === Phase 3: Skill 状态机推进(10Hz,稍慢也无妨) ===
        skill_advance_timer_ = nh_.createTimer(
            ros::Duration(0.1),
            &MissionManager::skillAdvanceTimerCallback, this);
    }

    void initTargetState() {
        current_target_.is_detected = false;
        current_target_.is_locked = false;  // 修复:初始未锁定,等 YOLO 命中再置 true
        current_target_.lock_time = 0.0;
        current_target_.is_shared = false;
    }

    // ============== 回调函数 ==============
    // 注: 已移除 /mission/mode (String) 和 /mission/waypoint_cmd (Path) 两条 legacy 路径。
    //     模式与航点的统一入口是 typed /mission/task_flow (TaskFlow.msg) +
    //     可选 typed /mission/work_mode (WorkMode.msg)。
    //     起飞/搜索/打击/返航的阶段切换全部由 taskFlowCallback() 解析完成。

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
     * PX4 home 回调 — 锁一次
     * PX4 的 LOCAL_POSITION_NED 参考系由 EKF2 在启动时锁定,
     * 后续 home 更新(MAV_CMD_DO_SET_HOME / disarm)不会重置 EKF2 origin,
     * 也不让 local_position 跳变。如果跟着 home 更新去重算 NED,
     * 反而让 setpoint 与 UAV 当前 local_position 不在同一 frame → 偏飞。
     */
    void homePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg) {
        if (ref_initialized_) return;  // 锁一次
        if (msg->geo.latitude == 0.0 && msg->geo.longitude == 0.0) {
            return;  // PX4 home 未稳定前发 0/0,忽略
        }
        ROS_WARN("[MissionManager] >>>> PX4 home locked: (%.7f, %.7f, %.2f)",
                 msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
        ref_lat_ = msg->geo.latitude;
        ref_lon_ = msg->geo.longitude;
        ref_alt_ = msg->geo.altitude;
        ref_initialized_ = true;
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
        ROS_INFO_THROTTLE(10.0, "[MissionManager] Received target info from other UAV: %s",
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
            // Mavros 输入是 ENU/FLU 坐标系，需要转换为 NED/FRD
            // 位置: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            current_pose_.pose.position.x = msg->pose.position.y;
            current_pose_.pose.position.y = msg->pose.position.x;
            current_pose_.pose.position.z = -msg->pose.position.z;
            // 四元数 ENU/FLU -> NED/FRD:
            //   q_ned = q_T * q_enu * q_S
            //   q_T = (1/√2, 1/√2, 0, 0)  世界系 ENU→NED(绕(1,1,0)轴 180°)
            //   q_S = (1, 0, 0, 0)         机体系 FLU→FRD(绕 x 轴 180°)
            // 展开后:
            const double kSqrtHalf = 0.7071067811865475;
            const auto& qe = msg->pose.orientation;
            current_pose_.pose.orientation.w = kSqrtHalf * (qe.w + qe.z);
            current_pose_.pose.orientation.x = kSqrtHalf * (qe.x + qe.y);
            current_pose_.pose.orientation.y = kSqrtHalf * (qe.x - qe.y);
            current_pose_.pose.orientation.z = kSqrtHalf * (qe.w - qe.z);
            current_pose_.header = msg->header;
        }
        is_pose_received_ = true;

        // 发布NWU姿态用于RViz显示
        // NED -> NWU: x不变, y取反, z取反
        geometry_msgs::PoseStamped uav_pose_nwu;
        uav_pose_nwu.pose.position.x = current_pose_.pose.position.x;
        uav_pose_nwu.pose.position.y = -current_pose_.pose.position.y;
        uav_pose_nwu.pose.position.z = -current_pose_.pose.position.z;
        // 四元数 NED/FRD -> NWU/FLU:对 yaw,只取 y,z 取反即可保持航向角取负
        // (NED yaw 83° CW from N → NWU yaw -83° CCW from N)
        uav_pose_nwu.pose.orientation.w =  current_pose_.pose.orientation.w;
        uav_pose_nwu.pose.orientation.x =  current_pose_.pose.orientation.x;
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

    /**
     * 弹射起飞 trigger 回调
     * 由外部弹射控制节点触发,本节点不实现发端
     * 仅当 is_catapult_takeoff_=true(当前 task_flow 首发是 skill_type=100)时认账,
     *   否则丢弃(防御性:防止地面起飞误触发)
     */
    void catapultTriggerCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data) return;
        if (!is_catapult_takeoff_) {
            ROS_WARN_THROTTLE(2.0, "[MissionManager] catapult_trigger received but current task_flow "
                                   "is NOT catapult takeoff (skill_type=%s) — ignored",
                              (skill_queue_.empty() ? "none" :
                               std::to_string(skill_queue_[0].msg.skill_type).c_str()));
            return;
        }
        catapult_trigger_received_ = true;
        ROS_WARN("[MissionManager] >>>> Catapult TRIGGER received (armed in PX4 catapult mode=%s)",
                 is_px4_catapult_mode_ ? "YES" : "NO (waiting for comm_node)");
    }

    /**
     * PX4 抛飞模式就绪回调(由 comm_node 切完 PX4 模式后发 true)
     * - true:  comm_node 已把 PX4 切到抛飞模式(MAV_CMD_DO_GO_ARMED/类似指令)
     * - false: 收到 reset,准备 reset 流程
     * 配合 is_catapult_takeoff_ 使用:只有当前 task_flow 是弹射起飞时才接收
     */
    void px4CatapultModeCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_px4_catapult_mode_ = msg->data;
        ROS_WARN("[MissionManager] PX4 catapult_mode_ready = %s",
                 is_px4_catapult_mode_ ? "TRUE (PX4 in catapult mode)" : "FALSE");
    }

    /**
     * Whether UAV is currently in flight (used to reject takeoff task_flow while airborne)
     * Ground states (return false): GROUND_IDLE / FAILED / HOLDING
     * In-flight states (return true): TAKING_OFF / HOVERING / WAYPOINT_FOLLOW /
     *   GUIDANCE_TRACK / GUIDANCE_STRIKE / RETURNING / COMPLETE
     * Note: COMPLETE counts as in-flight since UAV is usually still airborne / returning.
     */
    bool isInFlight() const {
        switch (current_phase_) {
            case MissionPhase::PHASE_GROUND_IDLE:
            case MissionPhase::PHASE_FAILED:
            case MissionPhase::PHASE_HOLDING:
                return false;
            case MissionPhase::PHASE_TAKING_OFF:
            case MissionPhase::PHASE_HOVERING:
            case MissionPhase::PHASE_WAYPOINT_FOLLOW:
            case MissionPhase::PHASE_GUIDANCE_TRACK:
            case MissionPhase::PHASE_GUIDANCE_STRIKE:
            case MissionPhase::PHASE_RETURNING:
            case MissionPhase::PHASE_COMPLETE:
                return true;
            default:
                return false;
        }
    }

    // ============== Phase 2: 类型化回调 ==============

    /**
     * TaskFlow 入口
     * - device_id 匹配 self_device_id_ 的 flow 才会被采纳(其它 UAV 的 flow 透传给 comm_node 转发)
     * - 收到后:清空 skill_queue_,把 flow.skills 拷贝进去,current_skill_index_=0
     *   然后立刻把第一个 skill 推到 waypoint_executor + 标记 has_task_flow_=true
     */
    void taskFlowCallback(const multi_uav_strike::TaskFlow::ConstPtr& msg) {
        // device_id 过滤(0=广播,其它=本机 device_id)
        if (msg->device_id != 0 && msg->device_id != self_device_id_) {
            return;
        }

        // === 飞行安全门:若 UAV 已在飞行中,且新 task_flow 首发是 takeoff skill,
        //     拒绝接收并报警 — 避免 GS 误操作覆盖当前飞行任务 ===
        // 场景:返航中(GROUND 触发)收新 takeoff;航点跟踪中收 takeoff;
        //       guidance 中收 takeoff;都应当拒绝。
        // 例外:phase=GROUND_IDLE/HOLDING 时(地面或悬停待命)允许接收。
        if (!msg->skills.empty() &&
            (msg->skills[0].skill_type == 100 || msg->skills[0].skill_type == 106) &&
            isInFlight()) {
            ROS_ERROR("[MissionManager] >>>> REJECT takeoff task_flow: UAV already in flight "
                      "(phase=%s, takeoff skill_type=%u). GS sent takeoff while airborne — "
                      "ignored for safety. Send IDLE first to abort current mission.",
                      missionPhaseToString().c_str(),
                      static_cast<unsigned>(msg->skills[0].skill_type));
            return;
        }

        // === 返航中收到新 task_flow 的兜底 ===
        // 场景:Return skill 触发了 AUTO.LAND,但中途(未 disarm 前)GS 决定不降了,下发新任务
        // 处理:清掉 is_landing_in_progress_,让 advanceSkillStateMachine 推新 skill
        //   不需要切 PX4 mode(下面会触发起飞流程,OFFBOARD+ARM 序列会接管)
        if (is_landing_in_progress_) {
            ROS_WARN("[MissionManager] New TaskFlow arrived during AUTO.LAND — aborting landing, "
                     "will reset to PHASE_GROUND_IDLE for fresh takeoff");
            is_landing_in_progress_  = false;
            landing_complete_logged_ = false;
            current_phase_          = MissionPhase::PHASE_GROUND_IDLE;
        }

        // 防御：如果当前 setpoint publisher 在跑(说明前序 flow 结束后进入 PHASE_HOLDING 或正在 takeoff),
        // 新 task_flow 到达时先停掉,避免与 waypoint_executor 双发 setpoint 冲突。
        if (setpoint_running_) {
            stopSetpointPublisher();
            ROS_WARN("[MissionManager] Stopped setpoint publisher before processing new task_flow");
        }

        ROS_WARN("[MissionManager] >>>> Received TaskFlow id=%s skills=%lu device_id=%u",
                 msg->flow_id.c_str(), msg->skills.size(), msg->device_id);

        latest_task_flow_ = msg;
        current_flow_id_  = msg->flow_id;
        has_task_flow_    = true;

        // 重置航点接收标志(避免前序 flow 的 is_waypoints_received_ 残留误导 performTakeoffHandoff)
        is_waypoints_received_ = false;

        // 把 skill 流拷进运行时队列
        skill_queue_.clear();
        skill_queue_.reserve(msg->skills.size());
        for (const auto& s : msg->skills) {
            SkillRuntime sr;
            sr.msg = s;
            sr.state = SkillState::PENDING;
            sr.state_enter_time = ros::Time::now();
            sr.last_event = "queued";
            skill_queue_.push_back(sr);
        }
        current_skill_index_ = 0;

        // === Skill flow 适配:TaskFlow 含非空 arrive_path/skill_area_path 的 skill 时,
        //     视为"航点已下发",让 runPx4TakeoffSequence 在 HOVERING 时能触发 performTakeoffHandoff()
        //     否则 takeoff 永远卡在 HOVERING 等 legacy waypointCallback
        bool has_path_skills = false;
        for (const auto& s : msg->skills) {
            if (!s.arrive_path.poses.empty() || !s.skill_area_path.poses.empty()) {
                has_path_skills = true;
                break;
            }
        }
        if (has_path_skills) {
            is_waypoints_received_ = true;
            current_waypoint_count_ = 1;  // 占位,让 performTakeoffHandoff 不报 0
        }

        // 第一个 skill 是 Takeoff(type=100 或 106)时:推进 phase=PHASE_TAKING_OFF
        // 注意:不再操作 work_mode(只表达"在任务区内做什么",起飞是 phase 不是 work_mode)
        // 协议 TODO(skill_type_mapping): 100=弹射起飞,106=地面起飞
        if (!skill_queue_.empty() && (skill_queue_[0].msg.skill_type == 100 || skill_queue_[0].msg.skill_type == 106)) {
            // === 弹射起飞判定 (skill_type=100) ===
            // 设置 is_catapult_takeoff_,并在每个新 task_flow 开始时清掉旧的 trigger 状态
            // 这样:新一发 task_flow 是弹射则重新等 trigger;不是则彻底忽略外部 trigger。
            bool new_is_catapult = (skill_queue_[0].msg.skill_type == 100);
            if (new_is_catapult != is_catapult_takeoff_) {
                ROS_WARN("[MissionManager] Takeoff type changed: %s -> %s "
                         "(resetting catapult trigger state)",
                         is_catapult_takeoff_ ? "CATAPULT(100)" : "GROUND(106)",
                         new_is_catapult ? "CATAPULT(100)" : "GROUND(106)");
            }
            is_catapult_takeoff_       = new_is_catapult;
            catapult_trigger_received_ = false;   // 每个新 task_flow 重新等 trigger
            // is_px4_catapult_mode_ 保持:comm_node 切的模式有持续性,不应每次 flow 重置
            //   (万一 comm_node 没发 false 关闭信号,这里避免误清)
            catapult_posctl_enter_time_ = ros::Time();  // isZero() 表示还没进入 POSCTL

            if (current_phase_ != MissionPhase::PHASE_TAKING_OFF) {
                // 缓存起飞前的 phase 和 work_mode,performTakeoffHandoff 用
                phase_before_takeoff_ = current_phase_;
                work_mode_before_takeoff_ = current_work_mode_;
                // 重置 takeoff_state_ 到 IDLE,支持 re-takeoff(前序 flow 结束后 HOLDING 状态再次起飞)
                if (takeoff_state_ == TakeoffState::TAKEOFF_COMPLETE ||
                    takeoff_state_ == TakeoffState::TAKEOFF_FAILED) {
                    ROS_WARN("[MissionManager] Re-takeoff detected: resetting takeoff_state_ from %s to IDLE",
                             (takeoff_state_ == TakeoffState::TAKEOFF_COMPLETE) ? "COMPLETE" : "FAILED");
                    takeoff_state_ = TakeoffState::TAKEOFF_IDLE;
                }
                current_phase_ = MissionPhase::PHASE_TAKING_OFF;
                ROS_WARN("[MissionManager] TaskFlow Takeoff skill to phase=TAKING_OFF "
                         "(cached prev_phase=%s, prev_work_mode=%s)",
                         missionPhaseToString(phase_before_takeoff_).c_str(),
                         workModeToString(work_mode_before_takeoff_).c_str());
            }
            // === 协议层接管:takeoff_altitude 由 Skill.msg 提供(GS/KCP 端下发)
            //     非 0 表示业务侧给了具体高度,覆盖 mission_manager 私有 param 兜底
            //     skill_type != 100 时该字段忽略(对照 Skill.msg 注释)
            const auto& tk = skill_queue_[0].msg;
            if (tk.takeoff_altitude > 0.0f) {
                takeoff_altitude_ = static_cast<double>(tk.takeoff_altitude);
                ROS_WARN("[MissionManager] Takeoff altitude from Skill msg: %.2f m (relative to home)",
                         takeoff_altitude_);
            } else {
                ROS_WARN("[MissionManager] Takeoff altitude missing in Skill msg, "
                         "fallback to private param ~takeoff_altitude = %.2f m",
                         takeoff_altitude_);
            }
        }

        // 立刻把第一个 skill 推到 waypoint_executor (takeoff 守卫在 pushCurrentSkillToExecutor 内部)
        if (!skill_queue_.empty()) {
            pushCurrentSkillToExecutor();
        }

        // === 协同打击特例:TaskFlow 首发 skill 是 Attack(105)时,
        //     必须保留当前 tracked_target_ —— 不管是 broadcast(GS 协同全队)
        //     还是精确点名本机(device_id == self_device_id_),
        //     都意味着 GS 期望本机用**当前已锁目标**执行这次打击,
        //     清空会让跟踪逻辑重启,目标丢失。
        //     其余情况(非 Attack 首发 / takeoff 等)按原逻辑清空旧目标。
        bool preserve_track = false;
        if (!skill_queue_.empty() && skill_queue_[0].msg.skill_type == 105) {
            preserve_track = true;
        }

        if (!preserve_track) {
            tracked_target_.is_valid = false;
            tracked_target_.lock_state = TrackLockState::NOT_LOCKED;
        } else {
            ROS_WARN("[MissionManager] TaskFlow first skill is Attack(105) "
                     "(device_id=%u). Preserving current track lock.",
                     msg->device_id);
            // current_target_ 不在这里同步 —— Attack skill 推到 waypoint_executor 后
            // 自身会带目标坐标下来覆盖,避免此处重复写造成姿态不一致
        }
        tracked_target_.lock_state = TrackLockState::NOT_LOCKED;
    }

    /**
     * 把当前 skill_queue_[current_skill_index_] 的 Skill 推到 waypoint_executor
     * 并把 state 从 PENDING → TRANSIT
     * 空 queue 时不做事
     */
    void pushCurrentSkillToExecutor() {
        if (current_skill_index_ >= skill_queue_.size()) {
            return;
        }
        const auto& sr = skill_queue_[current_skill_index_];
        // === 守卫:takeoff skill 不推给 executor ===
        // 起飞走 phase 通道 (runPx4TakeoffSequence() + performTakeoffHandoff()),
        //   不归 executor 管。如果把 arrive_path 推给 executor,它会立即开始飞,
        //   而 takeoff 还没爬升到目标高度,两套控制打架(handoff 强制 COMPLETE 也晚了一步)。
        //   即使 arrive_path 为空也不推 — 保持"takeoff 不下发"的语义一致,让
        //   后续 search/gather 真正要飞的航点从下一条 skill 开始下发。
        if (sr.msg.skill_type == 100 || sr.msg.skill_type == 106) {
            return;
        }
        // 推到 executor
        waypoint_skill_pub_.publish(sr.msg);

        // 状态推进 PENDING → TRANSIT
        if (sr.state == SkillState::PENDING) {
            skill_queue_[current_skill_index_].state = SkillState::TRANSIT;
            skill_queue_[current_skill_index_].state_enter_time = ros::Time::now();
            skill_queue_[current_skill_index_].last_event = "TRANSIT: pushed to executor";
            ROS_WARN("[MissionManager] Skill[%zu] id=%s type=%u to TRANSIT",
                     current_skill_index_, sr.msg.skill_id.c_str(), sr.msg.skill_type);
        }
    }

    /**
     * 类型化 WorkMode 回调
     * - 唯一的工作模式入口(已移除 /mission/mode String 兼容路径)
     * - WorkMode 只表达"在任务区域内做什么",阶段语义由 MissionPhase 承载
     */
    void typedWorkModeCallback(const multi_uav_strike::WorkMode::ConstPtr& msg) {
        typed_work_mode_ = msg->mode;
        typed_work_mode_set_time_ = ros::Time::now();
        has_typed_work_mode_ = true;

        // 映射 typed mode → 内部 WorkMode
        // (WorkMode.msg 已精简,本 switch 只处理当前活跃的 5 个值)
        WorkMode new_mode = current_work_mode_;
        switch (msg->mode) {
            case multi_uav_strike::WorkMode::IDLE:
                new_mode = WorkMode::IDLE;
                break;
            case multi_uav_strike::WorkMode::SEARCH_ONLY:
                new_mode = WorkMode::SEARCH_ONLY;
                break;
            case multi_uav_strike::WorkMode::SEARCH_TRACK:
                new_mode = WorkMode::SEARCH_TRACK;
                break;
            case multi_uav_strike::WorkMode::SEARCH_STRIKE:
                new_mode = WorkMode::SEARCH_STRIKE;
                break;
            case multi_uav_strike::WorkMode::DENIED_ENV_FLIGHT:
                new_mode = WorkMode::DENIED_ENV_FLIGHT;
                break;
            default:
                ROS_WARN_THROTTLE(5.0, "[MissionManager] Unknown typed work_mode=%u", msg->mode);
                return;
        }
        if (new_mode != current_work_mode_) {
            WorkMode old_mode = current_work_mode_;
            current_work_mode_ = new_mode;
            ROS_WARN("[MissionManager] Typed WorkMode: %s (overriding %s)",
                     workModeToString(new_mode).c_str(),
                     workModeToString(old_mode).c_str());
        }
    }

    /**
     * MAV_CMD_ATTACK typed 回调
     * 仅在 SEARCH_TRACK 模式下有意义(目标已锁定等地面站确认)
     *
     * action:
     *   0 = 打: 触发 strike + 上报 DetectTarget(type=1)
     *   1 = 忽略: 丢弃目标,继续 search
     *   2 = 暂存: 上报 DetectTarget(type=2),继续 search
     */
    void attackCmdCallback(const multi_uav_strike::AttackCmd::ConstPtr& msg) {
        // 必须在 LOCKED_WAIT_CONFIRM 状态(SEARCH_TRACK 已锁定目标等确认)
        if (!tracked_target_.is_valid ||
            tracked_target_.lock_state != TrackLockState::LOCKED_WAIT_CONFIRM) {
            ROS_WARN_THROTTLE(2.0, "[MissionManager] attack_cmd received but no target locked "
                                   "(action=%u), ignoring", msg->action);
            return;
        }

        multi_uav_strike::DetectTarget dt;
        dt.timestamp_us = ros::Time::now().toNSec() / 1000;
        if (tracked_target_.latest) {
            dt.label        = tracked_target_.latest->label;
            dt.confidence   = tracked_target_.latest->confidence;
            dt.dev_lat      = current_pose_.pose.position.x;  // 简化:用 UAV 位置作 dev
            dt.dev_lon      = current_pose_.pose.position.y;
            dt.dev_alt      = -current_pose_.pose.position.z;
            dt.obj_lat      = tracked_target_.latest->obj_lat;
            dt.obj_lon      = tracked_target_.latest->obj_lon;
            dt.obj_alt      = tracked_target_.latest->obj_alt;
            dt.target_type  = 3;  // 默认普通搜索目标
        }
        dt.img_format = 1;  // JPEG

        switch (msg->action) {
            case 0: {
                // 打击
                dt.target_type = 1;
                ROS_WARN("[MissionManager] AttackCmd action=0 (打) — triggering strike");
                triggerStrike();
                detect_target_pub_.publish(dt);
                break;
            }
            case 1: {
                // 忽略 — 不上报目标,但加黑名单(20s/20m/同 label 内不再自动锁)
                //   否则下一帧 YOLO 再命中同目标,checkYoloDrivenStrike 门控 4
                //   (lock_state==NOT_LOCKED) 满足 → 重新锁 → ignore 指令形同虚设
                //
                // 同时必须停掉已 active 的 guidance:
                //   - startGuidanceApproach() 设了 is_guidance_active_=true
                //   - guidance_control_node 持续向目标位置飞,UAV 不会停
                //   - 后续 YOLO 命中 → 门控 3 (is_guidance_active_) 早返回,门控 7 (黑名单) 永远跑不到
                //   - 必须 disableGuidance() + 发 "resume" 给 waypoint_executor 才能真正"忽略"
                ROS_WARN("[MissionManager] AttackCmd action=1 (忽略) — adding to blacklist, resume search");
                if (tracked_target_.latest) {
                    addIgnoredTarget(*tracked_target_.latest, tracked_target_.yolo_label);
                }
                if (is_guidance_active_) {
                    disableGuidance();
                    ROS_WARN("[MissionManager]   - guidance DISABLED (was active before ignore)");
                }
                // 清掉 TargetState,避免 handleSearchTrack/Strike 误以为"仍锁定"再次触发 guidance
                current_target_.is_locked   = false;
                current_target_.is_detected = false;
                current_target_.is_shared   = false;
                // 重启 waypoint_executor(它现在还是 stop 状态,要 resume 才能继续扫描)
                {
                    std_msgs::String wp_cmd;
                    wp_cmd.data = "resume";
                    waypoint_control_pub_.publish(wp_cmd);
                }
                // phase 回 WAYPOINT_FOLLOW(原本被 startGuidanceApproach 改成 GUIDANCE_TRACK,
                //   MissionState 上报里 GS 看到的应是航点跟踪阶段,而非 guidance)
                current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;
                tracked_target_.is_valid = false;
                tracked_target_.lock_state = TrackLockState::NOT_LOCKED;
                return;
            }
            case 2: {
                // 暂存 — 上报 DetectTarget(type=2) 同时加黑名单(本次跳过,接下来不应再自动锁)
                //   收尾动作与 action=1 完全一致(action=2 仅多 publish 一个 DetectTarget)
                dt.target_type = 2;
                ROS_WARN("[MissionManager] AttackCmd action=2 (暂存) — adding to blacklist");
                detect_target_pub_.publish(dt);
                if (tracked_target_.latest) {
                    addIgnoredTarget(*tracked_target_.latest, tracked_target_.yolo_label);
                }
                if (is_guidance_active_) {
                    disableGuidance();
                    ROS_WARN("[MissionManager]   - guidance DISABLED (was active before stash)");
                }
                current_target_.is_locked   = false;
                current_target_.is_detected = false;
                current_target_.is_shared   = false;
                {
                    std_msgs::String wp_cmd;
                    wp_cmd.data = "resume";
                    waypoint_control_pub_.publish(wp_cmd);
                }
                current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;
                tracked_target_.is_valid = false;
                tracked_target_.lock_state = TrackLockState::NOT_LOCKED;
                return;
            }
            default:
                ROS_WARN("[MissionManager] Unknown attack_cmd action=%u", msg->action);
                return;
        }

        // 无论打还是暂存,清掉锁定态(下一次识别需要重新等确认)
        tracked_target_.is_valid = false;
        tracked_target_.lock_state = TrackLockState::NOT_LOCKED;
    }

    /**
     * 触发 strike(用现有 strikes_distance_threshold 流程)
     * 把现有 TargetState 桥接到 guidance_control_node
     */
    void triggerStrike() {
        // 把跟踪目标写入 current_target_,走现有 startGuidanceApproach()
        if (tracked_target_.latest) {
            current_target_.is_detected = true;
            current_target_.is_locked   = true;
            current_target_.pose.pose.position.x = tracked_target_.latest->obj_lat;
            current_target_.pose.pose.position.y = tracked_target_.latest->obj_lon;
            current_target_.pose.pose.position.z = tracked_target_.latest->obj_alt;
        }
        // 强制进入 SEARCH_STRIKE 分支以触发现有 strike 流程
        current_work_mode_ = WorkMode::SEARCH_STRIKE;
        startGuidanceApproach();
    }

    /**
     * typed YoloDetection 回调
     * 行为取决于 current_work_mode_:
     *   IDLE/TAKEOFF: 忽略
     *   SEARCH_ONLY:   识别即上报 DetectTarget(type=3 普通搜索目标)
     *   SEARCH_TRACK:  首次 is_in_fov=true → 锁目标 + 发布 TrackingState(state=1, 等地面站确认)
     *   SEARCH_STRIKE: 识别即上报 DetectTarget(type=1 攻击目标) + 触发 strike
     */
    void typedYoloCallback(const multi_uav_strike::YoloDetection::ConstPtr& msg) {
        // 缓存最新一帧(给 skill 状态机的 checkYoloDrivenStrike() 用)
        latest_yolo_ = msg;
        has_latest_yolo_ = true;
        latest_yolo_time_ = ros::Time::now();

        if (!msg->is_in_fov) return;
        if (current_work_mode_ == WorkMode::IDLE) {
            return;
        }

        // === 修复:此函数只做目标回传,不做锁定/trigger 控制决策 ===
        // 之前:SEARCH_STRIKE 模式下 YOLO 命中即调 triggerStrike() + 改 lock_state,完全不看
        //        当前 skill 是不是 Attack、是否到了 IN_TASK。结果:arrive_path 阶段或 Search
        //        skill 里 YOLO 命中也会触发 strike,guidance 抢占,waypoint 跟踪被中断。
        // 现在:控制决策挪到 advanceSkillStateMachine IN_TASK 分支的 checkYoloDrivenStrike()。
        //      门控:skill_type==105 + state==IN_TASK + 未锁 + 未制导 + YOLO 有效。
        //      满足才触发 lock/strike。

        // 1) DetectTarget 上报(GS 关心的遥测,所有模式都发)
        multi_uav_strike::DetectTarget dt = buildDetectTarget(*msg, /*target_type=*/3);
        detect_target_pub_.publish(dt);

        // TODO(target_dedup): 当前每帧 YOLO 命中都会 publish DetectTarget,
        //   GS 端会收到大量重复目标上报。后续应实现 reported_targets_ 黑名单
        //   (与 ignored_targets_ 同构:ned_x/y + label + 上报时间),配合"目标
        //   估计稳定"判定(如 N 次连续命中后才上报一次),形成完整 dedup。
        //   与 ignored_targets_ 的区别:
        //     - ignored: GS 主动 ignore,本机不再自动锁
        //     - reported:本机识别上报 GS,GS 端不重复
        //   两个列表可并行维护,语义独立。当前阶段先实现 ignored,dedup 留作下个 PR。

        // 2) SEARCH_TRACK: 顺便发 TrackingState 通知 GS "正在跟踪"(信息性,
        //    不含锁定语义。锁定决策由 checkYoloDrivenStrike 在 IN_TASK 时做)
        if (current_work_mode_ == WorkMode::SEARCH_TRACK) {
            multi_uav_strike::TrackingState ts;
            ts.state   = 1;  // 1 = 跟踪中
            ts.flag    = 0;
            ts.target_dist = 50.0;  // 简化占位
            ts.target_lat  = dt.obj_lat;
            ts.target_lon  = dt.obj_lon;
            ts.target_alt  = dt.obj_alt;
            tracking_state_pub_.publish(ts);
        }

        // 之后不再做任何状态机/控制动作
    }

    /**
     * 从 YoloDetection 构造 DetectTarget(DRY:在 typedYoloCallback / checkYoloDrivenStrike
     * / attackCmdCallback 中复用)
     *
     * Phase 5 后续:用 gimbal_los + GPS 推算 obj_lat/lon/alt(目前简化用 UAV 位置占位)
     */
    multi_uav_strike::DetectTarget buildDetectTarget(
        const multi_uav_strike::YoloDetection& yolo,
        uint8_t target_type = 3) {
        multi_uav_strike::DetectTarget dt;
        dt.timestamp_us = yolo.stamp_us;
        dt.label        = 1;  // 占位
        dt.confidence   = yolo.confidence;
        dt.target_type  = target_type;
        dt.dev_lat      = current_pose_.pose.position.x;
        dt.dev_lon      = current_pose_.pose.position.y;
        dt.dev_alt      = -current_pose_.pose.position.z;
        // Phase 5 后续:用 gimbal LOS + 相机参数 + UAV GPS 推算目标真实位置
        dt.obj_lat      = dt.dev_lat;
        dt.obj_lon      = dt.dev_lon;
        dt.obj_alt      = 0.0;
        dt.img_format   = 1;
        return dt;
    }

    /**
     * YOLO 驱动的 strike/lock 决策 — 仅在 advanceSkillStateMachine IN_TASK 分支调用
     *
     * 门控条件(全部满足才执行):
     *   1. skill_type == 105 (Attack skill)   ← Search skill (102) 不会触发
     *   2. skill state == IN_TASK              ← 没到执行区域第一个点不会触发
     *   3. !is_guidance_active_               ← 没在制导中(避免重复触发)
     *   4. tracked_target_.lock_state == NOT_LOCKED  ← 没锁过目标
     *   5. latest_yolo_ 有效:has_latest_yolo_ + is_in_fov + age < 1.0s
     *   6. work_mode 必须是 SEARCH_STRIKE 或 SEARCH_TRACK(其他模式无意义)
     *   7. 新目标不在 ignored_targets_ 黑名单内(同 label + 距离 ≤ 20m + 未过期)
     *
     * 行为:
     *   - SEARCH_STRIKE: 构造 DetectTarget(SEARCH_ONLY 模式可以也构造),
     *                    设置 tracked_target_(LOCKED_AUTO) → triggerStrike()
     *   - SEARCH_TRACK:  构造 DetectTarget,设置 tracked_target_(LOCKED_WAIT_CONFIRM)
     *                    等 attack_cmd;由 attackCmdCallback 收到 action=0 后再 trigger
     *
     * 设计意图:把"识别→决策→动作"完整链路从 YOLO 回调挪到 skill 状态机,
     *          YOLO 回调只负责数据缓存和 GS 上报,职责单一。
     */
    void checkYoloDrivenStrike(const SkillRuntime& sr) {
        // 门控 6:work_mode 必须是 STRIKE/TRACK
        if (current_work_mode_ != WorkMode::SEARCH_STRIKE &&
            current_work_mode_ != WorkMode::SEARCH_TRACK) return;
        // 门控 1:搜索阶段可以进track，strike skill则可直接切入
        if (sr.msg.skill_type != 105 && sr.msg.skill_type != 102) return;
        // 门控 2:必须已到 IN_TASK(执行区域第一个点之后)
        if (sr.state != SkillState::IN_TASK) return;
        // 门控 3:已在制导中,跳过(避免重复 trigger)
        if (is_guidance_active_) return;
        // 门控 4:已锁目标,跳过(攻击/等待 attack_cmd 走另一条路径)
        if (tracked_target_.lock_state != TrackLockState::NOT_LOCKED) return;
        // 门控 5:YOLO 数据有效
        if (!has_latest_yolo_ || !latest_yolo_) return;
        if (!latest_yolo_->is_in_fov) return;
        double yolo_age = (ros::Time::now() - latest_yolo_time_).toSec();
        if (yolo_age > 1.0) return;  // 超过 1s 视为过期,等下一次 YOLO 命中
        // 门控 7:目标在 ignored_targets_ 黑名单内(GS 用 attack_cmd action=1/2 加的) → 跳过
        //   label-only 匹配:20s 内同 label 一律忽略
        //   空间维度留给未来云台+UAV pose 反推真实目标位置(C 方案)落地后再补
        if (isTargetIgnored(latest_yolo_->label)) {
            ROS_WARN_THROTTLE(2.0, "[MissionManager] >>>> YOLO+IN_TASK: target IGNORED "
                                   "(label=%s, in blacklist)",
                              latest_yolo_->label.c_str());
            return;
        }


        // === 触发 ===
        if (current_work_mode_ == WorkMode::SEARCH_STRIKE) {
            // 自动 strike:构造 DetectTarget(攻击目标),锁目标,trigger
            multi_uav_strike::DetectTarget dt = buildDetectTarget(*latest_yolo_, /*target_type=*/1);
            auto dt_ptr = boost::make_shared<multi_uav_strike::DetectTarget>(dt);
            tracked_target_.is_valid   = true;
            tracked_target_.latest     = dt_ptr;
            tracked_target_.yolo_label = latest_yolo_->label;  // 保留原始 label,用于 attack_cmd 加黑名单
            tracked_target_.lock_state = TrackLockState::LOCKED_AUTO;
            tracked_target_.locked_at  = ros::Time::now();

            ROS_WARN("[MissionManager] >>>> YOLO+Attack+IN_TASK [STRIKE]: trigger strike "
                     "(yolo_age=%.2fs conf=%.2f label=%s)",
                     yolo_age, latest_yolo_->confidence, latest_yolo_->label.c_str());
            guidance_speed_ = sr.msg.task_speed;
            triggerStrike();
        } else {
            // SEARCH_TRACK:锁目标等 attack_cmd
            multi_uav_strike::DetectTarget dt = buildDetectTarget(*latest_yolo_, /*target_type=*/3);
            auto dt_ptr = boost::make_shared<multi_uav_strike::DetectTarget>(dt);
            tracked_target_.is_valid   = true;
            tracked_target_.latest     = dt_ptr;
            tracked_target_.yolo_label = latest_yolo_->label;  // 保留原始 label
            tracked_target_.lock_state = TrackLockState::LOCKED_WAIT_CONFIRM;
            tracked_target_.locked_at  = ros::Time::now();
            guidance_speed_ = sr.msg.task_speed;
            startGuidanceApproach();
            ROS_WARN("[MissionManager] >>>> YOLO+Attack+IN_TASK [TRACK]: target locked, "
                     "waiting for attack_cmd (yolo_age=%.2fs conf=%.2f label=%s)",
                     yolo_age, latest_yolo_->confidence, latest_yolo_->label.c_str());
        }
    }

    /**
     * 判断给定 label 是否在 ignored_targets_ 黑名单内
     * 匹配规则(label-only, 2026-07 简化):
     *   1. label 必须完全相等(string equality)
     *   2. ignore_until > now
     * 注:不再做位置匹配。DetectTarget.obj_lat/lon 实际是 UAV pose 占位,
     *   扫描时 UAV 一边移动,位置匹配不稳;真实目标位置等云台+pose 反推(C 方案)
     *   落地后再补空间维度。
     * 副作用:每次调用顺手清理过期条目(throttle 1Hz,避免每帧都遍历)
     */
    bool isTargetIgnored(const std::string& label) {
        ros::Time now = ros::Time::now();
        // 节流清理过期项:1Hz 一次足够(ignore_until 精度秒级)
        if ((now - last_ignored_cleanup_).toSec() > 1.0) {
            auto it = ignored_targets_.begin();
            while (it != ignored_targets_.end()) {
                if (now >= it->ignore_until) {
                    ROS_INFO("[MissionManager] Ignored blacklist expired: label=%s "
                             "(alive %.1fs)",
                             it->label.c_str(),
                             (now - it->ignore_set_time).toSec());
                    it = ignored_targets_.erase(it);
                } else {
                    ++it;
                }
            }
            last_ignored_cleanup_ = now;
        }
        for (const auto& it : ignored_targets_) {
            if (it.label != label) continue;
            if (now >= it.ignore_until) continue;
            return true;  // label 相等 + 未过期 → 忽略
        }
        return false;
    }

    /**
     * 推一条目标到 ignored_targets_ 黑名单
     * 由 attackCmdCallback 在 action=1/2 时调用
     * @param dt 锁定的 DetectTarget (位置字段保留备用,目前不参与匹配)
     * @param yolo_label 锁定瞬间 YOLO 原始 string label
     */
    void addIgnoredTarget(const multi_uav_strike::DetectTarget& dt,
                          const std::string& yolo_label) {
        IgnoredTarget entry;
        entry.ned_x           = dt.obj_lat;   // 保留备用(C 方案启用时直接复用)
        entry.ned_y           = dt.obj_lon;
        entry.ned_alt         = dt.obj_alt;
        entry.label           = yolo_label;
        entry.ignore_set_time = ros::Time::now();
        entry.ignore_until    = entry.ignore_set_time + ros::Duration(ignored_retention_sec_);
        ignored_targets_.push_back(entry);
        ROS_WARN("[MissionManager] Added to IGNORED blacklist (label=%s "
                 "ttl=%.1fs, current_size=%zu)",
                 entry.label.c_str(),
                 ignored_retention_sec_, ignored_targets_.size());
    }

    /**
     * WaypointStatus typed 回调(Phase 4 启用)
     * 仅做缓存,advanceSkillStateMachine() 用 latest_wp_status_ 推进 SkillState
     */
    void waypointStatusCallback(const multi_uav_strike::WaypointStatus::ConstPtr& msg) {
        latest_wp_status_ = msg;
        latest_wp_status_time_ = ros::Time::now();
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
            // === 返航落地监控 (必须在 isTakeoffComplete 早 return 之后调用,避免落地检查被卡) ===
            // 正常情况下 AUTO.LAND 流程中 takeoff_state_ 仍是 COMPLETE,这里能跑到
            checkLandingComplete();
        }

        // 检查急停条件（毫米波雷达）
        checkEmergencyStop();

        // 根据工作模式执行对应行为
        // 注意: takeoff 流程由 runPx4TakeoffSequence() 推进,mission_loop 不再处理
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
            case WorkMode::DENIED_ENV_FLIGHT:
                handleDeniedEnvFlight();
                break;
        }

        // 发布状态
        publishStatus();
    }

    // PX4 SITL: 执行起飞状态机
    void runPx4TakeoffSequence() {
        // 关键门控：只有 PHASE_TAKING_OFF 或 PHASE_HOVERING 才推进起飞流程（避免 GROUND_IDLE 时 FCU 一连上就自动起飞）。
        // 注:HOLDING 也豁免 — enterHoldState() 需要 setpoint publisher 持续发 OFFBOARD setpoint 来稳悬停
        //    (taskFlowCallback 会在新 flow 到达时显式 stop, 这里早 return 不会漏停)
        if (current_phase_ != MissionPhase::PHASE_TAKING_OFF &&
            current_phase_ != MissionPhase::PHASE_HOVERING &&
            current_phase_ != MissionPhase::PHASE_HOLDING) {
            // 切出 TAKING_OFF 时：停止 setpoint 发布器（避免持续发送位置指令覆盖其他模块）
            if (setpoint_running_) {
                stopSetpointPublisher();
            }
            // 注意:不再重置 takeoff_state_。phase=WAYPOINT_FOLLOW/HOLDING 时 takeoff_state_=COMPLETE
            // 是 missionTimer 主循环能继续跑航点跟踪/悬停的前提, 不能清。
            return;
        }

        switch (takeoff_state_) {
            case TakeoffState::TAKEOFF_IDLE:
                // 已在 TAKEOFF 模式：等待 FCU 连接后开始
                if (!use_sim_ && is_px4_connected_) {
                    // 弹射起飞(skill_type=100)与地面起飞(skill_type=106)分流:
                    //   弹射:不能直接切 OFFBOARD,先让 PX4 进抛飞模式等外部 trigger
                    //   地面:走原流程,FCU 连上就 SET_MODE → COMMAND_LONG → ARM
                    if (is_catapult_takeoff_) {
                        takeoff_state_ = TakeoffState::TAKEOFF_CATAPULT_ARMED;
                        ROS_WARN("[MissionManager] PX4 Connected, CATAPULT takeoff flow — "
                                 "waiting for catapult_mode_ready=%s + catapult_trigger",
                                 is_px4_catapult_mode_ ? "true" : "false (need comm_node)");
                    } else {
                        takeoff_state_ = TakeoffState::TAKEOFF_WAITING_FCU;
                        ROS_WARN("[MissionManager] PX4 Connected, waiting for initialization...");
                    }
                }
                break;

            case TakeoffState::TAKEOFF_CATAPULT_ARMED:
                // 弹射起飞第一步:等 PX4 抛飞模式就绪 (comm_node 通过 px4/catapult_mode_ready 通知)
                //                  + 外部 trigger (mission/catapult_trigger)
                // 必须两者都到,因为:
                //   - 只到 PX4 模式没 trigger:不会发射, UAV 静等 → 浪费
                //   - 只到 trigger 没 PX4 模式:外部发弹射但 PX4 没准备好,危险
                if (!is_px4_catapult_mode_) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] CATAPULT_ARMED: waiting for PX4 catapult "
                                          "mode (comm_node sets it via px4/catapult_mode_ready)");
                    break;
                }
                if (!catapult_trigger_received_) {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] CATAPULT_ARMED: PX4 catapult mode ready, "
                                          "waiting for catapult_trigger from external node");
                    break;
                }
                // 两个都齐了 — 记录起始时间,准备监视 POSCTL 进入
                catapult_posctl_enter_time_ = ros::Time();
                takeoff_state_ = TakeoffState::TAKEOFF_CATAPULT_TRIGGERED;
                ROS_WARN("[MissionManager] >>>> CATAPULT armed+triggered, watching PX4 enter POSCTL "
                         "(commander will switch to POSCTL after launch detects sustained climb)...");
                break;

            case TakeoffState::TAKEOFF_CATAPULT_TRIGGERED: {
                // 弹射起飞第二步:等 PX4 自动切到 POSCTL(发射后 PX4 自主控制并进入位置模式)
                // 通过 mavros/state 反馈,current_mavros_state_.mode == "POSCTL" 视为进入
                // 兜底超时:60s 内没进 POSCTL 视为发射异常,转 FAILED
                if (current_mavros_state_.mode == "POSCTL") {
                    catapult_posctl_enter_time_ = ros::Time::now();
                    takeoff_state_ = TakeoffState::TAKEOFF_CATAPULT_POSCTL;
                    ROS_WARN("[MissionManager] >>>> CATAPULT: PX4 entered POSCTL (mode=%s, armed=%d)",
                             current_mavros_state_.mode.c_str(),
                             current_mavros_state_.armed ? 1 : 0);
                } else {
                    double since_trigger = catapult_posctl_enter_time_.isZero()
                        ? (ros::Time::now() - (ros::Time::now() - ros::Duration(catapult_posctl_stable_sec_))).toSec()
                        : 0.0;
                    // 注意:catapult_posctl_enter_time_ 此时 isZero (TRIGGERED 进入前清零),
                    // 用 takeoff_start_time_ (Phase=TAKING_OFF 时刻) 估算经过时间更稳
                    static ros::Time catapult_trigger_armed_time;  // 静态变量,记录 armed→triggered 的过渡
                    if (catapult_posctl_enter_time_.isZero() && catapult_trigger_armed_time.isZero()) {
                        catapult_trigger_armed_time = ros::Time::now();
                    }
                    double elapsed_in_triggered = (ros::Time::now() - catapult_trigger_armed_time).toSec();
                    if (elapsed_in_triggered > 60.0) {
                        ROS_ERROR("[MissionManager] CATAPULT: PX4 did not enter POSCTL within 60s, "
                                  "FAILED (current mode=%s)",
                                  current_mavros_state_.mode.c_str());
                        catapult_trigger_armed_time = ros::Time();  // 重置
                        takeoff_state_ = TakeoffState::TAKEOFF_FAILED;
                        current_phase_ = MissionPhase::PHASE_FAILED;
                    } else {
                        ROS_WARN_THROTTLE(2.0, "[MissionManager] CATAPULT: waiting for PX4 POSCTL "
                                              "(current mode=%s, %.1fs/60s)",
                                          current_mavros_state_.mode.c_str(), elapsed_in_triggered);
                    }
                    (void)since_trigger;  // 抑制未使用变量警告
                }
                break;
            }

            case TakeoffState::TAKEOFF_CATAPULT_POSCTL: {
                // 弹射起飞第三步:PX4 POSCTL 稳定 catapult_posctl_stable_sec_(1s) 后
                //                启动 setpoint publisher + 切 OFFBOARD 接管
                // 此处复用 TAKEOFF_WAITING_FCU 的 LOCAL_POSITION_NED rate 设置逻辑
                if (is_local_position_rate_set_ == false) {
                    if (set_message_rate_start_time_.isZero()) {
                        set_message_rate_start_time_ = ros::Time::now();
                    }
                    if (setMessageRate(32, local_position_rate_hz_)) {
                        is_local_position_rate_set_ = true;
                        set_message_rate_start_time_ = ros::Time();
                        set_message_rate_retry_count_ = 0;
                    } else {
                        set_message_rate_retry_count_++;
                        double elapsed = (ros::Time::now() - set_message_rate_start_time_).toSec();
                        bool retries_done = (set_message_rate_retry_count_ >= max_set_message_rate_retries_);
                        bool timeout = (elapsed > set_message_rate_max_wait_sec_);
                        if (retries_done || timeout) {
                            ROS_ERROR("[MissionManager] CATAPULT: LOCAL_POSITION_NED rate set GIVE UP — "
                                      "proceeding anyway");
                            is_local_position_rate_set_ = true;
                            set_message_rate_start_time_ = ros::Time();
                            set_message_rate_retry_count_ = 0;
                        } else {
                            break;  // 下个 tick 再试
                        }
                    }
                }
                // POSCTL 稳定等待(系统不抖后再接管)
                double posctl_elapsed = (ros::Time::now() - catapult_posctl_enter_time_).toSec();
                if (posctl_elapsed < catapult_posctl_stable_sec_) {
                    ROS_WARN_THROTTLE(0.3, "[MissionManager] CATAPULT: POSCTL stable wait %.2fs/%.2fs",
                                      posctl_elapsed, catapult_posctl_stable_sec_);
                    break;
                }
                // 稳定时间到 — 进 SETTING_OFFBOARD 接管
                ROS_WARN("[MissionManager] >>>> CATAPULT: POSCTL stable, switching to OFFBOARD takeover");
                takeoff_state_ = TakeoffState::TAKEOFF_SETTING_OFFBOARD;
                startSetpointPublisher();
                break;
            }

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
                        ROS_WARN_THROTTLE(2.0, "[MissionManager] >>>>> LOCAL_POSITION_NED rate set FAILED "
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
                if (triggerOffboardAndArm()) {
                    takeoff_state_ = TakeoffState::TAKEOFF_TAKEOFF_EXEC;
                    takeoff_start_time_ = ros::Time::now();
                    takeoff_setpoint_.pose.position.z = takeoff_altitude_;
                    ROS_WARN("[MissionManager] >>>> OFFBOARD + ARM SUCCESS, takeoff climb started "
                             "(sp=%.0fms, target alt=%.1f m)",
                             sp_elapsed * 1000.0, takeoff_altitude_);
                } else {
                    // 不进 FAILED 状态，下一 tick 会再走一遍 SETTING_OFFBOARD 重试整组动作
                    ROS_WARN("[MissionManager] >>>> OFFBOARD + ARM sequence FAILED (PX4 mode=%s armed=%d), "
                             "will retry next cycle",
                             current_mavros_state_.mode.c_str(),
                             current_mavros_state_.armed ? 1 : 0);
                }
                break;
            }

            case TakeoffState::TAKEOFF_TAKEOFF_EXEC: {
                // 检查高度 (NED: z 向下为正)
                double current_alt = -current_pose_.pose.position.z;
                if (current_alt >= takeoff_altitude_ - 1.0f) {  // 高度容差 1m
                    double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                    if (elapsed > takeoff_stable_time_) {  // 高度稳定时间（ROS 参数）
                        takeoff_state_ = TakeoffState::TAKEOFF_HOVERING;
                        takeoff_start_time_ = ros::Time::now();
                        current_phase_ = MissionPhase::PHASE_HOVERING;  // phase 推进:TAKING_OFF → HOVERING
                        ROS_WARN("[MissionManager] Takeoff altitude reached, hovering (phase=HOVERING)...");
                    }
                } else {
                    ROS_WARN_THROTTLE(2.0, "[MissionManager] Takeoff climbing: %.1f / %.1f m",
                                     current_alt, takeoff_altitude_);
                }

                // 超时检测
                double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                if (elapsed > 60.0) {
                    ROS_ERROR("[MissionManager] Takeoff timeout!");
                    takeoff_state_ = TakeoffState::TAKEOFF_FAILED;
                    current_phase_ = MissionPhase::PHASE_FAILED;  // phase 推进:→ FAILED
                }
                break;
            }

            case TakeoffState::TAKEOFF_HOVERING: {
                // 悬停一段时间后进入任务
                double elapsed = (ros::Time::now() - takeoff_start_time_).toSec();
                if (elapsed > takeoff_hover_time_) {  // 悬停等待时间（ROS 参数）
                    // === Auto-handoff 判定 ===
                    // 真实 GS 场景下: 任务 = TAKEOFF + 航点列表, 期间 phase 推进 TAKING_OFF→HOVERING→WAYPOINT_FOLLOW
                    // 不能等用户手动切 work_mode(违反 SET_WORKMODE 协议语义),必须在这里自动交接给 waypoint_executor。
                    if (is_waypoints_received_) {
                        performTakeoffHandoff();
                        // performTakeoffHandoff 会:
                        //   1. 恢复 work_mode_before_takeoff_ (或回落到 ~post_takeoff_work_mode)
                        //   2. 立刻停 setpoint_thread (gap ~20ms < 500ms PX4 timeout)
                        //   3. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE
                        //   4. current_phase_=PHASE_WAYPOINT_FOLLOW
                        // 下次 tick 时 runPx4TakeoffSequence 因为 phase!=TAKING_OFF 而 early-return,
                        // 不再进入 HOVERING 分支, 等价于原来的 is_takeoff_handoff_done_ 防重入效果。
                    } else {
                        // 还没收到航点, 保持 HOVERING, 让 setpoint_thread 持续发悬停点
                        // 保险: 避免在没航点的情况下交接, 导致 waypoint_executor 看到空队列报错
                        ROS_WARN_THROTTLE(2.0, "[MissionManager] HOVERING waiting for waypoints "
                                              "(is_waypoints_received_=%d, waypoint_count=%zu)",
                                              is_waypoints_received_ ? 1 : 0, current_waypoint_count_);
                        // 重置 elapsed, 再多等一拍, 直到航点到达
                        takeoff_start_time_ = ros::Time::now();
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
    // 触发条件: 高度达标 + hover 计时到 + 航点已下发 + phase 还在 TAKING_OFF/HOVERING
    // 流程:
    //   1. 恢复用户起飞前的工作模式(work_mode_before_takeoff_),
    //      若起飞前是 IDLE 则落到 ~post_takeoff_work_mode(默认 SEARCH_ONLY)
    //   2. waypoint_executor 已通过 typed Skill 收到新路径,无需再发 String mode
    //   3. 立即停 setpoint_thread (gap ~20ms, 远小于 PX4 COM_OFFBOARD_LOSS_TIMEOUT=500ms)
    //   4. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE (让 missionTimerCallback 继续跑)
    //   5. current_phase_=PHASE_WAYPOINT_FOLLOW (下次 tick runPx4TakeoffSequence early-return)
    void performTakeoffHandoff() {
        // 1. 恢复用户起飞前的工作模式,而不是硬切 SEARCH_ONLY
        //    场景:用户在地面站提前发过 SET_WORKMODE=SEARCH_TRACK,起飞 skill 不应覆盖用户意图
        //    兼容:如果起飞前是 IDLE(典型场景),回落到 ~post_takeoff_work_mode(默认 SEARCH_ONLY)
        WorkMode restore_mode = work_mode_before_takeoff_;
        if (restore_mode == WorkMode::IDLE) {
            restore_mode = post_takeoff_work_mode_;
        }
        bool mode_restored = (restore_mode != current_work_mode_);
        if (mode_restored) {
            current_work_mode_ = restore_mode;
        }
        work_mode_before_takeoff_ = WorkMode::IDLE;  // 一次性缓存,清空防误用
        ROS_WARN("[MissionManager] ===== AUTO-HANDOFF: TAKEOFF -> WAYPOINT_FOLLOW "
                 "(waypoints=%zu, work_mode=%s%s) =====",
                 current_waypoint_count_,
                 workModeToString(current_work_mode_).c_str(),
                 mode_restored ? " [restored]" : "");
        // phase_before_takeoff_ 不在此处清,因为它将在 GROUND_IDLE → TAKING_OFF → HOVERING → WAYPOINT_FOLLOW 链路结束后
        // 由新 task_flow 的 entry 路径再次覆盖。这里清掉反而可能在重新进入 PHASE_TAKING_OFF 的瞬间丢失兜底。
        disableGuidance();  // 起飞后默认关闭制导,等 SEARCH_TRACK/STRIKE 再 enable

        // 3. waypoint_executor 已通过 typed Skill(由 pushCurrentSkillToExecutor
        //    把下一条 Skill 推到 /waypoint_executor/skill) 收到新路径,无需再发
        //    旧的 String mode 通知。

        // 4. 立刻停 setpoint_thread (此时 waypoint_executor 即将接管)
        if (setpoint_running_) {
            stopSetpointPublisher();
            ROS_WARN("[MissionManager]   - Takeoff setpoint publisher stopped");
        }
        
        // 5. 推进 takeoff_state_ 到 TAKEOFF_COMPLETE
        //    让 isTakeoffComplete()=true, missionTimerCallback 继续跑 mission 主循环
        takeoff_state_ = TakeoffState::TAKEOFF_COMPLETE;

        // 6. 显式把起飞 skill 标记为 COMPLETE,而不是等 advanceSkillStateMachine
        //    自己走完 TRANSIT→EXIT_PENDING→COMPLETE(那要 100ms 才推进,中间还有 50ms 计时)
        //    起飞 skill 没有 arrive_path/skill_area_path,EXIT_PENDING 阶段无意义,直接 COMPLETE 更清晰
        //    下次 skillAdvanceTimerCallback tick 就会走 COMPLETE 分支推下一个 skill
        if (current_skill_index_ < skill_queue_.size() &&
            (skill_queue_[current_skill_index_].msg.skill_type == 100 ||
             skill_queue_[current_skill_index_].msg.skill_type == 106)) {
            auto& sr = skill_queue_[current_skill_index_];
            const std::string prev_state_str = skillStateStr(sr.state);
            sr.state = SkillState::COMPLETE;
            sr.state_enter_time = ros::Time::now();
            sr.last_event = "handoff forced " + prev_state_str + " to COMPLETE";
            ROS_WARN("[MissionManager]   - Takeoff skill[%zu] id=%s  COMPLETE (forced by handoff, prev=%s)",
                     current_skill_index_, sr.msg.skill_id.c_str(), prev_state_str.c_str());
        } else {
            ROS_WARN_THROTTLE(5.0, "[MissionManager]   - performTakeoffHandoff: current_skill_[%zu] is not takeoff "
                                  "(skill_type=%u), skip force-COMPLETE",
                              current_skill_index_,
                              (current_skill_index_ < skill_queue_.size())
                                  ? skill_queue_[current_skill_index_].msg.skill_type : 0);
        }

        // 7. 推进 MissionPhase
        current_phase_ = MissionPhase::PHASE_WAYPOINT_FOLLOW;

        ROS_WARN("[MissionManager] ===== AUTO-HANDOFF COMPLETE, UAV now waypoint-following =====");
    }

    // === HOLDING: 所有 skill 完成后原地悬停等新 task_flow ===
    // 复用 startSetpointPublisher() 的 position-hold 机制(刚修好的航向 + 50Hz + OFFBOARD 持续 setpoint)
    // 与 takeoff 的区别仅在于 z 目标:起飞是 takeoff_altitude_,holding 是当前高度
    // PX4 OFFBOARD 持续 setpoint 才能稳悬停,这是最简单可控的 hold 方案
    void enterHoldState() {
        if (is_pose_received_) {
        ROS_WARN_THROTTLE(5.0, "[MissionManager] HOLDING current_pose (NED): x=%.2f y=%.2f z=%.2f",
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z);
        }

        // 幂等:advanceSkillStateMachine() 在 10Hz 重复 tick,只要 last_skill 仍是 COMPLETE 就会再次进入这里。
        // 已经在 HOLDING 时直接 return,避免反复 start/stop setpoint publisher。
        if (current_phase_ == MissionPhase::PHASE_HOLDING) {
            return;
        }
        current_phase_ = MissionPhase::PHASE_HOLDING;
        // has_task_flow_ 保持 true(队列不清,用于 MissionState 上报 last_skill_type 供 GS 观测)
        // skill_queue_ 也不清
        // current_skill_index_ 保持指向最后一个完成的 skill

        if (!is_pose_received_) {
            ROS_WARN("[MissionManager] HOLDING: no pose yet (will wait inside startSetpointPublisher up to 10s)");
        }

        if (!setpoint_running_) {
            // 启动 setpoint publisher (它会 wait 最多 10s 等首帧 pose,然后 latch xy + 计算航向)
            startSetpointPublisher();
            // 覆盖 z 目标为当前高度(NED z 向下为正 → ENU z up 正数)
            if (is_pose_received_) {
                double current_alt_enu = -current_pose_.pose.position.z;
                takeoff_setpoint_.pose.position.z = current_alt_enu;
                ROS_WARN("[MissionManager] HOLDING setpoint z=%.2fm (current altitude, xy+heading latched from pose)",
                         current_alt_enu);
            }
        } else {
            // 已经运行中(理论上 performTakeoffHandoff 已经停了 setpoint,这里只是兜底)
            // 直接更新 z 为当前高度
            if (is_pose_received_) {
                takeoff_setpoint_.pose.position.z = -current_pose_.pose.position.z;
            }
        }

        ROS_WARN("[MissionManager] ===== HOLDING: pose+heading locked, awaiting next task_flow =====");
    }

    // === RETURN + PX4 AUTO.LAND 落地 ===
    // 由 advanceSkillStateMachine COMPLETE 分支(最后一个 skill = 103)调用
    // 流程:
    //   1. 停 waypoint_executor (避免与 PX4 内部控制冲突)
    //   2. 停 setpoint publisher (AUTO.LAND 接管,不需要 OFFBOARD setpoint)
    //   3. 禁用制导 (如有)
    //   4. setMode("AUTO.LAND") — PX4 自带降落 + 着陆后自动 disarm
    //   5. 标志 is_landing_in_progress_=true,phase=PHASE_RETURNING
    //   6. 后续由 checkLandingComplete() 在 missionTimer 中监控 current_mavros_state_.armed
    //      → false 判定落地完成
    // 注意:
    //   - PX4 AUTO.LAND 需要 UAV 已解锁(armed=true),否则会拒绝
    //   - PX4 着陆后自动 disarm(配置 COM_DISARM_LAND),无需我们再发 disarm 命令
    //   - 兜底:落地安全超时 landing_safety_timeout_sec_(默认 120s) 后若仍未 disarm,
    //     强制调 armVehicle(false) — 防止 PX4 因地形/传感器异常卡在 AUTO.LAND
    //   - 任务流中段出现 Return 也走同一入口(advanceSkillStateMachine 不区分位置)
    //   - 中途有用户重新下发 task_flow 时,taskFlowCallback 顶部会清 is_landing_in_progress_
    void triggerPx4Landing() {
        if (is_landing_in_progress_) {
            return;  // 幂等:已在降落流程,避免被 10Hz 状态机重复触发
        }
        is_landing_in_progress_   = true;
        landing_start_time_       = ros::Time::now();
        landing_complete_logged_  = false;
        current_phase_            = MissionPhase::PHASE_RETURNING;

        ROS_WARN("[MissionManager] ===== Return-skill  PX4 AUTO.LAND triggered =====");

        // 1. 停 waypoint_executor (AUTO.LAND 接管水平位置 + 下降率,executor 不应再发速度)
        {
            std_msgs::String cmd;
            cmd.data = "stop";
            waypoint_control_pub_.publish(cmd);
        }

        // 2. 停 setpoint publisher (PX4 AUTO.LAND 不接受 OFFBOARD setpoint,留着会冲突)
        if (setpoint_running_) {
            stopSetpointPublisher();
        }

        // 3. 禁用制导 (如有遗留)
        if (is_guidance_active_) {
            disableGuidance();
        }

        // 4. 切 PX4 到 AUTO.LAND (set_mode_client 仅在 !use_sim_ 初始化)
        if (!use_sim_ && set_mode_client_.exists()) {
            mavros_msgs::SetMode sm;
            sm.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(sm) && sm.response.mode_sent) {
                ROS_WARN("[MissionManager] AUTO.LAND: PX4 mode accepted, waiting for touchdown");
            } else {
                ROS_ERROR("[MissionManager] AUTO.LAND set FAILED (mode_sent=%d, current_mode=%s)",
                          sm.response.mode_sent ? 1 : 0,
                          current_mavros_state_.mode.c_str());
            }
        } else {
            ROS_WARN("[MissionManager] AUTO.LAND: use_sim_=%d, skipping actual mode call (sim mode); "
                     "waiting for disarm", use_sim_ ? 1 : 0);
        }
    }

    // 由 missionTimerCallback 每 tick 调用 — 监控 PX4 落地完成
    // 完成判定:current_mavros_state_.armed 从 true → false (PX4 AUTO.LAND 着陆后自动 disarm)
    // 兜底:启动后 landing_safety_timeout_sec_(默认 120s) 仍未 disarm → 强制 disarm
    void checkLandingComplete() {
        if (!is_landing_in_progress_) {
            return;
        }

        if (!current_mavros_state_.armed) {
            // 落地 + disarm 完成
            if (!landing_complete_logged_) {
                double elapsed = (ros::Time::now() - landing_start_time_).toSec();
                ROS_WARN("[MissionManager] ===== LANDING COMPLETE: PX4 disarmed "
                         "(elapsed=%.1fs, mode=%s) =====",
                         elapsed, current_mavros_state_.mode.c_str());
                ROS_WARN("[MissionManager] ===== UAV on ground. Awaiting next task_flow or shutdown. =====");
                landing_complete_logged_ = true;
                // 不再切回 PHASE_HOLDING (UAV 已在地面,setpoint publisher 不需要再起)
                // GS 通过 MissionState.phase=RETURNING + state.armed=false 即可判定任务结束
            }
            return;
        }

        // 还在 ARMED 状态 — 检查是否需要兜底
        double elapsed = (ros::Time::now() - landing_start_time_).toSec();
        if (elapsed > landing_safety_timeout_sec_) {
            // AUTO.LAND 卡住(可能因地形/传感器问题未触发 disarm),强制 disarm 兜底
            // 强制前:先尝试切 POSCTL (脱离 AUTO.LAND 状态机),再 disarm,某些固件要求模式非 LAND 才能 disarm
            if (!use_sim_ && set_mode_client_.exists()) {
                mavros_msgs::SetMode sm;
                sm.request.custom_mode = "POSCTL";
                if (set_mode_client_.call(sm)) {
                    ROS_WARN("[MissionManager]   - switched to POSCTL for force-disarm");
                }
                ros::Duration(0.2).sleep();
            }
            ROS_ERROR("[MissionManager] Landing timeout (%.1fs > %.1fs), forcing disarm",
                      elapsed, landing_safety_timeout_sec_);
            if (!use_sim_ && arming_client_.exists()) {
                armVehicle(false);  // 调 mavros/cmd/arming value=false
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "[MissionManager] AUTO.LAND in progress (elapsed=%.1fs, mode=%s, armed=%d)",
                              elapsed,
                              current_mavros_state_.mode.c_str(),
                              current_mavros_state_.armed ? 1 : 0);
        }
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

        // 计算起飞航向：用当前机头朝向发布 setpoint，避免 PX4 接管瞬间 yaw 回零而转圈
        // current_pose_ 是 NED 约定，mavros/setpoint_position/local 默认期望 ENU
        if (is_pose_received_) {
            const auto& q = current_pose_.pose.orientation;
            // NED 下的 yaw：0=北，顺时针为正
            double yaw_ned = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            cerr<<"[MissionManager] Takeoff setpoint heading: current NED yaw = "<< yaw_ned * 180.0 / M_PI << endl;
            // NED yaw → ENU yaw：ENU 0=东，逆时针为正
            // 关系：机头指 NED-北 等价于 ENU-北（ENU yaw=π/2）；机头指 NED-东 等价于 ENU-东（ENU yaw=0）
            double yaw_enu = M_PI_2 - yaw_ned;
            while (yaw_enu > M_PI)  yaw_enu -= 2.0 * M_PI;
            while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;
            // 绕 ENU Z 轴的旋转四元数 (0, 0, sin(yaw/2), cos(yaw/2))
            double half = 0.5 * yaw_enu;
            takeoff_setpoint_.pose.orientation.x = 0.0;
            takeoff_setpoint_.pose.orientation.y = 0.0;
            takeoff_setpoint_.pose.orientation.z = std::sin(half);
            takeoff_setpoint_.pose.orientation.w = std::cos(half);
            ROS_INFO("[MissionManager] Takeoff setpoint heading: NED yaw=%.2f -> ENU yaw=%.2f",
                     yaw_ned * 180.0 / M_PI, yaw_enu * 180.0 / M_PI);
        } else {
            // 没拿到姿态就保持默认 (yaw=0, ENU 朝东)，至少不会去强行改一个未知朝向
            takeoff_setpoint_.pose.orientation.x = 0.0;
            takeoff_setpoint_.pose.orientation.y = 0.0;
            takeoff_setpoint_.pose.orientation.z = 0.0;
            takeoff_setpoint_.pose.orientation.w = 1.0;
        }

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
                // 航向已在 startSetpointPublisher() 里按当前机头朝向算好，
                // 这里不要每帧覆盖成 w=1，否则 PX4 接管瞬间仍会 yaw 回零转圈
                takeoff_setpoint_pub_.publish(takeoff_setpoint_);
                setpoint_publish_count_++;
                //ros::spinOnce();
                rate.sleep();

                // 每 5s 打印一次发包状态 + PX4 当前 mode（多 UAV 对比观察用）
                if ((ros::Time::now() - last_log).toSec() > 5.0) {
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

        ROS_WARN("[MissionManager] >>>>> setMode(%s) called, PX4 current_mode=%s connected=%d armed=%d setpoint_pkts=%d",
                 mode.c_str(),
                 current_mavros_state_.mode.c_str(),
                 current_mavros_state_.connected ? 1 : 0,
                 current_mavros_state_.armed ? 1 : 0,
                 setpoint_publish_count_);

        // ===== Path 1: 标准 SET_MODE 服务（走 SET_MODE MAVLink 消息）=====
        bool set_mode_sent = false;
        if (set_mode_client_.call(set_mode) && set_mode.response.mode_sent) {
            set_mode_sent = true;
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
            // 每 2s 报一次当前 PX4 mode（10Hz tick 中每 20 帧一次，避免被噪声淹没）
            if (poll_count % 20 == 0) {
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
        } else {
            ROS_WARN("[MissionManager] >>>> SET_MODE service: RPC call FAILED");
        }

        // -------- Step 2: 50ms 后 COMMAND_LONG DO_SET_MODE --------
        // 命中 PX4 commander handle_command 路径 — 某些固件版本对这条路径更稳定
        ros::Duration(0.05).sleep();
        sendSetModeCommandLong("OFFBOARD");

        // -------- Step 3: 50ms 后 直接 ARM，不等 mode 确认 --------
        // 这是关键时机 — ARM 必须在 commander 还没 revert 之前到达
        ros::Duration(0.05).sleep();
        bool armed = armVehicle(true);

        // -------- Step 4: ARM 失败重试最多 3 次（每次 50ms） --------
        // PX4 commander 可能在 OFFBOARD transition 中，ARM 会被暂缓一会儿
        for (int i = 0; i < 3 && !armed && ros::ok(); ++i) {
            ros::Duration(0.05).sleep();
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
        // 避障只在"实际执行多机协同任务"阶段开,起飞/地面/悬停/返航期间不计算
        // (起飞阶段 UAV 垂直分离不会撞,地面静止,悬停等新指令也不需要主动避让)
        if (current_phase_ == MissionPhase::PHASE_WAYPOINT_FOLLOW ||
            current_phase_ == MissionPhase::PHASE_GUIDANCE_TRACK ||
            current_phase_ == MissionPhase::PHASE_GUIDANCE_STRIKE) {
            computeAndPublishAvoidanceVector();
        }
    }

    // ============== Phase 2: MissionState 1Hz 定频上报 ==============

    /**
     * 1Hz 周期发送 MissionState — 不论 Skill 状态如何都发(用户明确要求"定频")
     */
    void missionStateTimerCallback(const ros::TimerEvent&) {
        publishMissionState();
    }

    void publishMissionState() {
        multi_uav_strike::MissionState ms;
        ms.skill_flow_id = current_flow_id_;
        ms.skill_type    = -1;  // -1 表示无 active skill
        ms.skill_id      = "";
        ms.state         = 0;   // 默认未开始
        ms.phase         = static_cast<uint8_t>(current_phase_);  // 当前任务阶段(GS 用此判断 holding/waypoint/guidance)

        if (current_skill_index_ < skill_queue_.size()) {
            const auto& sr = skill_queue_[current_skill_index_];
            ms.skill_id   = sr.msg.skill_id;
            ms.skill_type = static_cast<int32_t>(sr.msg.skill_type);
            ms.state      = skillStateToInt(sr.state);
        }

        mission_state_pub_.publish(ms);
        last_mission_state_pub_time_ = ros::Time::now();
    }

    static int32_t skillStateToInt(SkillState s) {
        switch (s) {
            case SkillState::PENDING:        return 0;
            case SkillState::TRANSIT:        return 1;
            case SkillState::ENTRY_PENDING:  return 1;  // 执行中
            case SkillState::IN_TASK:        return 1;
            case SkillState::EXIT_PENDING:   return 1;
            case SkillState::COMPLETE:       return 3;
            case SkillState::FAILED:         return 4;
            default:                         return 0;
        }
    }

    static std::string skillStateStr(SkillState s) {
        switch (s) {
            case SkillState::PENDING:       return "PENDING";
            case SkillState::TRANSIT:       return "TRANSIT";
            case SkillState::ENTRY_PENDING: return "ENTRY_PENDING";
            case SkillState::IN_TASK:       return "IN_TASK";
            case SkillState::EXIT_PENDING:  return "EXIT_PENDING";
            case SkillState::COMPLETE:      return "COMPLETE";
            case SkillState::FAILED:        return "FAILED";
            default:                        return "UNKNOWN";
        }
    }

    // ============== Phase 3: Skill 状态机推进 ==============

    /**
     * 10Hz tick,推进当前 skill 的 SkillState 转换
     *
     * 状态转换(根据 waypoint_executor 的 WaypointStatus 反馈 + 距离判定):
     *   PENDING → TRANSIT:                executor 收到 Skill 后自动推
     *   TRANSIT → ENTRY_PENDING:           phase == ARRIVE 且 arrive_idx == arrive_total
     *   ENTRY_PENDING → IN_TASK:          phase == SKILL_AREA 且距 path[0] ≤ arrival_threshold
     *   IN_TASK → EXIT_PENDING:           phase == SKILL_AREA 且 skill_idx == skill_total
     *   EXIT_PENDING → COMPLETE:          等一帧发最终遥测后
     *
     * 失败:
     *   ENTRY_PENDING 超时 entry_gate_timeout_(30s) → FAILED
     *   IN_TASK 超时 in_task_timeout_(0=无限) → FAILED
     */
    void skillAdvanceTimerCallback(const ros::TimerEvent&) {
        if (skill_queue_.empty()) return;
        // 每 5s 心跳打印当前 skill 状态机状态（慢速参考；状态变化时已有单独日志）
        const SkillRuntime& sr = skill_queue_[current_skill_index_];
        ROS_INFO_THROTTLE(5.0, "[MissionManager] Skill[%zu] id=%s type=%u state=%s, phase=%s",
                          current_skill_index_, sr.msg.skill_id.c_str(),
                          static_cast<unsigned>(sr.msg.skill_type),
                          skillStateStr(sr.state).c_str(),
                          missionPhaseToString().c_str());
        if (current_skill_index_ >= skill_queue_.size()) return;
        advanceSkillStateMachine();
    }

    void advanceSkillStateMachine() {
        if (current_skill_index_ >= skill_queue_.size()) return;
        SkillRuntime& sr = skill_queue_[current_skill_index_];
        const ros::Time now = ros::Time::now();
        
        switch (sr.state) {
            case SkillState::PENDING:
                // 等 pushCurrentSkillToExecutor() 推过去
                pushCurrentSkillToExecutor();
                break;

            case SkillState::TRANSIT: {
                // 特殊门控:skill_type=100 (Takeoff) — 起飞没有 arrive_path/skill_area_path,
                //   用 isTakeoffComplete() 当门控;一旦起飞完成直接跳 EXIT_PENDING(中间状态无意义)
                if (sr.msg.skill_type == 100 || sr.msg.skill_type == 106) {
                    if (!isTakeoffComplete()) break;
                    sr.state = SkillState::EXIT_PENDING;
                    sr.state_enter_time = now;
                    sr.last_event = "TRANSIT to EXIT_PENDING (takeoff complete)";
                    ROS_WARN("[MissionManager] Skill[%zu] id=%s to EXIT_PENDING (takeoff)",
                             current_skill_index_, sr.msg.skill_id.c_str());
                    break;
                }
                // 普通门控:executor 报告 phase=ARRIVE 且 arrive_idx 走完 (arrive_idx >= arrive_total)
                // 修复:原来是 arrive_idx + 1 >= arrive_total,会在 idx = total - 1 (即飞机刚开始
                //   飞向最后一个 arrive 航点)时就触发,导致 COMPLETE 太早、enterHoldState 在中途捕获 pose,
                //   飞机继续飞到末点后又飞回捕获点。改为 idx >= total,等最后一个 arrive 航点 *到达* 才推进。
                if (!latest_wp_status_) break;
                if (latest_wp_status_->arrive_idx >= latest_wp_status_->arrive_total &&
                    latest_wp_status_->skill_id == sr.msg.skill_id) {
                    // === 修复:skill_area_path 为空时,executor 直接 ARRIVE→COMPLETE,
                    //   不会发 SKILL_AREA。如果照原逻辑进 ENTRY_PENDING 等 phase==SKILL_AREA,
                    //   永远等不到,30s 后 entry_gate_timeout 才 FAILED,期间 UAV 悬停不动。
                    //   语义上"已在入口"(没有 skill_area 需要进),直接进 IN_TASK。
                    if (sr.msg.skill_area_path.poses.empty()) {
                        sr.state = SkillState::IN_TASK;
                        sr.state_enter_time = now;
                        sr.in_task_enter_time = now;
                        if (sr.msg.skill_type == 101) {
                            // 101 集结合:记录进入时刻,等待其他 UAV 或 timeout
                            sr.gather_enter_time = now;
                        }
                        sr.last_event = "TRANSIT to IN_TASK (skill_area empty, skip ENTRY_PENDING)";
                        ROS_WARN("[MissionManager] Skill[%zu] id=%s to IN_TASK "
                                 "(type=%u, skill_area empty, skipping ENTRY_PENDING)",
                                 current_skill_index_, sr.msg.skill_id.c_str(),
                                 static_cast<unsigned>(sr.msg.skill_type));
                    } else {
                        sr.state = SkillState::ENTRY_PENDING;
                        sr.state_enter_time = now;
                        sr.entry_gate_enter_time = now;
                        sr.last_event = "TRANSIT to ENTRY_PENDING (arrive last point reached)";
                        ROS_WARN("[MissionManager] Skill[%zu] id=%s to ENTRY_PENDING",
                                 current_skill_index_, sr.msg.skill_id.c_str());
                    }
                }
                break;
            }

            case SkillState::ENTRY_PENDING: {
                // 门控:executor 报告 phase=SKILL_AREA(或 phase=COMPLETE — 同点 catch 不到 SKILL_AREA 的补救)
                //   - 正常情况:ARRIVE → SKILL_AREA → COMPLETE,mission_manager 看到 SKILL_AREA 时开门
                //   - 同点场景:ARRIVE/SKILL_AREA 同一坐标,executor 20ms 内 SKILL_AREA 跳过,
                //     mission_manager 的 10Hz tick 可能错过 SKILL_AREA → 改判 phase=COMPLETE 也开门
                //   - skill_area_path 空:line 2247 早就 TRANSIT → IN_TASK 跳过 ENTRY_PENDING,走不到这里
                bool in_skill_area = (latest_wp_status_ &&
                                      (latest_wp_status_->phase == multi_uav_strike::WaypointStatus::PHASE_SKILL_AREA ||
                                       latest_wp_status_->phase == multi_uav_strike::WaypointStatus::PHASE_COMPLETE) &&
                                      latest_wp_status_->skill_id == sr.msg.skill_id);
                if (in_skill_area) {
                    sr.state = SkillState::IN_TASK;
                    sr.state_enter_time = now;
                    sr.in_task_enter_time = now;
                    if (sr.msg.skill_type == 101) {
                        // 101 集结合:记录进入时刻,等待其他 UAV 或 timeout
                        sr.gather_enter_time = now;
                    }
                    // 修复:进 skill_area 时保持 PHASE_WAYPOINT_FOLLOW,不再提前切到 GUIDANCE_*
                    // 之前这里按 work_mode 提前切 phase,导致航点跟踪阶段 phase=GUIDANCE_*
                    // 但 guidance 还没 enable(YOLO 还没命中),状态错位;且 handleSearch*
                    // 一旦 is_locked 误置 true 就会立即调 startGuidanceApproach()。
                    // phase → GUIDANCE_* 的转换由 startGuidanceApproach() 在 YOLO 命中
                    // 实际启动 guidance 时切(见 startGuidanceApproach 头部)。
                    sr.last_event = "ENTRY_PENDING to IN_TASK";
                    ROS_WARN("[MissionManager] Skill[%zu] id=%s to IN_TASK (type=%u, phase=%s)",
                             current_skill_index_, sr.msg.skill_id.c_str(),
                             static_cast<unsigned>(sr.msg.skill_type),
                             missionPhaseToString().c_str());
                } else if (entry_gate_timeout_ > 0.0 &&
                           (now - sr.entry_gate_enter_time).toSec() > entry_gate_timeout_) {
                    sr.state = SkillState::FAILED;
                    sr.state_enter_time = now;
                    sr.last_event = "ENTRY_PENDING to FAILED (timeout)";
                    ROS_ERROR("[MissionManager] Skill[%zu] id=%s entry gate TIMEOUT (%.1fs)",
                              current_skill_index_, sr.msg.skill_id.c_str(),
                              (now - sr.entry_gate_enter_time).toSec());
                }
                break;
            }

            case SkillState::IN_TASK: {
                // === YOLO 驱动的 strike/lock 决策(只在 Attack skill + IN_TASK 触发)===
                // 见 checkYoloDrivenStrike() 的门控说明:
                //   1. skill_type == 105
                //   2. state == IN_TASK (本分支已满足)
                //   3. !is_guidance_active_
                //   4. tracked_target_.lock_state == NOT_LOCKED
                //   5. latest_yolo_ 有效 (1s 内 is_in_fov)
                //   6. work_mode == SEARCH_STRIKE / SEARCH_TRACK
                // 满足则触发:SEARCH_STRIKE → triggerStrike();SEARCH_TRACK → 仅锁目标等 attack_cmd
                checkYoloDrivenStrike(sr);

                // skill_type=101 集结合:等所有 expected_sns_ 到齐或 timeout
                if (sr.msg.skill_type == 101) {
                    bool all_arrived = checkAllUavsGathered();
                    bool timed_out   = gather_timeout_ > 0.0 &&
                                       (now - sr.gather_enter_time).toSec() > gather_timeout_;
                    if (all_arrived || timed_out) {
                        sr.state = SkillState::EXIT_PENDING;
                        sr.state_enter_time = now;
                        sr.last_event = timed_out ?
                            "IN_TASK to EXIT_PENDING (gather timeout)" :
                            "IN_TASK to EXIT_PENDING (gather complete)";
                        ROS_WARN("[MissionManager] Skill[101] Gather %s",
                                 timed_out ? "TIMEOUT" : "COMPLETE");
                    }
                    // 发本机 gather_status(其它 UAV 看)
                    publishGatherStatus();
                    break;
                }

                // 其它 skill_type:门控 = skill_idx 走完 (idx >= total),不限定 phase
                // 修复:原来是 skill_idx + 1 >= skill_total,会在 idx = total - 1 (即飞机刚开始
                //   飞向最后一个 skill_area 航点)时就触发,导致 enterHoldState 在中途捕获 pose,
                //   飞机继续飞到末点后又飞回捕获点。改为 idx >= total,等最后一个 skill_area 航点
                //   *到达* 才推进。此时 phase 已切到 SKILL_AREA(或 COMPLETE,无 skill_area),但 idx >= total
                //   仍成立。
                if (!latest_wp_status_) break;
                if (latest_wp_status_->skill_idx >= latest_wp_status_->skill_total &&
                    latest_wp_status_->skill_id == sr.msg.skill_id) {
                    sr.state = SkillState::EXIT_PENDING;
                    sr.state_enter_time = now;
                    sr.last_event = "IN_TASK to EXIT_PENDING (skill area done)";
                    ROS_WARN("[MissionManager] Skill[%zu] id=%s to EXIT_PENDING",
                             current_skill_index_, sr.msg.skill_id.c_str());
                }

                // in_task_timeout 检查
                if (in_task_timeout_ > 0.0 &&
                    (now - sr.in_task_enter_time).toSec() > in_task_timeout_) {
                    sr.state = SkillState::FAILED;
                    sr.state_enter_time = now;
                    sr.last_event = "IN_TASK to FAILED (timeout)";
                    ROS_ERROR("[MissionManager] Skill[%zu] id=%s in_task TIMEOUT (%.1fs)",
                              current_skill_index_, sr.msg.skill_id.c_str(),
                              (now - sr.in_task_enter_time).toSec());
                }
                break;
            }

            case SkillState::EXIT_PENDING: {
                // 收尾一帧后 → COMPLETE
                if ((now - sr.state_enter_time).toSec() > 0.1) {
                    sr.state = SkillState::COMPLETE;
                    sr.state_enter_time = now;
                    sr.last_event = "EXIT_PENDING to COMPLETE";
                    ROS_WARN("[MissionManager] Skill[%zu] id=%s to COMPLETE",
                             current_skill_index_, sr.msg.skill_id.c_str());
                }
                break;
            }

            case SkillState::COMPLETE: {
                // 推下一个 skill
                if (current_skill_index_ + 1 < skill_queue_.size()) {
                    current_skill_index_++;
                    // 注意:不再根据 skill_type 自动切 work_mode(违反协议语义:work_mode 只由 SET_WORKMODE 控制)
                    // 推进后调用 pushCurrentSkillToExecutor() 推到 waypoint_executor 即可
                    pushCurrentSkillToExecutor();
                    ROS_WARN("[MissionManager] >>>> Advancing to Skill[%zu]/%zu (type=%u, work_mode 保持 %s)",
                             current_skill_index_, skill_queue_.size(),
                             skill_queue_[current_skill_index_].msg.skill_type,
                             workModeToString().c_str());
                } else {
                    // 全部完成 — 检查最后一个 skill 是否是 Return (skill_type=103)
                    //   是 → 触发 PX4 AUTO.LAND 降落 (PX4 着陆后自动 disarm)
                    //   否 → 进入 PHASE_HOLDING, 原地+当前航向悬停等新 task_flow
                    const auto& last_skill = skill_queue_.back().msg;
                    if (last_skill.skill_type == 103 && !is_landing_in_progress_) {
                        ROS_WARN("[MissionManager] >>>> Last skill is Return (103) "
                                 " triggering PX4 AUTO.LAND (safety_timeout=%.1fs)",
                                 landing_safety_timeout_sec_);
                        triggerPx4Landing();
                    } else {
                        ROS_WARN_THROTTLE(5.0, "[MissionManager] >>>> All skills COMPLETE (%zu total) to PHASE_HOLDING",
                                 skill_queue_.size());
                        enterHoldState();
                    }
                }
                break;
            }

            case SkillState::FAILED: {
                // 失败 — 停在当前 skill,等用户手动干预(IDLE 模式 → 重新下发)
                ROS_ERROR_THROTTLE(5.0, "[MissionManager] Skill[%zu] id=%s FAILED, last_event=%s",
                                   current_skill_index_, sr.msg.skill_id.c_str(),
                                   sr.last_event.c_str());
                break;
            }
        }
    }

    /**
     * 101 集结 — 检查 expected_sns_ 是否都到了
     * 简化版本:从 heartbeats_ 缓存找,本机到达则总是 true
     */
    bool checkAllUavsGathered() const {
        if (expected_sns_.empty()) {
            // 没配置期望列表,默认自己到了就算完成
            return true;
        }
        // 当前实现:不查心跳,只检查本机 IN_TASK 即可
        // (Phase 7 会接 comm_node 的 inter_uav/gather_status)
        return true;
    }

    /**
     * 101 集结 — 发布本机 gather 状态
     */
    void publishGatherStatus() {
        multi_uav_strike::UavGatherStatus gs;
        gs.sn = nh_.getNamespace();
        if (!gs.sn.empty() && gs.sn[0] == '/') gs.sn = gs.sn.substr(1);
        gs.arrived_state = 1;  // 已到(IN_TASK 即视为已到)
        gs.lat = current_pose_.pose.position.x;
        gs.lon = current_pose_.pose.position.y;
        gs.alt = -current_pose_.pose.position.z;
        gs.priority = 100;  // 占位
        gather_status_pub_.publish(gs);
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

    void handleDeniedEnvFlight() {
        // 拒止环境飞行(预留模式,未实现)
        // 未来扩展:
        //   - GPS 拒止场景下用视觉/VIO 替代 GPS 定位
        //   - 强对抗环境下主动规避雷达/激光锁定
        //   - 与电子战模块配合
        // 当前实现:仅维持 IDLE-like 行为(不执行 search/track/strike),等待后续 PR
        current_task_status_ = TaskStatus::IDLE;
        ROS_WARN_THROTTLE(10.0, "[MissionManager] DENIED_ENV_FLIGHT mode reserved (not implemented yet)");
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

        // 修复:实际启动 guidance 时再切 phase → GUIDANCE_*
        // (之前 advanceSkillStateMachine 进 IN_TASK 时就切了,航点跟踪阶段 phase 提前变,
        //  且 is_guidance_active 与 phase 错位)。现在 phase 切换与 guidance 启动同步:
        //  YOLO 命中 → triggerStrike → startGuidanceApproach → 切 phase + enable + stop
        // 三件事同时发生,GS 看到 phase=GUIDANCE_* 必定意味着 guidance 真的在跑。
        if (current_work_mode_ == WorkMode::SEARCH_STRIKE) {
            current_phase_ = MissionPhase::PHASE_GUIDANCE_STRIKE;
        } else if (current_work_mode_ == WorkMode::SEARCH_TRACK) {
            current_phase_ = MissionPhase::PHASE_GUIDANCE_TRACK;
        }
        ROS_WARN("[MissionManager] >>>>> phase -> %s on guidance start",
                 missionPhaseToString().c_str());

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

        // 发送目标给制导,目前没有用
        guidance_target_pub_.publish(current_target_.pose);

        // 同步下发拦截速度(InterceptGuidance 用)
        std_msgs::Float32 speed_msg;
        speed_msg.data = static_cast<float>(guidance_speed_);
        guidance_speed_pub_.publish(speed_msg);

        ROS_WARN_THROTTLE(5.0, "[MissionManager] Starting guidance approach to target (mode: %s, guidance_speed_=%.2f)",
                          guidance_mode.c_str(), guidance_speed_);
    }

    void disableGuidance() {
        is_guidance_active_ = false;

        // 禁用制导
        std_msgs::Bool enable;
        enable.data = false;
        guidance_enable_pub_.publish(enable);

        ROS_INFO("[MissionManager] Guidance disabled for SEARCH_ONLY mode");
    }

    // void enableGuidance() {
    //     is_guidance_active_ = true;

    //     // 使能制导（guidance_control_node 在 enabled 但无目标时会持续发零速度，
    //     // 维持 PX4 OFFBOARD 心跳，避免 UAV 因 setpoint 断流触发 failsafe 降落）
    //     std_msgs::Bool enable;
    //     enable.data = true;
    //     guidance_enable_pub_.publish(enable);

    //     // 同步下发拦截速度(InterceptGuidance 用)
    //     std_msgs::Float32 speed_msg;
    //     speed_msg.data = static_cast<float>(guidance_speed_);
    //     guidance_speed_pub_.publish(speed_msg);

    //     ROS_INFO("[MissionManager] Guidance enabled (strike_speed=%.2f m/s)", guidance_speed_);
    // }

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

    // 默认打印 current_work_mode_；传参可打印任意 WorkMode(用于缓存/兜底值等场景)
    std::string workModeToString(WorkMode mode = static_cast<WorkMode>(-1)) {
        WorkMode m = (static_cast<int>(mode) < 0) ? current_work_mode_ : mode;
        switch (m) {
            case WorkMode::IDLE: return "IDLE";
            case WorkMode::SEARCH_ONLY: return "SEARCH_ONLY";
            case WorkMode::SEARCH_TRACK: return "SEARCH_TRACK";
            case WorkMode::SEARCH_STRIKE: return "SEARCH_STRIKE";
            case WorkMode::DENIED_ENV_FLIGHT: return "DENIED_ENV_FLIGHT";
            default: return "UNKNOWN";
        }
    }

    // 默认打印 current_phase_；传参可打印任意 MissionPhase
    std::string missionPhaseToString(MissionPhase phase = static_cast<MissionPhase>(-1)) {
        MissionPhase p = (static_cast<int>(phase) < 0) ? current_phase_ : phase;
        switch (p) {
            case MissionPhase::PHASE_GROUND_IDLE:      return "GROUND_IDLE";
            case MissionPhase::PHASE_TAKING_OFF:       return "TAKING_OFF";
            case MissionPhase::PHASE_HOVERING:         return "HOVERING";
            case MissionPhase::PHASE_WAYPOINT_FOLLOW:  return "WAYPOINT_FOLLOW";
            case MissionPhase::PHASE_GUIDANCE_TRACK:   return "GUIDANCE_TRACK";
            case MissionPhase::PHASE_GUIDANCE_STRIKE:  return "GUIDANCE_STRIKE";
            case MissionPhase::PHASE_RETURNING:        return "RETURNING";
            case MissionPhase::PHASE_COMPLETE:         return "COMPLETE";
            case MissionPhase::PHASE_FAILED:           return "FAILED";
            case MissionPhase::PHASE_HOLDING:          return "HOLDING";
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
