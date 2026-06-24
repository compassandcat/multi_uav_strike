/**
 * waypoint_executor_node.cpp
 * 航点执行器：将 GPS 航点列表转换为飞控可执行的 NED 速度指令
 *
 * 职责：
 * - 接收地面站发送的 GPS 航点（经纬高）
 * - GPS → NED 坐标转换（基准点：安阳）
 * - 计算速度指令，引导无人机到达航点
 * - 发布 NED 坐标系速度指令给飞控（或仿真器）
 * - 在 RViz 上显示航点（NWU 坐标系）
 *
 * 仿真/真机切换：
 * - use_sim=true: 发送到 /quad 命名空间（仿真）
 * - use_sim=false: 发送到 /mavros 命名空间（真机）
 *
 * 坐标系说明：
 * - 输入：GPS (lat, lon, alt)
 * - 内部：NED (North-East-Down)
 * - 输出：NED 速度指令给飞控
 * - 显示：NWU (用于 RViz)
 *
 * 订阅：
 * - /mission/waypoint_cmd           - GPS 航点命令（来自 mission_manager 或地面站）
 * - /quad/pose 或 /mavros/local_position/pose - 本机位置（NED）
 * - /inter_uav/other_uav_poses     - 邻居无人机位置
 *
 * 发布：
 * - /quad/setpoint_velocity/cmd_vel_unstamped 或 /mavros/... - NED 速度指令
 * - /waypoint_executor/status       - 执行状态
 * - /waypoint_executor/waypoints_rviz - 航点显示（RViz）
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <vector>

// 安阳基准点（用于 GPS → NED 转换）
const double ANYANG_LAT = 36.096;      // 安阳纬度
const double ANYANG_LON = 114.392;     // 安阳经度
const double ANYANG_ALT = 100.0;       // 安阳海拔（米）

class WaypointExecutor {
private:
    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ============== 订阅 ==============
    ros::Subscriber waypoint_cmd_sub_;       // GPS 航点命令
    ros::Subscriber self_pose_sub_;          // 本机位置（NED）
    ros::Subscriber other_uav_poses_sub_;    // 邻居无人机位置
    ros::Subscriber control_sub_;            // 控制命令（来自 mission_manager）
    ros::Subscriber mission_mode_sub_;       // 工作模式（SEARCH_ONLY/SEARCH_TRACK/SEARCH_STRIKE/IDLE）

    // ============== 发布 ==============
    ros::Publisher setpoint_velocity_pub_;    // NED 速度指令
    ros::Publisher flight_mode_pub_;         // 飞行模式
    ros::Publisher status_pub_;              // 执行状态
    ros::Publisher waypoints_rviz_pub_;     // 航点显示（RViz）
    ros::Publisher waypoint_path_pub_;      // 航点路径（RViz Path显示）

    // ============== 定时器 ==============
    ros::Timer executor_timer_;

    // ============== 状态 ==============
    // 航点队列
    struct Waypoint {
        double lat;      // 纬度
        double lon;     // 经度
        double alt;     // 高度
        double ned_x;   // NED X (North)
        double ned_y;   // NED Y (East)
        double ned_z;   // NED Z (Down)
        double desired_yaw_ned;  // 期望航向角（NED，北偏东，rad，来自地面站）
        bool reached;
    };
    std::vector<Waypoint> waypoint_queue_;
    size_t current_waypoint_index_;
    bool is_waypoints_received_;
    bool is_executing_;

// 本机状态（NED）
    double current_ned_x_;
    double current_ned_y_;
    double current_ned_z_;
    double current_yaw_ned_;  // 当前偏航角（NED，北偏东，rad）
    bool is_pose_received_;

    // 速度参数
    double uav_speed_;   // 飞行速度 (m/s)
    double arrival_threshold_;  // 到达阈值 (m)

    // ============== 参数 ==============
    double executor_rate_;
    int flight_mode_velocity_;
    int flight_mode_position_;

    // 基准点参数
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;

    // 仿真/真机切换
    bool use_sim_;
    std::string pose_topic_;
    std::string vel_topic_;

    // 航向控制选择：true=跟踪地面站发送的航点期望航向，false=计算航点指向
    bool use_desired_yaw_from_wp_;

    // 邻居无人机
    struct NeighborUav {
        double ned_x, ned_y, ned_z;
    };
    std::vector<NeighborUav> neighbors_;
    double avoidance_safe_distance_;

    // 无人机编号（用于区分颜色）
    int uav_id_;

    // 最后一个航点到位后是否已打印过日志（避免每周期刷屏）
    bool last_waypoint_reached_logged_;

    // 最后一个航点 P-controller 增益（用于位置保持，避免震荡）
    double hold_kp_;

    // 当前工作模式（SEARCH_ONLY/SEARCH_TRACK/SEARCH_STRIKE/IDLE）
    // 只有 SEARCH_ONLY 模式下航点执行器才发控制指令，其他模式交给 guidance_control
    std::string current_work_mode_;

public:
    WaypointExecutor() : nh_private_("~"),
        current_waypoint_index_(0),
        is_waypoints_received_(false),
        is_executing_(false),
        is_pose_received_(false),
        uav_speed_(5.0),
        arrival_threshold_(2.0),
        executor_rate_(50.0),
        flight_mode_velocity_(0),
        flight_mode_position_(2),
        ref_lat_(ANYANG_LAT),
        ref_lon_(ANYANG_LON),
        ref_alt_(ANYANG_ALT),
        use_sim_(true),
        avoidance_safe_distance_(10.0),
        last_waypoint_reached_logged_(false),
        hold_kp_(0.8),
        current_work_mode_("SEARCH_ONLY") {

        initParams();
        initSubscribers();
        initPublishers();
        initTimers();

        ROS_INFO("[WaypointExecutor] Initialized:");
        ROS_INFO("[WaypointExecutor]   Mode: %s", use_sim_ ? "SIMULATION (/quad)" : "REAL (/mavros)");
        ROS_INFO("[WaypointExecutor]   Ref: lat=%.6f, lon=%.6f, alt=%.1f", ref_lat_, ref_lon_, ref_alt_);
        ROS_INFO("[WaypointExecutor]   Speed: %.1f m/s, arrival_thresh: %.1f m", uav_speed_, arrival_threshold_);
    }

    void initParams() {
        nh_private_.param<double>("executor_rate", executor_rate_, 50.0);
        nh_private_.param<double>("uav_speed", uav_speed_, 5.0);
        nh_private_.param<double>("arrival_threshold", arrival_threshold_, 2.0);
        nh_private_.param<int>("flight_mode_velocity", flight_mode_velocity_, 0);
        nh_private_.param<int>("flight_mode_position", flight_mode_position_, 2);
        nh_private_.param<double>("avoidance_safe_distance", avoidance_safe_distance_, 10.0);
        // 最后一个航点位置保持的 P-controller 增益（1m 误差 → kp m/s）
        nh_private_.param<double>("hold_kp", hold_kp_, 0.8);

        // 自动从命名空间获取 uav_id（如 ns="uav0" → id=0）
        std::string ns = ros::this_node::getNamespace();
        size_t underscore_pos = ns.find("uav");
        if (underscore_pos != std::string::npos) {
            std::string num_str = ns.substr(underscore_pos + 3);
            uav_id_ = atoi(num_str.c_str());
        } else {
            uav_id_ = 0;
        }

        // 基准点参数（默认为安阳）
        nh_private_.param<double>("ref_lat", ref_lat_, ANYANG_LAT);
        nh_private_.param<double>("ref_lon", ref_lon_, ANYANG_LON);
        nh_private_.param<double>("ref_alt", ref_alt_, ANYANG_ALT);

        // 仿真/真机切换
        nh_private_.param<bool>("use_sim", use_sim_, true);
        if (use_sim_) {
            pose_topic_ = "quad/pose";
            vel_topic_ = "quad/setpoint_velocity/cmd_vel_unstamped";
        } else {
            pose_topic_ = "mavros/local_position/pose";
            vel_topic_ = "mavros/setpoint_velocity/cmd_vel_unstamped";
        }

        // 航向控制选择：true=跟踪地面站发送的期望航向，false=计算航点指向
        nh_private_.param<bool>("use_desired_yaw_from_wp", use_desired_yaw_from_wp_, true);
    }

    void initSubscribers() {
        // GPS 航点命令
        waypoint_cmd_sub_ = nh_.subscribe(
            "mission/waypoint_cmd", 10,
            &WaypointExecutor::waypointCmdCallback, this);

        // 本机位置（NED，来自飞控或仿真器）
        self_pose_sub_ = nh_.subscribe(
            pose_topic_, 10,
            &WaypointExecutor::selfPoseCallback, this);

        // 邻居无人机位置
        other_uav_poses_sub_ = nh_.subscribe(
            "inter_uav/other_uav_poses", 10,
            &WaypointExecutor::otherUavPosesCallback, this);

        // 控制命令（来自 mission_manager）
        control_sub_ = nh_.subscribe(
            "waypoint_executor/control", 10,
            &WaypointExecutor::controlCallback, this);

        // 工作模式（来自 mission_manager 转发的地面站指令）
        // 只有 SEARCH_ONLY 模式下航点执行器才发控制指令
        mission_mode_sub_ = nh_.subscribe(
            "mission/mode", 10,
            &WaypointExecutor::missionModeCallback, this);
    }

    void initPublishers() {
        // NED 速度指令（仿真或真机）
        setpoint_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(
            vel_topic_, 10);

        // 飞行模式（仿真或真机）
        std::string flight_mode_topic = use_sim_ ? "quad/flight_mode" : "mavros/flight_mode";
        flight_mode_pub_ = nh_.advertise<std_msgs::Int16>(
            flight_mode_topic, 10);

        // 执行状态
        status_pub_ = nh_.advertise<std_msgs::String>(
            "waypoint_executor/status", 10);

        // 航点显示（RViz - NWU 坐标系）
        waypoints_rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "waypoint_executor/waypoints_rviz", 10);

        // 航点路径（RViz Path显示）
        waypoint_path_pub_ = nh_.advertise<nav_msgs::Path>(
            "waypoint_executor/waypoint_path", 10);
    }

    void initTimers() {
        executor_timer_ = nh_.createTimer(
            ros::Duration(1.0 / executor_rate_),
            &WaypointExecutor::executorTimerCallback, this);
    }

    // ============== GPS <-> NED 转换 ==============

    /**
     * GPS (WGS84) -> NED 坐标转换
     */
    void gpsToNed(double lat, double lon, double alt,
                  double& ned_x, double& ned_y, double& ned_z) {
        const double EARTH_R = 6378137.0;

        // 纬度差 -> N (北向)
        double d_lat = lat - ref_lat_;
        ned_x = d_lat * M_PI / 180.0 * EARTH_R;

        // 经度差 -> E (东向)
        double d_lon = lon - ref_lon_;
        ned_y = d_lon * M_PI / 180.0 * EARTH_R * cos(ref_lat_ * M_PI / 180.0);

        // 高度差 -> D (下向)
        ned_z = -(alt - ref_alt_);
    }

    // ============== 回调函数 ==============

    void waypointCmdCallback(const nav_msgs::Path::ConstPtr& msg) {
        waypoint_queue_.clear();

        for (const auto& pose : msg->poses) {
            Waypoint wp;

            // 从 pose 中提取 GPS 信息
            // nav_msgs/Path 的 pose.position 存储 GPS: x=lat, y=lon, z=alt
            wp.lat = pose.pose.position.x;
            wp.lon = pose.pose.position.y;
            wp.alt = pose.pose.position.z;

            // GPS -> NED 转换
            gpsToNed(wp.lat, wp.lon, wp.alt, wp.ned_x, wp.ned_y, wp.ned_z);

            // 从 pose.orientation 提取期望航向角（四元数 -> yaw）
            // 如果是默认四元数 (w=1)，则期望航向为0
            double qx = pose.pose.orientation.x;
            double qy = pose.pose.orientation.y;
            double qz = pose.pose.orientation.z;
            double qw = pose.pose.orientation.w;

            // 检测是否是单位四元数（没有设置航向）
            if (fabs(qw - 1.0) < 0.01 && fabs(qx) < 0.01 && fabs(qy) < 0.01 && fabs(qz) < 0.01) {
                wp.desired_yaw_ned = 0.0;  // 默认朝向
            } else {
                // 从四元数提取 yaw
                wp.desired_yaw_ned = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            }

            wp.reached = false;
            waypoint_queue_.push_back(wp);

            ROS_INFO("[WaypointExecutor] GPS (%.6f, %.6f, %.1f) -> NED (%.2f, %.2f, %.2f) yaw=%.1fdeg",
                     wp.lat, wp.lon, wp.alt, wp.ned_x, wp.ned_y, wp.ned_z,
                     wp.desired_yaw_ned * 180.0 / M_PI);
        }

        current_waypoint_index_ = 0;
        is_waypoints_received_ = true;
        is_executing_ = true;
        last_waypoint_reached_logged_ = false;

        // 发布航点给 RViz 显示
        publishWaypointsForRviz();

        ROS_INFO("[WaypointExecutor] Received %lu GPS waypoints, starting execution",
                 waypoint_queue_.size());
    }

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 注意：根据 use_sim 区分输入坐标系
        //   - use_sim_=true  (jMAVSim):  /quad/pose 直接是 NED
        //   - use_sim_=false (PX4 SITL): /mavros/local_position/pose 是 ENU (REP-103)
        //                                必须先 ENU -> NED，否则后续 NED 减法/算速度全错
        if (use_sim_) {
            current_ned_x_ = msg->pose.position.x;
            current_ned_y_ = msg->pose.position.y;
            current_ned_z_ = msg->pose.position.z;
        } else {
            // ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            //   ENU: x=East,  y=North, z=Up
            //   NED: x=North, y=East, z=Down
            current_ned_x_ = msg->pose.position.y;   // ENU y (north) -> NED x
            current_ned_y_ = msg->pose.position.x;   // ENU x (east)  -> NED y
            current_ned_z_ = -msg->pose.position.z;  // ENU z (up)    -> NED z
        }

        // 从四元数提取偏航角（NED，北偏东）
        // NED坐标系下：yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        // 但更通用的公式（假设绕Z轴旋转）是：
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;

        // NED坐标系下从四元数提取yaw（ZYX顺序）
        // yaw (heading) = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        // 但由于我们使用的是Z轴向上的表示法，实际应该是：
        current_yaw_ned_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        is_pose_received_ = true;
    }

    void otherUavPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        neighbors_.clear();
        for (const auto& pose : msg->poses) {
            NeighborUav neighbor;
            // 接收的是 GPS: pose.position.x=lat, .y=lon, .z=alt
            // 转换为本地 NED
            gpsToNed(pose.position.x, pose.position.y, pose.position.z,
                     neighbor.ned_x, neighbor.ned_y, neighbor.ned_z);
            neighbors_.push_back(neighbor);
        }
    }

    // ============== 定时器回调 ==============

    void executorTimerCallback(const ros::TimerEvent&) {
        if (!is_pose_received_) {
            return;
        }

        // 只有 SEARCH_ONLY 模式下才发航点控制指令
        // 其他模式（SEARCH_TRACK/SEARCH_STRIKE/IDLE）由 guidance_control_node 接管
        if (current_work_mode_ != "SEARCH_ONLY") {
            return;
        }

        if (!is_executing_ || !is_waypoints_received_) {
            return;
        }

        executeWaypointFlight();
    }

    // ============== 模式回调 ==============

    void missionModeCallback(const std_msgs::String::ConstPtr& msg) {
        std::string new_mode = msg->data;
        if (new_mode == current_work_mode_) {
            return;
        }
        ROS_WARN("[WaypointExecutor] Work mode changed: %s -> %s",
                 current_work_mode_.c_str(), new_mode.c_str());
        current_work_mode_ = new_mode;

        // 切出 SEARCH_ONLY 模式时，立即停发指令并清空航点，避免与 guidance_control 抢控制权
        if (new_mode != "SEARCH_ONLY") {
            if (is_executing_) {
                ROS_WARN("[WaypointExecutor] Not SEARCH_ONLY anymore, pausing waypoint control");
            }
            is_executing_ = false;
            waypoint_queue_.clear();
            publishZeroVelocity();
        } else {
            // 切回 SEARCH_ONLY：等新的 waypoint_cmd 到来再继续，不要自动 resume 旧航点
            is_executing_ = false;
            is_waypoints_received_ = false;
            last_waypoint_reached_logged_ = false;
        }
    }

    // ============== 执行逻辑 ==============

    void executeWaypointFlight() {
        if (current_waypoint_index_ >= waypoint_queue_.size()) {
            ROS_INFO("[WaypointExecutor] All waypoints reached!");
            is_executing_ = false;
            publishZeroVelocity();
            // TODO: 后续应切换到定点模式等待新指令
            publishStatus("all_waypoints_completed");
            return;
        }

        Waypoint& current_wp = waypoint_queue_[current_waypoint_index_];

        double dx = current_wp.ned_x - current_ned_x_;
        double dy = current_wp.ned_y - current_ned_y_;
        double dz = current_wp.ned_z - current_ned_z_;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);

        // 计算期望航向角
        double desired_yaw_ned;
        if (use_desired_yaw_from_wp_) {
            // 跟踪地面站发送的航点期望航向
            desired_yaw_ned = current_wp.desired_yaw_ned;
        } else {
            if (current_waypoint_index_ == 0) {
                // 第一个航点：始终指向该航点
                desired_yaw_ned = atan2(dy, dx);
            } else {
                // 后续航点：使用上一个航点到当前航点的方向，不再切换
                const auto& prev_wp = waypoint_queue_[current_waypoint_index_ - 1];
                double dx_prev = current_wp.ned_x - prev_wp.ned_x;
                double dy_prev = current_wp.ned_y - prev_wp.ned_y;
                desired_yaw_ned = atan2(dy_prev, dx_prev);
            }
        }

        ROS_DEBUG_THROTTLE(1.0, "[WaypointExecutor] WP[%zu/%lu] dist=%.2f m, desired_yaw=%.1fdeg",
                          current_waypoint_index_, waypoint_queue_.size(), dist,
                          desired_yaw_ned * 180.0 / M_PI);

        if (dist < arrival_threshold_) {
            waypoint_queue_[current_waypoint_index_].reached = true;

            bool is_last = (current_waypoint_index_ == waypoint_queue_.size() - 1);
            if (is_last) {
                // 最后一个航点：不递增 index，持续发送速度指令维持位置
                // 否则 SITL 会因为停止 setpoint 进入 RTL/降落。
                if (!last_waypoint_reached_logged_) {
                    ROS_WARN("[WaypointExecutor] ===== Last waypoint %zu reached, holding position =====",
                             current_waypoint_index_);
                    last_waypoint_reached_logged_ = true;
                }
                // 不 return，落到下面的速度计算（用 P-controller）
            } else {
                ROS_WARN("[WaypointExecutor] ===== Waypoint %zu reached! =====", current_waypoint_index_);
                current_waypoint_index_++;
                publishZeroVelocity();
                return;
            }
        }

        // 计算速度指令（最后一个航点用 P-controller 维持位置，避免震荡）
        bool is_last = (current_waypoint_index_ == waypoint_queue_.size() - 1);
        double vx, vy, vz;
        computeVelocityCommand(current_wp.ned_x, current_wp.ned_y, current_wp.ned_z,
                              vx, vy, vz, is_last);

        // 机间避障（人工势场）
        applyInterUavAvoidance(vx, vy, vz);

        // 发布速度和航向指令
        publishVelocityCommandWithYaw(vx, vy, vz, desired_yaw_ned);

        std::ostringstream oss;
        oss << "wp:" << current_waypoint_index_ << "/" << waypoint_queue_.size()
            << ",dist:" << dist;
        publishStatus(oss.str());
    }

    void computeVelocityCommand(double target_x, double target_y, double target_z,
                                double& vx, double& vy, double& vz,
                                bool position_hold = false) {
        double dx = target_x - current_ned_x_;
        double dy = target_y - current_ned_y_;
        double dz = target_z - current_ned_z_;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);

        if (dist > 0.1) {
            if (position_hold) {
                // 最后一个航点用 P-controller：速度 ∝ 距离，距离 → 0 速度 → 0，避免震荡
                vx = hold_kp_ * dx;
                vy = hold_kp_ * dy;
                vz = hold_kp_ * dz;
            } else {
                vx = (dx / dist) * uav_speed_;
                vy = (dy / dist) * uav_speed_;
                vz = (dz / dist) * uav_speed_;
            }
        } else {
            vx = vy = vz = 0.0;
        }
    }

    /**
     * 机间避障（人工势场）
     * - 斥力：邻居无人机靠近时推开
     * - 引力：向目标点飞行
     */
    void applyInterUavAvoidance(double& vx, double& vy, double& vz) {
        if (neighbors_.empty()) {
            return;
        }

        double repulsion_gain = 20.0;  // 斥力增益
        double min_safe_distance = 5.0; // 安全距离阈值(m)

        double fx = 0.0, fy = 0.0, fz = 0.0;

        for (const auto& neighbor : neighbors_) {
            double dx = current_ned_x_ - neighbor.ned_x;
            double dy = current_ned_y_ - neighbor.ned_y;
            double dz = current_ned_z_ - neighbor.ned_z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);

            if (dist < min_safe_distance && dist > 0.1) {
                // 斥力与距离平方成反比
                double force_magnitude = repulsion_gain / (dist * dist);
                fx += (dx / dist) * force_magnitude;
                fy += (dy / dist) * force_magnitude;
                fz += (dz / dist) * force_magnitude;
            }
        }
        
        // 叠加到速度指令
        vx += fx;
        vy += fy;
        //vz += fz;

        // 限速
        double speed = sqrt(vx*vx + vy*vy + vz*vz);
        if (speed > uav_speed_ * 1.5) {
            double scale = (uav_speed_ * 1.5) / speed;
            vx *= scale;
            vy *= scale;
            vz *= scale;
        }
    }

    /**
     * 发布速度指令（包含偏航角速率）
     * @param vx, vy, vz NED 速度
     * @param desired_yaw_ned 期望偏航角（NED，北偏东，rad）
     *
     * NED 坐标系下：
     * - atan2(dy, dx) 给出从正北方向顺时针旋转的角度
     * - 正值 = 右转（东向）
     * - angular.z 作为角速率控制偏航
     *
     * PX4 SITL: 需要将 NED 转换为 ENU
     */
    void publishVelocityCommandWithYaw(double vx, double vy, double vz, double desired_yaw_ned) {
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = vx;
        vel_cmd.linear.y = vy;
        vel_cmd.linear.z = vz;

        // angular.z 直接发送期望航向角（rad），由飞控完成控制
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = desired_yaw_ned;

        // PX4 SITL: NED -> ENU 转换
        if (!use_sim_) {
            double ned_vx = vel_cmd.linear.x;
            double ned_vy = vel_cmd.linear.y;
            double ned_vz = vel_cmd.linear.z;
            double ned_yaw = vel_cmd.angular.z;
            // NED -> ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
            vel_cmd.linear.x = ned_vy;
            vel_cmd.linear.y = ned_vx;
            vel_cmd.linear.z = -ned_vz;
            vel_cmd.angular.z = -ned_yaw;
        }

        setpoint_velocity_pub_.publish(vel_cmd);

        std_msgs::Int16 mode_msg;
        mode_msg.data = flight_mode_velocity_;
        flight_mode_pub_.publish(mode_msg);

        // 实时打印调试信息（NWU 坐标系）
        double uav_nwu_x = current_ned_x_;
        double uav_nwu_y = -current_ned_y_;
        double uav_nwu_z = -current_ned_z_;

        double target_nwu_x = 0.0, target_nwu_y = 0.0, target_nwu_z = 0.0;
        double dist_to_target = 0.0;
        if (current_waypoint_index_ < waypoint_queue_.size()) {
            const auto& wp = waypoint_queue_[current_waypoint_index_];
            target_nwu_x = wp.ned_x;
            target_nwu_y = -wp.ned_y;
            target_nwu_z = -wp.ned_z;
            double dx = wp.ned_x - current_ned_x_;
            double dy = wp.ned_y - current_ned_y_;
            double dz = wp.ned_z - current_ned_z_;
            dist_to_target = sqrt(dx*dx + dy*dy + dz*dz);
        }

        double vel_nwu_x = vx;
        double vel_nwu_y = -vy;
        double vel_nwu_z = -vz;

        ROS_INFO_THROTTLE(0.5,
            "[Flight] NWU (%.2f, %.2f, %.2f) | Target WP[%zu]: (%.2f, %.2f, %.2f) dist=%.2fm | Vel(%.2f, %.2f, %.2f) | Yaw: cur=%.0f des=%.0f",
            uav_nwu_x, uav_nwu_y, uav_nwu_z,
            current_waypoint_index_,
            target_nwu_x, target_nwu_y, target_nwu_z,
            dist_to_target,
            vel_nwu_x, vel_nwu_y, vel_nwu_z,
            current_yaw_ned_ * 180.0 / M_PI,
            desired_yaw_ned * 180.0 / M_PI);
    }

    void publishZeroVelocity() {
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = 0.0;
        vel_cmd.linear.y = 0.0;
        vel_cmd.linear.z = 0.0;
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = 0.0;  // 停止时偏航角速率也为0

        setpoint_velocity_pub_.publish(vel_cmd);

        ROS_DEBUG_THROTTLE(1.0, "[WaypointExecutor] Zero velocity published");
    }

    void publishStatus(const std::string& status) {
        std_msgs::String status_msg;
        status_msg.data = status;
        status_pub_.publish(status_msg);
    }

    void publishWaypointsForRviz() {
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "waypoint_path";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.3;
        // 颜色根据 uav_id 设置
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.3 + 0.2 * uav_id_;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;

        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        for (size_t i = 0; i < waypoint_queue_.size(); ++i) {
            const auto& wp = waypoint_queue_[i];

            // NED -> NWU for RViz
            geometry_msgs::Point p;
            p.x = wp.ned_x;
            p.y = -wp.ned_y;
            p.z = -wp.ned_z;
            line_strip.points.push_back(p);

            // Path for RViz
            geometry_msgs::PoseStamped path_pose;
            path_pose.header.frame_id = "map";
            path_pose.header.stamp = ros::Time::now();
            path_pose.pose.position.x = wp.ned_x;
            path_pose.pose.position.y = -wp.ned_y;
            path_pose.pose.position.z = -wp.ned_z;
            path_pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(path_pose);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "waypoints";
            marker.id = i + 1;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = wp.ned_x;
            marker.pose.position.y = -wp.ned_y;
            marker.pose.position.z = -wp.ned_z;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 2.0;
            marker.scale.y = 2.0;
            marker.scale.z = 2.0;
            // 颜色根据 uav_id 设置：uav0=绿色系，uav1=蓝色系，uav2=黄色系
            if (uav_id_ == 0) {
                marker.color.r = 0.2;
                marker.color.g = 0.8;
                marker.color.b = 0.2;
            } else if (uav_id_ == 1) {
                marker.color.r = 0.2;
                marker.color.g = 0.4;
                marker.color.b = 0.8;
            } else if (uav_id_ == 2) {
                marker.color.r = 1.0;
                marker.color.g = 0.6;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        line_strip.lifetime = ros::Duration(0);
        marker_array.markers.push_back(line_strip);

        waypoints_rviz_pub_.publish(marker_array);
        waypoint_path_pub_.publish(path_msg);
    }

    // ============== 公共接口 ==============

    void controlCallback(const std_msgs::String::ConstPtr& msg) {
        std::string cmd = msg->data;
        ROS_WARN("[WaypointExecutor] >>>>> Received control command: %s", cmd.c_str());

        if (cmd == "stop") {
            stop();
        } else if (cmd == "pause") {
            pause();
        } else if (cmd == "resume") {
            resume();
        } else if (cmd.substr(0, 7) == "spiral:") {
            // 螺旋接近命令，暂不支持
            ROS_WARN("[WaypointExecutor] Spiral command not implemented yet: %s", cmd.c_str());
        }
    }

    void stop() {
        is_executing_ = false;
        waypoint_queue_.clear();
        ROS_WARN("[WaypointExecutor] >>>>> stop() called, about to publish zero velocity");
        publishZeroVelocity();
        ROS_WARN("[WaypointExecutor] >>>>> stop() completed");
        ROS_INFO("[WaypointExecutor] Stopped");
    }

    void pause() {
        is_executing_ = false;
        publishZeroVelocity();
        ROS_INFO("[WaypointExecutor] Paused");
    }

    void resume() {
        if (is_waypoints_received_) {
            is_executing_ = true;
            ROS_INFO("[WaypointExecutor] Resumed");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_executor_node");
    WaypointExecutor executor;
    ros::spin();
    return 0;
}
