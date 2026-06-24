#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <Eigen/Eigen>

#include "multi_uav_strike/guidance_strategies.h"

class GuidanceControlNode {
private:
    // ROS interfaces
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber target_pose_sub_;
    ros::Subscriber target_twist_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Subscriber los_angle_sub_;
    ros::Subscriber real_target_sub_;  // 真实目标位置用于评估
    ros::Subscriber enable_sub_;       // 使能控制（来自 mission_manager）
    ros::Subscriber mode_sub_;        // 模式控制："strike" 或 "track"
    ros::Subscriber other_uav_poses_sub_;  // 邻居无人机位置

    // Publishers
    ros::Publisher vel_cmd_pub_;          // setpoint_velocity/cmd_vel_unstamped
    ros::Publisher attitude_cmd_pub_;     // setpoint_attitude/attitude
    ros::Publisher attitude_rates_pub_;   // setpoint_attitude/cmd_vel (角速率)
    ros::Publisher thrust_cmd_pub_;       // thrust
    ros::Publisher flight_mode_pub_;      // flight_mode
    ros::Publisher strike_eval_pub_;      // 打击评估结果
    ros::Publisher thrust_dir_pub_;       // 推力方向可视化
    ros::Publisher uav_pose_nwu_pub_;     // NWU姿态发布(RViz用)
    ros::Publisher intercept_point_pub_; // 拦截点可视化
    ros::Publisher debug_pub_;           // 调试信息

    // Timer for guidance computation
    ros::Timer guidance_timer_;

    // Current state
    geometry_msgs::PoseStamped current_uav_pose_;
    geometry_msgs::PoseStamped current_target_pose_;
    geometry_msgs::TwistStamped current_target_twist_;
    geometry_msgs::Point current_los_angle_;
    geometry_msgs::Point real_target_pos_;  // 真实目标位置

    bool is_uav_pose_received_ = false;
    bool is_target_pose_received_ = false;
    bool is_target_twist_received_ = false;
    bool is_los_received_ = false;
    bool is_real_target_received_ = false;

    // 使能控制：SEARCH_ONLY 模式下 guidance 不工作
    bool is_enabled_;
    std::string current_mode_;  // "strike" 或 "track"

    // Guidance strategy
    std::unique_ptr<multi_uav_strike::GuidanceStrategy> guidance_strategy_;
    multi_uav_strike::GuidanceStrategyType current_strategy_type_;
    std::string current_strategy_name_;

    // Control mode
    int flight_mode_velocity_;
    int flight_mode_attitude_;

    // 仿真/真机切换
    bool use_sim_;
    std::string pose_topic_;
    std::string vel_cmd_topic_;
    std::string attitude_cmd_topic_;
    std::string attitude_rates_topic_;
    std::string thrust_cmd_topic_;

    // GPS 参考点（GPS -> NED 转换用）
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;

    // 跟踪约束参数
    double min_altitude_;            // 最小高度限制
    double max_tracking_distance_;   // 最大跟踪距离
    double desired_tracking_angle_;   // 期望跟踪角度（目标在机头前下方）

    // TRACK模式专用参数
    double track_altitude_;          // 固定跟踪高度（m）
    double track_horiz_dist_;        // 水平安全距离（m，stand-off距离，到此距离速度降为0）
    double track_P_;                 // P控制增益
    double track_I_;                 // I控制增益
    double track_max_speed_;         // 最大速度限制（m/s）
    double track_k_approach_;        // 距离→速度的斜率（m/s per m，距离每超出 stand-off 1m 速度增加多少）
    double track_integral_x_;        // PI积分项 x
    double track_integral_y_;        // PI积分项 y

    // 偏航角速度控制参数（PX4 SITL 用）
    double los_kp_yaw_;              // 偏航 P 控制增益
    double los_max_rate_;            // 偏航角速度限幅（rad/s）

    // STRIKE模式参数
    double strike_min_altitude_;      // 开始下降的最小高度阈值

    // 打击评估相关
    double strike_distance_threshold_;  // 打击成功距离阈值(m)
    bool first_strike_evaluated_;      // 是否已评估第一次打击
    double min_strike_distance_;       // 第一次打击最小距离
    double strike_altitude_;            // 打击时无人机高度
    double strike_time_;                // 打击时间戳
    bool is_approaching_;               // 是否正在接近目标
    double prev_distance_;              // 上一时刻距离

    // 机间避障相关
    struct NeighborUav {
        double ned_x, ned_y, ned_z;
    };
    std::vector<NeighborUav> neighbors_;
    double avoidance_safe_distance_;

public:
    GuidanceControlNode() : nh_private_("~"),
        first_strike_evaluated_(false),
        min_strike_distance_(std::numeric_limits<double>::max()),
        strike_altitude_(0.0),
        strike_time_(0.0),
        is_approaching_(true),
        prev_distance_(std::numeric_limits<double>::max()),
        is_enabled_(false),
        current_mode_("strike"),
        track_integral_x_(0.0),
        track_integral_y_(0.0),
        strike_min_altitude_(20.0),
        track_k_approach_(0.5),
        use_sim_(true) {
        initParams();
        initSubscribers();
        initPublishers();
        initGuidanceStrategy();

        // Create timer for guidance loop (50 Hz)
        guidance_timer_ = nh_.createTimer(
            ros::Duration(1.0 / 50.0),
            &GuidanceControlNode::guidanceTimerCallback,
            this);

        ROS_INFO("Guidance Control Node initialized with strategy: %s (disabled by default)",
                 current_strategy_name_.c_str());
    }

    void initParams() {
        // 仿真/真机切换
        nh_private_.param<bool>("use_sim", use_sim_, true);

        // GPS 参考点（仿真时所有 UAV 共享，PX4 SITL 时各 UAV 用各自的 home）
        nh_private_.param<double>("ref_lat", ref_lat_, 36.096);
        nh_private_.param<double>("ref_lon", ref_lon_, 114.392);
        nh_private_.param<double>("ref_alt", ref_alt_, 100.0);

        // 根据 use_sim 设置 topic 名称
        if (use_sim_) {
            pose_topic_ = "quad/pose";
            vel_cmd_topic_ = "quad/setpoint_velocity/cmd_vel_unstamped";
            attitude_cmd_topic_ = "quad/setpoint_attitude/attitude";
            attitude_rates_topic_ = "quad/setpoint_attitude/cmd_vel";
            thrust_cmd_topic_ = "quad/thrust";
        } else {
            pose_topic_ = "mavros/local_position/pose";
            vel_cmd_topic_ = "mavros/setpoint_velocity/cmd_vel_unstamped";
            attitude_cmd_topic_ = "mavros/setpoint_attitude/attitude";
            attitude_rates_topic_ = "mavros/setpoint_attitude/cmd_vel";
            thrust_cmd_topic_ = "mavros/setpoint_attitude/thrust";
        }

        // Strategy selection parameter
        std::string strategy_str;
        nh_private_.param<std::string>("guidance_strategy", strategy_str, "intercept");

        if (strategy_str == "intercept") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::INTERCEPT;
            current_strategy_name_ = "intercept";
        } else if (strategy_str == "minsnap") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::MINSNAP;
            current_strategy_name_ = "minsnap";
        } else if (strategy_str == "los") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::LOS;
            current_strategy_name_ = "los";
        } else {
            ROS_WARN("[Guidance] Unknown strategy '%s', defaulting to 'intercept'", strategy_str.c_str());
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::INTERCEPT;
            current_strategy_name_ = "intercept";
        }

        nh_private_.param<int>("flight_mode_velocity", flight_mode_velocity_, 0);
        nh_private_.param<int>("flight_mode_attitude", flight_mode_attitude_, 1);
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);  // 2米内视为击中

        // TRACK模式专用参数
        nh_private_.param<double>("track_altitude", track_altitude_, 30.0);       // 固定跟踪高度
        nh_private_.param<double>("track_horiz_dist", track_horiz_dist_, 8.0);    // 水平安全距离
        nh_private_.param<double>("track_P", track_P_, 1.0);                     // P控制增益
        nh_private_.param<double>("track_I", track_I_, 0.1);                     // I控制增益
        nh_private_.param<double>("uav_speed", track_max_speed_, 5.0);     // 最大速度限制
        // 距离→速度的连续斜率（消除内/外边界的硬切换：每超出 stand-off 1m 速度增加多少 m/s）
        nh_private_.param<double>("track_k_approach", track_k_approach_, 0.5);

        // 偏航角速度控制（PX4 SITL）
        nh_private_.param<double>("los_kp_yaw", los_kp_yaw_, 1.5);
        nh_private_.param<double>("los_max_rate", los_max_rate_, 1.2);

        // STRIKE模式专用参数
        nh_private_.param<double>("strike_min_altitude", strike_min_altitude_, 20.0);  // 开始下降的最小高度阈值

        ROS_INFO("[Guidance] Mode: %s, pose_topic: %s", use_sim_ ? "SIMULATION" : "PX4 SITL", pose_topic_.c_str());
    }

    void initSubscribers() {
        target_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &GuidanceControlNode::targetPoseCallback, this);

        target_twist_sub_ = nh_.subscribe(
            "target_estimated_twist", 10,
            &GuidanceControlNode::targetTwistCallback, this);

        uav_pose_sub_ = nh_.subscribe(
            pose_topic_, 10,
            &GuidanceControlNode::uavPoseCallback, this);

        los_angle_sub_ = nh_.subscribe(
            "target_los_angle", 10,
            &GuidanceControlNode::losAngleCallback, this);

        // 订阅真实目标位置用于评估（用于判断是否真正击中目标）
        real_target_sub_ = nh_.subscribe(
            "/target_position", 10,
            &GuidanceControlNode::realTargetCallback, this);

        // 使能控制（SEARCH_ONLY 模式下不工作）
        enable_sub_ = nh_.subscribe(
            "guidance/enable", 10,
            &GuidanceControlNode::enableCallback, this);

        // 模式控制（"strike" 或 "track"）
        mode_sub_ = nh_.subscribe(
            "guidance/mode", 10,
            &GuidanceControlNode::modeCallback, this);

        // 邻居无人机位置订阅（机间避障用）
        other_uav_poses_sub_ = nh_.subscribe(
            "inter_uav/other_uav_poses", 10,
            &GuidanceControlNode::otherUavPosesCallback, this);
    }

    void initPublishers() {
        vel_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(
            vel_cmd_topic_, 10);

        attitude_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            attitude_cmd_topic_, 10);

        attitude_rates_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
            attitude_rates_topic_, 10);

        thrust_cmd_pub_ = nh_.advertise<std_msgs::Float32>(
            thrust_cmd_topic_, 10);

        // flight_mode_pub_ 在仿真模式使用 quad/flight_mode，PX4 模式不使用（通过 mavros/set_mode 切换）
        if (use_sim_) {
            flight_mode_pub_ = nh_.advertise<std_msgs::Int16>(
                "quad/flight_mode", 10);
        }

        strike_eval_pub_ = nh_.advertise<std_msgs::Bool>(
            "strike_evaluation", 10);  // 发布打击评估结果

        thrust_dir_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "thrust_direction", 10);  // 发布推力方向可视化

        uav_pose_nwu_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "quad/pose_nwu", 10);  // NWU姿态发布(RViz用)

        intercept_point_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "intercept_point", 10);  // 拦截点可视化
    }

    void initGuidanceStrategy() {
        guidance_strategy_ = multi_uav_strike::createGuidanceStrategy(
            current_strategy_type_, nh_private_);
    }

    // Callback methods
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_target_pose_ = *msg;
        is_target_pose_received_ = true;
    }

    void targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_target_twist_ = *msg;
        is_target_twist_received_ = true;
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (use_sim_) {
            // ===== NED → NWU 坐标转换 =====
            // NED: X=North, Y=East, Z=Down
            // NWU: X=North, Y=West, Z=Up
            // 位置：X不变, Y取反, Z取反
            current_uav_pose_.pose.position.x = msg->pose.position.x;
            current_uav_pose_.pose.position.y = -msg->pose.position.y;
            current_uav_pose_.pose.position.z = -msg->pose.position.z;

            // 四元数：w,x不变, y,z取反 (等价于绕X轴旋转180度)
            current_uav_pose_.pose.orientation.w = msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x = msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        } else {
            // ===== ENU → NED → NWU =====
            // Mavros 输入是 ENU: X=East, Y=North, Z=Up
            // ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            // NED -> NWU: x_nwu = x_ned, y_nwu = -y_ned, z_nwu = -z_ned
            // 合成 ENU -> NWU: x = y_enu, y = -x_enu, z = z_enu
            current_uav_pose_.pose.position.x = msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z = msg->pose.position.z;

            // 四元数: ENU->NED (180°绕X) + NED->NWU (180°绕X) = 恒等
            // 但实际上两个180°旋转的合成仍是180°旋转，等价于只做一次
            current_uav_pose_.pose.orientation.w = msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x = msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        }

        current_uav_pose_.header.stamp = msg->header.stamp;
        current_uav_pose_.header.frame_id = msg->header.frame_id;
        is_uav_pose_received_ = true;
    }

    void losAngleCallback(const geometry_msgs::Point::ConstPtr& msg) {
        current_los_angle_ = *msg;
        is_los_received_ = true;
    }

    void realTargetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        real_target_pos_ = *msg;
        is_real_target_received_ = true;
    }

    void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_enabled_ = msg->data;
        if (is_enabled_) {
            ROS_WARN_THROTTLE(5.0, "[Guidance] Guidance ENABLED (mode: %s)", current_mode_.c_str());
        } else {
            ROS_WARN_THROTTLE(5.0, "[Guidance] Guidance DISABLED (SEARCH_ONLY mode)");
        }
    }

    void modeCallback(const std_msgs::String::ConstPtr& msg) {
        current_mode_ = msg->data;
        ROS_WARN_THROTTLE(5.0, "[Guidance] Mode changed to: %s", current_mode_.c_str());
    }

    // 邻居无人机位置回调（机间避障用）
    void otherUavPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        neighbors_.clear();
        for (const auto& pose : msg->poses) {
            NeighborUav neighbor;
            // 接收的是 GPS: pose.position.x=lat, .y=lon, .z=alt
            // 转换为本地 NED
            double ned_x, ned_y, ned_z;
            gpsToNed(pose.position.x, pose.position.y, pose.position.z,
                     ned_x, ned_y, ned_z);
            neighbor.ned_x = ned_x;
            neighbor.ned_y = ned_y;
            neighbor.ned_z = ned_z;
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

    // 机间避障（人工势场）
    void applyInterUavAvoidance(double& vx, double& vy, double& vz) {
        if (neighbors_.empty()) {
            return;
        }

        double repulsion_gain = 10.0;
        double min_safe_distance = 25.0;

        double fx = 0.0, fy = 0.0, fz = 0.0;

        for (const auto& neighbor : neighbors_) {
            double dx = current_uav_pose_.pose.position.x - neighbor.ned_x;
            double dy = current_uav_pose_.pose.position.y - neighbor.ned_y;
            double dz = current_uav_pose_.pose.position.z - neighbor.ned_z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < min_safe_distance) {
                if (dist < 2)
                    dist = 2;  // 避免除以零
                double force_magnitude = repulsion_gain / (dist * dist);
                fx += (dx / dist) * force_magnitude;
                fy += (dy / dist) * force_magnitude;
                fz += (dz / dist) * force_magnitude;
            }
        }

        vx += fx;
        vy += fy;

        // 限速
        double speed = sqrt(vx*vx + vy*vy + vz*vz);
        if (speed > track_max_speed_ * 1.5) {
            double scale = (track_max_speed_ * 1.5) / speed;
            vx *= scale;
            vy *= scale;
            vz *= scale;
        }
    }

    // 计算打击评估
    void evaluateStrike() {
        if (first_strike_evaluated_ || !is_real_target_received_) {
            return;
        }

        // 计算无人机到真实目标的距离
        double dx = current_uav_pose_.pose.position.x - real_target_pos_.x;
        double dy = current_uav_pose_.pose.position.y - real_target_pos_.y;
        double dz = current_uav_pose_.pose.position.z - real_target_pos_.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        // 检测是否到达最近点（从接近转为远离）
        if (is_approaching_) {
            if (distance < prev_distance_) {
                // 仍在接近
                prev_distance_ = distance;
            } else {
                // 开始远离，到达最近点
                is_approaching_ = false;
                min_strike_distance_ = prev_distance_;
                strike_altitude_ = current_uav_pose_.pose.position.z;
                strike_time_ = ros::Time::now().toSec();

                // 发布评估结果
                std_msgs::Bool eval_msg;
                eval_msg.data = (min_strike_distance_ < strike_distance_threshold_);
                strike_eval_pub_.publish(eval_msg);

                ROS_WARN("[STRIKE EVAL] ===== First Strike Evaluation =====");
                ROS_WARN("[STRIKE EVAL] Min distance: %.2f m", min_strike_distance_);
                ROS_WARN("[STRIKE EVAL] Strike altitude: %.2f m", strike_altitude_);
                ROS_WARN("[STRIKE EVAL] Strike time: %.2f s", strike_time_);
                ROS_WARN("[STRIKE EVAL] Success threshold: %.2f m", strike_distance_threshold_);
                ROS_WARN("[STRIKE EVAL] Strike result: %s",
                         (min_strike_distance_ < strike_distance_threshold_) ? "Success" : "Failure");
                ROS_WARN("[STRIKE EVAL] ============================");

                first_strike_evaluated_ = true;
            }
        }
    }

    void guidanceTimerCallback(const ros::TimerEvent&) {
        // SEARCH_ONLY 模式下 guidance 不工作
        if (!is_enabled_) {
            return;
        }

        if (!is_uav_pose_received_ || !is_target_pose_received_) {
            ROS_WARN_THROTTLE(1.0, "[Guidance] Waiting for UAV pose or target data...");
            return;
        }

        // Check if target twist is available (required for intercept)
        if (!is_target_twist_received_ &&
            current_strategy_type_ == multi_uav_strike::GuidanceStrategyType::INTERCEPT) {
            ROS_WARN_THROTTLE(1.0, "[Guidance] Target velocity not available for intercept guidance");
            return;
        }

        // Compute guidance command based on strategy type
        switch (current_strategy_type_) {
            case multi_uav_strike::GuidanceStrategyType::INTERCEPT: {

                // Set flight mode to velocity
                std_msgs::Int16 mode_msg;
                mode_msg.data = flight_mode_velocity_;
                flight_mode_pub_.publish(mode_msg);

                geometry_msgs::Twist vel_cmd;
                if (current_mode_ == "track") {
                    // TRACK模式：独立PI控制，不做STRIKE评估
                    // 固定高度 + PI控制水平位置
                    computeTrackVelocity(vel_cmd);
                    if (!use_sim_) {
                        convertVelNedToEnu(vel_cmd);
                    }
                    vel_cmd_pub_.publish(vel_cmd);
                } else {
                    // STRIKE模式
                    auto* strategy = static_cast<multi_uav_strike::InterceptGuidance*>(guidance_strategy_.get());
                    auto cmd = strategy->computeCommand(
                        current_uav_pose_,
                        current_target_pose_,
                        current_target_twist_);
                    // // 打印目标信息、无人机信息、速度指令信息用于调试
                    // ROS_INFO("[Guidance] Target Pos (NWU): [%.2f, %.2f, %.2f]",
                    //                    current_target_pose_.pose.position.x, current_target_pose_.pose.position.y, current_target_pose_.pose.position.z);
                    // ROS_INFO("[Guidance] UAV Pos (NWU): [%.2f, %.2f, %.2f], UAV Alt: %.2f m",
                    //                    current_uav_pose_.pose.position.x, current_uav_pose_.pose.position.y, current_uav_pose_.pose.position.z, -current_uav_pose_.pose.position.z);
                    // ROS_INFO("[Guidance] Velocity Command (NWU): [%.2f, %.2f, %.2f], Intercept Point (NWU): [%.2f, %.2f, %.2f]",
                    //                    cmd.velocity.x(), cmd.velocity.y(), cmd.velocity.z(),
                    //                    cmd.intercept_point.x(), cmd.intercept_point.y(), cmd.intercept_point.z());
                    // Publish velocity command - 转换为NED坐标发送
                    // NWU (guidance内部) -> NED (飞控期望)
                    // NED: x=North(不变), y=East(取反), z=Down(取反)
                    vel_cmd.linear.x = cmd.velocity.x();   // NED N
                    vel_cmd.linear.y = -cmd.velocity.y();  // NED E = -NWU W

                    // 基于视场角pitch的高度控制
                    // pitch大（绝对值）= 俯冲阶段 = 允许下降
                    // pitch小 = 接近阶段 = 限制下降
                    double pitch = current_los_angle_.y;  // 来自LOS（NED下俯仰角）
                    double pitch_threshold = 0.5f;  // ~0.5rad，小于此值限制下降
                    double current_alt = current_uav_pose_.pose.position.z;

                    if (fabs(pitch) > pitch_threshold) {
                        // pitch大或高度已低，正常下降
                        vel_cmd.linear.z = -cmd.velocity.z();
                    } else {
                        // pitch小且高度还高，限制下沉
                        double down_limit = -0.0;  // 最大下沉 0.1 m/s
                        double desired_z = -cmd.velocity.z();
                        if (desired_z > down_limit) {
                            desired_z = down_limit;
                        }
                        vel_cmd.linear.z = desired_z;
                    }

                    vel_cmd.angular.x = 0.0;
                    vel_cmd.angular.y = 0.0;
                    vel_cmd.angular.z = 0.0;
                    evaluateStrike();
                    if (!use_sim_) {
                        convertVelNedToEnu(vel_cmd);
                    }
                    vel_cmd_pub_.publish(vel_cmd);
                    publishInterceptPointMarker(cmd.intercept_point);
                }
                break;
            }

            case multi_uav_strike::GuidanceStrategyType::MINSNAP: {
                auto* strategy = static_cast<multi_uav_strike::MinSnapGuidance*>(guidance_strategy_.get());
                auto cmd = strategy->computeCommand(
                    current_uav_pose_,
                    current_target_pose_,
                    current_target_twist_);

                publishAttitudeThrust(cmd);
                break;
            }

            case multi_uav_strike::GuidanceStrategyType::LOS: {
                if (!is_los_received_) {
                    ROS_WARN_THROTTLE(1.0, "[Guidance] LOS angle not available for LOS guidance");
                    return;
                }
                auto* strategy = static_cast<multi_uav_strike::LosGuidance*>(guidance_strategy_.get());
                auto cmd = strategy->computeCommand(
                    current_uav_pose_,
                    current_target_pose_,
                    current_target_twist_,
                    current_los_angle_);

                publishAttitudeThrust(cmd);
                break;
            }
        }
    }

    // TRACK模式专用：PI控制速度计算
    void computeTrackVelocity(geometry_msgs::Twist& vel_cmd) {
        double dx = current_target_pose_.pose.position.x - current_uav_pose_.pose.position.x;
        double dy = current_target_pose_.pose.position.y - current_uav_pose_.pose.position.y;
        double dz = current_target_pose_.pose.position.z - current_uav_pose_.pose.position.z;

        double horiz_dist = sqrt(dx*dx + dy*dy);

        // 偏航角：让机头朝向目标
        // current_yaw 这里计算出是 NWU 坐标系的（左转正，从上方看是逆时针，0=North, -π/2=East）
        double current_yaw = atan2(2.0 * (current_uav_pose_.pose.orientation.w * current_uav_pose_.pose.orientation.z +
                                          current_uav_pose_.pose.orientation.x * current_uav_pose_.pose.orientation.y),
                                   1.0 - 2.0 * (current_uav_pose_.pose.orientation.y * current_uav_pose_.pose.orientation.y +
                                                current_uav_pose_.pose.orientation.z * current_uav_pose_.pose.orientation.z));
        // target_bearing 在 NWU 下计算：atan2(dy_west, dx_north)
        // 0=North, +π/2=West, π/-π=South, -π/2=East
        double target_bearing = atan2(dy, dx);
        double yaw_error = target_bearing - current_yaw;
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));

        // 偏航角控制：LOS 角度来自 gimbal_simulator（NED 坐标系：从上方看顺时针，0=North, +π/2=East）
        // 与 NWU 系的 current_yaw 符号相反（NED:北偏东为正, NWU:北偏西为正），
        // 需转换：yaw_nwu = -yaw_ned，统一使用 NWU 约定避免 180° 误转。
        double desired_yaw;
        if (is_los_received_) {
            desired_yaw = -current_los_angle_.x;  // NED → NWU
        } else {
            desired_yaw = target_bearing;
        }

        // PI控制：水平速度根据位置偏差计算
        double cmd_x = track_P_ * dx + track_I_ * track_integral_x_;
        double cmd_y = track_P_ * dy + track_I_ * track_integral_y_;

        // 速度限幅
        double speed = sqrt(cmd_x*cmd_x + cmd_y*cmd_y);
        if (speed > track_max_speed_) {
            cmd_x = cmd_x / speed * track_max_speed_;
            cmd_y = cmd_y / speed * track_max_speed_;
        }

        // 距离→速度的连续映射（消除内/外边界的硬切换）：
        //   - 距离 > stand-off：按 track_k_approach_ * (horiz_dist - stand-off) 给速度
        //   - 距离 <= stand-off：速度按相同斜率线性降到 0
        //   - 上限为 track_max_speed_
        // 这样靠近目标时速度自然收敛到 0，远离时单调增大，没有突变，姿态变化平滑。
        double approach_speed = std::max(0.0, track_k_approach_ * (horiz_dist - track_horiz_dist_));
        double track_speed_cap = std::min(approach_speed, track_max_speed_);

        // 用连续限幅替换原来的硬限幅，保证靠近时速度真正降到 0
        double speed_xy = sqrt(cmd_x*cmd_x + cmd_y*cmd_y);
        if (speed_xy > track_speed_cap) {
            cmd_x = cmd_x / speed_xy * track_speed_cap;
            cmd_y = cmd_y / speed_xy * track_speed_cap;
        } else if (horiz_dist <= track_horiz_dist_) {
            // 进入 stand-off 范围：PI 输出按当前速度限幅比例线性衰减，避免姿态突变
            double scale = (speed_xy > 1e-3) ? (track_speed_cap / speed_xy) : 0.0;
            cmd_x *= scale;
            cmd_y *= scale;
        }

        // 积分项：远离目标时累加，进入 stand-off 后清零
        if (horiz_dist > track_horiz_dist_) {
            track_integral_x_ += dx * 0.02;
            track_integral_y_ += dy * 0.02;
        } else {
            track_integral_x_ = 0.0;
            track_integral_y_ = 0.0;
        }
        track_integral_x_ = std::max(-5.0, std::min(5.0, track_integral_x_));
        track_integral_y_ = std::max(-5.0, std::min(5.0, track_integral_y_));

        // 航向：进入 stand-off 时不再强制保持 current_yaw，让 LOS 角度平滑过渡
        // （之前在 inner_threshold 内强制 desired_yaw = current_yaw 会导致出/入 stand-off
        //  时航向角速度突然反向，是姿态大角度跳变的另一个来源）
        if (horiz_dist <= track_horiz_dist_) {
            // 缓出：把航向也向 LOS 方向拉一点点，避免悬停时一直锁住航向
            double los_yaw_nwu = -current_los_angle_.x;
            double blend = 0.2 * (horiz_dist / std::max(track_horiz_dist_, 1e-3));
            desired_yaw = current_yaw * (1.0 - blend) + los_yaw_nwu * blend;
        }

        // 高度控制：固定高度 track_altitude_
        // NED: z>0向下，z<0向上
        double alt_error = current_uav_pose_.pose.position.z - track_altitude_;
        double cmd_z = std::max(-1.0, std::min(1.0, -0.5 * alt_error));  // P控制，高度低了向上(z<0)

        // 机间避障
        applyInterUavAvoidance(cmd_x, cmd_y, cmd_z);

        vel_cmd.linear.x = cmd_x;
        vel_cmd.linear.y = -cmd_y;  // NED E = -NWU W
        vel_cmd.linear.z = -cmd_z;   // NED D (z<0=up)

        // 偏航角速度控制：PX4 SITL 通过 mavros 的 angular.z 是角速度（rad/s），不是角度
        // desired_yaw/current_yaw 在 NWU 坐标系下（左转为正）
        // 计算角度误差并归一化到 [-pi, pi]
        double yaw_err_ctrl = desired_yaw - current_yaw;
        yaw_err_ctrl = atan2(sin(yaw_err_ctrl), cos(yaw_err_ctrl));

        // P 控制：角速度 = Kp * 角度误差
        double yaw_rate = los_kp_yaw_ * yaw_err_ctrl;

        // 角速度限幅
        yaw_rate = std::max(-los_max_rate_, std::min(los_max_rate_, yaw_rate));

        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        // ===========================================================================
        // 偏航角速度符号约定（避免再出现 +los_max_rate / -los_max_rate 跳变）：
        //
        //   内部约定 (NWU):    yaw_rate > 0  → 左转 (CCW from above)
        //   mavros 约定 (FRD): angular.z > 0 → 右转 (CW  from above)
        //   → 两者符号相反，需要在送 mavros 前取反。
        //
        //   这个取反操作由调用本函数后紧接着执行的 convertVelNedToEnu() 完成
        //   （见 line ~719: vel_cmd.angular.z = -ned_yaw;），所以这里必须直接
        //   写入 NWU 约定的 yaw_rate，绝不能再取反。
        //
        // 历史 bug：原代码写的是 vel_cmd.angular.z = -yaw_rate;，与 convertVelNedToEnu
        //   中的 -ned_yaw 形成双重取反，最终送到 mavros 的是 +yaw_rate。mavros 按 FRD
        //   约定解释为"右转"，但我们的内部逻辑是"需要左转"，于是 PX4 持续向错误方向
        //   打舵，yaw_error 累积至 ±π，los_kp_yaw_ * yaw_err 被限幅到 ±los_max_rate_
        //   之间反复跳变，外观就是航向在最大角速度处来回抖。
        //
        // 修正：去掉这里的负号，让 convertVelNedToEnu 单独完成 NWU→FRD 转换。
        // 验证：去掉负号后目标在北方时航向不再进入 ±max_rate 跳变，跟踪稳定。
        // ===========================================================================
        vel_cmd.angular.z = yaw_rate;

        // ROS_WARN_THROTTLE(0.5, "[Guidance-TRACK] dx=%.1f dy=%.1f horiz=%.1f alt=%.1f des_alt=%.1f vel(%.2f,%.2f,%.2f) yaw=%.0f",
        //                  dx, dy, horiz_dist, current_uav_pose_.pose.position.z, track_altitude_,
        //                  vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z,
        //                  desired_yaw * 180.0 / M_PI);
    }

    /**
     * 将 NED 速度指令转换为 ENU（用于 PX4 SITL）
     * NED: x=North, y=East, z=Down
     * ENU: x=East, y=North, z=Up
     */
    void convertVelNedToEnu(geometry_msgs::Twist& vel_cmd) {
        double ned_vx = vel_cmd.linear.x;
        double ned_vy = vel_cmd.linear.y;
        double ned_vz = vel_cmd.linear.z;
        double ned_yaw = vel_cmd.angular.z;

        // NED -> ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
        vel_cmd.linear.x = ned_vy;       // ENU East = NED East
        vel_cmd.linear.y = ned_vx;       // ENU North = NED North
        vel_cmd.linear.z = -ned_vz;      // ENU Up = -NED Down

        // Yaw: NED heading -> ENU heading
        // NED: 0=North, PI/2=East; ENU: 0=East, PI/2=North
        // yaw_enu = yaw_ned - PI/2, 但 PX4 期望的是 body frame rate
        // 这里简单取负值，与位置转换保持一致
        vel_cmd.angular.z = -ned_yaw;
    }

    void publishAttitudeThrust(const multi_uav_strike::AttitudeThrustCommand& cmd) {
        // Publish attitude (NWU → NED转换: w,x不变, y,z取反)
        geometry_msgs::PoseStamped att_cmd;
        att_cmd.header.stamp = ros::Time::now();
        att_cmd.header.frame_id = "map";
        att_cmd.pose.position.x = 0.0;
        att_cmd.pose.position.y = 0.0;
        att_cmd.pose.position.z = 0.0;
        att_cmd.pose.orientation.w = cmd.attitude.w();
        att_cmd.pose.orientation.x = cmd.attitude.x();
        att_cmd.pose.orientation.y = -cmd.attitude.y();  // NWU -> NED
        att_cmd.pose.orientation.z = -cmd.attitude.z();  // NWU -> NED
        attitude_cmd_pub_.publish(att_cmd);

        // Publish thrust
        std_msgs::Float32 thrust_cmd;
        thrust_cmd.data = cmd.thrust;
        thrust_cmd_pub_.publish(thrust_cmd);

        // Set flight mode to attitude
        std_msgs::Int16 mode_msg;
        mode_msg.data = flight_mode_attitude_;
        flight_mode_pub_.publish(mode_msg);

        // Publish angular velocity (yaw_rate) via setpoint_attitude/cmd_vel
        geometry_msgs::TwistStamped vel_cmd;
        vel_cmd.header.stamp = ros::Time::now();
        vel_cmd.header.frame_id = "map";
        vel_cmd.twist.linear.x = 0.0;
        vel_cmd.twist.linear.y = 0.0;
        vel_cmd.twist.linear.z = 0.0;
        vel_cmd.twist.angular.x = 0.0;
        vel_cmd.twist.angular.y = 0.0;
        vel_cmd.twist.angular.z = cmd.yaw_rate;  // 偏航角速率
        attitude_rates_pub_.publish(vel_cmd);

        // 发布推力方向可视化（用于RViz显示）
        publishThrustDirectionMarker(cmd);
    }

    // 发布推力方向可视化 marker
    void publishThrustDirectionMarker(const multi_uav_strike::AttitudeThrustCommand& cmd) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "thrust_direction";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // 箭头起点：无人机位置
        marker.points.resize(2);
        marker.points[0].x = current_uav_pose_.pose.position.x;
        marker.points[0].y = current_uav_pose_.pose.position.y;
        marker.points[0].z = current_uav_pose_.pose.position.z;

        // 箭头终点：根据姿态和推力计算方向
        Eigen::Quaterniond quat(cmd.attitude.w(), cmd.attitude.x(), cmd.attitude.y(), cmd.attitude.z());
        Eigen::Vector3d thrust_dir = quat * Eigen::Vector3d(0, 0, 1);  // 机体系Z轴为推力方向
        double arrow_length = 3.0;  // 箭头长度3米
        marker.points[1].x = marker.points[0].x + thrust_dir.x() * arrow_length;
        marker.points[1].y = marker.points[0].y + thrust_dir.y() * arrow_length;
        marker.points[1].z = marker.points[0].z + thrust_dir.z() * arrow_length;

        // 推力越大箭头越粗
        marker.scale.x = 0.1 + cmd.thrust * 0.3;  // 直径0.1~0.4m
        marker.scale.y = 0.2 + cmd.thrust * 0.6;   // 头部宽度0.2~0.8m
        marker.scale.z = 0.0;

        // 颜色：推力越大越红
        marker.color.r = 0.5 + cmd.thrust * 0.5;
        marker.color.g = 0.5 - cmd.thrust * 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0.1);
        thrust_dir_pub_.publish(marker);
    }

    // 发布拦截点可视化 marker
    void publishInterceptPointMarker(const Eigen::Vector3d& intercept_point) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "intercept_point";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = intercept_point.x();
        marker.pose.position.y = intercept_point.y();
        marker.pose.position.z = intercept_point.z();
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.scale.x = 1.0;  // 球直径1米
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.1);
        intercept_point_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "guidance_control_node");
    GuidanceControlNode node;
    ros::spin();
    return 0;
}