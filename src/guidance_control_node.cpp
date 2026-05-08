#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
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

    // 跟踪约束参数
    double min_altitude_;            // 最小高度限制
    double max_tracking_distance_;   // 最大跟踪距离
    double desired_tracking_angle_;   // 期望跟踪角度（目标在机头前下方）

    // TRACK模式专用参数
    double track_altitude_;          // 固定跟踪高度（m）
    double track_horiz_dist_;        // 水平安全距离（m，小于此值水平速度归0）
    double track_P_;                 // P控制增益
    double track_I_;                 // I控制增益
    double track_max_speed_;         // 最大速度限制（m/s）
    double track_integral_x_;        // PI积分项 x
    double track_integral_y_;        // PI积分项 y

    // 打击评估相关
    double strike_distance_threshold_;  // 打击成功距离阈值(m)
    bool first_strike_evaluated_;      // 是否已评估第一次打击
    double min_strike_distance_;       // 第一次打击最小距离
    double strike_altitude_;            // 打击时无人机高度
    double strike_time_;                // 打击时间戳
    bool is_approaching_;               // 是否正在接近目标
    double prev_distance_;              // 上一时刻距离

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
        track_integral_y_(0.0) {
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
    }

    void initSubscribers() {
        target_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &GuidanceControlNode::targetPoseCallback, this);

        target_twist_sub_ = nh_.subscribe(
            "target_estimated_twist", 10,
            &GuidanceControlNode::targetTwistCallback, this);

        uav_pose_sub_ = nh_.subscribe(
            "quad/pose", 10,
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
    }

    void initPublishers() {
        vel_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(
            "quad/setpoint_velocity/cmd_vel_unstamped", 10);

        attitude_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "quad/setpoint_attitude/attitude", 10);

        attitude_rates_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
            "quad/setpoint_attitude/cmd_vel", 10);

        thrust_cmd_pub_ = nh_.advertise<std_msgs::Float32>(
            "quad/thrust", 10);

        flight_mode_pub_ = nh_.advertise<std_msgs::Int16>(
            "quad/flight_mode", 10);

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
                    vel_cmd_pub_.publish(vel_cmd);
                } else {
                    // STRIKE模式
                    auto* strategy = static_cast<multi_uav_strike::InterceptGuidance*>(guidance_strategy_.get());
                    auto cmd = strategy->computeCommand(
                        current_uav_pose_,
                        current_target_pose_,
                        current_target_twist_);
                    // Publish velocity command - 转换为NED坐标发送
                    // NWU (guidance内部) -> NED (飞控期望)
                    // NED: x=North(不变), y=East(取反), z=Down(取反)
                    vel_cmd.linear.x = cmd.velocity.x();   // NED N
                    vel_cmd.linear.y = -cmd.velocity.y();  // NED E = -NWU W
                    vel_cmd.linear.z = -cmd.velocity.z();  // NED D = -NWU U (z<0=向上)
                    vel_cmd.angular.x = 0.0;
                    vel_cmd.angular.y = 0.0;
                    vel_cmd.angular.z = 0.0;
                    evaluateStrike();
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
        // current_yaw这里计算出是NWU坐标系的，左转是正
        double current_yaw = atan2(2.0 * (current_uav_pose_.pose.orientation.w * current_uav_pose_.pose.orientation.z +
                                          current_uav_pose_.pose.orientation.x * current_uav_pose_.pose.orientation.y),
                                   1.0 - 2.0 * (current_uav_pose_.pose.orientation.y * current_uav_pose_.pose.orientation.y +
                                                current_uav_pose_.pose.orientation.z * current_uav_pose_.pose.orientation.z));
        double target_bearing = atan2(dy, dx);
        double yaw_error = target_bearing - current_yaw;
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));

        // 偏航角控制：los是全局角度（NED），直接作为期望航向
        // current_los_angle_.x 是目标在NED下的绝对方位角，不是相对机头的偏移
        double desired_yaw;
        if (is_los_received_) {
            desired_yaw = current_los_angle_.x;  // 直接使用全局LOS角度
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

        // 缓冲带：防止在threshold附近航向跳变
        const double HYSTERESIS_MARGIN = 2.0;  // 缓冲带宽度
        double inner_threshold = track_horiz_dist_ - HYSTERESIS_MARGIN;
        double outer_threshold = track_horiz_dist_ + HYSTERESIS_MARGIN;

        if (horiz_dist < inner_threshold) {
            // 在内边界内，速度归0，航向保持当前朝向
            cmd_x = 0.0;
            cmd_y = 0.0;
            track_integral_x_ = 0.0;
            track_integral_y_ = 0.0;
            desired_yaw = current_yaw;  // 保持当前航向，不跳变
        } else if (horiz_dist < outer_threshold) {
            // 在缓冲带内，逐渐过渡航向，避免跳变
            double ratio = (horiz_dist - inner_threshold) / (2.0 * HYSTERESIS_MARGIN);
            ratio = std::max(0.0, std::min(1.0, ratio));
            // los_yaw是全局角度（NED），不是相对机头的偏移
            double los_yaw = current_los_angle_.x;
            desired_yaw = current_yaw * (1.0 - ratio) + los_yaw * ratio;
        } else {
            // 在外边界外，正常跟踪
            track_integral_x_ += dx * 0.02;
            track_integral_y_ += dy * 0.02;
            track_integral_x_ = std::max(-5.0, std::min(5.0, track_integral_x_));
            track_integral_y_ = std::max(-5.0, std::min(5.0, track_integral_y_));
        }

        // 高度控制：固定高度 track_altitude_
        // NED: z>0向下，z<0向上
        double alt_error = current_uav_pose_.pose.position.z - track_altitude_;
        double cmd_z = std::max(-1.0, std::min(1.0, -0.5 * alt_error));  // P控制，高度低了向上(z<0)

        vel_cmd.linear.x = cmd_x;
        vel_cmd.linear.y = -cmd_y;  // NED E = -NWU W
        vel_cmd.linear.z = -cmd_z;   // NED D (z<0=up)
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = -desired_yaw;

        // ROS_WARN_THROTTLE(0.5, "[Guidance-TRACK] dx=%.1f dy=%.1f horiz=%.1f alt=%.1f des_alt=%.1f vel(%.2f,%.2f,%.2f) yaw=%.0f",
        //                  dx, dy, horiz_dist, current_uav_pose_.pose.position.z, track_altitude_,
        //                  vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z,
        //                  desired_yaw * 180.0 / M_PI);
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