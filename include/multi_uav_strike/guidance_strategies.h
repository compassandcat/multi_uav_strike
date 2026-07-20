#ifndef GUIDANCE_STRATEGIES_H
#define GUIDANCE_STRATEGIES_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <memory>

namespace multi_uav_strike {

// Enum for strategy selection
enum class GuidanceStrategyType {
    INTERCEPT = 0,
    MINSNAP = 1,
    LOS = 2
};

// Velocity command output (for Intercept strategy)
struct VelocityCommand {
    Eigen::Vector3d velocity;
    Eigen::Vector3d intercept_point;  // 拦截点位置(NWU)
};

// Attitude + Thrust command output (for MinSnap and LOS strategies)
struct AttitudeThrustCommand {
    Eigen::Quaterniond attitude;
    double thrust;
    double yaw_rate;  // 偏航角速率（用于LOS控制）

    AttitudeThrustCommand() : thrust(0.0), yaw_rate(0.0) {}
};

// Base class for all guidance strategies
class GuidanceStrategy {
public:
    virtual ~GuidanceStrategy() = default;

    // Get strategy name
    const std::string& getName() const { return name_; }

    // Get strategy type
    virtual GuidanceStrategyType getType() const = 0;

    // 允许 mission_manager 运行时下发拦截速度(目前仅 Intercept 关心,默认 no-op)
    virtual void setUavSpeed(double /*speed*/) {}

protected:
    GuidanceStrategy(ros::NodeHandle& nh, const std::string& name)
        : nh_(nh), name_(name) {}

    ros::NodeHandle& nh_;
    std::string name_;
};

// Intercept Guidance: Predicts intercept point and generates velocity command
class InterceptGuidance : public GuidanceStrategy {
public:
    InterceptGuidance(ros::NodeHandle& nh);

    GuidanceStrategyType getType() const override { return GuidanceStrategyType::INTERCEPT; }

    // mission_manager 运行时下发拦截速度
    void setUavSpeed(double speed) override { uav_speed_ = speed; }

    VelocityCommand computeCommand(
        const geometry_msgs::PoseStamped& uav_pose,
        const geometry_msgs::PoseStamped& target_pose,
        const geometry_msgs::TwistStamped& target_twist);

private:
    double uav_speed_;
    double intercept_time_horizon_;

    // Compute intercept point
    Eigen::Vector3d computeInterceptPoint(
        const Eigen::Vector3d& uav_pos,
        const Eigen::Vector3d& target_pos,
        const Eigen::Vector3d& target_vel);
};

// Min-Snap Trajectory Guidance: Generates minimum-snap trajectories
class MinSnapGuidance : public GuidanceStrategy {
public:
    MinSnapGuidance(ros::NodeHandle& nh);
    ~MinSnapGuidance();

    GuidanceStrategyType getType() const override { return GuidanceStrategyType::MINSNAP; }

    AttitudeThrustCommand computeCommand(
        const geometry_msgs::PoseStamped& uav_pose,
        const geometry_msgs::PoseStamped& target_pose,
        const geometry_msgs::TwistStamped& target_twist);

    void updateTrajectory();

private:
    // Trajectory generation parameters
    double v_max_;
    double a_max_;
    int polynomial_order_;
    double sampling_interval_;
    double trajectory_lookahead_;

    // Tracking controller params
    double pos_p_gain_;
    double vel_p_gain_;
    double thrust_weight_;

    // Current trajectory state
    double trajectory_start_time_;
    bool trajectory_valid_;

    // Helper: compute attitude + thrust from acceleration
    AttitudeThrustCommand computeAttitudeThrust(
        const Eigen::Vector3d& desired_acceleration,
        const Eigen::Vector3d& position_error,
        const Eigen::Vector3d& velocity_error);

    // Generate trajectory to target
    bool generateTrajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& start_vel,
        const Eigen::Vector3d& goal_pos,
        const Eigen::Vector3d& goal_vel);

    // Sample current trajectory
    bool sampleTrajectory(double time_since_start,
                          Eigen::Vector3d& pos,
                          Eigen::Vector3d& vel,
                          Eigen::Vector3d& acc);
};

// LOS Guidance: Uses line-of-sight angle error for guidance
class LosGuidance : public GuidanceStrategy {
public:
    LosGuidance(ros::NodeHandle& nh);

    GuidanceStrategyType getType() const override { return GuidanceStrategyType::LOS; }

    AttitudeThrustCommand computeCommand(
        const geometry_msgs::PoseStamped& uav_pose,
        const geometry_msgs::PoseStamped& target_pose,
        const geometry_msgs::TwistStamped& target_twist,
        const geometry_msgs::Point& los_angle);

private:
    // LOS control params
    double los_kp_yaw_;
    double los_kp_pitch_;
    double los_max_rate_;
    double thrust_weight_;

    // Integrator state for yaw/pitch
    double integrated_yaw_error_;
    double integrated_pitch_error_;

    // Helper: compute attitude + thrust
    AttitudeThrustCommand computeAttitudeThrust(
        double desired_roll,
        double desired_pitch,
        double desired_yaw);
};

// Factory function to create strategies
std::unique_ptr<GuidanceStrategy> createGuidanceStrategy(
    GuidanceStrategyType type,
    ros::NodeHandle& nh);

} // namespace multi_uav_strike

#endif // GUIDANCE_STRATEGIES_H