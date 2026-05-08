#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h> // 用于RPY转四元数
#include <cmath>
#include <algorithm> // 用于max/min（替代clamp）
#include <random>       // 随机数相关
#include <chrono>       // 时间种子相关

// 云台模拟器类
class GimbalSimulator {
private:
    // ROS核心组件
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;       // 订阅目标位置
    ros::Subscriber uav_pose_sub_;     // 订阅无人机位姿
    ros::Publisher los_angle_pub_;     // 发布视场角（方位/俯仰）
    ros::Publisher gimbal_pose_pub_;   // 发布云台位姿（RViz显示）
    ros::Timer control_timer_;         // 云台控制主定时器（100Hz）

    // 配置参数（通过param获取）
    int image_width_;                  // 图像宽度，默认1920
    int image_height_;                 // 图像高度，默认1080
    double fov_deg_;                   // 视场角（度），默认45
    double fov_rad_;                   // 视场角（弧度），内部转换
    double gimbal_p_gain_;             // 云台比例控制增益，默认0.5
    double max_yaw_rate_;              // 最大偏航角速度（rad/s），默认1.0
    double max_pitch_rate_;            // 最大俯仰角速度（rad/s），默认0.8
    std::string uav_pose_topic_;       // 无人机pose话题名，默认"/uav_pose"
    double loop_freq_;                 // 控制循环频率，默认100Hz

    // 图像识别噪声参数
    double image_noise_std_dev_;        // 噪声标准差（单位：m，控制抖动幅度，作为ROS参数输入）
    std::default_random_engine rng_;    // 随机数引擎
    std::normal_distribution<double> noise_dist_; // 高斯噪声分布（均值0，标准差由参数指定）
    double gimbal_tracking_accuracy_ = 0.0;    // 云台跟踪精度（用于计算识别可信度）

    // 云台安装位置（NED机体坐标，固定[0.2, 0, 0]）
    const double gimbal_mount_x_ = 0.2;
    const double gimbal_mount_y_ = 0.0;
    const double gimbal_mount_z_ = 0.0;

    // 状态变量
    geometry_msgs::Point current_target_pos_;     // 当前目标位置（x/y=坐标，z=theta）
    geometry_msgs::PoseStamped current_uav_pose_; // 当前无人机位姿
    bool is_target_received_ = false;             // 是否收到目标数据
    bool is_uav_pose_received_ = false;           // 是否收到无人机位姿数据
    double current_gimbal_yaw_ = 0.0;             // 云台当前偏航角（rad，NED北偏东）
    double current_gimbal_pitch_ = 0.0;           // 云台当前俯仰角（rad，NED向下为正）

public:
    // 构造函数：初始化参数、订阅/发布者、定时器
    GimbalSimulator() {
        ros::NodeHandle n_param("~");
        // 1. 读取ROS参数（带默认值）
        n_param.param<int>("image_width", image_width_, 1920);
        n_param.param<int>("image_height", image_height_, 1080);
        n_param.param<double>("fov_deg", fov_deg_, 45.0);
        n_param.param<double>("gimbal_p_gain", gimbal_p_gain_, 0.8);
        n_param.param<double>("max_yaw_rate", max_yaw_rate_, 1.2);
        n_param.param<double>("max_pitch_rate", max_pitch_rate_, 1.2);
        // nh_.param<std::string>("uav_pose_topic", uav_pose_topic_, "/uav_pose");
        n_param.param<double>("loop_freq", loop_freq_, 100.0);
        // 读取噪声参数（默认无噪声）
        n_param.param<double>("image_noise_std_dev", image_noise_std_dev_, 0.0);

        // 初始化随机数引擎（用系统时间做种子，保证每次运行噪声不同）
        rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        // 初始化高斯噪声分布（均值0，标准差为配置的参数）
        noise_dist_ = std::normal_distribution<double>(0.0, image_noise_std_dev_);

        // 2. 视场角转换为弧度
        fov_rad_ = fov_deg_ * M_PI / 180.0;

        // 3. 创建订阅者
        target_sub_ = nh_.subscribe("/target_position", 10, &GimbalSimulator::targetCallback, this);
        uav_pose_sub_ = nh_.subscribe("quad/pose", 10, &GimbalSimulator::uavPoseCallback, this);

        // 4. 创建发布者
        los_angle_pub_ = nh_.advertise<geometry_msgs::Point>("target_los_angle", 10);
        gimbal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("gimbal_pose", 10);

        // 5. 创建控制定时器（100Hz）
        control_timer_ = nh_.createTimer(ros::Duration(1.0/loop_freq_), &GimbalSimulator::controlLoopCallback, this);

        // ========== 修改1：修正ROS_INFO的变量名笔误 ==========
        ROS_INFO("Params: image=%dx%d, FOV=%.1f deg, P-gain=%.2f, max_yaw_rate=%.2f rad/s, max_pitch_rate=%.2f rad/s",
                 image_width_, image_height_, fov_deg_, gimbal_p_gain_, max_yaw_rate_, max_pitch_rate_);
        ROS_INFO("Subscribed to: target=%s, uav_pose=quad/pose", "/target_position");
        ROS_INFO("Publishing to: los_angle=%s, gimbal_pose=%s", "/target_los_angle", "/gimbal_pose");
    }

    // 目标位置回调函数
    void targetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        // 目标位置：target_motion_simulator_node发布的是NED坐标
        // 但无人机也在NED坐标，我们需要计算相对位置
        // 因此直接使用msg（不转换坐标系）
        current_target_pos_ = *msg;
        is_target_received_ = true;
    }

    // 无人机位姿回调函数
    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // ===== NED → NWU 坐标转换 =====
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

    // 云台控制主循环（100Hz）
    void controlLoopCallback(const ros::TimerEvent&) {
        // 检查是否收到目标和无人机位姿
        if (!is_target_received_ || !is_uav_pose_received_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for target or UAV pose data...");
            return;
        }

        // 1. 计算目标相对于无人机的期望云台角度（偏航+俯仰）
        double desired_yaw, desired_pitch;
        calculateDesiredGimbalAngles(desired_yaw, desired_pitch);

        // 2. 云台比例控制（带速度限制，模拟滞后）
        updateGimbalAngles(desired_yaw, desired_pitch);

        // 3. 发布视场角消息（发布期望视线角，z为跟踪精度）
        publishLOSAngle(desired_yaw, desired_pitch);

        // 4. 发布云台位姿（用于RViz显示）
        publishGimbalPose();
    }

    // 核心：计算目标相对于无人机的期望云台角度（NED坐标系）
    void calculateDesiredGimbalAngles(double& desired_yaw, double& desired_pitch) {
        // 无人机位置（NED）
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;

        // 目标位置（NED，x/y为坐标，z无用）
        double target_x = current_target_pos_.x;
        double target_y = current_target_pos_.y;
        double target_z = 0.0; // 二维目标，z=0
        // ========== 新增：添加图像识别噪声 ==========
        if (image_noise_std_dev_ > 0.0) { // 仅当噪声参数>0时生效
            target_x += noise_dist_(rng_); // x方向加高斯噪声
            target_y += noise_dist_(rng_); // y方向加高斯噪声
        }

        // 云台安装位置（机体NED→转换到全局NED）
        // 简化：假设无人机机体朝向与全局NED一致（无机体偏航），实际可加机体姿态转换，这里先简化
        double gimbal_mount_x_global = uav_x + gimbal_mount_x_;
        double gimbal_mount_y_global = uav_y + gimbal_mount_y_;
        double gimbal_mount_z_global = uav_z + gimbal_mount_z_;

        // 目标相对于云台安装点的相对位置（NED）
        double dx = target_x - gimbal_mount_x_global;
        double dy = target_y - gimbal_mount_y_global;
        double dz = target_z - gimbal_mount_z_global;

        // 计算期望偏航角（方位角：北偏东，rad）
        desired_yaw = atan2(dy, dx);

        // 计算期望俯仰角（向下为正，rad）
        double horizontal_dist = sqrt(dx*dx + dy*dy);
        desired_pitch = atan2(dz, horizontal_dist);

        // 角度归一化到[-π, π]
        desired_yaw = atan2(sin(desired_yaw), cos(desired_yaw));
        desired_pitch = atan2(sin(desired_pitch), cos(desired_pitch));
    }

    // 云台比例控制器：跟踪期望角度，带最大角速度限制（模拟滞后）
    void updateGimbalAngles(double desired_yaw, double desired_pitch) {
        double dt = 1.0 / loop_freq_;

        // 1. 偏航控制
        double yaw_error = desired_yaw - current_gimbal_yaw_;
        // 误差归一化（避免跨±π的大误差）
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));
        // 比例控制计算期望角速度
        double desired_yaw_rate = gimbal_p_gain_ * yaw_error;
        // ========== 修改2：替换std::clamp为C++11兼容的限幅方式 ==========
        desired_yaw_rate = std::max(-max_yaw_rate_, std::min(desired_yaw_rate, max_yaw_rate_));
        // 更新当前偏航角
        current_gimbal_yaw_ += desired_yaw_rate * dt;
        current_gimbal_yaw_ = atan2(sin(current_gimbal_yaw_), cos(current_gimbal_yaw_));

        // 2. 俯仰控制（逻辑同偏航）
        double pitch_error = desired_pitch - current_gimbal_pitch_;
        pitch_error = atan2(sin(pitch_error), cos(pitch_error));
        double desired_pitch_rate = gimbal_p_gain_ * pitch_error;
        // ========== 修改3：替换std::clamp为C++11兼容的限幅方式 ==========
        desired_pitch_rate = std::max(-max_pitch_rate_, std::min(desired_pitch_rate, max_pitch_rate_));
        current_gimbal_pitch_ += desired_pitch_rate * dt;
        current_gimbal_pitch_ = atan2(sin(current_gimbal_pitch_), cos(current_gimbal_pitch_));
        // 根据角度误差计算识别可信度（表征图像畸变程度）
        double angular_error = sqrt(yaw_error*yaw_error + pitch_error*pitch_error);
        gimbal_tracking_accuracy_ = exp(-angular_error);  // 用指数衰减，误差越大精度越低
    }

    // 发布视场角消息（发布的是期望视线角，不是云台实际朝向）
    void publishLOSAngle(double desired_yaw, double desired_pitch) {
        geometry_msgs::Point los_angle_msg;
        los_angle_msg.x = desired_yaw;     // 期望方位角（真实LOS，rad）
        los_angle_msg.y = desired_pitch;   // 期望俯仰角（保持计算值的符号）
        // 跟踪精度：基于云台与目标的角度偏差（图像畸变/脱靶量）
        if (std::isnan(gimbal_tracking_accuracy_)) {
            gimbal_tracking_accuracy_ = 0.5;  // 默认中等置信度
        }
        los_angle_msg.z = gimbal_tracking_accuracy_;  // 云台跟踪精度
        los_angle_pub_.publish(los_angle_msg);
    }

    // 发布云台位姿（用于RViz显示：无人机位置+云台姿态四元数）
    void publishGimbalPose() {
        geometry_msgs::PoseStamped gimbal_pose_msg;

        gimbal_pose_msg.header.stamp = ros::Time::now();
        gimbal_pose_msg.header.frame_id = "map";

        // 位置：无人机位置（云台安装点简化为无人机位置，可扩展为安装点全局坐标）
        gimbal_pose_msg.pose.position = current_uav_pose_.pose.position;

        // 姿态：云台偏航+俯仰，横滚=0 → 转换为四元数
        tf::Quaternion gimbal_quat;
        gimbal_quat.setRPY(0.0, -current_gimbal_pitch_, current_gimbal_yaw_); // RPY顺序：滚转、俯仰、偏航
        // 转换为geometry_msgs::Quaternion
        gimbal_pose_msg.pose.orientation.x = gimbal_quat.x();
        gimbal_pose_msg.pose.orientation.y = gimbal_quat.y();
        gimbal_pose_msg.pose.orientation.z = gimbal_quat.z();
        gimbal_pose_msg.pose.orientation.w = gimbal_quat.w();

        gimbal_pose_pub_.publish(gimbal_pose_msg);
    }
};

// 主函数
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "gimbal_simulator_node");

    // 创建云台模拟器实例
    GimbalSimulator gimbal_sim;

    // 自旋
    ros::spin();

    return 0;
}