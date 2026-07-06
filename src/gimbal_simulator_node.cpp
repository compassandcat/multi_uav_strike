#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h> // 用于RPY转四元数
#include <cmath>
#include <algorithm> // 用于max/min（替代clamp）
#include <random>       // 随机数相关
#include <chrono>       // 时间种子相关

// 云台模拟器类 — Phase 1 改造:
// - /target_los_angle 维持语义:发的是"目标 LOS"(几何,不跟随机体),用户确认不可改
// - /gimbal_pose 用于 RViz 显示当前 gimbal 跟踪姿态(沿用原语义)
// - 新增 /camera/pose:相机在世界系的"实际朝向"= UAV 航向 + cam_yaw_offset + cam_pitch
//   (这是 detection_simulator_node 用来判定矩形 FOV 可视性的输入)
// - cam_yaw_offset_deg/cam_pitch_deg 由 launch param 配置,默认 yaw=0(正前方),pitch=-90(垂直下视)
class GimbalSimulator {
private:
    // ROS核心组件
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;       // 订阅目标位置
    ros::Subscriber uav_pose_sub_;     // 订阅无人机位姿
    ros::Publisher los_angle_pub_;     // 发布视场角（方位/俯仰）— 目标 LOS（保持）
    ros::Publisher gimbal_pose_pub_;   // 发布云台跟踪位姿（RViz）— 保持语义
    ros::Publisher camera_pose_pub_;   // 发布相机在世界系下的"实际朝向"(新增,Phase 1)
    ros::Timer control_timer_;         // 云台控制主定时器（100Hz）

    // 配置参数（通过param获取）
    int image_width_;                  // 图像宽度，默认1920
    int image_height_;                 // 图像高度，默认1080
    double fov_deg_;                   // 视场角（度），默认45
    double fov_rad_;                   // 视场角（弧度），内部转换
    double gimbal_p_gain_;             // 云台比例控制增益，默认0.5
    double max_yaw_rate_;              // 最大偏航角速度（rad/s），默认1.0
    double max_pitch_rate_;            // 最大俯仰角速度（rad/s），默认0.8
    std::string uav_pose_topic_;       // 无人机pose话题名
    double loop_freq_;                 // 控制循环频率，默认100Hz

    // 仿真/真机切换
    bool use_sim_;

    // 图像识别噪声参数
    double image_noise_std_dev_;        // 噪声标准差（单位：m，控制抖动幅度，作为ROS参数输入）
    std::default_random_engine rng_;
    std::normal_distribution<double> noise_dist_;
    double gimbal_tracking_accuracy_ = 0.0;

    // 云台安装位置（NED机体坐标，固定[0.2, 0, 0]）
    const double gimbal_mount_x_ = 0.2;
    const double gimbal_mount_y_ = 0.0;
    const double gimbal_mount_z_ = 0.0;

    // === 新增:相机相对机体的偏角参数 ===
    // cam_yaw_offset_deg:相机在机体坐标系内的偏航偏差(度)
    //                    0 表示与机体正前方一致
    // cam_pitch_deg:    相机在机体坐标系内的俯仰(度)
    //                    0=水平,-90=垂直下视
    double cam_yaw_offset_deg_;
    double cam_pitch_deg_;

    // 状态变量
    geometry_msgs::Point current_target_pos_;
    geometry_msgs::PoseStamped current_uav_pose_; // 已被转换为 NWU 坐标系
    bool is_target_received_ = false;
    bool is_uav_pose_received_ = false;
    double current_gimbal_yaw_ = 0.0;             // 云台当前偏航角（rad，NWU北偏东）
    double current_gimbal_pitch_ = 0.0;           // 云台当前俯仰角（rad，NWU向下为正）

public:
    GimbalSimulator() {
        ros::NodeHandle n_param("~");
        // 1. 读取ROS参数（带默认值）
        n_param.param<int>("image_width", image_width_, 1920);
        n_param.param<int>("image_height", image_height_, 1080);
        n_param.param<double>("fov_deg", fov_deg_, 45.0);
        n_param.param<double>("gimbal_p_gain", gimbal_p_gain_, 0.8);
        n_param.param<double>("max_yaw_rate", max_yaw_rate_, 1.2);
        n_param.param<double>("max_pitch_rate", max_pitch_rate_, 1.2);
        n_param.param<double>("loop_freq", loop_freq_, 100.0);
        n_param.param<double>("image_noise_std_dev", image_noise_std_dev_, 0.0);

        // 仿真/真机切换
        n_param.param<bool>("use_sim", use_sim_, true);
        if (use_sim_) {
            uav_pose_topic_ = "quad/pose";
        } else {
            uav_pose_topic_ = "mavros/local_position/pose";
        }

        // === 新增参数:相机相对于机体的偏角 ===
        n_param.param<double>("cam_yaw_offset_deg", cam_yaw_offset_deg_, 0.0);
        n_param.param<double>("cam_pitch_deg", cam_pitch_deg_, -90.0);

        // 随机数引擎
        rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        noise_dist_ = std::normal_distribution<double>(0.0, image_noise_std_dev_);

        // 视场角转换为弧度
        fov_rad_ = fov_deg_ * M_PI / 180.0;

        // 订阅
        target_sub_ = nh_.subscribe("/target_position", 10, &GimbalSimulator::targetCallback, this);
        uav_pose_sub_ = nh_.subscribe(uav_pose_topic_, 10, &GimbalSimulator::uavPoseCallback, this);

        // 发布:保持原有两个 + 新增 camera/pose
        los_angle_pub_ = nh_.advertise<geometry_msgs::Point>("target_los_angle", 10);
        gimbal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("gimbal_pose", 10);
        camera_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera/pose", 10);

        // 定时器
        control_timer_ = nh_.createTimer(ros::Duration(1.0/loop_freq_), &GimbalSimulator::controlLoopCallback, this);

        ROS_INFO("Params: image=%dx%d, FOV=%.1f deg, P-gain=%.2f, max_yaw_rate=%.2f rad/s, max_pitch_rate=%.2f rad/s",
                 image_width_, image_height_, fov_deg_, gimbal_p_gain_, max_yaw_rate_, max_pitch_rate_);
        ROS_INFO("Mode: %s, Subscribed to: target=%s, uav_pose=%s",
                 use_sim_ ? "SIMULATION" : "PX4 SITL", "/target_position", uav_pose_topic_.c_str());
        ROS_INFO("Publishing to: los_angle=%s, gimbal_pose=%s, camera/pose=%s",
                 "/target_los_angle", "/gimbal_pose", "/camera/pose");
        ROS_INFO("Camera offsets: yaw=%.1f deg, pitch=%.1f deg (relative to body)",
                 cam_yaw_offset_deg_, cam_pitch_deg_);
    }

    // 目标位置回调
    void targetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        current_target_pos_ = *msg;
        is_target_received_ = true;
    }

    // 无人机位姿回调
    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 维持原有转换(NED → NWU 或 ENU → NWU)
        if (use_sim_) {
            // NED → NWU: y,z 取反
            current_uav_pose_.pose.position.x = msg->pose.position.x;
            current_uav_pose_.pose.position.y = -msg->pose.position.y;
            current_uav_pose_.pose.position.z = -msg->pose.position.z;
            current_uav_pose_.pose.orientation.w = msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x = msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        } else {
            // ENU → NWU: x = y_enu, y = -x_enu, z = z_enu
            // (NED → NWU 等价于 180°绕X + 180°绕X = 恒等)
            current_uav_pose_.pose.position.x = msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z = msg->pose.position.z;
            current_uav_pose_.pose.orientation.w = msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x = msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        }
        current_uav_pose_.header.stamp = msg->header.stamp;
        current_uav_pose_.header.frame_id = msg->header.frame_id;
        is_uav_pose_received_ = true;
    }

    // 云台控制主循环（100Hz）
    void controlLoopCallback(const ros::TimerEvent&) {
        if (!is_target_received_ || !is_uav_pose_received_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for target or UAV pose data...");
            return;
        }

        // 1. 计算目标相对于无人机的期望云台角度（跟踪用途，仍采用原逻辑）
        double desired_yaw, desired_pitch;
        calculateDesiredGimbalAngles(desired_yaw, desired_pitch);

        // 2. 云台比例控制
        updateGimbalAngles(desired_yaw, desired_pitch);

        // 3. 发布目标 LOS(用户指示:此处是目标的 LOS,不是相机方向)
        publishLOSAngle(desired_yaw, desired_pitch);

        // 4. 发布云台位姿(用于 RViz 显示跟踪过程)
        publishGimbalPose();

        // 5. 新增:发布相机在世界系下的"实际朝向"
        //    = UAV 航向 + cam_yaw_offset / cam_pitch(机体坐标系)
        publishCameraPose();
    }

    // 计算目标相对无人机的期望云台角度(NWU 下,at fixed tracking 不变,纯几何)
    void calculateDesiredGimbalAngles(double& desired_yaw, double& desired_pitch) {
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        // z 不用(目标在地面)

        double target_x = current_target_pos_.x;
        double target_y = current_target_pos_.y;

        if (image_noise_std_dev_ > 0.0) {
            target_x += noise_dist_(rng_);
            target_y += noise_dist_(rng_);
        }

        // 云台安装位置(NWU)
        double gimbal_mount_x_global = uav_x + gimbal_mount_x_;
        double gimbal_mount_y_global = uav_y + gimbal_mount_y_;

        double dx = target_x - gimbal_mount_x_global;
        double dy = target_y - gimbal_mount_y_global;

        // 注意:NWU 下,z 实际为高度,目前 target_z 不参与
        desired_yaw = atan2(dy, dx);
        desired_yaw = atan2(sin(desired_yaw), cos(desired_yaw));
        // 期望俯仰保留原计算(后面会用于跟踪精度反馈)
        desired_pitch = atan2(0.0, sqrt(dx*dx + dy*dy));
        desired_pitch = atan2(sin(desired_pitch), cos(desired_pitch));
    }

    // 云台比例控制(P + 限速)
    void updateGimbalAngles(double desired_yaw, double desired_pitch) {
        double dt = 1.0 / loop_freq_;

        // 偏航
        double yaw_error = desired_yaw - current_gimbal_yaw_;
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));
        double desired_yaw_rate = gimbal_p_gain_ * yaw_error;
        desired_yaw_rate = std::max(-max_yaw_rate_, std::min(desired_yaw_rate, max_yaw_rate_));
        current_gimbal_yaw_ += desired_yaw_rate * dt;
        current_gimbal_yaw_ = atan2(sin(current_gimbal_yaw_), cos(current_gimbal_yaw_));

        // 俯仰
        double pitch_error = desired_pitch - current_gimbal_pitch_;
        pitch_error = atan2(sin(pitch_error), cos(pitch_error));
        double desired_pitch_rate = gimbal_p_gain_ * pitch_error;
        desired_pitch_rate = std::max(-max_pitch_rate_, std::min(desired_pitch_rate, max_pitch_rate_));
        current_gimbal_pitch_ += desired_pitch_rate * dt;
        current_gimbal_pitch_ = atan2(sin(current_gimbal_pitch_), cos(current_gimbal_pitch_));

        double angular_error = sqrt(yaw_error*yaw_error + pitch_error*pitch_error);
        gimbal_tracking_accuracy_ = exp(-angular_error);
    }

    // 发布目标 LOS 消息(user:这是目标的,不是相机的;保持原语义)
    void publishLOSAngle(double desired_yaw, double desired_pitch) {
        geometry_msgs::Point los_angle_msg;
        los_angle_msg.x = desired_yaw;
        los_angle_msg.y = desired_pitch;
        if (std::isnan(gimbal_tracking_accuracy_)) {
            gimbal_tracking_accuracy_ = 0.5;
        }
        los_angle_msg.z = gimbal_tracking_accuracy_;
        los_angle_pub_.publish(los_angle_msg);
    }

    // 发布云台位姿(RViz 显示跟踪过程)
    void publishGimbalPose() {
        geometry_msgs::PoseStamped gimbal_pose_msg;
        gimbal_pose_msg.header.stamp = ros::Time::now();
        gimbal_pose_msg.header.frame_id = "map";

        gimbal_pose_msg.pose.position = current_uav_pose_.pose.position;

        tf::Quaternion gimbal_quat;
        gimbal_quat.setRPY(0.0, -current_gimbal_pitch_, current_gimbal_yaw_);

        gimbal_pose_msg.pose.orientation.x = gimbal_quat.x();
        gimbal_pose_msg.pose.orientation.y = gimbal_quat.y();
        gimbal_pose_msg.pose.orientation.z = gimbal_quat.z();
        gimbal_pose_msg.pose.orientation.w = gimbal_quat.w();

        gimbal_pose_pub_.publish(gimbal_pose_msg);
    }

    // === 新增:发布相机在世界系下的"实际朝向"(Phase 1) ===
    //
    // 不再用 gimbal 跟踪姿态,而是基于 UAV 当前航向 + cam_yaw_offset/cam_pitch
    // 计算出相机机体系朝向,然后乘到 UAV 四元数上得到世界系朝向。
    //
    // 这么设计:
    //   - 检测仿真(detection_simulator)用它判矩形 FOV
    //   - "search scanning"等功能也是在这个基础上加 cam_yaw_offset 摇摆
    void publishCameraPose() {
        geometry_msgs::PoseStamped camera_pose_msg;
        camera_pose_msg.header.stamp = ros::Time::now();
        camera_pose_msg.header.frame_id = "map";

        // 相机位置 = UAV 位置 + 安装偏移(机体前向,这里简化为 UAV 位置)
        camera_pose_msg.pose.position = current_uav_pose_.pose.position;

        // 机体坐标系下的相机姿态 = (cam_yaw_offset, cam_pitch, 0)
        tf::Quaternion body_cam_quat;
        double yaw_body = cam_yaw_offset_deg_ * M_PI / 180.0;
        double pitch_body = cam_pitch_deg_ * M_PI / 180.0;
        body_cam_quat.setRPY(0.0, -pitch_body, yaw_body); // NWU 下 roll=0, pitch 向下为正,与 gimbal_pose 保持一致

        // UAV 在 NWU 下的姿态四元数
        tf::Quaternion uav_quat(
            current_uav_pose_.pose.orientation.x,
            current_uav_pose_.pose.orientation.y,
            current_uav_pose_.pose.orientation.z,
            current_uav_pose_.pose.orientation.w);

        // 世界系相机姿态 = UAV 姿态 * 机体相机姿态
        tf::Quaternion world_cam_quat = uav_quat * body_cam_quat;
        world_cam_quat.normalize();

        camera_pose_msg.pose.orientation.x = world_cam_quat.x();
        camera_pose_msg.pose.orientation.y = world_cam_quat.y();
        camera_pose_msg.pose.orientation.z = world_cam_quat.z();
        camera_pose_msg.pose.orientation.w = world_cam_quat.w();

        camera_pose_pub_.publish(camera_pose_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_simulator_node");
    GimbalSimulator gimbal_sim;
    ros::spin();
    return 0;
}
