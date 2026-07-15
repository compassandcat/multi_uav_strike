#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "multi_uav_strike/YoloDetection.h"
#include "multi_uav_strike/YoloDetections.h"
#include "multi_uav_strike/SimTarget.h"
#include "multi_uav_strike/SimTargets.h"

using namespace std;

/**
 * 云台模拟器 + 检测模拟器 合并版 (2026-07-07)
 *
 * 设计要点:
 * 1. 云台只能控制俯仰, 偏航固定为 0
 * 2. 相机俯仰约定: 0 = 水平, -π/2 = 垂直下视
 *    也就是说 gimbal_pitch_ ∈ [-π/2, 0]
 * 3. 控制算法: 先判断目标在当前云台角度下是否在视野中
 *    - 在视野中 → 让云台指向目标(计算 desired_pitch)
 *    - 不在视野中 → 回到默认角度 (default_pitch_deg, 默认 -45)
 * 4. FOV 判定: 矩形视场(类 detection_simulator), 把目标投到相机系
 * 5. 替代 detection_simulator_node: 在 gimbal 内做检测,
 *    发布 /<ns>/detection/yolo_result 和 /<ns>/detection/target_in_view
 * 6. 保留原 /target_los_angle, /gimbal_pose, /camera/pose
 *
 * 与 detection_simulator 的关键差异:
 * - 矩形 FOV 中心是当前云台指向(不再是固定的 cam_pitch=-90)
 * - 目标"在不在 FOV 中"取决于云台当前角度, 而非机体固定相机角度
 */

template<typename T>
static inline T clampv(T v, T lo, T hi) { return std::max(lo, std::min(v, hi)); }

class GimbalSimulator {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // 订阅
    ros::Subscriber target_sub_;
    ros::Subscriber uav_pose_sub_;

    // 发布
    ros::Publisher los_angle_pub_;     // 目标 LOS (保持)
    ros::Publisher gimbal_pose_pub_;   // 云台姿态 (RViz 用, 保持)
    ros::Publisher camera_pose_pub_;   // 相机世界姿态 (保持)
    ros::Publisher yolo_pub_;          // typed YoloDetection (单条,主目标,兼容 comm_node)
    ros::Publisher yolo_results_pub_;  // typed YoloDetections (多目标,供 target_estimator)
    ros::Publisher in_fov_pub_;        // Bool (合并自 detection_simulator)

    ros::Timer control_timer_;
    uint32_t frame_seq_ = 0;

    // === FOV / 图像参数 ===
    int image_width_;
    int image_height_;
    double fov_h_deg_;       // 水平 FOV (度)
    double fov_v_deg_;       // 垂直 FOV (度, 0 表示按方形像素从 fov_h 推导)
    double fov_h_rad_;
    double fov_v_rad_;
    double focal_x_, focal_y_;
    std::string label_;

    // === 控制参数 ===
    double gimbal_p_gain_;
    double max_pitch_rate_;
    double loop_freq_;
    bool use_sim_;
    double image_noise_std_dev_;
    std::string uav_pose_topic_;
    double desired_pitch, desired_yaw;

    // 固定模拟 JPEG(测试用,真实 gimbal 接入后由相机抓拍帧替换)
    std::vector<uint8_t> fake_jpeg_;
    std::string fake_jpeg_path_;

    // === 云台参数 (新约定: pitch ∈ [-π/2, 0]) ===
    //   current_gimbal_pitch_ = 0      → 水平
    //   current_gimbal_pitch_ = -π/2   → 垂直下视
    //   default_pitch_deg_             → 目标丢失时回退到此角度(默认 -45)
    double current_gimbal_pitch_ = 0.0;
    double default_pitch_deg_;
    double default_pitch_rad_;

    // 噪声
    std::default_random_engine rng_;
    std::normal_distribution<double> noise_dist_;

    // === 状态 ===
    std::vector<multi_uav_strike::SimTarget> targets_;   // 全部仿真目标(来自 /sim_targets)
    geometry_msgs::PoseStamped current_uav_pose_;   // NWU
    bool is_target_received_ = false;
    bool is_uav_pose_received_ = false;

    // === 每 loop tick 缓存的相机-目标投影(逐目标) ===
    //   由 recomputeAllProjections() 在循环开头一次性算出所有目标的投影,
    //   之后 FOV 判定 / 像素投影 / distance / camera pose / 逐目标 YoloDetection 都读缓存。
    struct TargetProj {
        uint32_t    id       = 0;
        std::string label;
        geometry_msgs::Point pos;      // 目标 NWU 世界坐标
        bool   visible   = false;      // 在 FOV 里
        double u_px      = 0.0;        // 像素 u
        double v_px      = 0.0;        // 像素 v
        double distance  = 0.0;        // 相机系距离
    };
    Eigen::Quaterniond cached_q_cam_world_;
    bool   cached_proj_valid_ = false;             // 数据齐了(有目标 + 有 pose)
    std::vector<TargetProj> projections_;          // 与 targets_ 对齐
    int    primary_idx_ = -1;                      // projections_ 内的主目标索引(FOV 内最近);-1=无

public:
    GimbalSimulator() {
        nh_private_ = ros::NodeHandle("~");

        // FOV / 图像
        nh_private_.param<int>("image_width", image_width_, 1920);
        nh_private_.param<int>("image_height", image_height_, 1080);
        nh_private_.param<double>("fov_h_deg", fov_h_deg_, 66.0);
        nh_private_.param<double>("fov_v_deg", fov_v_deg_, 52.0);   // 0 = 按方形像素从 fov_h 推导
        nh_private_.param<std::string>("label", label_, "general_target");

        // 控制
        nh_private_.param<double>("gimbal_p_gain", gimbal_p_gain_, 0.8);
        nh_private_.param<double>("max_pitch_rate", max_pitch_rate_, 1.2);
        nh_private_.param<double>("loop_freq", loop_freq_, 100.0);
        nh_private_.param<bool>("use_sim", use_sim_, true);
        nh_private_.param<double>("image_noise_std_dev", image_noise_std_dev_, 0.0);

        // 加载固定模拟 JPEG:放到 YoloDetection.img_data 给下游(mission_manager →
        // starling_bridge → DEVICE_TARGETS 0x2001)组装上行报文;真实 gimbal 接入后
        // 此参数失效,由相机抓拍帧替换。
        nh_private_.param<std::string>("fake_jpeg_path", fake_jpeg_path_,
                                        ros::package::getPath("multi_uav_strike") +
                                        "/config/fake_target.jpg");
        loadFakeJpeg(fake_jpeg_path_);

        // 新约定下的回退角
        nh_private_.param<double>("default_pitch_deg", default_pitch_deg_, 45.0);

        if (use_sim_) {
            uav_pose_topic_ = "quad/pose";
        } else {
            uav_pose_topic_ = "mavros/local_position/pose";
        }

        // 焦点长度(focal length in pixels)
        // focal_x / focal_y 独立计算,各自对应 fov_h / fov_v 半角的 tan
        //   focal_x = (W/2) / tan(fov_h/2)
        //   focal_y = (H/2) / tan(fov_v/2)
        // 若 fov_v_deg=0 (方形像素),则按 W/H 比例从 fov_h 推 fov_v
        fov_h_rad_ = fov_h_deg_ * M_PI / 180.0;
        if (fov_v_deg_ > 0.0) {
            fov_v_rad_ = fov_v_deg_ * M_PI / 180.0;
        } else {
            double aspect = static_cast<double>(image_width_) / image_height_;
            fov_v_rad_ = 2.0 * std::atan(std::tan(fov_h_rad_ / 2.0) / aspect);
        }
        focal_x_ = (image_width_  / 2.0) / std::tan(fov_h_rad_ / 2.0);
        focal_y_ = (image_height_ / 2.0) / std::tan(fov_v_rad_ / 2.0);

        default_pitch_rad_ = clampv(default_pitch_deg_ * M_PI / 180.0, 0.0, M_PI/2.0);
        current_gimbal_pitch_ = default_pitch_rad_;

        // 噪声
        rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        noise_dist_ = std::normal_distribution<double>(0.0, image_noise_std_dev_);

        // 订阅
        target_sub_ = nh_.subscribe("/sim_targets", 10,
                                    &GimbalSimulator::targetsCallback, this);
        uav_pose_sub_ = nh_.subscribe(uav_pose_topic_, 10,
                                      &GimbalSimulator::uavPoseCallback, this);

        // 发布(保持)
        los_angle_pub_   = nh_.advertise<geometry_msgs::Point>("target_los_angle", 10);
        gimbal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("gimbal_pose", 10);
        camera_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera/pose", 10);

        // 发布(合并自 detection_simulator)
        std::string ns = nh_.getNamespace();
        std::string yolo_t = "detection/yolo_result";
        if (!ns.empty() && ns != "/") yolo_t = ns + "/" + yolo_t;
        if (yolo_t.find("//") == 0) yolo_t = yolo_t.substr(1);
        yolo_pub_ = nh_.advertise<multi_uav_strike::YoloDetection>(yolo_t, 10);

        // 多目标批量检测(供 target_estimator)
        std::string yolos_t = "detection/yolo_results";
        if (!ns.empty() && ns != "/") yolos_t = ns + "/" + yolos_t;
        if (yolos_t.find("//") == 0) yolos_t = yolos_t.substr(1);
        yolo_results_pub_ = nh_.advertise<multi_uav_strike::YoloDetections>(yolos_t, 10);

        std::string fov_t = "detection/target_in_view";
        if (!ns.empty() && ns != "/") fov_t = ns + "/" + fov_t;
        if (fov_t.find("//") == 0) fov_t = fov_t.substr(1);
        in_fov_pub_ = nh_.advertise<std_msgs::Bool>(fov_t, 10);

        // 100Hz 主循环
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / loop_freq_),
                                         &GimbalSimulator::controlLoopCallback, this);

        ROS_INFO("[GimbalSim] image=%dx%d fov=(%.1fh x %.1fv deg) focal=(%.1f, %.1f)",
                 image_width_, image_height_, fov_h_deg_,
                 (fov_v_deg_ > 0.0 ? fov_v_deg_ : fov_v_rad_ * 180.0 / M_PI),
                 focal_x_, focal_y_);
        ROS_INFO("[GimbalSim] pitch conv: 0=horizontal, -90=down; default_pitch=%.1fdeg",
                 default_pitch_deg_);
        ROS_INFO("[GimbalSim] uav_pose=%s, publishing target_los_angle, gimbal_pose, "
                 "camera/pose, %s, %s",
                 uav_pose_topic_.c_str(), yolo_t.c_str(), fov_t.c_str());
    }

    // ====================================================================
    // 订阅回调
    // ====================================================================

    void targetsCallback(const multi_uav_strike::SimTargets::ConstPtr& msg) {
        targets_ = msg->targets;
        is_target_received_ = !targets_.empty();
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 与原版保持一致:NED/ENU → NWU
        if (use_sim_) {
            current_uav_pose_.pose.position.x =  msg->pose.position.x;
            current_uav_pose_.pose.position.y = -msg->pose.position.y;
            current_uav_pose_.pose.position.z = -msg->pose.position.z;
            current_uav_pose_.pose.orientation.w =  msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x =  msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        } else {
            // ENU → NWU: x = y_enu, y = -x_enu, z = z_enu
            current_uav_pose_.pose.position.x =  msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z =  msg->pose.position.z;
            // 四元数 mavros(ENU 世界 + FRD 机体) -> 内部(NWU 世界 + FLU 机体):
            //   q_internal = q_W · q_mavros · q_B
            //   q_W = (√2/2, 0, 0, -√2/2)  ENU→NWU: 绕世界 Z 旋转 -90° (E→N, N→W)
            //   q_B = (0, 1, 0, 0)          FRD→FLU: 绕机体 X 旋转 180° (F→-F≈F, R→L, D→U)
            // 展开后 (Hamilton 约定, q_B = (0,1,0,0) 会交换 w↔x、y↔-z):
            const double kSqrtHalf = 0.7071067811865475;
            const auto& qe = msg->pose.orientation;
            current_uav_pose_.pose.orientation.w = -kSqrtHalf * (qe.x + qe.y);
            current_uav_pose_.pose.orientation.x =  kSqrtHalf * (qe.w + qe.z);
            current_uav_pose_.pose.orientation.y =  kSqrtHalf * (qe.z - qe.w);
            current_uav_pose_.pose.orientation.z =  kSqrtHalf * (qe.x - qe.y);
        }
        current_uav_pose_.header.stamp = msg->header.stamp;
        current_uav_pose_.header.frame_id = msg->header.frame_id;
        is_uav_pose_received_ = true;
    }

    // ====================================================================
    // 主循环 (100Hz)
    // ====================================================================

    void controlLoopCallback(const ros::TimerEvent&) {
        // === 1. 一次性算出所有目标的相机投影(后续都读缓存),并选主目标 ===
        recomputeAllProjections();

        // 数据没到齐:发"空帧"+保持当前云台角度
        if (!cached_proj_valid_) {
            ROS_WARN_THROTTLE(5.0, "[GimbalSim] waiting for targets/pose ...");
            publishYoloDetections();
            return;
        }

        // === 2. 决定 desired_pitch(指向主目标;无主目标则回退默认角) ===
        bool has_primary = (primary_idx_ >= 0);
        if (has_primary) {
            const TargetProj& p = projections_[primary_idx_];
            desired_pitch = computePitchAndYawToTarget(p.pos.x, p.pos.y);
            ROS_INFO_THROTTLE(5.0, "[GimbalSim] primary target id=%u label=%s in view: "
                                "u_px=%.1f v_px=%.1f distance=%.2fm desired_pitch=%.2fdeg "
                                "(total_visible=%zu)",
                              p.id, p.label.c_str(), p.u_px, p.v_px, p.distance,
                              desired_pitch * 180.0 / M_PI, countVisible());
        } else {
            desired_pitch = default_pitch_rad_;
        }

        // === 3. 云台比例控制 ===
        updateGimbalAngles(desired_pitch);

        // === 4. 发布 ===
        publishLOSAngle(desired_pitch, desired_yaw);       // target_los_angle(指向主目标)
        publishGimbalPose();                  // gimbal_pose (RViz)
        publishCameraPose();                  // camera/pose (用缓存的 q_cam_world_)
        publishYoloDetections();              // YoloDetections(全部可见目标) + 单条 YoloDetection(主目标)
    }

    size_t countVisible() const {
        size_t n = 0;
        for (const auto& p : projections_) if (p.visible) ++n;
        return n;
    }

    // ====================================================================
    // 核心算法: 计算 desired_gimbal_angle
    // ====================================================================
    //
    // 关键改动: 先在"当前云台角度"下做 FOV 判定
    //   - 目标在视野中 → desired_pitch = atan2(高度,水平距) 指向目标
    //   - 目标不在视野中 → desired_pitch = default_pitch_rad_ (回退)
    // 偏航固定 0, 不参与计算(desired_yaw 一直为 0)
    //
    // 计算目标相对无人机的"几何"俯仰角(忽略云台当前状态)
    // 新约定: 0=水平, -π/2=垂直下视
    //   uav 高于 target 时, target 在视野下方, desired_pitch 为负
    double computePitchAndYawToTarget(double target_x, double target_y) {
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_alt = current_uav_pose_.pose.position.z;  // NWU: z=高度

        if (image_noise_std_dev_ > 0.0) {
            target_x += noise_dist_(rng_);
            target_y += noise_dist_(rng_);
        }

        double dx = target_x - uav_x;
        double dy = target_y - uav_y;
        double horiz = std::sqrt(dx*dx + dy*dy);
        double drop = uav_alt - 0.0;  // 目标假设在地面, 高度 0

        desired_yaw = atan2(dy, dx);
        desired_yaw = atan2(sin(desired_yaw), cos(desired_yaw));

        // atan2(drop, horiz) ∈ (0, π/2)  (目标在下方)
        // 新约定"看向下"为负, 所以取负号
        double pitch = std::atan2(drop, horiz);
        return clampv(pitch, 0.0, M_PI/2.0);
    }

    // ====================================================================
    // 单次投影: 对所有目标算出相机-目标几何, 缓存到 projections_, 并选主目标
    // ====================================================================
    //
    // 每 tick 在 controlLoopCallback 开头调一次, 后续全部读缓存:
    //   - projections_[i].visible/u_px/v_px/distance
    //   - cached_q_cam_world_   (camera/pose 发布)
    //   - primary_idx_          (FOV 内距 UAV 最近的目标; -1=无)
    //
    // 相机世界姿态 = uav_quat(NWU) * R(0, gimbal_pitch, 0)
    // 相机系约定: x=前, y=右, z=下
    // 目标在前方 (t_cam.z > 0) 且投影在矩形内 → visible
    void recomputeAllProjections() {
        cached_proj_valid_ = is_target_received_ && is_uav_pose_received_;
        projections_.clear();
        primary_idx_ = -1;

        if (!cached_proj_valid_) return;

        // 相机世界姿态(所有目标共用)
        cached_q_cam_world_ = currentCameraWorldQuat();
        Eigen::Matrix3d R_world_cam = cached_q_cam_world_.toRotationMatrix();

        double best_dist = 1e18;
        projections_.reserve(targets_.size());
        for (const auto& t : targets_) {
            TargetProj pr;
            pr.id    = t.id;
            pr.label = t.label;
            pr.pos   = t.position;

            Eigen::Vector3d world_diff(
                t.position.x - current_uav_pose_.pose.position.x,
                t.position.y - current_uav_pose_.pose.position.y,
                t.position.z - current_uav_pose_.pose.position.z);
            Eigen::Vector3d t_cam = R_world_cam.transpose() * world_diff;

            if (t_cam.z() > 0.05) {
                double u_ndc = t_cam.y() / t_cam.z();
                double v_ndc = -t_cam.x() / t_cam.z();
                pr.u_px = u_ndc * focal_x_ + image_width_  / 2.0;
                pr.v_px = v_ndc * focal_y_ + image_height_ / 2.0;
                bool u_ok = (pr.u_px >= 0.0 && pr.u_px < image_width_);
                bool v_ok = (pr.v_px >= 0.0 && pr.v_px < image_height_);
                pr.visible = u_ok && v_ok;
                pr.distance = t_cam.norm();
            }

            if (pr.visible && pr.distance < best_dist) {
                best_dist = pr.distance;
                primary_idx_ = static_cast<int>(projections_.size());
            }
            projections_.push_back(pr);
        }
    }

    // 计算相机世界姿态(纯函数, 不改状态)
    // 注意: 不写缓存, 调用方要么自己存、要么用 recomputeCameraTargetProjection() 缓存版
    Eigen::Quaterniond currentCameraWorldQuat() const {
        // body 系: UAV 机体 (NWU: x=前, y=左, z=上)
        // 相机相对机体只有 pitch (新约定: 0=水平, -π/2=下视)
        // 注意: setRPY 在 NWU 下, 负 pitch 意味着"低头"
        tf::Quaternion body_cam_quat;
        body_cam_quat.setRPY(0.0, current_gimbal_pitch_, 0.0);

        // UAV 在 NWU 下的姿态
        tf::Quaternion uav_quat(
            current_uav_pose_.pose.orientation.x,
            current_uav_pose_.pose.orientation.y,
            current_uav_pose_.pose.orientation.z,
            current_uav_pose_.pose.orientation.w);

        // 复合: 先 body 旋转, 再 UAV 旋转 → 世界系
        tf::Quaternion world_cam_quat = uav_quat * body_cam_quat;
        world_cam_quat.normalize();

        return Eigen::Quaterniond(world_cam_quat.w(),
                                  world_cam_quat.x(),
                                  world_cam_quat.y(),
                                  world_cam_quat.z());
    }

    // ====================================================================
    // 控制: P-control (只动 pitch)
    // ====================================================================
    void updateGimbalAngles(double desired_pitch) {
        double dt = 1.0 / loop_freq_;
        double pitch_error = desired_pitch - current_gimbal_pitch_;
        double rate = gimbal_p_gain_ * pitch_error;
        rate = clampv(rate, -max_pitch_rate_, max_pitch_rate_);
        current_gimbal_pitch_ += rate * dt;
        current_gimbal_pitch_ = clampv(current_gimbal_pitch_, 0.0, M_PI/2.0);
    }

    // ====================================================================
    // 发布(全部读 cached_*, 不再做几何计算)
    // ====================================================================

    // target_los_angle: 目标 LOS (Point.x=yaw, .y=pitch, .z=tracking_accuracy)
    // 维持原语义:发的是 desired (目标的几何 LOS),不是当前相机
    // 发布NED下的角度
    void publishLOSAngle(double desired_pitch, double desired_yaw) {
        geometry_msgs::Point msg;
        msg.x = -desired_yaw;  // 偏航
        msg.y = -desired_pitch;
        // tracking_accuracy: 当前 pitch 与 desired 的接近度(指数衰减)
        double err = std::fabs(desired_pitch - current_gimbal_pitch_);
        double acc = std::exp(-err);
        msg.z = std::isnan(acc) ? 0.5 : acc;
        los_angle_pub_.publish(msg);
    }

    // gimbal_pose: 当前云台在 NWU 下的姿态 (RViz 用)
    void publishGimbalPose() {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.position = current_uav_pose_.pose.position;

        tf::Quaternion q;
        // 新约定: pitch 直接传给 setRPY (NWU 下负 pitch = 低头)
        q.setRPY(0.0, current_gimbal_pitch_, 0.0);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        gimbal_pose_pub_.publish(msg);
    }

    // camera/pose: 相机世界姿态(直接复用缓存的 q_cam_world_)
    void publishCameraPose() {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.position = current_uav_pose_.pose.position;
        // msg.pose.orientation.x = cached_q_cam_world_.x();
        // msg.pose.orientation.y = cached_q_cam_world_.y();
        // msg.pose.orientation.z = cached_q_cam_world_.z();
        // msg.pose.orientation.w = cached_q_cam_world_.w();
        
        // body 系: UAV 机体 (NWU: x=前, y=左, z=上)
        // 相机相对机体只有 pitch (新约定: 0=水平, -π/2=下视)
        // 注意: setRPY 在 NWU 下, 负 pitch 意味着"低头"
        tf::Quaternion body_cam_quat;
        body_cam_quat.setRPY(0.0, -current_gimbal_pitch_, 0.0);

        // UAV 在 NWU 下的姿态
        tf::Quaternion uav_quat(
            current_uav_pose_.pose.orientation.x,
            current_uav_pose_.pose.orientation.y,
            current_uav_pose_.pose.orientation.z,
            current_uav_pose_.pose.orientation.w);

        // 复合: 先 body 旋转, 再 UAV 旋转 → 世界系
        tf::Quaternion world_cam_quat = uav_quat * body_cam_quat;
        world_cam_quat.normalize();

        msg.pose.orientation.x = world_cam_quat.x();
        msg.pose.orientation.y = world_cam_quat.y();
        msg.pose.orientation.z = world_cam_quat.z();
        msg.pose.orientation.w = world_cam_quat.w();
        camera_pose_pub_.publish(msg);
    }

    // 从一个 TargetProj 构造一条 YoloDetection(仅在 visible 时填 bbox/conf/img)
    multi_uav_strike::YoloDetection buildDetection(const TargetProj& pr, uint32_t frame_seq,
                                                   uint64_t stamp_us) {
        multi_uav_strike::YoloDetection out;
        out.frame_seq = frame_seq;
        out.stamp_us = stamp_us;
        out.label = pr.label;
        out.confidence = 0.0f;
        out.x_min = out.y_min = out.x_max = out.y_max = 0.0f;
        out.is_in_fov = false;
        if (pr.visible) {
            // 简化的 bbox: size ∝ 1/distance
            double size_px = std::min(image_width_, image_height_) * 0.05 +
                             1000.0 / (pr.distance + 1.0);
            size_px = std::min(size_px, std::min(image_width_, image_height_) * 0.4);
            double half = size_px / 2.0;
            out.x_min = (float)std::max(0.0, (pr.u_px - half) / image_width_);
            out.y_min = (float)std::max(0.0, (pr.v_px - half) / image_height_);
            out.x_max = (float)std::min(1.0, (pr.u_px + half) / image_width_);
            out.y_max = (float)std::min(1.0, (pr.v_px + half) / image_height_);
            out.confidence = (float)std::max(0.4, 0.95 - 0.001 * pr.distance);
            out.is_in_fov = true;
            // 可见帧才携带模拟 JPEG;真实 gimbal 接入后此分支替换为相机抓拍帧
            out.img_data = fake_jpeg_;
        }
        return out;
    }

    // 发布:YoloDetections(全部可见目标,供 target_estimator)
    //       + 单条 YoloDetection(主目标,兼容 comm_node)
    //       + Bool(是否有任一目标在 FOV)
    void publishYoloDetections() {
        uint32_t seq = ++frame_seq_;
        uint64_t stamp_us = ros::Time::now().toNSec() / 1000;

        multi_uav_strike::YoloDetections batch;
        bool any_visible = false;
        for (const auto& pr : projections_) {
            if (!pr.visible) continue;   // 只把在 FOV 内的目标放入批量(与原单目标语义一致)
            batch.detections.push_back(buildDetection(pr, seq, stamp_us));
            any_visible = true;
        }
        yolo_results_pub_.publish(batch);

        // 单条 = 主目标(无主目标则空帧 is_in_fov=false)
        multi_uav_strike::YoloDetection primary;
        if (primary_idx_ >= 0) {
            primary = buildDetection(projections_[primary_idx_], seq, stamp_us);
        } else {
            primary.frame_seq = seq;
            primary.stamp_us = stamp_us;
            primary.label = "";
            primary.confidence = 0.0f;
            primary.x_min = primary.y_min = primary.x_max = primary.y_max = 0.0f;
            primary.is_in_fov = false;
        }
        yolo_pub_.publish(primary);

        std_msgs::Bool in_fov_flag;
        in_fov_flag.data = any_visible;
        in_fov_pub_.publish(in_fov_flag);
    }

    // 从磁盘加载固定 JPEG 字节流;加载失败时 fake_jpeg_ 留空(下游 img_data 也空)
    void loadFakeJpeg(const std::string& path) {
        std::ifstream ifs(path, std::ios::binary | std::ios::ate);
        if (!ifs) {
            ROS_WARN("[GimbalSim] fake_jpeg_path='%s' open failed, img_data will be empty",
                     path.c_str());
            return;
        }
        const std::streamsize sz = ifs.tellg();
        if (sz <= 0) {
            ROS_WARN("[GimbalSim] fake_jpeg_path='%s' is empty", path.c_str());
            return;
        }
        ifs.seekg(0, std::ios::beg);
        fake_jpeg_.resize(static_cast<size_t>(sz));
        if (!ifs.read(reinterpret_cast<char*>(fake_jpeg_.data()), sz)) {
            ROS_WARN("[GimbalSim] fake_jpeg_path='%s' read failed", path.c_str());
            fake_jpeg_.clear();
            return;
        }
        ROS_INFO("[GimbalSim] loaded fake JPEG: path=%s size=%zu bytes",
                 path.c_str(), fake_jpeg_.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_simulator_node");
    GimbalSimulator gimbal_sim;
    ros::spin();
    return 0;
}