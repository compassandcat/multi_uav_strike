/**
 * detection_simulator_node.cpp
 *
 * 基于相机 FOV 几何投屏的"伪 YOLO 检测器"
 * 输入:
 *   - UAV 当前位姿(/<ns>/quad/pose 或 /<ns>/mavros/local_position/pose,带航向)
 *   - 目标真值(/target_position)
 *   - 相机世界系姿态(由 gimbal_simulator_node 发布到 /<ns>/camera/pose)
 * 输出:
 *   - /<ns>/detection/yolo_result  multi_uav_strike/YoloDetection (20Hz)
 *
 * 关键:矩形 FOV 判定(像 1920x1080 / 640x480),不是单纯 fov_deg 圆锥
 *      is_in_fov=false 时目标不在视野中,下游 mission_manager 据此过滤
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>

#include <cmath>
#include <vector>
#include <string>

#include "multi_uav_strike/YoloDetection.h"

// tf2 工具(简化:用 Eigen 等价手算四元数 → 旋转矩阵)
#include <Eigen/Dense>
#include <Eigen/Geometry>

class DetectionSimulator {
public:
    DetectionSimulator() : nh_(), nh_private_("~") {
        // ========== 参数加载 ==========
        nh_private_.param<bool>("use_sim", use_sim_, true);
        nh_private_.param<int>("image_width", image_width_, 1920);
        nh_private_.param<int>("image_height", image_height_, 1080);
        nh_private_.param<double>("fov_h_deg", fov_h_deg_, 60.0);     // 水平 FOV(度)
        nh_private_.param<double>("fov_v_deg", fov_v_deg_, 0.0);        // 0 表示按比例算
        nh_private_.param<double>("loop_freq", loop_freq_, 20.0);
        nh_private_.param<std::string>("uav_pose_topic", uav_pose_topic_, "");
        nh_private_.param<std::string>("target_topic", target_topic_, "/target_position");
        nh_private_.param<std::string>("camera_pose_topic", camera_pose_topic_, "camera/pose");
        nh_private_.param<std::string>("out_topic", out_topic_, "detection/yolo_result");
        nh_private_.param<std::string>("in_fov_topic", in_fov_topic_, "detection/target_in_view");
        nh_private_.param<std::string>("label", label_, "general_target");

        if (uav_pose_topic_.empty()) {
            uav_pose_topic_ = use_sim_ ? "quad/pose" : "mavros/local_position/pose";
        }

        // 焦点长度(focal length in pixels)
        fov_h_rad_ = fov_h_deg_ * M_PI / 180.0;
        if (fov_v_deg_ <= 0.0) {
            // 默认按图像长宽比自动算
            double aspect = static_cast<double>(image_width_) / image_height_;
            fov_v_rad_ = 2.0 * std::atan(std::tan(fov_h_rad_ / 2.0) / aspect);
        } else {
            fov_v_rad_ = fov_v_deg_ * M_PI / 180.0;
        }
        focal_x_ = (image_width_ / 2.0) / std::tan(fov_h_rad_ / 2.0);
        focal_y_ = (image_height_ / 2.0) / std::tan(fov_v_rad_ / 2.0);

        ROS_INFO("[DetectionSimulator] FOV: %.1f x %.1f deg  image: %dx%d  focal(px): %.1f x %.1f",
                 fov_h_deg_, (fov_v_rad_ * 180.0 / M_PI),
                 image_width_, image_height_, focal_x_, focal_y_);

        // ========== 订阅 ==========
        uav_pose_sub_ = nh_.subscribe(uav_pose_topic_, 10,
                                      &DetectionSimulator::uavPoseCb, this);

        target_sub_ = nh_.subscribe(target_topic_, 10,
                                    &DetectionSimulator::targetCb, this);

        camera_pose_sub_ = nh_.subscribe(camera_pose_topic_, 10,
                                         &DetectionSimulator::cameraPoseCb, this);

        // ========== 发布 ==========
        std::string out = out_topic_;
        // 若用户给了绝对路径,直接用;否则加上私有命名空间
        if (out_topic_.find("/") != 0) {
            out = nh_.getNamespace() + "/" + out_topic_;
            if (out.find("//") == 0) out = out.substr(1);
        }
        yolo_pub_ = nh_.advertise<multi_uav_strike::YoloDetection>(out, 10);

        std::string in_fov_t = in_fov_topic_;
        if (in_fov_topic_.find("/") != 0) {
            in_fov_t = nh_.getNamespace() + "/" + in_fov_topic_;
            if (in_fov_t.find("//") == 0) in_fov_t = in_fov_t.substr(1);
        }
        in_fov_pub_ = nh_.advertise<std_msgs::Bool>(in_fov_t, 10);

        // ========== 定时器(20Hz) ==========
        timer_ = nh_.createTimer(ros::Duration(1.0 / loop_freq_),
                                 &DetectionSimulator::tickCb, this);

        have_uav_pose_ = false;
        have_target_ = false;
        have_camera_pose_ = false;
        frame_seq_ = 0;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber uav_pose_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber camera_pose_sub_;
    ros::Publisher  yolo_pub_;
    ros::Publisher  in_fov_pub_;
    ros::Timer      timer_;

    std::string uav_pose_topic_;
    std::string target_topic_;
    std::string camera_pose_topic_;
    std::string out_topic_;
    std::string in_fov_topic_;
    std::string label_;

    bool use_sim_;
    int image_width_, image_height_;
    double fov_h_deg_, fov_v_deg_;
    double fov_h_rad_, fov_v_rad_;
    double focal_x_, focal_y_;
    double loop_freq_;

    // 最新 UAV 位姿(ENU,来自 mavros 或 sim)
    bool have_uav_pose_;
    geometry_msgs::PoseStamped uav_pose_enu_;

    // 相机世界系姿态(NED 框架下发布;但内部转换时我们直接用四元数)
    bool have_camera_pose_;
    geometry_msgs::PoseStamped camera_pose_world_;

    // 目标(ENU 全球,与 mission_manager 同约定)
    bool have_target_;
    geometry_msgs::Point target_enu_;

    uint32_t frame_seq_;

    // ============== 回调 ==============

    void uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        uav_pose_enu_ = *msg;
        // 若坐标系不是 ENU(unused field),保持现有策略
        have_uav_pose_ = true;
    }

    void cameraPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        camera_pose_world_ = *msg;
        have_camera_pose_ = true;
    }

    void targetCb(const geometry_msgs::Point::ConstPtr& msg) {
        target_enu_ = *msg;
        have_target_ = true;
    }

    // ============== 主循环 ==============
    void tickCb(const ros::TimerEvent&) {
        multi_uav_strike::YoloDetection out;
        out.frame_seq = ++frame_seq_;
        out.stamp_us = ros::Time::now().toNSec() / 1000;
        out.label = label_;
        out.confidence = 0.0;
        out.x_min = 0.0; out.y_min = 0.0; out.x_max = 0.0; out.y_max = 0.0;
        out.is_in_fov = false;

        std_msgs::Bool in_fov_flag;
        in_fov_flag.data = false;

        if (!have_uav_pose_ || !have_target_ || !have_camera_pose_) {
            // 必要数据未到齐,发"空帧",下游收到会忽略(is_in_fov=false)
            yolo_pub_.publish(out);
            in_fov_pub_.publish(in_fov_flag);
            return;
        }

        // ========== 矩形 FOV 几何判定 ==========
        // Step 1: 目标相对相机的位置(t_cam)
        Eigen::Vector3d t_cam = targetRelativeToCameraENU();

        // Step 2: 投影到像平面(归一化设备坐标 NDC)
        if (t_cam.z() <= 0.05) {
            // 目标在相机后方或紧贴相机,看不到
            yolo_pub_.publish(out);
            in_fov_pub_.publish(in_fov_flag);
            return;
        }

        double u_ndc = t_cam.x() / t_cam.z();    // tan(水平角)
        double v_ndc = t_cam.y() / t_cam.z();    // tan(垂直角)

        // Step 3: 像素位置(中心 0,原点左上)
        double u_px = (u_ndc * focal_x_) + image_width_ / 2.0;
        double v_px = (v_ndc * focal_y_) + image_height_ / 2.0;

        // Step 4: 矩形判定(注意像素 y 朝下)
        bool u_ok = (u_px >= 0.0 && u_px < image_width_);
        bool v_ok = (v_px >= 0.0 && v_px < image_height_);
        bool visible = u_ok && v_ok;

        if (visible) {
            // 简化的 bbox:以像素位置为中心,宽高 ∝ 1/distance
            // size_px = K / distance(米),默认 K=1000
            double distance = t_cam.norm();
            double size_px = std::min(image_width_, image_height_) * 0.05 +
                             1000.0 / (distance + 1.0);
            size_px = std::min(size_px, std::min(image_width_, image_height_) * 0.4);

            double half_w = size_px / 2.0;
            double half_h = size_px / 2.0;

            out.x_min = std::max(0.0, (u_px - half_w) / image_width_);
            out.y_min = std::max(0.0, (v_px - half_h) / image_height_);
            out.x_max = std::min(1.0, (u_px + half_w) / image_width_);
            out.y_max = std::min(1.0, (v_px + half_h) / image_height_);

            // 置信度随距离衰减,上限 0.95,下限 0.4
            double conf = std::max(0.4, 0.95 - 0.001 * distance);
            out.confidence = (float)conf;
            out.is_in_fov = true;
            in_fov_flag.data = true;
        } else {
            // 在 FOV 之外:清零+invalid
            // 仍发布,让下游知道是 fresh 帧
        }

        yolo_pub_.publish(out);
        in_fov_pub_.publish(in_fov_flag);
    }

    // 目标相对相机的向量(相机坐标系:x=右,y=下,z=前)
    // 输入是世界系(UAV/相机/目标都用同一世界系)
    Eigen::Vector3d targetRelativeToCameraENU() {
        // 取相机世界姿态四元数,逆旋转
        Eigen::Quaterniond q_cam_world(
            camera_pose_world_.pose.orientation.w,
            camera_pose_world_.pose.orientation.x,
            camera_pose_world_.pose.orientation.y,
            camera_pose_world_.pose.orientation.z);
        q_cam_world.normalize();
        Eigen::Matrix3d R_world_cam = q_cam_world.toRotationMatrix();

        // 目标 - 相机位置,转到相机系(逆旋转)
        Eigen::Vector3d world_diff(
            target_enu_.x - camera_pose_world_.pose.position.x,
            target_enu_.y - camera_pose_world_.pose.position.y,
            target_enu_.z - camera_pose_world_.pose.position.z);

        return R_world_cam.transpose() * world_diff;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "detection_simulator_node");
    DetectionSimulator node;
    ros::spin();
    return 0;
}
