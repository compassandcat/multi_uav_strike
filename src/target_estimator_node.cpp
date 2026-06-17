#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <random>
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>

struct Particle {
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    double weight;

    Particle() : x(0.0), y(0.0), z(0.0), vx(0.0), vy(0.0), vz(0.0), weight(1.0) {}
    Particle(double x_, double y_, double z_, double vx_, double vy_, double vz_, double w_) 
        : x(x_), y(y_), z(z_), vx(vx_), vy(vy_), vz(vz_), weight(w_) {}
};

class TargetEstimator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gimbal_los_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Subscriber real_target_sub_;  // 真实目标位置（调试用）
    ros::Publisher target_est_marker_pub_;
    ros::Publisher particles_marker_pub_;
    ros::Publisher target_est_pose_pub_;
    ros::Publisher target_est_twist_pub_;
    ros::Timer pf_timer_;

    int num_particles_;
    double init_dist_std_dev_;
    double process_noise_std_dev_;
    double observation_noise_std_dev_;
    double min_confidence_;
    double pf_loop_freq_;
    double uav_height_for_init_;
    double dist_prior_weight_;
    double optimal_observe_dist_;
    double vel_consistency_weight_;
    double guide_vel_gain_;
    // 新增：目标高度先验相关（解决距离坍缩核心参数）
    double target_z_prior_;        // 目标高度先验值（如地面目标设0.0，空中目标设具体值）
    double target_z_weight_;       // 高度误差的权重（控制高度约束的强度）
    double angle_error_dist_gain_; // 角度误差的距离惩罚系数（放大远距粒子的角度误差）

    // 仿真/真机切换
    bool use_sim_;
    std::string uav_pose_topic_;

    // 新增：估计目标marker颜色参数（launch可配置）
    double est_marker_r_;    // 红色通道 [0,1]
    double est_marker_g_;    // 绿色通道 [0,1]
    double est_marker_b_;    // 蓝色通道 [0,1]
    double est_marker_a_;    // 透明度 [0,1]，建议1.0

    std::vector<Particle> particles_;
    geometry_msgs::Point current_los_angle_;
    geometry_msgs::PoseStamped current_uav_pose_;
    geometry_msgs::Point real_target_pos_;  // 真实目标位置（调试用）
    bool is_real_target_received_ = false;
    bool is_los_received_ = false;
    bool is_uav_pose_received_ = false;
    bool is_particles_initialized_ = false;
    double tracking_accuracy_filter = 0.0;  // 初始化为1.0，避免启动时收敛慢
    double avg_particle_dist_;

    std::default_random_engine rng_;
    std::normal_distribution<double> normal_dist_;

    geometry_msgs::PoseStamped estimated_target_pose_;
    geometry_msgs::TwistStamped estimated_target_velocity_;

public:
    TargetEstimator() {
        ros::NodeHandle n_param("~");
        n_param.param<int>("num_particles", num_particles_, 500);
        n_param.param<double>("init_dist_std_dev", init_dist_std_dev_, 1.0);
        n_param.param<double>("process_noise_std_dev", process_noise_std_dev_, 0.2);
        n_param.param<double>("observation_noise_std_dev", observation_noise_std_dev_, 0.1);
        n_param.param<double>("min_confidence", min_confidence_, 0.2);
        n_param.param<double>("pf_loop_freq", pf_loop_freq_, 50.0);
        n_param.param<double>("uav_height_for_init", uav_height_for_init_, 10.0);
        n_param.param<double>("dist_prior_weight", dist_prior_weight_, 0.3);
        n_param.param<double>("optimal_observe_dist", optimal_observe_dist_, 20.0);
        n_param.param<double>("vel_consistency_weight", vel_consistency_weight_, 0.3);
        // 新增：目标高度先验+距离惩罚参数（解决粒子坍缩）
        n_param.param<double>("target_z_prior", target_z_prior_, 0.0);        // 默认地面目标，高度0
        n_param.param<double>("target_z_weight", target_z_weight_, 0.1);      // 高度约束权重，0~1
        n_param.param<double>("angle_error_dist_gain", angle_error_dist_gain_, 0.01); // 距离惩罚系数，按需调整
        // 新增：读取估计目标marker颜色参数（launch可配置）
        n_param.param<double>("est_marker_r", est_marker_r_, 1.0);
        n_param.param<double>("est_marker_g", est_marker_g_, 0.0);
        n_param.param<double>("est_marker_b", est_marker_b_, 0.0);
        n_param.param<double>("est_marker_a", est_marker_a_, 1.0);
        // 颜色值限幅[0,1]，防止传参错误
        est_marker_r_ = std::max(0.0, std::min(1.0, est_marker_r_));
        est_marker_g_ = std::max(0.0, std::min(1.0, est_marker_g_));
        est_marker_b_ = std::max(0.0, std::min(1.0, est_marker_b_));
        est_marker_a_ = std::max(0.0, std::min(1.0, est_marker_a_));

        // 仿真/真机切换
        n_param.param<bool>("use_sim", use_sim_, true);
        if (use_sim_) {
            uav_pose_topic_ = "quad/pose";
        } else {
            uav_pose_topic_ = "mavros/local_position/pose";
        }

        unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
        rng_.seed(seed);
        normal_dist_ = std::normal_distribution<double>(0.0, 1.0);

        gimbal_los_sub_ = nh_.subscribe("target_los_angle", 10, &TargetEstimator::gimbalLosCallback, this);
        uav_pose_sub_ = nh_.subscribe(uav_pose_topic_, 10, &TargetEstimator::uavPoseCallback, this);
        real_target_sub_ = nh_.subscribe("/target_position", 10, &TargetEstimator::realTargetCallback, this);

        target_est_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("target_estimated_marker", 10);
        particles_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("particles_marker_array", 10);
        target_est_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_estimated_pose", 10);
        target_est_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("target_estimated_twist", 10);

        pf_timer_ = nh_.createTimer(ros::Duration(1.0/pf_loop_freq_), &TargetEstimator::pfLoopCallback, this);

        ROS_INFO("Target Estimator (Particle Filter) initialized!");
        ROS_INFO("PF Params: num_particles=%d, init_dist_std=%.2fm, process_noise=%.2fm/s, obs_noise=%.2frad",
                 num_particles_, init_dist_std_dev_, process_noise_std_dev_, observation_noise_std_dev_);
        ROS_INFO("New Params: dist_prior=%.2f, vel_consistency=%.2f",
                 dist_prior_weight_, vel_consistency_weight_);
        ROS_INFO("Subscribed to: target_los=%s, uav_pose=%s", "/target_los_angle", uav_pose_topic_.c_str());
        ROS_INFO("Publishing to: target_est_marker=%s, particles=%s, target_est_pose=%s",
                 "/target_estimated_marker", "/particles_marker_array", "/target_estimated_pose");
        ROS_INFO("Anti-collapse Params: target_z_prior=%.2fm, z_weight=%.2f, angle_dist_gain=%.3f",
         target_z_prior_, target_z_weight_, angle_error_dist_gain_);
         //打印颜色参数
        ROS_INFO("Est Marker Color: R=%.2f, G=%.2f, B=%.2f, A=%.2f",
                est_marker_r_, est_marker_g_, est_marker_b_, est_marker_a_);
    }

    void gimbalLosCallback(const geometry_msgs::Point::ConstPtr& msg) {
        current_los_angle_ = *msg;
        // 加快收敛速度：0.9代替0.95，30次迭代可达95%收敛
        tracking_accuracy_filter = tracking_accuracy_filter * 0.9 + current_los_angle_.z * 0.1;
        is_los_received_ = true;
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (use_sim_) {
            // ===== NED → NWU 坐标转换 =====
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
            // 合成 ENU -> NWU: x = y_enu, y = -x_enu, z = z_enu
            current_uav_pose_.pose.position.x = msg->pose.position.y;
            current_uav_pose_.pose.position.y = -msg->pose.position.x;
            current_uav_pose_.pose.position.z = msg->pose.position.z;

            // 四元数: ENU->NED (180°绕X) + NED->NWU (180°绕X) = 恒等
            current_uav_pose_.pose.orientation.w = msg->pose.orientation.w;
            current_uav_pose_.pose.orientation.x = msg->pose.orientation.x;
            current_uav_pose_.pose.orientation.y = -msg->pose.orientation.y;
            current_uav_pose_.pose.orientation.z = -msg->pose.orientation.z;
        }

        current_uav_pose_.header.stamp = msg->header.stamp;
        current_uav_pose_.header.frame_id = msg->header.frame_id;
        is_uav_pose_received_ = true;
    }

    void realTargetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        // 真实目标位置（NWU坐标系）
        real_target_pos_ = *msg;
        is_real_target_received_ = true;
    }

    void pfLoopCallback(const ros::TimerEvent&) {
        if (!is_los_received_ || !is_uav_pose_received_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for gimbal LOS or UAV pose data...");
            return;
        }

        if (!is_particles_initialized_){
            if (tracking_accuracy_filter > 0.9) {
                initializeParticles();
                is_particles_initialized_ = true;
                ROS_INFO("Particles initialized! Total particles: %d", num_particles_);
            }
            return;
        }
        
        if(tracking_accuracy_filter < 0.5){
            ROS_WARN_THROTTLE(1.0, "[PF] Tracking accuracy low: filter=%.4f, los_z=%.4f, not updating.", tracking_accuracy_filter, current_los_angle_.z);
            return;
        }

        predictParticles();
        updateParticleWeights();
        resampleParticles();
        estimateTargetState();

        publishEstimatedTargetMarker();
        publishParticlesMarkerArray();
        publishEstimatedTargetPose();
        publishEstimatedTargetTwist();

        // 调试：对比估计位置与真实位置（只在误差超过阈值时输出）
        if (is_real_target_received_) {
            double est_x = estimated_target_pose_.pose.position.x;
            double est_y = estimated_target_pose_.pose.position.y;
            double est_z = estimated_target_pose_.pose.position.z;
            double real_x = real_target_pos_.x;
            double real_y = real_target_pos_.y;
            double real_z = real_target_pos_.z;
            double error_x = est_x - real_x;
            double error_y = est_y - real_y;
            double error_z = est_z - real_z;

            // 水平误差超过10米时才打印
            double horiz_error = sqrt(error_x*error_x + error_y*error_y);
            if (horiz_error > 10.0) {
                ROS_DEBUG_THROTTLE(1.0, "[PF] Large tracking error: %.1fm (est_z=%.2f)", horiz_error, error_z);
                ROS_DEBUG_THROTTLE(1.0, "Estimated target: x=%.2f, y=%.2f, z=%.2f (particles num: %lu)",
                           estimated_target_pose_.pose.position.x,
                           estimated_target_pose_.pose.position.y,
                           estimated_target_pose_.pose.position.z,
                           particles_.size());
            }
        }
    }

    void initializeParticles() {
        particles_.clear();
        particles_.reserve(num_particles_);

        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;

        double pitch = current_los_angle_.y;
        double yaw = current_los_angle_.x;

        // 检查pitch是否过小，避免距离计算发散
        if (fabs(sin(pitch)) < 0.01) {
            ROS_ERROR("[INIT] Pitch too small (%.4f), cannot calculate init distance!", pitch);
            return;
        }

        double init_dist = uav_z / fabs(sin(pitch));

        // 添加最大初始化距离限制，避免gimbal未锁定时产生巨大距离
        double max_init_dist = 1000.0;  // 最大1000米，对于地面目标足够
        if (init_dist > max_init_dist) {
            ROS_WARN("[INIT] Init dist (%.2f) > max (%.2f), using default range", init_dist, max_init_dist);
            init_dist = max_init_dist;
        }

        ROS_WARN("[INIT] UAV: (%.2f, %.2f, %.2f), yaw=%.2f, pitch=%.2f, dist=%.2f",
                 uav_x, uav_y, uav_z, yaw, pitch, init_dist);

        double target_x_init = uav_x + init_dist * cos(pitch) * cos(yaw);
        double target_y_init = uav_y + init_dist * cos(pitch) * sin(yaw);
        double target_z_init = target_z_prior_; // 优先使用目标高度先验
        ROS_WARN("[INIT] Target init: (%.2f, %.2f, %.2f)", target_x_init, target_y_init, target_z_init);

        for (int i = 0; i < num_particles_; ++i) {
            double x_noise = normal_dist_(rng_) * init_dist_std_dev_;
            double y_noise = normal_dist_(rng_) * init_dist_std_dev_;
            double z_noise = normal_dist_(rng_) * init_dist_std_dev_ * 0.5;

            double vx = normal_dist_(rng_) * 0.5;
            double vy = normal_dist_(rng_) * 0.5;
            double vz = normal_dist_(rng_) * 0.1;

            Particle p(
                target_x_init + x_noise,
                target_y_init + y_noise,
                target_z_init + z_noise,
                vx, vy, vz,
                1.0 / num_particles_
            );
            particles_.push_back(p);
        }

        calculateAvgParticleDist();

        // Debug: 打印初始粒子分布
        double px_mean = 0, py_mean = 0, pz_mean = 0;
        for (const auto& p : particles_) {
            px_mean += p.x;
            py_mean += p.y;
            pz_mean += p.z;
        }
        px_mean /= num_particles_;
        py_mean /= num_particles_;
        pz_mean /= num_particles_;
        ROS_WARN("[INIT] Particles initialized: count=%d, mean=(%.2f, %.2f, %.2f), avg_dist=%.2f",
                 num_particles_, px_mean, py_mean, pz_mean, avg_particle_dist_);
    }

    void calculateAvgParticleDist() {
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;
        double sum_dist = 0.0;

        for (const auto& p : particles_) {
            double dx = p.x - uav_x;
            double dy = p.y - uav_y;
            double dz = p.z - uav_z;
            sum_dist += sqrt(dx*dx + dy*dy + dz*dz);
        }

        avg_particle_dist_ = sum_dist / num_particles_;
    }

    void predictParticles() {
        double dt = 1.0 / pf_loop_freq_;

        for (auto& p : particles_) {
            p.x += p.vx * dt;
            p.y += p.vy * dt;
            p.z += p.vz * dt;

            p.vx += normal_dist_(rng_) * process_noise_std_dev_;
            p.vy += normal_dist_(rng_) * process_noise_std_dev_;
            p.vz += normal_dist_(rng_) * process_noise_std_dev_ * 0.5;

            double max_speed = 5.0;
            p.vx = std::max(-max_speed, std::min(p.vx, max_speed));
            p.vy = std::max(-max_speed, std::min(p.vy, max_speed));
            p.vz = std::max(-1.0, std::min(p.vz, 1.0));

            // 对于地面目标，强制将z拉向target_z_prior_
            // 使用较大的吸引力，防止粒子漂移到空中
            double z_pull_strength = 3.0;  // 越大越强制拉回先验高度
            double z_error = p.z - target_z_prior_;
            p.vz -= z_pull_strength * z_error * dt;
            // 限制vz，防止过冲
            p.vz = std::max(-2.0, std::min(p.vz, 2.0));

            // 限制粒子位置不发散太远
            double max_dist = 500.0;
            double dist = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            if (dist > max_dist) {
                p.x *= max_dist / dist;
                p.y *= max_dist / dist;
                p.z *= max_dist / dist;
            }
            // 对于地面目标，限制z在合理范围
            p.z = std::max(-5.0, std::min(p.z, 50.0));
        }
    }

    void updateParticleWeights() {
        double total_weight = 0.0;
        double confidence = current_los_angle_.z;

        // 安全检查：检测NaN
        bool confidence_valid = !(std::isnan(confidence) || std::isnan(tracking_accuracy_filter));

        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;
        double obs_yaw = current_los_angle_.x;
        double obs_pitch = current_los_angle_.y;

        // 计算加权平均速度
        double avg_vx = 0.0, avg_vy = 0.0, avg_vz = 0.0;
        for (const auto& p : particles_) {
            avg_vx += p.vx * p.weight;
            avg_vy += p.vy * p.weight;
            avg_vz += p.vz * p.weight;
        }

        for (auto& p : particles_) {
            if (!confidence_valid || confidence < min_confidence_) {
                p.weight = 1.0 / num_particles_;
            } else {
                double dx = p.x - uav_x;
                double dy = p.y - uav_y;
                double dz = p.z - uav_z;  // NED: target_z - uav_z (positive = below)
                double dist = sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < 1e-3) {
                    p.weight = 0.0;
                    continue;
                }

                // 1. 计算角度误差
                double pred_yaw = atan2(dy, dx);
                double pred_pitch = atan2(dz, sqrt(dx*dx + dy*dy));
                double yaw_error = atan2(sin(obs_yaw - pred_yaw), cos(obs_yaw - pred_yaw));
                double pitch_error = atan2(sin(obs_pitch - pred_pitch), cos(obs_pitch - pred_pitch));
                double raw_angle_error = yaw_error*yaw_error + pitch_error*pitch_error;

                // 距离加权放大角度误差（解决远距粒子坍缩）
                double weighted_angle_error = raw_angle_error * (1.0 + angle_error_dist_gain_ * dist);
                double angle_weight = exp(-weighted_angle_error/(2*observation_noise_std_dev_*observation_noise_std_dev_));

                // 2. 高度先验权重（地面目标 z=0）
                double z_error = fabs(p.z - target_z_prior_);
                double z_weight = std::max(0.0, 1.0 - z_error / 5.0);  // 5m内高权重

                // 3. 速度一致性权重（惩罚偏离平均速度的粒子）
                double vel_error = sqrt(pow(p.vx - avg_vx, 2) + pow(p.vy - avg_vy, 2) + pow(p.vz - avg_vz, 2));
                double vel_weight = std::max(0.0, 1.0 - vel_error / 5.0);  // 5m/s内高权重

                // 加权融合：角度为主，高度+速度为约束
                p.weight = angle_weight * (0.6 + 0.4 * z_weight) * (0.8 + 0.2 * vel_weight);

                // NaN/Inf安全检查
                if (std::isnan(p.weight) || std::isinf(p.weight) || p.weight < 1e-20) {
                    p.weight = 1.0 / num_particles_;
                }
            }
            total_weight += p.weight;
        }

        // 权重归一化
        if (std::isnan(total_weight) || std::isinf(total_weight) || total_weight < 1e-6) {
            for (auto& p : particles_) {
                p.weight = 1.0 / num_particles_;
            }
            total_weight = 1.0;
        } else {
            for (auto& p : particles_) {
                p.weight /= total_weight;
            }
        }
    }

    // 修正后的重采样函数
    void resampleParticles() {
        // 计算有效粒子数
        double sum_w_sq = 0.0;
        for (const auto& p : particles_) {
            sum_w_sq += p.weight * p.weight;
        }
        double effective_n = 1.0 / sum_w_sq;

        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);

        std::vector<double> cum_weights;
        cum_weights.reserve(num_particles_);
        double cum_sum = 0.0;
        for (const auto& p : particles_) {
            cum_sum += p.weight;
            cum_weights.push_back(cum_sum);
        }

        for (int i = 0; i < num_particles_; ++i) {
            double rand_val = ((double)rand() / RAND_MAX) * cum_sum;
            auto it = std::lower_bound(cum_weights.begin(), cum_weights.end(), rand_val);
            int idx = std::distance(cum_weights.begin(), it);
            if (idx >= num_particles_) idx = num_particles_ - 1;

            // 修正：避免重复声明，直接复制粒子
            Particle new_p = particles_[idx];
            // 适度扰动
            new_p.x += normal_dist_(rng_) * 0.2;
            new_p.y += normal_dist_(rng_) * 0.2;
            new_p.z += normal_dist_(rng_) * 0.1;
            new_p.weight = 1.0 / num_particles_;
            new_particles.push_back(new_p);
        }

        particles_ = std::move(new_particles);
    }

    void estimateTargetState() {
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        double sum_vx = 0.0, sum_vy = 0.0, sum_vz = 0.0;
        double sum_weight = 0.0;

        for (const auto& p : particles_) {
            sum_x += p.x * p.weight;
            sum_y += p.y * p.weight;
            sum_z += p.z * p.weight;
            sum_vx += p.vx * p.weight;
            sum_vy += p.vy * p.weight;
            sum_vz += p.vz * p.weight;
            sum_weight += p.weight;
        }

        if (sum_weight > 1e-6) {
            estimated_target_pose_.pose.position.x = sum_x / sum_weight;
            estimated_target_pose_.pose.position.y = sum_y / sum_weight;
            estimated_target_pose_.pose.position.z = sum_z / sum_weight;

            estimated_target_velocity_.twist.linear.x = sum_vx / sum_weight;
            estimated_target_velocity_.twist.linear.y = sum_vy / sum_weight;
            estimated_target_velocity_.twist.linear.z = sum_vz / sum_weight;
        }

        estimated_target_pose_.pose.orientation.x = 0.0;
        estimated_target_pose_.pose.orientation.y = 0.0;
        estimated_target_pose_.pose.orientation.z = 0.0;
        estimated_target_pose_.pose.orientation.w = 1.0;

        estimated_target_pose_.header.stamp = ros::Time::now();
        estimated_target_pose_.header.frame_id = "map";

        estimated_target_velocity_.header.stamp = ros::Time::now();
        estimated_target_velocity_.header.frame_id = "map";

        // ROS_WARN_THROTTLE(1.0, "[EST] target_pos=(%.2f, %.2f, %.2f), target_vel=(%.2f, %.2f, %.2f)",
        //                   estimated_target_pose_.pose.position.x,
        //                   estimated_target_pose_.pose.position.y,
        //                   estimated_target_pose_.pose.position.z,
        //                   estimated_target_velocity_.twist.linear.x,
        //                   estimated_target_velocity_.twist.linear.y,
        //                   estimated_target_velocity_.twist.linear.z);
    }

    void publishEstimatedTargetMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "estimated_target";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = estimated_target_pose_.pose.position;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        // ========== 核心修改：从ROS参数读取颜色 ==========
        marker.color.r = est_marker_r_;
        marker.color.g = est_marker_g_;
        marker.color.b = est_marker_b_;
        marker.color.a = est_marker_a_;
        // ==================================================
        marker.lifetime = ros::Duration(0);
        target_est_marker_pub_.publish(marker);
    }

    void publishParticlesMarkerArray() {
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        for (const auto& p : particles_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "particles";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = p.x;
            marker.pose.position.y = p.y;
            marker.pose.position.z = p.z;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.5;

            marker.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(marker);
        }

        particles_marker_pub_.publish(marker_array);
    }

    void publishEstimatedTargetPose() {
        target_est_pose_pub_.publish(estimated_target_pose_);
    }

    void publishEstimatedTargetTwist() {
        target_est_twist_pub_.publish(estimated_target_velocity_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_estimator_node");
    TargetEstimator estimator;
    ros::spin();
    return 0;
}