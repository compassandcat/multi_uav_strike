/**
 * waypoint_distributor_node.cpp
 * 航点分发器：将地面站航点分别发送给各架无人机
 *
 * 动态发现无人机数量：通过扫描 uav*_wp0_lat_offset 参数自动识别
 *
 * 订阅：
 * - /gs/waypoint_upload  - 地面站原始航点
 *
 * 发布：
 * - /uav0/mission/waypoint_cmd - UAV0 航点
 * - /uav1/mission/waypoint_cmd - UAV1 航点
 * - /uavN/mission/waypoint_cmd - UAVN 航点（N自动发现）
 *
 * 各无人机航点通过 launch 文件参数配置偏移量
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>

class WaypointDistributor {
private:
    ros::NodeHandle nh_;

    struct WaypointOffset {
        double lat_offset;
        double lon_offset;
        double alt;
    };

    struct UavWaypoints {
        std::string name;
        std::vector<WaypointOffset> offsets;
        ros::Publisher pub;
    };
    std::vector<UavWaypoints> uav_list_;

    ros::Subscriber waypoint_sub_;
    double ref_alt_;

public:
    WaypointDistributor() {
        nh_.param<double>("ref_alt", ref_alt_, 100.0);

        // 动态发现无人机
        discoverUavs();

        waypoint_sub_ = nh_.subscribe(
            "gs/waypoint_upload", 10,
            &WaypointDistributor::waypointCallback, this);

        ROS_INFO("[WaypointDistributor] Initialized with %zu UAVs:", uav_list_.size());
    }

    void discoverUavs() {
        // 扫描 uav0 ~ uav9 检查是否配置了航点偏移量
        for (int idx = 0; idx < 10; idx++) {
            std::string name = "uav" + std::to_string(idx);
            std::string key = name + "_wp0_lat_offset";
            double val;
            if (nh_.getParam(key, val)) {
                UavWaypoints uav;
                uav.name = name;
                loadWaypointOffsets(name, uav.offsets);
                uav.pub = nh_.advertise<nav_msgs::Path>(name + "/mission/waypoint_cmd", 10);
                uav_list_.push_back(uav);
                ROS_INFO("[WaypointDistributor]   %s: %zu waypoints", name.c_str(), uav.offsets.size());
            }
        }
    }

    void loadWaypointOffsets(const std::string& prefix, std::vector<WaypointOffset>& offsets) {
        for (int i = 0; i < 10; i++) {
            std::string lat_key = prefix + "_wp" + std::to_string(i) + "_lat_offset";
            std::string lon_key = prefix + "_wp" + std::to_string(i) + "_lon_offset";
            std::string alt_key = prefix + "_wp" + std::to_string(i) + "_alt";

            double lat_offset = 0.0, lon_offset = 0.0, alt = ref_alt_;
            if (!nh_.getParam(lat_key, lat_offset)) {
                break;
            }
            nh_.getParam(lon_key, lon_offset);
            nh_.getParam(alt_key, alt);

            WaypointOffset wo;
            wo.lat_offset = lat_offset;
            wo.lon_offset = lon_offset;
            wo.alt = alt;
            offsets.push_back(wo);
        }
    }

    void waypointCallback(const nav_msgs::Path::ConstPtr& msg) {
        for (size_t i = 0; i < uav_list_.size(); i++) {
            UavWaypoints& uav = uav_list_[i];
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "map";

            for (size_t j = 0; j < uav.offsets.size() && j < msg->poses.size(); j++) {
                geometry_msgs::PoseStamped wp = msg->poses[j];
                wp.pose.position.x += uav.offsets[j].lat_offset;
                wp.pose.position.y += uav.offsets[j].lon_offset;
                wp.pose.position.z = uav.offsets[j].alt;
                path.poses.push_back(wp);
            }

            if (!path.poses.empty()) {
                uav.pub.publish(path);
                ROS_INFO("[WaypointDistributor] Published %zu waypoints to %s",
                         path.poses.size(), uav.name.c_str());
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_distributor_node");
    WaypointDistributor wd;
    ros::spin();
    return 0;
}