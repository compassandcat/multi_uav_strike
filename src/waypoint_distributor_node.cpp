/**
 * waypoint_distributor_node.cpp
 * (DEPRECATED STUB) — 已移除 /gs/waypoint_upload 订阅:
 *   新地面站不再走 /gs/* String 路径,所有指令均通过 typed TaskFlow 抵达 mission_manager,
 *   本节点只保留 UAV 动态发现 + 空 publisher 占位,等待后续多机协同指令改造时再决定去留。
 *
 * 历史订阅：
 * - /gs/waypoint_upload  - 地面站原始航点(已移除)
 *
 * 历史发布：
 * - /uavN/mission/waypoint_cmd - UAVN 航点(已无数据源,保留 publisher 占位)
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

    double ref_alt_;

public:
    WaypointDistributor() {
        nh_.param<double>("ref_alt", ref_alt_, 100.0);

        // 动态发现无人机(保留 launch 参数扫描机制,确保老的多机 launch 不报错)
        discoverUavs();

        ROS_INFO("[WaypointDistributor] (deprecated stub) Initialized with %zu UAVs:",
                 uav_list_.size());
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

    // 已移除 waypointCallback(): /gs/waypoint_upload 无数据源,数据流改走 typed TaskFlow
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_distributor_node");
    WaypointDistributor wd;
    ros::spin();
    return 0;
}