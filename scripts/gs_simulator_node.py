#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gs_simulator_node.py
地面站模拟器节点

功能：
- 发布工作模式指令
- 发布测试航点列表
- 支持命令行参数配置

使用方法：
$ rosrun multi_uav_strike gs_simulator_node.py _mode:=IDLE
# 起飞后切换：
$ rostopic pub -1 /gs/mode_cmd std_msgs/String "TAKEOFF"
$ rostopic pub -1 /gs/mode_cmd std_msgs/String "SEARCH_ONLY"

模式约定（与 mission_manager 同步）：
- IDLE        ：飞机在地面等待，不发控制指令
- TAKEOFF     ：执行 PX4 SITL 起飞流程（切 OFFBOARD + 解锁 + 爬升 + 悬停）
- SEARCH_ONLY / SEARCH_TRACK / SEARCH_STRIKE ：任务模式，起飞后切换

航点坐标系约定（重要）：
- 内部 waypoints_local 以"参考 GPS 点为原点的局部 NED 偏移 + 相对高度"为单位
    x = 北向距离 (m)
    y = 东向距离 (m)
    z = 相对高度 (m，向上为正)
- 发布到 /gs/waypoint_upload 前自动通过 ned_to_gps() 转换为 GPS (lat, lon, alt)
- 修改航点时只动 waypoints_local 的米数即可，不用算 1/111000 这种经纬度换算
"""

import rospy
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import sys
import argparse
from math import cos, pi


class GroundStationSimulator:
    def __init__(self, mode="SEARCH_ONLY", auto_pub=True,
                 ref_lat=40.151347, ref_lon=116.980590, ref_alt=31.0):
        self.mode = mode
        self.auto_pub = auto_pub

        # 参考 GPS 点（NED 局部坐标系原点），与 common.yaml 的 ref_* 保持一致
        # 默认值对应北京测试点；切回安阳时改 launch 里 <param> 即可
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt

        # 发布者
        self.mode_pub = rospy.Publisher('gs/mode_cmd', std_msgs.msg.String, queue_size=10)
        self.waypoint_pub = rospy.Publisher('gs/waypoint_upload', nav_msgs.msg.Path, queue_size=10, latch=True)

        # 航点列表（测试用 - 方形航线，单位：米）
        self.waypoints_local = self.create_test_waypoints_local()

        rospy.loginfo("[GS_Simulator] Initialized with mode: %s", self.mode)
        rospy.loginfo("[GS_Simulator] Ref GPS: lat=%.6f, lon=%.6f, alt=%.1f",
                      self.ref_lat, self.ref_lon, self.ref_alt)

    def ned_to_gps(self, north_m, east_m, up_m):
        """局部 NED 偏移 + 相对高度 → GPS (lat, lon, alt)

        Args:
            north_m: 北向距离 (m)
            east_m:  东向距离 (m)
            up_m:    相对高度 (m，向上为正)
        Returns:
            (lat, lon, alt)
        """
        # 1° 纬度 ≈ 111000 m（赤道/两极近似都成立）
        d_lat = north_m / 111000.0
        # 1° 经度 ≈ 111000 * cos(lat) m
        d_lon = east_m / (111000.0 * cos(self.ref_lat * pi / 180.0))
        lat = self.ref_lat + d_lat
        lon = self.ref_lon + d_lon
        # up_m 向上为正；GPS 的 alt 也是向上为正（海拔），直接相加
        alt = self.ref_alt + up_m
        return lat, lon, alt

    def create_test_waypoints_local(self):
        """创建测试航点列表（局部 NED + 高度，单位：米）

        方形航线 (相对参考点)：
            WP1: 正北 10m, 高度 +10m
            WP2: 正北 10m, 正东 10m, 高度 +10m
            WP3:       0, 正东 10m, 高度 +10m
            WP4:       0,       0,   高度 +10m
        修改这里的数字就是改航线，单位是米。
        """
        # (north_m, east_m, up_m)
        waypoints_local = [
            (-10.0,   0.0, 10.0),
            (-10.0,  10.0, 10.0),
            ( 0.0,  10.0, 10.0),
            # ( 0.0,   0.0, 10.0),
        ]
        return waypoints_local

    def local_to_gps_waypoint(self, north_m, east_m, up_m):
        """把局部 NED + 高度航点转成 nav_msgs/Path 里的 PoseStamped（GPS）"""
        lat, lon, alt = self.ned_to_gps(north_m, east_m, up_m)

        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "map"
        # nav_msgs/Path 的 pose.position 约定：x=lat, y=lon, z=alt
        pose.pose.position.x = lat
        pose.pose.position.y = lon
        pose.pose.position.z = alt
        pose.pose.orientation.w = 1.0
        return pose

    def publish_waypoints(self):
        """发布航点列表（先把局部 NED 转成 GPS）"""
        path = nav_msgs.msg.Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        for north_m, east_m, up_m in self.waypoints_local:
            pose = self.local_to_gps_waypoint(north_m, east_m, up_m)
            path.poses.append(pose)
            lat, lon, alt = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            rospy.loginfo(
                "[GS_Simulator] Local (N=%.2f, E=%.2f, Up=%.2f)m -> GPS (lat=%.6f, lon=%.6f, alt=%.1f)",
                north_m, east_m, up_m, lat, lon, alt)

        self.waypoint_pub.publish(path)
        rospy.loginfo("[GS_Simulator] Published %d waypoints", len(path.poses))

    def publish_mode(self, mode):
        """发布工作模式"""
        self.mode = mode
        msg = std_msgs.msg.String()
        msg.data = mode
        self.mode_pub.publish(msg)
        rospy.logwarn("[GS_Simulator] Published mode: %s", mode)

    def run(self):
        """运行循环"""
        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            if self.auto_pub:
                # 首次发布航点（latch=True，后续只发一次）
                if not hasattr(self, '_waypoints_published'):
                    self.publish_waypoints()
                    self._waypoints_published = True
                    # self.publish_mode(self.mode)
                # 持续发布模式（确保订阅者收到）
                

            rate.sleep()

    def set_mode(self, mode):
        """外部设置模式"""
        self.publish_mode(mode)


def main():
    rospy.init_node('gs_simulator_node', anonymous=False)

    # 获取参数
    # 默认 IDLE：飞机在地面等待，不发任何控制指令
    mode = rospy.get_param('~mode', 'IDLE')
    auto_pub = rospy.get_param('~auto_pub', True)

    # 参考 GPS 点（默认值与 common.yaml 当前配置一致：北京）
    # 如需切换到安阳，可在 launch 里覆盖：
    #   <param name="ref_lat" value="36.096" />
    #   <param name="ref_lon" value="114.392" />
    #   <param name="ref_alt" value="100.0" />
    ref_lat = rospy.get_param('~ref_lat', 40.151467)
    ref_lon = rospy.get_param('~ref_lon', 116.979603)
    ref_alt = rospy.get_param('~ref_alt', 31.0)

    rospy.loginfo("[GS_Simulator] Starting with mode=%s, auto_pub=%s", mode, auto_pub)
    rospy.loginfo("[GS_Simulator] Ref GPS: lat=%.6f, lon=%.6f, alt=%.1f", ref_lat, ref_lon, ref_alt)

    simulator = GroundStationSimulator(
        mode=mode, auto_pub=auto_pub,
        ref_lat=ref_lat, ref_lon=ref_lon, ref_alt=ref_alt)
    simulator.run()


if __name__ == '__main__':
    main()