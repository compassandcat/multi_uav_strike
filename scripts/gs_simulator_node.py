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
$ rosrun multi_uav_strike gs_simulator_node.py _mode:=SEARCH_STRIKE
"""

import rospy
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import sys
import argparse
from math import cos, pi


class GroundStationSimulator:
    def __init__(self, mode="SEARCH_ONLY", auto_pub=True):
        self.mode = mode
        self.auto_pub = auto_pub

        # 发布者
        self.mode_pub = rospy.Publisher('gs/mode_cmd', std_msgs.msg.String, queue_size=10)
        self.waypoint_pub = rospy.Publisher('gs/waypoint_upload', nav_msgs.msg.Path, queue_size=10, latch=True)

        # 航点列表（测试用 - 方形航线）
        self.waypoints = self.create_test_waypoints()

        rospy.loginfo("[GS_Simulator] Initialized with mode: %s", self.mode)

    def create_test_waypoints(self):
        """创建测试航点列表（GPS 格式）

        GPS 格式说明：
        - x = 纬度 (lat)
        - y = 经度 (lon)
        - z = 高度 (alt，NED向下为正，所以是相对高度)

        基准点：安阳 (36.096, 114.392, 100.0)
        航点为相对于基准点的偏移（约10m间距，方便调试）
        """
        waypoints = []

        # 基准点: 安阳 (lat=36.096, lon=114.392, alt=100m)
        # 航点1: 基准点正北 10m 处
        lat1 = 36.096 + 10.0 / 111000.0  # 约 0.00009 度
        lon1 = 114.392
        alt1 = 110.0

        # 航点2: 继续正东 10m
        lat2 = lat1
        lon2 = 114.392 + 10.0 / (111000.0 * cos(36.096 * pi / 180.0))
        alt2 = 110.0

        # 航点3: 继续正南 10m
        lat3 = 36.096
        lon3 = lon2
        alt3 = 110.0

        # 航点4: 回正西 10m
        lat4 = 36.096
        lon4 = 114.392
        alt4 = 110.0

        test_points = [
            (lat1, lon1, alt1),
            (lat2, lon2, alt2),
            (lat3, lon3, alt3),
            (lat4, lon4, alt4),
        ]

        for lat, lon, alt in test_points:
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = lat
            pose.pose.position.y = lon
            pose.pose.position.z = alt
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)
            rospy.loginfo("[GS_Simulator] Waypoint: lat=%.6f, lon=%.6f, alt=%.1f", lat, lon, alt)

        return waypoints

    def publish_waypoints(self):
        """发布航点列表"""
        path = nav_msgs.msg.Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = self.waypoints

        self.waypoint_pub.publish(path)
        rospy.loginfo("[GS_Simulator] Published %d waypoints", len(self.waypoints))

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

                # 持续发布模式（确保订阅者收到）
                self.publish_mode(self.mode)

            rate.sleep()

    def set_mode(self, mode):
        """外部设置模式"""
        self.publish_mode(mode)


def main():
    rospy.init_node('gs_simulator_node', anonymous=False)

    # 获取参数
    mode = rospy.get_param('~mode', 'SEARCH_ONLY')
    auto_pub = rospy.get_param('~auto_pub', True)

    rospy.loginfo("[GS_Simulator] Starting with mode=%s, auto_pub=%s", mode, auto_pub)

    simulator = GroundStationSimulator(mode=mode, auto_pub=auto_pub)
    simulator.run()


if __name__ == '__main__':
    main()
