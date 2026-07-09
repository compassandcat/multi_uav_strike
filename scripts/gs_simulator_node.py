#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gs_simulator_node.py
地面站模拟器节点(只支持 typed TaskFlow,不再发 /gs/* legacy topics)

使用方法:
  $ rosrun multi_uav_strike gs_simulator_node.py _flow:=ground_search_strike _uav_ns:=uav0

可选 flow:
- ground_search_strike / ground_search_track / ground_gather_search / catapult_search_return / ground_search
- b47c_normal / groupb1bc  (真实任务数据,内置 ref_lat/lon 覆盖)

若无 flow 参数(空字符串),节点不发布任何东西,纯哑节点,等待外部 GS 推 TaskFlow。
"""

import rospy
import geometry_msgs.msg
import mavros_msgs.msg
import multi_uav_strike.msg as mus_msg
import sys
import argparse
import time
from math import cos, pi


# ============================================================================
# Phase 2+: 内嵌 DEMO TaskFlows(每套对应一个测试场景)
# ============================================================================
# 坐标系:相对参考点的米(NED 北, NED 东, 上为正高度)
# skill_type: 100=Takeoff, 101=Gather, 102=Search, 103=Return, 105=Attack

DEMO_FLOWS = {
    # ---------- Demo 1: 地面起飞 + 搜索 + 打击 ----------
    # 坐标系: NED (北=+x, 东=+y, 上=+z, 米)
    # 目标位置 NED: (-30, 10, 0)  ← 由 target_motion_simulator.yaml (x_init=-30, y_init=10) 决定
    # 相机方向: pitch=-90°(正下视), FOV 60°,
    #   在高度 z=30m 时地面可视半径 ≈ 30*tan(30°) ≈ 17m,正下视能看到目标
    # 搜索策略: 在 x=-30 这条南北线上飞,从 y=-30 飞到 y=50,会掠过目标 y=10
    "ground_search_strike": {
        "flow_id": "demo_ground_search_strike_001",
        "work_mode": 5,  # SEARCH_STRIKE — 识别即上报并自动打击
        "skills": [
            # 1) Takeoff (地面起飞, takeoff_subtype=0)
            {
                "skill_id": "takeoff_001",
                "skill_type": 100,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],  # 起飞不需 arrive_path
                "skill_area_path": [],
            },
            # 2) Search(到达搜索区,识别目标)
            {
                "skill_id": "search_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (0.0, -30.0, 30.0),   # 原地起飞到 30m 后先飞往搜索线南端上方
                ],
                "skill_area_path": [
                    (-30.0, -30.0, 30.0),  # 搜索线南端 (目标在 y=10,这里在下边)
                    (-30.0, 10.0, 30.0),   # 正对目标 (y=10 与目标 x/y 一致) — 应该在这里看到目标
                    (-30.0, 20.0, 30.0),   # 越过目标继续搜索
                ],
            },
            # 3) Attack(WP1=发现点,WP2=瞄准点,task_speed 是打击速度)
            # 目标点约在 (-30, 10, 0),瞄准点下降到 5m
            {
                "skill_id": "attack_001",
                "skill_type": 105,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 12.0,
                "arrive_path": [],
                "skill_area_path": [
                    (-30.0, 40.0, 30.0),  # WP1 = 发现点(目标上空)
                    (-30.0, 60.0, 0.0),   # WP2 = 瞄准点(下降到低空)
                ],
            },
            # 4) Return(返航)
            {
                "skill_id": "return_001",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),  # 返航点(原点上空)
                ],
            },
        ],
    },

    # ---------- Demo 2: 地面起飞 + 搜索跟踪(等地面站确认) ----------
    "ground_search_track": {
        "flow_id": "demo_ground_search_track_001",
        "work_mode": 4,  # SEARCH_TRACK — 识别后锁定 + 等地面站 attack_cmd 确认
        "skills": [
            {
                "skill_id": "takeoff_001",
                "skill_type": 100,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "search_track_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (50.0, 0.0, 30.0),
                    (50.0, 50.0, 30.0),
                ],
                "skill_area_path": [
                    (50.0, 50.0, 30.0),
                    (50.0, 150.0, 30.0),
                ],
            },
            {
                "skill_id": "return_001",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),
                ],
            },
        ],
    },

    # ---------- Demo 3: 地面起飞 + 集结等待(单机模拟等待另一 UAV) ----------
    "ground_gather_search": {
        "flow_id": "demo_ground_gather_search_001",
        "work_mode": 3,  # SEARCH_ONLY — 纯搜索,只上报不锁不打
        "skills": [
            {
                "skill_id": "takeoff_001",
                "skill_type": 100,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "gather_001",
                "skill_type": 101,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 0.0,  # 集结不需任务速度
                "arrive_path": [
                    (20.0, 0.0, 30.0),
                ],
                "skill_area_path": [
                    (20.0, 0.0, 30.0),  # 集结 HOVER 点
                ],
            },
            {
                "skill_id": "search_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (50.0, 0.0, 30.0),
                ],
                "skill_area_path": [
                    (50.0, 50.0, 30.0),
                    (50.0, 150.0, 30.0),
                ],
            },
            {
                "skill_id": "return_001",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),
                ],
            },
        ],
    },

    # ---------- Demo 4: 弹射起飞 + 搜索 + 返航(强校验:必有后续 skill) ----------
    "catapult_search_return": {
        "flow_id": "demo_catapult_search_return_001",
        "work_mode": 3,  # SEARCH_ONLY — 弹射后纯搜索,只验证强校验链路
        "skills": [
            {
                "skill_id": "takeoff_catapult_001",
                "skill_type": 100,
                "takeoff_subtype": 1,  # 弹射起飞
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "search_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (50.0, 0.0, 30.0),
                ],
                "skill_area_path": [
                    (50.0, 50.0, 30.0),
                    (50.0, 150.0, 30.0),
                ],
            },
            {
                "skill_id": "return_001",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),
                ],
            },
        ],
    },

    # ---------- Demo 5: 地面起飞 + 搜索 (用于调试gimbal和目标定位) ----------
    "ground_search": {
        "flow_id": "demo_ground_search_001",
        "work_mode": 3,  # SEARCH_ONLY — 仅搜索
        "skills": [
            # 1) Takeoff (地面起飞, takeoff_subtype=0)
            {
                "skill_id": "takeoff_001",
                "skill_type": 100,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],  # 起飞不需 arrive_path
                "skill_area_path": [],
            },
            # 2) Search(到达搜索区,识别目标)
            {
                "skill_id": "search_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (0.0, -30.0, 30.0),   # 原地起飞到 30m 后先飞往搜索线南端上方
                ],
                "skill_area_path": [
                    (-30.0, -30.0, 30.0),  # 搜索线南端 (目标在 y=10,这里在下边)
                    (-30.0, 10.0, 30.0),   # 正对目标 (y=10 与目标 x/y 一致) — 应该在这里看到目标
                    (-30.0, 30.0, 30.0),   # 越过目标继续搜索
                ],
            },
        ],
    },

    # ---------- Demo 6: 真实任务数据 — 起飞+集结+搜索+返航 ----------
    # 数据来自真实 GS 下发的 TaskFlow,航点为绝对 GPS 坐标
    # 参考点 (ref) = 起飞点 WP1: (36.0962811N, 114.3922342E, alt=0)
    # 已按 NED 偏移换算,publish_demo_flow 会自动覆盖 self.ref_lat/lon
    # 校验: 用上面 ref 反算回 GPS 应等于原始坐标 (允许 ~0.01m 误差)
    "b47c_normal": {
        "flow_id": "b47c_normal",
        "work_mode": 3,  # SEARCH_ONLY — 纯搜索,无打击
        "ref_lat": 36.0962811,
        "ref_lon": 114.39223419999998,
        "ref_alt": 0.0,
        "skills": [
            # 1) Takeoff — 原始 GPS: WP1=(36.0962811, 114.3922342, 30), WP2=(36.0965128, 114.3926923, 30)
            {
                "skill_id": "9bcd",
                "skill_type": 100,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 0,
                "cruise_speed": 0.0,
                "task_speed": 0.0,
                "arrive_path": [
                    (0.00, 0.00, 30.0),    # WP1 — 起飞点上空 (ref + 30m)
                    (25.71, 41.10, 30.0),  # WP2 — 起飞后过渡点 (约 48m 飞行)
                ],
                "skill_area_path": [],
                "params_json": '{"max_duration":900.0,"takeoff_height":30.0,"takeoff_time_interval":20.0}',
            },
            # 2) Gather — 原始 GPS: WP1=(36.0965128, 114.3926923, 30), WP2=(36.0967201, 114.3928568, 30)
            {
                "skill_id": "a0d6",
                "skill_type": 101,
                "takeoff_subtype": 0,
                "takeoff_altitude": 0.0,
                "priority": 0,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [
                    (25.71, 41.10, 30.0),  # WP1 — 接 takeoff WP2
                    (48.73, 55.86, 30.0),  # WP2 — 集结 HOVER 点
                ],
                "skill_area_path": [],
                "params_json": '{"gather_height":30.0,"gather_speed":5.0,"max_duration":900.0}',
            },
            # 3) Search — 原始 GPS: WP1=(36.0967201, 114.3928568, 30), WP2=(36.0964496, 114.3932106, 30)
            {
                "skill_id": "942b",
                "skill_type": 102,
                "takeoff_subtype": 0,
                "takeoff_altitude": 0.0,
                "priority": 0,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [
                    (48.73, 55.86, 30.0),  # WP1 — 接 gather WP2
                    (18.71, 87.61, 30.0),  # WP2 — 搜索终点
                ],
                "skill_area_path": [],
                "params_json": '{"max_duration":900.0,"safe_distance":10.0,"search_height":30.0,"search_speed":5.0}',
            },
            # 4) Return — 原始 GPS: WP1=(36.0964496, 114.3932106, 30), WP2=(36.0963810, 114.3925973, 30)
            {
                "skill_id": "bd3b",
                "skill_type": 103,
                "takeoff_subtype": 0,
                "takeoff_altitude": 0.0,
                "priority": 0,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [
                    (18.71, 87.61, 30.0),  # WP1 — 接 search WP2
                    (11.09, 32.58, 30.0),  # WP2 — 返航点(接近 home)
                ],
                "skill_area_path": [],
                "params_json": '{"max_duration":900.0,"return_height":30.0,"return_speed":5.0}',
            },
        ],
    },

    # ---------- Demo 7: 真实任务数据 — 纯起飞(单 skill)----------
    # 数据来自真实 GS 下发的 TaskFlow,航点为绝对 GPS 坐标
    # 参考点 (ref) = 起飞点 WP1: (36.0962778N, 114.3922288E, alt=0)
    "groupb1bc": {
        "flow_id": "groupb1bc",
        "work_mode": 3,  # SEARCH_ONLY — 占位,本 flow 只有 takeoff
        "ref_lat": 36.096277799999996,
        "ref_lon": 114.3922288,
        "ref_alt": 0.0,
        "skills": [
            # 1) Takeoff — 原始 GPS: WP1=(36.0962778, 114.3922288, 30), WP2=(36.0960795, 114.3922483, 30)
            {
                "skill_id": "b1bc",
                "skill_type": 100,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 0,
                "cruise_speed": 0.0,
                "task_speed": 0.0,
                "arrive_path": [
                    (0.00, 0.00, 30.0),     # WP1 — 起飞点上空 (ref + 30m)
                    (-22.01, 1.75, 30.0),   # WP2 — 起飞后过渡点 (约 22m 飞行)
                ],
                "skill_area_path": [],
                "params_json": '{"max_duration":900.0,"takeoff_height":30.0,"takeoff_time_interval":20.0}',
            },
        ],
    },
}


class GroundStationSimulator:
    def __init__(self, flow="", uav_ns="", device_id=0,
                 ref_lat=0.0, ref_lon=0.0, ref_alt=0.0):
        self.flow = flow
        self.uav_ns = uav_ns
        self.device_id = device_id

        # 参考 GPS 点(NED 局部坐标系原点):PX4 home 的延迟加载
        # 默认 0/0/0 表示"等 PX4 推 home 过来",见 _home_position_callback
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt

        # === 唯一发布者:typed TaskFlow(打到 /<uav_ns>/mission/task_flow)
        tf_topic = "mission/task_flow"
        if uav_ns:
            tf_topic = "/{}/{}".format(uav_ns, tf_topic)
        self.task_flow_pub = rospy.Publisher(tf_topic, mus_msg.TaskFlow, queue_size=10, latch=True)
        rospy.loginfo("[GS_Simulator] TaskFlow topic: %s", tf_topic)

        # === typed WorkMode 发布器(告诉 mission_manager 任务模式,SEARCH_STRIKE/TRACK/ONLY)
        #   mission_manager 收不到 work_mode 就一直 IDLE,YOLO 命中时不会触发 strike/lock。
        wm_topic = "mission/work_mode"
        if uav_ns:
            wm_topic = "/{}/{}".format(uav_ns, wm_topic)
        self.work_mode_pub = rospy.Publisher(wm_topic, mus_msg.WorkMode, queue_size=10, latch=True)
        rospy.loginfo("[GS_Simulator] WorkMode topic: %s", wm_topic)

        # === 订阅 PX4 home 自动填充 ref_* — 替代 yaml 硬编码 ===
        if uav_ns:
            home_topic = "/{}/mavros/home_position/home".format(uav_ns)
            self.home_sub = rospy.Subscriber(
                home_topic, mavros_msgs.msg.HomePosition,
                self._home_position_callback, queue_size=1)
            rospy.loginfo("[GS_Simulator] Subscribed to PX4 home on %s", home_topic)

        rospy.loginfo("[GS_Simulator] Initialized flow=%s",
                      self.flow or "(none)")

    def _home_position_callback(self, msg):
        """PX4 home 一次性锁定 — 第一帧有效数据后写入 ref_lat/lon/alt"""
        lat = msg.geo.latitude
        lon = msg.geo.longitude
        alt = msg.geo.altitude
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return  # PX4 home 还没稳定(发 0/0)
        # 第一次有效帧写入;之后再收也不再覆盖
        self.ref_lat = lat
        self.ref_lon = lon
        self.ref_alt = alt
        rospy.logwarn("[GS_Simulator] >>>> PX4 home loaded: lat=%.7f lon=%.7f alt=%.2f",
                        self.ref_lat, self.ref_lon, self.ref_alt)
        rospy.loginfo("[GS_Simulator] Ref GPS: lat=%.6f, lon=%.6f, alt=%.1f",
                      self.ref_lat, self.ref_lon, self.ref_alt)

    def ned_to_gps(self, north_m, east_m, up_m):
        """局部 NED 偏移 + 相对高度 → GPS (lat, lon, alt)"""
        d_lat = north_m / 111000.0
        d_lon = east_m / (111000.0 * cos(self.ref_lat * pi / 180.0))
        lat = self.ref_lat + d_lat
        lon = self.ref_lon + d_lon
        alt = up_m
        return lat, lon, alt

    def local_to_gps_waypoint(self, north_m, east_m, up_m):
        """把局部 NED + 高度航点转成 PoseStamped(GPS)"""
        lat, lon, alt = self.ned_to_gps(north_m, east_m, up_m)
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = lat
        pose.pose.position.y = lon
        pose.pose.position.z = alt
        pose.pose.orientation.w = 1.0
        return pose

    # ============== typed TaskFlow(唯一的 GS 输出通道)==============

    def build_task_flow(self, flow_name):
        """根据 flow_name 构造 multi_uav_strike/TaskFlow 消息"""
        if flow_name not in DEMO_FLOWS:
            rospy.logerr("[GS_Simulator] Unknown flow '%s', available: %s",
                         flow_name, list(DEMO_FLOWS.keys()))
            return None

        cfg = DEMO_FLOWS[flow_name]
        flow = mus_msg.TaskFlow()
        flow.flow_id = cfg["flow_id"]
        flow.issued_at_us = int(time.time() * 1e6)
        flow.device_id = self.device_id

        for skill_cfg in cfg["skills"]:
            s = mus_msg.Skill()
            s.skill_id = skill_cfg["skill_id"]
            s.skill_type = skill_cfg["skill_type"]
            s.takeoff_subtype = skill_cfg.get("takeoff_subtype", 0)
            s.takeoff_altitude = float(skill_cfg.get("takeoff_altitude", 0.0))
            s.priority = skill_cfg.get("priority", 100)
            s.cruise_speed = float(skill_cfg.get("cruise_speed", 8.0))
            s.task_speed = float(skill_cfg.get("task_speed", 8.0))
            s.formation = skill_cfg.get("formation", 0)
            s.params_json = skill_cfg.get("params_json", "")

            # arrive_path
            for north_m, east_m, up_m in skill_cfg.get("arrive_path", []):
                pose = self.local_to_gps_waypoint(north_m, east_m, up_m)
                s.arrive_path.poses.append(pose)
            s.arrive_path.header.frame_id = "map"
            s.arrive_path.header.stamp = rospy.Time.now()

            # skill_area_path
            for north_m, east_m, up_m in skill_cfg.get("skill_area_path", []):
                pose = self.local_to_gps_waypoint(north_m, east_m, up_m)
                s.skill_area_path.poses.append(pose)
            s.skill_area_path.header.frame_id = "map"
            s.skill_area_path.header.stamp = rospy.Time.now()

            flow.skills.append(s)

        return flow

    def publish_demo_flow(self, flow_name):
        """发布选中的 demo flow"""
        flow = self.build_task_flow(flow_name)
        if flow is None:
            return

        cfg = DEMO_FLOWS[flow_name]
        work_mode = cfg.get("work_mode", 3)  # 默认 SEARCH_ONLY

        # === flow 内置 ref 覆盖 ===
        # 部分真实任务数据 (b47c_normal / groupb1bc) 的航点是绝对 GPS,
        # 参考点 = 该 flow takeoff 第一个航点,需要先覆盖 self.ref_* 再发布
        # 否则 NED→GPS 转换会基于 PX4 home,产生几百米的系统性偏差
        cfg_ref_lat = cfg.get("ref_lat")
        cfg_ref_lon = cfg.get("ref_lon")
        if cfg_ref_lat is not None and cfg_ref_lon is not None:
            self.ref_lat = cfg_ref_lat
            self.ref_lon = cfg_ref_lon
            if "ref_alt" in cfg:
                self.ref_alt = cfg["ref_alt"]
            # 重建 flow — 上面 build_task_flow 用的还是旧 ref_lat/lon
            flow = self.build_task_flow(flow_name)
            rospy.logwarn("[GS_Simulator] >>>> Flow ref override: lat=%.7f lon=%.7f alt=%.2f (rebuilt flow)",
                          self.ref_lat, self.ref_lon, self.ref_alt)

        rospy.logwarn("[GS_Simulator] >>>> Publishing demo flow '%s' (id=%s, %lu skills, work_mode=%u)",
                      flow_name, flow.flow_id, len(flow.skills), work_mode)
        for i, s in enumerate(flow.skills):
            rospy.logwarn("[GS_Simulator]   Skill[%d]: id=%s type=%u arrive=%lu skill=%lu task_speed=%.1f",
                          i, s.skill_id, s.skill_type,
                          len(s.arrive_path.poses), len(s.skill_area_path.poses),
                          s.task_speed)

        # === 先发 WorkMode,再发 TaskFlow ===
        # mission_manager 收到 WorkMode 时,typedWorkModeCallback 会立即把 current_work_mode_
        # 设成 SEARCH_STRIKE/SEARCH_TRACK/SEARCH_ONLY(对应 handleSearch* 分支)
        # 后续 YOLO 命中会按 work_mode 走对应路径:SEARCH_STRIKE 自动 triggerStrike()、
        # SEARCH_TRACK 等 attack_cmd、SEARCH_ONLY 只上报 DetectTarget。
        # WorkMode 必须在 TaskFlow 之前发,避免 mission_manager 在 first skill 是 takeoff 时
        # 误把 work_mode 缓存为 IDLE 后丢失(虽然当前 taskFlowCallback 不再覆盖 work_mode,
        # 但 typed WorkMode 先到位可让 post_takeoff_work_mode_ 兜底时不覆盖用户意图)。
        wm_msg = mus_msg.WorkMode()
        wm_msg.mode = work_mode
        self.work_mode_pub.publish(wm_msg)
        rospy.logwarn("[GS_Simulator] >>>> WorkMode=%u published.", work_mode)

        # 等 1s 让 subscriber 先建好连接
        rospy.sleep(1.0)
        self.task_flow_pub.publish(flow)
        rospy.logwarn("[GS_Simulator] >>>> Demo flow published.")

    def run(self):
        """运行:发了 flow 就退,空 flow 就 idle 等外部 GS"""
        if not self.flow:
            rospy.loginfo("[GS_Simulator] No flow param, idle and wait for external TaskFlow")
            rospy.spin()
            return

        rospy.loginfo("[GS_Simulator] Waiting 3s before publishing flow '%s'...", self.flow)
        rospy.sleep(3.0)
        self.publish_demo_flow(self.flow)
        rospy.loginfo("[GS_Simulator] Flow published, idle.")


def main():
    rospy.init_node('gs_simulator_node', anonymous=False)

    flow = rospy.get_param('~flow', '')             # demo flow 名称(空=等外部 GS)
    uav_ns = rospy.get_param('~uav_ns', '')         # UAV 命名空间(如 "uav0")
    device_id = rospy.get_param('~device_id', 0)    # 0=广播
    
    ref_lat = rospy.get_param('~ref_lat', 0.0)
    ref_lon = rospy.get_param('~ref_lon', 0.0)
    ref_alt = rospy.get_param('~ref_alt', 0.0)

    rospy.loginfo("[GS_Simulator] Starting flow=%s",
                  flow or "(none)")
    rospy.loginfo("[GS_Simulator] Ref GPS: lat=%.6f, lon=%.6f, alt=%.1f",
                  ref_lat, ref_lon, ref_alt)

    simulator = GroundStationSimulator(
        flow=flow, uav_ns=uav_ns, device_id=device_id,
        ref_lat=ref_lat, ref_lon=ref_lon, ref_alt=ref_alt)
    simulator.run()


if __name__ == '__main__':
    main()