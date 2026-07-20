---
name: multi_uav_strike_user
description: 用户是多无人机救援系统开发者，负责粒子滤波定位和制导逻辑
type: user
---

## 用户背景
- 多无人机救援系统开发者（ROS1）
- 负责模块：粒子滤波目标定位（已实现）、制导逻辑（已实现 LOS/Intercept/MinSnap 三种策略）
- 其他模块由其他人负责：YOLO、云台驱动、雷达驱动、FastLio2、3D A*、视频推流、开机脚本
- **部署平台**：RK3588J 机载计算机（算力有限，需控制计算开销）

## 项目结构
- 工作目录：/home/jack/catkin_ws/src/multi_uav_strike
- 三型无人机：激光雷达型、战斗部型、通信链路型
- 坐标系约定：NED ↔ NWU 转换（位置 x不变/y取反/z取反，四元数 w,x不变/y,z取反）

## 已完成模块
- target_motion_simulator_node：目标运动模拟
- gimbal_simulator_node：云台 LOS 角度计算
- target_estimator_node：粒子滤波三维定位
- guidance_control_node：制导控制（Intercept/MinSnap/LOS 三种策略）
- guidance_strategies：制导策略库

## 重要设计决策
1. **YOLO 不在本 package 内** - 目标检测由其他模块提供，只订阅 `/detection/yolo_result`
2. **工作模式由地面站预设** - 无人机上电预加载航线和模态，筒射完毕后才执行
3. **避障不独立成节点** - 嵌入在搜索阶段实现，减少节点数量
4. **机间通信和地面站通信共用 comm_node** - 从同一硬件接口获取所有信息
5. **RTK 模拟目的** - 衔接仿真与实物/飞控

## 三型无人机工作模式（战斗部型）
- 全图搜索（MODE_SEARCH_ONLY）：识别目标只回传，不跟踪不救援
- 搜索即跟踪（MODE_SEARCH_TRACK）：识别后锁定+螺旋定位，不救援
- 搜索即救援（MODE_SEARCH_STRIKE）：定位完成后制导救援

## 避障策略
- **机间避障**：搜索全程开启，人工势场法
- **毫米波避障**：低速（<12m/s）时开启，检测障碍物立即停飞控

## 待开发模块（按优先级）
1. comm_node（通信节点，统一机间+地面站通信）P0
2. mission_manager_node（任务管理器）P0
3. waypoint_executor_node（航点执行器）P0
4. 机间避障逻辑（嵌入搜索阶段）P1
5. radar_driver_node（毫米波雷达+急停）P1
6. lidar_driver_node（激光雷达驱动）P2
7. FastLio2/A* 集成 P2
8. gimbal_driver_node P3
9. video_stream_node P3
10. boot_detect.sh P3

## 用户偏好
- 代码直接写，不喜欢过度抽象
- 调试输出用 ROS_WARN 和 ROS_DEBUG_THROTTLE
- LOS 制导有 bug 需调试（航向不跟踪问题）

## 项目文档
- 详细需求：docs/SYSTEM_REQUIREMENTS.md
- 开发路线图：docs/DEVELOPMENT_ROADMAP.md
