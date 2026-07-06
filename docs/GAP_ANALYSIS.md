# 协议对齐与架构差距分析

> 目标：单个无人机自动执行「地面起飞 → 搜索（跟踪或即查即打）→ 打击」任务流，仅在搜索跟踪阶段与地面站交互。
>
> 对比依据：
> - `docs/TZS通信接口文档4.0.md` — 主协议
> - `docs/无人机任务协议补充规范文档.md` — 补充规范
>
> 当前代码版本：`multi_uav_strike` + `starling_bridge` (WIP)
>
> **本文档只做分析，不包含任何代码改动。**

---

## 0. 总览：当前架构 vs 协议要求

| 维度 | 协议要求 | 当前状态 | 状态 |
| --- | --- | --- | --- |
| 任务入口 | 地面站通过 MAVLink `MAV_CMD_SET_WORKMODE(51000)` + KCP `CLUSTER_MISSION_CTL(0x1001)` 下发 | 仅支持 ROS 内部 `/gs/mode_cmd` (String) | ❌ 缺失 |
| 任务流模型 | skill_flow（有序 skills[]），技能类型 100/101/102/103/105 | 仅硬切换 IDLE/TAKEOFF/SEARCH_* 工作模式 | ❌ 缺失 |
| 任务执行 | 按 skill 顺序执行，完成一个进入下一个 | 单段执行，无状态推进器 | ❌ 缺失 |
| 工作模式（任务区内） | SEARCH_TRACK / SEARCH_STRIKE / SEARCH_ONLY | 已实现，但硬切换 | ⚠️ 部分 |
| 目标检测 | 仅云台可见时识别，识别时机受 FOV/航向限制 | 全程假设可见 | ❌ 错误假设 |
| 跟踪行为 | SEARCH_TRACK 模式下识别即锁定（不发目标） | 已实现 yoloResultCallback → lock | ⚠️ 部分 |
| 即查即打 | SEARCH_STRIKE 模式下识别即打击 | 已实现，但 lock 后立即启 guidance | ⚠️ 部分 |
| 跟踪上报 TRACKING_STATE (43000) | 进入跟踪状态时上报 | 未实现 | ❌ 缺失 |
| 目标上报 DEVICE_TARGETS (0x2001) | 含目标信息+图片（二进制） | 未实现 | ❌ 缺失 |
| 打击指令 MAV_CMD_ATTACK (51001) | param1=0/1/2（打/取消/暂存） | 未实现 | ❌ 缺失 |
| 任务状态上报 CLUSTER_MISSION_STATE (0x1002) | 定频上报当前技能状态 | 未实现 | ❌ 缺失 |
| 速度两段制 | 任务区外巡航、任务区内任务专属 | 仅单一 uav_speed | ❌ 缺失 |
| 目标类型枚举 | 1=打击 2=跟踪暂存 3=普通（位运算） | 仅 string "person"/"vehicle" | ❌ 缺失 |
| 多机协同 | 心跳分级（base/5Hz/20Hz） | 固定频率 | ❌ 缺失 |
| 桥接层 starling_bridge | KCP/UDP 双向、KCP_HEARTBEAT、CLUSTER_MISSION_CTL 路由 | 已实现 MAVLink/KCP 基础通信，但**未接到 ROS 任务节点** | ⚠️ 框架在，无业务 |
| 桥接层与机载节点的消息总线 | MissionFlow/MissionState/DetectTarget 等 .msg | 已定义，订阅/发布关系未建立 | ❌ 缺失 |

---

## 1. 协议入口层（任务下发）

### 1.1 协议要求的入口

按 `TZS通信接口文档4.0.md` §3，地面站下发任务分两步：

1. **MAV_CMD_SET_WORKMODE(51000)** — 设置整机工作模式（0=即察即打 / 1=搜索跟踪 / 2=搜索 / 3=据止搜索）
2. **CLUSTER_MISSION_CTL(0x1001) over KCP** — 下发技能编排 JSON

技能编排 JSON（§9.2）含：
- `skill_flow_id`（整批任务 ID）
- `devices_sn`（目标 SN 列表）
- `skills[]`（有序，元素含 `skill_type`/`skill_id`/`skill_ctl`/`arrive_path`/`skill_area`/`skill_area_path`/`skill_params`）
- `constraints`（fence/threat_area）

### 1.2 现状

- `gs_simulator_node.py` 通过 `/gs/mode_cmd` (String) 直接发模式字符串
- `comm_node.cpp` 只透传该字符串到 `/mission/mode`
- `mission_manager` 仅作为模式开关，没有「技能」概念
- `waypoint_distributor_node` 用 launch 静态偏移分发航点，无 skill_flow 概念
- `starling_bridge/msg/MissionFlow.msg` 已定义但**无任何节点订阅它**
- `starling_bridge/src/bridge_node.cpp` 只 `sendDeviceRegister` / `sendPayloadStatus` / KCP 心跳，**没有从 KCP 解析 CLUSTER_MISSION_CTL 并发布 MissionFlow**

### 1.3 需要补齐的环节

| 项 | 文件 | 现状 | 需要 |
| --- | --- | --- | --- |
| 任务入口解析器 | `starling_bridge` KCP 接收侧 | 未实现 | 解析 0x1001 业务帧 → 发布 `MissionFlow.msg`（SN + skill_flow_id + JSON） |
| WorkMode 路由 | `starling_bridge` MAVLink 接收侧 | 未实现 | 接收 MAV_CMD_SET_WORKMODE → 发布 `WorkMode.msg` |
| JSON 解析器 | 新增 `mission_flow_parser_node` 或合并进 mission_manager | 未实现 | 把 JSON 拆成「分段航线 + 任务区 + 参数」，驱动状态机 |
| mission_manager 改造 | `mission_manager_node.cpp` | 单段硬切换 | 改为「技能编排执行器」，按 skills[] 推进 |

> ⚠️ **提醒**：任务流的"分段航线+任务区"概念与现在 `waypoint_executor` 的「单段 GPS Path」不兼容。技能编排里每个 skill 都自带 `arrive_path`（到达区前的航线）+ `skill_area`（区域）+ `skill_area_path`（区域内航线）。当前 `waypoint_executor_node` 只能接受一个 Path，无法区分到达段和区内段。需要在 `waypoint_executor` 里增加分段概念，或者拆成两个节点（arrive 段 + search 段）。

---

## 2. 任务流推进器（Mission Phase 状态机）

### 2.1 协议要求

按补充规范 §1.1：技能执行期间 `CLUSTER_MISSION_STATE` 报 **执行中**；抵达最后一个航点时标记 **完成**。这意味着：

- 每个 skill 都有自己的「执行中 → 完成」状态
- 完成后自动进入下一个 skill
- 任务流执行器需要持续上报当前 skill_flow_id / skill_id / skill_type / state

按补充规范 §2：集结完成后**自动终止，等待触发**；搜索完成后自动进入下一任务；返航到最终点自动降落。

### 2.2 现状

`mission_manager` 现在只有 `WorkMode`（IDLE/TAKEOFF/SEARCH_*）和 `MissionPhase`（GROUND_IDLE/TAKING_OFF/HOVERING/WAYPOINT_FOLLOW/...），是**单段任务**的状态机。**没有任何代码能区分：**
- 当前 skill_flow_id 是什么
- 当前 skill_id 是什么
- 当前 skill_type 是什么
- skills[] 推进到第几个了

### 2.3 需要补齐

| 项 | 文件 | 需要做什么 |
| --- | --- | --- |
| skill_flow 状态机 | `mission_manager_node.cpp` | 增加 `current_skill_flow_`、`skill_index_`、`current_skill_type_`、`skill_state_` 成员 |
| skill 推进逻辑 | `mission_manager_node.cpp` | 监听 `waypoint_executor/status`（已完成）+ `guidance_evaluation`（已打击）→ 推进到下一个 skill |
| CLUSTER_MISSION_STATE 周期性发布 | `mission_manager_node.cpp` 或新节点 | 按 `TZS通信接口文档4.0.md` §4.14 的 JSON 结构封装发布到 ROS 话题（桥接节点会再封为 KCP） |
| 任务完成判定 | `mission_manager_node.cpp` | 抵达 skill_area 最后一个航点 → 完成；guidance 报告命中目标（type=1 打击）→ 完成 |

> ⚠️ **提醒**：当前 `mission_manager` 的 `performTakeoffHandoff()` 把 TAKEOFF 之后切到 `SEARCH_ONLY`，这个逻辑和协议要求的「起飞技能完成后自动进入下一技能（如 Gather/Search）」不一致。**现在的 auto-handoff 是一次性补丁**，完整的任务流执行器需要把 TAKEOFF 当作 skill[0]，然后等待 skill[1]（Gather/Search）下发的 arrive_path。
>
> **建议**：保留现有 `TAKEOFF_IDLE → TAKEOFF_COMPLETE` 状态机作为「起飞技能子状态机」，新增 `MISSION_FLOW_IDLE → EXECUTING_SKILL_i → ALL_COMPLETE` 作为顶层状态机，两者用事件耦合。

---

## 3. 任务区内工作模式（§6 + 补充规范 §1.1）

### 3.1 协议要求

按补充规范 §1.1 + §6.3：地面站下发任务时，会同时发送**任务区内工作模式**：

| 模式值 | 名称 | 含义 |
| --- | --- | --- |
| 0 | 即查即打 | 识别到目标直接打击，标记为打击目标（type=1） |
| 1 | 搜索跟踪 | 识别后进入跟踪，**不回传**，等 GS `MAV_CMD_ATTACK` |
| 2 | 搜索打击 | 纯搜索，上报目标后等 GS 指令 |
| 3 | 据止搜索 | GPS 据止环境，相对坐标系搜索 |

按补充规范 §4.3，目标回传时机：
- **即查即打**：识别即打，回传目标 type=1（打击）
- **搜索跟踪**：识别后**不回传**，等 MAV_CMD_ATTACK：
  - param1=0（打）：打，回传 type=1
  - param1=1（忽略）：回到搜索
  - param1=2（暂存）：回传 type=2（跟踪暂存），回到搜索
- **搜索打击**（纯搜索）：识别即回传 type=3（普通），等 GS 指令

### 3.2 现状

`mission_manager.cpp` 现在只有：
- `SEARCH_ONLY` = 纯搜索（上报）
- `SEARCH_TRACK` = 识别即跟踪（不上报目标，启动 guidance）
- `SEARCH_STRIKE` = 识别即打击（启动 guidance）

**问题**：
1. 「搜索跟踪」识别后确实进入了跟踪，但**没有触发 TRACKING_STATE 上报**，且**永远不上报目标**，符合协议要求但没和 GS 打通。
2. 「即查即打」和「搜索打击」是不同概念，但当前 `SEARCH_STRIKE` 含义是「即查即打」，而「搜索打击」=「纯搜索+等指令」= 当前的 `SEARCH_ONLY` 模式。**模式名不对应**。
3. 协议要求的「地面站发 MAV_CMD_ATTACK 后才执行打击」**完全没有实现** — 现在是 YOLO 检测即启动 guidance。
4. 协议要求的 `param1=1/2`（忽略/暂存）逻辑**完全没有**。

### 3.3 需要补齐

| 项 | 文件 | 现状 | 需要 |
| --- | --- | --- | --- |
| 模式名重映射 | `mission_manager.cpp` | `SEARCH_TRACK`/`SEARCH_STRIKE`/`SEARCH_ONLY` | 与 TZS `MAV_CMD_SET_WORKMODE` param1 0/1/2/3 对齐（注意：现在命名跟协议语义混了） |
| 工作模式与 skill 绑定 | `mission_manager.cpp` | 全局 work_mode | 改为「当前 skill 的 work_mode」，不同 skill 可不同模式 |
| YOLO → TRACKING_STATE 联动 | `mission_manager.cpp` | `yoloResultCallback` 仅做 lock | 搜索跟踪模式下 lock → 发布 `TRACKING_STATE` ROS 消息（待桥接层封 MAVLink 43000） |
| YOLO → target 不上报（搜索跟踪） | `mission_manager.cpp` | 不做处理 | 确认逻辑：搜索跟踪模式 lock 后**绝不**触发 `reportTargetToGs()`，等 MAV_CMD_ATTACK |
| MAV_CMD_ATTACK 订阅 | 新增 `mission_manager.cpp` 或新节点 | 未实现 | 订阅桥接层转发的 ATTACK 指令 → 决定打/忽略/暂存 |
| 打击执行联动 | `mission_manager.cpp` + `guidance_control_node` | YOLO lock 即启 guidance | MAV_CMD_ATTACK param1=0 才启 guidance；命中目标时上报 type=1 DEVICE_TARGETS |
| 暂存 / 忽略 | `mission_manager.cpp` | 未实现 | param1=2 → 上报 type=2，回到搜索；param1=1 → 上回搜索，无上报 |

> ⚠️ **提醒**：现在 `guidance_control_node` 一旦 enable=true 就一直跑，直到 disable。如果「搜索跟踪模式」要等 GS 指令才打击，需要 `guidance_control` 处于 enable 但**不发目标**的"待机"状态，或者只在收到 MAV_CMD_ATTACK 之后再 enable。当前代码已经把 enable 但无目标时改为发零速 hover，是对的；需要新增「enable + 收到 ATTACK 指令 → 喂目标给 guidance」的衔接逻辑。

---

## 4. 目标类型与上报（§11 + 补充规范 §4）

### 4.1 协议要求

`DEVICE_TARGETS(0x2001)` 上报 Body 包含：
- `report_id` (uint32, 小端, ms 时间戳低 32 位)
- `obj_count` (uint32, 小端)
- 每个 DetectObject 含：
  - 时间戳 ts_us (int64, 小端)
  - 无人机位置/姿态 (4×double, 小端)
  - 目标位置 (3×double, 小端)
  - `label` (uint32)
  - `confidence` (float)
  - **`type` (uint8)**: 按补充规范 §4.2：1=打击，2=跟踪暂存，3=普通搜索
  - `img_fmt` (uint8, 1=JPEG)
  - `img_size` (uint32) + `img_data` (JPEG 字节流)
  - `sn_data` (设备 SN)

### 4.2 现状

- `comm_node.cpp::yoloResultCallback` 是空 stub，**YOLO 结果完全没解析**
- `target_info_pub_` 只发 `geometry_msgs::PoseStamped`，没有图片、没有 type 字段
- 没有任何节点生产 `starling_bridge/DetectTargets.msg`
- `bridge_node.cpp` 没有订阅 `DetectTargets`，也没有封装 DEVICE_TARGETS 二进制 body

### 4.3 需要补齐

| 项 | 文件 | 现状 | 需要 |
| --- | --- | --- | --- |
| YOLO 检测消息 | `comm_node.cpp` | 仅空 stub | 解析 `/detection/yolo_result`（当前是 String），提取 class/conf/x/y/z/img |
| 目标类型位运算 | `comm_node.cpp` + `target_estimator_node.cpp` | 无 type 字段 | 按 §4.1 增加 `target_type`（uint8，1/2/3） |
| DEVICE_TARGETS 发布 | 新节点或并入 mission_manager | 未实现 | 触发条件：搜索打击 type=3 / 跟踪暂存 type=2 / 打击完成 type=1 |
| 图片二进制支持 | `multi_uav_strike` + `starling_bridge` | 无 | YOLO/感知节点需要发布 JPEG bytes；桥接层需要把它们拼到 body |
| JPEG 图来源 | 整个项目 | 没有任何节点产生图片 | 仿真环境可以用假图片字节流或截图，真机需要相机驱动 |

> ⚠️ **提醒**：仿真环境（jMAVSim / multimodequad_ros）根本没有相机图片流。要在仿真里跑通「目标+图片」上报，需要：
> 1. 仿真节点生成假图片（比如随机 JPEG 字节流）
> 2. 或绕过图片字段，仅上报 metadata 用于协议联调
>
> 建议**分两步实现**：先不带图片，只跑 DEVICE_TARGETS 的 metadata 部分；图片能力放到后续阶段。

---

## 5. TRACKING_STATE 上报（§6）

### 5.1 协议要求

`MAVLINK_MSG_ID_TRACKING_STATE(43000)`：
- `time_boot_ms` (uint64)
- `tracking_state` (uint8)：0=未跟踪，1=跟踪中

### 5.2 现状

- `mission_manager.cpp` 完全没生成 TRACKING_STATE 消息
- `bridge_node.cpp` 没有订阅 tracking state 话题
- `MAVLinkComm` 类（推测，未读完整）需要扩展 sendTrackingState

### 5.3 需要补齐

| 项 | 文件 | 需要 |
| --- | --- | --- |
| TRACKING_STATE ROS 话题 | `mission_manager.cpp` | 新增 publisher（如 `/mission/tracking_state`），在搜索跟踪模式下检测到目标 → 发 state=1 |
| 桥接层订阅 → 封装 MAVLink 43000 | `bridge_node.cpp` | 订阅 `tracking_state` → mavlink_.sendTrackingState() |
| `MavlinkComm::sendTrackingState` | `mavlink_comm.cpp` | 新增方法，按 mavlink 43000 定义封装 |

---

## 6. 云台视角限制（核心架构问题）

### 6.1 协议/物理约束

按用户描述 + 补充规范 §4.3：
- **2 轴云台**：俯仰 + 横滚（roll 0），**没有偏航轴**
- **航向跟随机体**：云台只能看到机体正前方 FOV 锥内的目标
- **FOV 有限**：默认 45°（`fov_deg_`）
- 当无人机沿航点飞行时，**目标若不在视野内，必须等下一圈飞回才能再次看到**

按补充规范 §4.3：「在跟踪模式中，识别到目标后立刻跟随，**但不回传目标**」，说明 UAV 在识别后必须进入跟踪状态（不再是「先识别再上报」），引导机体朝目标飞。

### 6.2 现状（严重偏差）

`gimbal_simulator_node.cpp` 当前实现：
- 云台**有 yaw 控制**（`current_gimbal_yaw_`、`max_yaw_rate_=1.2 rad/s`）
- yaw 由「目标相对于无人机的方位角」驱动 → **云台会自动旋转对准目标**
- 结果：UAV 沿航点直线飞，云台指向旁边目标，**目标始终在视野内**

这是**与真实硬件不符**的假设。真实 2 轴云台没有 yaw 自由度，云台视野 = 机体正前方 FOV 锥。

### 6.3 需要补齐

| 项 | 文件 | 现状 | 需要 |
| --- | --- | --- | --- |
| 云台 yaw 模型 | `gimbal_simulator_node.cpp` | 有 yaw 自由度，自动跟随目标 | **改为无 yaw 自由度**：`current_gimbal_yaw_` 恒等于 `current_uav_yaw_`（机体航向），或者直接删除 yaw 控制，只保留 pitch |
| 目标可见性判定 | `gimbal_simulator_node.cpp` + `target_estimator_node.cpp` | 无 | 增加判断：目标方向相对机体航向的偏角 < FOV/2 才"可见"，否则不可见 |
| 搜索跟踪模式衔接 | `mission_manager.cpp` | `yoloResultCallback` 仅 lock | 识别到目标后**必须让 UAV 转向目标方向**（改写航点 / 调航向），否则飞过去之后目标就出视野了 |
| YOLO 输入 | 整个项目 | 无 yolo_result 发布 | 仿真环境里：基于 gimbal FOV + target 位置 + UAV 位置姿态做几何判定，发布 "看到/没看到" 事件 |
| 跟踪中的"看到/丢失" | `target_estimator_node.cpp` | 始终在估计 | 加状态机：`TRACKING_VISIBLE` / `TRACKING_LOST`，丢失超过 N 秒回搜索 |

> ⚠️ **关键提醒**：用户原话："现在全程都能看到目标，实际上云台有视角限制...在看不到目标的时候无人机应该保持航点运行直到看到目标"。
>
> **这是当前最大的架构偏差**：
> 1. `gimbal_simulator` 必须改成"yaw=机体航向"，否则跟踪阶段 gimbal 会偷偷旋转让目标始终可见。
> 2. `yoloResultCallback` 依赖 YOLO 输入，但目前 YOLO 是空 stub — 仿真环境需要按 FOV 几何关系模拟 YOLO 检测输出。
> 3. `target_estimator_node` 当前用 PF 在 gimbal LOS 基础上做 3D 定位，**PF 假设目标连续可见**。改成"目标间歇可见"后需要处理：粒子在目标消失期间靠运动模型预测；目标重新可见后立即重新初始化。
>
> **建议实现顺序**（影响最大、阻塞后续所有功能）：
> 1. gimbal yaw 模型改为跟随机体航向
> 2. 加 FOV 几何判定 + 模拟 YOLO 检测输出
> 3. target_estimator 增加可见性状态机
> 4. mission_manager 搜索跟踪模式下，识别目标后切换航向控制（让 UAV 朝目标飞）

---

## 7. 速度两段制（§3）

### 7.1 协议要求

按补充规范 §3：
- 所有任务必须携带速度参数
- 任务区**外**用**巡航速度**
- 任务区**内**用**任务专属速度**
- 抵达任务区内第一个航点**之后**才激活任务功能模式

按 §9.2.6，技能参数里有 `gather_speed`、`search_speed`、`return_speed`、`takeoff_height`、`takeoff_time_interval` 等。

### 7.2 现状

- `waypoint_executor.yaml` 只有 `uav_speed=2.0`（单一速度）
- `guidance_control.yaml` 只有 `track_max_speed`、`track_k_approach`
- `mission_manager.yaml` 没有任务速度参数
- 没有「任务区内/外」判定逻辑

### 7.3 需要补齐

| 项 | 文件 | 需要 |
| --- | --- | --- |
| 任务区判定 | `waypoint_executor_node.cpp` 或 mission_manager | 计算当前 UAV 位置 vs `skill_area`（多边形/圆/点）→ 是否在区内 |
| 双段速度切换 | `waypoint_executor_node.cpp` | 区内用 `skill_speed`，区外用 `cruise_speed` |
| 任务区激活标志 | `mission_manager.cpp` | 进入区内 → 触发任务功能（识别即跟踪 / 识别即打击 / 跟踪上报） |
| 任务速度参数注入 | `mission_manager.cpp` | 从 skill_params 提取 gather_speed/search_speed/return_speed 写入对应节点参数 |

---

## 8. 桥接层业务路由（starling_bridge → ROS 节点）

### 8.1 现状

`starling_bridge/src/bridge_node.cpp` 现在的 `onSlowTimer`：
```cpp
mavlink_->sendDeviceRegister(...);  // 42000 上报
mavlink_->sendPayloadStatus();      // 42001 上报
kcp_->tick1s();                     // HELLO / KCP 心跳
```

`onMavlinkTimer` 是空函数。

`onKcpTimer` 只调 `kcp_->update()`。

**没有任何业务路由**：
- 不订阅 ROS 节点的 `MissionFlow` / `MissionState` / `DetectTargets` / `TrackingState` 话题
- 不解析 MAVLink 的 SET_WORKMODE / MAV_CMD_ATTACK / MAV_CMD_MEDIA_SWITCH / MAV_CMD_CTRL 指令
- 不把 MAVLink 指令分发到 ROS 节点

### 8.2 需要补齐

| 项 | 文件 | 需要 |
| --- | --- | --- |
| KCP 业务帧分发 | `kcp_comm.cpp` | 解析 0x1001 → 发布 `MissionFlow.msg`；解析 0x1002（上行方向不应有，KCP 是机→地，0x1002 是反向，机载不应收到） |
| MAVLink 指令分发 | `mavlink_comm.cpp` + `bridge_node.cpp` | 订阅 mavlink from 话题；解析 51000/51001/50003/50000 → 发布对应 ROS 消息 |
| ROS→KCP 上报 | `bridge_node.cpp` | 订阅 `MissionState.msg` → KCP 封 0x1002；订阅 `DetectTargets.msg` → KCP 封 0x2001；订阅 `TrackingState.msg` → mavlink 43000 |
| 桥接层与任务节点命名空间 | `bridge_node.cpp` + launch | 当前 `bridge_node` 不在 `uav*` namespace 内（看 launch `single_uav_px4_sitl_test.launch:171`），桥接节点需要按 SN 路由 |

> ⚠️ **提醒**：当前 `bridge_node` 也不订阅 ROS 侧的 `/mavros/state`、`/mavros/local_position/pose`、`/detection/yolo_result` 等数据，**没有把 PX4 的状态/位置打包成 MAVLink 消息发出去**。`onSlowTimer` 里那几个 test 函数都被注释掉了，真机或 SITL 跑起来时地面站拿不到任何 MAVLink 状态回传（除了 42000/42001 这两条设备注册/载荷类型）。
>
> **starling_bridge 当前状态：WIP 框架在，缺所有业务路由。**

---

## 9. 多机协同（§6 + 补充规范 §6）

### 9.1 协议要求

按补充规范 §6：
- 互通数据：本机 SN + 飞行模式 + 当前任务 + GPS 坐标
- 心跳频率分级：
  - 默认：基础频率
  - 间距 < 40m：5Hz
  - 间距 < 20m：20Hz
- 模式切换通信：地面站每次下发任务都会发模式指令

### 9.2 现状

- `comm_node.cpp` 用固定 10Hz `telemetry_rate_` 通过 `/inter_uav/self_pose` 发
- 没有「基础/5Hz/20Hz」分级
- 没有「飞行模式 + 当前任务」信息
- `inter_uav/other_uav_poses` 只发位置数组，没有 SN/模式/任务

### 9.3 需要补齐

| 项 | 文件 | 需要 |
| --- | --- | --- |
| 邻居距离分级 | `comm_node.cpp` | 计算每架邻居的距离，对每个邻居单独发对应频率 |
| 互通数据扩展 | `comm_node.cpp` | PoseArray 增加 SN/模式/任务字段（改自定义 msg 或 PoseStamped 字段） |
| 模式同步 | `mission_manager.cpp` ↔ `comm_node.cpp` | mission_manager 把 current_skill_type 发给 comm_node，comm_node 嵌进互通消息 |

> ⚠️ **提醒**：当前「最低目标：单 UAV」可以不实现多机协同。建议把这一节标记为 P2（次要）。

---

## 10. 配置/启动/坐标系一致性

### 10.1 GPS 参考点不一致

多个地方 ref_lat/lon/alt 都有默认值，互相不一致：

| 文件 | ref_lat | ref_lon | ref_alt |
| --- | --- | --- | --- |
| `common.yaml` | 36.058800 (安阳) | 114.557221 | 63.0 |
| `comm_node.cpp` 默认值 | 36.096 | 114.392 | 100.0 |
| `mission_manager.cpp` 默认值 | 36.096 | 114.392 | 100.0 |
| `guidance_control.cpp` 默认值 | 36.096 | 114.392 | 100.0 |
| `waypoint_executor.cpp` 默认值 (硬编码常量) | 36.096 | 114.392 | 100.0 |
| `gs_simulator.py` 默认值 | 40.151347 (北京) | 116.980590 | 31.0 |
| `gs_simulator.py` launch 注入 (`single_uav_strike_test.launch`) | (用 default) | | |

**风险**：launch 没显式注入 ref_* 时，C++ 节点用 36.096 系列，gs_simulator 用 40.x 系列，**坐标系对不上，UAV 飞错地方**。

> ⚠️ **提醒**：所有 launch 文件必须显式注入 ref_lat/lon/alt，且全部节点默认值要统一（建议全用 common.yaml 的安阳 36.058800 系）。

### 10.2 use_sim 模式不一致

`common.yaml` 默认 `use_sim: false`（PX4 SITL），但 `single_uav_strike_test.launch` 用 `multimodequad_ros`（jMAVSim）仿真。

`single_uav_strike_test.launch` 没有加载 common.yaml（直接 <param> 注入），所以 waypoint_executor 用 `use_sim=true`，但 guidance_control 也用默认 `use_sim=true`（默认）。**当前 launch 自洽，但启动其他 launch 时容易混。**

> ⚠️ **提醒**：建议所有 launch 顶部 `<rosparam command="load" file="..." />` 一次，所有节点再按需 param 覆盖。

### 10.3 multi_uav_strike 的 YAML 加载机制

`single_uav_strike_test.launch` 直接 `<param>` 注入每个节点的参数，没用 `<rosparam command="load" file="..." />`。

`single_uav_px4_sitl_test.launch` 用 `<rosparam command="load" file="..." />` 加载 common.yaml + 各节点 yaml。

**两种风格混用**，改参数时要找多个地方。

> ⚠️ **提醒**：统一成 `single_uav_px4_sitl_test.launch` 的 `<rosparam command="load" />` 风格，YAML 文件做单一来源。

---

## 11. 缺失的 ROS 节点（综合）

按上面分析，**新架构需要增加以下节点或大幅改造现有节点**：

| 节点 | 性质 | 职责 |
| --- | --- | --- |
| `task_flow_executor_node`（**新增**） | 任务流推进器 | 接收 `MissionFlow`，解析 skills[]，按顺序推进；维护 skill_index、current_skill_type、skill_state；周期性发布 `MissionState` |
| `work_mode_router_node` 或合并进 mission_manager | 改造 | 把 SET_WORKMODE param1 (0/1/2/3) 映射到搜索跟踪/即查即打/搜索/据止搜索逻辑 |
| `attack_cmd_handler_node` 或合并进 mission_manager | 改造 | 订阅 MAV_CMD_ATTACK，决定打/忽略/暂存 |
| `detection_simulator_node`（**新增**） | 仿真支持 | 在仿真环境里按 FOV + UAV+target 几何关系模拟 YOLO 检测结果，发布 `/detection/yolo_result` |
| `target_classifier_node`（**新增**） | 目标分类 | 按 §4.2/§8 把目标分成 type=1/2/3，并发布 `DetectTargets.msg` |
| `device_targets_publisher_node`（**新增**） | 协议上报 | 把 DetectTargets + JPEG 封装成 DEVICE_TARGETS 二进制 body（机内协议）；桥接层负责封 KCP |
| `tracking_state_publisher_node` 或合并进 mission_manager | 改造 | 检测到目标 → state=1；丢失目标 → state=0 |
| `bridge_node` 业务路由 | 大幅改造 | 订阅 ROS 任务/检测/跟踪话题 → 封 MAVLink/KCP；订阅 MAVLink/KCP 指令 → 发布 ROS 任务话题 |

**`mission_manager_node` 可以选择**：
- 选项 A：保持单一节点，逐步加 skill 状态机 + work_mode 路由 + 攻击指令处理 + 上报
- 选项 B：拆成 `task_flow_executor` + `mission_manager`（只管 PX4 takeoff/landing/单段航点）

> ⚠️ **关键提醒**：建议**选项 B**。当前 mission_manager 已经接近 1700 行（含大量 PX4 OFFBOARD 时序处理），再加 skill 状态机会突破 3000 行，难以维护。
>
> 推荐结构：
> ```
> task_flow_executor_node    ← 解析 MissionFlow, 推进 skill 状态, 发布 MissionState
>   ├→ work_mode_router_node  ← 把 SET_WORKMODE 转成内部指令
>   ├→ attack_cmd_handler     ← 处理 MAV_CMD_ATTACK
>   ├→ mission_manager_node   ← PX4 takeoff/handoff/land + 调度子模块
>   ├→ waypoint_executor_node ← 航点跟踪 (含分段速度)
>   └→ guidance_control_node  ← 跟踪/打击制导
> starling_bridge_node        ← KCP/MAVLink ↔ ROS 双向路由
> detection_simulator_node    ← 仿真 YOLO 输出
> target_classifier_node      ← 类型分类, 发布 DetectTargets
> ```
>
> 但**先不拆**，可以等 skill 状态机和 work_mode 路由稳定后再拆。

---

## 12. 实施优先级建议

按阻塞关系排序，**先做最底层的物理/几何/协议入口**：

### 第一阶段：物理/几何基础（阻塞所有功能）
1. **gimbal yaw 模型改造** — 改为跟随机体航向，删除自由 yaw 控制
2. **FOV 几何判定** — `gimbal_simulator` 或新节点判定目标是否在视野内
3. **detection_simulator_node** — 按几何判定发布 `/detection/yolo_result` 模拟 YOLO
4. **target_estimator 可见性状态机** — TRACKING_VISIBLE / TRACKING_LOST，超时回搜索

### 第二阶段：协议入口（机内）
5. **starling_bridge 业务路由** — 解析 SET_WORKMODE / MAV_CMD_ATTACK / CLUSTER_MISSION_CTL
6. **task_flow_executor_node** — 解析 MissionFlow，推进 skill 状态，发布 MissionState
7. **waypoint_executor 分段** — 区分 arrive_path / skill_area_path 两段，区分区内外速度

### 第三阶段：业务联动
8. **work_mode 路由** — 0/1/2/3 对应即查即打/搜索跟踪/搜索/据止搜索
9. **MAV_CMD_ATTACK 处理** — param1=0/1/2 分支
10. **DEVICE_TARGETS 上报** — 含 type 字段
11. **TRACKING_STATE 上报** — state=1/0

### 第四阶段：完整性
12. **多机协同心跳分级** — 5Hz/20Hz 距离触发
13. **任务流示例 JSON** — 给地面站一份可用的 skill_flow 模板

### 第五阶段：配置一致性
14. **所有 launch 显式注入 ref_lat/lon/alt**
15. **统一 yaml 加载风格**

---

## 13. 风险与陷阱

| 风险 | 描述 | 缓解 |
| --- | --- | --- |
| 状态机爆炸 | mission_manager 加 skill 状态机后接近 3000 行 | 拆成 task_flow_executor_node |
| 协议理解偏差 | skill_area / arrive_path / skill_area_path 概念现在没区分 | 先在 waypoint_executor 里只支持单段，等 skill 概念落地后再加段 |
| 仿真 vs 真机鸿沟 | jMAVSim 没有相机/真实目标 → detection_simulator 必须存在 | 把 detection_simulator 作为仿真环境的标准组件 |
| CLUSTER_MISSION_CTL 大 payload | JSON 几 KB 经 KCP 分片 | 已用 KCP，理论上没问题，但 KCP 配置（nodelay/wnd）要调好 |
| 任务流 vs 起飞衔接 | 当前 auto-handoff 是补丁，完整任务流要把 TAKEOFF 当 skill[0] | 保留现有 takeoff 状态机作为 skill 子状态 |
| 桥接层 MAVLink 通道与 ROS 话题映射 | SET_WORKMODE / MAV_CMD_ATTACK / MAV_CMD_CTRL 等命令的 MAVLink 命令字段到 ROS msg 的映射要仔细 | 参考 mavlink 官方文档和 `TZS通信接口文档4.0.md` §3/§8/§12 的 param 字段定义 |
| 桥接层网络字节序 | KCP 内部小端，业务层帧头大端，Body 按各 msg 自定义 — 当前 `kcp_comm.cpp`/`mavlink_comm.cpp` 未读完整，要看实现是否一致 | 实施时务必先读完 `kcp_comm.cpp` 和 `mavlink_comm.cpp` |
| starling_bridge 编译/链接 | CMakeLists 链接 ikcp.c + mavlink headers，没用过要看 build 是否通过 | 第一阶段先 `catkin_make` 验证编译 |

---

## 14. 总结

按当前代码状态，「单 UAV 自动执行地面起飞→搜索跟踪/即查即打→打击」**短期内无法实现**，主要卡在：

1. **协议入口完全没接上**（starling_bridge 没业务路由，ROS 节点不订阅 MissionFlow）
2. **云台视角模型错了**（gimbal 有 yaw 自由度，假设目标全程可见）
3. **任务流概念缺失**（mission_manager 没有 skill 状态机）
4. **目标类型/上报完全没做**（yolo stub、DEVICE_TARGETS、TRACKING_STATE 都没）
5. **MAV_CMD_ATTACK 完全没接**（地面站指令通路断）
6. **速度两段制没做**（任务区内/外无区分）

**最高优先级**：先修第 2 项（云台视角）和第 1 项（协议入口），这两项不解决，其他都做不下去。

**第二优先级**：任务流推进器（task_flow_executor）+ work_mode 路由，这是协议语义层骨架。

**第三优先级**：目标类型/上报/ATTACK 处理，这是业务交互。

---

> 注：本文档只做架构分析，不修改任何代码。后续按 §12 优先级逐步实施。