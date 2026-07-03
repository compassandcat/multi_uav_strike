### 较3.0版本**修正内容**：

1. **磁力计校准**：将 `MAG_CAL_REPORT`(APM 专属消息) 替换为 PX4 实际的 `[cal]` STATUSTEXT 协议，删除了 `MAG_CAL_REPORT` 字段表和 `MAG_CAL_STATUS` 枚举表。
2. **加速度计校准**：删除了 GCS 为每面发送确认指令的错误流程（那是 APM 的 `MAV_CMD_ACCELCAL_VEHICLE_POS` 模式），改为正确的 PX4 自动检测姿态流程——GCS 只发一次启动命令，PX4 固件通过 `[cal]` STATUSTEXT 自动通知姿态检测和面完成。删除板级微调（水平校准）。
3. **加速度计校准**：有两种不同的使用场景，一般都需要：
   - **`param5=1`（完整 6 面）**：首次校准或重新标定使用。飞行器需要依次摆放 6 个面，计算加速度计的 scale 和 offset。这是**主要的校准方式**。
   - **`param5=2`（水平微调）**：已有完整校准的前提下，仅矫正水平面偏置。飞行器只需水平静止，计算 trim 值。用于装机后微调或轻微碰撞后的修正。
     前端UI 中应为两者提供分开入口：加速度计页面提供"Calibrate Accelerometer"按钮对应 `param5=1`，水平校准页面提供"Calibrate Level"按钮对应 `param5=2`。实际使用时先做完整 6 面校准，后续需要时再做水平微调。
4. **补充协议细节**：添加了完整的 `[cal]` STATUSTEXT 消息格式汇总表、`CAL_MAG_SIDES` 位掩码说明、6 面 side 名称对照表。
6. ***\*删除 PREFLIGHT_STORAGE(param1=1) 持久化步骤\****：原文档在各校准流程末尾添加了 `MAV_CMD_PREFLIGHT_STORAGE(param1=1)` 步骤，要求 GCS 发送存储指令将校准结果写入 FLASH。经参考 QGC 源码 `SensorsComponentController.cc:100-151`（`_stopCalibration()`）及 `SensorsComponentController.cc:375-383`（`_refreshParams()`），在校准完成后***\*不发送\**** `MAV_CMD_PREFLIGHT_STORAGE(param1=1)`——PX4 固件在 `[cal] calibration done` 前内部已调用 `param_save_default()` 自动持久化校准结果到 FLASH，GCS 无需额外干预。仅在校准完成后通过 `_refreshParams()` 从飞控回读更新后的 `CAL_*`/`SENS_*` 参数值。唯一使用 `MAV_CMD_PREFLIGHT_STORAGE` 的场景是 `resetFactoryParameters()` 发送 `param3=3`（重置出厂参数），与校准结果持久化无关。
6. 目前电量从SYS_STATUS获取，未使用BATTERY_STATUS
7. 删除**MISSION_CURRENT**接口，无需显示当前航点

# 通信接口文档

## 1、一些重要说明

### 1.1 通信协议

采用mavlink2.0标准协议+UDP（KCP封装）的通信协议方案。

原因如下：

MAVLink 在以下场景存在限制：

1. 不适合承载复杂嵌套结构
2. 不适合承载较大图片二进制数据

### 1.2 MAVLink协议

地面站mavlink模块监听**0.0.0.0:14551**地址，接收任意ip往14551端口发送的消息，通过 **Boost.Asio**库的`socket.async_receive_from()` 从获取到ip。

每个MAVLink消息头部都包含 `system_id` `component_id`字段（1-255），地面站接收消息时从消息头解析：

```c
// MAVLink消息头结构
mavlink_message_t msg;
// 接收并解析消息后
uint8_t system_id = msg.sysid;  // 发送方的system_id
uint8_t component_id = msg.compid;  // 组件ID
```

### 1.3 KCP(UDP)协议设计

kcp监听0.0.0.0:18990地址，接收任意ip往18990端口发送的消息

底层网络传输使用 **UDP**

在 UDP 之上引入 **KCP**，用于实现可靠传输

KCP 负责分片、重组、重传、乱序处理

业务层仅定义业务消息格式，不再重复定义 UDP 级别的序号、分片、重传和 CRC 机制

#### 1.3.1 UDP外层

| 偏移 | 字段        | 类型    | 字节数      | 说明                |
| ---- | ----------- | ------- | ----------- | ------------------- |
| 0    | magic       | uint16  | 2           | 固定 `0xABCD`，大端 |
| 2    | type        | uint8   | 1           | 类型，见下表        |
| 3    | payload_len | uint32  | 4           | payload 长度，大端  |
| 7    | payload     | uint8[] | payload_len | 可变长 payload      |

UDP外层 type 枚举及 payload 说明：

| type | 名称                | payload 内容                       | 字节序           |
| ---- | ------------------- | ---------------------------------- | ---------------- |
| 1    | HANDSHAKE_HELLO     | 空（不用解析                       |                  |
| 2    | HANDSHAKE_HELLO_ACK | conv (4B)                          | **大端**         |
| 3    | KCP_DATA            | KCP 编码数据（含 KCP 头 + 业务层） | KCP 内部**小端** |
| 4    | KCP_HEARTBEAT       | conv (4B)                          | **小端**         |

\> ***\*注意\****：type=2 (HELLO_ACK) 和 type=4 (KCP_HEARTBEAT) 中的 conv 使用不同字节序，原因见 §2.1.2 和 §2.1.3 的说明。

\> ***\*注意：只有 type=3 时才包含 KCP 层和业务层。\**** type=1/2/4 的 payload 直接跟在 UDP 头后面，没有 KCP 头和业务头。

#### 1.3.2 KCP层

KCP Segment 由 KCP 库自动生成，**通常使用者无需关注**，**调用ikcp_send 即可**，每段固定 24 字节头（IKCP_OVERHEAD），全部字段小端序：

| 偏移 | 字段 | 类型    | 字节数 | 说明                                                         |
| ---- | ---- | ------- | ------ | ------------------------------------------------------------ |
| 0    | conv | uint32  | 4      | 会话 ID                                                      |
| 4    | cmd  | uint8   | 1      | 命令类型：`81`(PUSH 数据推送), `82`(ACK 确认), `83`(WASK 窗口询问), `84`(WINS 窗口应答)等 |
| 5    | frg  | uint8   | 1      | 分片倒计数，0 表示最后一片（非流模式）                       |
| 6    | wnd  | uint16  | 2      | 接收窗口剩余大小                                             |
| 8    | ts   | uint32  | 4      | 发送时间戳ms                                                 |
| 12   | sn   | uint32  | 4      | 发送序号                                                     |
| 16   | una  | uint32  | 4      | 已确认序号（对方期望的下一个序号）                           |
| 20   | len  | uint32  | 4      | 数据长度                                                     |
| 24   | data | uint8[] | len    | 用户数据（业务层内容）                                       |

#### 1.3.3 业务层（仅 type=3 时包含）

业务层 header 共 11 字节，***\*全部字段为大端序\****：

| 偏移     | 字段       | 类型    | 字节数     | 说明                    |
| -------- | ---------- | ------- | ---------- | ----------------------- |
| 0        | Magic      | uint16  | 2          | 0xABEF                  |
| 2        | Version    | uint8   | 1          | 当前 `1`                |
| 3        | MsgID      | uint16  | 2          | 消息类型标识            |
| 5        | SNLen      | uint16  | 2          | 设备序列号长度          |
| 7        | PayloadLen | uint32  | 4          | Payload 长度            |
| 11       | SN         | SNLen   | SNLen      | 设备序列号 (UTF-8)      |
| 11+SNLen | Payload    | uint8[] | PayloadLen | 消息体 (JSON 或 二进制) |

防御性限制：SNLen ≤ 1024, PayloadLen ≤ 10MB。

不同的kcp自定义消息通过业务层的MsgID进行区分。

#### 1.3.4 KCP 库使用说明

KCP 库对使用者是***\*黑盒\****：使用者只需填充业务层数据，调用 KCP 库接口，KCP 库自动完成分包、加头、重传、拥塞控制，即**KCP层**的封装。使用者在注册的回调函数中包裹 **UDP 外层**头并发送。

***\*使用者需要做的事情：\****

\```

① 构建业务层二进制 buffer (11B 业务头 + SN + JSON/二进制 body)

② 调用 ikcp_send(kcp, buffer, len)  — KCP 自动按 MSS 切分并放入发送队列

③ 调用 ikcp_update(kcp, now_ms)    — 驱动 KCP 内部刷新 (可定时循环调用)

④ 在 output 回调中接收 KCP 拼好的 segment，包裹 UDP 外层头并 socket.sendto()

\```

## 2.建立连接

### 2.1 mavlink建立连接

#### 2.1.1**MAVLINK_MSG_ID_HEARTBEAT**

mavlink心跳，详见4.1

#### 2.1.2MAVLINK_MSG_ID_DEVICE_REGISTER

（自定义消息，42000）（机载端->地面站）

机载端发送mavlink自定义消息将自身设备信息上报，消息为MAVLINK_MSG_ID_DEVICE_REGISTER，Message ID为42000，地面站解析记录sn，并与ip，sysid映射。

| 字段       | 类型         | 单位 | 说明                                               |
| ---------- | ------------ | ---- | -------------------------------------------------- |
| `sn`       | char[32]     | —    | 设备序列号，全局唯一标识                           |
| `name`     | char[32]     | —    | 设备显示名称                                       |
| `type`     | uint8        | —    | 平台类型：1=UAV（无人机）, 2=UGV, 3=USV            |
| `sub_type` | **uint16_t** | —    | 当前固定为110，表示四旋翼                          |
| vendor     | **uint16_t** | —    | 厂商识别：1=DJI, 2=AUTEL, 3=NHY, 99=CUSTOM（自研） |

### 2.2 kcp连接流程

#### UDP头结构体

```
struct __attribute__((packed)) UdpHeader {
    uint16_t magic;       // 0xABCD
    uint8_t  type;        // 1: HELLO, 2: HELLO_ACK, 3: KCP_DATA
    uint32_t payload_len; // payload 长度
};
// __attribute__((packed))的作用告诉编译器：结构体成员之间不插入任何填充字节，按最小对齐排布。
```

#### 2.2.1 HANDSHAKE_HELLO

（机载端->地面站）(裸 UDP 包，不走 KCP)

机载端需要向地面站的18990端口持续发送HANDSHAKE_HELLO消息，消息定义如下：

| 偏移 | 字段        | 类型     | 字节数 | 说明                                 |
| ---- | ----------- | -------- | ------ | ------------------------------------ |
| 0    | magic       | uint16_t | 2      | **大端**，固定值 `0xABCD`            |
| 2    | type        | uint8_t  | 1      | `0x01` (HANDSHAKE_HELLO)             |
| 3    | payload_len | uint32_t | 4      | **大端**，payload 长度，0，无payload |

#### 2.2.2 HANDSHAKE_HELLO_ACK

（地面站->机载端）(裸 UDP 包，不走 KCP)

地面站接收到HANDSHAKE_HELLO消息后，回复 HANDSHAKE_HELLO_ACK 到**该 HELLO 包的源 IP 和源端口**（即设备端发送 HELLO 时所用的端口），消息定义如下：

| 偏移 | 字段        | 类型   | 字节数 | 说明                         |
| ---- | ----------- | ------ | ------ | ---------------------------- |
| 0    | magic       | uint16 | 2      | **大端**，固定值 `0xABCD`    |
| 2    | type        | uint8  | 1      | `0x02` (HANDSHAKE_HELLO_ACK) |
| 3    | payload_len | uint32 | 4      | **大端**，payload 长度       |
| 7    | conv        | uint32 | 4      | **大端**，payload，          |

conv 是 KCP 协议的**会话标识符**（Conversation ID），uint32，由地面站端自增计数器生成，通过 HELLO_ACK 发给设备端。设备端用它创建 KCP 实例（`ikcp_create(conv, ...)`），后续所有 KCP_DATA(type=3) 和 KCP_HEARTBEAT(type=4) 报文都靠这个 conv 值关联到正确的 `KcpSession`。不同设备的 HELLO 会分配不同的 conv，实现多设备会话隔离。

\> ***\*HELLO_ACK 中 conv 为什么用大端\****：握手阶段设备端尚未创建 KCP 实例，无法使用 KCP 库函数读取 conv，因此 HELLO_ACK 的 conv 采用与 UDP 外层一致的***\*大端\****序，设备端直接按大端解析即可。

设备解析conv，创建session，停止发HELLO

#### 2.2.3 KCP_HEARTBEAT（机载端->地面站）

(裸 UDP 包，不走 KCP)

| 偏移 | 字段        | 类型   | 字节数 | 说明                                              |
| ---- | ----------- | ------ | ------ | ------------------------------------------------- |
| 0    | magic       | uint16 | 2      | **大端**，固定值 `0xABCD`                         |
| 2    | type        | uint8  | 1      | 0x04 (KCP_HEARTBEAT）                             |
| 3    | payload_len | uint32 | 4      | **大端**，payload 长度                            |
| 7    | conv        | uint32 | 4      | **小端**（与 KCP 库一致，用 `ikcp_getconv` 读取） |

KCP_HEARTBEAT 用小端的原因（KCP 实例已创建，直接用 `ikcp_getconv` 读取即可

#### 2.2.4 KCP_HEARTBEAT（地面站->机载端）

(裸 UDP 包，不走 KCP)

| 偏移 | 字段        | 类型   | 字节数 | 说明                                              |
| ---- | ----------- | ------ | ------ | ------------------------------------------------- |
| 0    | magic       | uint16 | 2      | **大端**，固定值 `0xABCD`                         |
| 2    | type        | uint8  | 1      | 0x04 (KCP_HEARTBEAT)                              |
| 3    | payload_len | uint32 | 4      | **大端**，payload 长度                            |
| 7    | conv        | uint32 | 4      | **小端**（与 KCP 库一致，用 `ikcp_getconv` 读取） |

KCP_HEARTBEAT 用小端的原因（KCP 实例已创建，直接用 `ikcp_getconv` 读取即可

机载端与地面站互发KCP_HEARTBEAT，维持会话连接，5秒无包 → 地面站清理session

## 3.工作模式设置MAV_CMD_SET_WORKMODE

（自定义指令，51000）(地面站->机载端)

按实际工作流程，提供四种工作模式：即察即打，搜索跟踪，搜索打击，据止搜索。

**(1)即察即打**：无人机执行区域搜索任务时，识别到目标即打击，无需人员介入；

**(2)搜索跟踪**：无人机执行区域搜索任务时，识别到目标进入跟踪模式，操作人员手动切换进入第一视角，自行判断是否打击目标；

**(3)搜索打击**：无人机执行区域搜索任务时，识别到目标后，将目标信息和图片回传到地面指控终端，操作人员可自行选择多无人机进行打击；

**(4)据止搜索**：该工作模式下全程gps据止环境，使用相对坐标系，无人机执行区域搜索任务，回传目标信息和图片

使用COMMAND_LONG自定义指令：

command = 51000

| 字段             | 类型     | 单位 | 说明                                                |
| ---------------- | -------- | ---- | --------------------------------------------------- |
| target_system    | uint8_t  | -    | 目标系统ID                                          |
| target_component | uint8_t  | -    | 目标组件ID                                          |
| command          | uint16_t | -    | 51000                                               |
| confirmation     | uint8_t  | -    | 确认标志（首次发送为0）                             |
| param1           | float    | -    | **0，1，2，3 //即察即打，搜索跟踪，搜索，据止搜索** |
| param2           | float    | -    | 0，1//默认全局坐标系，本地坐标系                    |
| param3~7         | float    | -    | 未使用（保留）                                      |

注：
**全局坐标系即为默认的WGS84经纬度 + 相对Home高度**，纬度、经度采用 WGS84 坐标系；
纬度、经度以 int32 表示，单位为 deg * 1e7；
高度采用相对 Home 点高度，单位为 meter；
航向采用弧度，0 表示北向，顺时针为正。

**本地坐标系**以飞行器本次起飞点为原点，坐标为 (0, 0, 0)；
x 轴指向北，单位 m；
y 轴指向东，单位 m；
z 轴指向地，单位 m，向下为正；
z 为负值表示高于起飞点。

## 4.无人机状态(机载端->地面站)

### 4.1 心跳包HEARTBEAT(0)（mavlink）

| 字段            | 类型     | 单位 | 说明         |
| --------------- | -------- | ---- | ------------ |
| type            | uint8_t  | -    | 无人机类型   |
| autopilot       | uint8_t  | -    | 自驾仪类型   |
| **base_mode**   | uint8_t  | -    | 系统模式位图 |
| **custom_mode** | uint32_t | -    | 自定义模式   |
| system_status   | uint8_t  | -    | 系统状态标志 |
| mavlink_version | uint8_t  | -    | MAVLink版本  |

### 4.2 载荷类型PAYLOAD_STATUS_COMMON(42001,mavlink自定义)

自定义消息，Message ID为42001，用于通过载荷类型区分无人机类型，例如：携带弹药的为”查打一体机“。

| 字段          | 类型     | 单位 | 说明                                                     |
| ------------- | -------- | ---- | -------------------------------------------------------- |
| field_flags   | uint8_t  | -    | 有效字段掩码，目前置为0即可                              |
| payload_type  | uint8_t  |      | 雷达RADAR=5`, `通信中继COMM_RELAY=6`, `弹药MUNITION=7    |
| payload_state | uint8_t  |      | 载荷运行状态：0=未激活, 1=激活/运行中（**默认为1即可**） |
| fault_code    | uint16_t |      | 故障/错误码，**0 表示无故障**                            |
| temperature   | float    | degC | 载荷温度（摄氏度）（如不需要，给0即可）                  |

### 4.3 电池状态BATTERY_STATUS(147)（mavlink）

| 字段                  | 类型         | 单位  | 说明                                                         |
| --------------------- | ------------ | ----- | ------------------------------------------------------------ |
| current_consumed      | int32_t      | mAh   | 已消耗电量，-1 表示不提供消耗估算                            |
| energy_consumed       | int32_t      | hJ    | 已消耗能量，-1 表示不提供能量消耗估算                        |
| **temperature**       | int16_t      | cdegC | 电池温度（摄氏度×100），`INT16_MAX` 表示未知                 |
| **voltages[10]**      | uint16_t[10] | mV    | 电芯 1~10 的电压；电池电压 > (UINT16_MAX-1) 时拆分到多个电芯位；未知/无效填 `UINT16_MAX` |
| **current_battery**   | int16_t      | cA    | 电池电流（安培×100），-1 表示未测量                          |
| **id**                | uint8_t      | -     | 电池ID                                                       |
| `battery_function`    | uint8_t      | —     | 电池功能（动力、航电等），见 `MAV_BATTERY_FUNCTION` 枚举     |
| type                  | uint8_t      | -     | 电池类型/化学体系（锂电、镍氢等），见 `MAV_BATTERY_TYPE` 枚举 |
| **battery_remaining** | int8_t       | %     | 剩余电量百分比 [0-100]，-1 表示不提供估算                    |
| **time_remaining**    | int32_t      | s     | 剩余续航时间（秒），0 表示不提供估算                         |
| `charge_state`        | uint8_t      | —     | 放电程度状态（正常、低电量、故障等），见 `MAV_BATTERY_CHARGE_STATE` 枚举 |
| **voltages_ext[4]**   | uint16_t[4]  | mV    | 扩展单体电压（11-14号单体，0xFFFF=未使用）                   |
| `mode`                | uint8_t      | —     | 电池模式，0=不支持模式上报或正常使用模式                     |
| `fault_bitmask`       | uint32_t     | —     | 故障/健康指示位掩码，见 `MAV_BATTERY_FAULT` 枚举             |

### 4.4 位置GLOBAL_POSITION_INT(33)（mavlink）

| 字段         | 类型     | 单位  | 说明                                         |
| ------------ | -------- | ----- | -------------------------------------------- |
| time_boot_ms | uint32_t | ms    | 系统启动后的时间戳                           |
| **lat**      | int32_t  | degE7 | 纬度（WGS84，1E-7度）                        |
| **lon**      | int32_t  | degE7 | 经度（WGS84，1E-7度）                        |
| **alt**      | int32_t  | mm    | 海拔高度（MSL）                              |
| relative_alt | int32_t  | mm    | 相对地面高度                                 |
| **vx**       | int16_t  | cm/s  | 地面X方向速度（北向为正）                    |
| **vy**       | int16_t  | cm/s  | 地面Y方向速度（东向为正）                    |
| **vz**       | int16_t  | cm/s  | 地面Z方向速度（下向为正）                    |
| hdg          | uint16_t | cdeg  | 航向角（0.0-359.99度，若未知则为UINT16_MAX） |

### 4.5 姿态ATTITUDE(30)（mavlink）

| 字段         | 类型     | 单位  | 说明               |
| ------------ | -------- | ----- | ------------------ |
| time_boot_ms | uint32_t | ms    | 系统启动后的时间戳 |
| **roll**     | float    | rad   | 横滚角             |
| **pitch**    | float    | rad   | 俯仰角             |
| **yaw**      | float    | rad   | 偏航角             |
| rollspeed    | float    | rad/s | 横滚角速度         |
| pitchspeed   | float    | rad/s | 俯仰角速度         |
| yawspeed     | float    | rad/s | 偏航角速度         |

### 4.6 云台姿态MOUNT_ORIENTATION(265)（mavlink）

| 字段         | 类型     | 单位 | 说明                                         |
| ------------ | -------- | ---- | -------------------------------------------- |
| time_boot_ms | uint32_t | ms   | 系统启动后的时间戳                           |
| roll         | float    | deg  | 横滚角（度）                                 |
| **pitch**    | float    | deg  | **俯仰角（度，-90=向下，0=水平，+90=向上）** |
| yaw          | float    | deg  | 偏航角（度）                                 |
| yaw_absolute | float    | deg  | 绝对偏航角（相对于北向）                     |

### 4.7 卫星数GPS_RAW_INT(24) （mavlink）

| 字段                                                         | 类型       | 单位    | 说明                                                         |
| :----------------------------------------------------------- | :--------- | :------ | :----------------------------------------------------------- |
| time_usec                                                    | `uint64_t` | us      | 时间戳                                                       |
| lat                                                          | `int32_t`  | degE7   | 纬度(WGS84, EGM96 ellipsoid)                                 |
| lon                                                          | `int32_t`  | degE7   | 经度(WGS84, EGM96 ellipsoid)                                 |
| alt                                                          | `int32_t`  | 毫米    | 海拔高度（MSL）                                              |
| eph                                                          | `uint16_t` |         | GPS 水平精度因子 HDOP×100，未知填 `UINT16_MAX`               |
| epv                                                          | `uint16_t` |         | GPS 垂直精度因子 VDOP×100，未知填 `UINT16_MAX`               |
| vel                                                          | `uint16_t` | 厘米/秒 | GPS 地面速度，未知填 `UINT16_MAX`                            |
| cog                                                          | `uint16_t` | cdeg    | 地面航向（运动方向，非机头朝向）×100，范围 0~359.99°，未知填 `UINT16_MAX` |
| fix_type                                                     | `uint8_t`  |         | GPS 定位类型，见 `GPS_FIX_TYPE` 枚举                         |
| satellites_visible                                           | `uint8_t`  |         | 可见卫星数，未知填 `UINT8_MAX`                               |
| alt_ellipsoid [++](https://mavlink.io/zh/messages/common.html#mav2_extension_field) | `int32_t`  | 毫米    | WGS84 椭球面以上高度，正值向上                               |
| h_acc [++](https://mavlink.io/zh/messages/common.html#mav2_extension_field) | `uint32_t` | 毫米    | 水平位置不确定度                                             |
| v_acc [++](https://mavlink.io/zh/messages/common.html#mav2_extension_field) | `uint32_t` | 毫米    | 高度不确定度                                                 |
| vel_acc [++](https://mavlink.io/zh/messages/common.html#mav2_extension_field) | `uint32_t` | 毫米/秒 | 速度不确定度                                                 |
| hdg_acc [++](https://mavlink.io/zh/messages/common.html#mav2_extension_field) | `uint32_t` | degE5   | 航向/航迹不确定度（度数×10⁵）                                |
| yaw [++](https://mavlink.io/zh/messages/common.html#mav2_extension_field) | `uint16_t` | cdeg    | 地球坐标系下的偏航角（北为 0）。GPS 不提供偏航填 0；应提供但当前无法提供填 `UINT16_MAX`；正北填 36000 |

### 4.8 指令应答COMMAND_ACK(77) （mavlink）

| 字段                 | 类型     | 单位 | 说明                                                         |
| -------------------- | -------- | ---- | ------------------------------------------------------------ |
| **command**          | uint16_t | -    | 对应的命令ID（MAV_CMD）                                      |
| **result**           | uint8_t  | -    | 命令执行结果，见 `MAV_RESULT` 枚举                           |
| **progress**         | uint8_t  | %    | result` 为 `MAV_RESULT_IN_PROGRESS` 时的进度百分比 [0-100]，未知填 `UINT8_MAX |
| **result_param2**    | int32_t  | -    | 附加结果信息，可携带命令专属的错误原因枚举值（0 表示未使用/未知） |
| **target_system**    | uint8_t  | -    | 命令发送方的系统 ID                                          |
| **target_component** | uint8_t  | -    | 命令发送方的组件 ID                                          |

### 4.9 参数PARAM_VALUE(22) （mavlink）

| 字段             | 类型     | 单位 | 说明                                                         |
| ---------------- | -------- | ---- | ------------------------------------------------------------ |
| **param_value**  | float    | -    | 参数值                                                       |
| **param_count**  | uint16_t | -    | 参数总数                                                     |
| **param_index**  | uint16_t | %    | 当前参数的索引号                                             |
| **param_id[16]** | char[16] | -    | 参数 ID（名称），≤15 字节时以 NULL 结尾，满 16 字节时无 NULL 终止符 |
| **param_type**   | uint8_t  | -    | 参数类型，见 `MAV_PARAM_TYPE` 枚举                           |

### 4.10 系统状态 SYS_STATUS(1)（mavlink）

| 字段                                       | 类型   | 单位 | 说明                                                         |
| :----------------------------------------- | :----- | :--- | :----------------------------------------------------------- |
| `onboard_control_sensors_present`          | uint32 | —    | 位掩码，指示机载控制器和传感器是否存在：0=不存在，1=存在     |
| `onboard_control_sensors_enabled`          | uint32 | —    | 位掩码，指示机载控制器和传感器是否已启用：0=未启用，1=已启用 |
| `onboard_control_sensors_health`           | uint32 | —    | 位掩码，指示机载控制器和传感器是否健康：0=故障，1=正常       |
| `load`                                     | uint16 | d%   | 主循环最大占用率，范围 [0-1000]，应始终低于 1000             |
| `voltage_battery`                          | uint16 | mV   | 电池电压，`UINT16_MAX` 表示飞控未发送                        |
| `current_battery`                          | int16  | cA   | 电池电流，-1 表示飞控未发送                                  |
| `drop_rate_comm`                           | uint16 | c%   | 通信丢包率（UART/I2C/SPI/CAN），所有链路上接收时损坏的包占比 |
| `errors_comm`                              | uint16 | —    | 通信错误计数（UART/I2C/SPI/CAN），所有链路上接收时损坏的包数 |
| `errors_count1`                            | uint16 | —    | 飞控自定义错误计数 1                                         |
| `errors_count2`                            | uint16 | —    | 飞控自定义错误计数 2                                         |
| `errors_count3`                            | uint16 | —    | 飞控自定义错误计数 3                                         |
| `errors_count4`                            | uint16 | —    | 飞控自定义错误计数 4                                         |
| `battery_remaining`                        | int8   | %    | 剩余电量百分比，-1 表示飞控未发送                            |
| `onboard_control_sensors_present_extended` | uint32 | —    | 扩展位掩码，传感器存在状态（扩展位 32-63）                   |
| `onboard_control_sensors_enabled_extended` | uint32 | —    | 扩展位掩码，传感器启用状态（扩展位 32-63）                   |
| `onboard_control_sensors_health_extended`  | uint32 | —    | 扩展位掩码，传感器健康状态（扩展位 32-63）                   |

### 4.11 状态文本STATUSTEXT(253)（mavlink）

| 字段        | 类型     | 单位 | 说明                                                         |
| :---------- | :------- | :--- | :----------------------------------------------------------- |
| `severity`  | uint8    | —    | 状态严重等级，遵循 RFC-5424 定义（0=emergency, 1=alert, 2=critical, 3=error, 4=warning, 5=notice, 6=info, 7=debug） |
| `text[50]`  | char[50] | —    | 状态文本消息，UTF-8 编码，无 NULL 终止符                     |
| `id`        | uint16   | —    | 消息唯一标识，用于将多个分片组装成一条完整的长文本；0 表示是唯一的块可立即输出 |
| `chunk_seq` | uint8    | —    | 当前分片序号（从 0 开始）；text 中出现 NULL 字符视为最后一块 |

### 4.12 飞控版本 AUTOPILOT_VERSION(148)（mavlink）

| 字段                           | 类型      | 单位 | 说明                                                         |
| :----------------------------- | :-------- | :--- | :----------------------------------------------------------- |
| `capabilities`                 | uint64    | —    | 能力位掩码，见 `MAV_PROTOCOL_CAPABILITY` 枚举                |
| `uid`                          | uint64    | —    | 硬件唯一标识（见 `uid2`）                                    |
| `flight_sw_version`            | uint32    | —    | 固件版本号，4 字节编码为：`major.minor.patch.FIRMWARE_VERSION_TYPE` |
| `middleware_sw_version`        | uint32    | —    | 中间件版本号                                                 |
| `os_sw_version`                | uint32    | —    | 操作系统版本号                                               |
| `board_version`                | uint32    | —    | 硬件/板卡版本号（低 8 位为硅片 ID），高 16 位为板卡类型枚举值 |
| `vendor_id`                    | uint16    | —    | 板卡厂商 ID                                                  |
| `product_id`                   | uint16    | —    | 产品 ID                                                      |
| `flight_custom_version[8]`     | uint8[8]  | —    | 固件自定义版本字段，通常为 git hash 前 8 字节                |
| `middleware_custom_version[8]` | uint8[8]  | —    | 中间件自定义版本字段，通常为 git hash 前 8 字节              |
| `os_custom_version[8]`         | uint8[8]  | —    | 操作系统自定义版本字段，通常为 git hash 前 8 字节            |
| `uid2[18]`                     | uint8[18] | —    | 硬件唯一标识（优先于 `uid`，非零时以本字段为准）             |

### 4.13 心跳消息HEARTBEAT

(机载->地面）（kcp, msg_id = 0x0001）

| 偏移      | 字段        | 类型   | 字节数 | 说明                                        |
| :-------- | :---------- | :----- | :----- | :------------------------------------------ |
| 0         | magic       | uint16 | 2      | **大端**，固定值 `0xABEF`                   |
| 2         | version     | uint8  | 1      | 固定值 `0x01`                               |
| 3         | msg_id      | uint16 | 2      | **大端**，`0x0001` (HEARTBEAT)              |
| 5         | sn_len      | uint16 | 2      | **大端**，设备 SN 字符串长度                |
| 7         | payload_len | uint32 | 4      | **大端**，心跳时为 `0x00000000` (body 为空) |
| 11        | sn          | string | sn_len | 设备序列号（例如 `"DEVICE-001"`）           |
| 11+sn_len | payload     | —      | 0      | 心跳消息无 payload（body=""）               |

**注意**：该心跳消息被封装在 KCP_DATA（UDP header type=3）中，经 KCP 可靠传输后到达地面站，与 `kcp_link.cpp` 中 type=4 的 KCP Link 层裸 UDP 心跳是两层独立机制。

### 4.14 设备任务状态CLUSTER_MISSION_STATE

（机载->地面）（kcp, msg_id=0x1002）

| 偏移      | 字段        | 类型   | 字节数      | 说明                         |
| :-------- | :---------- | :----- | :---------- | :--------------------------- |
| 0         | magic       | uint16 | 2           | **大端**，固定值 `0xABEF`    |
| 2         | version     | uint8  | 1           | 固定值 `0x01`                |
| 3         | msg_id      | uint16 | 2           | **大端**，`0x1002`           |
| 5         | sn_len      | uint16 | 2           | **大端**，设备 SN 字符串长度 |
| 7         | payload_len | uint32 | 4           | **大端**，body 的 JSON 长度  |
| 11        | sn          | string | sn_len      | 设备序列号                   |
| 11+sn_len | payload     | string | payload_len | JSON（见下方）               |

**Payload JSON 结构**（定频上报当前执行的技能类型）：

```
{
    "skill_flow_id": "flow-uuid",
    "skill_id": "skill-uuid",
    "skill_type": 1,
    "state": 0
}
```

| 字段            | 类型   | 说明                                                         |
| :-------------- | :----- | :----------------------------------------------------------- |
| `skill_flow_id` | string | 当前任务流 ID，与下发指令中的 `skill_flow_id` 对应           |
| `skill_id`      | string | 当前正在执行的技能 ID,与下发指令中的 `skill_id`对应          |
| `skill_type`    | int    | 当前正在执行的技能类型（100弹射起飞，101集结，102搜索，103返航） |
| state           | int    | 当前技能执行状态（0=未开始, 1=执行中, 2=暂停, 3=完成 等）    |

**上报时机**：设备**定频**（跟随 `TemplatesTransit` 状态变化）上报当前正在执行的技能，业务系统据此跟踪各设备的任务执行进度。

## 5.传感器校准

### 5.1 地面站发送的校准指令使用COMMAND_LONG指令

command = MAV_CMD_PREFLIGHT_CALIBRATION (241)，当前业务只涉及**磁力计、加速度计、陀螺仪**三类传感器校准，每次命令只能进行一类传感器校准，例如**磁力计校准使用param2 = 1，其余校准位应置为0**

### 5.2 校准过程

**协议概述**：PX4 传感器校准使用统一的 `MAV_CMD_PREFLIGHT_CALIBRATION`(241) 命令启动不同类别的校准，通过 param1~param7 区分校准类型。校准过程中的状态反馈全部通过 `[cal]` 前缀的 `STATUSTEXT` 消息实现（不使用 `MAG_CAL_REPORT` 等专用 MAVLink 消息，后者属于 APM 协议）。校准完成后 PX4 固件自动将结果持久化到 FLASH，GCS 无需额外发送 `MAV_CMD_PREFLIGHT_STORAGE(param1=1)`。QGC 仅通过 `_refreshParams()` 回读更新后的 `CAL_*`/`SENS_*` 参数值。

> **版本要求**：QGroundControl 要求 PX4 固件版本 ≥ 1.4.1，校准协议版本号为 `2`（即 `[cal] calibration started: 2 <type>` 中的版本号）。低于此版本或协议版本不匹配时，校准仍可进行但图形化反馈会被禁用，退化为纯文本日志模式。

**`MAV_CMD_PREFLIGHT_CALIBRATION`(241) 参数字段含义**：

| 字段             | 类型     | 单位 | 说明                                                         |
| ---------------- | -------- | ---- | ------------------------------------------------------------ |
| target_system    | uint8_t  | -    | 目标系统ID                                                   |
| target_component | uint8_t  | -    | 目标组件ID                                                   |
| command          | uint16_t | -    | **MAV_CMD_PREFLIGHT_CALIBRATION(241)**                       |
| confirmation     | uint8_t  | -    | `0`=首次发送，`1~255`=重传确认                               |
| param1           | float    | -    | **陀螺仪校准**：`0`=取消当前校准，`1`=启动校准               |
| param2           | float    | -    | **磁力计校准**：`0`=取消当前校准，`1`=启动校准               |
| param3           | float    | -    | -                                                            |
| param4           | float    | -    | -                                                            |
| param5           | float    | -    | **加速度计校准**：`0`=取消当前校准，`1`=完整 6 面校准，`2`=板级微调(水平面) |
| param6           | float    | -    | -                                                            |
| param7           | float    | -    | -                                                            |

**PX4 `[cal]` STATUSTEXT 协议消息汇总（前端显示用）**：

| 消息格式                                             | 阶段   | 说明                                              |
| :--------------------------------------------------- | :----- | :------------------------------------------------ |
| `[cal] calibration started: 2 <type>`                | 启动   | 校准开始，version=2，type 为 accel/mag/gyro/level |
| `[cal] <side> orientation detected`                  | 进行中 | PX4 检测到飞行器已摆放至指定姿态                  |
| `[cal] Hold still, measuring <side> side`            | 进行中 | (仅加速度计) 提示保持静止，正在采样               |
| `[cal] <side> side calibration: progress <N>`        | 进行中 | (仅磁力计) 当前面旋转采集进度 N(0~100)            |
| `[cal] <side> side result: [x y z]`                  | 进行中 | (仅加速度计) 当前面采集的原始加速度值             |
| `[cal] progress <N>`                                 | 进行中 | 总体校准进度百分比 N(0~100)                       |
| `[cal] <side> side done, rotate to a different side` | 进行中 | 当前面采集完成，进入下一面                        |
| `[cal] <side> side already completed`                | 进行中 | 该面已完成，无需重复                              |
| `[cal] calibration done: <type>`                     | 完成   | 校准完成，结果已自动持久化到 FLASH                |
| `[cal] calibration cancelled`                        | 取消   | 校准被取消                                        |
| `[cal] calibration failed`                           | 失败   | 校准失败                                          |

其中 `<side>` 取值及对应 6 面姿态：

| side    | 姿态                 |
| :------ | :------------------- |
| `down`  | 水平正放（底面朝下） |
| `up`    | 倒置（底面朝上）     |
| `left`  | 左侧朝下             |
| `right` | 右侧朝下             |
| `front` | 机头朝下             |
| `back`  | 机尾朝下             |

`CAL_MAG_SIDES` 参数用于配置磁力计所需校准的面数（位掩码：bit0=back, bit1=front, bit2=left, bit3=right, bit4=up, bit5=down），如参数不存在则默认 6 面。

---

#### 5.2.1 陀螺仪校准

**功能说明**：对 IMU 陀螺仪进行零偏校准，飞行器必须保持绝对静止。

**交互流程**：

| 序号 | 方向       | 消息           | 关键字段                               | 说明                                 |
| :--- | :--------- | :------------- | :------------------------------------- | :----------------------------------- |
| 1    | GCS → 机载 | `COMMAND_LONG` | `command=241`, `param1=1`              | 发起陀螺仪校准                       |
| 2    | 机载 → GCS | `COMMAND_ACK`  | `command=241`, `result=5`(IN_PROGRESS) | 校准开始执行                         |
| 3    | 机载 → GCS | `STATUSTEXT`   | `[cal] calibration started: 2 gyro`    | 校准协议启动，版本 2                 |
| 4    | 机载 → GCS | `STATUSTEXT`   | `[cal] down orientation detected`      | 检测到水平姿态，开始采样             |
| 5    | 机载 → GCS | `STATUSTEXT`   | `[cal] calibration done: gyro`         | 校准完成，结果已自动持久化           |
| 6    | GCS(内部)  | —              | `_refreshParams()`                     | 从飞控回读更新后的 CAL_*/SENS_* 参数 |

---

#### 5.2.2 加速度计校准

**功能说明**：对加速度计进行标定。只做完整 6 面校准。PX4 固件**自动检测姿态变化**，GCS 只需发送一次启动命令，无需为每个面发送确认指令。

##### 完整 6 面校准

| 序号 | 方向       | 消息           | 关键字段                                           | 说明                                 |
| :--- | :--------- | :------------- | :------------------------------------------------- | :----------------------------------- |
| 1    | GCS → 机载 | `COMMAND_LONG` | `command=241`, `param5=1`(FULL)                    | 发起 6 面校准                        |
| 2    | 机载 → GCS | `COMMAND_ACK`  | `command=241`, `result=5`(IN_PROGRESS)             | 进入校准模式                         |
| 3    | 机载 → GCS | `STATUSTEXT`   | `[cal] calibration started: 2 accel`               | 校准协议启动                         |
| 4    | 机载 → GCS | `STATUSTEXT`   | `[cal] Place vehicle LEVEL`                        | 提示摆放第 1 面（水平正放）          |
| 5    | —          | —              | —                                                  | 人工摆放到位并静止                   |
| 6    | 机载 → GCS | `STATUSTEXT`   | `[cal] down orientation detected`                  | PX4 自动检测到姿态到位               |
| 7    | 机载 → GCS | `STATUSTEXT`   | `[cal] Hold still, measuring down side`            | 正在采样，须保持静止                 |
| 8    | 机载 → GCS | `STATUSTEXT`   | `[cal] down side result: [x y z]`                  | 当前面采集的原始值                   |
| 9    | 机载 → GCS | `STATUSTEXT`   | `[cal] progress <N>`                               | 总体进度 N(0~100)                    |
| 10   | 机载 → GCS | `STATUSTEXT`   | `[cal] down side done, rotate to a different side` | 当前面完成，提示下一面               |
| ...  | 循环       | 步骤 5~10      | 依次: down→left→right→front→back→up                | PX4 自动检测，GCS 无需发指令         |
| 11   | 机载 → GCS | `STATUSTEXT`   | `[cal] calibration done: accel`                    | 6 面采集完成，结果已自动持久化       |
| 12   | GCS(内部)  | —              | `_refreshParams()`                                 | 从飞控回读更新后的 CAL_*/SENS_* 参数 |

##### 

#### 5.2.3 磁力计校准

**功能说明**：对磁力计进行硬铁/软铁校准。PX4 使用 `[cal]` STATUSTEXT 文本协议反馈状态（不使用 `MAG_CAL_REPORT`），GCS 仅发送一次启动命令，之后 PX4 自动检测旋转姿态。

**CAL_MAG_SIDES 参数**（位掩码，控制校准面对数量）：

| bit  | 面               | 说明 |
| :--- | :--------------- | :--- |
| bit0 | back (机尾朝下)  |      |
| bit1 | front (机头朝下) |      |
| bit2 | left (左侧朝下)  |      |
| bit3 | right (右侧朝下) |      |
| bit4 | up (倒置)        |      |
| bit5 | down (水平正放)  |      |

**交互流程**：

| 序号 | 方向       | 消息           | 关键字段                                           | 说明                                 |
| :--- | :--------- | :------------- | :------------------------------------------------- | :----------------------------------- |
| 1    | GCS → 机载 | `COMMAND_LONG` | `command=241`, `param2=1`                          | 发起磁力计校准                       |
| 2    | 机载 → GCS | `COMMAND_ACK`  | `command=241`, `result=5`(IN_PROGRESS)             | 校准开始                             |
| 3    | 机载 → GCS | `STATUSTEXT`   | `[cal] calibration started: 2 mag`                 | 校准协议启动                         |
| 4    | 机载 → GCS | `STATUSTEXT`   | `[cal] Rotate vehicle around all axes`             | 提示旋转飞行器                       |
| 5    | 机载 → GCS | `STATUSTEXT`   | `[cal] left orientation detected`                  | PX4 自动检测到左侧姿态               |
| 6    | 机载 → GCS | `STATUSTEXT`   | `[cal] left side calibration: progress <N>`        | 当前面旋转采集进度 N(0~100)          |
| 7    | 机载 → GCS | `STATUSTEXT`   | `[cal] left side done, rotate to a different side` | 当前面完成，旋转至下一面             |
| ...  | 循环       | 步骤 5~7       | 按 CAL_MAG_SIDES 配置的面对重复                    | 自动检测，GCS 无需发指令             |
| 8    | 机载 → GCS | `STATUSTEXT`   | `[cal] progress <100>`                             | 所有面采集完成，拟合计算中           |
| 9    | 机载 → GCS | `STATUSTEXT`   | `[cal] calibration done: mag`                      | 校准完成，结果已自动持久化           |
| 10   | GCS(内部)  | —              | `_refreshParams()`                                 | 从飞控回读更新后的 CAL_*/SENS_* 参数 |

---

**`MAV_CMD_PREFLIGHT_STORAGE`(245) 参数字段**（校准流程中 QGC 仅使用 `param3=3`，详见注意事项）：

| param1 | 操作                           | QGC 校准流程中使用                                           |
| :----- | :----------------------------- | :----------------------------------------------------------- |
| `0`    | 从 FLASH 读取参数到 RAM        | 否                                                           |
| `1`    | 将 RAM 参数持久化写入 FLASH    | **否**（PX4 固件在校准完成时内部自动调用 `param_save_default()`） |
| `2`    | 重置参数为默认值但不写入 FLASH | 否                                                           |
| `3`    | 重置参数为工厂默认值           | **是**（`resetFactoryParameters()` 中发送）                  |

**`COMMAND_ACK` 返回码**：

| 值   | 枚举名        | 含义   |
| :--- | :------------ | :----- |
| `0`  | `ACCEPTED`    | 成功   |
| `5`  | `IN_PROGRESS` | 执行中 |

---

**注意事项**：

- 校准须在飞控未解锁状态下执行（`HEARTBEAT.base_mode` 含 `MAV_MODE_PREFLIGHT` 或 `MAV_MODE_FLAG_SAFETY_ARMED==0`）
- `MAV_CMD_PREFLIGHT_CALIBRATION` 单次命令只能指定一种传感器校准，所有 param 设为 `0` 表示取消当前校准
- **PX4 校准状态反馈全部通过 `[cal]` 前缀的 STATUSTEXT 消息，不使用 `MAG_CAL_REPORT`(APM 专用) 或 `MAV_CMD_ACCELCAL_VEHICLE_POS`(APM 专用)**
- PX4 校准完成后固件内部自动调用 `param_save_default()` 将结果持久化到 FLASH，GCS 无需发送 `MAV_CMD_PREFLIGHT_STORAGE(param1=1)`。QGC 的 `_refreshParams()` 仅从飞控回读更新后的参数值
- `COMMAND_LONG.target_system` 设飞控 `system_id`(1~254)，`target_component` 为 `MAV_COMP_ID_AUTOPILOT1`(1)
- PX4 加速度计/磁力计校准过程中，GCS **不需要**为每个姿态面发送确认指令，PX4 固件会自动检测姿态变化并通过 `[cal]` STATUSTEXT 通知

## 6.跟踪状态回传TRACKING_STATE

（自定义消息，43000）（机载->地面）

自定义状态消息，**msg id = 43000，用于**搜索跟踪**模式下，无人机进入目标跟踪状态后，上报给地面站

| 字段           | 类型     | 单位 | 说明                           |
| -------------- | -------- | ---- | ------------------------------ |
| time_boot_ms   | uint64_t | ms   | 系统启动后的时间戳             |
| tracking_state | uint8_t  | -    | 跟踪状态（0=未跟踪，1=跟踪中） |

## 7.视频流开启与关闭MAV_CMD_MEDIA_SWITCH

（自定义指令，50003）（地面站->机载端）

双方提前约定好rtsp地址格式，地面站主动请求某架无人机的第一视角视频流，天空端接收到请求后，将指定无人机的视频流推送到**地面站流媒体服务器**，地面站前端从本地流媒体服务器拉取视频流进行播放

rtsp地址格式：rtsp://{net_ip}:8554/{sn}/live/stream{camera_id}/{channel}

使用COMMAND_LONG自定义指令：

command = 50003

| 参数              | 类型     | 单位 | 说明                                                |
| ----------------- | -------- | ---- | --------------------------------------------------- |
| **target_system** | uint8_t  | -    | **目标系统ID（天空端）**                            |
| target_component  | uint8_t  | -    | 目标组件ID                                          |
| command           | uint16_t | -    | 指令ID（50003）                                     |
| **param1**        | float    |      | **camera_id：0,1,2**                                |
| **param2**        | float    |      | **switch：0=关闭推流, 1=开启推流**                  |
| param3            | float    |      | channel:0:可见光  1:红外  2:带算法框的视频流 3:点云 |
| param4~7          | float    | -    | 未使用（保留）                                      |

## 8.跟踪模式下的打击/取消打击MAV_CMD_ATTACK

(自定义指令，51001)（地面站->机载端）

根据tzs要求，打击即解保

适用于**搜索跟踪**模式下，对已进入跟踪状态的无人机下发打击/取消打击指令

使用COMMAND_LONG自定义指令：

command = 51001

| 参数              | 类型     | 单位  | 说明                     |
| ----------------- | -------- | ----- | ------------------------ |
| **target_system** | uint8_t  | -     | **目标系统ID（天空端）** |
| target_component  | uint8_t  | -     | 目标组件ID               |
| command           | uint16_t | -     | 指令ID（51001）          |
| **param1**        | float    | **-** | 0=打击，1=取消打击       |
| param2~7          | float    | -     | 未使用（保留）           |

## 9.任务流CLUSTER_MISSION_CTL（地面→ 机载）（kcp）

**消息名称**: CLUSTER_MISSION_CTL
**消息 ID**: `0x1001`
**方向**: 地面→ 机载
**场景**: 任务部署后，将已规划航线、拆分为单机的复杂任务指令下发至对应设备

------

### 9.1 传输协议分层

| 层           | 说明                                                         |
| :----------- | :----------------------------------------------------------- |
| **UDP 外层** | `Magic(2B, 0xABCD)` + `Type(1B)` + `PayloadLen(4B)`；（KCP 下层承载） |
| **KCP 层**   | KCP 协议自行封包，业务层无需关心                             |
| **业务层**   | 本规范定义的可变长二进制帧，携带设备 SN 和 JSON 格式的任务数据 |

### 9.2业务层

#### 9.2.1帧结构如下：

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|            Magic (2B)          |   Version (1B)|              |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|             MsgID (2B)         |            SNLen (2B)        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          PayloadLen (4B)                      |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                         SN (SNLen bytes)                      |
|                               ...                             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     Payload (PayloadLen bytes)                |
|                               ...                             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

#### 9.2.2 业务层帧头字段

| 偏移 | 字段         | 长度 | 字节序 | 说明                         |
| :--- | :----------- | :--- | :----- | :--------------------------- |
| 0    | `Magic`      | 2    | 大端   | 固定值 `0xABEF`，用于帧同步  |
| 2    | `Version`    | 1    | —      | 固定值 `1`                   |
| 3    | `MsgID`      | 2    | 大端   | 消息类型，本消息为 `0x1001`  |
| 5    | `SNLen`      | 2    | 大端   | SN 字段的字节长度            |
| 7    | `PayloadLen` | 4    | 大端   | Payload 的字节长度           |
| 11   | `SN`         | 变长 | —      | 目标设备序列号，UTF-8 字符串 |
| —    | `Payload`    | 变长 | —      | 任务数据，UTF-8 编码的 JSON  |

**帧头总长度**: 11 字节（不含 SN 和 Payload）

**约束**:

- `SNLen` ≤ 1024
- `PayloadLen` ≤ 10 MB

------

#### 9.2.3 Payload 数据结构（JSON Schema）

Payload 为一个 JSON 对象，即**技能编排**，顶层结构如下：

```
{
  "skill_flow_id": "<string>",
  "devices_sn": ["<string>"],
  "skills": [<skill>, <skill>, ...],
  "constraints": { "fence": [...], "threat_area": [...] }
}
```

| 字段            | 类型     | 必填 | 说明                                                         |
| :-------------- | :------- | :--- | :----------------------------------------------------------- |
| `skill_flow_id` | string   | 是   | 技能编排的唯一标识，同一批次下发给同一设备的多个 skill 共享此 ID |
| `devices_sn`    | string[] | 是   | 目标设备 SN 列表（部署后已拆为单机，通常仅有 1 个元素）      |
| `skills`        | object[] | 是   | 有序的原子技能（单个任务）列表，按数组顺序依次执行，元素可单个，可多个 |
| `constraints`   | object   | 否   | 约束条件，包含围栏(`fence`)和威胁区域(`threat_area`)         |

#### 9.2.4 原子技能 `skill`（单个任务）

```
{
  "skill_type": <int>,
  "skill_id": "<string>",
  "skill_ctl": 1,
  "devices_sn": ["<string>"],
  "arrive_path": <waypoints>,
  "skill_area": <area>,
  "skill_area_path": [<device_waypoints>, ...],
  "skill_area_flag": <int>,
  "skill_params": <skill_params>
}
```

| 字段              | 类型     | 必填 | 说明                                                         |
| :---------------- | :------- | :--- | :----------------------------------------------------------- |
| `skill_type`      | int      | 是   | 技能类型（见 9.2.5 枚举）                                    |
| `skill_id`        | string   | 是   | 技能唯一标识                                                 |
| `skill_ctl`       | int      | 是   | 执行控制：`1`=上传部署(默认立即执行) `2`=暂停 `3`=恢复 `0`=停止 |
| `devices_sn`      | string[] | 是   | 执行该技能的目标设备列表                                     |
| `arrive_path`     | object   | 否   | 到达任务区域前的航线（waypoints）                            |
| `skill_area`      | object   | 否   | 任务执行区域，geo-fence 或兴趣区                             |
| `skill_area_path` | object[] | 否   | 任务区域内的航线，按设备 SN 分发                             |
| `skill_area_flag` | int      | 是   | 区域描述方式：<br />`0`=无区域，skill_area无效； <br />`1`= `skill_area` 有效，描述区域，不做区域内航线生成； <br />`2`= `skill_area_path` 有效，区域内生成的航线，对应每个设备<br /> `3`=`skill_area`和`skill_area_path`两者均有效 |
| `skill_params`    | object   | 是   | 技能执行参数（见 9.2.6）                                     |

####  9.2.5 `skill_type` 枚举

| 值   | 名称    | 说明           |
| :--- | :------ | :------------- |
| 100  | Takeoff | 弹射起飞       |
| 101  | Gather  | 集结           |
| 102  | Search  | 搜索           |
| 103  | Return  | 返航           |
| 104  | Relay   | 中继(暂不实现) |
| 105  | Attack  | 多目标打击     |

#### 9.2.6 `skill_params` 参数

| 字段                  | 类型  | 单位 | 说明                            |
| :-------------------- | :---- | :--- | :------------------------------ |
| takeoff_height        | float | m    | 起飞到达高度                    |
| takeoff_time_interval | float | s    | 弹射时间间隔                    |
| gather_speed          | float | m/s  | 集结速度                        |
| gather_height         | float | m    | 集结高度                        |
| search_speed          | float | m/s  | 搜索速度                        |
| search_height         | float | m    | 搜索高度                        |
| safe_distance         | float | m    | 安全距离                        |
| formation             | int   | -    | 队形：0一字形，1圆形，2无编队   |
| return_speed          | float | m/s  | 返航速度                        |
| return_height         | float | m    | 返航高度                        |
| target_type           | int   | -    | 目标类型：0:persion, 1:car，... |
| max_duration          | float | s    | 技能最长执行时长                |

------

#### 9.2.7  航线waypoints

其中`seq` 是预留的冗余标识，一般填 0 

```
{
  "frame": 0,
  "points": [
    {
      "x": <double>,
      "y": <double>,
      "z": <double>,
      "seq": <int>,
      "params": [<float>, ...],
      "actions": [
        {
          "action_type": <int>,
          "action_params": [<float>, ...]
        }
      ]
    }
  ],
  "global_params": [<float>, ...]
}
```

| 字段                               | 类型    | 说明                                                         |
| :--------------------------------- | :------ | :----------------------------------------------------------- |
| `frame`                            | int     | 坐标系：`0`=WGS84 `1`=WGS84相对高度 `2`=WGS84地形高度 `10`=LOCAL_ENU `11`=LOCAL_NED `20`=BODY_FRD `30`=MAP |
| `points[].x`                       | double  | 纬度（°），当 frame=0                                        |
| `points[].y`                       | double  | 经度（°），当 frame=0                                        |
| `points[].z`                       | double  | 高度（m），当 frame=0                                        |
| `points[].seq`                     | int     | 航点序号                                                     |
| `points[].actions[].action_type`   | int     | 航点动作类型                                                 |
| `points[].actions[].action_params` | float[] | 航点动作参数                                                 |
| `global_params`                    | float[] | 航线全局参数                                                 |

#### 9.2.8 区域 `area`

```
{
  "type": <int>,
  "data": [<double>, ...]
}
```

| `type` | 含义   | `data` 格式                              |
| :----- | :----- | :--------------------------------------- |
| 0      | 点     | `[lat, lon]`                             |
| 1      | 圆     | `[lat, lon, radius, _]`                  |
| 2      | 多边形 | `[lat1, lon1, lat2, lon2, ...]`          |
| 3      | 空心圆 | `[lat, lon, outer_radius, inner_radius]` |

#### 9.2.9 设备航线 `device_waypoints`

```
{
  "device_sn": "<string>",
  "waypoints": <waypoints>
}
```

将航线与设备 SN 绑定，`skill_area_path` 中每条航线对应一台设备。

#### 9.2.10 约束条件 `constraints`

```
{
  "fence": [<area>, ...],
  "threat_area": [<area>, ...]
}
```

| 字段          | 说明                       |
| :------------ | :------------------------- |
| `fence`       | 电子围栏区域列表，禁止进入 |
| `threat_area` | 威胁区域列表               |

------

### 9.3 完整示例

解析实例见**附件：1**

一个部署到设备 `425UAV001` 的"起飞->集结->搜索 "任务：

```
{
  "skill_flow_id": "flow_20240604_001",
  "devices_sn": ["425UAV001"],
  "skills": [
    {
      "skill_type": 100,
      "skill_id": "takeoff_001",
      "skill_ctl": 1,
      "devices_sn": ["425UAV001"],
      "skill_params": {
        "takeoff_height": 30,
        "takeoff_time_interval": 10.0
      }
    },
    {
      "skill_type": 101,
      "skill_id": "gather_001",
      "skill_ctl": 1,
      "devices_sn": ["425UAV001"],
      "arrive_path": {
        "frame": 0,
        "points": [
          { "x": 31.2304, "y": 121.4737, "z": 100.0, "seq": 0 },
          { "x": 31.2400, "y": 121.4800, "z": 80.0,  "seq": 1 }
        ],
        "global_params": []
      },
      "skill_area": { "type": 1, "data": [31.24, 121.48, 300.0, 0.0] },
      "skill_area_flag": 1,
      "skill_params": {
        "gather_speed": 10.0,
        "gather_height": 30.0
      }
    },
    {
      "skill_type": 102,
      "skill_id": "search_001",
      "skill_ctl": 1,
      "devices_sn": ["425UAV001"],
      "arrive_path": {
        "frame": 0,
        "points": [
          { "x": 31.2304, "y": 121.4737, "z": 100.0, "seq": 0 },
          { "x": 31.2400, "y": 121.4800, "z": 80.0,  "seq": 1 }
        ],
        "global_params": []
      },
      "skill_area": { "type": 1, "data": [31.24, 121.48, 300.0, 0.0] },
      "skill_area_flag": 1,
      "skill_area_path": [
        {
            "device_sn": "425UAV001",
            "waypoints": $waypoints$
        },
        {
            "device_sn": "425UAV002",
            "waypoints": $waypoints$
        },
        {
            "device_sn": "425UAV003",
            "waypoints": $waypoints$
        }
    ],
      "skill_params": {
        "search_speed": 10.0,
        "search_height": 30.0
      }
    }
  ],
  "constraints": {
    "fence": [
      { "type": 2, "data": [31.22, 121.46, 31.22, 121.50, 31.26, 121.50, 31.26, 121.46] }
    ],
    "threat_area": []
  }
}
```

------

### 9.4 关键约定

1. **部署后任务已按单机拆分**：一个 `skill_flow` 在一次下发中仅针对一台设备，`devices_sn` 顶层数组通常只有 1 个元素。
2. **SN 取帧头，不取 JSON**：目标设备 SN 同时存在于业务层帧头的 `SN` 字段和 JSON `devices_sn` 字段中，机载端应以帧头 `SN` 为准进行路由。
3. **skills 有序执行**：`skills` 数组有序，设备应按数组顺序依次执行每个技能，前一个完成后进入下一个。
4. **坐标系默认 WGS84**：`frame=0` 时航点坐标 `x=纬度(°)`、`y=经度(°)`、`z=高度(m)`。
5. **可选字段为 null 时写为缺省**：`arrive_path`、`skill_area`、`skill_params` 等可选字段，若不存在则在 JSON 中不出现该 key，而非设为 `null`。
6. **约束区域在技能间共享**：`constraints` 作用于整个 `skill_flow`，所有 `skills` 均需遵守。

## 10.目标打击

接口定义同 CLUSTER_MISSION_CTL（0x1001），Payload 为 CommSkillFlow 结构。

| 参数              | 值                                             |
| :---------------- | :--------------------------------------------- |
| `skill_type`      | `105`（ATTACK）                                |
| `skill_ctl`       | `1`                                            |
| `skill_area_flag` | `1`                                            |
| `arrive_path`     | 航线                                           |
| `skill_area`      | 打击目标区域(目标点为圆心，杀伤范围为半径的圆) |

`skill_params`：

```
{
  "target_type": 0, //目标类型：0:persion, 1:car，...
  ...
}
```

完整示例：

```
{
  "skill_flow_id": "tzs_attack_20260605_001",
  "devices_sn": ["425UAV001"],
  "skills": [
    {
      "skill_type": 105,
      "skill_id": "attack_425UAV001_001",
      "skill_ctl": 1,
      "devices_sn": ["425UAV001"],
      "arrive_path": {
        "frame": 0,
        "points": [
          { "x": 31.2304, "y": 121.4737, "z": 100.0, "seq": 0 },
          { "x": 31.2315, "y": 121.4742, "z": 105.0, "seq": 1 },
          { "x": 31.2330, "y": 121.4748, "z":  80.0, "seq": 2 },
          { "x": 31.2340, "y": 121.4750, "z":  50.0, "seq": 3 }
        ],
        "global_params": []
      },
      "skill_area": {
        "type": 0,
        "data": [31.2340, 121.4750]
      },
      "skill_area_flag": 1,
      "skill_params": {
        "target_type": [0]
      }
    }
  ],
  "constraints": {
    "fence": [],
    "threat_area": []
  }
}
```

## 11.目标详情和图片回传

DEVICE_TARGETS(机载->地面)（msg_id=0x2001 ）

### 11.1 UDP 外层

使用 `htons`/`htonl`，**网络字节序（大端）**。

```
struct __attribute__((packed)) UdpHeader {
    uint16_t magic;       // 0xABCD
    uint8_t  type;        // 1=HELLO, 2=HELLO_ACK, 3=KCP_DATA, 4=KCP_HEARTBEAT
    uint32_t payload_len; // KCP 有效载荷长度
};
```

| 偏移 | 字段          | 类型   | 字节数 | 说明                                                 |
| :--- | :------------ | :----- | :----- | :--------------------------------------------------- |
| 0    | `magic`       | uint16 | 2      | 固定魔数 `0xABCD`（大端）                            |
| 2    | `type`        | uint8  | 1      | 包类型：1=握手请求, 2=握手应答, 3=KCP数据, 4=KCP心跳 |
| 3    | `payload_len` | uint32 | 4      | 后续 KCP 数据的字节长度，（大端）                    |

------

### 11.2 业务层

**帧头字段为大端序**，Body（Payload）字节序见各消息定义。KCP 可靠传输层保证完整到达，上层从 KCP 流中按此格式切割帧。

```
| Magic(2) | Version(1) | MsgID(2) | SNLen(2) | PayloadLen(4) | SN(SNLen) | Body(PayloadLen) |
```

| 偏移      | 字段          | 类型    | 字节数      | 说明                                        |
| :-------- | :------------ | :------ | :---------- | :------------------------------------------ |
| 0         | `magic`       | uint16  | 2           | 固定魔数 `0xABEF`（大端）                   |
| 2         | `version`     | uint8   | 1           | 协议版本号，当前为 1                        |
| 3         | `msg_id`      | uint16  | 2           | 消息类型：`0x2001` = DEVICE_TARGETS（大端） |
| 5         | `sn_len`      | uint16  | 2           | 设备序列号字节长度（大端）                  |
| 7         | `payload_len` | uint32  | 4           | Body 部分的字节长度（大端）                 |
| 11        | `sn`          | char[]  | sn_len      | 设备序列号字符串                            |
| 11+sn_len | `body`        | uint8[] | payload_len | 目标信息+图片二进制数据（见11.2.1）         |

------

### 11.2.1 Body （DEVICE_TARGETS 的 Payload）

**主机字节序（小端）**。

#### 11.2.1.1 Body 头部

| 偏移 | 字段        | 类型   | 字节数 | 说明                                        |
| :--- | :---------- | :----- | :----- | :------------------------------------------ |
| 0    | `report_id` | uint32 | 4      | 上报批次 ID，取当前毫秒时间戳低 32 位，小端 |
| 4    | `obj_count` | uint32 | 4      | 本批次包含的目标数量 N，小端                |

#### 11.2.1.2 DetectObject（每个目标，共 obj_count 个）

| 偏移      | 字段         | 类型     | 字节数   | 说明                                    |
| :-------- | :----------- | :------- | :------- | :-------------------------------------- |
| 0         | `ts_us`      | int64    | 8        | 检测时间戳，微秒（Unix 纪元），小端     |
| 8         | `dev_lat`    | double   | 8        | 无人机纬度（度），小端                  |
| 16        | `dev_lon`    | double   | 8        | 无人机经度（度），小端                  |
| 24        | `dev_alt`    | double   | 8        | 无人机海拔高度（米，MSL），小端         |
| 32        | `dev_yaw`    | double   | 8        | 无人机偏航角（**弧度**），小端          |
| 40        | `obj_lat`    | double   | 8        | 目标纬度（度），小端                    |
| 48        | `obj_lon`    | double   | 8        | 目标经度（度），小端                    |
| 56        | `obj_alt`    | double   | 8        | 目标海拔高度（米，MSL），小端           |
| 64        | `label`      | uint32   | 4        | 目标类别标签 ID，小端                   |
| 68        | `confidence` | float    | 4        | 置信度（0.0~1.0），当前固定填 0.0，小端 |
| 72        | `type`       | uint8    | 1        | 目标类型：0=未知，小端                  |
| 73        | `img_fmt`    | uint8    | 1        | 图片格式：1=JPEG，小端                  |
| 74        | `reserved`   | uint8[3] | 3        | 保留字段，填 0                          |
| 77        | `sn_len`     | uint16   | 2        | 设备序列号字节长度，小端                |
| 79        | `img_size`   | uint32   | 4        | JPEG 图片数据的字节长度，小端           |
| 83        | `sn_data`    | char[]   | sn_len   | 设备序列号字符串                        |
| 83+sn_len | `img_data`   | uint8[]  | img_size | 原始 JPEG 图片字节流                    |

------

### 11.3 注意事项

1. **字节序分层**：
   - UDP 帧头 → **大端**（`htons`/`htonl`）
   - 业务层帧头 → **帧头字段为大端序**，Body（Payload）字节序见各消息定义
   - Body 数据 → **小端**
2. **图片格式**：`img_fmt=1` 表示 JPEG，`img_data` 为完整 JPEG 文件二进制。无额外编码或压缩。
3. **confidence 和 type**：若机载算法不提供这两个字段，固定填 `confidence=0.0`、`type=0`。
4. **KCP 可靠传输**：KCP 参数为快速模式（nodelay=1, interval=10ms, resend=2, nc=1），窗口 128，MTU 1400。握手通过 HELLO/HELLO_ACK 包建立会话后，所有数据走 `type=3 (KCP_DATA)`。
5. **无分片机制**：单条 `DEVICE_TARGETS` 消息携带同一批次的所有目标及对应图片，Body 长度上限 见一位10MB，超出直接丢弃。

## 12.设置模式

COMMAND_LONG(76)，MAV_CMD_CTRL(50000) ctl_type=10，用于设置设备飞行/运行模式

| 字段   | 类型  | 单位 | 说明                                                         |
| ------ | ----- | ---- | ------------------------------------------------------------ |
| param1 | float | -    | ctl_type，固定值 10（set_mode）                              |
| param2 | float | -    | 模式编号，设备端定义（例如手动模式具体对应哪种需根据机载端定义） |
| param3 | float | -    | 保留，填 0                                                   |
| param4 | float | -    | 保留，填 0                                                   |
| param5 | float | -    | 保留，填 0                                                   |
| param6 | float | -    | 保留，填 0                                                   |
| param7 | float | -    | 保留，填 0                                                   |

## 13.遥控器杆量控制

MANUAL_CONTROL(69)，用于地面站通过手柄控制无人机

| 字段    | 类型     | 单位 | 说明                                           |
| ------- | -------- | ---- | ---------------------------------------------- |
| target  | uint8_t  | -    | 目标系统ID                                     |
| x       | int16_t  | -    | X轴杆量（俯仰，-1000~1000，负=前，正=后）      |
| y       | int16_t  | -    | Y轴杆量（横滚，-1000~1000，负=左，正=右）      |
| z       | int16_t  | -    | Z轴杆量（油门，0~1000，0=最小，1000=最大）     |
| r       | int16_t  | -    | R轴杆量（偏航，-1000~1000，负=左转，正=右转）  |
| buttons | uint16_t | -    | 按钮状态位（bit6控云台pitch+,bit7控云台pitch-) |

地面站只做透传，建议机载端映射如下：

| 统一轴 | UAV解释 | 飞控内部 |
| ------ | ------- | -------- |
| x      | 前后杆  | pitch    |
| y      | 左右杆  | roll     |
| z      | 油门杆  | throttle |
| r      | 偏航杆  | yaw      |

| 模式     | 左手上下 | 左手左右 | 右手上下 | 右手左右 |
| :------- | :------- | :------- | :------- | :------- |
| **主流** | 油门(z)  | 偏航(r)  | 俯仰(x)  | 横滚(y)  |