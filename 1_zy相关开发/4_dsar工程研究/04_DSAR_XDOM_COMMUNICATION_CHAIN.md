# DSAR-HQ X-DOM 通信链路完整分析 —— JSON信号适配、代码生成、端到端数据流

> 覆盖：JSON配置架构 → 代码生成管道 → SOC侧完整调用链 → MCU侧完整调用链 → 四条端到端数据流 → 平台函数参考表 → DCMS命名空间 → 产品变体对比
> 基于 fawhq_e001_10 车型，工具版本 v0.17

---

## 一、总体架构：两侧 + 四路 + 两种JSON

```
┌──────────────────────────────────────────────────────────────────────────┐
│                            X-DOM 通信总体架构                              │
│                                                                          │
│  ┌──────────┐    CAN/Ethernet     ┌──────────┐    DCMS IPC     ┌────────┐│
│  │ 外部ECU   │ ←──────────────→  │ MCU (FW) │ ←────────────→ │ SOC    ││
│  │          │   AUTOSAR COM栈     │          │   跨核通信      │ (APP)  ││
│  └──────────┘                    └──────────┘                └────────┘│
│                                                                          │
│  两种JSON驱动代码生成:                                                     │
│  ├── x_dom_can_rt_*.json     → CAN信号跨域路由 (SOC侧+MCU侧)              │
│  └── x_dom_someip_rt_*.json  → SOME/IP服务跨域路由 (仅SOC侧)             │
│                                                                          │
│  四路数据流:                                                              │
│  A路: CAN→MCU→SOC (外部CAN信号上行到AD应用)                               │
│  B路: SOC→MCU→CAN (AD控制命令下行到CAN总线)                               │
│  C路: SOC→MCU (IMU传感器等SOC数据下发到MCU)                               │
│  D路: MCU→SOC (雷达原始数据等MCU数据上报到SOC)                            │
└──────────────────────────────────────────────────────────────────────────┘
```

### 1.1 文件位置总览

```
适配仓(dsar-hq) 车型目录:
  SOC侧:
    src/dsar_app/product/faw/fawhq_e001_10/
      ├── proxy/
      │   ├── x_dom_can_rt_gen_com/        ★ SOC CAN路由生成代码 (COM域)
      │   ├── x_dom_can_rt_gen_uds/        ★ SOC CAN路由生成代码 (UDS域)
      │   ├── x_dom_someip_rt_gen_com/     ★ SOC SOME/IP路由生成代码 (COM域)
      │   └── x_dom_someip_rt_gen_uds/     ★ SOC SOME/IP路由生成代码 (UDS域)
      │
  MCU侧:
    src/dsar_fw/product/faw/fawhq_e001_10/
      ├── x_dom_can_rt_config/             ★ MCU CAN路由生成代码
      │   ├── x_dom_can_rt_cfg.c/h         路由表+信号定义+CAN帧缓存
      │   ├── x_dom_can_rt_sigif.c/h       SigIf_Get/Set信号读写函数
      │   └── x_dom_can_rt_dbg.c/h         调试shell命令
      │
平台仓(dsar-hq-plat) 运行时引擎:
  dsar-plat-bf/dsar_app/cdd/
    ├── x_dom_can_rt/
    │   ├── msg_route_soc.cpp              SOC CAN帧解析引擎
    │   ├── sig_route_soc.cpp              SOC CAN信号路由
    │   ├── x_dom_can_rt_soc.cpp           SOC 初始化入口
    │   └── p2p_parser.cpp                 P2P解析器
    ├── x_dom_someip_rt/
    │   ├── service_route.cpp              SOC SOME/IP路由引擎
    │   ├── x_dom_someip_rt.cpp            SOC 初始化入口
    │   └── x_dom_someip_rt_com_cfg.h      全局配置结构体
    └── dcms_adapt/
        └── dcms_mcu_api.cpp               DCMS API封装

  dsar-sip/dsar_fw/sip_extend/x_dom_can_rt/
    ├── x_dom_can_rt_mcu.c                 MCU CAN路由运行时
    └── msg_route_mcu.c                    MCU CAN帧路由
```

---

## 二、JSON配置文件架构

### 2.1 CAN跨域路由JSON (x_dom_can_rt_*.json)

**输入文件**位于代码生成工具的配置目录，作用于SOC侧和MCU侧：

```json
{
  "product_name": "fawhq_e001_10",
  "tool_version": "v0.17",
  "routes": [
    {
      "name": "dji_uds_service",        // 路由目标域
      "read_mcu_rx_sigs": {
        "select_mode": "include",        // 或 "exclude"
        "signals": ["IBC_BrakePedalStatus_IBC_1", ...]
      },
      "read_mcu_tx_sigs": { ... },
      "write_to_mcu_sigs": { ... },
      "read_mcu_tx_msgs": { ... },
      "read_mcu_rx_msgs": { ... },
      "write_to_vip_sigs": { ... }
    },
    {
      "name": "dji_application",         // AD域路由
      ...
    },
    {
      "name": "dji_com_service",         // COM域自身路由
      ...
    },
    {
      "name": "mcu",                     // MCU侧配置
      ...
    }
  ],
  "dbc_files": [...],
  "flags": {
    "com_realtime_fwd_to_app": true,     // COM域实时转发到APP
    "mcu_ecu_shell_debug": false
  }
}
```

**六种信号操作类型**：

| 操作类型 | 方向 | 说明 | 例 |
|---------|------|------|----|
| `read_mcu_rx_sigs` | CAN→MCU→SOC | MCU收到的CAN信号，转发给SOC | IBC_BrakePedalStatus |
| `read_mcu_tx_sigs` | SOC→MCU→CAN | SOC发出的控制信号，MCU转发到CAN | VDC_DecelerationReq |
| `write_to_mcu_sigs` | SOC→MCU→CAN | SOC信号写入，MCU调用Com_SendSignal | ADV_SteeringWheelAngleReq |
| `read_mcu_tx_msgs` | SOC→MCU→CAN | 读整个CAN帧(TX方向) | 整帧监控 |
| `read_mcu_rx_msgs` | CAN→MCU→SOC | 读整个CAN帧(RX方向) | 整帧监控 |
| `write_to_vip_sigs` | SOC→MCU(内部) | SOC信号写入MCU内部VIP层 | 内部通信 |

### 2.2 SOME/IP跨域路由JSON (x_dom_someip_rt_*.json)

**仅作用于SOC侧**，配置SOME/IP服务的跨域转发：

```json
{
  "product_name": "fawhq_e001_10",
  "protocol_version": "V2.0.5",
  "com_app": {
    "consumed_events_cfg": [
      {
        "server_id": 1,
        "service_name": "PS_VM_SetManagerMaster",
        "method_name": "SetMgt_NotifyONchgAccountDataDeleteCmd",
        "direction": "RX",
        "timeout_ms": 100,
        "structs": [ /* 结构体字段定义 */ ],
        "alignment": 8,
        "byte_order": "Motorola",
        "serialization": "someip-mlv2"
      },
      // ... 120+个服务
    ]
  }
}
```

**关键特性**：
- 定义结构体对齐方式（alignment=8表示8字节对齐）、字节序（Motorola）、序列化选项
- 每个服务指定timeout用于超时检测
- `direction: RX` 表示COM域接收后转发到APP域
- `direction: TX` 表示APP域通过COM域发送到外部ECU

---

## 三、代码生成管道

### 3.1 一个JSON → 四个输出目录

```
x_dom_can_rt_fawhq_e001_10.json
        │
        ▼  SIP代码生成工具 (v0.17)
        │
        ├──→ SOC/COM: proxy/x_dom_can_rt_gen_com/
        │      x_dom_can_rt_cfg.cpp/h     路由表 + CanMsg + SigHdl + 转发函数
        │      x_dom_can_rt_sigif.cpp/h    信号 SigIf_Get/Set 函数
        │
        ├──→ SOC/UDS: proxy/x_dom_can_rt_gen_uds/
        │      x_dom_can_rt_cfg.cpp/h      (fawhq_e001_10中几乎为空)
        │      x_dom_can_rt_sigif.cpp/h
        │
        └──→ MCU:     x_dom_can_rt_config/
               x_dom_can_rt_cfg.c/h        路由表 + CAN帧缓存buf + 信号表
               x_dom_can_rt_sigif.c/h      MCU侧 SigIf_Get/Set
               x_dom_can_rt_dbg.c/h        调试shell命令
```

### 3.2 SOC侧生成代码结构 (COM域)

**x_dom_can_rt_cfg.h** — 核心配置头文件：

```cpp
// 功能开关
#define XDOMCANRT_DJI_COM_SERVICE_EN                    1  // COM域=1
#define XDOMCANRT_MSG_ROUTE_EN                          1  // CAN帧路由
#define XDOMCANRT_READ_MCU_RX_SIGS_EN                   1  // 读MCU收到的CAN信号
#define XDOMCANRT_READ_MCU_TX_SIGS_EN                   1  // 读MCU发出的CAN信号
#define XDOMCANRT_WRITE_SIGS_TO_MCU_EN                  1  // 写信号到MCU
#define XDOMCANRT_WRITE_SIGS_TO_VIP_EN                  1  // 写信号到VIP

// 所有读信号的 SigHdl 声明 (模板实例化)
extern SigHdl<float, uint16_t, float, float> VDC_DecelerationReq_AEB_VDC_1_default_mcu_tx_sighdl;
extern SigHdl<uint8_t, uint8_t, uint8_t, uint8_t> IBC_BrakePedalStatus_IBC_1_default_mcu_rx_sighdl;
// ... 17个mcu_tx + 33个mcu_rx = 50个SigHdl

// CAN帧查找表
extern const std::map<CanMsgIdPair, CanMsg*> msg_route_mcu_tx_rx_msgs_map;
// 37个CAN帧 (4个TX + 33个RX)

// 转发函数声明
extern void sig_route_send_mcu_tx_rx_sigs(void);
extern void sig_route_send_write_to_mcu_sigs(void);
```

**x_dom_can_rt_cfg.cpp** — 核心配置实现文件，包含：

1. **CanMsg对象** — 每个CAN帧一个CanMsg缓存对象
2. **SigHdl对象** — 每个信号一个SigHdl，绑定到对应CanMsg的位偏移
3. **msg_route_mcu_tx_rx_msgs_map** — CAN ID → CanMsg* 查找表
4. **sig_route_send_mcu_tx_rx_sigs()** — 读取所有信号物理值，打包成结构体，通过DCMS发送给AD/UDS
5. **sig_route_send_write_to_mcu_sigs()** — 读取AD/UDS通过DCMS发来的写信号，调用SigHdl写入CanMsg缓存

### 3.3 MCU侧生成代码结构

**x_dom_can_rt_cfg.h** — MCU侧配置：

```c
// 功能开关
#define XDOMCANRT_READ_MCU_RX_SIGS_EN          1
#define XDOMCANRT_READ_MCU_TX_SIGS_EN          1
#define XDOMCANRT_WRITE_SIGS_TO_MCU_EN         1
#define XDOMCANRT_WRITE_SIGS_FROM_MCU_TO_VIP_EN  1
#define XDOMCANRT_SUPPORT_E2E_EN               1

// CAN帧原始字节缓存 (write_to_vip方向: SOC→MCU)
extern uint8_t mcu_write_to_vip_msg_VDC_1_default_0x0x118_buf[32];
extern uint8_t mcu_write_to_vip_msg_ADV_2_default_0x0x100_buf[32];
// ... 共9个CAN帧缓存

// CAN RX帧过滤表 (36个RX帧)
extern can_msg_filter_item_with_e2e_t msg_route_rx_msg_filters[36];

// CAN TX帧过滤表 (4个TX帧) — MCU监控SOC发出的CAN信号
extern const can_msg_filter_item_t msg_route_tx_msg_filters[4];

// 信号写表 — SOC→MCU→CAN的信号 (37个信号)
extern const can_sig_base_info_mcu_with_e2e_t msg_route_write_sig_tbl[37];

// 信号读/写函数: 通过宏 SigErr_SendSignal → Com_SendSignal()
#define SigErr_SendSignal(signal_id, signal_data) Com_SendSignal(signal_id, signal_data)
```

**x_dom_can_rt_sigif.c** — MCU侧SigIf信号接口：

每个信号生成两个函数：`SigIf_Get_xxx()` 从CAN原始字节提取物理值，`SigIf_Set_xxx()` 将物理值转换为原始字节写入CAN帧缓存。

---

## 四、SOC侧完整调用链

### 4.1 初始化链

```
dji_bf_app 进程启动
  → dlopen("com") → libfawhq_e001_10_com.so
    → dlsym("app_com_core_init")
      → app_com_core_init()
        → x_dom_can_rt_init("com")          ★ CAN路由初始化
        │   [x_dom_can_rt_soc.cpp]
        │   ├→ 注册DCMS主题:
        │   │   "/x_dom_can_rt/mcu_tx_rx_msgs"    (sub)  接收MCU上传的CAN帧
        │   │   "/x_dom_can_rt/write_to_mcu_msgs"  (pub)  发送写信号到MCU
        │   │   "/x_dom_can_rt/read_sigs/com_app"  (pub)  COM信号到APP
        │   │   "/x_dom_can_rt/read_sigs/uds_app"  (pub)  COM信号到UDS
        │   └→ 注册回调: msg_route_mcu_tx_rx_msgs_topic_cbk
        │
        → x_dom_someip_rt_init("com")       ★ SOME/IP路由初始化
        │   [x_dom_someip_rt.cpp]
        │   ├→ 读取 proxy 注入的 g_x_dom_someip_config
        │   ├→ 注册DCMS主题:
        │   │   "/x_dom_someip_rt/com_to_app/ad_app"  (pub)  COM SOME/IP→AD
        │   │   "/x_dom_someip_rt/com_to_app/uds_app" (pub)  COM SOME/IP→UDS
        │   │   "/x_dom_someip_rt/app_to_com"         (sub)  APP SOME/IP→COM
        │   └→ 注册回调: svc_route_app_to_com_topic_cbk
        │
        → add_period_work(50ms, x_dom_can_rt_main_func)
        → add_period_work(50ms, svc_route_cycle_pub)
```

### 4.2 周期性运行链 (50ms)

```
x_dom_can_rt_main_func()  ← 50ms定时
  [x_dom_can_rt_soc.cpp]
  ├→ msg_route_main_function()
  │   [msg_route_soc.cpp]
  │   └→ msg_route_mcu_tx_rx_can_msgs_proc()
  │       ├→ 从 ThreadSafeQueue 取出 MCU 上传的 CAN 帧
  │       ├→ 构造 key {channel, can_id}
  │       ├→ 查找 msg_route_mcu_tx_rx_msgs_map[key] → CanMsg*
  │       └→ can_msg_ptr->set_data(data, dlc, status)  // 缓存原始字节
  │
  └→ sig_route_main_function()
      [sig_route_soc.cpp]
      └→ sig_route_send_mcu_tx_rx_sigs_proc()  // COM域执行
          → sig_route_send_mcu_tx_rx_sigs()     // extern → proxy定义
            [x_dom_can_rt_gen_com/x_dom_can_rt_cfg.cpp]
            ├→ 遍历所有 SigHdl:
            │   VDC_DecelerationReq_sighdl.signal_if_get(phy)
            │   IBC_BrakePedalStatus_sighdl.signal_if_get(phy)
            │   ... (50个信号)
            ├→ 打包成结构体 sig_route_read_tx_rx_sigs_dji_application_t
            ├→ dcms_mcu_topic_send(DCMS_TOPIC_READ_SIGS_APP, ...) → AD
            └→ dcms_mcu_topic_send(DCMS_TOPIC_READ_SIGS_UDS, ...) → UDS

svc_route_cycle_pub()  ← 50ms定时
  [service_route.cpp]
  ├→ check_and_send_svc_list_via_dcms_on_timeout(
  │     *svc_route_recv_svc_info_map_ptr,  // 120+服务
  │     svc_list_ad_app,
  │     "/x_dom_someip_rt/com_to_app/ad_app")
  │   └→ 遍历服务列表，检查超时+有更新 →打包→DCMS发送
  │
  └→ check_and_send_svc_list_via_dcms_on_timeout(
        *svc_route_recv_svc_info_map_ptr,
        svc_list_uds_app,
        "/x_dom_someip_rt/com_to_app/uds_app")
```

### 4.3 CAN帧接收回调链 (MCU→SOC, 异步)

```
MCU上传CAN帧
  │ DCMS: "/x_dom_can_rt/mcu_tx_rx_msgs"
  ▼
msg_route_mcu_tx_rx_msgs_topic_cbk(data, len)
  [msg_route_soc.cpp]
  ├→ 解析 can_msg_base_with_status_t 结构体
  │   {channel, can_id, timestamp, data[64], dlc, status}
  ├→ 构造 CanMsgCbCtx
  └→ push 到 ThreadSafeQueue (等待50ms周期处理)
```

### 4.4 SOME/IP数据接收回调链 (外部ECU→SOC)

```
外部ECU SOME/IP报文 → Ethernet → COM进程 SoAd → SomeIpTp
  → Cdd_SomeIpTp.c 分发 (846个回调)
    ├─ 732个服务: 空函数体 → LdCom→Com → COM进程内消费 (danvince路径)
    └─ 114个服务:
        x_dom_someip_rt_rx_handle(SERVER_ID, payload, len)
          [x_dom_someip_rt.cpp]
          → svc_route_rx_handle(id, data, len)
            [service_route.cpp]
            ├→ 查找 svc_route_recv_svc_info_map[id] → SvcItemInfo*
            ├→ item_ptr->set_origin_data(id, data, len)  // 缓存原始数据
            └→ 周期打包 → DCMS → AD/UDS (50ms)
```

---

## 五、MCU侧完整调用链

### 5.1 MCU侧X-DOM运行时

MCU侧代码运行在FreeRTOS/SafeRTOS上，单一地址空间，直接函数调用。

```
app_task_handler()  ← RTOS任务
  [appl_worklist.c]
  → 遍历高频工作链表 → 调用各模块注册的handler

x_dom_can_rt 模块 (由JSON生成配置):
  x_dom_can_rt_main_function()
    [dsar-sip/dsar_fw/sip_extend/x_dom_can_rt/]
    │
    ├── read_mcu_rx_sigs 方向 (CAN→MCU→SOC):
    │   1. AUTOSAR Com_ReceiveSignal() 读取CAN信号 (由AUTOSAR COM栈接收)
    │   2. 通过 SigIf 将信号值写入DCMS消息结构体
    │   3. dcms_mcu_topic_send() 发送到SOC
    │
    ├── write_to_mcu_sigs 方向 (SOC→MCU→CAN):
    │   1. 从DCMS接收SOC发来的信号 (在DCMS回调中)
    │   2. 通过 SigIf 将信号值写入CAN帧原始字节缓存
    │   3. Com_SendSignal() 发送到CAN总线
    │
    ├── write_to_vip_sigs 方向 (SOC→MCU内部):
    │   1. 从DCMS接收SOC发来的CAN帧原始数据
    │   2. 写入 mcu_write_to_vip_msg_*_buf[]
    │   3. MCU内部其他模块直接读取这些buf
    │
    └── read_mcu_tx_sigs 方向 (监控SOC发出的CAN信号):
        1. MCU侧监控SOC发到CAN的信号
        2. 读取后通过DCMS上报 (用于调试/记录)
```

### 5.2 MCU侧非X-DOM通信链 (直接AUTOSAR COM + acu_dev + DCMS)

MCU侧除了X-DOM生成的信号路由，还有直接基于AUTOSAR COM协议栈的信号处理：

#### 雷达数据上行链 (MCU→SOC)

```
AUTOSAR COM栈接收CAN帧 (雷达CAN FD)
  → Com_ReceiveSignal() 读取每个雷达目标字段
    [mradar_obtain.c]
    zyt_mradar_com():
      ├→ MRR_Com_Get(RadarModel)     → radar_model
      ├→ MRR_Com_Get(ObjSum)         → object_sum
      ├→ MRR_Com_Get(ObjId)          → objects[i].object_id
      ├→ MRR_Com_Get(DistX)          → objects[i].dist_x
      ├→ MRR_Com_Get(RelVelX)        → objects[i].rel_vel_x
      └→ ... (每个目标35+个字段, 最多39个目标即1365+次信号读取)
        │
        │ 通过 acu_dev (MCU内部设备抽象层) 共享数据
        ▼
    [mradar_info_send.c]
    zyt_mradar_comm_send():  ← 20ms周期
      ├→ acu_dev_read(ACU_MRADAR_GET_RADAR_MODEL)  // 从acu_dev读
      ├→ acu_dev_read(ACU_MRADAR_GET_OBJ_SUM)
      ├→ acu_dev_read(ACU_MRADAR_GET_TIME)
      ├→ acu_dev_read(ACU_MRADAR_GET_FAULT_ST)
      ├→ acu_dev_read(ACU_MRADAR_GET_OBJ)           // 读所有39个目标
      ├→ zyt_radar_proxy_sensortime_update_period() // 时间对齐
      │   → sync_signal_aligned_time_us_by_freq()   // 对齐到指定频率
      └→ zyt_radar_raw_dcms_send(DCMS_TOPIC_MRR_SIGNAL, &data, sizeof(data))
          → dcms_mcu_topic_send_msg()               // DCMS→SOC
```

**acu_dev设备抽象层**：MCU内部各模块之间的数据共享机制。一个模块(如mradar_obtain.c)通过`acu_dev_write()`写入数据，另一个模块(如mradar_info_send.c)通过`acu_dev_read()`读取。类似于发布/订阅模式但在同一地址空间内通过共享内存实现。

**时间对齐机制**：`sync_signal_aligned_time_us_by_freq(&gs_time_align, get_time_us_32bits(), RADAR_RAW_DATA_DCMS_FREQUENCY)` 确保DCMS消息的时间戳对齐到指定频率，保证SOC侧收到数据的时序一致性。

#### IMU传感器数据下行链 (SOC→MCU)

```
SOC进程 (AD域)
  │ DCMS: DCMS_TOPIC_IMU_SENSOR
  ▼
[vehicle_sensor.c] (MCU侧)
vehicle_sensor_imu_recv_cb(msg):  ← DCMS回调
  ├→ memcpy(&vehicle_sensor.sensor_get.imu, msg->data, sizeof(Imu))
  ├→ vehicle_sensor.sensor_get.imu_sensor_time = msg->sensor_time / 1000000.0
  └→ (MCU内部其他模块通过 sensor_dev 接口读取IMU数据)

同时存在备用通路:
vcu_perception_imu_sensor_recv_msg_cb1(ImuData, size, SensorTimeStamp):
  ├→ memcpy(&vehicle_sensor.sensor_get.imu, ImuData, sizeof(Imu))
  └→ sensortime_invalid() 检查时间戳有效性
```

#### 执行器控制链 (MCU内部 AUTOSAR COM)

```
[vehicle_lat.c] (横向控制)
vehicle_lat_get():  ← 100Hz
  ├→ Com_ReceiveSignal(IBC_BrakePedalStatus)     // CAN信号直读
  ├→ Com_ReceiveSignal(VDC_SteeringWheelAngle)   // 方向盘角度
  ├→ Com_ReceiveSignal(EPS_AngleControlSt)       // EPS握手状态
  └→ ... (20+个CAN信号直接读取)

vehicle_lat_set():  ← 100Hz
  └→ (根据控制算法计算后)
      ├→ SigIf_Set_VDC_DecelerationReq(&value)   // 通过生成代码写
      │   → Com_SendSignal(VDC_DecelerationReq, raw)
      ├→ SigIf_Set_ADV_SteeringWheelAngleReq(&angle)
      └→ ...

vehicle_lat_main_function():  ← 100Hz
  → vehicle_lat_get() → vehicle_lat_set() → recorder_record()
```

---

## 六、四条端到端数据流 (函数级详解)

### A路：CAN→MCU→SOC→AD应用 (外部CAN信号上行)

```
物理CAN总线
  │ CAN Controller (MCU) 收到CAN FD帧 (如 id=0x118, VDC_1)
  ▼
MCU侧 AUTOSAR COM栈:
  CanIf_RxIndication() → PduR_RxIndication() → Com_RxIndication()
    → Com_ReceiveSignal() 更新信号值 (AUTOSAR内部)
    │
    │ (X-DOM路径: x_dom_can_rt模块)
    ▼
x_dom_can_rt_mcu.c (dsar-sip):
  ├→ 从 Com 读取信号原始值
  ├→ 按CAN帧打包为 can_msg_base_with_status_t {channel, can_id, dlc, data[64], status}
  └→ dcms_mcu_topic_send(DCMS_TOPIC_XDOMCANRT_MCU_TX_RX_MSGS, &msg, sizeof(msg))
      │
      │  DCMS 跨核 IPC (UDP over Ethernet / 共享内存)
      ▼
SOC侧 COM进程:
  msg_route_mcu_tx_rx_msgs_topic_cbk(data, len)
    [msg_route_soc.cpp]
    ├→ 解析 can_msg_base_with_status_t
    └→ push → ThreadSafeQueue

  ─── 下一个50ms周期 ───

  msg_route_mcu_tx_rx_can_msgs_proc()
    ├→ 从队列取出 can_msg_base_with_status_t
    ├→ 构造 key = {channel=0, can_id=0x118}
    ├→ auto it = msg_route_mcu_tx_rx_msgs_map.find(key)
    │   → 找到 CanMsg_VDC_1_default_0x118
    └→ CanMsg_VDC_1_default_0x118.set_data(data, dlc, status)
        // 将64字节CAN FD原始数据缓存到CanMsg对象

  sig_route_send_mcu_tx_rx_sigs_proc()
    → sig_route_send_mcu_tx_rx_sigs()  [proxy生成代码]
      ├→ VDC_DecelerationReq_sighdl.signal_if_get(&phy)
      │   [SigHdl<float, uint16_t, float, float>]
      │   ├→ CanMsg::get_data() → 获取缓存CAN原始字节
      │   ├→ extract_bits(data_buf, start_bit=64, len=10, is_big_endian=false)
      │   │   → 从第64bit起取10bit → raw = 0x200
      │   └→ phy = (float)raw * 0.01f + (-10.21f) = -5.09 m/s²
      │
      ├→ 打包到 sig_route_read_tx_rx_sigs_dji_application_t 结构体
      └→ dcms_mcu_topic_send(DCMS_TOPIC_READ_SIGS_APP, &sigs, sizeof(sigs))
          │
          │  DCMS 进程间IPC
          ▼
AD进程:
  sig_route_mcu_rx_sigs_topic_cbk(data, len)
    [sig_route_soc.cpp, XDOMCANRT_DJI_COM_SERVICE_EN=0]
    → sig_route_update_mcu_tx_rx_sigs_from_com(data, len)  [proxy生成]
      ├→ 校验hash和长度
      ├→ memcpy(&local_copy, data, len)
      └→ APP可通过 SigIf_Get_VDC_DecelerationReq_AEB_VDC_1_Get(&decel) 读取
```

### B路：SOC AD应用→MCU→CAN总线 (控制命令下行)

```
AD进程:
  APP控制算法计算出目标减速度/转向角
    │
    ▼
  SigIf_Set_VDC_DecelerationReq_AEB_VDC_1_Set(&phy_value)
    [proxy/x_dom_can_rt_gen_uds/ 或 app_ad_core 生成的SigIf]
    ├→ VDC_DecelerationReq_sighdl.signal_if_set(phy_value)
    │   [SigHdl]
    │   ├→ raw = (phy - offset) / factor = (-5.09 - (-10.21)) / 0.01 = 512
    │   ├→ compose_bits(data_buf, start_bit=64, len=10, raw, is_big_endian=false)
    │   └→ CanMsg::set_data(data_buf) → 更新CAN帧缓存
    │
    └→ dcms_mcu_topic_send(DCMS_TOPIC_XDOMCANRT_WRITE_TO_MCU_MSGS, &msg, sizeof(msg))
        │
        │  DCMS (可能跨进程COM→再转跨核)
        ▼
MCU侧:
  x_dom_can_rt DCMS回调 (在DCMS消息泵 comm_task_handler 中)
    ├→ 解析 signal_id + raw_value
    ├→ 查找 msg_route_write_sig_tbl[signal_id]
    │   → 获取 CAN_ID, channel, start_bit, len, is_big_endian, factor, offset
    ├→ 构造CAN帧原始字节: compose_bits(buf, start_bit, len, raw, is_big_endian)
    └→ Com_SendSignal(signal_id, &raw_value)  → AUTOSAR COM栈
        │
        ▼
AUTOSAR COM栈: Com_SendSignal() → PduR_Transmit() → CanIf_Transmit()
  → CAN Controller → 物理CAN总线 → 外部ECU (如VDC执行器)
```

### C路：SOC→MCU (IMU传感器等数据下发)

```
SOC侧 AD进程:
  (感知/定位模块产生IMU数据)
    │
    ▼
  dcms_mcu_topic_send(DCMS_TOPIC_IMU_SENSOR, &imu_data, sizeof(Imu))
    │
    │  DCMS 跨核 IPC
    ▼
MCU侧:
  vehicle_sensor_imu_recv_cb(msg)  [vehicle_sensor.c]
    ├→ memcpy(&vehicle_sensor.sensor_get.imu, msg->data, sizeof(Imu))
    ├→ vehicle_sensor.sensor_get.imu_sensor_time = msg->sensor_time / 1e6
    │
    └→ (MCU内部其他模块通过 sensor_dev_get_imu() 等接口读取)
       如: vehicle_lat_get() 可能使用IMU的车辆加速度/角速度
```

### D路：MCU→SOC (雷达原始数据上报)

```
物理CAN总线 (雷达CAN FD)
  │ CAN Controller
  ▼
AUTOSAR COM栈接收:
  Com_ReceiveSignal(MRR_Obj_0_DistX) → 原始值
  Com_ReceiveSignal(MRR_Obj_0_DistY)
  ... (最多39目标 × 35+字段 = 1365+次信号读)

[mradar_obtain.c]
zyt_mradar_com():  ← 20ms周期
  ├→ MRR_Com_Get(radar_model, &model)
  ├→ MRR_Com_Get(object_sum, &sum)
  ├→ for i in 0..sum:
  │     MRR_Com_Get(objects[i].dist_x, &dx)
  │     MRR_Com_Get(objects[i].dist_y, &dy)
  │     MRR_Com_Get(objects[i].rel_vel_x, &vx)
  │     MRR_Com_Get(objects[i].rel_vel_y, &vy)
  │     MRR_Com_Get(objects[i].object_id, &id)
  │     MRR_Com_Get(objects[i].exist_prob, &prob)
  │     MRR_Com_Get(objects[i].class_prob, &cls)
  │     ... (acc_x, acc_x_std, acc_y, dist_x_std, dist_y_std, rel_vel_x_std, rel_vel_y_std)
  └→ acu_dev_write(ACU_MRADAR_DATA, &radar_data)  // 写入MCU内部共享区

[mradar_info_send.c]
zyt_mradar_comm_send():  ← 20ms周期
  ├→ zyt_radar_proxy_sensortime_update_period()
  │   └→ sync_signal_aligned_time_us_by_freq(&gs_time_align, get_time_us_32bits(), FREQ)
  │       → gs_time_align.aligned_time_stamp = 对齐后的时间戳
  │
  ├→ acu_dev_read(ACU_MRADAR_GET_RADAR_MODEL, &model)
  ├→ acu_dev_read(ACU_MRADAR_GET_OBJ_SUM, &sum)
  ├→ acu_dev_read(ACU_MRADAR_GET_TIME, &time)
  ├→ acu_dev_read(ACU_MRADAR_GET_FAULT_ST, &fault)
  ├→ acu_dev_read(ACU_MRADAR_GET_OBJ, objects[0..38])  // 读所有目标
  │
  └→ zyt_radar_raw_dcms_send(DCMS_TOPIC_MRR_SIGNAL,
  │       (uint8_t*)&g_stmradar_dev_get_info, sizeof(g_stmradar_dev_get_info))
  │   ├→ zyt_update_radar_proxy_sensortime(&sensortime)
  │   │   → sensortime = gs_time_align.aligned_time_stamp
  │   └→ dcms_mcu_topic_send_msg(DCMS_TOPIC_MRR_SIGNAL, &raw_msg)
  │
  │  DCMS 跨核 IPC
  ▼
SOC侧 COM进程:
  (DCMS回调接收, 解包后路由给AD域消费者)
  → AD进程: 感知/融合模块使用雷达目标数据
```

---

## 七、平台函数参考表

### 7.1 SOC侧平台函数 (dsar-plat-bf)

| 函数/类 | 文件 | 作用 | 调用者 |
|---------|------|------|--------|
| `x_dom_can_rt_init(name)` | x_dom_can_rt_soc.cpp | CAN路由初始化:注册DCMS主题+回调 | SOC进程入口 |
| `x_dom_can_rt_main_func()` | x_dom_can_rt_soc.cpp | CAN路由周期任务(50ms) | 工作链表调度 |
| `x_dom_someip_rt_init(name)` | x_dom_someip_rt.cpp | SOME/IP路由初始化 | SOC进程入口 |
| `x_dom_someip_rt_rx_handle()` | x_dom_someip_rt.cpp | SOME/IP数据接收处理 | Cdd_SomeIpTp.c回调 |
| `x_dom_someip_rt_set(key, val)` | x_dom_someip_rt.cpp | 配置注入(KV对) | Proxy初始化 |
| `msg_route_main_function()` | msg_route_soc.cpp | CAN帧解析周期任务 | x_dom_can_rt_main_func |
| `msg_route_mcu_tx_rx_can_msgs_proc()` | msg_route_soc.cpp | 处理MCU上传的CAN帧 | msg_route_main_function |
| `sig_route_main_function()` | sig_route_soc.cpp | CAN信号路由周期任务 | x_dom_can_rt_main_func |
| `sig_route_send_mcu_tx_rx_sigs_proc()` | sig_route_soc.cpp | COM域:读信号+发送 | sig_route_main_function |
| `svc_route_rx_handle()` | service_route.cpp | SOME/IP单服务接收处理 | x_dom_someip_rt_rx_handle |
| `svc_route_cycle_pub()` | service_route.cpp | SOME/IP周期打包转发(50ms) | 工作链表调度 |
| `svc_route_app_to_com_topic_cbk()` | service_route.cpp | APP→COM SOME/IP数据回调 | DCMS |
| `svc_route_com_to_app_topic_cbk()` | service_route.cpp | COM→APP SOME/IP数据回调 | DCMS |
| `dcms_mcu_topic_send_msg()` | dcms_mcu_api.cpp | DCMS发送消息 | 所有模块 |
| `dcms_mcu_topic_setup_callback()` | dcms_mcu_api.cpp | 注册DCMS回调 | 初始化代码 |
| `dcms_mcu_topic_setup_rcv_msg_callback()` | dcms_mcu_api.cpp | 注册DCMS接收回调(类型检查) | 初始化代码 |
| `dcms_mcu_topic_register()` | dcms_mcu_api.cpp | 批量注册DCMS话题 | 初始化代码 |
| `sig_route_init()` | sig_route_soc.cpp | 信号路由初始化 | x_dom_can_rt_init |
| `msg_route_init()` | msg_route_soc.cpp | CAN帧路由初始化 | x_dom_can_rt_init |

**CanMsg类** (`msg_route_soc.h`):

| 方法 | 作用 |
|------|------|
| `CanMsg(channel, can_id, dlc)` | 构造函数,绑定CAN帧 |
| `set_data(data, dlc, status)` | 缓存收到的CAN帧原始字节+状态 |
| `get_data(data_buf)` | 读取缓存的CAN帧原始字节 |
| `get_status()` | 获取CAN帧状态 (NO_ERROR/CRC_ERROR/BZ_ERROR/TIMEOUT/INITIAL) |

**SigHdl模板类** (`sig_handle_soc.hpp`):

| 方法 | 作用 |
|------|------|
| `SigHdl(can_msg_ref, start_bit, len, factor, offset, is_big_endian, child_id, value_type)` | 构造,绑定CanMsg引用+位域参数 |
| `signal_if_get(phy_t&)` | 从CanMsg缓存提取位域→物理值转换 |
| `signal_if_set(phy_t)` | 物理值→位域→写入CanMsg缓存 |

**SvcItemInfo类** (`x_dom_someip_rt_com.h`):

| 方法 | 作用 |
|------|------|
| `set_origin_data(id, data, len)` | 缓存收到的SOME/IP原始数据 |
| `get_usr_data_ext(offset, pData, Deserialize)` | 反序列化→读取应用层数据 |
| `set_usr_data_ext(offset, pData, Serialize)` | 序列化→写入应用层数据 |
| `fwd_svc_via_dcms(topic)` | 通过DCMS转发服务数据 |
| `fwd_svc_via_someip()` | 通过SOME/IP发送到外部ECU |

### 7.2 MCU侧平台函数 (dsar-sip + AUTOSAR BSW)

| 函数/宏 | 文件 | 作用 |
|---------|------|------|
| `Com_ReceiveSignal(sig_id, &value)` | AUTOSAR BSW | 从CAN总线读取信号物理值 |
| `Com_SendSignal(sig_id, &value)` | AUTOSAR BSW | 发送信号到CAN总线 |
| `dcms_mcu_topic_send_msg()` | DCMS MCU API | MCU侧DCMS发送 |
| `dcms_mcu_topic_setup_callback()` | DCMS MCU API | MCU侧DCMS回调注册 |
| `dcms_mcu_topic_setup_rcv_msg_callback()` | DCMS MCU API | MCU侧DCMS接收回调注册 |
| `sync_signal_aligned_time_us_by_freq()` | sync_common.h | 时间戳对齐到指定频率 |
| `acu_dev_read(dev, cmd, buf, len)` | acu_dev.h | 从MCU内部设备抽象层读取数据 |
| `acu_dev_write(dev, cmd, buf, len)` | acu_dev.h | 写入MCU内部设备抽象层 |
| `acu_dev_get(ACU_DEVICE_ID(type,idx))` | acu_dev.h | 获取设备句柄 |
| `add_period_work(node, handler, arg)` | appl_worklist.c | 注册周期工作任务 |
| `SigIf_Get_xxx()` | x_dom_can_rt_sigif.c (生成) | MCU侧信号物理值读取 |
| `SigIf_Set_xxx()` | x_dom_can_rt_sigif.c (生成) | MCU侧信号物理值写入 |
| `MRR_Com_Get(field, &value)` | mradar_obtain.c (宏) | 雷达信号读取封装(raw→phy) |

### 7.3 SDK ↔ Proxy 设计模式

| 模式 | 描述 | 实例 |
|------|------|------|
| **extern符号** | Proxy定义函数，SDK中extern声明，通过`--whole-archive`链接 | `extern void sig_route_send_mcu_tx_rx_sigs()` |
| **指针注入** | Proxy通过`x_dom_someip_rt_set(key, ptr)`将数据指针注入SDK | `g_x_dom_someip_config.svc_route_recv_svc_info_map_ptr = &map` |
| **回调注册** | Proxy将函数指针传给SDK构造的SvcItemInfo | `SvcItemInfo(id, name, timeout, SigIf_OnRecvData, ...)` |
| **模板引用绑定** | Proxy实例化SigHdl模板并绑定CanMsg引用 | `SigHdl<float,uint16_t,float,float> sighdl(canmsg, start_bit, len, ...)` |
| **CAN帧查找表** | Proxy定义全局map，SDK运行时查找 | `const map<CanMsgIdPair, CanMsg*> msg_route_mcu_tx_rx_msgs_map` |

---

## 八、DCMS Topic命名空间

### 8.1 Topic ID 段划分

```
0x00000 - 0x0FFFF : APP域话题
0x10000 - 0x2FFFF : COM域话题
0x30000 - 0x30FFF : UDS域话题
0x33000 - 0x33200 : X_DOM_CAN_RT  话题
0x33300 - 0x33500 : X_DOM_SOMEIP_RT 话题
```

### 8.2 X-DOM CAN RT 话题

| Topic名称 | Direction | 数据格式 | 发布者 | 订阅者 |
|-----------|-----------|---------|--------|--------|
| `/x_dom_can_rt/mcu_tx_rx_msgs` | MCU→SOC | `can_msg_base_with_status_t` | MCU x_dom_can_rt | SOC COM msg_route |
| `/x_dom_can_rt/write_to_mcu_msgs` | SOC→MCU | 信号ID+值数组 | SOC COM sig_route | MCU x_dom_can_rt |
| `/x_dom_can_rt/write_to_vip_msgs` | SOC→MCU | CAN帧原始字节 | SOC COM | MCU内部VIP模块 |
| `/x_dom_can_rt/read_sigs/com_app` | COM内部 | 信号结构体 | COM sig_route(写) | COM自身(用于内部转发) |
| `/x_dom_can_rt/read_sigs/ad_app` | COM→AD | 信号结构体 | COM sig_route | AD sig_route回调 |
| `/x_dom_can_rt/read_sigs/uds_app` | COM→UDS | 信号结构体 | COM sig_route | UDS sig_route回调 |

### 8.3 X-DOM SOME/IP RT 话题

| Topic名称 | Direction | 数据格式 | 发布者 | 订阅者 |
|-----------|-----------|---------|--------|--------|
| `/x_dom_someip_rt/com_to_app/ad_app` | COM→AD | SvcDataPackHdl打包 | COM service_route | AD |
| `/x_dom_someip_rt/com_to_app/uds_app` | COM→UDS | SvcDataPackHdl打包 | COM service_route | UDS |
| `/x_dom_someip_rt/app_to_com` | AD/UDS→COM | SvcDataPackHdl打包 | AD/UDS | COM service_route |

### 8.4 传感器/执行器直接话题

| Topic名称 | Direction | 说明 |
|-----------|-----------|------|
| `DCMS_TOPIC_IMU_SENSOR` | SOC→MCU | SOC IMU数据下发到MCU |
| `DCMS_TOPIC_MRR_SIGNAL` | MCU→SOC | MCU中程雷达目标数据上报 |
| `DCMS_TOPIC_FCRL_SIGNAL` | MCU→SOC | 前角雷达左 |
| `DCMS_TOPIC_FCRR_SIGNAL` | MCU→SOC | 前角雷达右 |
| `DCMS_TOPIC_RCRL_SIGNAL` | MCU→SOC | 后角雷达左 |
| `DCMS_TOPIC_RCRR_SIGNAL` | MCU→SOC | 后角雷达右 |
| `DCMS_TOPIC_SYS_MODE_TOTAL` | MCU→SOC | 系统模式信息 |
| `DCMS_TOPIC_SOMEIP_TO_CAN_MRR_RX` | SOC→MCU | SOMEIP雷达配置→CAN |

---

## 九、产品变体对比 (fawhq_p301 vs fawhq_e001_10)

### 9.1 核心差异

| 维度 | fawhq_p301 | fawhq_e001_10 |
|------|-----------|---------------|
| SOC Proxy架构 | SOC集中式: `proxy_soc.cpp` | 标准化分组: SensorGroup/等分组 |
| MCU Proxy架构 | `proxy.c` 调度ActuatorProxy/BCMProxy等 | `ChassisProxy.c` 空实现,信号由生成代码处理 |
| X-DOM CAN路由 | 通过SOC proxy集中分发 | COM域→DCMS→AD域 标准comif路由 |
| 雷达路由 | SOC侧集中处理 | MCU侧mradar_obtain + mradar_info_send via acu_dev |
| IMU路由 | SOC直发 | vehicle_sensor DCMS回调 + 备用通路 |
| CAN_UDS_V4 | 启用 | OFF (UDS不通过comif收CAN信号) |
| Adasis v3 | 未关注 | ComIPduCallout回调接收ADASIS v3地图数据 |
| Shell命令 | 较少 | 丰富 (zyt_mradar debug shell等) |

### 9.2 SOC Proxy差异详解

**p301** (SOC集中式, 在COM.so中):
```
COM域 proxy_soc.cpp:
  oem_proxy_init() → add_period_work("proxy", proxy_main_function, 100Hz)
    proxy_main_function():
      ├→ radar_proxy_run()     @20Hz
      ├→ uss_proxy_run()       @20Hz
      └→ 其他SOC代理           @20Hz
```

**e001_10** (标准化分组):
```
COM域 proxy/ 目录:
  proxy_soc.cpp       → 入口 + 初始化
  SensorGroup/        → 传感器代理分组
  x_dom_can_rt_gen_com/  → CAN信号路由生成代码
  x_dom_someip_rt_gen_com/ → SOME/IP路由生成代码
  comif_cfg/          → comif配置
```

### 9.3 MCU Proxy差异详解

**p301** (显式调度):
```c
// proxy.c → proxy_main_function() @100Hz:
if (tick % 5 == 0) {  // 20Hz
    ActuatorProxy_Main();
    BCMProxy_Main();
    ChassisProxy_Main();
    HMIProxy_Main();
    PartnerProxy_Main();
    mradar_dev_send();
}
```

**e001_10** (工作链表+生成代码):
```c
// ChassisProxy.c → ChassisProxy_Main() 函数体为空
// 信号处理由 x_dom_can_rt_config 生成的 SigIf 自动完成
// 通过工作链表注册周期任务
```

---

## 十、快速参考：如何使用一个CAN信号

在DSAR项目中添加一个新的CAN信号需要在以下位置操作：

### Step 1: JSON配置
在 `x_dom_can_rt_fawhq_e001_10.json` 中，确定信号属于哪个路由组 (dji_uds_service / dji_application / dji_com_service / mcu)，添加到对应的 `read_mcu_rx_sigs` / `read_mcu_tx_sigs` / `write_to_mcu_sigs` 等列表中。

### Step 2: 重新生成代码
运行SIP代码生成工具，自动生成：
- `proxy/x_dom_can_rt_gen_com/` — SOC COM域: SigHdl声明 + CanMsg + 转发函数
- `proxy/x_dom_can_rt_gen_uds/` — SOC UDS域: 信号结构体成员
- `x_dom_can_rt_config/` — MCU: 信号读写表 + CAN帧缓存

### Step 3: APP侧使用 (SOC AD域)

读取外部CAN信号:
```cpp
// 声明 (生成代码)
extern SigHdl<float, uint16_t, float, float> MyNewSignal_sighdl;

// 读取
float phy_value;
MyNewSignal_sighdl.signal_if_get(phy_value);
// 或: SigIf_Get_MyNewSignal_XXX_Get(&phy_value);
```

写入控制命令到CAN:
```cpp
SigIf_Set_ADV_SteeringWheelAngleReq_XXX_Set(&angle);
// 内部: SigHdl::signal_if_set() → CanMsg::set_data() → DCMS → MCU → Com_SendSignal()
```

### Step 4: MCU侧使用

读取:
```c
float phy_value;
SigIf_Get_IBC_BrakePedalStatus_IBC_1_Get(&phy_value);
```

写入:
```c
SigIf_Set_VDC_DecelerationReq_AEB_VDC_1_Set(&decel);
// 内部: 更新 mcu_write_to_vip_msg_VDC_1_default_0x0x118_buf[]
//       如果配置了 write_to_mcu: Com_SendSignal(signal_id, &raw)
```

---

## 十一、关键文件索引

```
=== JSON配置 ===
(代码生成工具配置目录)/x_dom_can_rt_fawhq_e001_10.json       CAN信号跨域路由JSON
(代码生成工具配置目录)/x_dom_someip_rt_fawhq_e001_10.json    SOME/IP跨域路由JSON (84014行)

=== SOC侧生成代码 (COM域) ===
src/dsar_app/product/faw/fawhq_e001_10/proxy/x_dom_can_rt_gen_com/
  x_dom_can_rt_cfg.h                  功能开关 + SigHdl声明 + CanMsg声明
  x_dom_can_rt_cfg.cpp                路由表 + CanMsg + SigHdl实例化 + 转发函数
  x_dom_can_rt_sigif.h/cpp            SigIf_Get/Set

=== SOC侧生成代码 (UDS域) ===
src/dsar_app/product/faw/fawhq_e001_10/proxy/x_dom_can_rt_gen_uds/
  (e001_10中几乎为空, CAN_UDS_V4=OFF)

=== SOC侧生成代码 (COM域 SOME/IP) ===
src/dsar_app/product/faw/fawhq_e001_10/proxy/x_dom_someip_rt_gen_com/
  x_dom_someip_rt_cfg.h/cpp           SvcItemInfo + 路由表 + NodeRecvInfo (120+服务)
  x_dom_someip_rt_sigif_get.cpp       SigIf_OnRecvData + SigIf_xxx_Get
  x_dom_someip_rt_sigif_set.cpp       SigIf_xxx_Set
  x_dom_someip_rt_protocol_*.cpp      Serialize/Deserialize

=== SOC侧生成代码 (UDS域 SOME/IP) ===
src/dsar_app/product/faw/fawhq_e001_10/proxy/x_dom_someip_rt_gen_uds/
  (UDS OTA服务: 3个recv + 4个send)

=== MCU侧生成代码 ===
src/dsar_fw/product/faw/fawhq_e001_10/x_dom_can_rt_config/
  x_dom_can_rt_cfg.h                  功能开关 + CAN帧缓存buf + 信号写表 + 过滤表
  x_dom_can_rt_cfg.c                  信号表 + CAN帧表实现
  x_dom_can_rt_sigif.h/c              MCU侧 SigIf_Get/Set
  x_dom_can_rt_dbg.h/c                调试shell命令

=== 平台仓运行时引擎 (SOC侧) ===
dsar-hq-plat/dsar-plat-bf/dsar_app/cdd/x_dom_can_rt/
  msg_route_soc.cpp/h                 CAN帧解析引擎 + CanMsg类 + ThreadSafeQueue
  sig_route_soc.cpp/h                 CAN信号路由引擎
  x_dom_can_rt_soc.cpp/h             初始化入口 + DCMS注册
  sig_handle_soc.hpp                  SigHdl模板 (位提取+物理值转换)
  x_dom_can_rt_com.h                 can_msg_t / can_sig_base_info_mcu_t 等数据结构
  x_dom_can_rt_internal.h            DCMS话题名称宏 + 任务周期定义

dsar-hq-plat/dsar-plat-bf/dsar_app/cdd/x_dom_someip_rt/
  service_route.cpp                   SOME/IP路由引擎 (收发/打包/解包/转发)
  x_dom_someip_rt.cpp                 SOME/IP初始化入口
  x_dom_someip_rt_com.h              SvcItemInfo / SvcDataPackHdl / NodeRecvInfo
  x_dom_someip_rt_com_cfg.h          全局配置结构体 g_x_dom_someip_config

dsar-hq-plat/dsar-plat-bf/dsar_app/cdd/dcms_adapt/
  dcms_mcu_api.cpp/h                  DCMS API封装 (send/register/callback)

=== 平台仓运行时引擎 (MCU侧) ===
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/x_dom_can_rt/
  x_dom_can_rt_mcu.c                  MCU CAN路由运行时
  msg_route_mcu.c                     MCU CAN帧路由

=== MCU侧通信源码 ===
src/dsar_fw/proxy_comm/proxy_sensor/
  mradar_obtain.c                     雷达数据采集 (AUTOSAR COM → acu_dev)
  mradar_info_send.c                  雷达数据发送 (acu_dev → DCMS → SOC)

src/dsar_fw/product/faw/fawhq_e001_10/xwire/
  vehicle_sensor.c                    IMU传感器接收 (DCMS回调) + ADASIS v3处理

src/dsar_fw/product/faw/fawhq_p301/xwire/
  vehicle_lat.c                       横向控制 (AUTOSAR COM直读直写)

=== AUTOSAR协议栈桥接 ===
src/dsar_app/product/faw/fawhq_e001_10/autosar_adapter/microsar_config_com/
  Cdd_SomeIpTp.c                      SOME/IP 846个回调分发 (114个接comif)

src/dsar_fw/product/faw/fawhq_p301/com_if/
  someip_if.c                         SOC→MCU SOMEIP转CAN (MRR配置等)
```

---

## 十二、与参考文档的关系

本文档作为 `01_DSAR_DOMAIN_ARCHITECTURE.md` 和 `03_DSAR_COMMUNICATION_ANALYSIS.md` 的补充和深化：

- **01文档** 定义了域架构、进程模型、.so加载关系、FW任务模型、平台仓子项目结构
- **03文档** 详述了SOC侧comif vs danvince两条路径、SDK↔Proxy设计模式、SOME/IP服务路由
- **本文档** 聚焦于 JSON→代码生成→端到端数据流，特别补全了：
  - JSON配置文件的具体结构和六种信号操作类型
  - 代码生成的四个输出目录和文件级别内容
  - MCU侧基于AUTOSAR COM + acu_dev + DCMS的完整通信链
  - 雷达原始数据(MCU→SOC)和IMU传感器(SOC→MCU)的端到端流
  - 所有DCMS话题的命名空间映射
  - fawhq_p301与fawhq_e001_10两个产品变体的proxy架构对比
  - 平台函数参考表（SOC侧和MCU侧）
