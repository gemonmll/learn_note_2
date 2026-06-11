# DSAR-HQ 标定功能完整架构设计与调用链分析

> 覆盖：芯片拓扑与文件分布 → SOC↔MCU通信架构(9个DCMS Topic) → DMM模式切换4层调用链 → UDS触发静态标定完整流程 → HMI触屏触发两条路径 → 前置条件检测 → 错误码三级翻译 → Delta数据大端转换 → DAGS感知原子能力协商 → Shell调试命令 → 动静态标定对比
> 基于 fawhq E001/E202/E007/E009 车型，芯片平台 SA8650

---

## 一、总体架构

### 1.1 芯片拓扑与标定功能分布

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                           SA8650 芯片                                         │
│                                                                              │
│  ┌──────────────────────────────────────┐  ┌────────────────────────────────┐│
│  │ SOC 侧 (应用处理器)                   │  │ MCU 侧 (R52 安全岛)             ││
│  │ OS: QNX 7.1                          │  │ OS: FreeRTOS / SafeRTOS        ││
│  │ 进程: dji_ad_app / dji_application   │  │                                ││
│  │                                      │  │ ┌────────────────────────────┐ ││
│  │ ┌──────────────────────────────────┐ │  │ │ DiagCalib_Main.c           │ ││
│  │ │ DiagCalib (车型适配, DSAR-HQ)     │ │  │ │ 前置条件检测(20+车辆信号)    │ ││
│  │ │  DiagCalib_Common.c  Topic初始化  │ │  │ │ DTC触发存储                 │ ││
│  │ │  DiagCalib_Static.c   RID/DID     │ │  │ │ DiagCalib_Common.c DMM客户端│ ││
│  │ │  DiagCalib_Dynamic.c  RID/DID     │ │  │ └────────────────────────────┘ ││
│  │ │  HmiStatic/HmiDynamic HMI状态机   │ │  │                                ││
│  │ │  DmmCalibMode.c       DMM客户端   │ │  │ SOC平台DMM服务端的DCMS客户端:   ││
│  │ └──────────────────────────────────┘ │  │ ┌────────────────────────────┐ ││
│  │                                      │  │ │ DmmCalibMode.c (FW侧)      │ ││
│  │ ┌──────────────────────────────────┐ │  │ │ 也可发送DMM标定模式命令      │ ││
│  │ │ DMM平台层 (DSAR-HQ-PLAT)          │ │  │ └────────────────────────────┘ ││
│  │ │  asw_mcu.c  dmm_calib_service_cb()│ │  │                                ││
│  │ │  dmm_main_common.c Dmm_SetEvent() │ │  │ 感知引擎 (AD域, 另一进程/核)    ││
│  │ │  ServiceAck.c Dmm_SetPerMode()    │ │  │ ┌────────────────────────────┐ ││
│  │ │           DAGS感知原子能力协商     │ │  │ │ 标定计算 + 结果上报          │ ││
│  │ └──────────────────────────────────┘ │  │ │ DCMS_SUB_EOLCALIB_RESULT   │ ││
│  │                                      │  │ │ DCMS_CALIB_DATA_ACK_RTOS   │ ││
│  │ AUTOSAR DCM (UDS协议栈)              │  │ └────────────────────────────┘ ││
│  │ DoIP (TCP 13400)                     │  │                                ││
│  └──────────────────────────────────────┘  └────────────────────────────────┘│
│                                                                              │
│  跨核通信: DCMS (IPC pub/sub + service request-response, 共享内存)             │
│  关键: DMM标定模式切换为SOC内部调用(SOC DiagCalib → SOC平台DMM服务),            │
│        前置条件检测为SOC→MCU跨核Service调用                                    │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 文件分布总览

```
适配仓(dsar-hq) 车型目录:

  SOC侧 (UDS诊断进程):
    src/dsar_app/product/faw/oem_feature/DiagCalib/
      ├── DiagCalib_CommonType.h          ★ 所有标定数据结构定义 (370行)
      ├── DiagCalib_Common.h              ★ 公共接口与全局变量声明
      ├── DiagCalib_Common.c              ★ DCMS Topic初始化与结果路由 (299行)
      ├── DiagCalib_Static.c              ★ UDS RID/DID EOL静态标定 (1396行)
      ├── DiagCalib_Static.h              ★ 静态标定接口声明
      ├── DiagCalib_HmiStatic.c           ★ HMI触屏静态标定状态机
      ├── DiagCalib_HmiStatic.h           ★ HMI静态标定接口
      ├── DiagCalib_HmiDynamic.c          ★ HMI触屏动态标定状态机 (627行)
      ├── DiagCalib_HmiDynamic.h          ★ HMI动态标定接口
      ├── DiagCalib_HmiCalibType.h        ★ HMI触屏坐标区域定义 (149行)
      ├── DmmCalibMode/
      │   ├── DmmCalibMode.c              ★ SOC侧DMM模式设置/查询 (102行)
      │   ├── DmmCalibMode.h              ★ DMM模式接口声明
      │   ├── DmmCalibMode_Type.h         ★ DMM模式类型定义 (43行)
      │   └── DmmCalibMode_If.h           ★ 外部引用接口

  MCU侧 (FreeRTOS安全岛):
    src/dsar_fw/product/faw/oem_feature/diagnosis/DiagCalib/
      ├── DiagCalib_Main.c                ★ MCU入口: 前置条件+Topic初始化 (730行)
      ├── DiagCalib_Main.h                ★ MCU侧接口声明
      └── DiagCalib_Common.c              ★ MCU侧DMM模式命令 (106行)

平台仓(dsar-hq-plat) 运行时引擎:

  SOC侧平台:
    dsar-plat-ad/src/dsar_app/app_core/app_ad_core/
      ├── dmm_common/dmm_main_common.c    ★ 平台层Dmm_SetEvent/Dmm_GetCalibResult
      ├── mode_manager/ServiceAck.c       ★ Dmm_SetPerMode真实实现+DAGS协商
      └── app_interface/asw_mcu_service/
          └── asw_mcu.c                   ★ dmm_calib_service_cb (DCMS服务端, 998-1053行)

  MCU侧固件:
    dsar_fw/app_core/diag/DmmCalibMode/
      └── DmmCalibMode.c                  ★ FW侧DMM标定模式shell命令 (204行)
    dsar_fw/app_core/dmm/asw_app/
      └── dmm_if_common.c                 ★ FW侧Dmm_SetEvent/Dmm_GetCalibResult
```

### 1.3 两种触发路径总览

```
触发源:
  ┌─ UDS DoIP (诊断仪) ──→ AUTOSAR DCM ──→ RID/DID Callout ──→ DiagCalib_Static.c
  │                                                                │
  └─ HMI 车机触屏 ──→ DCMS Topic ──→ DiagCalib_HmiStatic.c       │
                     │                DiagCalib_HmiDynamic.c       │
                     │                                              │
                     ▼                                              ▼
               DmmCalibMode.c  ──→ DCMS_CLIENT_DMM_CALIB_MODE_CMD ──→ SOC平台 asw_mcu.c
                                                                        │
                                                                        ▼
                                                                  Dmm_SetEvent()
                                                                        │
                                                                        ▼
                                                                  ServiceAck.c
                                                                  Dmm_SetPerMode() → DAGS协商
```

---

## 二、DCMS 通信架构

### 2.1 9个DCMS Topic完整清单

标定功能共使用9个DCMS Topic，涵盖SOC↔MCU跨核通信和SOC↔感知引擎进程间通信：

| # | Topic 宏 | 方向 | 通信模式 | 周期 | 用途 |
|---|---------|------|---------|------|------|
| 1 | `DCMS_TOPIC_CALIB_PRECONDITION_CHECK` | SOC→MCU | **Service** (req-resp) | 按需 | 前置条件检测请求/响应 |
| 2 | `DCMS_CLIENT_DMM_CALIB_MODE_CMD` | SOC→SOC平台 / MCU→SOC平台 | **Service** (req-resp) | 按需 | DMM标定模式设置/查询 |
| 3 | `DCMS_CLIENT_START_EOLCALIB_RTOS` | SOC→感知 | Topic (单向) | 按需 | 启动/停止标定请求 |
| 4 | `DCMS_SUB_EOLCALIB_RESULT_RTOS` | 感知→SOC+MCU | Topic (订阅) | 1s周期 | 标定结果状态上报 |
| 5 | `DCMS_CALIB_DATA_ACK_RTOS` | 感知→SOC | Topic (订阅) | 按需 | 标定结果Delta数据返回 |
| 6 | `DCMS_CLINET_GETCALIB_PARAM_RTOS` | SOC→感知 | Topic (单向) | 按需 | 请求标定Delta参数 |
| 7 | `DCMS_TOPIC_DIAG_AVM_GESTURE` | MCU→SOC | Topic (订阅) | 实时 | HMI触屏手势坐标上报 |
| 8 | `DCMS_TOPIC_AVM_STATE_INFO` | MCU→SOC | Topic (订阅) | 实时 | AVM全景界面状态 |
| 9 | `DCMS_TOPIC_HMI_CALIB_REQ` | SOC→MCU | Topic (单向) | 按需 | HMI标定结果/停止请求反馈 |

### 2.2 DCMS Topic 与 Service 的区分机制 — 编译期静态配置

DCMS 框架不通过运行时判断来区分 Topic 和 Service，而是通过**编译期静态配置表** `DCMS_MCU_UDS_TOPIC_CONFIG` [dcms_mcu_config_uds.h](dcms_mcu_config_uds.h#L82) 中的四个互斥标志位来确定每个端点的角色：

```c
#define DCMS_MCU_UDS_TOPIC_CONFIG \
{   /*TOPIC_ID                           TOPIC_NAME               PUB_EN SUB_EN CLIENT SERVER */ \
    /* Topic 订阅者 — 只收不发 */
    {0, DCMS_SUB_EOLCALIB_RESULT_RTOS,     "/sys/eol_result_rtos",         0u, 1u, 0u, 0u, ...}, \
    {0, DCMS_CALIB_DATA_ACK_RTOS,          "/sys/send_stedelta",           0u, 1u, 0u, 0u, ...}, \
    {0, DCMS_TOPIC_DIAG_AVM_GESTURE,       "/sys/diag_avm_gesture/v1",    0u, 1u, 0u, 0u, ...}, \
    /* Topic 发布者 — 只发不收 */
    {0, DCMS_TOPIC_HMI_CALIB_REQ,          "/sys/hmi_calib/request/v1",   1u, 0u, 0u, 0u, ...}, \
    /* Service Client — 发送请求+接收应答 */
    {0, DCMS_CLIENT_DMM_CALIB_MODE_CMD,    "/sys/dmm_calib/v1",           0u, 0u, 1u, 0u, ...}, \
    {0, DCMS_TOPIC_CALIB_PRECONDITION_CHECK,"/sys/calib_precondition_check",0u,0u,1u, 0u, ...},\
    /* Service Server — 接收请求+发送应答（在平台侧配置文件 dmm_dcms_configs.h 中） */
    /* {0, DCMS_TOPIC_ASW_CALIB_SERVER,     "/sys/dmm_calib/v1",           0u, 0u, 0u, 1u, ...}, */ \
}
```

**四种角色的 API 差异**：

| 角色 | 配置标志 | 发送API | 接收API | 应答机制 |
|------|---------|---------|---------|---------|
| **PUB** (发布者) | `PUB_EN=1` | `dcms_mcu_topic_send()` | — | 无应答，单向推送 |
| **SUB** (订阅者) | `SUB_EN=1` | — | `dcms_mcu_topic_setup_callback()` | 无应答，被动接收 |
| **CLIENT** (客户端) | `CLIENT=1` | `dcms_mcu_topic_send()` | `dcms_mcu_service_client_setup_resp_callback()` | 期待对方通过 `service_server_set_ackdata()` 回复 |
| **SERVER** (服务端) | `SERVER=1` | `dcms_mcu_service_server_set_ackdata()` | `dcms_mcu_topic_setup_callback()` | 收到请求后调用 ackdata 回复 |

**核心设计**: Server 侧不需要运行时判断 Topic vs Service。DCMS 框架启动时读取配置表，根据标志位确定每个端点的行为模式。CLIENT 发送消息后，框架自动将 SERVER 的 `ackdata` 路由回 CLIENT 的 `resp_callback`。同一份配置表分别编译进 SOC 和 MCU 两侧固件，两侧的同一个 topic path 可以配置为互补的角色（一端 CLIENT，另一端 SERVER）。

对于标定前置条件检查，SOC 侧 `DCMS_TOPIC_CALIB_PRECONDITION_CHECK` 配置为 `CLIENT=1`，MCU 侧同一 topic path 配置为 `SUB_EN=1`，MCU 在回调中完成车辆信号检查后调用 `dcms_mcu_service_server_set_ackdata()` 将结果返回给 SOC。

### 2.3 Topic通信模式详解

#### Service模式 (Topic 1, 2)

Topic 1和Topic 2使用DCMS的**Service请求-响应模式**，不同于普通pub/sub：

```
SOC侧 (Client)                           MCU侧 (Server)
─────────────                           ─────────────
dcms_mcu_topic_send()                   dcms_mcu_topic_setup_callback()
  │                                       │
  ▼                                       ▼
DCMS路由层 ←────── IPC共享内存 ──────→ 回调函数处理
  │                                       │
  ▼                                       ▼
service_client_setup_resp_callback()    dcms_mcu_service_server_set_ackdata()
  接收响应                                发送响应
```

关键区别：
- **Service Client** (SOC): 调用 `dcms_mcu_service_client_setup_resp_callback()` 注册响应回调
- **Service Server** (MCU): 调用 `dcms_mcu_service_server_set_ackdata()` 发送响应数据
- **普通Topic** (Topic 3-9): 调用 `dcms_mcu_topic_setup_callback()` 注册回调接收数据

#### Topic 1 (前置条件) Service调用链

```
SOC发送: dcms_mcu_topic_send(DCMS_TOPIC_CALIB_PRECONDITION_CHECK, {REQ_STATIC/DYNAMIC})
  → MCU回调: DiagCalib_PreconditonCheckCallbackAck()
    → 读取20+车辆信号 (DiagProxy_Get*)
    → 位掩码组装前置条件结果
    → dcms_mcu_service_server_set_ackdata(DCMS_TOPIC_CALIB_PRECONDITION_CHECK, &respResult, 1)
      → SOC回调: DiagCalib_GetPreconditionDataCallback()
        → 设置 gPreconditionCalibStatus.getPreconditionStatus
        → 设置 gPreconditionCalibStatus.getFlag = true
```

#### Topic 2 (DMM模式) Service调用链 — 核心路径

**关键纠正**: `dmm_calib_service_cb()` 位于 SOC 侧平台代码 [asw_mcu.c](asw_mcu.c#L998)，而非 MCU 侧。DMM 标定模式切换为 **SOC 内部调用**（SOC 车型适配代码 → SOC 平台 DMM 服务），MCU 固件也可作为另一个 CLIENT 向同一服务发送命令。

```
SOC车型适配 (DiagCalib/DmmCalibMode.c):
  dcms_mcu_topic_send(DCMS_CLIENT_DMM_CALIB_MODE_CMD, {cmd, mode})
    │  Topic path: "/sys/dmm_calib/v1" (CLIENT=1)
    ▼
DCMS 路由层 → SOC平台 (同一进程或进程间):
    │  Topic path: "/sys/dmm_calib/v1" (SERVER=1, 注册为 DCMS_TOPIC_ASW_CALIB_SERVER)
    ▼
SOC平台: dmm_calib_service_cb() [asw_mcu.c:998]
  ├─ cmd==SET: Dmm_SetEvent(event) → Dmm_SetPerMode(DAGS_FACTORY/DYNAMIC_CALIB) → DAGS协商
  ├─ cmd==QUERY: Dmm_GetCalibResult()
  └─ dcms_mcu_service_server_set_ackdata(DCMS_TOPIC_ASW_CALIB_SERVER, {cmd, status})
       │
       ▼
SOC车型适配回调: DiagCalib_DmmModeStatusCallback() [DmmCalibMode.c:71]
  → gDmmCalibModeStatus = data[1]

// MCU侧同样可作为独立CLIENT发送DMM命令 (dsar_fw DmmCalibMode.c):
  dcms_mcu_topic_send(DCMS_CLIENT_DMM_CALIB_MODE_CMD, ...) → 同一SOC平台SERVER
```

### 2.4 SOC侧Topic初始化

[DiagCalib_Common.c:276](DiagCalib_Common.c#L276) — `DiagCalib_CalibrationTopicInit()`:

```c
void DiagCalib_CalibrationTopicInit(void)
{
    // Topic 1: 前置条件检测 (Service Client)
    dcms_mcu_service_client_setup_resp_callback(
        DCMS_TOPIC_CALIB_PRECONDITION_CHECK,
        DiagCalib_GetPreconditionDataCallback);

    // Topic 4: 标定结果状态 (1s周期订阅)
    dcms_mcu_topic_setup_callback(
        DCMS_SUB_EOLCALIB_RESULT_RTOS,
        DiagCalib_CalibResultStatusCallback, NULL, 0u);

    // Topic 5: 标定Delta数据 (按需订阅)
    dcms_mcu_topic_setup_callback(
        DCMS_CALIB_DATA_ACK_RTOS,
        DiagCalib_CalibResultDataCallback, NULL, 0u);

    // Topic 2: DMM模式切换 (Service Client)
    dcms_mcu_service_client_setup_resp_callback(
        DCMS_CLIENT_DMM_CALIB_MODE_CMD,
        DiagCalib_DmmModeStatusCallback);

    // Topic 7: HMI手势坐标
    dcms_mcu_topic_setup_callback(
        DCMS_TOPIC_DIAG_AVM_GESTURE,
        DiagCalib_GetHmiCalibRequestCallBack, NULL, 0u);

    // Topic 8: AVM界面状态
    dcms_mcu_topic_setup_callback(
        DCMS_TOPIC_AVM_STATE_INFO,
        DiagCalib_GetAvmInfoCallBack, NULL, 0u);
}
```

### 2.5 MCU侧Topic初始化

[DiagCalib_Main.c:281](DiagCalib_Main.c#L281):

```c
void DiagCalib_CalibrationTopicInit(void)
{
    // Topic 1: 前置条件检测 (Service Server — 接收SOC请求，处理后回复)
    dcms_mcu_topic_setup_callback(
        DCMS_TOPIC_CALIB_PRECONDITION_CHECK,
        DiagCalib_PreconditonCheckCallbackAck, NULL, 0u);

    // Topic 4: 标定结果状态 (感知引擎推送到MCU侧，用于MCU本地状态跟踪)
    dcms_mcu_topic_setup_callback(
        DCMS_SUB_EOLCALIB_RESULT_RTOS,
        DiagCalib_CalibResultStatusCallback, NULL, 0u);

    // DiagProxy初始化 (车辆信号读取接口)
    DiagProxy_DcmsInit();

    // Topic 2: DMM模式切换 (Service Client — MCU可独立向SOC平台DMM服务发送标定模式命令)
    dcms_mcu_service_client_setup_resp_callback(
        DCMS_CLIENT_DMM_CALIB_MODE_CMD,
        DiagCalib_DmmModeStatusCallback);
}
```

**注**: MCU侧的 `DCMS_CLIENT_DMM_CALIB_MODE_CMD` 注册为 **Service Client**，意味着 MCU 固件可以独立向 SOC 平台 DMM 服务发送标定模式设置/查询命令。SOC 车型适配层的 DiagCalib 也以相同方式注册为另一个 CLIENT。两者共享同一个 SERVER — SOC 平台 `dmm_calib_service_cb()`。

### 2.6 标定触发与结果上报数据流

```
                    SOC (UDS域)                          感知引擎 (AD域)
                    ──────────                          ──────────────
                    
DiagCalib_CalibStartTopicSend()
  │
  ├─ dcms_mcu_get_curr_sensor_usec(&sensorTime)
  ├─ 填充 CalibStartRequestType {sensortime, calibtype, detail_type}
  └─ dcms_mcu_topic_send(DCMS_CLIENT_START_EOLCALIB_RTOS)
       │
       ▼
     感知引擎接收请求，开始标定计算
       │
       ├──→ [1s周期] dcms_mcu_topic_send(DCMS_SUB_EOLCALIB_RESULT_RTOS)
       │      → SOC: DiagCalib_CalibResultStatusCallback()
       │        → StaticCalib_ResultStatusCheck() / DynCalib_ResultStatusCheck()
       │      → MCU: DiagCalib_CalibResultStatusCallback()
       │
       └──→ [按需返回] dcms_mcu_topic_send(DCMS_CALIB_DATA_ACK_RTOS)
              → SOC: DiagCalib_CalibResultDataCallback()
                → StaticCalib_ResultDataCheck() / DynCalib_ResultDataCheck()
                → 设置 gGetCalibDataDeltaFlag = true
```

---

## 三、DMM 标定模式切换 —— 四层调用链

### 3.1 架构总览

```
Layer 1 (SOC 车型代码)          Layer 2 (DCMS进程内)         Layer 3 (SOC 平台DMM)      Layer 4 (DAGS感知)
─────────────────────          ────────────────────        ─────────────────────        ────────────────
DiagCalib_DmmModeCmd()         DCMS Service               dmm_calib_service_cb()     ServiceAck.c
  │                              │                          │                          │
  ├─ SET + EVENT_START_* ────→  DCMS_CLIENT_DMM_xxx ────→  ├─ Dmm_SetEvent()   ──→   Dmm_SetPerMode()
  │                              │                          │   (dmm_main_common.c)       (DAGS_FACTORY/DYNAMIC)
  ├─ QUERY ─────────────────→  ====================== ────→  ├─ Dmm_GetCalibResult()     │
  │                              │                          │   ├─ 系统模式检查           │
  │                              │                          │   └─ Dmm_GetPerModeResult() │
  ▼                              ▼                          ▼                            ▼
DiagCalib_DmmModeStatusCallback() dcms_mcu_service_      ack_data={cmd,status}       per_dags_mask协商
  gDmmCalibModeStatus = ack[1]    server_set_ackdata()     返回SOC适配层
```

**关键**: DMM四层调用链全部运行在SOC侧（同一进程或SOC内部DCMS通信），不涉及MCU。
MCU固件的DMM客户端(DmmCalibMode.c)可以独立发送命令到同一个SOC平台DMM服务。

### 3.2 Layer 1 — SOC侧DMM模式命令

[DmmCalibMode.c:29](DmmCalibMode.c#L29):

```c
int32_t DiagCalib_DmmModeCmd(DmmCalibModeCmd cmd, uint8_t data)
{
    uint8_t sendData[2] = {cmd, data};
    // cmd: DSAR_DMM_CALIB_MODE_SET(1) 或 DSAR_DMM_CALIB_MODE_QUERY(2)
    // data: DMM_EVENT_STOP_CALIB(0) / START_STATIC(1) / START_DYN(2)

    if (cmd == DSAR_DMM_CALIB_MODE_SET) {
        dcms_mcu_topic_send(DCMS_CLIENT_DMM_CALIB_MODE_CMD, sendData, 2);
        DiagCalib_DmmModeStatusReset();  // ← 重置为 ALLOW
    } else if (cmd == DSAR_DMM_CALIB_MODE_QUERY) {
        dcms_mcu_topic_send(DCMS_CLIENT_DMM_CALIB_MODE_CMD, sendData, 2);
    }
}

// 状态全局变量
uint8_t gDmmCalibModeStatus = DSAR_DMM_CALIB_ALLOW;  // 初始值0

// 回调更新
int32_t DiagCalib_DmmModeStatusCallback(const uint8_t* data, uint32_t len) {
    gDmmCalibModeStatus = data[1];  // data[] = {cmd_echo, status}
}
```

关键设计：SOC侧每次SET之后立即重置 `gDmmCalibModeStatus = ALLOW`，然后等待DCMS Service响应异步更新。

### 3.3 Layer 2 — DCMS Topic映射（SOC内部）

SOC车型适配层使用 `DCMS_CLIENT_DMM_CALIB_MODE_CMD`（CLIENT=1）发送，DCMS 框架通过共享 topic 路径 `/sys/dmm_calib/v1` 路由到 SOC 平台层的 `DCMS_TOPIC_ASW_CALIB_SERVER`（SERVER=1），后者注册在 [dmm_dcms_configs.h](dmm_dcms_configs.h#L197)。两者在编译期通过同一条 `/sys/dmm_calib/v1` 路径关联，DCMS 框架自动完成 CLIENT→SERVER 的请求-响应路由。

MCU 固件侧的 `DCMS_CLIENT_DMM_CALIB_MODE_CMD`（也配置为 CLIENT=1，同路径）可以向同一 SERVER 独立发送 DMM 命令。

### 3.4 Layer 3 — SOC平台DMM服务回调

[asw_mcu.c:998](asw_mcu.c#L998):

```c
static int32_t dmm_calib_service_cb(const uint8_t* data, uint32_t len)
{
    uint8_t cmd = data[0];
    uint8_t ack_data[2] = {DMM_CALIB_MODE_NONE, 0};

    if (cmd == DMM_CALIB_MODE_SET) {
        Dmm_SetEvent((eEventToDmm)data[1]);     // ★ 调用平台层Dmm_SetEvent
        ack_data[0] = DMM_CALIB_MODE_SET;
        ack_data[1] = Dmm_GetCalibResult();      // ★ 返回当前结果
    } else if (cmd == DMM_CALIB_MODE_QUERY) {
        ack_data[0] = DMM_CALIB_MODE_QUERY;
        ack_data[1] = Dmm_GetCalibResult();
    }

    dcms_mcu_service_server_set_ackdata(DCMS_TOPIC_ASW_CALIB_SERVER, ack_data, 2);
    return 0;
}
```

注册位置（asw_mcu.c初始化时调用）:
```c
dcms_mcu_topic_setup_callback(DCMS_TOPIC_ASW_CALIB_SERVER, dmm_calib_service_cb, NULL, 0u);
```

### 3.5 Layer 3.5 — Dmm_SetEvent 与 Dmm_GetCalibResult

[dmm_main_common.c:13](dmm_main_common.c#L13) — 平台层实现:

```c
static eEventToDmm g_dmm_event;

void Dmm_SetEvent(eEventToDmm event)
{
    g_dmm_event = event;

    if (event == EVENT_START_STATIC_CALIB)
        Dmm_SetPerMode(DAGS_FACTORY_CALIB, 0);    // mode=16
    else if (event == EVENT_START_DYN_CALIB)
        Dmm_SetPerMode(DAGS_DYNAMIC_CALIB, 0);    // mode=18
    else
        Dmm_SetPerMode(0, 0);                     // STOP → 清除模式
}

uint8 Dmm_GetCalibResult(void)
{
    eDmmCalibStatus ret;

    // 空闲模式 → 允许标定
    if (SystemModeSR_Pp_SysMode == SYS_MODE_INITIAL
        || SystemModeSR_Pp_SysMode == SYS_MODE_STANDBY
        || SystemModeSR_Pp_SysMode == SYS_MODE_READY
        || SystemModeSR_Pp_SysMode == SYS_MODE_OFF
        || SystemModeSR_Pp_SysMode == SYS_MODE_ERROR)
    {
        ret = DMM_CALIB_ALLOW;
    }
    // 标定模式 → 查询DAGS协商结果
    else if (SystemModeSR_Pp_SysMode == SYS_MODE_CALIBRATING)  // =12
    {
        ret = Dmm_GetPerModeResult();  // → WAITING / SUCCESS / FAIL
    }
    // 行车/泊车模式 → 拒绝标定
    else
    {
        ret = DMM_CALIB_REJECT;
    }

    return ret;
}
```

系统模式检查逻辑：
- `SYS_MODE_INITIAL(0)` / `STANDBY(1)` / `READY(2)` / `OFF(3)` / `ERROR(7)` → **ALLOW**
- `SYS_MODE_CALIBRATING(12)` → 查询DAGS协商结果 → **WAITING / SUCCESS / FAIL**
- 其他模式（行车/泊车等）→ **REJECT**

### 3.6 Layer 4 — DAGS感知原子能力协商

[ServiceAck.c:215](ServiceAck.c#L215):

```c
void Dmm_SetPerMode(uint8 mode, uint8_t submode)
{
    if (per_dags_request != mode || per_dags_submode_request != submode)
    {
        if (per_dags_request != mode)
        {
            // 切换模式 → 重置mask
            per_dags_mask = Cal_PER_DAGS_Mask; // 0x1F6
        }
        else if (per_dags_submode_request != submode)
        {
            // 子模式切换 → 局部mask
            per_dags_mask = (1 << 2) | (1 << 6); // 0x44
            // 所有服务的ModeStatus设为3 (req)
            for (i = 0; i < per_service_table_size; i++)
                per_service_table[i].pAck->ModeStatus = 3;
        }

        per_dags_count = 1;  // ← 标记"协商进行中"
        per_dags_request = mode;
        per_dags_submode_request = submode;

        // 通知PHMC系统模式管理器
        PHMCSysmode.dmm_set_domain_mode.mode_req = per_dags_request;
    }
}

uint8 Dmm_GetPerModeResult(void)
{
    uint8 ret;

    if (per_dags_count > 0)
        ret = DMM_CALIB_WAITING;   // count>0 → 协商未完成
    else if (per_dags_mask == 0)
        ret = DMM_CALIB_SUCCESS;   // mask=0 → 所有服务已确认
    else
        ret = DMM_CALIB_FAIL;      // mask≠0 → 有服务拒绝

    return ret;
}
```

DAGS协商流程：
1. `Dmm_SetPerMode(DAGS_FACTORY_CALIB=16 或 DAGS_DYNAMIC_CALIB=18)` → 设置 `per_dags_count=1`
2. `ServiceAckRunnable()` 周期性遍历 `per_service_table[]`（9个感知原子能力服务），通过 `dcms_mcu_topic_send()` 向每个待确认的服务发送 `DmmDomainModeReq`
3. 各服务逐个确认（ack），检查 `ModeAck == mode && ModeStatus == 0`，成功后清除对应的 `per_dags_mask` 位
4. `Dmm_GetPerModeResult()` 轮询检查：
   - `per_dags_count > 0` → **WAITING** (协商中)
   - `per_dags_mask == 0` → **SUCCESS** (全部确认)
   - `per_dags_mask != 0` → **FAIL** (有服务拒绝)

**9个DAGS感知原子能力服务**（来自 `ServiceAck.c` 的 `per_service_table[]`）：

| 序号 | 服务名称 | 说明 |
|------|---------|------|
| 1 | vp | 视觉感知 (Visual Perception) |
| 2 | sensor_calib | 传感器标定 (Sensor Calibration) |
| 3 | vins | 视觉惯性里程计 (Visual-Inertial Navigation) |
| 4 | dle | 深度学习引擎 (Deep Learning Engine) |
| 5 | mot | 多目标跟踪 (Multi-Object Tracking) |
| 6 | freespace | 可行驶区域 (Free Space Detection) |
| 7 | semantics | 语义分割 (Semantic Segmentation) |
| 8 | hmi | 人机交互 (Human-Machine Interface) |
| 9 | diagnosis | 诊断服务 (Diagnosis Service) |

模式切换（如进入/退出标定）需要通过掩码 `Cal_PER_DAGS_Mask = 0x1F6` 向这9个服务逐一协商，全部确认后才算切换成功。超时由 `Cal_DMM_DagsModeTimeout` 控制。

---

## 四、UDS DoIP 触发静态标定 —— 完整调用链

### 4.1 RID/DID 一览

| RID/DID | 名称 | 类型 | 实际DCM Callout函数 | 说明 |
|---------|------|------|--------------------|------|
| **RID 0x5100** | TopViewCalibEol | startRoutine | `eol_topViewCalibEol_requestStart_routine()` | 周视4相机标定 (前/后/左/右) |
| **RID 0x5101** | StereoFrontBackArround | startRoutine | `eol_stereoFrontBackArround_requestStart_routine()` | 双目+前视+后视+环视 (7相机, 1R10V) |
| **RID 0x5102** | AfterSales Dyn Calib | startRoutine | `all_afterSales_dynCalib_requestStart()` | 售后动态标定 |
| **RID 0x5103** | LiDAR Calibration | startRoutine | `Eol_0x5103_LiDAR_calibration_requestStart_routine()` | 激光雷达标定 (条件: `HAVE_JIMU_FRONT_LIDAR`) |
| **DID 0x2000** | TopView Delta Data | readData | `eol_topview_result_readData()` | 周视相机标定Delta参数 (x,y,z,roll,pitch,yaw) |
| **DID 0x2001** | Precondition Status | readData | `eol_topViewPrecondition_readData()` | 前置条件状态读取 |
| **DID 0x2002** | Error Code | readData | `eol_topView_errorCode_readData()` | 错误码读取 |
| **DID 0x2003** | Stereo Delta Data | readData | `eol_stereoFrontBackArround_result_readData()` | 双目相机标定Delta参数 |
| **DID 0x2004** | Precondition Status (Stero) | readData | `eol_stereoFrontBackArround_precondition_readData()` | 1R10V前置条件 |
| **DID 0x2005** | Error Code (Stereo) | readData | `eol_stereoFrontBackArround_errorCode_readData()` | 1R10V错误码 |
| **DID 0x2006** | Dyn Precondition | readData | `afterSales_precondition_status_readData()` | 动态标定前置条件 |
| **DID 0x2008** | Dyn Delta Data | readData | `afterSales_resultDelta_readData()` | 动态标定Delta数据 |
| **DID 0x200A** | Dyn Error Code | readData | `afterSales_errorCode_readData()` | 动态标定错误码 |
| **DID 0x200E** | LiDAR Error Code | readData | `EOL_LiDAR_calibration_Fault_Reason_1Bytes_ReadData()` | 激光雷达错误码 |
| **DID 0x200F** | LiDAR Precondition | readData | `EOL_LiDAR_precondition_result_2Bytes_ReadData()` | 激光雷达前置条件 |
| **DID 0x2010** | LiDAR Delta Data | readData | `EOL_LiDAR_Calibration_result_24Bytes_ReadData()` | 激光雷达Delta参数 |

### 4.2 RID 0x5100 完整状态机

[DiagCalib_Static.c:932](DiagCalib_Static.c#L932):

```
外部诊断仪 ──→ DoIP(TCP 13400) ──→ AUTOSAR DCM ──→ eol_topViewCalibEol_requestStart_routine()
                                                        │
                      ┌─────────────────────────────────┤
                      ▼                                 ▼
              DCM_INITIAL                          DCM_PENDING
              ──────────                          ──────────
              1. StaticCalib_RidInit()            1. sStaticCalibSetModeTime += 5ms
              2. topic_send(PRECONDITION_CHECK,    2. if (time > 1000ms && time%300==0
                 {REQ_STATIC})                         && precondition.getFlag):
              3. DmmModeCmd(SET,                      DmmModeCmd(QUERY)
                 EVENT_START_STATIC_CALIB)             │
              4. gCurCalibCamera =                     ├─ precondition==0 && DMM==SUCCESS
                 kCalibIdxStcTopView                      → CalibStartTopicSend()
              5. return DCM_E_PENDING                     → return DCM_E_OK ★ 标定启动
                                                       ├─ precondition > 0
                                                          → NRC 0x22 (条件不满足)
                                                       ├─ DMM==WAITING || DMM==ALLOW
                                                          → return DCM_E_PENDING
                                                       └─ else
                                                          → NRC 0x10 (General Reject)
                                                    3. DiagCalib_SetModeTimeOutFlag()
                                                       if (time > 20000ms) → E_NOT_OK
```

关键时序：
- 10ms周期轮询（DCM_PENDING每10ms调用一次）
- 前1秒：等待MCU侧DMM模式切换和前置条件响应
- 1秒后：每300ms查询一次DMM状态
- 20秒超时：`CALIB_SETMODE_TIMEOUT_20000MS`

### 4.3 RID 0x5101 (1R10V 7相机) 流程

与RID 0x5100几乎相同，区别在于：
- `gCurCalibCamera = kCalibIdxStcSteFrBaAR_faw` (复合类型=16)
- 包含7个相机：双目左(4)、双目右(6)、前视(9)、后视(0)、左环视(16)、右环视(19)、后视单目(11)

### 4.4 DID readData 两阶段模式

以DID 0x2000 (TopView Delta Data)为例：

```
DCM_INITIAL:
  StaticCalib_ResultDataRequest()      ← 发送CalibResultDataRequestType请求
  return DCM_E_PENDING

DCM_PENDING:
  if (gGetCalibDataDeltaFlag == true):
    memcpy(Data, gCalibResultData, sizeof)  ← 拷贝Delta数据到UDS响应
    return DCM_E_OK
  else:
    DiagCalib_GetDidTimeOutFlag(calibGetDidTime, &ret)
    if (time > 15000ms): return E_NOT_OK
    return DCM_E_PENDING
```

### 4.5 结果状态路由

[DiagCalib_Common.c:125](DiagCalib_Common.c#L125):

```c
static int32_t DiagCalib_CalibResultStatusCallback(const uint8_t* data, uint32_t len)
{
    memcpy(&gCalibResultStatus, data, sizeof(CalibResultStatusType));

    if (gCurCalibCamera == kCalibIdxStcTopView
        || gCurCalibCamera == kCalibIdxStcSteFrBaAR_faw
        || gCurCalibCamera == kCalibIdxLidarFrontMono)
    {
        StaticCalib_ResultStatusCheck();   // → 静态标定错误码映射
    }
    else if (gCurCalibCamera == kCalibIdxDynALL_faw)
    {
        DynCalib_ResultStatusCheck();      // → 动态标定错误码映射
    }
}
```

通过全局变量 `gCurCalibCamera` 区分当前标定类型，将感知引擎返回的原始状态数据路由到对应的处理函数。

---

## 五、HMI 车机触屏触发标定

### 5.1 触屏坐标区域设计

HMI标定通过车机AVM全景界面的特定触摸区域触发，分车型设计：

```
车型 E001/E202 (carModel=30, 32):

  ┌────────────────────────────────────┐
  │  (0,0)                AVM 界面      │
  │                                    │
  │  ┌──────────┐                      │
  │  │ 进入标定  │ (822-1022, 0-200)    │
  │  └──────────┘                      │
  │                                    │
  │     ┌──────────┐                   │
  │     │ 静态开始  │ (398-498, 674-1126)│
  │     │ 动态开始  │ (524-624, 474-926) │
  │     │ 停止标定  │ (848-948, 474-926) │
  │     └──────────┘                   │
  └────────────────────────────────────┘

车型 E007/E009 (carModel=31, 33):

  ┌────────────────────────────────────┐
  │                         AVM 界面    │
  │              ┌──────────┐          │
  │              │ 进入标定  │          │
  │              │(968-1168,           │
  │              │ 1080-1280)│          │
  │              └──────────┘          │
  │                                    │
  │  (1538-1990, 591-691)  静态开始     │
  │  (1538-1990, 717-817)  动态开始     │
  │  (1538-1990, 1080-1180) 停止标定    │
  │                                    │
  │  (50-170, 178-298)  AVM X退出按钮   │
  └────────────────────────────────────┘
```

### 5.2 HMI 手势数据流

```
车机AVM触屏
  │
  ▼
MCU侧采集 → dcms_mcu_topic_send(DCMS_TOPIC_DIAG_AVM_GESTURE)
  │
  ▼
SOC侧回调: DiagCalib_GetHmiCalibRequestCallBack()
  │
  ├─ 解析 ViAVMGestureStrtType {
  │    touchPointCoordinate_X, touchPointCoordinate_Y,
  │    endGestureType, processGestureType
  │  }
  │
  └─ 存储到 gViAVMGestureStrtRx
       │
       ▼
  HmiStaticCalib_TopviewProcess()  或  HmiDynCalib_Process()
    ├─ 过滤: 只接受点按手势 (endGestureType!=4 && processGestureType!=4)
    ├─ 判断坐标区域 → 进入/开始/停止 标定
    └─ 条件: AVM界面必须打开 (AvmStatusInfo.AvmState==1)
```

### 5.3 HMI 静态标定状态机

[DiagCalib_HmiStatic.c:153](DiagCalib_HmiStatic.c#L153):

```
                   Request_Start_Calib
                        │
                        ▼
   ┌──────────┐     ┌───────────┐     ┌──────────────┐     ┌──────────────┐
   │allowCalib│ ──→ │stateChange│ ──→ │inCalibration │ ──→ │calibSuccess  │
   └──────────┘     └───────────┘     └──────────────┘     └──────────────┘
        ▲                 │                  │                    │
        │                 │ 超时/拒绝        │ 超时/失败          │
        │                 ▼                  ▼                    │
        │           ┌──────────────┐   ┌──────────────┐           │
        └───────────│ calibFailure │←──│ calibFailure │←──────────┘
                    └──────────────┘   └──────────────┘
                    
   allowCalib:     sTimeOutMs=0, 进入stateChange
   stateChange:    每300ms查询DMM状态, SUCCESS→发启动请求→inCalibration
                   超时20s/REJECT/FAIL→calibFailure
   inCalibration:  每1500ms打印状态, 3s后开始检查结果
                   全部成功→calibSuccess, 任一失败→calibFailure
                   超时150s→calibFailure
   calibSuccess:   发送DMM STOP, 发送HMI_SUCCESS到车机, 回到allowCalib
   calibFailure:   发送DMM STOP, 按优先级(cam0>cam2>cam7>cam9)上报错误码, 回到allowCalib
```

### 5.4 HMI 动态标定状态机

与静态标定状态机结构完全相同，区别在于：
- 使用 `kCalibIdxDynALL_faw` (复合类型=17) 作为标定类型
- 超时时间：`DYNAMIC_CALIB_ALLCAM_TIMEOUT = 1000000ms` (1000秒)
- DMM事件: `DMM_EVENT_START_DYN_CALIB`
- 退出时发送 `detail_type = DynStopCalib`

### 5.5 HMI 互斥锁机制

```c
HmiCalibRequestMutex_t HmiCailbReqMutex = Request_None;

// 静态标定开始: HmiCailbReqMutex = Request_StaticCalib
// 动态标定开始: HmiCailbReqMutex = Request_DynCalib
// 标定停止:     HmiCailbReqMutex = Request_None

// 进入条件: HmiCailbReqMutex == Request_None
// 这确保静态和动态标定不会同时进行
```

### 5.6 标定退出/停止时的清理动作

无论标定成功还是失败，退出时统一执行以下清理（以静态为例，[DiagCalib_HmiStatic.c:254-298](DiagCalib_HmiStatic.c#L254)）：

```c
// calibSuccess / calibFailure 分支:
sCalibStatus     = allowCalib;           // 1. 状态机复位
sHmiCalibRequest = Request_Default;       // 2. 请求类型复位
sStartCalib      = false;                // 3. 标定启动标志清除
gCurCalibCamera  = CALIB_CAMERA_DEFAULT;  // 4. 当前标定相机复位
DiagCalib_DmmModeCmd(SET, DMM_EVENT_STOP_CALIB);  // 5. ★ 发送DMM停止标定
// 6. 上报最终结果到车机HMI

// Request_Stop_Calib 分支 (line 306-327):
HmiCailbReqMutex  = Request_None;        // 1. 释放互斥锁
sCalibStatus     = allowCalib;           // 2. 状态机复位
sHmiCalibRequest = Request_Default;       // 3. 请求类型复位
gCurCalibCamera  = CALIB_CAMERA_DEFAULT;  // 4. 当前标定相机复位
HmiStaticCalib_CalibStopTopicSend();     // 5. ★ 发送停止请求到感知引擎
DiagCalib_DmmModeCmd(SET, DMM_EVENT_STOP_CALIB);  // 6. ★ 发送DMM停止标定
HmiStaticCalib_HmiIndexSendStopRequestToDmmTx();  // 7. ★ 通知HMI退出
```

**重要结论：标定模式切换不涉及进程清理。** 切换过程仅包含：
1. 状态机变量复位（`sCalibStatus`, `sHmiCalibRequest`, `gCurCalibCamera`）
2. DMM模式SET为STOP → `Dmm_SetPerMode(0, 0)` → 清除DAGS模式，重置 `per_dags_mask`
3. 向感知引擎发送停止消息（`detail_type = StaStopCalib/DynStopCalib`）
4. 向HMI发送停止通知（`DCMS_TOPIC_HMI_CALIB_REQ`, `leftP=0xFF`）

不存在 `Update_KillProcess()` 或类似的进程清理操作。`Update_KillProcess()` 仅用于OTA刷写流程（DID 0xF199, RID 0xFF00），与标定完全无关。

---

## 六、前置条件检测 —— 完整信号清单

### 6.1 数据流

```
SOC: dcms_mcu_topic_send(PRECONDITION_CHECK, {reqPreconditionType})
  │
  └─→ MCU: DiagCalib_PreconditonCheckCallbackAck()
        │
        ├─ 读取20+车辆信号 (DiagProxy_Get*)
        ├─ 按标定类型做位掩码判断
        ├─ 组装 respResult.respPreconditionStatus
        ├─ 调试绕过: if (gPreconditionCalibDebug==DEBUG) → respResult=0
        └─ dcms_mcu_service_server_set_ackdata(PRECONDITION_CHECK, &respResult, 1)
             │
             └─→ SOC: DiagCalib_GetPreconditionDataCallback()
                   gPreconditionCalibStatus.getPreconditionStatus = data[0]
                   gPreconditionCalibStatus.getFlag = true
```

### 6.2 静态标定前置条件 (REQ_STATIC=1)

| 条件 | 信号源 | 检查逻辑 | Bit位 |
|------|--------|---------|-------|
| 四门两盖 | FL/FR/RL/RR Door, Tailgate, FrontHood | 任意门或盖打开 → 置位 | `Static_FourDoorTwoCovers_PosBit` |
| 档位 | LeverInfo | `leverInfo != LEVERINFO_N_GEAR(2)` → 置位 | `Static_GearNot_N_PosBit` |
| 车灯 | HiBeam, LowBeam, 左右转向, 廓灯 | 检查代码存在但已注释（不生效） | `Static_CarLightNotClose_PosBit`(注释) |
| 胎压 | TPMS FL/FR/RL/RR | 任意胎压异常 → 置位 | `Static_TirePressAbnormal_PosBit` |
| 主动悬架 | ActiveSuspensionInfo | `!=0` → 置位 | `Static_ActiveSuspensionAbnormal_PosBit` |

### 6.3 动态标定前置条件 (REQ_DYNAMIC=2)

| 条件 | 信号源 | 检查逻辑 | Bit位 |
|------|--------|---------|-------|
| 四门两盖 | 同上 | 任意门或盖打开 → 置位 | `Dyn_FourDoorTwoCovers_PosBit` |
| 车辆静止 | VehicleSpeed | `speed != 0` → 置位（非静止） | `Dyn_CarStatusNotStill_PosBit` |
| 轮速异常 | WheelSpeed FL/FR/RL/RR | 任意轮速异常 → 置位 | `Dyn_WheelSpeedAbnormal_PosBit` |
| 胎压 | TPMS FL/FR/RL/RR | 任意胎压异常 → 置位 | `Dyn_TirePressAbnormal_PosBit` |

### 6.4 所有读取的车辆信号 (DiagProxy接口)

```
DiagProxy_GetFLDoorStatus()           DiagProxy_GetFRDoorStatus()
DiagProxy_GetRLDoorStatus()           DiagProxy_GetRRDoorStatus()
DiagProxy_GetTailgateDoorSt()         DiagProxy_GetFrontHoodStatus()
DiagProxy_GetHighBeamSwtSt()          DiagProxy_GetLowBeamSt()
DiagProxy_GetTurnLitLeftSt()          DiagProxy_GetTurnLitRightSt()
DiagProxy_GetPositionLitSwtSt()       DiagProxy_GetLeverInfo()
DiagProxy_GetVehicleSpeed()           DiagProxy_GetWheelSpeedDirection_FL_Q()
DiagProxy_GetWheelSpeedDirection_FR_Q()  DiagProxy_GetWheelSpeedDirection_RL_Q()
DiagProxy_GetWheelSpeedDirection_RR_Q()  DiagProxy_GetTPMS_FLTPMSError()
DiagProxy_GetTPMS_FRTPMSError()       DiagProxy_GetTPMS_RLTPMSError()
DiagProxy_GetTPMS_RRTPMSError()       DiagProxy_GetActiveSuspensionInfo()
```

---

## 七、错误码三级翻译链

### 7.1 翻译链总览

```
感知引擎 (ZYT内部码)  →  DiagCalib_Static.c  →  UDS DID输出 (FAW OEM码)
                                            │
                                            ▼
                       DiagCalib_HmiStatic.c / DiagCalib_HmiDynamic.c
                                            │
                                            ▼
                                      车机HMI显示 (HMI码)
```

### 7.2 第一级: ZYT内部码 → FAW OEM码

[DiagCalib_CommonType.h:258-273](DiagCalib_CommonType.h#L258):

| ZYT内部码 | 值 | FAW OEM码 (静态) | 值 |
|-----------|-----|------------------|------|
| ZYT_Normal_ERRORCODE | 0x01 | FAW_Normal_ERRORCODE | 0x00 |
| ZYT_NoImage_ERRORCODE | 0x10 | FAW_NoImage_ERRORCODE | 0x01 |
| ZYT_IntrinsicMissing_ERRORCODE | 0x13 | FAW_IntrinsicMissing_ERRORCODE | 0x02 |
| ZYT_ChartNotInView_ERRORCODE | 0x20 | FAW_ChartNotInView_ERRORCODE | 0x03 |
| ZYT_IntrinsicOutOfRange_ERROR | 0x75 | FAW_IntrinsicOutOfRange_ERROR | 0x04 |
| ZYT_ChartDetectError_ERRORCODE | 0x60 | FAW_ChartDetectError_ERRORCODE | 0x05 |
| ZYT_ErrorCheckFail_ERRORCODE | 0x70 | FAW_ErrorCheckFail_ERRORCODE | 0x06 |
| ZYT_ResultOutOfRange_ERRORCODE | 0x40 | FAW_ResultOutOfRange_ERRORCODE | 0x07 |
| ZYT_ParamSystemError_ERRORCODE | 0xA0 | FAW_ParamSystemError_ERRORCODE | 0x08 |
| ZYT_CommSystemError_ERRORCODE | 0xA1 | FAW_CommSystemError_ERRORCODE | 0x09 |
| ZYT_CalcTimeExceed_ERRORCODE | 0x80 | FAW_CalcTimeExceed_ERRORCODE | 0x0A |
| ZYT_MechTFMissing_ERRORCODE | 0x12 | FAW_MechTFMissing_ERRORCODE | 0x0B |
| ZYT_sInProcess_ERROECODE | 0x02 | FAW_SInProcess_ERROECODE | 0x0C |
| ZYT_GeneralError_ERROECODE | 0xFF | FAW_GeneralError_ERROECODE | 0x0D |

静态和动态使用不同的FAW错误码枚举（`FawCalibEolErrorcodeType` vs `FawCalibDynErrorcodeType`），动态多了 `FAW_DYN_IIIumination_ERRORCODE=0x05` 和 `FAW_DYN_Software_ERRORCODE=0x0C`。

### 7.3 第二级: FAW OEM码 → HMI显示码

[DiagCalib_HmiCalibType.h:94-111](DiagCalib_HmiCalibType.h#L94):

| HMI码 | 值 | 含义 |
|-------|-----|------|
| Hmi_FAW_CalibSuccess | 0x3 | 标定成功 |
| Hmi_NoImage_ERRORCODE | 0x4 | 无图像 |
| Hmi_IntrinsicMissing_ERRORCODE | 0x5 | 内参缺失 |
| Hmi_ChartNotInView_ERRORCODE | 0x6 | 标定板不在视野 |
| Hmi_IntrinsicOutOfRange_ERROR | 0x7 | 内参超范围 |
| Hmi_ChartDetectError_ERRORCODE | 0x8 | 标定板检测失败 |
| Hmi_ErrorCheckFail_ERRORCODE | 0x9 | 重投影误差大 |
| Hmi_ResultOutOfRange_ERRORCODE | 0xA | 结果超范围 |
| Hmi_ParamSystemError_ERRORCODE | 0xB | 参数系统错误 |
| Hmi_CommSystemError_ERRORCODE | 0xC | 通信系统错误 |
| Hmi_CalcTimeExceed_ERRORCODE | 0xD | 计算超时 |
| Hmi_MechTFMissing_ERRORCODE | 0xE | 机械TF缺失 |
| Hmi_Software_ERRORCODE | 0xF | 软件错误 |
| Hmi_SInProcess_ERROECODE | 0x10 | 标定进行中 |
| Hmi_GeneralError_ERROECODE | 0x11 | 通用错误 |

### 7.4 失败优先级上报

标定失败时按相机优先级上报错误码（[DiagCalib_HmiStatic.c:272-291](DiagCalib_HmiStatic.c#L272)）：

1. **cam0** (后视, TopView_Rear) — 最高优先级
2. **cam2** (左视, TopView_Left)
3. **cam7** (右视, TopView_Right)
4. **cam9** (前视, TopView_Front) — 最低优先级

只上报第一个检测到错误的相机，后续相机即使有错误也会被 `sCalibPriorityStatus != 0` 跳过。

---

## 八、Delta 参数数据流与大端转换

### 8.1 数据流

```
SOC请求: dcms_mcu_topic_send(DCMS_CLINET_GETCALIB_PARAM_RTOS, CalibResultDataRequestType)
  │
  ▼
感知引擎计算并返回: dcms_mcu_topic_send(DCMS_CALIB_DATA_ACK_RTOS, CalibResultDataType)
  │
  ▼
SOC: DiagCalib_CalibResultDataCallback()
  │
  ├─ memcpy(&gCalibResultData, data, sizeof(CalibResultDataType))
  ├─ 根据 gCurCalibCamera 路由到 StaticCalib_ResultDataCheck() 或 DynCalib_ResultDataCheck()
  ├─ 按 ResultDeltaDataMapType 从 results[] byte array 中提取每相机6个float (x,y,z,roll,pitch,yaw)
  ├─ DiagCalib_BigEndianConvsionDelta() 大端转换
  └─ 设置 gGetCalibDataDeltaFlag = true, gGetDynCalibDataDeltaFlag = true
```

### 8.2 相机Delta数据映射

| 相机 | cameraType | deltaX | deltaY | deltaZ | deltaRoll | deltaYaw | deltaPitch |
|------|-----------|--------|--------|--------|-----------|----------|------------|
| 后视 TopView_Rear | kCalibIdxStcTopView | 0 | 4 | 8 | 12 | 16 | 20 |
| 前视 TopView_Front | kCalibIdxStcTopView | 24 | 28 | 32 | 36 | 40 | 44 |
| 左视 TopView_Left | kCalibIdxStcTopView | 48 | 52 | 56 | 60 | 64 | 68 |
| 右视 TopView_Right | kCalibIdxStcTopView | 72 | 76 | 80 | 84 | 88 | 92 |

每个相机6个float × 4字节 = 24字节，4个相机共96字节，存储在 `result_type=0x02(Delta)` 的 `results[0][0-95]` 中。

### 8.3 大端转换实现

[DiagCalib_Common.c:109](DiagCalib_Common.c#L109):

```c
uint8_t DiagCalib_BigEndianConvsionDelta(uint8_t* taget, uint8_t* source, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        taget[i] = source[length - i - 1];  // 逐字节反转
    }
    return 0;
}

// 调用示例 (转换一个float, 4字节):
DiagCalib_BigEndianConvsionDelta(deltaX, &gCalibResultData.results[camIdx][data_start_byte + deltaXIndex], 4);
```

感知引擎输出为小端序（ARM），UDS诊断协议要求大端序，因此在写入DID结果数组前逐字节反转。

### 8.4 结果数据请求模式

| 模式 | 发送值 | 响应值 | 含义 |
|------|--------|--------|------|
| rModeDelta | 0x02 | 0x08 | Delta相对变化量 |
| rModeOemAbs | 0x03 | 0x09 | OEM坐标系绝对位置 |
| rModeOemDelta | 0x04 | 0x0A | OEM坐标系Delta |
| rModeEgoAbs | 0x05 | 0x0B | 自车坐标系绝对位置 |

---

## 九、Shell 调试命令

### 9.1 SOC侧 (UDS进程)

无独立Shell命令，通过UDS诊断仪DoIP协议触发。

### 9.2 MCU侧 (FreeRTOS Shell)

以下命令在MCU的 `mc_shell` 中可用：

**前置条件调试:**

```bash
# 查看所有前置条件信号值
calib_debug_precondition

# 开启前置条件调试模式（跳过所有检查，respResult=0）
calib_debug_precondition 1
```

**手动触发标定 (跳过UDS):**

```bash
# 启动静态周视标定
start_static_topview_calib

# 启动静态7相机标定 (1R10V)
start_static_SteFrBaAR_calib

# 启动动态全部相机标定
start_dyn_fawAll_calib

# 获取标定参数
get_param_calib

# 停止静态周视标定
stop_static_topview_calib

# 获取静态标定状态
get_static_calib_status
```

**DMM标定模式:**

```bash
# 设置标定模式: 0=退出 1=静态 2=动态
dmm_set_calib_mode {0|1|2}

# 查询当前DMM标定模式状态
dmm_get_calib_mode
# 输出: 0=ALLOW 1=REJECT 2=WAITING 3=SUCCESS 4=FAIL
```

### 9.3 FW应用层 (dsar_fw app_core)

[dsar_fw/app_core/diag/DmmCalibMode/DmmCalibMode.c:141](DmmCalibMode.c#L141):

```bash
# FW应用层Shell命令 (与MCU侧功能相同)
dmm_set_calib_mode {mode}  # 0=退出 1=静态 2=动态
dmm_get_calib_mode          # 查询状态
```

---

## 十、标定类型枚举对照表

### 10.1 CalibClassType 完整枚举

| 枚举值 | 数值 | 说明 | 使用场景 |
|--------|------|------|---------|
| kCalibIdxStcStereoCam4 | 0 | 静态双目相机 | 基础类型 |
| kCalibIdxStcTopView | 1 | 静态周视 (4相机) | EOL RID 0x5100 |
| kCalibIdxStcBackView | 2 | 静态后视单目 | 基础类型 |
| kCalibIdxStcSurroudView | 3 | 静态环视 | 基础类型 |
| kCalibIdxFrontTeleView | 4 | 前视长焦 | 基础类型 |
| kCalibIdxLidarFrontMono | 6 | 激光雷达前向单目 | RID 0x5103 |
| kCalibIdxLidarFrontStereo | 7 | 激光雷达前向双目 | 基础类型 |
| kCalibIdxDynStereoCam4 | 8 | 动态双目 | 基础类型 |
| kCalibIdxDynTopView | 9 | 动态周视 | 基础类型 |
| kCalibIdxDynBackView | 10 | 动态后视 | 基础类型 |
| kCalibIdxDynSurroudView | 11 | 动态环视 | 基础类型 |
| kCalibIdxAllStaticView_byd | 12 | BYD 5R7V全静态 | BYD车型复合 |
| kCalibIdxAllDynView_byd | 13 | BYD 5R7V全动态 | BYD车型复合 |
| kCalibIdxDynLidarFrontMono | 14 | Jimu动态激光雷达 | Jimu车型 |
| kCalibIdxDynAllView_gwm | 15 | GWM 3R7V全动态 | GWM车型复合 |
| **kCalibIdxStcSteFrBaAR_faw** | **16** | **FAW 1R10V全静态 (7相机)** | **FAW RID 0x5101** |
| **kCalibIdxDynALL_faw** | **17** | **FAW 1R10V全动态** | **FAW HMI动态** |
| kCalibIdxDynALL_fawjimu | 20 | FAW 1R10V+Jimu全动态 | FAW Jimu车型 |

复合类型（≥12）在DSAR-APP中展开为多个基础类型处理。

### 10.2 相机ID映射表

| 相机名称 | camera_id | 说明 |
|---------|-----------|------|
| id_camTopView_Rear | 0 | 周视后视 |
| id_camTopView_Left | 2 | 周视左视 |
| id_camStereo_Left | 4 | 双目左 |
| id_camStereo_Right | 6 | 双目右 |
| id_camTopView_Right | 7 | 周视右视 |
| id_camTopView_Front | 9 | 周视前视 |
| id_camBackView | 11 | 后视单目 |
| id_camSurroudView_LF | 16 | 环视左前 |
| id_camSurroudView_LR | 17 | 环视左后 |
| id_camSurroudView_RR | 18 | 环视右后 |
| id_camSurroudView_RF | 19 | 环视右前 |

---

## 十一、静态标定 vs 动态标定 完整对比

| 维度 | 静态标定 (EOL) | 动态标定 (After-sales) |
|------|---------------|----------------------|
| **触发方式** | UDS DoIP (诊断仪) / HMI触屏 | HMI触屏 (主要) |
| **使用场景** | 产线EOL工位 | 4S店售后 |
| **DMM事件** | `EVENT_START_STATIC_CALIB` | `EVENT_START_DYN_CALIB` |
| **DAGS模式** | `DAGS_FACTORY_CALIB = 16` | `DAGS_DYNAMIC_CALIB = 18` |
| **前置条件** | 四门两盖关 + N档 + 胎压正常 + 悬架正常 | 四门两盖关 + 车辆静止 + 轮速正常 + 胎压正常 |
| **SOC标定类型** | kCalibIdxStcTopView(1) / kCalibIdxStcSteFrBaAR_faw(16) | kCalibIdxDynALL_faw(17) |
| **RID** | 0x5100, 0x5101, 0x5103 | 无独立RID (HMI触发) |
| **HMI超时** | `STATIC_CALIB_ALLCAM_TIMEOUT = 150s` | `DYNAMIC_CALIB_ALLCAM_TIMEOUT = 1000s` |
| **错误码枚举** | `FawCalibEolErrorcodeType` (14个) | `FawCalibDynErrorcodeType` (15个, 多了Illumination和Software错误) |
| **HMI操作区域** | E001/202: (398-498, 674-1126) | E001/202: (524-624, 474-926) |
| **停止操作** | `StaStopCalib = 0x7F` | `DynStopCalib = 0x7F` |
| **结果状态查询** | RID requestResults | HMI回调 |
| **Delta数据获取** | DID 0x2000/0x2003 readData (DCM两阶段) | 同静态 (通过gCurCalibCamera路由) |

### 共同点

- 相同的DMM模式切换4层架构
- 相同的DCMS通信Topic集合
- 相同的状态机模型 (allowCalib→stateChange→inCalibration→success/failure)
- 相同的退出清理动作 (状态复位 + DMM STOP + 感知引擎停止 + HMI通知)
- 相同的错误码三级翻译链
- 相同的互斥锁机制 (静态和动态不能同时进行)

---

## 十二、关键文件索引

### SOC侧 (车型适配层)

| 文件 | 行数 | 说明 |
|------|------|------|
| [DiagCalib_CommonType.h](DiagCalib_CommonType.h) | 370 | 全部数据结构: 标定类型、状态码、错误码、手势类型、Delta数据映射 |
| [DiagCalib_Common.h](DiagCalib_Common.h) | 36 | 公共接口与全局变量声明 |
| [DiagCalib_Common.c](DiagCalib_Common.c) | 299 | DCMS Topic初始化(6个Topic)、结果路由、大端转换、超时检测 |
| [DiagCalib_Static.c](DiagCalib_Static.c) | 1396 | UDS RID/DID callout: 0x5100/0x5101/0x5103 startRoutine + DID readData |
| [DiagCalib_Static.h](DiagCalib_Static.h) | — | 静态标定RID/DID接口声明 |
| [DiagCalib_HmiStatic.c](DiagCalib_HmiStatic.c) | — | HMI触屏静态标定: 手势识别 + 5状态机 + 错误码映射 |
| [DiagCalib_HmiDynamic.c](DiagCalib_HmiDynamic.c) | 627 | HMI触屏动态标定: 流程同静态, 超时1000s |
| [DiagCalib_HmiCalibType.h](DiagCalib_HmiCalibType.h) | 149 | 触屏坐标区域(4车型×4区域)、HMI错误码、状态机枚举 |
| [DmmCalibMode.c](DmmCalibMode.c) | 102 | SOC侧DMM模式SET/QUERY + 异步回调更新 |
| [DmmCalibMode_Type.h](DmmCalibMode_Type.h) | 43 | DMM命令/事件/状态枚举定义 |

### MCU侧 (车型适配层)

| 文件 | 行数 | 说明 |
|------|------|------|
| [DiagCalib_Main.c](DiagCalib_Main.c) | 730 | MCU入口: 前置条件检测(20+信号)、Topic初始化、10个Shell命令 |
| [DiagCalib_Common.c](DiagCalib_Common.c) | 106 | MCU侧DMM模式命令(与SOC侧对称) |

### 平台仓 (SOC侧)

| 文件 | 行数 | 关键函数 |
|------|------|---------|
| [dmm_main_common.c](dmm_main_common.c) | 71 | `Dmm_SetEvent()`, `Dmm_GetCalibResult()` — 系统模式检查 |
| [ServiceAck.c](ServiceAck.c#L215-L291) | — | `Dmm_SetPerMode()` 真实实现, `Dmm_GetPerModeResult()`, DAGS感知原子能力协商 |
| [asw_mcu.c](asw_mcu.c#L998-L1053) | — | `dmm_calib_service_cb()` — DCMS标定Service服务端回调 |

### 平台仓 (MCU侧固件)

| 文件 | 行数 | 说明 |
|------|------|------|
| [DmmCalibMode.c](DmmCalibMode.c) | 204 | FW应用层DMM标定模式 + Shell命令 |
| [dmm_if_common.c](dmm_if_common.c) | 1018 | FW侧DMM核心逻辑 (Dmm_SetEvent/Dmm_GetCalibResult) |

---

## 十三、关键常量速查表

| 常量 | 值 | 说明 |
|------|-----|------|
| `CALIB_SETMODE_TIMEOUT_20000MS` | 20000 | DMM模式切换超时 (ms) |
| `CALIB_GETDID_TIMEOUT_15000MS` | 15000 | DID数据读取超时 (ms) |
| `STATIC_CALIB_ALLCAM_TIMEOUT` | 150000 | HMI静态标定超时 (ms) = 150s |
| `DYNAMIC_CALIB_ALLCAM_TIMEOUT` | 1000000 | HMI动态标定超时 (ms) = 1000s |
| `CALIB_REQCALIB_TIMEOUT` | 3000 | 进入标定页面触摸超时 (ms) |
| `CALIB_REQUEST_STARTHMICALIB_FOUR_CNT` | 4 | 进入标定所需连续触摸次数 |
| `CALIB_RESULT_STATUS_LEN` | 127 | 单个相机结果状态数据长度 |
| `kMax` | 32 | 最大相机/结果数量 |
| `DAGS_FACTORY_CALIB` | 16 | 静态标定DAGS模式 |
| `DAGS_DYNAMIC_CALIB` | 18 | 动态标定DAGS模式 |
| `SYS_MODE_CALIBRATING` | 12 | PHMC系统标定模式 |
| `LEVERINFO_N_GEAR` | 2 | N档档位值 |

---

## 十四、架构设计要点总结

1. **分层解耦**: 车型代码(DiagCalib_*) → DCMS通信层 → 平台DMM层(dmm_main_common/ServiceAck) → DAGS感知引擎，每层通过标准接口交互，车型代码不直接操作DAGS
2. **异步状态机**: UDS RID/DID使用DCM标准的DCM_INITIAL→DCM_PENDING两阶段异步模式，HMI使用5状态状态机，均避免阻塞
3. **Service与Topic混用**: 前置条件检测和DMM模式切换使用Service模式(请求-响应)，结果上报使用Topic模式(发布-订阅)，充分利用DCMS两种通信范式
4. **全局状态路由**: 通过 `gCurCalibCamera` 单一变量控制结果数据流向(静态/动态)，简洁但需注意并发保护
5. **车型差异化配置**: 触屏坐标区域、标定相机组合、超时时间等通过宏定义和条件编译适配不同车型，HMI互斥锁防止动静态冲突
6. **不杀进程**: 标定模式切换仅做逻辑状态重置和DAGS模式通知，不涉及进程/线程生命周期管理
