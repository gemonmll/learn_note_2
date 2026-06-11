# 07 UDS Transport Protocol (UdsTp) 架构详解

> **文档目的**：完整、详细、准确地记录 DSAR 平台中 UDS Transport Protocol (UdsTp) 的架构设计，覆盖 SOC (APP) 侧 Master 端、MCU (FW) 侧 Satellite 端、DCMS 跨核通信链路、SID 分发机制、状态机、超时处理以及 UdsTransfer 模块。

---

## 目录

1. [架构概览](#1-架构概览)
2. [DCMS Topic 架构](#2-dcms-topic-架构)
3. [SOC 侧 UdsTp Master 端](#3-soc-侧-udstp-master-端)
4. [MCU 侧 UdsTp Satellite 端](#4-mcu-侧-udstp-satellite-端)
5. [SID 分发表](#5-sid-分发表)
6. [帧布局 (Frame Layout)](#6-帧布局-frame-layout)
7. [状态机](#7-状态机)
8. [NRC 错误处理](#8-nrc-错误处理)
9. [超时机制](#9-超时机制)
10. [UdsTransfer 模块](#10-udstransfer-模块)
11. [完整数据流](#11-完整数据流)
12. [UpdateProxy / InterUpdate / DiagUtility 模块](#12-updateproxy--interupdate--diagutility-模块)
13. [文件索引](#13-文件索引)

---

## 1. 架构概览

### 1.1 整体架构

UdsTp 采用 **Master/Satellite（主从）架构**，在 SOC (APP) 和 MCU (FW) 两侧各有一个 UdsTp 实例，通过 DCMS (Domain Communication Middleware Service) 中间件的 Pub/Sub 机制进行跨核通信。

```
┌─────────────────────────────────────────────────────────────────┐
│                        SOC (APP) Side                           │
│  ┌──────────┐   ┌──────────────┐   ┌───────────────────────┐   │
│  │ DoIP/    │   │   UdsTp      │   │     UdsTransfer       │   │
│  │ External  │──▶│   (Master)   │──▶│  (0x34/35/36/37/38)  │   │
│  │ Tester   │   │              │   │                       │   │
│  └──────────┘   └──────┬───────┘   └───────────────────────┘   │
│                        │ Pub/Sub                                │
│                        │ DCMS IPC                               │
├────────────────────────┼────────────────────────────────────────┤
│                        │         MCU (FW) Side                  │
│                        ▼                                         │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    UdsTp (Satellite)                      │   │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐        │   │
│  │  │0x22 DID │ │0x2E DID │ │0x31 RID │ │ 0x28    │ ...    │   │
│  │  │ Read    │ │ Write   │ │ Routine │ │ CommCtl │        │   │
│  │  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘        │   │
│  │       ▼            ▼           ▼           ▼              │   │
│  │  ┌───────────────────────────────────────────────────┐   │   │
│  │  │              AUTOSAR DCM / DEM / NvM               │   │   │
│  │  └───────────────────────────────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 核心设计原则

| 原则 | 说明 |
|------|------|
| **谁发请求谁做 Master** | SOC 侧主动发起 UDS 请求，所以 SOC 侧是 Master，MCU 侧是 Satellite |
| **异步非阻塞** | 所有 UDS 请求采用两阶段异步模式：DCM_INITIAL（发送）→ DCM_PENDING（轮询等待响应） |
| **抽象服务层** | 将各 SID 的请求/响应逻辑抽象为 `UdsTp_*_Abstract` 通用状态机 |
| **依赖注入** | UdsTransfer 通过函数指针表实现平台解耦 |

### 1.3 UDS 服务类型

| SID | 服务名 | 描述 | 处理位置 |
|-----|--------|------|----------|
| 0x22 | ReadDataByIdentifier | 通过 DID 读取数据 | SOC 抽象层 + MCU Satellite |
| 0x2E | WriteDataByIdentifier | 通过 DID 写入数据 | SOC 抽象层 + MCU Satellite |
| 0x31 | RoutineControl | 例程控制 (启动/停止/查询) | SOC 抽象层 + MCU Satellite |
| 0x14 | ClearDiagnosticInformation | 清除诊断信息 (DTC) | MCU Satellite |
| 0x19 | ReadDTCInformation | 读取 DTC 信息 | MCU Satellite |
| 0x28 | CommunicationControl | 通信控制 (CAN 启停) | MCU Satellite |
| 0x85 | ControlDTCSetting | DTC 设置控制 | MCU Satellite |
| 0x34 | RequestDownload | 请求下载 (刷写准备) | UdsTransfer |
| 0x35 | RequestUpload | 请求上传 | UdsTransfer |
| 0x36 | TransferData | 传输数据 | UdsTransfer |
| 0x37 | RequestTransferExit | 传输退出 | UdsTransfer |
| 0x38 | RequestFileTransfer | 请求文件传输 | UdsTransfer |

---

## 2. DCMS Topic 架构

### 2.1 Topic 分配

UdsTp 使用 3 个 DCMS Topic 进行跨核通信：

| Topic ID | Topic 名称 | 方向 | 用途 |
|----------|-----------|------|------|
| `0x11000` | `DCMS_ADP_TOPIC_COM_UDSTP` | SOC → MCU | 通用 UDS 请求（APP 发送，FW 接收） |
| `0x21000` | `DCMS_ADP_TOPIC_GE_UDSTP` | MCU → SOC | 通用 UDS 响应 (FW 发送，APP 接收) |
| `0x31000` | `DCMS_ADP_TOPIC_UDS_V4_TP` | SOC ↔ MCU | UDS V4 传输通道 (DoIP 透传) |

```
SOC (Master)                          MCU (Satellite)
───────────                          ────────────────
                                          ┌──────────────┐
PUB ──────▶ 0x11000 COM_UDSTP ──────▶ SUB │ g_UdsCmdMap  │
              (UDS Request)                │ SID Dispatch │
                                          └──────┬───────┘
SUB ◀────── 0x21000 GE_UDSTP  ◀────── PUB  │ Handler     │
              (UDS Response)               └──────────────┘
                                          ┌──────────────┐
PUB/SUB ◀──▶ 0x31000 UDS_V4_TP ◀──▶ PUB/SUB │ DoIP Proxy │
              (DoIP Passthrough)           └──────────────┘
```

### 2.2 Topic 注册

**SOC 侧 (plat-bf)** — `UdsTp.c:UdsTp_Cb_Monitor_Register()`：
- 订阅 `0x21000` (GE_UDSTP) 接收 MCU 响应
- 订阅 `0x31000` (UDS_V4_TP) 接收 DoIP 数据
- 向 `0x11000` (COM_UDSTP) 发布请求
- 向 `0x31000` (UDS_V4_TP) 发布 DoIP 数据

**MCU 侧 (dsar-sip)** — `UdsTp_Satellite.c`：
- 订阅 `0x11000` (COM_UDSTP) 接收 SOC 请求
- 向 `0x21000` (GE_UDSTP) 发布响应
- 双向订阅 `0x31000` (UDS_V4_TP)

---

## 3. SOC 侧 UdsTp Master 端

### 3.1 文件位置

| 文件 | 路径 | 说明 |
|------|------|------|
| UdsTp.c | `dsar-hq-plat/dsar-plat-bf/dsar_app/diag/udsonip/UdsTp/UdsTp.c` | 核心实现 |
| UdsTp.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/diag/udsonip/UdsTp/UdsTp.h` | 内部头文件 |
| UdsTp_If.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/diag/UdsTp_If.h` | 公开 API |
| UdsTp_Types.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/diag/UdsTp_Types.h` | 类型定义 |
| PlatDiagTypes.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/diag/PlatDiagTypes.h` | 平台诊断类型 |

### 3.2 核心数据结构

```c
// UdsTp.h — 帧布局宏
#define UDSTP_14SVC_REQ_SID      0x14
#define UDSTP_DID_RDATA_REQ_SID  0x22
#define UDSTP_28SVC_REQ_SID      0x28
#define UDSTP_DID_WDATA_REP_SID  0x2E
#define UDSTP_RC_REQ_SID         0x31
#define UDSTP_NEGRSP_SID         0x7F  // 否定响应 SID

// NRC 代码
#define UDSTP_UDS_NRC_CNDT_NOT_CORRECT   0x22  // 条件不正确
#define UDSTP_UDS_NRC_REQ_SEQ_ERROR      0x24  // 请求序列错误
#define UDSTP_UDS_NRC_REQUESTOUTOFRANGE  0x31  // 请求超出范围
#define UDSTP_UDS_NRC_PENDING            0x78  // 等待中

// 超时参数
#define UDSTP_UDS_TIMEOUT          (50 * 1000)  // 50秒超时
#define UDSTP_REPREAT_REQ_CNT      100          // Pending 重发周期数

// DCMS Topic ID
#define DCMS_ADP_TOPIC_COM_UDSTP   0x11000
#define DCMS_ADP_TOPIC_GE_UDSTP    0x21000
#define DCMS_ADP_TOPIC_UDS_V4_TP   0x31000
```

```c
// UdsTp_Types.h — 角色/功能类型
typedef enum {
    UDSTP_ROLE_READ_DID    = 0x01,  // 0x22 读 DID
    UDSTP_ROLE_WRITE_DID   = 0x02,  // 0x2E 写 DID
    UDSTP_ROLE_ROUTINE     = 0x03,  // 0x31 例程控制
    UDSTP_ROLE_DIAGSVC     = 0x04,  // 0x14/0x19/0x85 综合诊断
    UDSTP_ROLE_CROSS_DIAGSVC = 0x05, // 跨域诊断服务
    UDSTP_ROLE_TRANSFER    = 0x06,  // 0x34-0x38 传输服务
} udstp_rolefunc_t;
```

### 3.3 核心 API (UdsTp_If.h)

```c
// 基础 UDS 请求
void UdsTp_Read_Did_Interface(void *prm);       // 读 DID (0x22)
void UdsTp_Write_Did_Interface(void *prm);      // 写 DID (0x2E)
void UdsTp_Rc_Interface(void *prm);             // 例程控制 (0x31)
void UdsTp_DiagSvc_Interface(void *prm);        // 综合诊断服务 (0x14/0x19/0x85)
void UdsTp_DiagSvc_Cross_Interface(void *prm);  // 跨域诊断服务

// 数据传输
void UdsTp_Transfer_Data_0x36(void *prm);       // TransferData

// 事件回调
void UdsTp_Event_Read_Did(void *prm);
void UdsTp_Event_Write_Did(void *prm);
void UdsTp_Event_Rc(void *prm);
void UdsTp_Event_DiagSvc(void *prm);
void UdsTp_Event_Transfer(void *prm);

// 监控线程
void *UdsTp_Cb_Monitor(void *prm);              // 核心监控/重发线程
```

### 3.4 抽象服务状态机

SOC 侧的每个 SID 服务都实现为统一的 "Abstract" 状态机模式：

```
                    ┌──────────────┐
                    │   IDLE /     │
                    │   COMPLETED  │
                    └──────┬───────┘
                           │ 外部调用 Interface()
                           ▼
                    ┌──────────────┐
          ┌────────▶│ DCM_INITIAL  │ 发送 UDS 请求到 DCMS
          │         └──────┬───────┘
          │                │
          │                ▼
          │         ┌──────────────┐
          │         │ DCM_PENDING  │ 等待响应 / 检查超时
          │         └──┬───────┬───┘
          │            │       │
          │   响应到达  │       │ 超时 / 0x78 Pending
          │            │       │
          │            ▼       ▼
          │         ┌──────┐  ┌──────────────────┐
          │         │ DONE │  │ 重发请求 / TIMEOUT │
          │         └──────┘  └──────────────────┘
          │
          └── 0x78 NRC (等待MCU处理中)
```

**关键函数**：
- `UdsTp_Read_Abstract()` — 0x22 读 DID 抽象状态机
- `UdsTp_Write_Abstract()` — 0x2E 写 DID 抽象状态机
- `UdsTp_Rc_Abstract()` — 0x31 例程控制抽象状态机
- `UdsTp_DiagSvc_Abstract()` — 0x14/0x19/0x28/0x85 综合诊断抽象状态机
- `UdsTp_DiagSvc_Cross_Abstract()` — 跨域诊断抽象状态机

### 3.5 监控线程 (UdsTp_Cb_Monitor)

监控线程是 Master 侧的核心调度器：

```
┌──────────────────────────────────────────────┐
│            UdsTp_Cb_Monitor (20ms 周期)       │
│                                               │
│  1. 检查各服务的超时状态                       │
│     - UDSTP_UDS_TIMEOUT = 50s                 │
│                                               │
│  2. 收到 0x78 (Pending) 时触发重发             │
│     - 每 UDSTP_REPREAT_REQ_CNT (100) 个周期   │
│     - 即 ~500ms 重发一次                       │
│                                               │
│  3. 收到正常响应时回调上层                      │
│     - 调用注册的 Callback 函数                 │
│                                               │
│  4. 超时处理                                  │
│     - 上报 NRC 0x10 (General Reject)           │
│     - 通知上层服务失败                         │
└──────────────────────────────────────────────┘
```

---

## 4. MCU 侧 UdsTp Satellite 端

### 4.1 文件位置

| 文件 | 路径 | 说明 |
|------|------|------|
| UdsTp_Satellite.c | `dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/UdsTp/UdsTp_Satellite.c` | Satellite 核心实现 |
| UdsTp_Satellite_Types.h | `dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/UdsTp/UdsTp_Satellite_Types.h` | 类型定义 |
| UdsTp_Satellite_Func.c | `dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/UdsTp/UdsTp_Satellite_Func.c` | 各服务代理函数 |
| UdsTp_Satellite_Func.h | `dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/UdsTp/UdsTp_Satellite_Func.h` | 代理函数声明 |

### 4.2 Satellite 核心流程

```
┌─────────────────────────────────────────────────────────────┐
│           UdsTp_Satellite 主入口（接收 DCMS 请求）            │
│                                                              │
│  1. 从 DCMS Topic 0x11000 接收 UDS PDU                       │
│                         │                                    │
│                         ▼                                    │
│  2. 解析 SID (PDU[0])                                       │
│     ┌──────────────────────────────────────────┐            │
│     │  SID 匹配 (g_UdsCmdMap[])                │            │
│     │  ┌──────┬───────────────────────────┐    │            │
│     │  │ 0x22 │ → UdsTp_Satellite_Read_Did│    │            │
│     │  │ 0x2E │ → UdsTp_Satellite_Write_Did│   │            │
│     │  │ 0x31 │ → UdsTp_Satellite_Routine  │    │            │
│     │  │ 0x14 │ → UdsTp_Satellite_14Svc    │    │            │
│     │  │ 0x19 │ → UdsTp_Satellite_19Svc    │    │            │
│     │  │ 0x28 │ → UdsTp_Satellite_28Svc    │    │            │
│     │  │ 0x85 │ → UdsTp_Satellite_85Svc    │    │            │
│     │  └──────┴───────────────────────────┘    │            │
│     └──────────────────────────────────────────┘            │
│                         │                                    │
│                         ▼                                    │
│  3. 调用 AUTOSAR DCM 接口                                    │
│     - Dcm_ReadData() / Dcm_WriteData()                       │
│     - Dem_ClearDTC() / Dem_GetDTC()                          │
│     - ComM_RequestComMode() (0x28 CAN控制)                   │
│                         │                                    │
│                         ▼                                    │
│  4. 打包响应 → 发布到 DCMS Topic 0x21000                      │
└─────────────────────────────────────────────────────────────┘
```

### 4.3 SID 分发表 (g_UdsCmdMap)

```c
// UdsTp_Satellite.c
static const udstp_cmd_map_t g_UdsCmdMap[] = {
    {0x22, UdsTp_Satellite_Read_Did},      // ReadDataByIdentifier
    {0x2E, UdsTp_Satellite_Write_Did},     // WriteDataByIdentifier
    {0x31, UdsTp_Satellite_Routine},       // RoutineControl
    {0x28, UdsTp_Satellite_28Svc},         // CommunicationControl
    {0x85, UdsTp_Satellite_85Svc},         // ControlDTCSetting
    {0x14, UdsTp_Satellite_14Svc},         // ClearDiagnosticInformation
    {0x19, UdsTp_Satellite_19Svc},         // ReadDTCInformation
};
```

### 4.4 Satellite 类型定义

```c
// UdsTp_Satellite_Types.h

// UDS 服务分类
typedef enum {
    UDSTP_UDS_CLASS_DID   = 0,  // DID 类 (0x22, 0x2E)
    UDSTP_UDS_CLASS_RID   = 1,  // RID 类 (0x31)
    UDSTP_UDS_CLASS_SVC   = 2,  // 服务类 (0x14, 0x19, 0x28, 0x85)
    UDSTP_UDS_CLASS_TRANS = 3,  // 传输类 (0x34-0x38)
} udstp_uds_class_t;

// DID 配置
typedef struct {
    uint16_t did;                    // Data Identifier
    uint8_t  *data_ptr;             // 数据指针（指向 NvM 或 RAM）
    uint16_t data_len;              // 数据长度
    uint8_t  access_mode;           // 访问模式 (RO/RW)
} udstp_did_config_t;

// RID 配置
typedef struct {
    uint16_t rid;                    // Routine Identifier
    void (*start_func)(void*);      // 启动函数
    void (*stop_func)(void*);       // 停止函数
    void (*query_func)(void*);      // 查询函数
} udstp_routine_config_t;

// 服务代理
typedef struct {
    uint8_t  sid;                    // Service ID
    void (*process_func)(void*);    // 处理函数
} udstp_diagsvc_t;
```

### 4.5 Satellite 代理函数 (UdsTp_Satellite_Func.c)

```c
// DCM 服务桥接 — 直接调用 AUTOSAR DCM 接口
void UdsTp_Satellite_DemSvc_Operation(void *prm);

// 0x28 CommunicationControl — CAN 网络管理控制
void UdsTp_Satellite_28Svc(void *prm);

// 0x85 ControlDTCSetting — DTC 设置开关
void UdsTp_Satellite_85Svc(void *prm);

// 0x14 ClearDiagnosticInformation
void UdsTp_Satellite_14Svc(void *prm);

// 0x19 ReadDTCInformation
void UdsTp_Satellite_19Svc(void *prm);

// PDUR 发送完成回调 — 用于 DoIP 透传模式
void UdsTp_Satellite_PduR_Transmit_Cbk(void *prm);
```

### 4.6 DID / RID 查找

MCU Satellite 侧维护 DID 和 RID 配置表，在收到请求时进行查找：

```
收到 0x22 请求 (DID=0xF190)
        │
        ▼
┌──────────────────────────┐
│ g_DidConfigTable[]       │
│ ┌──────────┬───────────┐ │
│ │ 0xF190   │ VIN 数据   │ │ ──▶ 匹配！返回 data_ptr
│ │ 0xF191   │ ECU 硬件号 │ │
│ │ 0xF192   │ 序列号    │ │
│ │ ...       │ ...       │ │
│ └──────────┴───────────┘ │
└──────────────────────────┘

收到 0x31 请求 (RID=0x0203)
        │
        ▼
┌──────────────────────────┐
│ g_RoutineConfigTable[]   │
│ ┌──────┬────────────────┐│
│ │0x0203│ SecurityAccess ││ ──▶ 匹配！调用 start_func()
│ │0xFF00│ ECU Reset      ││
│ │ ...   │ ...            ││
│ └──────┴────────────────┘│
└──────────────────────────┘
```

---

## 5. SID 分发表

### 5.1 完整 SID 映射总表

| SID | 服务名称 | ISO 14229 定义 | Master (SOC) | Satellite (MCU) | 备注 |
|-----|---------|---------------|-------------|-----------------|------|
| 0x10 | DiagnosticSessionControl | 会话控制 | — | — | 由 DoIP/DCM 处理 |
| 0x11 | ECUReset | ECU 复位 | — | — | 由 DoIP/DCM 处理 |
| 0x14 | ClearDiagnosticInformation | 清除诊断信息 | UdsTp_DiagSvc_Abstract | UdsTp_Satellite_14Svc | 清除 DTC |
| 0x19 | ReadDTCInformation | 读 DTC 信息 | UdsTp_DiagSvc_Abstract | UdsTp_Satellite_19Svc | DTC 读取 |
| 0x22 | ReadDataByIdentifier | 按 ID 读数据 | UdsTp_Read_Abstract | UdsTp_Satellite_Read_Did | DID 读取 |
| 0x27 | SecurityAccess | 安全访问 | — | — | 由 DoIP/DCM 处理 |
| 0x28 | CommunicationControl | 通信控制 | UdsTp_DiagSvc_Abstract | UdsTp_Satellite_28Svc | CAN 控制 |
| 0x2E | WriteDataByIdentifier | 按 ID 写数据 | UdsTp_Write_Abstract | UdsTp_Satellite_Write_Did | DID 写入 |
| 0x31 | RoutineControl | 例程控制 | UdsTp_Rc_Abstract | UdsTp_Satellite_Routine | RID 例程 |
| 0x34 | RequestDownload | 请求下载 | UdsTransfer | — | 刷写准备 |
| 0x35 | RequestUpload | 请求上传 | UdsTransfer | — | 上传准备 |
| 0x36 | TransferData | 传输数据 | UdsTransfer | — | 数据传输 |
| 0x37 | RequestTransferExit | 传输出口 | UdsTransfer | — | 退出传输 |
| 0x38 | RequestFileTransfer | 文件传输 | UdsTransfer | — | FTP-like |
| 0x3E | TesterPresent | 测试仪在线 | — | — | 由 DoIP/DCM 处理 |
| 0x85 | ControlDTCSetting | DTC 设置控制 | UdsTp_DiagSvc_Abstract | UdsTp_Satellite_85Svc | DTC 开关 |

### 5.2 跨域诊断 (Cross Domain)

当 SOC 侧的诊断请求需要路由到其他域（非本地 MCU）时，使用跨域诊断服务：

```
外部 Tester → DoIP → SOC UdsTp → DCMS (0x31000) → 其他域 ECU
                                                      │
                           UdsTp_DiagSvc_Cross_Abstract() 负责路由
```

---

## 6. 帧布局 (Frame Layout)

### 6.1 通用 UDS 请求帧 (SOC → MCU, Topic 0x11000)

```
Byte 0    Byte 1    Byte 2    Byte 3    ...    Byte N
┌─────────┬─────────┬─────────┬─────────┬─────┬─────────┐
│   SID   │ SubFn/  │  DID/RID │  Data   │ ... │  Data   │
│         │  DID_H  │  (可选)  │         │     │         │
└─────────┴─────────┴─────────┴─────────┴─────┴─────────┘
```

各 SID 的具体布局：

**0x22 ReadDataByIdentifier**:
```
┌─────────┬─────────┬─────────┐
│  0x22   │ DID_Hi  │ DID_Lo  │
└─────────┴─────────┴─────────┘
```

**0x2E WriteDataByIdentifier**:
```
┌─────────┬─────────┬─────────┬─────────┬─────┬─────────┐
│  0x2E   │ DID_Hi  │ DID_Lo  │ Data[0] │ ... │ Data[N] │
└─────────┴─────────┴─────────┴─────────┴─────┴─────────┘
```

**0x31 RoutineControl**:
```
┌─────────┬─────────┬─────────┬─────────┬───────────┐
│  0x31   │ SubFn   │ RID_Hi  │ RID_Lo  │ RtnData[] │
│         │(01启动) │         │         │  (可选)    │
│         │(02停止) │         │         │           │
│         │(03查询) │         │         │           │
└─────────┴─────────┴─────────┴─────────┴───────────┘
```

**0x14 ClearDiagnosticInformation**:
```
┌─────────┬─────────┬─────────┬─────────┐
│  0x14   │ DTC[0]  │ DTC[1]  │ DTC[2]  │
│         │  (高字节)│  (中字节)│  (低字节)│
└─────────┴─────────┴─────────┴─────────┘
```

### 6.2 通用 UDS 响应帧 (MCU → SOC, Topic 0x21000)

**正常响应**:
```
┌─────────┬─────────┬─────────┬─────┬─────────┐
│ SID|0x40│  Data   │  Data   │ ... │  Data   │
│(正响应) │         │         │     │         │
└─────────┴─────────┴─────────┴─────┴─────────┘
```
- 正响应 SID = 请求 SID + 0x40
- 例: 0x22 → 0x62, 0x2E → 0x6E, 0x31 → 0x71

**否定响应**:
```
┌─────────┬─────────┬─────────┐
│  0x7F   │  ReqSID │   NRC   │
└─────────┴─────────┴─────────┘
```
- Byte 0: 0x7F (否定响应 SID)
- Byte 1: 原始请求 SID
- Byte 2: NRC (Negative Response Code)

### 6.3 DoIP 透传帧 (Topic 0x31000)

DoIP 透传帧保持原始 UDS PDU 格式不变，直接在 SOC 与 MCU 之间透明传输：

```
┌─────────┬─────────┬─────────┬─────┬─────────┐
│   SID   │  ... PDU Data ...              │
└─────────┴─────────┴─────────┴─────┴─────────┘
```

---

## 7. 状态机

### 7.1 SOC Master 侧抽象状态机

所有 SID 使用统一的两阶段异步状态机：

```
                         ┌──────────┐
                         │ DCMS_IDLE│
                         └────┬─────┘
                              │ Interface() 调用
                              ▼
                    ┌─────────────────┐
                    │  DCM_INITIAL    │
                    │  构造 PDU       │
                    │  发布到 0x11000 │
                    └────┬───────┬────┘
                         │       │
                    成功 │       │ 失败
                         │       ▼
                         │  ┌──────────┐
                         │  │   ERROR  │ → 回调上层 NRC
                         │  └──────────┘
                         ▼
                    ┌─────────────────┐
              ┌────▶│  DCM_PENDING    │
              │     │  等待响应       │
              │     │  检查超时       │
              │     └──┬──────┬───┬──┘
              │        │      │   │
              │  响应  │ 0x78 │   │ 超时 (50s)
              │  到达  │ Pend │   │
              │        │      │   │
              │        ▼      │   ▼
              │  ┌──────────┐│  ┌──────────┐
              │  │ 重发请求  ││  │ TIMEOUT  │ → 回调上层 NRC 0x10
              │  │ (每500ms)││  └──────────┘
              │  └──────────┘│
              │        │      │
              │        └──────┘
              │
              ▼
         ┌─────────────────┐
         │  DCM_COMPLETED  │ → 回调上层成功 / 否定响应
         └─────────────────┘
```

### 7.2 UdsTransfer 状态机

UdsTransfer 维护自己的状态机用于刷写/上传流程：

```
┌──────────────────────────────────────────────────────────────┐
│                   UdsTransferStateEnum                        │
│                                                               │
│  ┌────────────────┐                                          │
│  │  NORMAL_STATE   │ ◀────────────────────────────────────┐  │
│  │  (空闲)         │                                       │  │
│  └───────┬────────┘                                       │  │
│          │ 0x34 RequestDownload                            │  │
│          ▼                                                 │  │
│  ┌────────────────┐                                        │  │
│  │ DOWNLOAD_STATE  │ 0x36 TransferData (循环)              │  │
│  │ (下载中)       │─────────┐                              │  │
│  └───────┬────────┘         │                              │  │
│          │ 0x37 Exit        │                              │  │
│          ▼                  │                              │  │
│  ┌────────────────┐         │                              │  │
│  │  EXIT_STATE     │─────────┘                              │  │
│  └────────────────┘                                        │  │
│                                                               │
│          ┌────────────────┐                                   │
│          │  UPLOAD_STATE   │ ◀── 0x35 RequestUpload           │
│          │  (上传中)       │ 0x36 TransferData (循环)         │
│          └───────┬────────┘                                   │
│                  │ 0x37 Exit                                  │
│                  ▼                                            │
│          ┌────────────────┐                                   │
│          │  EXIT_STATE     │                                   │
│          └────────────────┘                                   │
│                                                               │
│          ┌────────────────┐                                   │
│          │ CERTDOWNLOAD   │ ◀── 证书下载专用                   │
│          │ (证书下载)     │                                   │
│          └────────────────┘                                   │
└──────────────────────────────────────────────────────────────┘
```

### 7.3 Satellite 侧方向状态机

MCU Satellite 侧维护一个方向状态机，用于跟踪当前处于接收还是发送阶段：

```
┌──────────────────┐        ┌──────────────────┐
│  RECEIVE_STATE    │ ─────▶│  PROCESS_STATE    │
│  接收 SOC 请求    │        │  处理/分发请求    │
└──────────────────┘        └────────┬─────────┘
                                     │
                                     ▼
                            ┌──────────────────┐
                            │  TRANSMIT_STATE   │
                            │  发送响应到 SOC   │
                            └────────┬─────────┘
                                     │
                                     ▼
                            ┌──────────────────┐
                            │  RECEIVE_STATE    │ (循环)
                            └──────────────────┘
```

---

## 8. NRC 错误处理

### 8.1 NRC 代码定义

| NRC | ISO 14229 名称 | 含义 | 触发场景 |
|-----|---------------|------|---------|
| 0x10 | General Reject | 一般拒绝 | 未分类错误 / 超时 |
| 0x11 | Service Not Supported | 服务不支持 | SID 不在 g_UdsCmdMap 中 |
| 0x12 | SubFunction Not Supported | 子功能不支持 | 0x31 的子功能码无效 |
| 0x13 | Incorrect Message Length | 消息长度错误 | PDU 长度不符合要求的字节数 |
| 0x22 | Conditions Not Correct | 条件不正确 | 当前会话/状态下不允许该操作 |
| 0x24 | Request Sequence Error | 请求序列错误 | 传输流程中 SID 顺序不对 |
| 0x31 | Request Out Of Range | 请求超出范围 | DID/RID 不在有效范围内 |
| 0x70 | Upload/Download Not Accepted | 上传/下载不被接受 | UdsTransfer 状态不允许 |
| 0x71 | Transfer Data Suspended | 传输数据暂停 | 传输被暂停 |
| 0x72 | General Programming Failure | 一般编程失败 | 刷写过程中出错 |
| 0x73 | Wrong Block Sequence Counter | 块序列号错误 | 0x36 TransferData 块序号不对 |
| 0x78 | Response Pending | 响应等待中 | MCU 处理时间较长，通知 Master 等待 |

### 8.2 NRC 处理流程

```
收到否定响应 (SID=0x7F)
        │
        ▼
┌───────────────────────────────────┐
│  解析 NRC                         │
│                                   │
│  ┌─────────────────────────────┐  │
│  │ NRC == 0x78 (Pending)       │  │
│  │  → 不通知上层               │  │
│  │  → Monitor 线程触发重发     │  │
│  │  → 保持 DCM_PENDING 状态    │  │
│  └─────────────────────────────┘  │
│                                   │
│  ┌─────────────────────────────┐  │
│  │ NRC == 0x13 (长度错误)      │  │
│  │  → 检查 PDU 构造是否正确    │  │
│  │  → 回调上层错误             │  │
│  └─────────────────────────────┘  │
│                                   │
│  ┌─────────────────────────────┐  │
│  │ NRC == 0x31 (超出范围)      │  │
│  │  → DID/RID 不在配置表中     │  │
│  │  → 回调上层错误             │  │
│  └─────────────────────────────┘  │
│                                   │
│  ┌─────────────────────────────┐  │
│  │ 其他 NRC                     │  │
│  │  → 回调上层，带回 NRC 码    │  │
│  │  → 状态机回到 IDLE          │  │
│  └─────────────────────────────┘  │
└───────────────────────────────────┘
```

---

## 9. 超时机制

### 9.1 超时参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `UDSTP_UDS_TIMEOUT` | 50000 ms (50s) | UDS 请求总超时时间 |
| `UDSTP_REPREAT_REQ_CNT` | 100 周期 | Pending 重发间隔计数器 |
| Monitor 线程周期 | 20 ms | 监控线程执行周期 |

### 9.2 超时处理流程

```
┌──────────────────────────────────────────────────────────┐
│              UdsTp_Cb_Monitor (每 20ms 执行)              │
│                                                           │
│  for each active 服务:                                    │
│    │                                                      │
│    ├── 检查 elapsed_time > UDSTP_UDS_TIMEOUT (50s)?       │
│    │   ├── YES → 超时！                                   │
│    │   │   ├── 设置状态为 TIMEOUT                         │
│    │   │   ├── 构造 NRC 0x10 错误响应                     │
│    │   │   └── 回调上层 Callback (失败)                   │
│    │   │                                                  │
│    │   └── NO → 继续等待                                  │
│    │                                                      │
│    └── 检查是否需要重发？                                 │
│        ├── 状态 == DCM_PENDING                            │
│        ├── 上次收到 0x78 NRC                              │
│        ├── 重发计数器 % UDSTP_REPREAT_REQ_CNT == 0        │
│        │   (即每 100 周期 ≈ 500ms 重发一次)               │
│        └── YES → 重新发布请求到 DCMS Topic 0x11000        │
└──────────────────────────────────────────────────────────┘
```

### 9.3 0x78 Pending 重发机制

```
MCU 处理中...
     │
     ▼
返回 0x7F + SID + 0x78 (NRC Pending)
     │
     ▼
SOC Master 收到 0x78:
  1. 不通知上层
  2. 保持 DCM_PENDING 状态
  3. 启动重发计数器
     │
     ▼
每 100 个 Monitor 周期 (~500ms):
  → 重新发送原始请求到 0x11000
  → MCU 继续处理，可能再返回 0x78
     │
     ├── MCU 处理完成 → 正常响应 → DCM_COMPLETED → 回调上层
     └── 超时 50s → TIMEOUT → 回调上层 NRC 0x10
```

---

## 10. UdsTransfer 模块

### 10.1 概述

UdsTransfer 是 SOC 侧负责 UDS 数据传输服务（0x34-0x38）的核心模块，位于 `plat-bf/dsar_app/diag/udsonip/UdsTransfer/`。

### 10.2 文件位置

| 文件 | 路径 | 说明 |
|------|------|------|
| UdsTransfer.c | `plat-bf/dsar_app/diag/udsonip/UdsTransfer/UdsTransfer.c` (2164 行) | 核心实现 |
| UdsTransfer.h | `plat-bf/dsar_app/diag/udsonip/UdsTransfer/UdsTransfer.h` | 内部头文件 |
| UdsTransfer_Type.h | `plat-bf/dsar_app/include/dsar_plat_bf/diag/UdsTransfer_Type.h` | 类型定义 |

### 10.3 核心数据结构

```c
// UdsTransfer_Type.h

// 操作模式
typedef enum {
    MODE_OF_OPERATION_NONE     = 0x00,
    MODE_OF_OPERATION_DOWNLOAD = 0x01,  // 下载 (刷写)
    MODE_OF_OPERATION_UPLOAD   = 0x02,  // 上传
} ModeOfOperationEnum;

// 文件传输操作
typedef enum {
    FT_OP_ADD_FILE    = 0x01,  // 添加文件
    FT_OP_DELETE_FILE = 0x02,  // 删除文件
    FT_OP_REPLACE_FILE = 0x03, // 替换文件
    FT_OP_READ_FILE   = 0x04,  // 读取文件
    FT_OP_READ_DIR    = 0x05,  // 读取目录
} ft_op_cfg_e;

// 传输状态机
typedef enum {
    UDS_TRANSFER_STATE_NORMAL   = 0,  // 空闲
    UDS_TRANSFER_STATE_DOWNLOAD = 1,  // 下载中
    UDS_TRANSFER_STATE_UPLOAD   = 2,  // 上传中
    UDS_TRANSFER_STATE_CERT     = 3,  // 证书下载
} UdsTransferStateEnum;

// 工作线程状态
typedef enum {
    THREAD_STATUS_IDLE      = 0,
    THREAD_STATUS_RUNNING   = 1,
    THREAD_STATUS_COMPLETED = 2,
    THREAD_STATUS_ERROR     = 3,
} UdsTransferThreadStatusEnum;

// 传输上下文
typedef struct {
    ModeOfOperationEnum    mode;             // 当前操作模式
    UdsTransferStateEnum   state;            // 当前状态
    uint8_t                *data_buffer;     // 数据缓冲区指针
    uint32_t               buffer_size;      // 缓冲区大小
    uint32_t               data_offset;      // 数据偏移
    uint32_t               total_size;       // 总数据大小
    uint32_t               block_sequence;   // 块序列号
    uint8_t                memory_address[4]; // 内存地址
    uint8_t                memory_size[4];    // 内存大小
    // ... 更多字段
} UdsTransferContext_t;

// 循环数据缓冲区 (8 MiB)
typedef struct {
    uint8_t  *buffer;          // 缓冲区 (8 MiB)
    uint32_t read_offset;      // 读指针
    uint32_t write_offset;     // 写指针
    uint32_t data_size;        // 当前有效数据大小
    pthread_mutex_t mutex;     // 互斥锁
} UdsTransferDataBuffer_t;

// 依赖注入配置 (12+ 函数指针)
typedef struct {
    // 存储操作
    int32_t (*storage_write)(uint32_t addr, const uint8_t *data, uint32_t len);
    int32_t (*storage_read)(uint32_t addr, uint8_t *data, uint32_t len);
    int32_t (*storage_erase)(uint32_t addr, uint32_t len);

    // 网络/DCMS 通信
    int32_t (*send_response)(uint8_t *data, uint32_t len);
    int32_t (*send_request)(uint8_t *data, uint32_t len);

    // 超时/定时器
    int32_t (*start_timer)(uint32_t timeout_ms);
    int32_t (*stop_timer)(void);

    // 状态回调
    void (*on_transfer_complete)(int32_t result);
    void (*on_transfer_error)(int32_t error_code);
    void (*on_progress)(uint32_t current, uint32_t total);

    // ... 更多
} UdsTransfer_Config_t;
```

### 10.4 核心 API

```c
// 请求下载 0x34 — 准备刷写环境
void UdsTransfer_RequestDownload_0x34(uint8_t *payload, uint32_t len);
// 流程:
//   1. 解析内存地址、大小
//   2. 分配 8 MiB 循环缓冲区
//   3. 检查存储空间
//   4. 切换到 DOWNLOAD_STATE
//   5. 启动下载工作线程

// 请求上传 0x35 — 准备上传环境
void UdsTransfer_RequestUpload_0x35(uint8_t *payload, uint32_t len);

// 传输数据 0x36 — 核心数据块传输
void UdsTransfer_TransferData_0x36(uint8_t *payload, uint32_t len);
// 流程 (下载模式):
//   1. 验证块序列号 (block_sequence++)
//   2. 数据写入循环缓冲区
//   3. 下载线程从缓冲区取数据写入存储
//   4. 返回确认响应
// 流程 (上传模式):
//   1. 从存储读取数据块
//   2. 打包发送给请求方

// 请求传输退出 0x37 — 结束传输
void UdsTransfer_RequestTransferExit_0x37(uint8_t *payload, uint32_t len);
// 流程:
//   1. 等待工作线程完成
//   2. 校验数据完整性
//   3. 释放缓冲区
//   4. 切换到 NORMAL_STATE

// 文件传输 0x38 — FTP-like 文件操作
void UdsTransfer_RequestFileTransfer_0x38(uint8_t *payload, uint32_t len);
```

### 10.5 下载流程 (0x34 → 0x36* → 0x37)

```
Tester                        SOC UdsTransfer            MCU
  │                                │                      │
  │── 0x34 RequestDownload ──────▶│                      │
  │   (addr, size, format)         │                      │
  │                                │── 分配 8MiB Buffer   │
  │                                │── 状态→DOWNLOAD      │
  │◀── 0x74 Positive Response ────│                      │
  │   (maxBlockLength)            │                      │
  │                                │                      │
  │── 0x36 TransferData ────────▶│                      │
  │   (blockSeq=1, data)          │                      │
  │                                │── 写入循环缓冲区     │
  │                                │    下载线程→存储     │
  │◀── 0x76 Positive Response ────│                      │
  │                                │                      │
  │── 0x36 TransferData ────────▶│                      │
  │   (blockSeq=2, data)          │         ...          │
  │◀── 0x76 Positive Response ────│                      │
  │         ... (重复)             │                      │
  │                                │                      │
  │── 0x37 RequestTransferExit ──▶│                      │
  │                                │── 等待线程完成       │
  │                                │── 数据校验           │
  │                                │── 释放缓冲区         │
  │                                │── 状态→NORMAL        │
  │◀── 0x77 Positive Response ────│                      │
```

### 10.6 循环缓冲区与工作线程

```
┌──────────────────────────────────────────────────────────────┐
│                    UdsTransfer 数据流                          │
│                                                               │
│  UDS 0x36 请求                                                 │
│      │                                                        │
│      ▼                                                        │
│  ┌──────────────────────────────────────┐                     │
│  │        Cyclic Buffer (8 MiB)          │                     │
│  │                                       │                     │
│  │  ┌─────────────────────────────────┐ │                     │
│  │  │ Data │ Data │ Data │ ... │ Free │ │                     │
│  │  └──────┴──────┴──────┴─────┴──────┘ │                     │
│  │    ▲                          ▲       │                     │
│  │    │ write_offset              │ read_offset               │
│  │    │ (UDS 写入)               │ (线程读取)                 │
│  │  └──────────────────────────────┘     │                     │
│  └──────────────┬───────────────────────┘                     │
│                 │                                              │
│                 ▼                                              │
│  ┌──────────────────────────────────────┐                     │
│  │     Download Thread (工作线程)        │                     │
│  │                                       │                     │
│  │  while (state == DOWNLOAD) {          │                     │
│  │    data = read_from_cyclic_buffer();  │                     │
│  │    storage_write(addr, data, len);    │                     │
│  │    addr += len;                       │                     │
│  │  }                                    │                     │
│  └──────────────────────────────────────┘                     │
│                 │                                              │
│                 ▼                                              │
│  ┌──────────────────────────────────────┐                     │
│  │         Storage (Flash/File)          │                     │
│  └──────────────────────────────────────┘                     │
└──────────────────────────────────────────────────────────────┘
```

---

## 11. 完整数据流

### 11.1 典型 0x22 读 DID 请求完整流程

```
外部 Tester (DoIP)                SOC UdsTp (Master)            MCU UdsTp (Satellite)
══════════════════                ══════════════════            ═══════════════════════

1. UDS Req (0x22, DID=0xF190)
        │
        ▼
2. DoIP Gateway 解析
   → 路由到 UdsTp
        │
        ▼
3.                    UdsTp_Read_Did_Interface()
                       → 构造 PDU: [0x22, 0xF1, 0x90]
                       → 状态 = DCM_INITIAL
                       → PUB to 0x11000 ─────────────────────▶
                                                                 4. SUB from 0x11000
                                                                    解析 SID=0x22
                                                                    g_UdsCmdMap[0x22]
                                                                    → Read_Did(0xF190)
                                                                        │
                                                                        ▼
                                                                    5. 查找 DID 配置表
                                                                       DID=0xF190 = VIN
                                                                        │
                                                                        ▼
                                                                    6. Dcm_ReadData(0xF190)
                                                                       → 返回 VIN 数据
                                                                        │
                                                                        ▼
                                                                    7. 构造响应: [0x62, data...]
                                                                       PUB to 0x21000
                         ◀──────────────────────────────────────────
8. SUB from 0x21000
   解析 SID=0x62 (正响应)
   状态 = DCM_COMPLETED
   回调上层 Callback
        │
        ▼
9. UdsTp_Event_Read_Did()
   → 返回数据给调用方
```

### 11.2 0x31 例程控制完整流程

```
调用方                         SOC UdsTp                    MCU UdsTp
══════                         ═══════════                  ═══════════

1. UdsTp_Rc_Interface()
   (rid=0xFF00, subfn=0x01 启动)
        │
        ▼
2.  构造 PDU: [0x31, 0x01, 0xFF, 0x00]
     状态 = DCM_INITIAL
     PUB to 0x11000 ──────────────────────────▶
                                                      3. SUB from 0x11000
                                                         解析 SID=0x31, SubFn=0x01
                                                         g_UdsCmdMap[0x31]
                                                         → Routine(0xFF00, START)
                                                             │
                                                             ▼
                                                         4. g_RoutineConfigTable[0xFF00]
                                                            → diagnostic_session_control()
                                                             │
                                                             ▼
                                                         5. 执行业务逻辑...
                                                             │
                                                             ▼
                                                         6. 构造响应: [0x71, 0x01, result...]
                                                            PUB to 0x21000
     ◀──────────────────────────────────────
7. SUB from 0x21000
   解析响应
   状态 = DCM_COMPLETED
   回调 UdsTp_Event_Rc()
```

### 11.3 跨域诊断数据流

```
外部 Tester → DoIP → SOC (域A) → DCMS 0x31000 → MCU (域A)
                              │
                              │ UdsTp_DiagSvc_Cross_Abstract()
                              │ 识别目标域不是本地
                              │
                              ▼
                         DCMS 0x31000 (UDS_V4_TP)
                              │
                              ▼
                         域B 的 UdsTp Satellite → DCM → 响应
                              │
                              ▼
                         原路返回 → Tester
```

### 11.4 Pending (0x78) 处理数据流

```
调用方              SOC Master              MCU Satellite
══════              ══════════              ═════════════

1. 发送 0x31 请求 ──▶ DCM_INITIAL ────────▶
                                            2. 接收请求
                                               开始处理...
                                               处理时间较长
                                               │
                                               ▼
                                            3. PUB 0x7F + 0x31 + 0x78
                    ◀── DCM_PENDING ◀────────  (Response Pending)
                    │
                    │  Monitor 线程:
                    │  检测到 0x78
                    │  不上报上层
                    │  开始计数
                    │
                    ├── 100 周期后 (~500ms):
                    │   重发 0x31 请求 ───────▶
                    │                        4. 仍在处理...
                    │                           再次返回 0x78
                    │◀────────────────────────
                    │
                    ├── 200 周期后 (~1s):
                    │   再次重发 ─────────────▶
                    │                        5. 处理完成！
                    │                           返回正响应 0x71
                    │◀────────────────────────
                    │
6. 收到正响应 ◀────── DCM_COMPLETED
   回调上层成功
```

---

## 12. UpdateProxy / InterUpdate / DiagUtility 模块

### 12.1 UpdateProxy

UpdateProxy 是 SOC 侧的更新代理模块，负责协调通过 UDS 进行的 ECU 固件更新流程。

```
┌──────────────────────────────────────────────────────┐
│                  UpdateProxy                          │
│                                                       │
│  1. 接收更新请求（本地/远程）                          │
│  2. 校验更新包签名和完整性                            │
│  3. 通过 UdsTp + UdsTransfer 下发固件                 │
│     - 0x34 RequestDownload → 准备 MCU 刷写环境       │
│     - 0x36 TransferData → 分块发送固件数据           │
│     - 0x37 RequestTransferExit → 完成刷写             │
│  4. 验证刷写结果                                      │
│  5. 触发 ECU 复位 (0x11)                             │
└──────────────────────────────────────────────────────┘
```

### 12.2 InterUpdate

InterUpdate 是 MCU 内部的刷写管理模块，负责与 Bootloader 交互。

```
┌──────────────────────────────────────────────────────┐
│                  InterUpdate (MCU 内部)               │
│                                                       │
│  1. 接收 UpdateProxy 的刷写指令                       │
│  2. 管理 Bootloader 通信                             │
│  3. Flash 分区管理 (App / Boot / Data)                │
│  4. 刷写进度上报                                      │
│  5. 安全校验 (签名验证)                               │
└──────────────────────────────────────────────────────┘
```

### 12.3 DiagUtility

DiagUtility 是诊断工具集模块，提供常用的诊断辅助功能。

```
┌──────────────────────────────────────────────────────┐
│                  DiagUtility                          │
│                                                       │
│  1. DID 数据格式化/转换                               │
│  2. DTC 码格式化                                      │
│  3. VIN/ECU 信息读取封装                              │
│  4. Shell 诊断命令接口 (ecu_shell)                    │
│  5. 工厂参数读写封装                                  │
└──────────────────────────────────────────────────────┘
```

---

## 13. 文件索引

### 13.1 SOC 侧文件 (plat-bf)

| 文件 | 完整路径 | 说明 |
|------|---------|------|
| UdsTp.c | `dsar-hq-plat/dsar-plat-bf/dsar_app/diag/udsonip/UdsTp/UdsTp.c` | Master 核心实现 |
| UdsTp.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/diag/udsonip/UdsTp/UdsTp.h` | 内部头文件 (SID/NRC/Topic 常量) |
| UdsTp_IfTest.cpp | `dsar-hq-plat/dsar-plat-bf/dsar_app/diag/udsonip/UdsTp/UdsTp_IfTest.cpp` | Shell 测试工具 |
| UdsTp_If.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/diag/UdsTp_If.h` | 公开 API |
| UdsTp_Types.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/diag/UdsTp_Types.h` | 类型定义 |
| PlatDiagTypes.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/diag/PlatDiagTypes.h` | 平台诊断类型 |
| UdsTransfer.c | `dsar-hq-plat/dsar-plat-bf/dsar_app/diag/udsonip/UdsTransfer/UdsTransfer.c` | 传输服务 (0x34-0x38) |
| UdsTransfer.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/diag/udsonip/UdsTransfer/UdsTransfer.h` | 传输服务头文件 |
| UdsTransfer_Type.h | `dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/diag/UdsTransfer_Type.h` | 传输服务类型 |

### 13.2 MCU 侧文件 (dsar-sip)

| 文件 | 完整路径 | 说明 |
|------|---------|------|
| UdsTp_Satellite.c | `dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/UdsTp/UdsTp_Satellite.c` | Satellite 核心实现 |
| UdsTp_Satellite_Types.h | `dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/UdsTp/UdsTp_Satellite_Types.h` | Satellite 类型定义 |
| UdsTp_Satellite_Func.c | `dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/UdsTp/UdsTp_Satellite_Func.c` | 代理函数实现 |
| UdsTp_Satellite_Func.h | `dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/UdsTp/UdsTp_Satellite_Func.h` | 代理函数声明 |

### 13.3 DCMS Topic 索引

| Topic ID | 宏定义 | 方向 | 用途 |
|----------|--------|------|------|
| 0x11000 | `DCMS_ADP_TOPIC_COM_UDSTP` | SOC → MCU | UDS 请求 |
| 0x21000 | `DCMS_ADP_TOPIC_GE_UDSTP` | MCU → SOC | UDS 响应 |
| 0x31000 | `DCMS_ADP_TOPIC_UDS_V4_TP` | 双向 | DoIP 透传 |

### 13.4 关键常量索引

| 常量 | 值 | 说明 |
|------|-----|------|
| UDSTP_14SVC_REQ_SID | 0x14 | ClearDiagnosticInformation |
| UDSTP_DID_RDATA_REQ_SID | 0x22 | ReadDataByIdentifier |
| UDSTP_28SVC_REQ_SID | 0x28 | CommunicationControl |
| UDSTP_DID_WDATA_REP_SID | 0x2E | WriteDataByIdentifier |
| UDSTP_RC_REQ_SID | 0x31 | RoutineControl |
| UDSTP_NEGRSP_SID | 0x7F | 否定响应 SID |
| UDSTP_UDS_NRC_CNDT_NOT_CORRECT | 0x22 | 条件不正确 |
| UDSTP_UDS_NRC_REQ_SEQ_ERROR | 0x24 | 请求序列错误 |
| UDSTP_UDS_NRC_REQUESTOUTOFRANGE | 0x31 | 请求超出范围 |
| UDSTP_UDS_NRC_PENDING | 0x78 | 等待中 |
| UDSTP_UDS_TIMEOUT | 50000 | 50秒超时 |
| UDSTP_REPREAT_REQ_CNT | 100 | Pending 重发间隔 (周期数) |
