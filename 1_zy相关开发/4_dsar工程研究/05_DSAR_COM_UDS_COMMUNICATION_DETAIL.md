# DSAR-HQ COM域与UDS域通信架构详解 —— 库依赖、调用链、有无comif对比

> 覆盖：COM/UDS库依赖全景 → Cdd_SomeIpTp.c分发机制 → SOME/IP收发完整调用链 → CAN信号调用链 → DoIP诊断路径 → SDK↔Proxy函数调用关系 → 配置驱动差异化 → 有/无comif对比
> 基于 fawhq_e001_10 车型，部分对比 fawhq_p301

---

## 一、总体架构与核心概念

### 1.1 三域进程模型

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        SOC 侧 (QNX/Linux)                                 │
│                                                                          │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐        │
│  │ dji_com_service  │  │ dji_uds_service  │  │ dji_application  │        │
│  │ (COM域)          │  │ (UDS域)          │  │ (AD域)           │        │
│  │                  │  │                  │  │                  │        │
│  │ 有完整SOME/IP栈: │  │ 无SOME/IP栈:     │  │ 无SOME/IP栈:     │        │
│  │ SomeIpTp,Sd,     │  │ 仅有DoIP(13400)  │  │ 纯消费者         │        │
│  │ LdCom,Com        │  │ Dcm,Dem          │  │                  │        │
│  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘        │
│           │                     │                     │                  │
│           └─────────────────────┴─────────────────────┘                  │
│                                 │                                        │
│                    DCMS 进程间IPC (唯一传输层)                             │
│                                 │                                        │
├─────────────────────────────────┼────────────────────────────────────────┤
│                        MCU 侧 (R52 FreeRTOS)                              │
│  CAN Controller ← CAN物理总线    │                                        │
│  x_dom_can_rt (FW侧comif)       │                                        │
└─────────────────────────────────┴────────────────────────────────────────┘
```

### 1.2 三个关键概念的层次关系

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                  │
│  DCMS = 进程间/IPC传输层                                          │
│    来源: Qualcomm DCOS (dcos_dcms BSP库)                         │
│    平台仓封装: dcms_adapt (dcms_mcu_api.cpp)                     │
│    作用: 二进制数据在进程间搬运（SOC↔MCU跨核、COM↔UDS跨进程）     │
│    不管: 数据格式、路由逻辑、序列化/反序列化                       │
│                                                                  │
│  comif = DCMS之上的路由框架                                       │
│    来源: 平台仓SDK (service_route.cpp, msg_route_soc.cpp等)      │
│    作用: 服务/信号路由配置、SvcDataPackHdl打包/解包、             │
│          超时检测、优先级仲裁、SigIf类型安全接口                   │
│    配置: JSON → 代码生成工具 → proxy代码                          │
│                                                                  │
│  danvince = AUTOSAR Vector DaVinci原生路径                        │
│    AUTOSAR BSW (SomeIpTp → LdCom → Com) 在COM进程内直接消费       │
│    不经过comif SDK，不跨域转发                                    │
│                                                                  │
│  ┌──────────┐                                                    │
│  │  comif   │  ← 路由+序列化+缓存框架 (SDK)                       │
│  ├──────────┤                                                    │
│  │  DCMS    │  ← IPC传输层 (BSP)                                  │
│  └──────────┘                                                    │
│                                                                  │
│  comif 不是 DCMS 的替代品，comif 在 DCMS 之上。                   │
│  两者是分层关系，不是并行关系。                                    │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 为什么UDS不能直接收SOME/IP

**UDS域没有SomeIpTp协议栈。** 证据来自 `microsar_config_uds.cmake`：

```cmake
# microsar_config_uds.cmake — UDS AUTOSAR BSW 模块清单

# 以下四个关键模块全部被注释掉:
# list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST Com)        # 第3行: ✗
# list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST LdCom)      # 第19行: ✗
# list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST Sd)         # 第25行: ✗ (服务发现)
# list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST SomeIpTp)   # 第27行: ✗ (SOME/IP协议栈)

# UDS实际链接的模块 (数据面):
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST SoAd)         # ✔ Socket适配 (仅DoIP端口13400)
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST PduR)         # ✔ PDU路由
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST Dcm)          # ✔ 诊断通信管理
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST DoIP)         # ✔ 诊断over IP
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST Dem)          # ✔ 诊断事件管理
```

而COM域有完整的SOME/IP协议栈：

```cmake
# microsar_config_com.cmake — COM AUTOSAR BSW 模块清单
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST Com)           # ✔
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST LdCom)         # ✔ 大数据COM
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST Sd)            # ✔ 服务发现
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST SomeIpTp)      # ✔ SOME/IP传输协议
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST SoAd)          # ✔ (绑定SOME/IP端口)
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST PduR)          # ✔
```

**物理现实**: 外部ECU的SOME/IP UDP报文到达SOC以太网控制器后，操作系统根据端口号路由到COM进程的SoAd。UDS进程的SoAd只绑定DoIP端口（UDP 13400 / TCP 13400），根本收不到SOME/IP报文。因此，UDS获取SOME/IP跨域服务数据的唯一途径是：**COM进程通过DCMS转发**。

---

## 二、COM域与UDS域库依赖全景

### 2.1 构建入口

所有域的.so由 `consys_bf.cmake` 中的宏统一构建。核心逻辑：

```cmake
# consys_bf.cmake

# 第149-150行: 每个域的基础共享库 (所有域都链接)
target_link_libraries(${PRODUCT_NAME}_com PRIVATE plat_bf_cdd dcos_dcms app_core)
target_link_libraries(uds_v4           PRIVATE plat_bf_cdd dcos_dcms app_core)

# 第173-194行: 根据PRODUCT_CONFIG条件添加proxy静态库到列表中
# ...
# 第213行: COM域 --whole-archive 链接
target_link_libraries(${PRODUCT_NAME}_com PRIVATE
    -Wl,--whole-archive ${product_static_lib_list} autosar_com_${PRODUCT_NAME}
    oem_proxy_${PRODUCT_NAME} -Wl,--no-whole-archive)

# 第237行: UDS域 --whole-archive 链接
target_link_libraries(uds_v4 PRIVATE
    -Wl,--whole-archive ${product_static_lib_list_uds_v4}
    autosar_uds_${PRODUCT_NAME}_v4 -Wl,--no-whole-archive)
```

`--whole-archive` 的作用：强制链接静态库中的所有符号，确保proxy中通过`extern`声明的函数（如`sig_route_send_mcu_tx_rx_sigs`、`svc_route_write_someip_init`等）能被SDK代码解析到。

### 2.2 COM域 (libdji_com_service.so) 完整依赖树

```
libdji_com_service.so
│
├── 【BSP平台层】dcos_dcms
│   Qualcomm DCOS提供的DCMS IPC实现
│   所有跨核/跨进程通信的底层传输
│
├── 【BSP平台层】mini_dcos
│   DCOS的精简封装
│
├── 【平台SDK层】plat_bf_cdd.so (SHARED)
│   与UDS域加载的是同一份.so文件，运行时行为由g_x_dom_someip_config区分
│   ├── cdd/x_dom_someip_rt/service_route.cpp     SOME/IP路由引擎
│   ├── cdd/x_dom_someip_rt/x_dom_someip_rt.cpp   初始化入口
│   ├── cdd/x_dom_can_rt/msg_route_soc.cpp        CAN帧解析引擎
│   ├── cdd/x_dom_can_rt/sig_route_soc.cpp        CAN信号路由 (XDOMCANRT_DJI_COM_SERVICE_EN=1)
│   ├── cdd/x_dom_can_rt/x_dom_can_rt_soc.cpp     初始化入口
│   ├── cdd/x_dom_can_rt/p2p_parser.cpp           P2P解析器
│   └── cdd/dcms_adapt/dcms_mcu_api.cpp           DCMS API封装
│
├── 【平台SDK层】app_core
│   进程框架 (工作链表调度、周期任务)
│
├── 【适配Proxy层】x_dom_someip_rt_gen_lib_com_{PRODUCT}.a (--whole-archive)
│   ├── proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_cfg.cpp
│   │   ★ 120+ SvcItemInfo (AD+UDS的recv端)
│   │   ★ svc_route_recv_svc_info_map (120+项)
│   │   ★ svc_route_send_svc_info_map
│   │   ★ NodeRecvInfo: ad_app(120+服务) + uds_app(3个OTA服务)
│   │   ★ svc_route_recv_svc_info_list
│   │   ★ x_dom_someip_rt_set() 注入com_service_enabled=true等配置
│   ├── proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_sigif_get.cpp
│   ├── proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_sigif_set.cpp
│   └── proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_protocol_*.cpp
│
├── 【适配Proxy层】x_dom_can_rt_gen_lib_com_{PRODUCT}.a (--whole-archive)
│   ├── proxy/x_dom_can_rt_gen_com/x_dom_can_rt_cfg.cpp
│   │   ★ 37个CanMsg对象 (4 MCU_TX + 33 MCU_RX)
│   │   ★ 50+个SigHdl对象
│   │   ★ msg_route_mcu_tx_rx_msgs_map CAN帧查找表
│   │   ★ sig_route_send_mcu_tx_rx_sigs() 信号读取+DCMS转发
│   ├── proxy/x_dom_can_rt_gen_com/x_dom_can_rt_sigif.cpp
│   │   ★ SigIf_Get_xxx() → SigHdl::signal_if_get()
│   └── (SDK源码: msg_route_soc.cpp, sig_route_soc.cpp 编译进此库)
│
├── 【AUTOSAR协议栈】autosar_com_{PRODUCT} (--whole-archive)
│   ├── AUTOSAR BSW: Com, LdCom, Sd, SomeIpTp, SoAd, PduR, BswM, ComM...
│   ├── Cdd_SomeIpTp.c ★ 核心分发文件 (e001_10: 902个回调, 125个接comif)
│   └── Appl配置代码 (net_diagnosis_params.c等)
│
└── 【车型适配】oem_proxy_{PRODUCT} (--whole-archive)
    车型特有的代理逻辑
```

### 2.3 UDS域 (libuds_v4.so) 完整依赖树

```
libuds_v4.so
│
├── 【BSP平台层】dcos_dcms
│   (同COM域, 同一BSP)
│
├── 【BSP平台层】udsonip
│   UDS on IP 协议栈 (独立于AUTOSAR)
│
├── 【平台SDK层】plat_bf_cdd.so (SHARED)
│   ★ 与COM域加载的是磁盘上同一份.so文件
│   运行时行为由UDS proxy注入的g_x_dom_someip_config决定:
│     com_service_enabled = false
│     app_service_enabled = true
│     topic_node_name = "uds_app_comif"
│     svc_route_recv_svc_info_map_ptr → 3个OTA recv服务
│     svc_route_send_svc_info_map_ptr → 4个OTA send服务
│     svc_route_recv_svc_info_list_ptr = nullptr (不做跨域转发)
│
├── 【平台SDK层】app_core
│
├── 【适配Proxy层】x_dom_someip_rt_gen_lib_uds_{PRODUCT}_v4.a (--whole-archive)
│   ├── proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_cfg.cpp
│   │   ★ 3个recv SvcItemInfo: OTA_DownloadCmd_RX, OTA_DownloadStatus_RX, OTA_Script_RX
│   │   ★ 4个send SvcItemInfo: OTA_DownloadCmd_TX, OTA_DownloadStatus_TX,
│   │                          OTA_DownloadScript_TX, OTA_NotifyScript_TX
│   │   ★ svc_route_recv_svc_info_map (3项)
│   │   ★ svc_route_send_svc_info_map (4项)
│   │   ★ svc_route_write_someip_init() — 注册DCMS回调接收COM转发
│   │   ★ x_dom_someip_rt_prod_cfg() — 注入app_service_enabled=true
│   ├── proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_sigif_get.cpp
│   │   ★ SigIf_OnRecvData(id): switch/case → SigIf_xxx_Get → Deserialize → 用户回调
│   ├── proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_sigif_set.cpp
│   │   ★ SigIf_xxx_Set(): Serialize → fwd_svc_via_dcms("/x_dom_someip_rt/app_to_com")
│   └── proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_protocol_*.cpp
│       ★ Serialize / Deserialize 函数
│
├── 【适配Proxy层】x_dom_can_rt_gen_lib_uds_{PRODUCT}_v4.a (--whole-archive)
│   ├── proxy/x_dom_can_rt_gen_uds/x_dom_can_rt_cfg.cpp
│   │   ★ fawhq_e001_10: CAN_UDS_V4=OFF, 几乎为空
│   │   ★ sig_route_update_mcu_tx_rx_sigs_from_com() — hash校验+memcpy
│   └── (SDK源码: sig_route_soc.cpp 编译进此库, XDOMCANRT_DJI_COM_SERVICE_EN=0)
│       ★ 注意: UDS不包含msg_route_soc.cpp，不直接接收CAN帧
│
├── 【AUTOSAR协议栈】autosar_uds_{PRODUCT}_v4 (--whole-archive)
│   ★ 注意: 无 SomeIpTp, Sd, LdCom, Com (全部被注释掉)
│   ├── SoAd (仅DoIP端口13400), PduR
│   ├── Dcm (诊断通信管理), Dem (诊断事件管理)
│   ├── DoIP (诊断over IP, ISO 13400)
│   ├── BswM (BSW模式管理), ComM (通信管理)
│   ├── Fee (Flash EEPROM模拟), Fls (Flash驱动), NvM (非易失存储)
│   └── DiagnosticService.c — 车型UDS服务实现 (诊断会话、文件传输、reset处理)
│
├── 【车型适配】diagproxy_ge_{PRODUCT}.a
│   └── DiagProxy/DiagProxy_Com.cpp
│       ★ C→C++ 桥接层: UDS诊断代码(danvince路径)调comif的SigIf_xxx_Get/Set
│
└── 【车型适配】其他oem_feature模块
    ├── security_cert_manager (安全证书管理)
    ├── DiagCalib (诊断校准)
    ├── FactoryParam (工厂参数)
    ├── SecurityAccess (安全访问, 0x27服务)
    ├── DcmApp_Reset (诊断复位)
    ├── UpdateAdapter (升级适配)
    └── SubscribeProcess (订阅处理)
```

### 2.4 库依赖对比表

| 层次 | 库/模块 | COM域 | UDS域 | 说明 |
|------|---------|-------|-------|------|
| BSP | dcos_dcms | ✔ | ✔ | 同一个BSP IPC库 |
| BSP | udsonip | ✗ | ✔ | UDS专用: UDS on IP协议栈 |
| SDK | plat_bf_cdd.so | ✔ | ✔ | 同一份.so，**运行时行为不同** |
| SDK | app_core | ✔ | ✔ | 进程框架 |
| Proxy | x_dom_someip_rt_gen_{dom} | ✔ | ✔ | 不同proxy: COM 120+服务 vs UDS 3+4个OTA |
| Proxy | x_dom_can_rt_gen_{dom} | ✔ | ✔ | COM有37 CanMsg+50 SigHdl, UDS几乎为空 |
| AUTOSAR | SomeIpTp | ✔ | ✗ | UDS被注释掉 |
| AUTOSAR | Sd (服务发现) | ✔ | ✗ | UDS被注释掉 |
| AUTOSAR | LdCom | ✔ | ✗ | UDS被注释掉 |
| AUTOSAR | Com | ✔ | ✗ | UDS被注释掉 |
| AUTOSAR | SoAd | ✔ | ✔ | COM绑SOME/IP端口, UDS仅绑DoIP 13400 |
| AUTOSAR | PduR | ✔ | ✔ | 两者都有 |
| AUTOSAR | Dcm | ✗ | ✔ | UDS专用: 诊断管理 |
| AUTOSAR | DoIP | ✗ | ✔ | UDS专用: 诊断over IP |
| AUTOSAR | Dem | ✗ | ✔ | UDS专用: 诊断事件 |
| AUTOSAR | BswM/ComM | ✗ | ✔ | UDS专用: 模式管理 |
| AUTOSAR | Fee/Fls/NvM | ✗ | ✔ | UDS专用: 存储栈 |
| OEM | DiagProxy | ✗ | ✔ | UDS专用: C→C++桥接 |

---

## 三、Cdd_SomeIpTp.c —— 分发机制的分水岭

### 3.1 文件概况

`Cdd_SomeIpTp.c` 是理解整个系统最关键的文件。它是AUTOSAR SOME/IP协议栈与上层应用之间的桥接层，由JSON配置工具自动生成。

| 车型 | 路径 | 回调总数 | comif数 | danvince数 |
|------|------|---------|---------|-----------|
| fawhq_p301 | `autosar_adapter/microsar_config_com/Appl/GenData/Cdd_SomeIpTp.c` | 846 | 114 | 732 |
| fawhq_e001_10 | `autosar_adapter/microsar_config_com/Appl/GenData/Cdd_SomeIpTp.c` | 902 | 125 | 777 |

每个SOME/IP服务对应一个 `LdComRxIndication` 回调函数。从SomeIpTp收到数据 → 按Service ID + Method ID分发到对应回调。

### 3.2 两种回调模式

```cpp
// ============ 模式1: comif路径 (跨域转发) ============
// e001_10中有125个, p301中有114个
// 特征: 函数体内调用 x_dom_someip_rt_rx_handle()
//       受 #ifdef X_DOM_SOMEIP_RT_COM_EN 保护
//
FUNC(void, RTE_CODE) LdComRxIndication_Client__Sf_SA_FDD_SeatOccupyInfo__..._Event_RX(
    PduInfoPtr)
{
    uint8 *Data = (uint8 *)PduInfoPtr->SduDataPtr;
    SOMEIP_DEBUG_INFO("<COMIF_SOMEIP_RTE_CALL_BACK> Sf_SA_FDD_SeatOccupyInfo");
    #ifdef X_DOM_SOMEIP_RT_COM_EN
        // 进入comif SDK → 路由判决 → DCMS → UDS/AD
        x_dom_someip_rt_rx_handle(
            SERVER_ID_Sf_SA_FDD_SeatOccupyInfo_RX,
            Data + CDD_SOMEIPTP_XF_DATA_OFFSET,        // 跳过8字节头部
            PduInfoPtr->SduLength - CDD_SOMEIPTP_XF_DATA_OFFSET);
    #endif
}

// ============ 模式2: danvince路径 (COM进程内消费) ============
// e001_10中有777个, p301中有732个
// 特征: 函数体为空 (只有debug打印)
//       LdCom → Com 在COM进程内完成数据消费，不跨域
//
FUNC(void, RTE_CODE) LdComRxIndication_Client__Veh_VM_PowerModeInfo__..._FieldNotify_RX(
    PduInfoPtr)
{
    uint8 *Data = (uint8 *)PduInfoPtr->SduDataPtr;
    SOMEIP_DEBUG_INFO("LdComRxIndication_Client__Veh_VM_PowerModeInfo__PMM_NotifyONChgIGSt__FieldNotify_RX");
    // 函数体为空! 数据由 AUTOSAR LdCom → Com 在 COM 进程内消费
    // 不进comif, 不跨域转发
}
```

### 3.3 编译开关: X_DOM_SOMEIP_RT_COM_EN

```cmake
# microsar_config_com.cmake:41 (e001_10 和 p301 都有)
add_definitions(-DX_DOM_SOMEIP_RT_COM_EN)
```

这个宏定义在COM域的AUTOSAR BSW编译中，使得Cdd_SomeIpTp.c中125(或114)个comif回调的函数体被激活。如果这个宏未定义，所有`x_dom_someip_rt_rx_handle()`调用都会被预处理器移除，所有服务都走danvince路径。

### 3.4 CDD_SOMEIPTP_XF_DATA_OFFSET

```c
// Cdd_SomeIpTp.h
#define CDD_SOMEIPTP_XF_DATA_OFFSET  (8 + 0)  // 跳过8字节的SOME/IP传输层头部
```

传给comif SDK的payload已经去掉了SOME/IP传输层头部，从应用层数据开始。

---

## 四、SOME/IP 数据流完整调用链

### 4.1 收方向: 外部ECU → COM → DCMS → UDS

这是UDS获取SOME/IP跨域服务数据的**唯一途径**。

```
外部ECU
  │ SOME/IP over Ethernet UDP
  ▼
═══════════════════════════ COM 进程 ═══════════════════════════

SoAd (绑定SOME/IP端口)
  → SomeIpTp 解析SOME/IP协议头
    (Service ID, Method ID, Client ID, Session ID, Message Type...)
  → Cdd_SomeIpTp.c 按 Service ID + Method ID 分发
       │
       ├─ 777个服务: 空函数体 → LdCom → Com → COM进程内AUTOSAR消费
       │              (danvince路径, 不进comif)
       │
       └─ 125个服务: #ifdef X_DOM_SOMEIP_RT_COM_EN
                      x_dom_someip_rt_rx_handle(SERVER_ID, payload, len)
                      │
                      ▼
【SDK】x_dom_someip_rt_rx_handle(id, data, len)
       │ [x_dom_someip_rt.cpp:87]
       │ if (g_x_dom_someip_config.com_service_enabled)
       │
       └→ 【SDK】svc_route_rx_handle(id, data, len)
                │ [service_route.cpp:80]
                │
                │ 【SDK→Proxy】查找 SvcItemInfo
                │ auto it = g_x_dom_someip_config
                │     .svc_route_recv_svc_info_map_ptr->find(id);
                │   ↑ 指向 COM proxy 的 svc_route_recv_svc_info_map (120+项)
                │
                │ 【SDK→SDK】缓存原始数据
                │ item_ptr->set_origin_data(id, data, len);
                │   ↑ [x_dom_someip_rt_com.h] SvcItemInfo 类方法
                │   内部: memcpy缓存 + m_recv_count++
                │
                │ 如果是立即发送型(immediate):
                │   svc_route_immediately_pub(id, data, len, count)
                │
                ▼
── 50ms周期任务边界 ──

【SDK】svc_route_cycle_pub()
       │ [service_route.cpp:561]
       │ if (g_x_dom_someip_config.com_service_enabled)  ← COM域=true
       │
       │ 遍历 svc_route_recv_svc_info_list_ptr (指向proxy的vector)
       │
       ├─ NodeRecvInfo for ad_app:
       │   svc_list: {120+ AD服务ID}
       │   topic_name: "/x_dom_someip_rt/com_to_app/ad_app"
       │
       └─ NodeRecvInfo for uds_app:
           svc_list: {SERVER_ID_OTA_DownloadCmd_RX,
                       SERVER_ID_OTA_DownloadStatus_RX,
                       SERVER_ID_OTA_DownloadScript_RX}
           topic_name: "/x_dom_someip_rt/com_to_app/uds_app"
           │
           └→ check_and_send_svc_list_via_dcms_on_timeout()
                │ 遍历 svc_list, 每个 SvcItemInfo:
                │   检查: 是否超时? 是否有新数据?
                │   打包: SvcDataPackHdl::add_data(id, data, len, count)
                │         ↑ 多个服务可打包进一个DCMS消息
                │
                │ 【SDK→DCMS】发送
                │ dcms_mcu_topic_send(topic_id, packed_data, size)
                ▼
         DCMS IPC 跨进程传输
         Topic: "/x_dom_someip_rt/com_to_app/uds_app"

═══════════════════════════ UDS 进程 ═══════════════════════════

【SDK】svc_route_com_to_app_topic_cbk(data, len)
       │ [service_route.cpp:147]
       │ static函数, 由svc_route_write_someip_init注册为DCMS回调
       │ g_x_dom_someip_config.app_service_enabled == true  ← UDS走这个分支
       │
       │ SvcDataUnPackHdl DataUnPackHdl(data, len);
       │ while(1):
       │   DataUnPackHdl.get_next_data(item_id, item_data, item_len, recv_count);
       │   │ ↑ 逐条解包 (一个DCMS消息可包含多个服务)
       │   │
       │   │ 【SDK→Proxy】查找
       │   │ auto it = g_x_dom_someip_config
       │   │     .svc_route_recv_svc_info_map_ptr->find(item_id);
       │   │   ↑ 指向 UDS proxy 的 svc_route_recv_svc_info_map (仅3项)
       │   │
       │   │ 【SDK→SDK】写入数据
       │   │ item_ptr->set_com_recv_count(com_recv_count);
       │   │ item_ptr->set_origin_data(item_id, item_data, item_len);
       │   │   ↑ 内部: 缓存数据 + m_recv_func(id) 回调
       │   ▼
       │
       └→ m_recv_func(id)  ← 函数指针, 指向Proxy回调
             │
             ▼
【Proxy】SigIf_OnRecvData(id)
         │ [x_dom_someip_rt_sigif_get.cpp]
         │
         │ switch(id):
         │   case SERVER_ID_OTA_DownloadCmd_RX:
         │     if (SigIf_OTA_DownloadCmd_Cb != nullptr) {
         │         cpp_PS_SelfDLCmd_strt data;
         │         // 【Proxy→SDK】反序列化
         │         SigIf_OTA_DownloadCmd_Get(&data);
         │         //   → SvcItemInfo::get_usr_data_ext<>(offset, pData, Deserialize)
         │         //   → Deserialize(m_origin_data, size, pData)
         │         SigIf_OTA_DownloadCmd_Cb(&data, SigIf_OTA_DownloadCmd_Ctx);
         │     }
         │     break;
         │   case SERVER_ID_OTA_DownloadStatus_RX: ...
         │   case SERVER_ID_OTA_DownloadScript_RX: ...
         ▼
【APP】用户回调: OnOTASelfDownloadCmd(&data, ctx)
```

### 4.2 发方向: UDS → DCMS → COM → 外部ECU

```
═══════════════════════════ UDS 进程 ═══════════════════════════

【APP】业务逻辑要发送SOME/IP响应
  │
  ▼
【Proxy】SigIf_OTA_DownloadStatus_Set(&resp_data)
         │ [x_dom_someip_rt_sigif_set.cpp]
         │
         │ 【Proxy→SDK】序列化写入
         │ someip_send_svc_info_provided_method_..._TX
         │   .set_usr_data_ext<>(offset, pData, Serialize);
         │   ↑ 内部: Serialize(pData, m_usr_data, &size)
         │
         │ 【Proxy→SDK】立即通过DCMS转发到COM
         │   .fwd_svc_via_dcms("/x_dom_someip_rt/app_to_com", false);
         │   ↑ 内部: SvcDataPackHdl::pack(m_usr_data)
         │          → dcms_mcu_topic_send()
         ▼
   DCMS IPC 跨进程传输
   Topic: "/x_dom_someip_rt/app_to_com"

═══════════════════════════ COM 进程 ═══════════════════════════

【SDK】svc_route_app_to_com_topic_cbk(data, len)
       │ [service_route.cpp:107]
       │
       │ SvcDataUnPackHdl 解包
       │
       │ 【SDK→Proxy】查找 send map
       │ auto it = g_x_dom_someip_config
       │     .svc_route_send_svc_info_map_ptr->find(id);
       │
       │ 【SDK→SDK】缓存数据
       │ item_ptr->set_data(id, data, len);
       │
       │ 【SDK→SDK】通过SOME/IP发送到外部ECU
       │ item_ptr->fwd_svc_via_someip();
       │   → Cdd_SomeIpTp Tx path → SomeIpTp → SoAd → Ethernet
       ▼
外部ECU
```

### 4.3 COM proxy关键数据结构

```cpp
// proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_cfg.cpp (COM proxy)

// 1. 定义所有跨域服务的 SvcItemInfo (120+个)
//    注意: COM proxy同时定义了AD和UDS的recv端SvcItemInfo
//    因为COM是转发者，需要知道所有目标域需要什么服务

/* UDS OTA recv服务 */
SvcItemInfo someip_recv_svc_info_provided_method_PS_FM_OTASelfDL_..._RX(
    SERVER_ID_OTA_DownloadCmd_RX,
    "PS_FM_OTASelfDL_Mgmt_HAD_JO__...",
    100,              // timeout in ms, 超时后不发送
    SigIf_OnRecvData, // 回调 (COM侧不使用此回调，作为占位符传入)
    0);

/* AD算法 recv服务 (120+个) */
SvcItemInfo someip_recv_svc_info_provided_method_PS_VM_SetManagerMaster_RX(
    SERVER_ID_SetManagerMaster_RX, ...);
// ... 共120+个

// 2. 全局查找表 — SDK运行时通过server_id查SvcItemInfo
const map<uint32_t, SvcItemInfo*> svc_route_recv_svc_info_map = {
    /*uds_app*/ {SERVER_ID_OTA_DownloadCmd_RX,    &..._RX},
    /*uds_app*/ {SERVER_ID_OTA_DownloadStatus_RX,  &..._RX},
    /*uds_app*/ {SERVER_ID_OTA_DownloadScript_RX,  &..._RX},
    /*ad_app*/  {SERVER_ID_HMI_xxx_RX,            &..._RX},
    // ... 共120+项
};

// 3. 转发节点列表 — SDK遍历此列表决定发给谁
//    UDS域应接收的服务 (来自COM proxy)
static NodeRecvInfo svc_route_recv_svc_info_uds_app = {
    {
        SERVER_ID_OTA_DownloadCmd_RX,
        SERVER_ID_OTA_DownloadStatus_RX,
        SERVER_ID_OTA_DownloadScript_RX,
    },
    "/x_dom_someip_rt/com_to_app/uds_app"  // DCMS topic
};

static NodeRecvInfo svc_route_recv_svc_info_ad_app = {
    { SERVER_ID_HMI_xxx_RX, /* ... 120+个 */ },
    "/x_dom_someip_rt/com_to_app/ad_app"
};

// 4. 转发列表
vector<NodeRecvInfo*> svc_route_recv_svc_info_list = {
    &svc_route_recv_svc_info_ad_app,
    &svc_route_recv_svc_info_uds_app,
};

// 5. 配置注入 — 将以上数据注入SDK的g_x_dom_someip_config
void x_dom_someip_rt_prod_cfg() {
    x_dom_someip_rt_set("com_service_en",               (void*)(true), ...);
    x_dom_someip_rt_set("svc_route_recv_svc_info_list", (void*)(&list), ...);
    x_dom_someip_rt_set("svc_route_recv_svc_info_map",  (void*)(&recv_map), ...);
    x_dom_someip_rt_set("svc_route_send_svc_info_map",  (void*)(&send_map), ...);
    // ...
}
```

### 4.4 UDS proxy关键数据结构

```cpp
// proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_cfg.cpp (UDS proxy)

// 1. 接收服务 (从DCMS收COM转发来的数据)
SvcItemInfo someip_recv_svc_info_provided_method_PS_FM_OTASelfDL_..._RX(
    SERVER_ID_OTA_DownloadCmd_RX,
    "PS_FM_OTASelfDL_Mgmt_HAD_JO__...",
    100,              // timeout
    SigIf_OnRecvData, // m_recv_func = 数据到达时回调此函数
    0);

const map<uint32_t, SvcItemInfo*> svc_route_recv_svc_info_map = {
    {SERVER_ID_OTA_DownloadCmd_RX,    &someip_recv_svc_info_..._RX},
    {SERVER_ID_OTA_DownloadStatus_RX, &someip_recv_svc_info_..._RX},
    {SERVER_ID_OTA_DownloadScript_RX, &someip_recv_svc_info_..._RX},
};

// 2. 发送服务 (UDS→DCMS→COM→外部ECU)
SvcItemInfo someip_send_svc_info_provided_method_PS_FM_OTASelfDL_..._TX(
    SERVER_ID_OTA_DownloadCmd_TX, "PS_FM_OTASelfDL_Mgmt_HAD_JO__...", 0);

const map<uint32_t, SvcItemInfo*> svc_route_send_svc_info_map = {
    {SERVER_ID_OTA_DownloadCmd_TX,    &someip_send_svc_info_..._TX},
    {SERVER_ID_OTA_DownloadStatus_TX, &someip_send_svc_info_..._TX},
    {SERVER_ID_OTA_DownloadScript_TX, &someip_send_svc_info_..._TX},
    {SERVER_ID_OTA_NotifyScript_TX,   &someip_send_svc_info_..._TX},
};

// 3. 注册DCMS回调 — 告知SDK要订阅哪个topic
void svc_route_write_someip_init(dcms_mcu_topic_cbk cbk) {
    topic_id = dcms_adp_get_topic_id("uds_app_comif",
        "/x_dom_someip_rt/com_to_app/uds_app");
    dcms_mcu_topic_setup_callback(topic_id, cbk, ack_data, sizeof(ack_data));
}

// 4. 配置注入 — 告知SDK: 我是APP域, 不是COM域
void x_dom_someip_rt_prod_cfg() {
    x_dom_someip_rt_set("app_service_en",                (void*)(true), ...);
    x_dom_someip_rt_set("topic_node_name",               (void*)("uds_app_comif"), ...);
    x_dom_someip_rt_set("svc_route_recv_svc_info_map",   (void*)(&recv_map), ...);
    x_dom_someip_rt_set("svc_route_send_svc_info_map",   (void*)(&send_map), ...);
    x_dom_someip_rt_set("svc_route_write_someip_init",   (void*)(init_func), ...);
}
```

---

## 五、CAN 信号流完整调用链

### 5.1 收方向: MCU → DCMS → COM → DCMS → UDS

```
物理CAN总线 → CAN Controller (MCU)
  │
  ▼
═══════════════════════════ MCU (R52) ═══════════════════════════

AUTOSAR COM栈: CanIf → PduR → Com_RxIndication
  → Com_ReceiveSignal() 更新信号值
  → x_dom_can_rt (FW侧): 按CAN帧打包 can_msg_base_with_status_t
  → dcms_mcu_topic_send("/x_dom_can_rt/mcu_tx_rx_msgs")

  DCMS 跨核 IPC

═══════════════════════════ COM 进程 ═══════════════════════════

【SDK】msg_route_mcu_tx_rx_msgs_topic_cbk(data, len)
       │ [msg_route_soc.cpp]
       │ 解析 can_msg_base_with_status_t {channel, can_id, timestamp, data[64], dlc, status}
       └→ push → ThreadSafeQueue (异步解耦)

── 50ms周期任务边界 ──

【SDK】msg_route_main_function()
       └→ msg_route_mcu_tx_rx_can_msgs_proc()
            │ [msg_route_soc.cpp:93]
            │ 从ThreadSafeQueue取出CAN帧
            │ 构造查找key: {channel=0, can_id=0x118}
            │
            │ 【SDK→Proxy】查找路由表
            │ auto it = msg_route_mcu_tx_rx_msgs_map.find(key);
            │   ↑ proxy/x_dom_can_rt_gen_com/x_dom_can_rt_cfg.cpp 定义的全局 map
            │
            │ 【SDK→SDK】缓存原始字节
            │ can_msg_ptr->set_data(data, dlc, status);
            │   ↑ [msg_route_soc.h] CanMsg类
            │   37个CanMsg对象数据被更新 (4 MCU_TX + 33 MCU_RX)
            ▼

【SDK】sig_route_send_mcu_tx_rx_sigs_proc()
       │ [sig_route_soc.cpp:25]
       │ #if XDOMCANRT_DJI_COM_SERVICE_EN  // COM域=1
       │
       └→ 【Proxy】sig_route_send_mcu_tx_rx_sigs()  // extern → proxy定义
                  │ [x_dom_can_rt_gen_com/x_dom_can_rt_cfg.cpp]
                  │
                  ├─ sig_route_send_tx_rx_sigs_dji_uds_service()
                  │   (fawhq_e001_10中为空体, CAN_UDS_V4=OFF)
                  │
                  └─ sig_route_send_tx_rx_sigs_dji_application()
                        │ 遍历所有SigHdl对象:
                        │
                        │ 【Proxy→SDK】读信号物理值
                        │ VDC_DecelerationReq_sighdl.signal_if_get(phy);
                        │   ↑ [sig_handle_soc.hpp] SigHdl<float, uint16_t, float, float>
                        │
                        │ SDK内部:
                        │   1. CanMsg::get_data() → 获取缓存CAN原始字节
                        │   2. extract_bits(data_buf, start_bit=64, len=10, Intel格式)
                        │      → raw = 0x200 (假设值)
                        │   3. phy = (float)raw * 0.01f + (-10.21f)
                        │      → phy = -5.09 m/s²
                        │
                        │ 打包成结构体
                        │ dcms_mcu_topic_send(DCMS_TOPIC_READ_SIGS_APP, ...) → AD
                        │ dcms_mcu_topic_send(DCMS_TOPIC_READ_SIGS_UDS, ...) → UDS

  DCMS IPC 跨进程传输

═══════════════════════════ UDS 进程 ═══════════════════════════

【SDK】sig_route_mcu_rx_sigs_topic_cbk(data, len)
       │ [sig_route_soc.cpp:66]
       │ XDOMCANRT_DJI_COM_SERVICE_EN=0 → 注册为回调 (消费者角色)
       │
       └→ 【Proxy】sig_route_update_mcu_tx_rx_sigs_from_com(data, len)
                  │ [x_dom_can_rt_gen_uds/x_dom_can_rt_cfg.cpp:53]
                  │
                  │ const sig_route_read_tx_rx_sigs_dji_uds_service_t *sig =
                  │     (const sig_route_read_tx_rx_sigs_dji_uds_service_t *)data;
                  │
                  │ lock(mutex)
                  │ if (hash匹配 && 长度匹配):
                  │     memcpy(&local_copy, data, len);
                  │     update_flag = true;
                  ▼
【APP】可直接读取: sig_route_read_tx_rx_sigs_dji_uds_service.xxx_field
```

**fawhq_e001_10的现状**: `CONFIG_X_DOM_CAN_RT_UDS_V4=OFF`，UDS的CAN信号结构体仅定义了基本hash校验和管理debug shell，实际信号字段为空。即该车型UDS不通过comif接收CAN信号。

### 5.2 CAN信号物理值转换链（以VDC减速度请求为例）

```
CAN FD帧 (id=0x118, dlc=32, cycle=20ms)
  │
  ▼
[CanMsg缓存] data_buf[64] — 第8~9字节包含start_bit=64起共10bit的原始值
  │
  ▼
[SigHdl::signal_if_get]
  raw = extract_bits(data_buf, start_bit=64, len=10, is_big_endian=false)
      = 0x200 (假设值512)
  │
  ▼
[物理值转换]
  phy = (float)raw * factor + offset
      = (float)512 * 0.01 + (-10.21)
      = 5.12 + (-10.21)
      = -5.09 m/s²
  │
  ▼
[APP使用]
  SigIf_Get_VDC_DecelerationReq_AEB_VDC_1_Get(&decel);
  // decel = -5.09 m/s²
```

### 5.3 COM CAN proxy关键数据结构

```cpp
// proxy/x_dom_can_rt_gen_com/x_dom_can_rt_cfg.h

// 功能开关
#define XDOMCANRT_DJI_COM_SERVICE_EN                    1  // COM域=1 (转发者)
#define XDOMCANRT_MSG_ROUTE_EN                          1
#define XDOMCANRT_READ_MCU_RX_SIGS_EN                   1
#define XDOMCANRT_READ_MCU_TX_SIGS_EN                   1

// SigHdl模板实例化声明 (每个信号一个)
extern SigHdl<float, uint16_t, float, float>
    VDC_DecelerationReq_AEB_VDC_1_default_mcu_tx_sighdl;
extern SigHdl<uint8_t, uint8_t, uint8_t, uint8_t>
    IBC_BrakePedalStatus_IBC_1_default_mcu_rx_sighdl;
// ... 共50+个SigHdl

// CAN帧查找表
extern const std::map<CanMsgIdPair, CanMsg*> msg_route_mcu_tx_rx_msgs_map;
// 37个CAN帧 (4 MCU_TX + 33 MCU_RX)

// 转发函数 (extern声明，SDK通过此符号调用proxy)
extern void sig_route_send_mcu_tx_rx_sigs(void);
```

---

## 六、UDS DoIP 诊断路径 (不经过COM)

这条路径完全在UDS进程内完成，不需要COM域参与。

```
外部诊断仪 (诊断工具/OTA服务器)
  │ DoIP (ISO 13400)
  │ UDP 13400 (车辆发现), TCP 13400 (诊断会话)
  ▼
═══════════════════════════ UDS 进程 ═══════════════════════════

SoAd (绑定DoIP端口13400)
  → PduR (PDU路由)
  → Dcm (诊断通信管理: 会话控制、安全访问、Tester Present)
       │
       ├→ DiagnosticService.c
       │    SysModeMap, UdsDealFileThread, 文件CRC校验, reset处理
       │    车型特有的UDS服务实现
       │
       ├→ Dcm_Callout_Stubs.c
       │    Dcm回调模板: 会话切换、安全级别切换、0x38文件传输
       │
       ├→ DiagSwc_Callback.c (自动生成)
       │    DID读/写回调:
       │    DiagDid_0xF187_FAW_Part_Number_ReadData()
       │    DiagDid_0xF190_VIN_ReadData() / WriteData()
       │    → rte_config.DID_Read_ScvGRop / DID_Write_ScvGrop
       │
       └→ DiagProxy_Com.cpp  ★ C→C++桥接
            │ 允许UDS诊断代码(danvince C代码)访问comif C++接口
            │
            │ 例如读取COM域转发的OTA下载命令:
            │ DiagProxy_GetOTARRSelfDownloadCmdSig()
            │   → SigIf_HAD_OTA__HAD_OTASelfDL_RRSelfDownloadCmd_Get()
            │     → SvcItemInfo::get_usr_data_ext<>()
            │       → Deserialize() 反序列化
            │
            │ 例如写入诊断响应:
            │ DiagProxy_SetOTARRSelfDownloadStatusSig()
            │   → SigIf_xxx_Set()
            │     → SvcItemInfo::set_usr_data_ext<>(Serialize)
            │       → fwd_svc_via_dcms() → DCMS → COM → SOME/IP → 外部ECU
            ▼
```

**关键点**: DiagProxy_Com.cpp 是C→C++的桥接层。UDS的AUTOSAR诊断代码（Dcm回调）是C语言，而comif SDK的SvcItemInfo/SigIf是C++模板类。DiagProxy提供了一层`extern "C"`函数封装，让诊断代码能间接访问comif跨域信号。

---

## 七、SDK ↔ Proxy 函数调用关系

### 7.1 四种设计模式

| 模式 | 描述 | 关键实例 |
|------|------|---------|
| **指针注入** | Proxy通过`x_dom_someip_rt_set(key, ptr)`将数据指针写入SDK的`g_x_dom_someip_config`全局结构体 | `g_x_dom_someip_config.svc_route_recv_svc_info_map_ptr = &proxy_map` |
| **extern符号** | Proxy定义函数，SDK中extern声明，通过`--whole-archive`在链接时解析 | `extern void sig_route_send_mcu_tx_rx_sigs()` |
| **回调注册** | Proxy将函数指针作为参数传给SDK构造函数 | `SvcItemInfo(..., SigIf_OnRecvData)` → `m_recv_func` |
| **模板引用绑定** | Proxy实例化模板类并绑定CanMsg引用，SDK通过模板方法操作 | `SigHdl<float, uint16_t, float, float>(can_msg_ref, start_bit, len, factor, offset, ...)` |

### 7.2 SDK提供给Proxy的接口 (Proxy调用SDK)

| 函数/方法 | 所在文件 | 作用 |
|-----------|---------|------|
| `CanMsg::set_data(data, dlc, status)` | [msg_route_soc.h](msg_route_soc.h) | 缓存CAN帧原始字节 |
| `CanMsg::get_data(data_buf)` | [msg_route_soc.h](msg_route_soc.h) | 读取缓存的CAN帧 |
| `CanMsg::get_status()` | [msg_route_soc.h](msg_route_soc.h) | 获取CAN帧状态 |
| `SigHdl::signal_if_get(phy_t&)` | [sig_handle_soc.hpp](sig_handle_soc.hpp) | 位提取 + 物理值转换 |
| `SigHdl::signal_if_set(phy_t)` | [sig_handle_soc.hpp](sig_handle_soc.hpp) | 物理值 → 位域 → 写入缓存 |
| `SvcItemInfo::set_origin_data(id, data, len)` | [x_dom_someip_rt_com.h](x_dom_someip_rt_com.h) | 缓存SOME/IP原始数据 |
| `SvcItemInfo::get_usr_data_ext<>(offset, pData, Deserialize)` | [x_dom_someip_rt_com.h](x_dom_someip_rt_com.h) | 反序列化读取应用层数据 |
| `SvcItemInfo::set_usr_data_ext<>(offset, pData, Serialize)` | [x_dom_someip_rt_com.h](x_dom_someip_rt_com.h) | 序列化写入应用层数据 |
| `SvcItemInfo::fwd_svc_via_dcms(topic)` | [x_dom_someip_rt_com.h](x_dom_someip_rt_com.h) | 通过DCMS转发 |
| `SvcItemInfo::fwd_svc_via_someip()` | [x_dom_someip_rt_com.h](x_dom_someip_rt_com.h) | 通过SOME/IP发送到外部ECU |
| `x_dom_someip_rt_set(key, val, ...)` | [x_dom_someip_rt.cpp](x_dom_someip_rt.cpp) | 配置注入(KV对) |
| `dcms_mcu_topic_send(topic_id, data, len)` | [dcms_mcu_api.cpp](dcms_mcu_api.cpp) | DCMS发送 |
| `dcms_mcu_topic_setup_callback(topic_id, cbk, ...)` | [dcms_mcu_api.cpp](dcms_mcu_api.cpp) | DCMS回调注册 |

### 7.3 Proxy提供给SDK的数据/函数 (SDK访问Proxy)

| 数据/函数 | Proxy文件 | 被SDK访问方式 |
|-----------|----------|-------------|
| `msg_route_mcu_tx_rx_msgs_map` | x_dom_can_rt_gen_com/x_dom_can_rt_cfg.cpp | SDK直接引用全局变量 |
| `svc_route_recv_svc_info_map` | x_dom_someip_rt_gen_com/x_dom_someip_rt_cfg.cpp | 通过`g_x_dom_someip_config`指针 |
| `svc_route_send_svc_info_map` | x_dom_someip_rt_gen_com/x_dom_someip_rt_cfg.cpp | 通过`g_x_dom_someip_config`指针 |
| `svc_route_recv_svc_info_list` | x_dom_someip_rt_gen_com/x_dom_someip_rt_cfg.cpp | 通过`g_x_dom_someip_config`指针 |
| `sig_route_send_mcu_tx_rx_sigs()` | x_dom_can_rt_gen_com/x_dom_can_rt_cfg.cpp | extern符号, --whole-archive链接 |
| `sig_route_update_mcu_tx_rx_sigs_from_com()` | x_dom_can_rt_gen_uds/x_dom_can_rt_cfg.cpp | extern符号 |
| `SigIf_OnRecvData(id)` | x_dom_someip_rt_gen_uds/x_dom_someip_rt_sigif_get.cpp | 函数指针注入 (m_recv_func) |
| `svc_route_write_someip_init(cbk)` | x_dom_someip_rt_gen_uds/x_dom_someip_rt_cfg.cpp | extern符号 |
| `Deserialize() / Serialize()` | x_dom_someip_rt_gen_uds/x_dom_someip_rt_protocol_*.cpp | 模板参数传入SDK方法 |

### 7.4 关键设计: plat_bf_cdd.so 的复用

```
磁盘上: 一份 plat_bf_cdd.so
         ├── service_route.cpp  (comif路由引擎)
         ├── x_dom_someip_rt.cpp (初始化)
         ├── msg_route_soc.cpp   (CAN帧解析)
         ├── dcms_mcu_api.cpp    (DCMS封装)
         └── ...

运行时:
  COM进程加载 plat_bf_cdd.so ── g_x_dom_someip_config.com_service_enabled = true
                                   .svc_route_recv_svc_info_map_ptr → COM proxy (120+项)
                                   .svc_route_recv_svc_info_list_ptr → COM proxy (ad+uds)
                                   行为: 转发者

  UDS进程加载 plat_bf_cdd.so ── g_x_dom_someip_config.app_service_enabled = true
                                   .svc_route_recv_svc_info_map_ptr → UDS proxy (3项)
                                   .svc_route_recv_svc_info_list_ptr → nullptr
                                   行为: 消费者

同一个 svc_route_cycle_pub() 函数:
  COM进程执行: 遍历NodeRecvInfo列表 → 打包 → DCMS → UDS/AD
  UDS进程执行: com_service_enabled=false → 跳过转发逻辑
               app_service_enabled=true → 扫描send服务
```

同样，`sig_route_soc.cpp` 通过静态库方式分别编译进COM和UDS的proxy库，使用不同的 `XDOMCANRT_DJI_COM_SERVICE_EN` 宏值：

```cpp
// sig_route_soc.cpp — 编译进COM proxy库 (XDOMCANRT_DJI_COM_SERVICE_EN=1)
//                    同时也编译进UDS proxy库 (XDOMCANRT_DJI_COM_SERVICE_EN=0)

void sig_route_send_mcu_tx_rx_sigs_proc(void)
{
    #if XDOMCANRT_DJI_COM_SERVICE_EN  // COM=1, UDS=0
        // COM: 读CanMsg缓存 → SigHdl解析 → DCMS发送
        sig_route_send_mcu_tx_rx_sigs();  // extern → proxy定义
    #endif
}
```

---

## 八、有/无 comif 对比

### 8.1 Cdd_SomeIpTp.c 中的差异

```
                    │  有 comif (125个服务)              │  无 comif / danvince (777个服务)
────────────────────┼──────────────────────────────────┼──────────────────────────────────────
 回调函数体          │  调用 x_dom_someip_rt_rx_handle() │  空函数体 (只有 SOMEIP_DEBUG_INFO)
 编译条件            │  #ifdef X_DOM_SOMEIP_RT_COM_EN    │  无条件编译
 数据流向            │  COM → DCMS → UDS/AD 跨域转发    │  COM进程内 LdCom → Com 消费
 目标消费者          │  UDS进程 / AD进程                  │  COM进程自身
 SDK参与             │  service_route.cpp 路由引擎        │  无SDK参与
 Proxy参与           │  路由表 + SvcItemInfo + SigIf      │  无Proxy参与
 序列化/反序列化     │  SvcDataPackHdl / Serialize        │  AUTOSAR Com内部完成
 超时检测            │  SvcItemInfo timeout机制           │  无
 延迟                │  50ms周期 + DCMS传输               │  ~实时 (同进程函数调用)
```

### 8.2 架构对比图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         有 comif 的架构                                      │
│                                                                             │
│  外部ECU ──SOME/IP──→ COM进程                    UDS进程                     │
│                       ┌──────────────┐           ┌──────────────┐            │
│                       │ SomeIpTp/Sd  │           │              │            │
│                       │ LdCom/Com    │           │  Dcm/DoIP    │            │
│                       │ Cdd_SomeIpTp │           │  SoAd(13400) │            │
│                       │   ↓          │           │              │            │
│                       │ comif SDK ───DCMS────────→ comif SDK   │            │
│                       │ (转发者)     │           │ (消费者)     │            │
│                       │ proxy路由表  │           │ proxy SigIf  │            │
│                       └──────────────┘           └──────────────┘            │
│                                                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                         无 comif / danvince 的架构                           │
│                                                                             │
│  外部ECU ──SOME/IP──→ COM进程                    UDS进程                     │
│                       ┌──────────────┐           ┌──────────────┐            │
│                       │ SomeIpTp/Sd  │           │              │            │
│                       │ LdCom/Com    │           │  Dcm/DoIP    │            │
│                       │ Cdd_SomeIpTp │           │  SoAd(13400) │            │
│                       │   ↓          │           │              │            │
│                       │ 空回调→LdCom │           │ (不收SOME/IP)│            │
│                       │ →Com(进程内) │           │              │            │
│                       └──────────────┘           └──────────────┘            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 8.3 为什么需要两种路径共存

并不是所有SOME/IP服务都需要跨域共享：

- **comif路径 (125个)**: 服务于需要跨域共享的服务。例如 OTA下载命令（外部ECU → COM → UDS）、HMI状态信息（外部ECU → COM → AD）。这些服务的消费者不在COM进程内。
- **danvince路径 (777个)**: 服务于COM进程自身消费的信号。例如车身电源模式、门锁状态等只有COM域关心的信号。数据在COM进程内的AUTOSAR栈中直接消费，不需要IPC开销。

---

## 九、完整物理链路汇总

```
═══════════════════════════════════════════════════════════════════════════════
                        SOME/IP 收方向 (外部ECU → UDS)
═══════════════════════════════════════════════════════════════════════════════

外部ECU → Ethernet → COM(SoAd/SomeIpTp) → Cdd_SomeIpTp.c
  ┌─ 777个: 空函数体 → LdCom → Com → COM进程内消费 (danvince)
  └─ 125个: x_dom_someip_rt_rx_handle() → svc_route_rx_handle()
       → 查proxy路由表(svc_route_recv_svc_info_map)
       → SvcItemInfo::set_origin_data() 缓存
       → 50ms: svc_route_cycle_pub()
         → SvcDataPackHdl打包 → DCMS("/x_dom_someip_rt/com_to_app/uds_app")
           → UDS: svc_route_com_to_app_topic_cbk()
             → SvcDataUnPackHdl解包
             → 查UDS proxy map(svc_route_recv_svc_info_map, 3项)
             → SvcItemInfo::set_origin_data()
               → m_recv_func = SigIf_OnRecvData(id)
                 → SigIf_xxx_Get() → Deserialize() → APP回调

═══════════════════════════════════════════════════════════════════════════════
                        SOME/IP 发方向 (UDS → 外部ECU)
═══════════════════════════════════════════════════════════════════════════════

APP → SigIf_xxx_Set(pData)
  → SvcItemInfo::set_usr_data_ext<>(offset, pData, Serialize)
  → SvcItemInfo::fwd_svc_via_dcms("/x_dom_someip_rt/app_to_com")
    → DCMS → COM: svc_route_app_to_com_topic_cbk()
      → 查COM proxy send map → SvcItemInfo::fwd_svc_via_someip()
        → Cdd_SomeIpTp Tx → SomeIpTp → SoAd → Ethernet → 外部ECU

═══════════════════════════════════════════════════════════════════════════════
                        CAN 信号 (MCU → SOC COM → UDS)
═══════════════════════════════════════════════════════════════════════════════

CAN总线 → MCU(x_dom_can_rt) → DCMS("/x_dom_can_rt/mcu_tx_rx_msgs")
  → COM: msg_route_mcu_tx_rx_msgs_topic_cbk() → ThreadSafeQueue
  → 50ms: msg_route_mcu_tx_rx_can_msgs_proc()
    → 查 msg_route_mcu_tx_rx_msgs_map → CanMsg::set_data()
  → sig_route_send_mcu_tx_rx_sigs_proc()
    → sig_route_send_mcu_tx_rx_sigs() [proxy extern]
      → 遍历SigHdl → signal_if_get() → 物理值转换
      → DCMS → UDS: sig_route_mcu_rx_sigs_topic_cbk()
        → sig_route_update_mcu_tx_rx_sigs_from_com() [proxy extern]
          → hash校验 → memcpy → APP读

═══════════════════════════════════════════════════════════════════════════════
                        DoIP 诊断 (外部诊断仪 → UDS, 不经过COM)
═══════════════════════════════════════════════════════════════════════════════

外部诊断仪 → DoIP(端口13400) → UDS(SoAd/DoIP) → PduR → Dcm
  → DiagnosticService.c / DiagSwc_Callback.c / Dcm_Callout_Stubs.c
    → (可选) DiagProxy_Com.cpp → SigIf_xxx_Get/Set → comif跨域数据访问

═══════════════════════════════════════════════════════════════════════════════
```

---

## 十、关键文件索引

| 文件 | 位置 | 作用 |
|------|------|------|
| **Cdd_SomeIpTp.c** | `autosar_adapter/microsar_config_com/Appl/GenData/` | ★ 核心分发: e001_10有902个回调,125个接comif; p301有846个,114个接comif |
| **Cdd_SomeIpTp.h** | 同上 | SOME/IP类型定义, CDD_SOMEIPTP_XF_DATA_OFFSET |
| **service_route.cpp** | `plat-bf/dsar_app/cdd/x_dom_someip_rt/` | ★ comif SDK: SOME/IP路由引擎 (收发/打包/解包/转发) |
| **x_dom_someip_rt.cpp** | 同上 | comif SDK: 初始化入口, x_dom_someip_rt_rx_handle() |
| **x_dom_someip_rt_com.h** | `plat-bf/dsar_app/include/.../comif/` | SvcItemInfo, SvcDataPackHdl/SvcDataUnPackHdl, NodeRecvInfo |
| **x_dom_someip_rt_com_cfg.h** | `plat-bf/dsar_app/cdd/x_dom_someip_rt/` | ★ 全局配置结构体 g_x_dom_someip_config (L58-98) |
| **msg_route_soc.cpp** | `plat-bf/dsar_app/cdd/x_dom_can_rt/` | comif SDK: CAN帧解析引擎, ThreadSafeQueue |
| **msg_route_soc.h** | `plat-bf/dsar_app/include/.../cdd/` | CanMsg类: set_data/get_data/超时检测 |
| **sig_route_soc.cpp** | `plat-bf/dsar_app/cdd/x_dom_can_rt/` | comif SDK: CAN信号路由 (编译时XDOMCANRT_DJI_COM_SERVICE_EN区分COM/UDS) |
| **sig_handle_soc.hpp** | `plat-bf/dsar_app/include/.../cdd/` | ★ SigHdl模板: 位提取 + 物理值转换 |
| **dcms_mcu_api.cpp** | `plat-bf/dsar_app/cdd/dcms_adapt/` | DCMS API封装 |
| **x_dom_someip_rt_cfg.cpp** | `proxy/x_dom_someip_rt_gen_com/` | ★ COM proxy: 120+SvcItemInfo, NodeRecvInfo, 路由表, x_dom_someip_rt_set注入 |
| **x_dom_someip_rt_cfg.cpp** | `proxy/x_dom_someip_rt_gen_uds/` | ★ UDS proxy: 3+4个OTA SvcItemInfo, svc_route_write_someip_init |
| **x_dom_someip_rt_sigif_get.cpp** | `proxy/x_dom_someip_rt_gen_uds/` | UDS proxy: SigIf_OnRecvData() + SigIf_xxx_Get() |
| **x_dom_someip_rt_sigif_set.cpp** | `proxy/x_dom_someip_rt_gen_uds/` | UDS proxy: SigIf_xxx_Set() → fwd_svc_via_dcms() |
| **x_dom_someip_rt_protocol_*.cpp** | `proxy/x_dom_someip_rt_gen_uds/` | UDS proxy: Serialize/Deserialize |
| **x_dom_can_rt_cfg.cpp** | `proxy/x_dom_can_rt_gen_com/` | ★ COM CAN proxy: 37CanMsg + 50+SigHdl + 转发函数 |
| **x_dom_can_rt_cfg.cpp** | `proxy/x_dom_can_rt_gen_uds/` | UDS CAN proxy (e001_10几乎为空): hash校验+memcpy |
| **DiagProxy_Com.cpp** | `DiagProxy/` | ★ C→C++桥接: UDS诊断代码调comif SigIf |
| **microsar_config_com.cmake** | `autosar_adapter/microsar_config_com/` | COM BSW模块 + X_DOM_SOMEIP_RT_COM_EN定义 (L41) |
| **microsar_config_uds.cmake** | `oem_feature/autosar_adapter/microsar_config_uds_v4/` | ★ UDS BSW模块 (SomeIpTp/Sd/LdCom/Com被注释) |
| **consys_bf.cmake** | `dsar_app/consys/` | 各域.so链接配置, --whole-archive |
| **fawhq_e001_10_config.cmake** | `product/faw/fawhq_e001_10/` | Product config: CAN_UDS_V4=OFF, SOMEIP_RT_COM=ON, SOMEIP_RT_UDS_V4=ON |

---

## 十一、FAQ

### Q1: comif 和 DCMS 是什么关系？
**comif 在 DCMS 之上。** DCMS是唯一的IPC传输通道（BSP层 `dcos_dcms`），comif是DCMS之上的路由+序列化+缓存框架。comif内部大量调用 `dcms_mcu_topic_send()` 和 `dcms_mcu_topic_setup_callback()`。两者是分层关系，不是并行关系。

### Q2: 同进程内为什么还要用DCMS？
**同进程内不需要。** AUTOSAR BSW调用comif SDK是直接函数调用（`Cdd_SomeIpTp.c` → `x_dom_someip_rt_rx_handle()`），不经过DCMS。DCMS只在跨核（SOC↔MCU）和跨进程（COM↔UDS、COM↔AD）时使用。

### Q3: 为什么UDS不直接收SOME/IP，一定要经过COM？
**UDS没有SomeIpTp协议栈。** UDS的AUTOSAR BSW只有Dcm/DoIP/PduR/SoAd(DoIP端口13400)，没有SomeIpTp/Sd/LdCom/Com这四个关键模块。操作系统将SOME/IP端口的数据路由到COM进程的SoAd，UDS进程根本收不到。

### Q4: 同一个SDK代码在COM和UDS中行为一样吗？
**不一样。** `service_route.cpp` 编译进 `plat_bf_cdd.so`，COM和UDS加载同一份.so，但通过 `g_x_dom_someip_config` 运行时的不同配置走不同分支。`sig_route_soc.cpp` 则以不同 `XDOMCANRT_DJI_COM_SERVICE_EN` 宏值分别编译进COM和UDS的静态库。

### Q5: 846/902个SOME/IP服务中为什么只有114/125个走comif？
只有需要跨域共享的服务才需要comif转发。以 `Veh_VM_PowerModeInfo` 服务为例：`PMM_NotifyONChgPowerMode` 需要通知UDS/AD，走comif；`PMM_NotifyONChgIGSt` 只有COM域自己用，走danvince在COM进程内的AUTOSAR栈消费。777个COM内部服务不需要IPC开销。

### Q6: UDS域有没有可能直接解析SOME/IP数据？
**理论上可以，但架构选择不这么做。** UDS链接了 `plat_bf_cdd.so`（包含service_route.cpp）、UDS proxy（包含SvcItemInfo/SigIf）、以及 `dcos_dcms`。它有能力反序列化SOME/IP数据。但数据来源只能是DCMS——因为物理SOME/IP报文只能到达COM进程的SoAd。UDS通过DCMS接收COM打包好的SvcDataPackHdl二进制数据，再用同样的Deserialize函数解析。

### Q7: --whole-archive 的作用是什么？
确保proxy静态库中的所有符号都被链接进.so，即使SDK代码中没有直接引用。这是实现SDK↔Proxy解耦的关键——SDK通过 `extern` 声明引用proxy定义的符号，linker通过 `--whole-archive` 保证这些符号被导出。

### Q8: DiagProxy_Com.cpp 为什么需要存在？
因为UDS的AUTOSAR诊断代码（Dcm回调、DiagnosticService.c等）是C语言写的（danvince路径），而comif的SvcItemInfo/SigIf是C++模板类。DiagProxy提供了 `extern "C"` 的函数封装，让C代码能间接调用C++的comif接口，实现诊断代码读取跨域SOME/IP信号的能力。

---

## 十二、comif 工具链

```
JSON 配置文件 (车型信号定义)
  ├── x_dom_someip_rt_fawhq_e001_10.json    SOME/IP跨域路由JSON (84014行)
  └── x_dom_can_rt_fawhq_e001_10.json       CAN信号跨域路由JSON
       │
       ▼
代码生成工具 (v0.17)
  │
  ├─→ SOC COM proxy: proxy/x_dom_someip_rt_gen_com/
  │     x_dom_someip_rt_cfg.cpp    SvcItemInfo + 路由表 + NodeRecvInfo
  │     x_dom_someip_rt_sigif_get/set.cpp
  │     x_dom_someip_rt_protocol_*.cpp
  │
  ├─→ SOC UDS proxy: proxy/x_dom_someip_rt_gen_uds/
  │     (同上结构, 仅OTA服务)
  │
  ├─→ SOC COM CAN proxy: proxy/x_dom_can_rt_gen_com/
  │     x_dom_can_rt_cfg.cpp       CanMsg + SigHdl + 转发函数
  │     x_dom_can_rt_sigif.cpp     SigIf_Get/Set
  │
  ├─→ SOC UDS CAN proxy: proxy/x_dom_can_rt_gen_uds/
  │     (fawhq_e001_10中几乎为空, CAN_UDS_V4=OFF)
  │
  ├─→ Cdd_SomeIpTp.c/h              AUTOSAR SOME/IP分发
  │     #ifdef X_DOM_SOMEIP_RT_COM_EN
  │       x_dom_someip_rt_rx_handle(SERVER_ID_..., payload, len)
  │     #endif
  │
  └─→ MCU x_dom_can_rt_config/      MCU侧CAN信号路由
        x_dom_can_rt_cfg.c/h        路由表 + CAN帧缓存 + 信号写表
        x_dom_can_rt_sigif.c/h      SigIf_Get/Set
```

---

## 十三、与参考文档的关系

本文档作为 [03_DSAR_COMMUNICATION_ANALYSIS.md](03_DSAR_COMMUNICATION_ANALYSIS.md) 和 [04_DSAR_XDOM_COMMUNICATION_CHAIN.md](04_DSAR_XDOM_COMMUNICATION_CHAIN.md) 的深化补充：

- **03文档**: 详述SOC侧comif vs danvince两条路径、SDK↔Proxy设计模式、各域通信流程总览
- **04文档**: 聚焦JSON配置→代码生成→端到端四路数据流、MCU侧完整调用链、DCMS命名空间
- **本文档**: 聚焦 **COM↔UDS两个域之间的通信架构**，特别深化了：
  - COM和UDS两域的**完整库依赖树**（BSP→SDK→Proxy→AUTOSAR→OEM五层）
  - **Cdd_SomeIpTp.c的分发机制**和X_DOM_SOMEIP_RT_COM_EN编译开关
  - **SOME/IP收/发方向的函数级完整调用链**（含COM proxy和UDS proxy的数据结构）
  - **CAN信号从MCU到UDS的函数级调用链**
  - **DoIP诊断路径**及其与comif的交汇点（DiagProxy_Com.cpp桥接）
  - **SDK↔Proxy双向函数调用关系表**
  - **同一份plat_bf_cdd.so的配置驱动差异化机制**
  - **有/无comif的架构级对比**
