# DSAR APP侧通信架构全解析 —— CAN与SOME/IP信号处理链路

---

## 一、先决知识：芯片架构与进程模型

### 1.1 SA8650 芯片拓扑

```
┌──────────────────────────────────────────────────────────────────┐
│                        SA8650 芯片                                │
│                                                                  │
│  ┌────────────────────────────┐  ┌──────────────────────────────┐│
│  │ SOC 侧 (应用处理器)         │  │ MCU 侧 (R52 安全岛)           ││
│  │ OS: QNX 7.1 / Linux        │  │ OS: FreeRTOS / SafeRTOS      ││
│  │ 编译: CMake → .so + ELF    │  │ 编译: Makefile → .bin 固件    ││
│  │                            │  │                              ││
│  │ 三个独立进程:               │  │ CAN Controller ← 物理CAN总线   ││
│  │  dji_com_service  (COM域)  │  │                              ││
│  │  dji_uds_service  (UDS域)  │  │ x_dom_can_rt (FW侧comif)     ││
│  │  dji_application  (AD域)   │  │ Com_SendSignal() → AUTOSAR   ││
│  │                            │  │                              ││
│  │ 以太网控制器 ← SOME/IP/DoIP │  │                              ││
│  └────────────────────────────┘  └──────────────────────────────┘│
│                                                                  │
│  跨核通信: DCMS (IPC pub/sub)                                     │
│  跨进程通信: DCMS (同一SOC上的进程间IPC)                            │
└──────────────────────────────────────────────────────────────────┘
```

### 1.2 三个域的职责

| 域 | 进程名 | .so 文件 | 核心职责 |
|---|---|---|---|
| COM | dji_com_service | libdji_com_service.so | SOME/IP协议栈宿主、CAN帧接入、跨域信号路由 |
| UDS | dji_uds_service | libuds_v4.so | UDS诊断(DoIP)、OTA管理、从COM接收跨域信号 |
| AD | dji_application | libdji_application.so | 自动驾驶感知/规划/控制算法 |

### 1.3 关键认知：为什么 UDS 不能直接收 SOME/IP

**UDS 域没有 SomeIpTp 协议栈。** 证据来自 `microsar_config_uds.cmake`：

```cmake
# UDS AUTOSAR BSW 模块:
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST Dcm)       # ✔ 诊断管理
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST DoIP)       # ✔ 诊断 over IP
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST PduR)       # ✔ PDU路由
list(APPEND DIAG_AUTOSAR_BSW_COMPONENT_LIST SoAd)       # ✔ Socket适配 (for DoIP)

# 以下全部被注释掉:
# list(APPEND ... Com)        # ✗ 没有
# list(APPEND ... LdCom)      # ✗ 没有
# list(APPEND ... Sd)         # ✗ 没有 (服务发现)
# list(APPEND ... SomeIpTp)   # ✗ 没有 (SOME/IP协议栈)
```

而 COM 域有完整的 SOME/IP 协议栈：

```cmake
# COM AUTOSAR BSW 模块:
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST Com)         # ✔
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST Sd)          # ✔ 服务发现
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST LdCom)       # ✔ 大数据COM
list(APPEND COM_AUTOSAR_BSW_COMPONENT_LIST SomeIpTp)    # ✔ SOME/IP协议栈
```

**物理现实**：外部 ECU 的 SOME/IP UDP 报文到达 SOC 以太网控制器后，由操作系统根据端口号路由到 COM 进程的 SoAd。UDS 进程的 SoAd 只绑定 DoIP 端口（13400），根本收不到 SOME/IP 报文。

---

## 二、两条通信路径总览：danvince vs comif

### 2.1 概念澄清

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                  │
│  DCMS = 进程间通信传输层 (IPC pub/sub)                            │
│    来源: Qualcomm DCOS (dcos_dcms 库)                            │
│    平台仓封装: dcms_adapt (dcms_mcu_api.cpp)                     │
│    作用: 把二进制数据从进程A搬到进程B（或SOC↔MCU跨核）             │
│    不管: 数据格式、路由逻辑、序列化                                │
│                                                                  │
│  comif = DCMS之上的路由框架                                       │
│    来源: 平台仓 SDK (service_route.cpp, msg_route_soc.cpp 等)    │
│    作用: 服务/信号路由配置、序列化/反序列化、超时检测、仲裁         │
│    配置: JSON → 工具自动生成 proxy 代码                            │
│                                                                  │
│  danvince = AUTOSAR Vector DaVinci 原生路径                       │
│    AUTOSAR BSW (SomeIpTp → LdCom → Com) 在进程内直接消费          │
│    不经过 comif，不跨域转发                                        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

**comif 不是 DCMS 的替代，comif 在 DCMS 之上。** 两者是分层关系，不是并行关系。

### 2.2 两条路径在 Cdd_SomeIpTp.c 中的体现

[Cdd_SomeIpTp.c](Cdd_SomeIpTp.c) 是理解整个系统最关键的文件。每个 SOME/IP 服务对应一个 `LdComRxIndication` 回调函数，共 846 个。其中只有 114 个走 comif 路径。

```cpp
// 【comif 路径: 跨域转发】 — 114个服务
FUNC(void, RTE_CODE) LdComRxIndication_Client__...__Event_RX(PduInfoPtr)
{
    uint8 *Data = (uint8 *)PduInfoPtr->SduDataPtr;
    SOMEIP_DEBUG_INFO("...");
    #ifdef X_DOM_SOMEIP_RT_COM_EN
        // → 进 comif SDK → 路由判决 → DCMS → UDS/AD
        x_dom_someip_rt_rx_handle(SERVER_ID_..., 
            Data + CDD_SOMEIPTP_XF_DATA_OFFSET, 
            PduInfoPtr->SduLength - CDD_SOMEIPTP_XF_DATA_OFFSET);
    #endif
}

// 【danvince 路径: COM进程内消费】 — 732个服务
FUNC(void, RTE_CODE) LdComRxIndication_Client__...__FieldNotify_RX(PduInfoPtr)
{
    uint8 *Data = (uint8 *)PduInfoPtr->SduDataPtr;
    SOMEIP_DEBUG_INFO("...");
    // 函数体为空! 数据由 AUTOSAR LdCom → Com 在 COM 进程内消费
}
```

---

## 三、平台仓 SDK 与车型适配仓 Proxy 的分工

### 3.1 目录对应关系

```
平台仓 (plat-bf)                               车型适配仓 (proxy)
─────────────────────────────────────         ─────────────────────────────────
cdd/x_dom_can_rt/                             proxy/x_dom_can_rt_gen_com/
  msg_route_soc.cpp    ← 运行时引擎              x_dom_can_rt_cfg.cpp    ← 路由表+信号定义
  sig_route_soc.cpp    ← 信号路由                x_dom_can_rt_sigif.cpp  ← 信号get/set
  x_dom_can_rt_soc.cpp ← 初始化入口
  p2p_parser.cpp       ← P2P解析               proxy/x_dom_can_rt_gen_uds/
                                                (同上结构, fawhq_e001_10几乎为空)

cdd/x_dom_someip_rt/                           proxy/x_dom_someip_rt_gen_com/
  service_route.cpp    ← SOME/IP路由引擎          x_dom_someip_rt_cfg.cpp    ← SvcItemInfo+路由表
  x_dom_someip_rt.cpp  ← 初始化入口               x_dom_someip_rt_sigif_get.cpp ← Get函数
  x_dom_someip_rt_com_cfg.h ← 全局配置结构体      x_dom_someip_rt_sigif_set.cpp ← Set函数
                                                  x_dom_someip_rt_protocol_*.cpp ← 序列化

cdd/dcms_adapt/                                proxy/x_dom_someip_rt_gen_uds/
  dcms_mcu_api.cpp     ← DCMS API封装            (同上结构, UDS域用)
```

### 3.2 SDK ↔ Proxy 调用关系总结

```
┌────────────────────────────────────────────────────────────────┐
│                                                                 │
│  SDK 提供给 Proxy 的接口 (Proxy 调 SDK):                         │
│                                                                 │
│  CanMsg::set_data()          缓存CAN帧原始字节                   │
│  CanMsg::get_data()          读取缓存的CAN帧                     │
│  SigHdl::signal_if_get()     位提取 + 物理值转换 (Intel/Motorola) │
│  SvcItemInfo::set_origin_data()  缓存SOME/IP原始数据             │
│  SvcItemInfo::get_usr_data_ext<>()  反序列化读取                 │
│  SvcItemInfo::set_usr_data_ext<>()  序列化写入                   │
│  SvcItemInfo::fwd_svc_via_dcms()    通过DCMS转发                 │
│  SvcItemInfo::fwd_svc_via_someip()  通过SOME/IP发送              │
│  x_dom_someip_rt_set()           配置注入                       │
│  dcms_mcu_topic_send()           DCMS发送                       │
│  dcms_mcu_topic_setup_callback() DCMS回调注册                   │
│                                                                 │
│  Proxy 提供给 SDK 的数据 (SDK 访问 Proxy):                        │
│                                                                 │
│  msg_route_mcu_tx_rx_msgs_map       CAN帧 → CanMsg* 查找表       │
│  svc_route_recv_svc_info_map_ptr    SOME/IP server_id → SvcItemInfo* │
│  svc_route_send_svc_info_map_ptr    发送服务查找表               │
│  svc_route_recv_svc_info_list_ptr   跨域转发 NodeRecvInfo 列表   │
│  SigIf_OnRecvData()                 数据到达回调 (函数指针注入)   │
│  sig_route_send_mcu_tx_rx_sigs()    CAN信号读取+转发 (extern声明) │
│  sig_route_update_mcu_tx_rx_sigs_from_com()  信号缓存 (extern)   │
│  svc_route_write_someip_init()      DCMS回调注册 (extern)       │
│  Deserialize() / Serialize()        序列化/反序列化函数          │
│                                                                 │
└────────────────────────────────────────────────────────────────┘
```

**关键设计模式**：
1. **指针注入**: `g_x_dom_someip_config` 全局结构体，Proxy 通过 `x_dom_someip_rt_set("key", ptr)` 注入数据指针
2. **外部符号**: `extern` 声明的函数，实际定义在 Proxy 中，通过 `--whole-archive` 链接解析
3. **回调注册**: SvcItemInfo 构造时接收 `SigIf_OnRecvData` 函数指针，SDK 在数据到达时回调
4. **模板 + 引用绑定**: `SigHdl<phy_t, raw_t, factor_t, offset_t>` 由 Proxy 实例化并绑定 CanMsg 引用

---

## 四、COM 域 (dji_com_service) 通信详细分析

### 4.1 编译与链接

```
libdji_com_service.so
├── plat_bf_cdd.so (SHARED, 包含 service_route.cpp, dcms_adapt 等)
├── x_dom_someip_rt_gen_lib_com_{PRODUCT}.a  (--whole-archive)
│   ├── proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_cfg.cpp
│   ├── proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_sigif_get.cpp
│   ├── proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_sigif_set.cpp
│   └── proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_protocol_*.cpp
├── x_dom_can_rt_gen_lib_com_{PRODUCT}.a  (--whole-archive)
│   ├── proxy/x_dom_can_rt_gen_com/x_dom_can_rt_cfg.cpp
│   ├── proxy/x_dom_can_rt_gen_com/x_dom_can_rt_sigif.cpp
│   ├── SDK: msg_route_soc.cpp (XDOMCANRT_DJI_COM_SERVICE_EN=1)
│   ├── SDK: sig_route_soc.cpp (XDOMCANRT_DJI_COM_SERVICE_EN=1)
│   ├── SDK: x_dom_can_rt_soc.cpp
│   └── SDK: p2p_parser.cpp
├── autosar_com (AUTOSAR BSW: SomeIpTp, Sd, LdCom, Com, PduR, SoAd...)
├── dcos_dcms (BSP层DCMS实现)
└── mini_dcos
```

### 4.2 接口1: CAN 帧从 MCU 接入 (MCU → COM)

这是一条**跨核链路**，CAN 物理总线数据由 MCU 接收后通过 DCMS 发到 SOC。

```
MCU (R52)
  CAN Controller 接收物理CAN帧
    → x_dom_can_rt (FW侧comif)
    → dcms_mcu_topic_send("/x_dom_can_rt/mcu_tx_rx_msgs")
         │
         │  DCMS 跨核 IPC
         ▼
COM 进程
  【SDK】msg_route_mcu_tx_rx_msgs_topic_cbk(data, len)
         │ [msg_route_soc.cpp]
         │ 将 can_msg_base_with_status_t 推入 ThreadSafeQueue
         ▼
  【SDK】msg_route_main_function()  ← 50ms定时任务
         └→ msg_route_mcu_tx_rx_can_msgs_proc()
              │ [msg_route_soc.cpp:93]
              │ 从队列取出 CAN 帧
              │ 构造查找 key: {channel, can_id}
              │
              │ 【SDK → Proxy】查找路由表
              │ auto it = msg_route_mcu_tx_rx_msgs_map.find(key);
              │           ↑ proxy/x_dom_can_rt_cfg.cpp 定义的全局 map
              │
              │ 【SDK → SDK】缓存原始字节
              │ can_msg_ptr->set_data(data, dlc, status);
              │           ↑ [msg_route_soc.h] CanMsg 类方法
              ▼
         37个 CanMsg 对象的数据被更新
         (4个MCU_TX + 33个MCU_RX)
              │
              │ 【SDK → Proxy】周期读取信号
              ▼
  【SDK】sig_route_send_mcu_tx_rx_sigs_proc()
         │ [sig_route_soc.cpp:25] XDOMCANRT_DJI_COM_SERVICE_EN=1
         └→ sig_route_send_mcu_tx_rx_sigs()
              │ ↑ proxy/x_dom_can_rt_cfg.cpp (extern 声明)
              │
              ├→ sig_route_send_tx_rx_sigs_dji_uds_service()
              │   (fawhq_e001_10 中为空体)
              │
              └→ sig_route_send_tx_rx_sigs_dji_application()
                    │ 遍历所有 SigHdl 对象
                    │
                    │ 【Proxy → SDK】读信号物理值
                    │ VDC_DecelerationReq_sighdl.signal_if_get(phy);
                    │   ↑ SigHdl<float, uint16_t, float, float>
                    │
                    │ SDK 内部:
                    │   1. CanMsg::get_data() → 获取缓存CAN字节
                    │   2. extract_bits(start_bit=64, len=10, Intel格式)
                    │   3. phy = raw * 0.01 + (-10.21)
                    │
                    │ 打包成结构体
                    │ dcms_mcu_topic_send() → UDS/AD
                    ▼
              DCMS → UDS 进程 / AD 进程
```

**CAN 信号解析链路（以 VDC 减速度请求为例）**：

```
CAN帧 (id=0x118, dlc=32, cycle=20ms, CANFD)
  ↓ [存入 CanMsg 缓存]
data_buf[8..9] 包含 64bit起共10bit的原始值
  ↓ [SigHdl::signal_if_get]
raw = extract_bits(data_buf, start_bit=64, len=10, is_big_endian=false)
    = 0x200 (假设值)
  ↓ [物理值转换]
phy = (float)0x200 * (float)0.01 + (float)(-10.21)
    = 512 * 0.01 + (-10.21)
    = -5.09 m/s²
  ↓ [APP调用]
SigIf_Get_VDC_DecelerationReq_AEB_VDC_1_Get(&decel);
```

### 4.3 接口2: SOME/IP 从外部 ECU 接入 → comif 转发 (ECU → COM → UDS/AD)

```
外部ECU
  │ SOME/IP over Ethernet UDP
  ▼
COM 进程: SoAd (绑定 SOME/IP 端口)
  → SomeIpTp 解析 SOME/IP 协议头
    (Service ID, Method ID, Client ID, Session ID, Message Type...)
  → Cdd_SomeIpTp.c 按 Service ID + Method ID 分发到对应回调函数
       │
       ├─ 732个服务: 空函数体 → LdCom → Com → COM进程内 AUTOSAR 消费
       │              (danvince 路径, 不进 comif)
       │
       └─ 114个服务: #ifdef X_DOM_SOMEIP_RT_COM_EN
                       x_dom_someip_rt_rx_handle(SERVER_ID, payload, len)
                       │
                       ▼
              【SDK】x_dom_someip_rt_rx_handle()
                       │ [x_dom_someip_rt.cpp]
                       └→ svc_route_rx_handle(id, data, len)
                            │ [service_route.cpp:80]
                            │ g_x_dom_someip_config.com_service_enabled == true
                            │
                            │ 【SDK → Proxy】查找 SvcItemInfo
                            │ auto it = g_x_dom_someip_config
                            │     .svc_route_recv_svc_info_map_ptr->find(id);
                            │   ↑ 指向 proxy 的 svc_route_recv_svc_info_map
                            │
                            │ 【SDK → SDK】缓存原始数据
                            │ item_ptr->set_origin_data(id, data, len);
                            │   ↑ [x_dom_someip_rt_com.h] SvcItemInfo 类方法
                            │   内部: 缓存data + 更新m_recv_count
                            │
                            │ 如果是立即发送型(immediate):
                            │   svc_route_immediately_pub(id, data, len, count)
                            │
                            ▼
              【SDK】svc_route_cycle_pub()  ← 50ms 定时
                       │ [service_route.cpp:561]
                       │ 遍历 svc_route_recv_svc_info_list_ptr
                       │   ↑ 指向 proxy 的 vector<NodeRecvInfo*>
                       │
                       │ ├─ NodeRecvInfo for uds_app:
                       │ │   svc_list: {OTA_DownloadCmd_RX, OTA_DownloadStatus_RX, OTA_Script_RX}
                       │ │   topic_name: "/x_dom_someip_rt/com_to_app/uds_app"
                       │ │
                       │ └─ NodeRecvInfo for ad_app:
                       │     svc_list: {120+ AD服务ID}
                       │     topic_name: "/x_dom_someip_rt/com_to_app/ad_app"
                       │
                       └→ check_and_send_svc_list_via_dcms_on_timeout()
                            │ 遍历 svc_list, 每个 SvcItemInfo:
                            │   检查是否超时 + 有更新
                            │   打包: SvcDataPackHdl::add_data(id, data, len, count)
                            │
                            │ 【SDK → DCMS】发送
                            │ dcms_mcu_topic_send(topic_id, packed_data, size)
                            ▼
                    DCMS → UDS进程 / AD进程
```

**COM proxy 中的关键数据结构**：

```cpp
// proxy/x_dom_someip_rt_gen_com/x_dom_someip_rt_cfg.cpp

// 1. 定义所有跨域服务的 SvcItemInfo (包含 AD + UDS 的 recv 端)
/*uds_app*/ SvcItemInfo someip_recv_svc_info_provided_method_PS_FM_OTASelfDL_..._RX(
    SERVER_ID_..., "PS_FM_OTASelfDL_Mgmt_HAD_JO__...", 
    100,              // timeout in ms
    SigIf_OnRecvData, // 回调 (COM侧不使用, 但作为占位符传入)
    0);

// 2. 全局查找表
const map<uint32_t, SvcItemInfo*> svc_route_recv_svc_info_map = {
    /*uds_app*/ {SERVER_ID_OTA_DownloadCmd_RX,   &someip_recv_svc_info_..._RX},
    /*uds_app*/ {SERVER_ID_OTA_DownloadStatus_RX, &someip_recv_svc_info_..._RX},
    /*uds_app*/ {SERVER_ID_OTA_DownloadScript_RX, &someip_recv_svc_info_..._RX},
    /*ad_app*/  {SERVER_ID_HMI_xxx_RX,           &someip_recv_svc_info_..._RX},
    // ... 共120+项
};

// 3. UDS 域应接收的服务列表
static NodeRecvInfo svc_route_recv_svc_info_uds_app = {
    { SERVER_ID_OTA_DownloadCmd_RX, 
      SERVER_ID_OTA_DownloadStatus_RX,
      SERVER_ID_OTA_DownloadScript_RX },
    "/x_dom_someip_rt/com_to_app/uds_app"
};

// 4. 转发列表
vector<NodeRecvInfo*> svc_route_recv_svc_info_list = {
    &svc_route_recv_svc_info_ad_app,
    &svc_route_recv_svc_info_uds_app,
};

// 5. 配置注入
x_dom_someip_rt_set("com_service_en", (void*)(true), ...);
x_dom_someip_rt_set("svc_route_recv_svc_info_list", (void*)(&list), ...);
```

### 4.4 接口3: COM 接收 UDS/AD 发来的 SOME/IP → 转发到外部 ECU

```
UDS/AD 进程
  │ SigIf_xxx_Set(pData) → Serialize() → fwd_svc_via_dcms()
  │ dcms_mcu_topic_send("/x_dom_someip_rt/app_to_com")
  ▼
COM 进程
  【SDK】svc_route_app_to_com_topic_cbk(data, len)
         │ [service_route.cpp:107]
         │ SvcDataUnPackHdl 解包
         │
         │ 【SDK → Proxy】查找 send map
         │ auto it = g_x_dom_someip_config.svc_route_send_svc_info_map_ptr->find(id);
         │
         │ 【SDK → SDK】缓存数据
         │ item_ptr->set_data(id, data, len);
         │
         │ 【SDK → SDK】通过 SOME/IP 发送到外部 ECU
         │ item_ptr->fwd_svc_via_someip();
         │   → Cdd_SomeIpTp Tx path → SomeIpTp → SoAd → Ethernet
         ▼
  外部ECU
```

---

## 五、UDS 域 (dji_uds_service) 通信详细分析

### 5.1 编译与链接

```
libuds_v4.so
├── plat_bf_cdd.so (SHARED, 与COM同一份.so，但不同进程加载)
├── x_dom_someip_rt_gen_lib_uds_{PRODUCT}_v4.a  (--whole-archive)
│   ├── proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_cfg.cpp
│   ├── proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_sigif_get.cpp
│   ├── proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_sigif_set.cpp
│   └── proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_protocol_0.cpp
├── x_dom_can_rt_gen_lib_uds_{PRODUCT}_v4.a  (--whole-archive)
│   ├── proxy/x_dom_can_rt_gen_uds/x_dom_can_rt_cfg.cpp
│   ├── proxy/x_dom_can_rt_gen_uds/x_dom_can_rt_sigif.cpp
│   ├── SDK: sig_route_soc.cpp (XDOMCANRT_DJI_COM_SERVICE_EN=0)  ← 不同配置!
│   └── SDK: x_dom_can_rt_soc.cpp
│   (注意: 没有 msg_route_soc.cpp，UDS 不直接收 CAN 帧)
├── autosar_uds_v4 (Dcm, Dem, DoIP, PduR, SoAd(DoIP), BswM, ComM, Fee, Fls, NvM)
│   (注意: 没有 SomeIpTp, Sd, LdCom, Com)
├── diagproxy_ge_{PRODUCT}.a
│   └── DiagProxy/DiagProxy_Com.cpp (C→C++ 桥接层)
├── dcos_dcms (BSP层DCMS实现)
└── udsonip (UDS on IP 协议栈)
```

### 5.2 接口1: DoIP 诊断路径 (danvince, 不进 comif)

**这条路径不经过 COM。** 外部诊断仪通过 DoIP（UDP 13400 / TCP 13400）直接与 UDS 进程通信。

```
外部诊断仪
  │ DoIP (ISO 13400)
  ▼
UDS 进程: SoAd (DoIP 端口)
  → PduR → Dcm (诊断通信管理)
       │
       ├→ DiagnosticService.c          (车型适配: UDS 服务实现)
       │    SysModeMap, UdsDealFileThread, 文件CRC, reset处理
       │
       ├→ Dcm_Callout_Stubs.c          (Dcm 回调模板)
       │    会话切换、安全级别切换、0x38 文件传输
       │
       ├→ DiagSwc_Callback.c           (DID 读/写回调，自动生成)
       │    DiagDid_0xF187_FAW_Part_Number_ReadData()
       │    DiagDid_0xF190_VIN_ReadData/WriteData()
       │    → rte_config.DID_Read_ScvGRop / DID_Write_ScvGrop
       │
       └→ DiagProxy_Com.cpp            (C → C++ 桥接)
            │ 可调用 proxy 的 SigIf_xxx_Get/Set
            │ 读取 COM 域通过 comif 转发的跨域信号
            │
            │ 例如: DiagProxy_GetOTARRSelfDownloadCmdSig()
            │       → SigIf_HAD_OTA__HAD_OTASelfDL_RRSelfDownloadCmd_Get()
            ▼
```

### 5.3 接口2: comif 接收 COM 转发的 SOME/IP 数据

这是 UDS 获取 SOME/IP 跨域服务数据的**唯一途径**。

```
COM 进程
  │ DCMS topic: "/x_dom_someip_rt/com_to_app/uds_app"
  ▼
UDS 进程
  【SDK】svc_route_com_to_app_topic_cbk(data, len)
         │ [service_route.cpp:147]
         │ g_x_dom_someip_config.app_service_enabled == true  → UDS走这个分支
         │
         │ SvcDataUnPackHdl DataUnPackHdl(data, len);
         │ while(1):
         │   DataUnPackHdl.get_next_data(item_id, item_data, item_len, recv_count);
         │   │ ↑ 逐条解包 (一个 DCMS 消息可包含多个服务)
         │   │
         │   │ 【SDK → Proxy】查找
         │   │ auto it = g_x_dom_someip_config
         │   │     .svc_route_recv_svc_info_map_ptr->find(item_id);
         │   │   ↑ 指向 UDS proxy 的 svc_route_recv_svc_info_map (3项)
         │   │
         │   │ 【SDK → SDK】写入数据
         │   │ item_ptr->set_com_recv_count(com_recv_count);
         │   │ item_ptr->set_origin_data(item_id, item_data, item_len);
         │   │   ↑ 内部: 缓存原始数据 + 更新计数 + m_recv_func(id) 回调
         │   ▼
         │
         └→ m_recv_func(id)  ← 函数指针
               │
               ▼
  【Proxy】SigIf_OnRecvData(id)
           │ [x_dom_someip_rt_sigif_get.cpp:31]
           │
           │ switch(id):
           │   case SERVER_ID_OTA_DownloadCmd_RX:
           │     if (SigIf_xxx_Cb != nullptr) {
           │         cpp_PS_SelfDLCmd_strt data;
           │         // 【Proxy → SDK】反序列化
           │         SigIf_xxx_Get(&data);
           │         // → SvcItemInfo::get_usr_data_ext<>(offset, pData, Deserialize)
           │         // → Deserialize(m_origin_data, size, pData)
           │         SigIf_xxx_Cb(&data, SigIf_xxx_Ctx);  // 用户回调
           │     }
           │     break;
           │   case SERVER_ID_OTA_DownloadStatus_RX:
           │     ...
           ▼
  APP 用户回调: OnOTASelfDownloadCmd(&data, ctx)
```

**UDS proxy 中的关键配置**：

```cpp
// proxy/x_dom_someip_rt_gen_uds/x_dom_someip_rt_cfg.cpp

// 1. 接收服务 (从 DCMS 收 COM 转发)
SvcItemInfo someip_recv_svc_info_provided_method_PS_FM_OTASelfDL_..._RX(
    SERVER_ID_..., "PS_FM_OTASelfDL_Mgmt_HAD_JO__...",
    100,              // timeout
    SigIf_OnRecvData, // m_recv_func = 指向 proxy 回调
    0);

const map<uint32_t, SvcItemInfo*> svc_route_recv_svc_info_map = {
    {SERVER_ID_OTA_DownloadCmd_RX,    &...},
    {SERVER_ID_OTA_DownloadStatus_RX, &...},
    {SERVER_ID_OTA_DownloadScript_RX, &...},
};

// 2. 发送服务 (通过 DCMS 发送到 COM, 由 COM 转发为 SOME/IP)
SvcItemInfo someip_send_svc_info_provided_method_PS_FM_OTASelfDL_..._TX(
    SERVER_ID_TX, "PS_FM_OTASelfDL_Mgmt_HAD_JO__...", 0);

const map<uint32_t, SvcItemInfo*> svc_route_send_svc_info_map = {
    {SERVER_ID_OTA_DownloadCmd_TX,    &...},
    {SERVER_ID_OTA_DownloadStatus_TX, &...},
    {SERVER_ID_OTA_DownloadScript_TX, &...},
    {SERVER_ID_OTA_NotifyScript_TX,   &...},
};

// 3. 注册 DCMS 回调
void svc_route_write_someip_init(dcms_mcu_topic_cbk cbk) {
    topic_id = dcms_adp_get_topic_id("uds_app_comif", 
        "/x_dom_someip_rt/com_to_app/uds_app");
    dcms_mcu_topic_setup_callback(topic_id, cbk, ack_data, sizeof(ack_data));
}

// 4. 配置注入
x_dom_someip_rt_prod_cfg() {
    x_dom_someip_rt_set("app_service_en", (void*)(true), ...);
    x_dom_someip_rt_set("topic_node_name", (void*)("uds_app_comif"), ...);
    x_dom_someip_rt_set("svc_route_recv_svc_info_map", (void*)(&recv_map), ...);
    x_dom_someip_rt_set("svc_route_send_svc_info_map", (void*)(&send_map), ...);
    x_dom_someip_rt_set("svc_route_write_someip_init", (void*)(init_func), ...);
}
```

### 5.4 接口3: comif 反向发送 (UDS → COM → 外部 ECU)

```
UDS 进程: APP
  │ SigIf_xxx_Set(pData)
  ▼
  【Proxy】SigIf_xxx_Set()
           │ [x_dom_someip_rt_sigif_set.cpp]
           │
           │ 【Proxy → SDK】序列化写入
           │ someip_send_svc_info_xxx_TX
           │   .set_usr_data_ext<>(offset, pData, Serialize);
           │   ↑ 内部: Serialize(pData, m_usr_data, &size)
           │
           │ 【Proxy → SDK】立即转发
           │   .fwd_svc_via_dcms("/x_dom_someip_rt/app_to_com", false);
           │   ↑ 内部: pack(m_usr_data) → dcms_mcu_topic_send()
           ▼
  DCMS → COM 进程 → svc_route_app_to_com_topic_cbk()
                    → SvcItemInfo::fwd_svc_via_someip()
                    → Cdd_SomeIpTp Tx → SomeIpTp → SoAd → Ethernet → 外部ECU
```

### 5.5 接口4: comif CAN 信号接收 (COM → DCMS → UDS)

```
COM 进程
  │ DCMS topic: "/x_dom_can_rt/read_sigs/uds_app"
  │ 携带序列化后的 CAN 信号结构体
  ▼
UDS 进程
  【SDK】sig_route_mcu_rx_sigs_topic_cbk(data, len)
         │ [sig_route_soc.cpp:66]
         │ XDOMCANRT_DJI_COM_SERVICE_EN=0 → 注册为回调
         │
         └→ 【Proxy】sig_route_update_mcu_tx_rx_sigs_from_com(data, len)
                    │ [x_dom_can_rt_gen_uds/x_dom_can_rt_cfg.cpp:53]
                    │
                    │ const sig_route_read_tx_rx_sigs_dji_uds_service_t *sig =
                    │     (const sig_route_read_tx_rx_sigs_dji_uds_service_t *)data;
                    │
                    │ lock(mutex)
                    │ sig_route_dji_uds_service_update_cnt++
                    │
                    │ if (hash 匹配 && 长度匹配):
                    │     sig_route_dji_uds_service_update_flag = true;
                    │     memcpy(&sig_route_read_tx_rx_sigs_dji_uds_service,
                    │            data, len);
                    ▼
  APP 可直接读: sig_route_read_tx_rx_sigs_dji_uds_service.xxx_field
```

**fawhq_e001_10 的现状**：CAN_UDS_V4=OFF，UDS 的 sig_route_send_tx_rx_sigs_dji_uds_service 为空体，即该车型 UDS 不通过 comif 接收 CAN 信号。CAN 信号结构体仅定义了基本的 hash 校验和管理 debug shell。

---

## 六、SDK 同一份代码在不同域的差异化行为

`plat_bf_cdd.so` 中的 `service_route.cpp` 同时被 COM 和 UDS 进程加载（同一份 ELF 共享库），但行为完全不同。

### 6.1 配置驱动的分支

```cpp
// service_route.cpp — 同一份代码两种角色

void svc_route_cycle_pub(void)  // 50ms 定时
{
    // ============ COM 域执行 ============
    if (g_x_dom_someip_config.com_service_enabled)  
    {
        // COM: 遍历 NodeRecvInfo 列表, 打包发给 UDS/AD
        for (auto& elem : *g_x_dom_someip_config.svc_route_recv_svc_info_list_ptr)
        {
            check_and_send_svc_list_via_dcms_on_timeout(
                *g_x_dom_someip_config.svc_route_recv_svc_info_map_ptr,
                elem->svc_list,
                elem->topic_name.c_str());
        }
    }
    
    // ============ UDS/AD 域执行 ============
    if (g_x_dom_someip_config.app_service_enabled)  
    {
        // UDS: 扫描自己的 send 服务, 超时发送
        check_and_send_svc_via_dcms_on_timeout(
            *g_x_dom_someip_config.svc_route_send_svc_info_map_ptr,
            "/x_dom_someip_rt/app_to_com");
    }
}
```

### 6.2 各域配置对比

| 配置项 | COM进程 | UDS进程 |
|---|---|---|
| `com_service_enabled` | **true** | false |
| `app_service_enabled` | false | **true** |
| `topic_node_name` | com_app_comif | **uds_app_comif** |
| `svc_route_recv_svc_info_map_ptr` | 120+服务 | **3个** OTA服务 |
| `svc_route_send_svc_info_map_ptr` | 所有TX服务 | **4个** OTA响应 |
| `svc_route_recv_svc_info_list_ptr` | ad_app + uds_app | **nullptr** |

### 6.3 sig_route_soc.cpp 同样有差异化分支

```cpp
// sig_route_soc.cpp

void sig_route_send_mcu_tx_rx_sigs_proc(void)
{
    #if XDOMCANRT_DJI_COM_SERVICE_EN  // COM域=1, UDS域=0
        // COM: 从 CanMsg 缓存读信号, 打包发送给 UDS/AD
        sig_route_send_mcu_tx_rx_sigs();  // extern → proxy 定义
    #endif
}

// DCMS 回调注册:
// COM域: 注册 "/x_dom_can_rt/write_sigs" as server (接收 UDS 发来的)
// UDS域: 注册 "/x_dom_can_rt/read_sigs/uds_app" as client (接收 COM 发来的)
```

---

## 七、完整物理链路汇总

```
═══════════════════════════════════════════════════════════════════════════
                        CAN 信号 (以 VDC_DecelerationReq 为例)
═══════════════════════════════════════════════════════════════════════════

物理CAN → MCU(CAN Controller) → MCU(x_dom_can_rt) 
  → DCMS → COM(msg_route_soc) → proxy路由表 → CanMsg缓存 → SigHdl解析
    → 打包结构体 → DCMS → AD(sig_route回调) → memcpy到本地 → APP

═══════════════════════════════════════════════════════════════════════════
                     SOME/IP 信号收方向 (外部ECU → UDS)
═══════════════════════════════════════════════════════════════════════════

外部ECU → Ethernet → COM(SoAd/SomeIpTp) → Cdd_SomeIpTp.c
  ┌─ 732个服务: 空函数体 → LdCom → Com → COM进程内AUTOSAR消费 (danvince)
  └─ 114个服务: x_dom_someip_rt_rx_handle() → service_route.cpp
       → 查proxy路由表 → SvcDataPackHdl打包 → DCMS
         → UDS(svc_route_com_to_app_topic_cbk) → SvcDataUnPackHdl解包
           → 查proxy map → SvcItemInfo::set_origin_data()
             → m_recv_func=SigIf_OnRecvData()
               → SigIf_xxx_Get() → Deserialize() → APP回调

═══════════════════════════════════════════════════════════════════════════
                     SOME/IP 信号发方向 (UDS → 外部ECU)
═══════════════════════════════════════════════════════════════════════════

APP → SigIf_xxx_Set() → SvcItemInfo::set_usr_data_ext(Serialize)
  → fwd_svc_via_dcms() → DCMS
    → COM(svc_route_app_to_com_topic_cbk) → 查proxy send map
      → SvcItemInfo::fwd_svc_via_someip()
        → Cdd_SomeIpTp Tx → SomeIpTp → SoAd → Ethernet → 外部ECU

═══════════════════════════════════════════════════════════════════════════
                     DoIP 诊断 (外部诊断仪 → UDS)
═══════════════════════════════════════════════════════════════════════════

外部诊断仪 → DoIP(端口13400) → UDS(SoAd/DoIP) → PduR → Dcm
  → DiagnosticService.c / DiagSwc_Callback.c / Dcm_Callout_Stubs.c
    → (可选) DiagProxy_Com.cpp → SigIf_xxx_Get() 读 comif 跨域数据

═══════════════════════════════════════════════════════════════════════════
```

---

## 八、关键文件索引

| 文件 | 位置 | 作用 |
|---|---|---|
| **Cdd_SomeIpTp.c** | autosar_adapter/microsar_config_com/Appl/GenData/ | SOME/IP 分发：846个回调，114个接 comif |
| **Cdd_SomeIpTp.h** | 同上 | SOME/IP 类型定义、缓冲结构 |
| **service_route.cpp** | plat-bf/dsar_app/cdd/x_dom_someip_rt/ | comif SDK: SOME/IP 路由引擎 |
| **x_dom_someip_rt.cpp** | 同上 | comif SDK: 初始化入口 |
| **x_dom_someip_rt_com.h** | plat-bf/dsar_app/include/.../comif/ | SvcItemInfo, SvcDataPackHdl, NodeRecvInfo 定义 |
| **x_dom_someip_rt_com_cfg.h** | plat-bf/dsar_app/cdd/x_dom_someip_rt/ | 全局配置结构体 g_x_dom_someip_config |
| **msg_route_soc.cpp** | plat-bf/dsar_app/cdd/x_dom_can_rt/ | comif SDK: CAN 帧解析引擎 |
| **sig_route_soc.cpp** | 同上 | comif SDK: CAN 信号路由 |
| **sig_handle_soc.hpp** | plat-bf/dsar_app/include/.../cdd/ | SigHdl 模板: CAN位提取+物理值转换 |
| **msg_route_soc.h** | plat-bf/dsar_app/include/.../cdd/ | CanMsg 类: CAN帧缓存 |
| **dcms_mcu_api.cpp** | plat-bf/dsar_app/cdd/dcms_adapt/ | DCMS API 封装 |
| **x_dom_someip_rt_cfg.cpp** | proxy/x_dom_someip_rt_gen_com/ | COM proxy: SvcItemInfo + 路由表 (120+服务) |
| **x_dom_someip_rt_cfg.cpp** | proxy/x_dom_someip_rt_gen_uds/ | UDS proxy: SvcItemInfo + 路由表 (3+4个OTA服务) |
| **x_dom_someip_rt_sigif_get.cpp** | 同上 | UDS proxy: SigIf_OnRecvData + SigIf_xxx_Get |
| **x_dom_someip_rt_sigif_set.cpp** | 同上 | UDS proxy: SigIf_xxx_Set |
| **x_dom_someip_rt_protocol_*.cpp** | 同上 | UDS proxy: Serialize/Deserialize |
| **x_dom_can_rt_cfg.cpp** | proxy/x_dom_can_rt_gen_com/ | COM proxy: CanMsg + SigHdl + 转发函数 |
| **x_dom_can_rt_sigif.cpp** | 同上 | COM proxy: SigIf_Get/Set CAN信号 |
| **DiagProxy_Com.cpp** | DiagProxy/ | C→C++ 桥接, UDS诊断调 comif 信号 |
| **microsar_config_com.cmake** | autosar_adapter/microsar_config_com/ | COM BSW 模块 + X_DOM_SOMEIP_RT_COM_EN 定义 |
| **microsar_config_uds.cmake** | autosar_adapter/microsar_config_uds_v4/ | UDS BSW 模块 (无 SomeIpTp/Sd/LdCom/Com) |
| **consys_bf.cmake** | dsar_app/consys/ | 各域 .so 链接配置 |
| **fawhq_e001_10_config.cmake** | product/faw/fawhq_e001_10/ | product config: COM/CAN/UDS 开关 |

---

## 九、FAQ

### Q1: comif 和 DCMS 是什么关系？
**comif 在 DCMS 之上。** DCMS 是唯一的 IPC 传输通道（由 BSP 层 `dcos_dcms` 提供），comif 是 DCMS 之上的路由+序列化+缓存框架。comif 内部大量调用 `dcms_mcu_topic_send()` 和 `dcms_mcu_topic_setup_callback()`。

### Q2: 同域内为什么还要用 DCMS？
**同进程内不需要。** AUTOSAR BSW 调用 comif SDK 是直接函数调用（`Cdd_SomeIpTp.c` → `x_dom_someip_rt_rx_handle()`）。DCMS 只在以下场景使用：MCU↔SOC 跨核、COM↔UDS 跨进程、COM↔AD 跨进程。

### Q3: comif 的优势是什么？
**工具自动生成全链路代码。** 传统方式需要人工：从 AUTOSAR Com 信号读值 → 手动序列化 → 手动 IPC 发送 → 对端手动反序列化。comif 方式：JSON config → 工具生成 Cdd_SomeIpTp.c 调用点 + proxy 路由表 + SigIf 类型安全接口 + Serialize/Deserialize 函数。

### Q4: 为什么 UDS 不直接收 SOME/IP，一定要经过 COM？
**UDS 没有 SomeIpTp 协议栈。** UDS 的 AUTOSAR BSW 只有 Dcm/DoIP/PduR/SoAd(DoIP)，没有 SomeIpTp/Sd/LdCom/Com。SOME/IP 端口在 OS 层被路由到 COM 进程的 SoAd。

### Q5: 同一个 SDK 代码在 COM 和 UDS 中行为一样吗？
**不一样。** `service_route.cpp` 等 SDK 代码编译为 `plat_bf_cdd.so`，COM 和 UDS 进程各自加载同一份 .so，但通过 `g_x_dom_someip_config` 中的不同配置走不同分支（COM 做转发者，UDS 做消费者）。同样，`sig_route_soc.cpp` 被编译到 COM 和 UDS 各自的静态库中，通过 `XDOMCANRT_DJI_COM_SERVICE_EN` 宏差异化行为。

### Q6: 846 个 SOME/IP 服务中为什么只有 114 个走 comif？
因为只有需要跨域共享的服务才需要转发。例如 `Veh_VM_PowerModeInfo` 服务中，`PMM_NotifyONChgPowerMode` 需要通知 UDS/AD，所以走 comif；而 `PMM_NotifyONChgIGSt` 只有 COM 域自己用，所以走 danvince 在 COM 进程的 AUTOSAR 栈内消费。

---

## 十、comif 工具链

```
JSON 配置文件 (车型信号定义)
  │
  ▼
代码生成工具 (x_dom_someip_rt config tool / x_dom_can_rt config tool)
  │
  ├→ Cdd_SomeIpTp.h/c         AUTOSAR SOME/IP 分发 (含 X_DOM_SOMEIP_RT_COM_EN 条件编译)
  ├→ proxy/x_dom_can_rt_gen_*/x_dom_can_rt_cfg.cpp     CAN信号 + CanMsg + SigHdl + 转发函数
  ├→ proxy/x_dom_can_rt_gen_*/x_dom_can_rt_sigif.cpp   CAN SigIf_Get/Set
  ├→ proxy/x_dom_someip_rt_gen_*/x_dom_someip_rt_cfg.cpp  SvcItemInfo + 路由表 + NodeRecvInfo
  ├→ proxy/x_dom_someip_rt_gen_*/x_dom_someip_rt_sigif_get.cpp  SigIf_OnRecvData + Get
  ├→ proxy/x_dom_someip_rt_gen_*/x_dom_someip_rt_sigif_set.cpp  SigIf_Set
  ├→ proxy/x_dom_someip_rt_gen_*/x_dom_someip_rt_protocol_*.cpp Serialize/Deserialize
  └→ x_dom_can_rt_cfg.h (FW侧)   MCU侧 CAN 信号定义
```
