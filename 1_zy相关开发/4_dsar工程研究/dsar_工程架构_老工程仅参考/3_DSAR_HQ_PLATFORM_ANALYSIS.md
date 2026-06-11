# DSAR-HQ-PLAT 平台仓三大子项目详细分析

---

## 一、平台仓总览

`dsar-hq-plat` 是 DSAR 项目的**平台公共代码仓**，代码通过 Conan 包管理器发布，供适配仓 `dsar-hq` 依赖。包含三个独立子项目：

```
dsar-hq-plat/
├── dsar-plat-ad/     ★ AD 平台 — 自动驾驶应用框架 (SOC 侧)
├── dsar-plat-bf/     ★ BF 平台 — 基础功能 + 诊断 + CDD (SOC 侧为主)
└── dsar-sip/         ★ SIP 协议栈 — AUTOSAR 信号接口协议 (SOC + MCU)
```

### 1.1 三大子项目在编译链路中的位置

```
┌──────────────────────────────────────────────────────────────┐
│                         编译时                                │
│                                                              │
│  dsar-hq-plat (平台仓)                                       │
│  ├── dsar-plat-ad  ──Conan包──→  dsar_app CMake 编译         │
│  │    (头文件 + 源码 + 预编译静态库)                             │
│  │                                                            │
│  ├── dsar-plat-bf  ──Conan包──→  dsar_app CMake 编译         │
│  │    (头文件 + 源码 + 预编译静态库 + CDD + 诊断)                │
│  │                                                            │
│  └── dsar-sip      ──Git Submodule──→  dsar_fw Makefile 编译  │
│       (MCU侧源码)   ──Conan包间接──→  dsar_app CMake 编译     │
│       (SOC侧源码)                                             │
│                                                              │
│  dsar-hq (适配仓)                                            │
│  ├── dsar_fw/  ← 依赖 dsar-sip 源码 + dsar-bsw submodule     │
│  └── dsar_app/ ← 依赖 dsar-plat-ad + dsar-plat-bf Conan 包   │
└──────────────────────────────────────────────────────────────┘
```

---

## 二、dsar-plat-ad — AD 平台（自动驾驶应用框架）

### 2.1 定位

**Conan 包**: `dsar-plat/<version>@TPV1/release`（如 `v2.1261.0-beta.87`）

提供 SOC 侧自动驾驶应用的**核心框架代码**，不包含车型特定的信号实现。适配仓通过 PAL 工具 + 车型配置填充具体信号。

### 2.2 完整目录结构与功能模块

```
dsar-hq-plat/dsar-plat-ad/src/dsar_app/
│
├── app_core/app_ad_core/                 ★ AD 应用核心框架
│   │
│   ├── dsar_plat/                         ★ 平台抽象层 (最核心)
│   │   ├── dsar_plat_config.cmake         平台配置 + Conan 桥接 + PAL 路径
│   │   ├── ad/                            AD (自动驾驶) 平台
│   │   ├── common/                        公共平台代码
│   │   ├── proxy/                         车辆信号代理框架
│   │   │   └── 定义: 信号代理的接口/基类/注册机制
│   │   │   └── 适配仓实现: vehicle_proxy_${product}
│   │   ├── business/                      业务逻辑框架
│   │   ├── sim/                           仿真平台 (CONFIG_DSAR_PLAT_SIM)
│   │   └── extension/                     扩展接口
│   │
│   ├── asw_app_core/                     应用软件核心
│   │   └── 应用层软件组件 (ASW-SWC) 框架
│   │
│   ├── mode_manager/                     模式管理器
│   │   └── 自动驾驶模式切换 (手动/ACC/TJA/HWA/NRP/HPA/APA...)
│   │
│   ├── domain_function/                  领域功能
│   │   ├── gateway_hmicom/               网关 — 人机交互通信
│   │   │   └── SOC ↔ 车机 (HMI) 的消息/信号传递
│   │   └── gateway_mqttcom/              网关 — MQTT 通信 (可选)
│   │
│   ├── dmm_common/                       DMM (动态模型管理) 公共
│   │   └── 车辆动力学模型 + 运动状态管理
│   │
│   ├── plat_app_interface/              平台应用接口
│   │   └── 对外暴露的标准化 API
│   │
│   └── fusa_app/                         功能安全应用
│       └── 安全相关功能 (安全等级 ASIL-B/D)
│
├── app_core/app_ge_core/                 GE (通用事件) 核心
│   └── gb_dssad/                         ★ DSSAD 国标数据记录 (公共代码)
│       ├── edr_data_process.cpp          主循环入口 (50ms周期任务)
│       ├── dssad_event_trigger.c/h       事件检测引擎 (7种事件边沿检测)
│       ├── dssad_data_pack.c/h           JSON序列化(cJSON) + 数据发送
│       ├── dssad_common.c/h              DssadConfig_t 函数指针注册框架
│       ├── dssad_if.h                    对外接口
│       └── cJSON.c/h                     JSON 库
│
├── app_core/app_bf_core/                 BF (基础功能) 核心
│
├── platform_desc.json                    平台描述 (编译工具链/交叉编译器/系统库)
│
└── CMakeLists.txt                        安装规则 (install TARGETS/HEADERS)
```

### 2.3 适配仓如何使用 dsar-plat-ad

```
适配仓 dsar_app/CMakeLists.txt:
  │
  ├── include(dsar_plat_ad.cmake)          ← 导入 AD 平台配置
  │     ├── 设置: DSAR_PLAT_AD_DIR         (平台包安装路径)
  │     ├── 设置: PAL 工具路径
  │     └── 设置: APP_CORE_INC (平台头文件路径)
  │
  ├── 引用平台源文件:
  │     ├── dsar_plat_config               静态库 → lib${product}_app.so
  │     ├── plat_ad_demo                   静态库 → lib${product}_app.so
  │     ├── plat_ad_fusa_app               静态库 → lib${product}_app.so
  │     ├── plat_ad_app_core               ★ 静态库 → lib${product}_app.so
  │     ├── plat_ad_cndtm                  静态库 → lib${product}_app.so
  │     ├── plat_ad_tlv                    静态库 → lib${product}_app.so
  │     ├── plat_vehicle_proxy             静态库 → lib${product}_app.so
  │     └── plat_common                    静态库 → lib${product}_app.so
  │
  └── PAL 代码生成:
        pal_gen → 从 JSON 生成车型信号代码
          ├── 输入: dsar-hq-plat/.../pal_param/*.json
          ├── 输出: product/${车型}/param/
          └── 产物: param_${product} 静态库
```

### 2.4 dsar-plat-ad 中的关键 cmake 文件

[dsar_plat_ad.cmake](dsar-hq-plat/dsar-plat-ad/src/dsar_app/dsar_plat_ad.cmake) 的作用：

```
dsar_plat_ad.cmake:
  ├── 设置平台安装路径
  │     DSAR_PLAT_AD_INSTALL_DIR = Conan 包的安装目录
  │
  ├── 导出可执行程序/库路径
  │     CMAKE_INSTALL_BIN_PREFIX  = /bin
  │     CMAKE_INSTALL_LIB_PREFIX  = /lib
  │     CMAKE_INSTALL_RUNTIME_CONF_PREFIX = /etc
  │
  ├── 配置 PAL 工具
  │     PAL_TOOL_PATH = DSAR_PLAT_AD_INSTALL_DIR/tools/pal_gen
  │     pal_build() 宏: JSON → C++ 代码生成
  │
  └── 配置全局变量调试工具
        GLOBAL_VAR_DBG_TOOL_PATH → elf 解析生成全局变量信息
```

---

## 三、dsar-plat-bf — BF 平台（基础功能 + 诊断 + CDD）

### 3.1 定位

**Conan 包**: `dsar-plat-bf/<version>@TPV1/release`（如 `v2.31.0-beta.6`）

提供**基础功能模块**，涵盖诊断通信 (DoIP/UDS)、复杂驱动 (CDD)、PAL 参数抽象层、Shell 基础设施、数据记录、系统管理等。主要服务于 SOC 侧，部分代码也通过 dsar-sip 间接服务于 MCU 侧。

### 3.2 完整目录结构与功能模块

```
dsar-hq-plat/dsar-plat-bf/
│
├── dsar_app/                              SOC 侧平台代码
│   │
│   ├── cdd/                               ★ 复杂驱动层 (Complex Device Driver)
│   │   ├── dcms_adapt/                    DCMS 适配 (跨核通信 SOC 端)
│   │   │   └── dcms_mcu_api.h            DCMS Topic 发送/订阅 API
│   │   │   └── SOC侧 MCU 消息收发实现
│   │   │
│   │   ├── system/                        系统管理
│   │   ├── dump/                          异常转储
│   │   ├── recorder/                      数据记录
│   │   ├── log/                           日志系统
│   │   │
│   │   ├── x_dom_can_rt/                  ★ 预生成的 CAN 信号路由 (SOC 侧)
│   │   │   └── DSAR_Plat_CAN_RT 框架接口
│   │   │   └── 预编译为 .a 静态库发布
│   │   │
│   │   ├── x_dom_someip_rt/               ★ 预生成的 SOMEIP 信号路由 (SOC 侧)
│   │   │   └── DSAR_Plat_SOMEIP_RT 框架接口
│   │   │   └── 预编译为 .a 静态库发布
│   │   │
│   │   ├── pal/                            PAL (参数抽象层) 生成代码
│   │   │   └── 参数表读写 + 信号配置表
│   │   │
│   │   ├── shell_soc/                     ★ SOC Shell 服务 + 客户端
│   │   │   ├── shellsoc_server.cpp         shellsoc_s_init + shell_req_cb (服务端)
│   │   │   ├── shellsoc_client.cpp         shellsoc_c_init / shellsoc_c_do_cmd (客户端)
│   │   │   └── shell_soc_register.cpp      RegisterShell → ELF 解析 → dlsym
│   │   │
│   │   ├── lifecycle_manager/             生命周期管理
│   │   ├── init/                          初始化模块
│   │   ├── var_dbg/                       全局变量调试 (ELF解析)
│   │   ├── trace/                         轨迹记录
│   │   └── exception_dump/                异常转储
│   │
│   ├── com/                               SOMEIP 通信框架
│   │   └── SOC 侧 SOMEIP 服务/客户端实现
│   │
│   ├── diag/                              诊断模块
│   │   ├── udsonip/                       ★ DoIP (UDS over IP) 协议栈
│   │   │   ├── IPv4 服务实现
│   │   │   └── IPv6 服务实现
│   │   └── log_export/                    日志导出
│   │
│   ├── ge/                                ★ GE (通用事件) 进程
│   │   ├── gb_dssad/                      DSSAD 公共代码 (与 AD 侧共享)
│   │   ├── app_calib/                     应用标定
│   │   ├── fl_eFenceDownload/             FL 围栏下载
│   │   └── sys_self_diag/                 系统自诊断
│   │
│   ├── tool/export/                       工具
│   │   └── dsar_shell/                    ★ dshell 终端工具
│   │       ├── dsar_shell_simple_history.cpp  dshell 主程序 (main, 命令分发)
│   │       ├── shell_pal.cpp/h               PAL 参数表命令实现
│   │       ├── shell_config_simple.cpp        配置文件解析
│   │       ├── cmd_history.c/h               命令历史 (上下箭头)
│   │       └── config/sa86xx/                 SA8650 MCU 设备 UDP 地址配置
│   │
│   └── include/dsar_plat_bf/              BF 平台公共头文件
│       ├── cdd/
│       │   ├── shell_soc_cmd.h             DECLEAR_CMD 宏 (SOC侧) + t_shell_cmd_info
│       │   ├── shell_soc.h                 RegisterShell + shellsoc_s/c 接口
│       │   ├── dshell_soc.h                dshell_soc_init + cmd_add + try_exec_cmd
│       │   └── dcms_mcu_api.h              DCMS Topic 发送/订阅 API
│       └── ...
│
└── dsar_fw/                               MCU 侧预编译库 (少量)
    └── bsp_al/bsp_core/                    BSP 核心
```

### 3.3 适配仓如何使用 dsar-plat-bf

```
适配仓 dsar_app/CMakeLists.txt:
  │
  ├── include(dsar_plat_bf.cmake)          ← 导入 BF 平台配置
  │     ├── 设置: DSAR_PLAT_BF_DIR          (BF 平台包安装路径)
  │     ├── 设置: X_DOM_CAN_RT_SRC           (CAN 路由预编译库)
  │     ├── 设置: X_DOM_SOMEIP_RT_SRC        (SOMEIP 路由预编译库)
  │     ├── 设置: PAL 参数表路径
  │     └── 设置: dshell 编译开关
  │
  ├── 引用平台静态库:
  │     ├── autosar_com                     静态库 → lib${product}_com.so
  │     ├── autosar_uds                     静态库 → libdji_uds_v4.so / lib${product}_uds_v6.so
  │     ├── dcos_dcms                       静态库 → lib${product}_com.so
  │     ├── plat_bf_cdd                     静态库 → lib${product}_com.so
  │     └── plat_bf_ge                      静态库 → lib${product}_com.so
  │
  └── dshell 编译:
        ├── 源码: dsar-hq-plat/dsar-plat-bf/dsar_app/tool/export/dsar_shell/
        └── 产出: dshell 可执行工具
```

### 3.4 BF 平台的关键服务

#### DCMS 中间件 (cdd/dcms_adapt/)

DCMS (DJI Cross-core Message Service) 是 SOC ↔ MCU 跨核通信的核心中间件：

```
SOC 侧 API:
  dcms_mcu_topic_setup_callback(topic_id, callback, ...)  ← 订阅 MCU 消息
  dcms_mcu_topic_send_msg(topic_id, msg)                  ← 发送消息到 MCU
  dcms_mcu_topic_send(topic_id, msg)                      ← 发送消息到 MCU (立即发送)

MCU 侧 API (对称):
  dcms_mcu_topic_setup_callback(topic_id, callback, ...)  ← 订阅 SOC 消息
  dcms_mcu_topic_send_msg(topic_id, msg)                  ← 发送消息到 SOC
  dcms_mcu_topic_send(topic_id, msg)                      ← 立即发送
```

#### DoIP 诊断栈 (diag/udsonip/)

实现 ISO 13400 (DoIP) 协议，支持 IPv4 和 IPv6：

```
DoIP 服务:
  ├── dji_doip_service_ipv4               ← IPv4 诊断入口 (可执行程序)
  │     └── 加载: libdji_uds_v4.so        ← IPv4 UDS 服务 SO
  │
  └── dji_doip_service_ipv6               ← IPv6 诊断入口 (可执行程序)
        └── 加载: lib${product}_uds_v6.so ← IPv6 UDS 服务 SO

支持的 UDS 服务 (ISO 14229):
  ├── 0x10 诊断会话控制
  ├── 0x11 ECU 复位
  ├── 0x22 按 ID 读取数据
  ├── 0x2E 按 ID 写入数据
  ├── 0x27 安全访问
  ├── 0x28 通信控制
  ├── 0x3E 测试器保持在线
  ├── 0x85 DTC 设置控制
  └── ...
```

#### Shell 基础设施 (cdd/shell_soc/)

参见 `02_ECU_SHELL_ARCHITECTURE.md` 的详细分析。核心机制：

```
命令注册: DECLEAR_CMD 宏 → .shell_soc_cmd 链接器段
命令发现: RegisterShell() → 解析 ELF → dlopen/dlsym
命令调用: dshell → DCOS RPC → shell_req_cb() → handler()
```

#### DSSAD 公共代码 (ge/gb_dssad/)

参见 `01_DSSAD_ARCHITECTURE.md` 的详细分析。公共代码在 `dsar-plat-bf/dsar_app/ge/gb_dssad/` 中，车型适配代码在 `dsar-hq/src/dsar_app/product/*/oem_feature/` 中。

---

## 四、dsar-sip — SIP 协议栈（信号接口协议）

### 4.1 定位

**发布方式**: 同时作为 **Git Submodule** (在 dsar-hq 中) 和 **源码** (在 dsar-hq-plat 中)

SIP (Signal Interface Protocol) 是 AUTOSAR 标准的信号接口协议栈，负责：
- CAN 信号的路由和转发（跨域 CAN → SOC/MCU 双向）
- SOMEIP 信号的路由和转发
- AUTOSAR 通信栈的 MCU 侧实现
- SecOC (安全车载通信) 管理
- 网络管理 (NM)
- ECU 状态管理 (ESH)
- UDS 传输层 (UdsTp)

### 4.2 完整目录结构与功能模块

```
dsar-hq-plat/dsar-sip/
│
├── dsar_app/                              SOC 侧 (编译进 SOC .so 文件)
│   ├── sip_autosar/                        AUTOSAR 信号路由 SOC 侧
│   └── sip_extend/                        SIP 扩展 SOC 侧
│       ├── app_reset/                     应用复位
│       └── include/                        SOC 侧 SIP 头文件
│
├── dsar_fw/                               ★ MCU 侧 (编译进 MCU 固件)
│   ├── sip_autosar/                        AUTOSAR 信号路由 MCU 侧
│   └── sip_extend/                        SIP 扩展 MCU 侧
│       │
│       ├── bsw_manager_v2/                ★ BSW 管理器 v2
│       │   ├── bswm_esh_state.c           ECU 状态处理器 (ESH)
│       │   ├── bswm_esh_timer.c           ESH 定时器
│       │   ├── nm_monitor.c               网络管理监控
│       │   ├── nm_service_diag28.c       诊断28服务通信控制
│       │   ├── nm_channel_active.c        NM 通道控制
│       │   └── nm_dataflow_monitor.c      数据流监控
│       │
│       ├── can_com_platform/             CAN 通信平台
│       │   ├── com_fl_mcu.c               FL 通信 MCU 侧
│       │   └── can_detect_platform.c      CAN 检测平台
│       │
│       ├── can_gateway/                   CAN 网关
│       │   ├── can_gateway_master_new.c   CAN 网关主控 (新版)
│       │   └── net_blackbox.c             网络黑匣子
│       │
│       ├── diag/                          诊断
│       │   ├── SysDiag/
│       │   │   └── SysDiag_Common.c        系统诊断公共 (功能状态/故障状态)
│       │   └── UdsTp/                     UDS 传输层
│       │
│       ├── osal/                          OS 抽象层
│       │   ├── freertos/                  FreeRTOS 适配
│       │   └── safertos/                  SafeRTOS 适配
│       │
│       ├── secoc/                         ★ SecOC 安全车载通信
│       │   ├── secoc_manager/
│       │   │   └── secoc_shell.c          SecOC 安全统计 shell
│       │   └── fvm/
│       │       └── 新鲜度值管理 (Freshness Value Manager)
│       │
│       ├── sim/                           SIP 仿真
│       ├── timesync/                      时间同步
│       ├── x_dom_can_rt/                  CAN 信号路由 MCU 侧
│       │   └── msg_route_mcu.c            MCU 侧报文路由
│       └── x_dom_someip_rt/               SOMEIP 信号路由 MCU 侧
│
├── include/                              跨 SOC/MCU 的公共头文件
│
└── tools/                                 代码生成工具
    └── 从 ARXML (AUTOSAR XML) 生成 C 信号代码
```

### 4.3 SOC 侧 vs MCU 侧的双重身份

dsar-sip 的独特之处在于它同时在 SOC 和 MCU 两侧提供功能：

```
                    dsar-sip 源码
                    ═══════════
                          │
          ┌───────────────┴───────────────┐
          │                               │
    dsar_app/sip_*                    dsar_fw/sip_*
    (SOC 侧源码)                      (MCU 侧源码)
          │                               │
          ▼                               ▼
    编译进 SOC .so:                   编译进 MCU .bin:
    lib${product}_com.so               sailsw3.bin / sailsw1.bin
    (通过 Conan 包的静态库)            (通过 Git Submodule 源码编译)
          │                               │
          ├── AUTOSAR COM (SOC侧)         ├── AUTOSAR COM (MCU侧)
          ├── SIP Extend (SOC侧)          ├── BSW Manager v2
          └── x_dom_*_rt (SOC侧框架)      ├── CAN Gateway
                                          ├── SecOC / FVM
                                          ├── UDS Transport Layer
                                          └── x_dom_*_rt (MCU侧路由)
```

### 4.4 SecOC — 安全车载通信

SecOC (Secure On-Board Communication) 是 AUTOSAR 标准中的安全通信模块，用于保护 CAN 总线消息的完整性和新鲜度：

```
SecOC 工作流程:
  │
  ├── 发送端:
  │     ├── 计算消息的 MAC (消息认证码)
  │     ├── 从 FVM 获取新鲜度值 (FV, Freshness Value)
  │     ├── 将 MAC + FV 附加到 CAN 消息
  │     └── 发送受保护的消息
  │
  ├── 接收端:
  │     ├── 提取 MAC + FV
  │     ├── 验证 FV 是否新鲜 (防重放攻击)
  │     ├── 重新计算 MAC 并与收到的 MAC 比对
  │     └── 验证通过 → 传递消息; 失败 → 上报安全事件
  │
  └── Shell 调试命令:
        secoc              → 查看 SecOC 统计 (消息数/失败数/重放检测)
        fvm                → 查看 FVM 新鲜度值
```

### 4.5 CAN 网关 (can_gateway/)

CAN 网关负责在不同 CAN 总线之间路由报文：

```
CAN 网络拓扑:
  ┌──────────┐   ┌──────────┐   ┌──────────┐
  │ CAN-FD 1 │   │ CAN-FD 2 │   │ CAN-FD 3 │
  │ (底盘)    │   │ (动力)    │   │ (车身)    │
  └────┬─────┘   └────┬─────┘   └────┬─────┘
       │               │               │
       └───────────────┼───────────────┘
                       │
               ┌───────┴────────┐
               │  CAN Gateway    │ ← can_gateway_master_new.c
               │  (MCU 侧实现)    │
               └───────┬────────┘
                       │
                   DCMS Topic
                       │
               ┌───────┴────────┐
               │  x_dom_can_rt  │ ← SOC 侧使用 CAN 信号
               │  (SOC 侧实现)   │
               └────────────────┘

Shell 调试:
  cangateway_monitor     → 查看网关路由状态
  net_blackbox           → 网络黑匣子 (记录异常)
  dataflow               → 数据流监控
  nm_monitor             → 网络管理监控
```

### 4.6 BSW 管理器 v2 (bsw_manager_v2/)

BSW 管理器负责基础软件的模式管理和状态协调：

```
BSW 管理器功能:
  ├── ECU 状态处理 (ESH)
  │     ├── 启动/运行/休眠/唤醒 状态机
  │     └── Shell: esh, esh_timer
  │
  ├── 网络管理 (NM)
  │     ├── NM 通道激活/停用
  │     └── Shell: nm_channel, nm_monitor
  │
  ├── 通信控制
  │     ├── 诊断28服务 (通信控制)
  │     └── Shell: comcontrol
  │
  ├── 数据流监控
  │     └── Shell: dataflow
  │
  └── 唤醒源管理
        └── Shell: wksrc (ecum.c)
```

---

## 五、三大子项目之间的依赖关系

### 5.1 依赖图

```
                    ┌──────────────┐
                    │  dsar-plat-ad│  AD 自动驾驶应用框架
                    │  (SOC侧)     │
                    └──────┬───────┘
                           │ 依赖
                           ▼
                    ┌──────────────┐
                    │  dsar-plat-bf│  BF 基础功能 (CDD/诊断/Shell/DCMS/PAL)
                    │  (SOC侧为主) │
                    └──────┬───────┘
                           │ 关联
                           ▼
                    ┌──────────────┐
                    │   dsar-sip   │  SIP 协议栈 (AUTOSAR通信/SecOC/CAN网关)
                    │  (SOC+MCU)   │
                    └──────┬───────┘
                           │ 关联 (通过 BSW)
                           ▼
                    ┌──────────────┐
                    │   dsar-bsw   │  AUTOSAR BSW (MCU 底层驱动+协议栈)
                    │   (MCU侧)    │
                    └──────────────┘
```

### 5.2 各子项目在编译产物中的分布

| 编译产物 | dsar-plat-ad | dsar-plat-bf | dsar-sip | dsar-bsw | dsar-hq 适配代码 |
|---------|:-----------:|:-----------:|:--------:|:--------:|:--------------:|
| MCU 固件 (sailsw3.bin) | | | ★ (源码) | ★ (源码) | ★ (源码) |
| MCU 固件 (sailsw1.bin) | | | ★ (源码) | ★ (源码) | ★ (源码) |
| lib${product}_com.so | | ★ (预编译库) | ★ (预编译库) | | ★ (源码) |
| libdji_uds_v4.so | | ★ (预编译库) | | | ★ (源码) |
| lib${product}_uds_v6.so | | ★ (预编译库) | | | ★ (源码) |
| lib${product}_app.so | ★ (预编译库) | | ★ (间接) | | ★ (源码) |
| dji_ad_app | ★ (链接) | | | | ★ (源码) |
| dji_bf_app | | ★ (链接) | | | ★ (源码) |
| doip_service_* | ★ (链接) | ★ (预编译库) | | | ★ (源码) |
| dshell | | ★ (源码编译) | | | |

### 5.3 三者代码共享机制

```
代码共享方式:
  │
  ├── dsar-plat-ad ←→ dsar-plat-bf:
  │     两者通过 Conan 依赖关系间接关联
  │     DSSAD 公共代码在 plat-bf/ge/gb_dssad/ 中
  │     AD 侧通过 PAL 工具生成信号代码间接使用 plat-bf 的 x_dom_can_rt 框架
  │
  ├── dsar-plat-bf ←→ dsar-sip:
  │     SOC 侧: plat-bf 的 CDD 代码调用 sip 的 AUTOSAR COM 接口
  │     MCU 侧: 两者源码都被编译进 MCU 固件，共享 include 路径
  │
  └── dsar-sip ←→ dsar-bsw:
        dsar-sip 的 MCU 侧代码依赖 dsar-bsw 提供的 AUTOSAR BSW 接口
        (如 CanIf, PduR, Com, SecOC 等 AUTOSAR 标准模块)
```

---

## 六、关键文件索引

```
=== dsar-plat-ad ===
dsar-hq-plat/dsar-plat-ad/src/dsar_app/
├── dsar_plat_ad.cmake                       AD 平台桥接 cmake (安装路径/PAL工具)
├── app_core/app_ad_core/dsar_plat/           ★ 平台抽象层 (proxy/business/sim/extension)
├── app_core/app_ad_core/asw_app_core/        应用软件核心
├── app_core/app_ad_core/mode_manager/        模式管理器
├── app_core/app_ad_core/domain_function/     领域功能 (gateway_hmicom/mqttcom)
├── app_core/app_ad_core/dmm_common/          DMM 公共
├── app_core/app_ad_core/plat_app_interface/  平台应用接口
├── app_core/app_ad_core/fusa_app/            功能安全应用
└── app_core/app_ge_core/gb_dssad/            DSSAD 公共代码

=== dsar-plat-bf ===
dsar-hq-plat/dsar-plat-bf/dsar_app/
├── dsar_plat_bf.cmake                       BF 平台桥接 cmake (X_DOM_CAN_RT_SRC/PAL路径)
├── cdd/dcms_adapt/                          ★ DCMS 跨核通信 SOC 端
├── cdd/x_dom_can_rt/                        ★ 预生成 CAN 信号路由框架
├── cdd/x_dom_someip_rt/                     ★ 预生成 SOMEIP 信号路由框架
├── cdd/pal/                                  PAL 参数抽象层
├── cdd/shell_soc/                           ★ SOC Shell 服务/客户端 (ELF解析+dlsym)
├── cdd/system/                               系统管理
├── cdd/dump/                                 异常转储
├── cdd/log/                                  日志系统
├── cdd/lifecycle_manager/                   生命周期管理
├── cdd/var_dbg/                             全局变量调试工具
├── com/                                      SOMEIP 通信框架
├── diag/udsonip/                            ★ DoIP 诊断协议栈
├── ge/gb_dssad/                              DSSAD 公共代码
├── ge/app_calib/                             应用标定
├── ge/fl_eFenceDownload/                     FL 围栏下载
├── tool/export/dsar_shell/                  ★ dshell 终端工具
└── include/dsar_plat_bf/cdd/                 BF 平台公共头文件
    ├── shell_soc_cmd.h                       DECLEAR_CMD 宏 (SOC侧)
    ├── shell_soc.h                           RegisterShell 接口
    ├── dshell_soc.h                          dshell 客户端接口
    └── dcms_mcu_api.h                        DCMS Topic API

=== dsar-sip ===
dsar-hq-plat/dsar-sip/
├── dsar_fw/sip_extend/
│   ├── bsw_manager_v2/                      ★ BSW 管理器 (NM/ESH/唤醒源)
│   ├── secoc/secoc_manager/                 ★ SecOC 安全管理 + FVM
│   ├── can_gateway/                         ★ CAN 网关 + 黑匣子
│   ├── can_com_platform/                     CAN 通信平台 + FL 通信
│   ├── diag/SysDiag/                         系统诊断
│   ├── diag/UdsTp/                           UDS 传输层
│   ├── osal/freertos/                        FreeRTOS OS 抽象层
│   ├── osal/safertos/                        SafeRTOS OS 抽象层
│   ├── sim/                                  SIP 仿真
│   ├── timesync/                            时间同步
│   ├── x_dom_can_rt/                         CAN 信号路由 MCU 侧
│   └── x_dom_someip_rt/                      SOMEIP 信号路由 MCU 侧
│
├── dsar_app/sip_extend/
│   ├── app_reset/                           应用复位
│   └── include/                             SOC 侧 SIP 头文件
│
└── include/                                 跨 SOC/MCU 公共头文件
```
