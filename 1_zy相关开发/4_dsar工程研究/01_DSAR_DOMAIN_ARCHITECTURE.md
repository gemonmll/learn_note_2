# DSAR-HQ 域架构与通信机制

> 基于 FAW 项目 fawhq_e001_10（CONFIG_APP_GE_PROCESS=ON 分离模式），SA8650 芯片，QNX/Linux + RTOS 双系统。
> 上一轮文档中的以下错误已在本版纠正：
> - `libdji_uds_v4.so` 实际编译产物名为 `libuds_v4.so`（无 dji_ 前缀）
> - `dji_bf_app` 不加载 uds .so，UDS .so 由 `dji_doip_service_ipv4`/`ipv6` 各自加载
> - COM域和GE域是同一个 .so、同一个进程，UDS域是独立进程+独立 DCOS 域
> - `vehicle_proxy`/`adapter`/`param` 在 COM.so 和 AD.so 中是**不同源码目录**编译的不同 target

---

## 一、工程概览

### 1.1 工程定位

DSAR-HQ 是自动驾驶域控制器（SA8650 芯片）的**基础软件适配仓**，负责将平台公共代码（`dsar-hq-plat`）适配到具体车型。核心工作包括：

- **行泊车信号适配**：CAN / SOMEIP 信号的路由、转换、代理
- **诊断通信**：UDS (ISO 14229) 诊断服务的 DoIP 实现
- **DSSAD 数据记录**：国标 GB/T 要求的自动驾驶数据存储系统
- **跨核通信**：SOC (QNX/Linux) ↔ MCU (RTOS) 之间的 DCMS 消息传输
- **ECU Shell**：跨 ECU 的远程调试/诊断命令系统

### 1.2 双仓协作模型

```
dsar_app_fw_platform_vip/                     ← 工作空间
├── dsar-hq/          ← ★ 适配仓（本项目）      车型适配代码 + 编译入口
└── dsar-hq-plat/      ← 平台仓                 公共平台代码，发布为 Conan 包
```

- **平台仓 (dsar-hq-plat)**：提供公共框架、算法、协议栈，通过 Conan 包管理器发布。包含三个独立子项目：dsar-plat-ad（AD平台）、dsar-plat-bf（BF平台）、dsar-sip（SIP协议栈）
- **适配仓 (dsar-hq)**：依赖平台仓的 Conan 包，添加车型特有的信号适配、配置、shell 命令

### 1.3 芯片与操作系统

目标芯片为 **SA8650**（Qualcomm 自动驾驶 SoC），内部包含两个异构核心：

```
┌──────────────────────────────────────────────────────────┐
│                    SA8650 芯片                            │
│                                                          │
│  ┌─────────────────────────┐  ┌────────────────────────┐ │
│  │ SOC 侧 (HPC, Application│  │ MCU 侧 (Safety Island) │ │
│  │ Processor)              │  │                        │ │
│  │                         │  │                        │ │
│  │ OS: QNX 7.1 / Linux     │  │ OS: FreeRTOS / SafeRTOS│ │
│  │ 编译: CMake              │  │ 编译: Makefile         │ │
│  │ 产出: .so 动态库 + ELF   │  │ 产出: .bin 固件        │ │
│  │                         │  │                        │ │
│  │ ★ dsar_app              │  │ ★ dsar_fw              │ │
│  └─────────────────────────┘  └────────────────────────┘ │
│                                                          │
│  跨核通信: DCMS (发布/订阅中间件) + UDP/IPC               │
└──────────────────────────────────────────────────────────┘
```

### 1.4 Git Submodules

dsar-hq 适配仓包含以下 Git Submodules：

| Submodule | 路径 | 说明 |
|-----------|------|------|
| **dsar-bsw** | `src/dsar_fw/bsp_al/bsp_core` 附近 | AUTOSAR BSW (基础软件) — MCU 侧底层驱动和 AUTOSAR 协议栈 |
| **dsar-sip** | 通过 SIP_EXTEND_LIB_PATH 引用 | SIP 协议栈 — AUTOSAR 信号接口协议（CAN/SOMEIP 信号路由） |
| **control** | `src/dsar_app/app_core/app_ad_core/ctrl/` | 控制算法模块 |

### 1.5 适配仓 (dsar-hq) 顶层目录结构

```
dsar-hq/
├── conanfile.py                      ★ Conan 依赖声明 + 构建入口
├── conan_config.json                  构建配置（产品列表、镜像打包参数）
├── conan_product_requirements.py      各车型的依赖版本映射表
├── conan_version_dsar_bsw.py          dsar-bsw 版本号
├── conan_version_dsar_plat.py         dsar-plat / dsar-vip-plat 版本号
├── conan_version_dsar_plat_bf.py      dsar-plat-bf 版本号
├── conan_version_dsar_plat_ad.py      dsar-plat-ad 版本号
├── Build/                             构建脚本 (clone 自 dji_build_script)
│
├── src/
│   ├── dsar_fw/                       ★ MCU 侧固件代码 + Makefile 编译
│   │   ├── build/                       Makefile 入口 (main_sa8xx.mk)
│   │   ├── include/                     公共头文件 + include_all.mk
│   │   ├── proxy_comm/                  MCU 侧代理通信
│   │   ├── xwire/                       执行器旁路 / 车辆信息
│   │   ├── app_core/                    应用核心 (GE/版本/工作列表)
│   │   ├── product/                     车型产品代码 (faw/...)
│   │   └── ...                          (sip_extend/等子模块)
│   │
│   ├── dsar_app/                      ★ SOC 侧应用代码 + CMake 编译
│   │   ├── CMakeLists.txt               SOC 编译顶层入口
│   │   ├── entry/                       可执行程序入口 (dji_ad_app, dji_bf_app, doip)
│   │   ├── consys/                      CMake 宏定义 (consys_ad.cmake, consys_bf.cmake)
│   │   ├── app_core/                    应用核心 (GE/BF/AD 核心)
│   │   ├── product/                     车型产品代码 (faw/fawhq_*/)
│   │   ├── include/                     公共头文件
│   │   └── release/                     运行时脚本/参数 (scripts, runtime_param, pal_param)
│   │
│   └── include/                        跨 dsar_fw + dsar_app 的公共头文件
│       └── local_dji_msgs/             DJI 消息定义
│           ├── local_dji_msg_dsar_fw/   MCU 侧消息
│           └── local_dji_msg_dsar_app/  SOC 侧消息
```

---

## 二、域的定义与物理部署

### 1.1 四大域

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          SOC 侧 (A核, QNX/Linux)                          │
│                                                                          │
│  ┌─ COM域 ──────────────────────────────────────────────────────────┐   │
│  │  物理形态: libfawhq_e001_10_com.so                                 │   │
│  │  功能: CAN/SOMEIP信号路由、AUTOSAR COM协议栈适配、OEM信号代理      │   │
│  │  源码: app_core/app_com_core/ + product/.../proxy/x_dom_*_gen_com/ │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─ GE域 (General Event) ───────────────────────────────────────────┐   │
│  │  物理形态: 与 COM域 打包在同一个 libfawhq_e001_10_com.so 内        │   │
│  │  功能: 标定、参数管理、DSSAD国标数据记录、远程参数、VPM等            │   │
│  │  源码: app_core/app_ge_core/                                       │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                               │                                          │
│  两者关系: COM和GE在源码层面是两个 app_core 子目录，但在编译时被         │
│  编译成两个静态库后一起通过 --whole-archive 打包进同一个 .so，           │
│  由 dji_bf_app 一个进程统一加载，调用两个独立的入口函数。                │
│                                                                          │
│  ┌─ UDS域 ──────────────────────────────────────────────────────────┐   │
│  │  物理形态: libuds_v4.so (注意：无产品前缀！)                        │   │
│  │           libfawhq_e001_10_uds_v6.so (fawhq_e001_10中v6=OFF不编译) │   │
│  │  功能: DoIP诊断服务 (ISO 13400 + ISO 14229 UDS)                    │   │
│  │  源码: app_core/app_udsonip/                                       │   │
│  │  特殊: 运行在 MINIDCOS_DOMAIN(1)，与其他域不在同一个DCOS域          │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─ AD域 ──────────────────────────────────────────────────────────┐   │
│  │  物理形态: libfawhq_e001_10_app.so                                  │   │
│  │  功能: 行泊车感知/规划/控制适配、车辆信号代理、功能安全              │   │
│  │  源码: app_core/app_ad_core/product/.../                            │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
```

### 1.2 .so 命名规则（关键纠正）

| 域 | CMake target 名 | 实际 .so 文件名 | 为什么 |
|---|---|---|---|
| COM+GE | `${PRODUCT_NAME}_com` | `libfawhq_e001_10_com.so` | 产品前缀 |
| UDS v4 | `uds_v4` | `libuds_v4.so` | **无前缀**，因为 `check_autosar_uds_v4_build_first_run` 机制让多产品编译时只生成一份 |
| UDS v6 | `${PRODUCT_NAME}_uds_v6` | `libfawhq_e001_10_uds_v6.so` | 产品前缀 |
| AD | `${PRODUCT_NAME}_app` | `libfawhq_e001_10_app.so` | 产品前缀 |

UDS v4 的 `.so` 名称不带产品前缀，这是因为在 `consys_bf.cmake` 中直接写死了 `add_library(uds_v4 SHARED ...)`（第234行），而 COM、UDS v6、AD 都使用了 `${PRODUCT_NAME}` 变量。这是刻意设计——UDS v4 诊断协议栈在不同车型之间完全共享，不需要按产品区分。

### 1.3 进程与 .so 加载关系

```
┌─ 进程: dji_bf_app (DCOS_DOMAIN=0) ──────────────────────────────────┐
│  dlopen("com") → libfawhq_e001_10_com.so                             │
│    dll_syms_load(handle, "app_com_core_init", ...)  → COM通信核心入口│
│    dll_syms_load(handle, "app_ge_core_init", ...)   → GE通用功能入口 │
│  关键: 两次 dlsym 用的是同一个 dlopen handle，COM和GE在同一个 .so 内 │
└──────────────────────────────────────────────────────────────────────┘

┌─ 进程: dji_ad_app (DCOS_DOMAIN=0) ──────────────────────────────────┐
│  dlopen("app") → libfawhq_e001_10_app.so                             │
│    dll_syms_load(handle, "app_ad_core_init", ...)  → AD自动驾驶入口  │
│  在 FAW 分离模式下(APP_GE_PROCESS=ON)，此进程不加载 com.so            │
└──────────────────────────────────────────────────────────────────────┘

┌─ 进程: dji_doip_service_ipv4 (MINIDCOS_DOMAIN=1) ───────────────────┐
│  dlopen("uds_v4") → libuds_v4.so                                     │
│    dll_syms_load(handle, "app_uds_service_init", ...) → UDS诊断v4入口│
└──────────────────────────────────────────────────────────────────────┘

┌─ 进程: dji_doip_service_ipv6 (MINIDCOS_DOMAIN=1) ───────────────────┐
│  dlopen("uds_v6") → libfawhq_e001_10_uds_v6.so                      │
│    dll_syms_load(handle, "app_uds_service_init", ...) → UDS诊断v6入口│
│  (fawhq_e001_10 中 CONFIG_APP_UDSONIP_IPV6=OFF，实际不编译此进程)   │
└──────────────────────────────────────────────────────────────────────┘

┌─ 进程: dji_log_export ──────────────────────────────────────────────┐
│  独立可执行文件，不加载任何 .so                                       │
│  链接: plat_bf_cdd + plat_bf_log_export + dsar_plat_config          │
└──────────────────────────────────────────────────────────────────────┘
```

**合并模式（非 FAW 项目，APP_GE_PROCESS=OFF）**：
只有一个 `dji_application` 进程，同时 dlopen `app` 和 `com`，内部逻辑通过 `#ifdef APP_GE_PROCESS` 开关控制。

**关于 dji_com_service.cpp 和 dji_ge_app.cpp**：
这两个源文件存在于 `entry/` 目录下但并不参与 fawhq_e001_10 的编译（entry/CMakeLists.txt 中没有对应的可执行文件 target），它们是为其他项目配置保留的入口变体。

---

## 三、.so 之间的依赖关系

### 2.1 编译时：完全互不依赖

四个 .so 编译时 **完全互不链接、互不依赖**。证据：

1. 每个 .so 的 `target_link_libraries` 中不包含任何其他项目内 .so
2. 每个 .so 设置了 `-Wl,--no-undefined`，如果存在未解析的外部符号引用会直接链接失败
3. 所有 .so 通过 `--whole-archive` 各自打包一份所需的静态库副本（如 `plat_bf_cdd` 在 com.so、uds_v4.so、app.so 中各有一份）

### 2.2 运行时：通过 DCMS 话题通信

同芯片内跨进程、跨 DCOS 域的所有数据交换，全部走 DCMS 话题总线：

```
┌──────────────────┐    DCMS 话题     ┌──────────────────┐
│ dji_bf_app       │                  │ dji_ad_app       │
│ lib*_com.so      │  DCOS_DOMAIN=0   │ lib*_app.so      │
│ COM域 + GE域     │←────同域IPC────→│ AD域             │
└────────┬─────────┘                  └────────┬─────────┘
         │                                     │
         │     ┌──────────────────┐            │
         │     │ dji_doip_        │            │
         └─────│ service_ipv4     │────────────┘
               │ libuds_v4.so     │  跨域 IPC
               │ MINIDCOS_DOMAIN=1│  (UDP)
               └──────────────────┘
```

DCMS 提供四种通信模式，通过 `dcms_mcu_topic_cfg_t` 的标志位配置：

| 模式 | 标志位 | 场景举例 |
|---|---|---|
| 发布/订阅 | `pub_en=1, sub_en=1` | COM域发布CAN信号 → AD域订阅 |
| 客户端/服务端 | `client=1, server=1` | AD域请求诊断数据 → UDS域响应 |
| 同步RPC | `send_sync()` | 带超时的请求-响应，如参数读写 |
| 类型化消息 | `DCMS_TYPE(T)` | protobuf格式的结构化消息 |

**为什么同进程内也不用直接函数调用？** — 统一编程模型。不管通信双方在同进程、跨进程、还是跨芯片，代码写法完全一样。DCMS 底层会根据域 ID 自动选择最优传输方式（进程内直接传递、共享内存、或 UDP）。

### 2.3 DCOS 域的含义

- **DCOS_DOMAIN (0)**：标准业务域，dji_bf_app 和 dji_ad_app 都在此域，相互之间低延迟通信
- **MINIDCOS_DOMAIN (1)**：系统管理域，dji_doip_service_ipv4/ipv6 在此域，用于 OTA 升级、工厂参数、UDS 诊断、ECU 复位等系统级操作。与 DCOS_DOMAIN(0) 隔离

---

## 四、SOC 侧跨进程通信：全部走 DCMS

### 3.1 话题注册

每个进程初始化时通过话题配置表注册自己关注的话题：

```c
// 示例：COM域注册
static const dcms_mcu_topic_cfg_t dcms_mcu_topic_cfg_com[] = {
    {0, DCMS_TOPIC_SOMEIP_TX, "/sys/someip_tx/v1", 1/*pub*/, 0, 0, 0},
    {0, DCMS_TOPIC_SHELL_COM, "/sys/shell/dji_com_service/v1", 0, 0, 0, 1/*server*/},
    // ... 数十个话题
};
dcms_mcu_topic_register("com_app", dcms_mcu_topic_cfg_com, count);
```

### 3.2 典型数据流

```
CAN信号上行 (MCU→SOC→各域):
  MCU CAN驱动 → x_dom_can_rt_mcu → DCMS_TOPIC_XDOMCANRT_MCU_TX_RX_MSGS
    → COM域(x_dom_can_rt_soc)订阅 → 解析信号 → COM域再发布到业务话题
      → AD域订阅业务话题 → vehicle_proxy → adapter → AD算法

诊断请求下行 (外部诊断仪→UDS域):
  诊断仪 → Ethernet → DoIP → dji_doip_service_ipv4
    → libuds_v4.so 处理UDS协议
    → 如需读写CAN信号: DCMS_CLIENT_* → COM域 → DCMS → MCU → CAN总线
```

---

## 五、SOC ↔ MCU 跨芯片通信

### 4.1 架构

```
SOC 侧 (QNX/Linux)                          MCU 侧 (RTOS)
─────────────────                          ────────────
DCMS 话题                                  物理 CAN 总线
  ↑                                            ↑
  │  UDP over Ethernet                         │ AUTOSAR COM 栈 (Com_SendSignal/Com_ReceiveSignal)
  │                                            │
  ├── x_dom_can_rt_soc.cpp ── UDP ──→ x_dom_can_rt_mcu.c ──→ Com_SendSignal()
  │   (SOC信号→DCMS话题)              (DCMS话题→CAN信号)
  │                                            │
  └── x_dom_can_rt_soc.cpp ←── UDP ── x_dom_can_rt_mcu.c ←── Com_ReceiveSignal()
      (DCMS话题→SOC信号)              (CAN帧→DCMS话题)
```

**关键设计原则**：
- MCU 侧是 CAN/Ethernet 硬件的 owner，SOC 侧不直接接触物理总线
- SOC 侧可以通过 DCMS 话题反向控制 CAN 信号（写入 `DCMS_TOPIC_XDOMCANRT_WRITE_TO_MCU_MSGS` 话题，MCU 收到后调用 `Com_SendSignal()` 发送到车辆总线）
- 多写源时通过**信号仲裁机制**（`sig_route_arbitrate_item_t`）决定优先级和超时

### 4.2 共享代码

SOC 和 MCU 两侧共享同一套 SIP 配置，但运行时代码来自不同位置：

| 侧 | x_dom_can_rt 运行时代码来源 | 信号配置来源 |
|---|---|---|
| SOC | `dsar-plat-bf` conan 包 (`src/cdd/x_dom_can_rt/sig_route_soc.cpp`) | `product/.../proxy/x_dom_can_rt_gen_*/` |
| MCU | `dsar-sip` submodule (`dsar_fw/sip_extend/x_dom_can_rt/msg_route_mcu.c`) | `product/.../x_dom_can_rt_config/` |

两侧的信号ID、CAN ID、E2E参数定义来自同一套 JSON 描述文件，由 SIP 工具分别生成 SOC 和 MCU 版本的配置代码。

---

## 六、FW 侧（MCU）架构

### 5.1 构建与运行模型

```
编译: GNU Make (递归 Make + Kbuild风格 subdir-y/obj-y)
产出: 单一固件 dsar_fw.elf → objcopy → dsar_fw.bin
运行: RTOS (FreeRTOS / SafeRTOS)
```

与 SOC 侧的本质区别：

| | SOC 侧 | MCU 侧 |
|---|---|---|
| 编译产物 | 多个 .so + 多个 ELF | 单一 .elf/.bin |
| 加载方式 | dlopen 动态加载 | 编译时静态链接 |
| 进程模型 | 多进程 (Linux/QNX) | 单一地址空间 (RTOS) |
| 模块隔离 | 进程隔离，通过 DCMS 通信 | 全局 include 命名空间，直接函数调用 |

### 5.2 任务模型

FW 侧**不是每个模块一个线程**。总共只有少数几个 RTOS 任务：

```c
// appl_main.c — 4个RTOS任务
app_task_handler()      // 遍历高频+中频工作链表
low_freq_task_handler() // 遍历低频工作链表
comm_task_handler()     // DCMS消息泵: dcms_msg_handle_task_handler()
dbsw_task_handler()     // AUTOSAR BSW任务 (仅AUTOSAR OS)
```

模块通过**注册工作节点（work node）**把自己的处理函数挂到共享的工作链表上：

```c
WORK_NODE_DECLEAR(my_name, my_handler, arg);
add_period_work(task_high, &my_work);   // 注册到高频链表
add_init_work(task_mid, 0, init_fn);    // 注册一次性初始化
```

任务处理函数遍历链表依次调用每个注册的处理函数。这是一种**协作式调度**模型——所有模块共享同一个线程，不需要锁或信号量来保护共享数据（因为它们本身就运行在同一个执行上下文中）。

### 5.3 FW 内部模块通信：三种方式

| 方式 | 机制 | 使用场景 |
|---|---|---|
| **直接函数调用** | 全局 include 路径 + `#include` + 直接调函数 | 紧耦合模块间，如 diag/DmmCalibMode.c `#include "dmm_if_common.h"` 后调用 `Dmm_SetEvent()` |
| **DCMS 话题** | `dcms_mcu_topic_send()` / `dcms_mcu_topic_setup_callback()` | FW内部松耦合（如 fusa→DMM 降级命令）或 MCU↔SOC 跨芯片通信 |
| **AUTOSAR COM 栈直调** | `Com_SendSignal()` / `Com_ReceiveSignal()` | 物理 CAN 总线读写 |

**全局 include 路径机制**能使任何模块 include 任何已启用模块的公共头文件：

```makefile
# common_include_asw.mk
CINCLUDE_FILE += app_core/dmm/asw_app/asw_out_api
CINCLUDE_FILE += app_core/dmm/asw_app/asw_src/
# 当 CONFIG_APP_DMM_ASW_APP=y 时，所有 app_core 模块都能 include DMM 的头文件
```

### 5.4 DCMS 在 FW 侧的角色

DCMS 在 FW 侧既用于 MCU↔SOC 跨芯片通信（主要用途），也用于 FW 内部模块间的松耦合通信。DCMS 的注册表不区分"同芯片"还是"跨芯片"——路由层自动处理。

FW 侧的大部分 DCMS 话题属于 `MINIDCOS_DOMAIN(1)`，用于 OTA 升级、工厂参数、UDS 诊断、复位等系统管理类操作。

### 5.5 FW 关键模块详解

#### SecOC — 安全车载通信

SecOC (Secure On-Board Communication) 是 AUTOSAR 标准中的安全通信模块，用于保护 CAN 总线消息的完整性和新鲜度，防止重放攻击。源码位于 `dsar-sip/dsar_fw/sip_extend/secoc/`。

```
SecOC 工作流程:

  发送端:
    ├── 计算消息的 MAC (消息认证码)
    ├── 从 FVM 获取新鲜度值 (FV, Freshness Value)
    ├── 将 MAC + FV 附加到 CAN 消息
    └── 发送受保护的消息

  接收端:
    ├── 提取 MAC + FV
    ├── 验证 FV 是否新鲜 (防重放攻击)
    ├── 重新计算 MAC 并与收到的 MAC 比对
    └── 验证通过 → 传递消息; 失败 → 上报安全事件

  Shell 调试命令:
    secoc              → 查看 SecOC 统计 (消息数/失败数/重放检测)
    fvm                → 查看 FVM 新鲜度值
```

**源码结构**：
- `secoc_manager/secoc_shell.c` — SecOC 安全统计 shell 命令
- `fvm/` — 新鲜度值管理 (Freshness Value Manager)，维护发送端和接收端的 FV 计数器，确保每条安全消息使用唯一的新鲜度值

#### CAN 网关

CAN 网关负责在不同 CAN 总线之间路由报文。源码位于 `dsar-sip/dsar_fw/sip_extend/can_gateway/`。

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
```

**源码结构**：
- `can_gateway_master_new.c` — CAN 网关主控（新版），负责路由表管理和报文转发
- `net_blackbox.c` — 网络黑匣子，记录异常通信事件

**Shell 命令**：`cangateway_monitor`（网关路由状态）、`net_blackbox`（网络黑匣子）、`dataflow`（数据流监控）、`nm_monitor`（网络管理监控）

#### BSW 管理器 v2

BSW 管理器负责基础软件的模式管理和状态协调。源码位于 `dsar-sip/dsar_fw/sip_extend/bsw_manager_v2/`。

```
BSW 管理器功能:
  ├── ECU 状态处理 (ESH)
  │     ├── 启动/运行/休眠/唤醒 状态机
  │     └── Shell: esh, esh_timer
  │
  ├── 网络管理 (NM)
  │     ├── NM 通道激活/停用
  │     ├── 诊断28服务通信控制 (nm_service_diag28.c)
  │     ├── 数据流监控 (nm_dataflow_monitor.c)
  │     └── Shell: nm_channel, nm_monitor
  │
  ├── 通信控制
  │     ├── 诊断28服务 (通信控制)
  │     └── Shell: comcontrol
  │
  └── 唤醒源管理
        └── Shell: wksrc
```

**源码文件**：
- `bswm_esh_state.c` — ECU 状态处理器 (ESH)
- `bswm_esh_timer.c` — ESH 定时器
- `nm_monitor.c` — 网络管理监控
- `nm_service_diag28.c` — 诊断28服务通信控制
- `nm_channel_active.c` — NM 通道激活/停用
- `nm_dataflow_monitor.c` — 数据流监控

#### ECU Shell 系统（跨 ECU 远程命令）

ECU Shell 提供了一个跨 SOC/MCU 的远程命令系统：

```
命令注册: DECLEAR_CMD 宏 → .shell_soc_cmd 链接器段（SOC侧）/ shell_cmd 注册表（MCU侧）
命令发现: RegisterShell() → 解析 ELF → dlopen/dlsym（SOC侧）
命令调用: dshell → DCOS RPC → shell_req_cb() → handler()
```

SOC 侧的 shell 基础设施（`dsar-plat-bf/dsar_app/cdd/shell_soc/`）：
- `shellsoc_server.cpp` — `shellsoc_s_init` + `shell_req_cb`（服务端，接收远程命令）
- `shellsoc_client.cpp` — `shellsoc_c_init` / `shellsoc_c_do_cmd`（客户端，发起远程命令）
- `shell_soc_register.cpp` — `RegisterShell` → ELF 解析 → `dlsym` 自动发现命令

MCU 侧的 shell 命令分散在各模块中，通过 `#include` 公共头文件获得声明后直接注册。

### 5.6 dsar-sip 的双重身份

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
    lib*_com.so / libuds_v4.so        sailsw3.bin / sailsw1.bin
    (通过 Conan 包的静态库)            (通过 Git Submodule 源码编译)
          │                               │
          ├── AUTOSAR COM (SOC侧)         ├── AUTOSAR COM (MCU侧)
          ├── SIP Extend (SOC侧)          ├── BSW Manager v2
          └── x_dom_*_rt (SOC侧框架)      ├── CAN Gateway
                                          ├── SecOC / FVM
                                          ├── UDS Transport Layer
                                          ├── CAN Communication Platform
                                          ├── System Diagnosis
                                          └── x_dom_*_rt (MCU侧路由)
```

---

## 七、源码组织与各域归属

### 7.1 APP 侧（SOC）

```
src/dsar_app/
├── app_core/
│   ├── CMakeLists.txt              → 根据 CONFIG_* 条件编译子目录
│   ├── app_com_core/               → COM域 (lib*_com.so)
│   │   └── app_com_core.cpp        COM通信核心入口
│   ├── app_ge_core/                → GE域 (lib*_com.so)
│   │   └── app_ge_core.cpp         GE通用功能核心入口
│   ├── app_udsonip/                → UDS域 (libuds_v4.so / lib*_uds_v6.so)
│   │   ├── appl_ipv4/              IPv4应用层
│   │   ├── uds_proxy/             UDS代理
│   │   ├── uds_transfer_v4/       传输层v4
│   │   ├── uds_innerupdate_v4/    内部升级v4
│   │   └── uds_interdiag_v4/      内部诊断v4
│   ├── app_ad_core/                → AD域 (lib*_app.so)
│   │   └── product/faw/.../       车型AD核心 (proxy/adapter/fusa/runnable)
│   └── log_export/                 → 独立进程 dji_log_export
│
├── entry/                          → 可执行程序入口
│   ├── dji_bf_app.cpp              dji_bf_app (加载com.so)
│   ├── dji_ad_app.cpp              dji_ad_app (加载app.so)
│   ├── dji_uds_service_v4.cpp      dji_doip_service_ipv4 (加载uds_v4.so)
│   └── dji_uds_service_v6.cpp      dji_doip_service_ipv6 (加载uds_v6.so)
│
├── consys/
│   ├── consys_bf.cmake             BF产品编译宏 (product_build_marcos, asst_product_build_marcos)
│   └── consys_ad.cmake             AD产品编译宏 (app_product_build_marcos)
│
└── product/faw/                    → FAW OEM 适配
    ├── faw_config.cmake            FAW全局配置
    ├── oem_feature/                OEM通用特性 (所有FAW车型共享)
    ├── fawhq_e001_10/              ← e001_10 车型
    │   ├── fawhq_e001_10_config.cmake  产品级 CONFIG_* 开关
    │   ├── proxy/                  信号代理 (BF侧)
    │   │   ├── x_dom_can_rt_gen_com/   SIP生成CAN信号路由(COM侧)
    │   │   ├── x_dom_someip_rt_gen_com/ SIP生成SOME/IP路由(COM侧)
    │   │   ├── x_dom_can_rt_gen_uds/    SIP生成CAN信号路由(UDS侧)
    │   │   └── x_dom_someip_rt_gen_uds/ SIP生成SOME/IP路由(UDS侧)
    │   ├── autosar_adapter/        AUTOSAR协议栈配置
    │   │   ├── microsar_config_com/    COM协议栈配置
    │   │   └── microsar_config_uds_v4/ UDS v4协议栈配置
    │   ├── oem_product_feature/    OEM产品特性
    │   └── app_adapt/              应用适配器
    └── fawhq_p301/                 另一个车型 (结构类似)
```

### 7.2 FW 侧（MCU）

```
src/dsar_fw/
├── Makefile                       → include build/main.mk
├── build/
│   ├── main.mk                    主入口 (解析 oem/arch 参数)
│   ├── main_sa8xx.mk              SA8650/SA8775 平台编译
│   ├── main_tda4.mk               TDA4 平台编译
│   ├── main_rh850.mk              RH850 平台编译
│   ├── main_x86.mk                x86 模拟编译
│   └── clang/clang.mk             工具链定义
│
├── include/
│   ├── common_include_all.mk      全局 include 路径
│   ├── common_include_autosar_bsw.mk  AUTOSAR BSW 头文件路径
│   └── common_include_dcos.mk     DCOS 头文件路径
│
├── app_core/
│   ├── Makefile                   条件编译子模块 (subdir-y)
│   ├── appl_main.c                任务创建入口
│   ├── appl_worklist.c            工作链表调度
│   ├── dmm/                       诊断管理
│   ├── diag/                      诊断服务
│   ├── fusa/                      功能安全 (降级表等)
│   ├── can_com_platform/          CAN通信平台
│   ├── can_gateway/               CAN网关 (SA8650/SA8775)
│   ├── secoc/                     Secure O-C通信
│   ├── osal/                      OS抽象层 (FreeRTOS/SafeRTOS)
│   ├── bsw_manager_v2/            BSW管理器 (NM/ESH)
│   ├── pm_sync/                   电源管理同步
│   ├── rtmon/                     运行时监控
│   └── version_app/               版本信息
│
├── proxy_comm/                    代理通信层
├── ctrl/                          控制逻辑
├── xwire/                         执行器旁路/车辆信息
│
└── product/faw/                   FAW OEM 适配
    └── fawhq_e001_10/
        ├── Makefile + .config     Kconfig配置
        ├── proxy/                 信号代理 (MCU侧)
        ├── adapter/               适配器 (MCU侧)
        ├── config/                应用配置
        ├── x_dom_can_rt_config/   SIP生成: CAN信号路由表
        ├── x_dom_someip_rt_config/ SIP生成: SOMEIP信号路由表
        └── microsar_config/       AUTOSAR MicroSAR配置
```

### 7.3 关键区分：同名的 proxy/adapter 在两个产品目录下

容易被混淆的重要细节：

| 组件 | COM/GE域使用的源码路径 | AD域使用的源码路径 |
|---|---|---|
| proxy | `product/faw/fawhq_e001_10/proxy/` | `app_core/app_ad_core/product/faw/fawhq_e001_10/proxy/` |
| adapter | `product/faw/fawhq_e001_10/adapter/` | `app_core/app_ad_core/product/faw/fawhq_e001_10/adapter/` |
| param | PAL生成到 `product/.../proxy/` 附近 | PAL生成到 `app_ad_core/product/.../param/` |

两者是**不同的源码目录、不同的 CMake target、编译进不同的 .so**。COM 侧的 proxy 处理通信层信号路由，AD 侧的 proxy 处理应用层车辆信号语义适配。

---

## 八、平台仓三大子项目概览

平台仓 `dsar-hq-plat` 包含三个独立子项目，通过 Conan 包管理器发布：

### 8.1 dsar-plat-ad — AD 平台（自动驾驶应用框架）

**Conan 包**: `dsar-plat/<version>@TPV1/release` (v2.1261.0-beta.87)

提供 SOC 侧自动驾驶应用的**核心框架代码**，不包含车型特定的信号实现。

```
dsar-hq-plat/dsar-plat-ad/src/dsar_app/
├── app_core/app_ad_core/                 ★ AD 应用核心框架
│   ├── dsar_plat/                         平台抽象层 (proxy/business/sim/extension)
│   │   └── dsar_plat_config.cmake         Conan 包桥接 + PAL 路径配置
│   ├── asw_app_core/                      应用软件核心
│   ├── mode_manager/                      模式管理器
│   ├── domain_function/                   领域功能 (gateway_hmicom/mqttcom)
│   ├── dmm_common/                        DMM 公共模块
│   ├── plat_app_interface/               平台应用接口
│   └── fusa_app/                         功能安全应用
│
├── app_core/app_ge_core/gb_dssad/        DSSAD 国标数据记录公共代码
│   ├── edr_data_process.cpp              主循环入口 (50ms周期)
│   ├── dssad_event_trigger.c/h           事件检测引擎 (7种事件边沿检测)
│   ├── dssad_data_pack.c/h               JSON 序列化 + 发送
│   ├── dssad_common.c/h                  函数注册 + 工具宏
│   └── cJSON.c/h                         JSON 库
│
└── platform_desc.json                    平台描述 (编译工具链/系统配置)
```

### 8.2 dsar-plat-bf — BF 平台（基础功能 + 诊断 + CDD）

**Conan 包**: `dsar-plat-bf/<version>@TPV1/release` (v2.31.0-beta.6)

提供基础功能模块：诊断通信 (DoIP/UDS)、复杂驱动 (CDD)、PAL 工具、Shell 基础设施等。

```
dsar-hq-plat/dsar-plat-bf/dsar_app/
├── cdd/                                   ★ 复杂驱动层 (SOC 侧)
│   ├── dcms_adapt/                          DCMS 适配 (跨核通信 SOC 端)
│   ├── system/                             系统管理 (生命周期/健康监控)
│   ├── dump/                               异常转储
│   ├── recorder/                           数据记录
│   ├── log/                                日志系统
│   ├── x_dom_can_rt/                       ★ 预生成的 CAN 信号路由代码 (SOC 侧)
│   ├── x_dom_someip_rt/                    ★ 预生成的 SOMEIP 信号路由代码 (SOC 侧)
│   ├── pal/                                PAL (参数抽象层) 生成代码
│   ├── shell_soc/                          ★ SOC Shell 服务/客户端实现
│   ├── lifecycle_manager/                 生命周期管理
│   ├── init/                              初始化模块
│   ├── var_dbg/                           全局变量调试 (elf解析工具)
│   ├── trace/                             轨迹记录
│   └── exception_dump/                    异常转储
│
├── com/                                    SOMEIP 通信框架
├── diag/
│   ├── udsonip/                            DoIP (UDS over IP) 协议栈
│   └── log_export/                         日志导出
│
├── ge/                                     ★ GE (通用事件) 进程模块
│   ├── gb_dssad/                           DSSAD 公共代码 (与 ad 侧共用)
│   ├── app_calib/                          应用标定
│   ├── fl_eFenceDownload/                  FL 围栏下载
│   └── sys_self_diag/                      系统自诊断
│
├── tool/export/dsar_shell/                 ★ dshell 终端工具
│   ├── dsar_shell_simple_history.cpp       dshell 主程序 (main, 命令分发)
│   ├── shell_pal.cpp/h                     PAL 参数表命令
│   └── config/sa86xx/                      SA8650 平台配置
│
└── include/dsar_plat_bf/
    ├── cdd/shell_soc_cmd.h                 DECLEAR_CMD 宏 (SOC侧)
    ├── cdd/dcms_mcu_api.h                  DCMS Topic API
    └── ...
```

### 8.3 dsar-sip — SIP 协议栈（信号接口协议 + AUTOSAR 通信）

**发布方式**: Git Submodule（源码参与编译）。同时存在于 SOC 和 MCU 两侧。

```
dsar-hq-plat/dsar-sip/
├── dsar_app/                               SOC 侧 SIP 模块
│   ├── sip_autosar/                         AUTOSAR 信号路由 (SOC侧)
│   └── sip_extend/
│       ├── app_reset/                       应用复位
│       └── include/                         SOC 侧 SIP 头文件
│
├── dsar_fw/                                ★ MCU 侧 SIP 模块 (编译进固件)
│   ├── sip_autosar/                         AUTOSAR 信号路由 (MCU侧)
│   └── sip_extend/
│       ├── bsw_manager_v2/                  BSW 管理器 v2 + NM/ESH shell 命令
│       ├── can_com_platform/                CAN 通信平台 + FL 通信
│       ├── can_gateway/                     CAN 网关 (master/new) + shell 命令
│       ├── diag/
│       │   ├── SysDiag/                     系统诊断 (车辆诊断功能/故障状态)
│       │   └── UdsTp/                       UDS 传输层
│       ├── osal/
│       │   ├── freertos/                    FreeRTOS OS 抽象层
│       │   └── safertos/                    SafeRTOS OS 抽象层
│       ├── secoc/
│       │   ├── secoc_manager/               SecOC 安全管理器 + FVM
│       │   └── fvm/                         新鲜度值管理 (Freshness Value Manager)
│       ├── sim/                             SIP 仿真
│       ├── timesync/                        时间同步
│       ├── x_dom_can_rt/                    MCU 侧 CAN 信号路由
│       └── x_dom_someip_rt/                 MCU 侧 SOMEIP 信号路由
│
└── include/                                 跨 SOC/MCU 的公共头文件
```

### 8.4 三大子项目依赖关系

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

### 8.5 各子项目在编译产物中的分布

| 编译产物 | dsar-plat-ad | dsar-plat-bf | dsar-sip | dsar-bsw | dsar-hq 适配代码 |
|---------|:-----------:|:-----------:|:--------:|:--------:|:--------------:|
| MCU 固件 (sailsw3.bin) | | | ★ (源码) | ★ (源码) | ★ (源码) |
| lib*_com.so | | ★ (预编译库) | ★ (预编译库) | | ★ (源码) |
| libuds_v4.so | | ★ (预编译库) | | | ★ (源码) |
| lib*_app.so | ★ (预编译库) | | ★ (间接) | | ★ (源码) |
| dshell | | ★ (源码编译) | | | |

---

## 九、关键文件索引

```
=== 构建配置 ===
dsar-hq/conanfile.py                           Conan 依赖声明 + 构建流程入口
dsar-hq/conan_version_dsar_plat.py             dsar-plat-ad 版本号
dsar-hq/conan_version_dsar_plat_bf.py          dsar-plat-bf 版本号
dsar-hq/conan_version_dsar_plat_ad.py          dsar-plat-ad 版本号
dsar-hq/conan_version_dsar_bsw.py              dsar-bsw 版本号
dsar-hq/conan_product_requirements.py          各车型依赖版本映射
dsar-hq/conan_config.json                      构建+镜像打包配置

=== MCU 侧编译入口 ===
dsar-hq/src/dsar_fw/build/main.mk              主 Makefile 入口 (解析 oem/arch)
dsar-hq/src/dsar_fw/build/main_sa8xx.mk        SA8650 MCU 编译 Makefile
dsar-hq/src/dsar_fw/include/common_include_all.mk  全局 include 路径配置
dsar-hq/src/dsar_fw/app_core/appl_main.c        任务创建 (4个RTOS任务)
dsar-hq/src/dsar_fw/app_core/appl_worklist.c    工作链表调度

=== SOC 侧编译入口 ===
dsar-hq/src/dsar_app/CMakeLists.txt            SOC 编译 CMake 顶层入口
dsar-hq/src/dsar_app/consys/consys_ad.cmake    AD 产品编译宏 (app_product_build_marcos)
dsar-hq/src/dsar_app/consys/consys_bf.cmake    BF 产品编译宏 (product_build_marcos)
dsar-hq/src/dsar_app/entry/CMakeLists.txt      可执行程序入口 (dji_ad_app, dji_bf_app, doip)

=== 进程入口 ===
dsar-hq/src/dsar_app/entry/dji_ad_app.cpp      AD 主进程 (加载 app.so)
dsar-hq/src/dsar_app/entry/dji_bf_app.cpp      BF 主进程 (加载 com.so, COM+GE)
dsar-hq/src/dsar_app/entry/dji_uds_service_v4.cpp  UDS v4 诊断服务
dsar-hq/src/dsar_app/entry/dji_uds_service_v6.cpp  UDS v6 诊断服务

=== 车型配置 ===
dsar-hq/src/dsar_app/product/faw/faw_config.cmake        FAW 全局 CONFIG_* (含 APP_GE_PROCESS=ON)
dsar-hq/src/dsar_app/product/faw/fawhq_e001_10/fawhq_e001_10_config.cmake  车型级 CONFIG_* 开关

=== 平台仓桥接 ===
dsar-hq-plat/dsar-plat-ad/src/dsar_app/dsar_plat_ad.cmake   AD 平台安装路径 + PAL 工具
dsar-hq-plat/dsar-plat-bf/dsar_app/dsar_plat_bf.cmake       BF 平台安装路径 + Conan 桥接
dsar-hq-plat/dsar-plat-bf/dsar_app/tool/export/dsar_shell/  dshell 终端工具
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/                   MCU 侧 AUTOSAR 协议栈

=== FW 关键模块 ===
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/bsw_manager_v2/    BSW 管理器 (NM/ESH)
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/can_gateway/       CAN 网关
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/secoc/             SecOC 安全通信
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/x_dom_can_rt/      MCU 侧 CAN 信号路由
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/diag/              系统诊断 + UDS 传输层
```
