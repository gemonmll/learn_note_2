# DSAR-HQ 工程全景概览

---

## 一、工程定位

DSAR-HQ 是自动驾驶域控制器（SA8650 芯片）的**基础软件适配仓**，负责将平台公共代码（`dsar-hq-plat`）适配到具体车型。核心工作包括：

- **行泊车信号适配**：CAN / SOMEIP 信号的路由、转换、代理
- **诊断通信**：UDS (ISO 14229) 诊断服务的 DoIP 实现
- **DSSAD 数据记录**：国标 GB/T 要求的自动驾驶数据存储系统
- **跨核通信**：SOC (QNX/Linux) ↔ MCU (RTOS) 之间的 DCMS 消息传输
- **ECU Shell**：跨 ECU 的远程调试/诊断命令系统

### 1.1 双仓协作模型

```
dsar_app_fw_platform_vip/                     ← 工作空间
├── dsar-hq/          ← ★ 适配仓（本项目）      车型适配代码 + 编译入口
└── dsar-hq-plat/      ← 平台仓                 公共平台代码，发布为 Conan 包
```

- **平台仓 (dsar-hq-plat)**：提供公共框架、算法、协议栈，通过 Conan 包管理器发布
- **适配仓 (dsar-hq)**：依赖平台仓的 Conan 包，添加车型特有的信号适配、配置、shell 命令

---

## 二、芯片与操作系统

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

---

## 三、适配仓 (dsar-hq) 目录结构

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

## 四、平台仓 (dsar-hq-plat) 三大子项目

平台仓不是单一的代码库，而是**三个独立的 Conan 包**，分别负责不同层次的功能：

### 4.1 dsar-plat-ad — AD 平台（SOC 侧自动驾驶应用）

**Conan 包**: `dsar-plat/<version>@TPV1/release` (v2.1261.0-beta.87)

提供自动驾驶应用的核心框架代码，被 SOC 侧的 `dsar_app` 编译链接。

```
dsar-hq-plat/dsar-plat-ad/src/dsar_app/
├── app_core/app_ad_core/               ★ AD 应用核心框架
│   ├── dsar_plat/                       平台适配层 (proxy/business/sim/extension)
│   ├── dsar_plat_config.cmake           Conan 包桥接 + PAL 路径配置
│   ├── asw_app_core/                    应用软件核心
│   ├── mode_manager/                    模式管理器
│   ├── domain_function/                 领域功能 (网关/域控)
│   ├── dmm_common/                      DMM 公共模块
│   ├── plat_app_interface/             平台应用接口
│   └── fusa_app/                       功能安全应用
│
├── app_core/app_ge_core/gb_dssad/      DSSAD (国标数据记录) 公共代码
│   ├── edr_data_process.cpp            主循环入口
│   ├── dssad_event_trigger.c/h         事件检测引擎
│   ├── dssad_data_pack.c/h             JSON 序列化 + 发送
│   ├── dssad_common.c/h                函数注册 + 工具宏
│   └── cJSON.c/h                       JSON 库
│
└── platform_desc.json                  平台描述 (编译工具链/系统配置)
```

### 4.2 dsar-plat-bf — BF 平台（基础功能 + 诊断 + CDD）

**Conan 包**: `dsar-plat-bf/<version>@TPV1/release` (v2.31.0-beta.6)

提供基础功能模块：诊断通信 (DoIP/UDS)、复杂驱动 (CDD)、PAL 工具、Shell 基础设施等。同时服务于 SOC 侧和 MCU 侧。

```
dsar-hq-plat/dsar-plat-bf/dsar_app/
├── cdd/                                ★ 复杂驱动层 (SOC 侧)
│   ├── dcms_adapt/                      DCMS 适配 (跨核通信 SOC 端)
│   ├── system/                         系统管理 (生命周期/健康监控)
│   ├── dump/                           异常转储
│   ├── recorder/                       数据记录
│   ├── log/                            日志系统
│   ├── x_dom_can_rt/                   ★ 预生成的 CAN 信号路由代码 (SOC 侧)
│   ├── x_dom_someip_rt/                ★ 预生成的 SOMEIP 信号路由代码 (SOC 侧)
│   ├── pal/                            PAL (参数抽象层) 生成代码
│   ├── shell_soc/                       SOC Shell 服务/客户端实现
│   ├── lifecycle_manager/             生命周期管理
│   ├── init/                           初始化模块
│   ├── var_dbg/                        全局变量调试 (elf解析工具)
│   ├── trace/                          轨迹记录
│   └── exception_dump/                 异常转储
│
├── com/                                SOMEIP 通信框架
├── diag/
│   ├── udsonip/                        DoIP (UDS over IP) 协议栈
│   └── log_export/                     日志导出
│
├── ge/                                 ★ GE (通用事件) 进程模块
│   ├── gb_dssad/                       DSSAD 公共代码 (与 ad 侧共用)
│   ├── app_calib/                      应用标定
│   ├── fl_eFenceDownload/              FL 围栏下载
│   └── sys_self_diag/                  系统自诊断
│
├── tool/export/dsar_shell/             ★ dshell 终端工具
│   ├── dsar_shell_simple_history.cpp   dshell 主程序 (main, 命令分发)
│   ├── shell_pal.cpp/h                 PAL 参数表命令
│   └── config/sa86xx/                   SA8650 平台配置
│
└── include/dsar_plat_bf/
    ├── cdd/shell_soc_cmd.h             DECLEAR_CMD 宏 (SOC侧)
    ├── cdd/dcms_mcu_api.h              DCMS Topic API
    └── ...
```

### 4.3 dsar-sip — SIP 协议栈（信号接口协议 + AUTOSAR 通信）

**作为 Git Submodule 存在于 dsar-hq 中**，同时源码也在 `dsar-hq-plat/dsar-sip/`。提供 AUTOSAR 标准的信号接口协议 (Signal Interface Protocol) 栈。SOC 和 MCU 两侧都使用。

```
dsar-hq-plat/dsar-sip/
├── dsar_app/                           SOC 侧 SIP 模块
│   ├── sip_autosar/                     AUTOSAR 信号路由 (SOC侧)
│   └── sip_extend/                     SIP 扩展
│       ├── app_reset/                   应用复位
│       └── include/                     SOC 侧 SIP 头文件
│
├── dsar_fw/                            ★ MCU 侧 SIP 模块 (编译进固件)
│   ├── sip_autosar/                     AUTOSAR 信号路由 (MCU侧)
│   └── sip_extend/
│       ├── bsw_manager_v2/             BSW 管理器 v2 + NM/ESH shell 命令
│       ├── can_com_platform/           CAN 通信平台 + FL 通信
│       ├── can_gateway/                 CAN 网关 (master/new) + shell 命令
│       ├── diag/
│       │   ├── SysDiag/                系统诊断 (车辆诊断功能/故障状态)
│       │   └── UdsTp/                  UDS 传输层
│       ├── osal/
│       │   ├── freertos/               FreeRTOS OS 抽象层
│       │   └── safertos/               SafeRTOS OS 抽象层
│       ├── secoc/
│       │   ├── secoc_manager/          SecOC 安全管理器 + FVM
│       │   └── fvm/                    新鲜度值管理 (Freshness Value Manager)
│       ├── sim/                        SIP 仿真
│       ├── timesync/                   时间同步
│       ├── x_dom_can_rt/               MCU 侧 CAN 信号路由
│       └── x_dom_someip_rt/            MCU 侧 SOMEIP 信号路由
│
└── include/                            跨 SOC/MCU 的公共头文件
```

---

## 五、Git Submodules

dsar-hq 适配仓包含以下 Git Submodules：

| Submodule | 路径 | 说明 |
|-----------|------|------|
| **dsar-bsw** | `src/dsar_fw/bsp_al/bsp_core` 附近 | AUTOSAR BSW (基础软件) — MCU 侧底层驱动和 AUTOSAR 协议栈 |
| **dsar-sip** | 通过 SIP_EXTEND_LIB_PATH 引用 | SIP 协议栈 — AUTOSAR 信号接口协议（CAN/SOMEIP 信号路由） |
| **control** | `src/dsar_app/app_core/app_ad_core/ctrl/` | 控制算法模块 |

---

## 六、关键文件索引

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
dsar-hq/src/dsar_fw/build/main_sa8xx.mk        SA8650 MCU 编译 Makefile 入口
dsar-hq/src/dsar_fw/include/common_include_all.mk  MCU 公共头文件路径配置

=== SOC 侧编译入口 ===
dsar-hq/src/dsar_app/CMakeLists.txt            SOC 编译 CMake 顶层入口
dsar-hq/src/dsar_app/consys/consys_ad.cmake    AD 产品编译宏 (app_product_build_marcos)
dsar-hq/src/dsar_app/consys/consys_bf.cmake    BF 产品编译宏 (product_build_marcos)
dsar-hq/src/dsar_app/entry/CMakeLists.txt      可执行程序入口 (dji_ad_app, dji_bf_app, doip)

=== 平台仓桥接 ===
dsar-hq-plat/dsar-plat-ad/src/dsar_app/dsar_plat_ad.cmake   AD 平台安装路径 + PAL 工具
dsar-hq-plat/dsar-plat-ad/src/dsar_app/dsar_plat_bf.cmake   BF 平台安装路径 + Conan 桥接
dsar-hq-plat/dsar-plat-bf/dsar_app/tool/export/dsar_shell/  dshell 终端工具
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/                   MCU 侧 AUTOSAR 协议栈

=== 构建过程分析 ===
dsar-hq-buildlog/buildlog.txt                  完整编译日志 (Conan 依赖解析 + 编译 + 打包)
```
