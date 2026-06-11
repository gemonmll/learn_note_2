# ECU Shell 完整架构设计与使用指南

---

## 一、概述

ECU Shell 是一个跨 ECU 的远程调试/诊断命令系统。它的核心设计思想是：**在板子启动后，通过终端连接到一个交互式 shell 进程，可以调用代码中预定义的函数（类似打桩/注入机制），读写内部变量、触发事件、发送 DCMS 消息等。**

ECU Shell 分为两套独立但通过 DCMS topic 互通的子系统：

```
┌──────────────────────────────────────────────────────────────────┐
│                      ECU Shell 体系总览                           │
│                                                                   │
│  SOC/APP 侧 (HPC, QNX/Linux)           MCU/FW 侧 (SA8650 RTOS)   │
│  ═══════════════════════════           ═══════════════════════   │
│                                                                   │
│  [dshell 交互终端]                     [mc_shell 交互终端]         │
│       │                                     │                     │
│       ├─ PAL 命令 (本地参数表)               ├─ DECLEAR_CMD 遍历    │
│       │  pal_tbl_read / pal_item_write       │  .shellcmd 段        │
│       │                                      │                     │
│       ├─ DCOS RPC → APP .so 服务             ├─ 直接调用 handler()  │
│       │  ┌──────────────┐                    │  ┌──────────────┐   │
│       │  │ dji_uds_v4   │                    │  │ factory_param │   │
│       │  │ dji_uds_v6   │                    │  │ dssad_set_... │   │
│       │  │ dji_ad_app   │                    │  │ secoc / fvm   │   │
│       │  │ lidar_proxy  │                    │  │ nm_monitor ...│   │
│       │  └──────────────┘                    │  └──────┬───────┘   │
│       │                                      │         │           │
│       └─ UDP/IPC ────────────────────────────→ MCU shell server   │
│                                              (dsar/dsar_mcu设备)   │
│                                                                   │
│       ┌──────── DCMS Topic 互通 (跨核) ─────────────┐             │
│       │                                              │             │
│       │  SOC 订阅 DCMS_FACTORY_PARAM_SET  ←──────────┤─ MCU 发布   │
│       │  SOC 发布 DCMS_FACTORY_PARAM_SET_RESULT ─────→│─ MCU 订阅  │
│       │  SOC 订阅 DCMS_DSSAD_TRIGGER_DEBUG ←─────────┤─ MCU 发布   │
│       └──────────────────────────────────────────────┘             │
└──────────────────────────────────────────────────────────────────┘
```

### 1.1 两种使用场景

| 场景 | 命令入口 | 目标 | 通信方式 |
|------|---------|------|---------|
| **SOC Shell** | `dshell -d dsar_app` | SOC 本地 APP 服务 | DCOS RPC (SOMEIP) |
| **MCU Shell** | `dshell -d dsar` / `dshell -d dsar_mcu` | MCU 固件中的 shell 命令 | UDP/IPC |
| **单次执行** | `dshell -s "cmd"` | 取决于 -d 参数 | 同上 |

---

## 二、FW (MCU) 侧 ECU Shell 架构

### 2.1 设计理念

MCU 侧的 ECU Shell 运行在 RTOS 上，shell 命令代码直接编译进固件。采用**链接器段自动注册**机制——写命令的人只需用 `DECLEAR_CMD` 宏声明，不需要手动写任何注册代码。

### 2.2 命令注册机制

**宏定义** (来自 `mc_shell/shell.h`):

```c
// shell_cmd 结构体定义
struct shell_cmd
{
    const char *cmd_name;       // 命令名
    uint8_t max_args;           // 最大参数个数
    uint16_t flags;             // 标志位
    const char *description;    // 命令描述
    const char *usage;          // 使用说明
    int32_t (*do_cmd)(const struct mc_console *csl,
                      const char *const *const argv,
                      int32_t argc);  // 命令处理函数指针
};

// 注册宏: 将 shell_cmd 结构体放入 .shellcmd 段
#define DECLEAR_CMD(_name, _max_args, _flag, _do_cmd, _desc, _usage) \
    __DATA_SECTION(shell_cmd_##_name, ".shellcmd")                   \
    const struct shell_cmd shell_cmd_##_name = {                     \
        #_name, _max_args, _flag, _desc, _usage, _do_cmd             \
    }
```

**关键设计**：`__DATA_SECTION(..., ".shellcmd")` 将每个命令结构体放入独立的 `.shellcmd` 链接器段。MCU 固件链接时，所有分散在各 `.c` 文件中的命令结构体被收集到一段连续内存中。Shell 运行时只需遍历这个段，即可发现所有命令。

**编译开关**：所有 shell 命令代码被 `#ifdef MC_SHELL_SUPPORT` 包裹，生产版本可以裁剪。

### 2.3 命令处理函数模板

```c
#ifdef MC_SHELL_SUPPORT
#include "mc_shell/shell.h"

static int32_t do_my_command(const struct mc_console *csl,
                              const char *const *const arg,
                              int32_t arg_size)
{
    // 1. 参数校验
    if (arg_size < 2)
    {
        shell_print("usage: my_command <param1> <param2>\r\n");
        return -22;
    }

    // 2. 解析参数 (arg[0]是命令名本身, arg[1]起是实际参数)
    uint32_t val = simple_strtoul(arg[1], NULL, 0);

    // 3. 执行业务逻辑 — 可以直接调用任何固件内部函数
    //    ...

    // 4. 输出结果
    shell_print("result: val=%d\r\n", val);
    return 0;
}

// 注册命令
DECLEAR_CMD(my_command, 5, 0, do_my_command,
            "my_command", "my_command <param1> <param2>");
#endif
```

### 2.4 shell_print 输出机制

MCU 侧的 `shell_print` 本质是 `printf`：

```c
#define shell_print printf
```

输出通过串口/UDP 回传给 `dshell` 终端。

### 2.5 在 Shell 命令中发送 DCMS Topic

MCU 侧的命令处理函数可以直接调用 `dcms_mcu_topic_send_msg()` 向 SOC 侧发送消息。这是最常用的跨核通信方式：

```c
// 构造消息
dcms_mcu_msg_t dcms_mcu_msg = {0};
dcms_mcu_msg.data    = (uint8_t *)&my_data;
dcms_mcu_msg.datalen = sizeof(my_data);

// 发送
int ret = dcms_mcu_topic_send_msg(TOPIC_ID, &dcms_mcu_msg);
shell_print("topic send ret:%d\r\n", ret);
```

### 2.6 已注册的 MCU Shell 命令一览

**适配仓 (dsar-hq/src/dsar_fw/):**

| 命令名 | 源文件 | 功能 |
|--------|--------|------|
| `factory_param_nvm_set` | Diag_CrossEcuShell.c | 写工厂参数到 SOC 侧 NVM |
| `hexfactory_param_nvm_set` | Diag_CrossEcuShell.c | 按字节写工厂参数 |
| `factory_param_nvm_get` | Diag_CrossEcuShell.c | 查询工厂参数写入结果 |
| `dssad_set_event_status` | Diag_CrossEcuShell.c | DSSAD 事件调试注入 |
| `set_event_status` | DemProcess.c | 设置 DEM 事件状态 |
| `get_event_status` | DemProcess.c | 查询 DEM 事件状态 |
| `get_nvm_block_status` | DemProcess.c | 查询 NVM 块状态 |
| `dtc_enable_debug` | Diag_DTCEnable.c | DTC 使能调试 |
| `calib_debug_precondition` | DiagCalib_Main.c | 标定前置条件调试 |
| `start_static_topview_calib` | DiagCalib_Main.c | 启动静态全景标定 |
| `version_app` | version_app.c | 版本信息查询 |
| `appl_worklist` | appl_worklist.c | 工作列表调试 |
| `act_proxy` | acu_shell.c | 执行器旁路控制 |
| `dmm_degrade_status` | degrade_table_main.c | 降级状态查询 |
| `rtmon_debug` | rtmon_remote.c | 运行时监控调试 |
| `pm_sync` | pm_sync_sys_param.c | 参数同步触发 |

**平台仓 (dsar-hq-plat/dsar-sip/dsar_fw/):**

| 命令名 | 源文件 | 功能 |
|--------|--------|------|
| `x_dom_can_rt_dbg` | msg_route_mcu.c | CAN 信号路由调试 |
| `cangateway_monitor` | can_gateway_master_new.c | CAN 网关监控 |
| `secoc` | secoc_shell.c | SecOC 安全统计 |
| `fvm` | secoc_manager.c | 新鲜度值管理 |
| `nm_monitor` | nm_monitor.c | 网络管理监控 |
| `comcontrol` | nm_service_diag28.c | 通信控制 |
| `nm_channel` | nm_channel_active.c | NM 通道控制 |
| `dataflow` | nm_dataflow_monitor.c | 数据流监控 |
| `net_blackbox` | net_blackbox.c | 网络黑匣子 |
| `esh` | bswm_esh_state.c | ECU 状态处理器 |
| `esh_timer` | bswm_esh_timer.c | ESH 定时器调试 |
| `com_fl_mcu` | com_fl_mcu.c | FL 通信调试 |
| `veh_diag_get_func_status` | SysDiag_Common.c | 车辆诊断功能状态 |
| `veh_diag_set_fault_status` | SysDiag_Common.c | 设置故障状态 |
| `can_detect_platform` | can_detect_platform.c | CAN 检测调试 |
| `wksrc` | ecum.c | 唤醒源查询 |

---

## 三、APP (SOC) 侧 ECU Shell 架构

### 3.1 设计理念

SOC 侧 ECU Shell 与 MCU 侧的关键区别：

1. **SOC 侧是多进程架构**：每个 APP 服务编译为独立的 `.so` 动态库 (`libfawhq_p301_com.so`、`libdji_uds_v4.so` 等)，命令分布在不同 `.so` 中
2. **通过 DCOS RPC 远程调用**：`dshell` 工具不直接调用函数，而是通过 DCOS (SOMEIP) 向目标服务发起 RPC 请求
3. **命令发现是动态的**：目标服务启动时自解析 ELF 的 `.shell_soc_cmd` 段，将命令信息通过 DCOS 暴露给 `dshell`

### 3.2 整体架构图

```
┌────────────────────────────────────────────────────────────────────────┐
│                        SOC APP 侧 ECU Shell                             │
│                                                                         │
│  [dshell 终端进程]           [各个 APP .so 动态库进程]                     │
│  ════════════════           ═══════════════════════                     │
│                                                                         │
│  main()                                                                │
│    │                                                                    │
│    ├─ arg_parse()              libfawhq_p301_com.so                     │
│    │    -d dsar_app              │  [启动时]                             │
│    │                              ├─ dcos::Init("dji_ad_app")           │
│    ├─ dcos::Init("ecu_shell")    ├─ shellsoc_s_init("dji_ad_app", 0)   │
│    │                              │    ├─ 注册 DCMS topic                │
│    ├─ dshell_soc_init()          │    │   /sys/shell/dji_ad_app/v2      │
│    │    │                         │    ├─ RegisterShell()                │
│    │    ├─ shellsoc_c_init()      │    │    ├─ dladdr() 找到自身 .so     │
│    │    │    启动 DCOS client     │    │    ├─ 解析 ELF 头               │
│    │    │                         │    │    ├─ 遍历段表找到               │
│    │    ├─ get_cur_all_cmds()     │    │    │  .shell_soc_cmd 段         │
│    │    │    DCOS RPC 获取        │    │    ├─ 读取所有 t_shell_cmd_info │
│    │    │    所有已注册命令        │    │    ├─ dlopen+dlsym 解析符号地址 │
│    │    │                         │    │    └─ 写入 CSV /tmp/shell_soc/ │
│    │    └─ 插入 DictionaryTree    │    │                                 │
│    │         (用于 Tab 补全)       │    └─ shell_req_cb() ← DCMS 回调     │
│    │                              │         │                            │
│    ├─ shell_pal_cmd_init()        │         ├─ 收到 "get_cmds" →         │
│    │    注册 PAL 命令              │         │   返回所有命令列表           │
│    │                              │         │                            │
│    └─ while(true) 主循环          │         ├─ 收到 "cmd_name arg" →     │
│         │                         │         │   1. 遍历 g_cmd_info_maps   │
│         ├─ shell_pal_try_exec()   │         │   2. 匹配 cmd_name         │
│         │   (PAL 参数读写)         │         │   3. 调用 handler 函数指针  │
│         │                         │         │   4. 回传 output 字符串     │
│         └─ dshell_soc_try_exec()  │         └────────────────────────    │
│              │                    │                                      
│              ├─ "shell_soc_help"  │                                      
│              ├─ "list <srv>"      │                                      
│              └─ "<srv> <cmd> <arg>"│                                     
│                   │                                                      
│                   └─ shellsoc_c_do_cmd()                                 
│                        │                                                 
│                        └─ DCOS RPC (SOMEIP) ────→ shell_req_cb()        
└────────────────────────────────────────────────────────────────────────┘
```

### 3.3 SOC 侧命令注册机制

SOC 侧同样使用链接器段机制，但比 MCU 侧更复杂——因为涉及动态库的函数地址重定位。

**宏定义** ([shell_soc_cmd.h](dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/cdd/shell_soc_cmd.h#L75)):

```cpp
// C++ 版本命令结构体 (32字节对齐)
typedef struct
{
    char cmd_name[64];      // 命令名
    char desc[64];          // 描述
    char usages[64];        // 用法说明
    char func_name[64];     // 函数名 (字符串，用于 dlsym)
    void *func_addr;        // 函数地址 (段中为偏移，运行时修正)
    uint32_t max_args;      // 最大参数数
    uint32_t flags;         // 标志
    char func_type;         // 0=C++, 1=C
    char rev[47];
} __attribute__((packed)) t_shell_cmd_info;

// 注册宏：放入 .shell_soc_cmd 段
#define DECLEAR_CMD(name, max_args, flags, do_cmd, desc, usage) \
    t_shell_cmd_info \
    __attribute__((section("shell_soc_cmd"))) \
    __attribute__((used)) \
    const shell_cmd_##name = { \
        #name, desc, usage, #do_cmd, \
        (void *)do_cmd, max_args, flags, SHELL_FUNC_CPP \
    };
```

**C++ 命令处理函数签名**：

```cpp
// 必须用 EXPORT_SHELL_SYMBOL 导出符号，确保 dlopen+dlsym 能找到
EXPORT_SHELL_SYMBOL int32_t my_soc_command(std::vector<std::string>& argv,
                                            std::string& output);

// 输出通过 shell_print 宏写入 output 字符串
#define shell_print output += string_format
```

### 3.4 服务启动注册流程

每个 APP `.so` 在启动时调用 `shellsoc_s_init()` 注册为 Shell 服务：

```cpp
// 示例: dji_uds_v4 服务注册 (appl_ipv4_shell.cpp)
RegisterShell("dji_uds_v4", "ipv4");
if (shellsoc_s_init("dji_uds_v4", 1) < 0) {
    DEBUG_ERROR("shellsoc_s_init fail\n");
}
```

**`shellsoc_s_init()` 内部流程** ([shellsoc_server.cpp:145-173](dsar-hq-plat/dsar-plat-bf/dsar_app/cdd/shell_soc/shellsoc_server.cpp#L145-L173))：

1. 原子检查防止重复初始化
2. 注册 DCMS service topic: `/sys/shell/<name>/v2`
3. 注册 DCMS 回调 `shell_req_cb` — 这是接收 `dshell` RPC 请求的入口
4. 调用 `RegisterShell()` → 解析自身 ELF 的 `.shell_soc_cmd` 段 → 通过 `dlopen`/`dlsym` 修正函数地址 → 写入 CSV 缓存文件

**`RegisterShell()` 内部流程** ([shell_soc_register.cpp](dsar-hq-plat/dsar-plat-bf/dsar_app/cdd/shell_soc/shell_soc_register.cpp))：

1. `dladdr()` 获取当前代码所在的 `.so` 路径和基地址
2. 打开 `.so` 文件，解析 ELF 头
3. 遍历段表，找到 `.shell_soc_cmd` 段
4. 读取所有 `t_shell_cmd_info` 结构体
5. 对于 `.so` 文件：通过 `dlopen` + `dlsym` 用函数名字符串解析真实地址
6. 将命令列表写入 `/tmp/shell_soc/<app_name>.<lib_name>` CSV 文件

### 3.5 dshell 终端的命令分发

`dshell` 是运行在 SOC 上的交互终端程序 (编译自 [dsar_shell_simple_history.cpp](dsar-hq-plat/dsar-plat-bf/dsar_app/tool/export/dsar_shell/dsar_shell_simple_history.cpp))。

**启动方式**：

```bash
# 交互模式 - 连接 SOC APP 服务
dshell -d dsar_app

# 交互模式 - 连接 MCU (通过 UDP)
dshell -d dsar

# 单次执行模式
dshell -d dsar_app -s "list dji_uds_v4"
```

**对 `dsar_app` 设备的命令分发** (优先级从高到低)：

| 步骤 | 函数 | 说明 |
|------|------|------|
| 1 | `shell_pal_try_exec_cmd()` | PAL 参数表命令 (pal_tbl_read, pal_item_write...) |
| 2 | `dshell_soc_try_exec_cmd()` | DCOS RPC → APP 服务的注册命令 |

**`dshell_soc_try_exec_cmd()` 支持的命令格式**：

| 输入格式 | 说明 | 示例 |
|---------|------|------|
| `shell_soc_help` | 打印使用帮助 | `shell_soc_help` |
| `list <srv_name> [filter]` | 列出某服务的所有命令 | `list dji_uds_v4` |
| `shell_soc_cmd_do <srv> <cmd> <args>` | 显式执行远程命令 | `shell_soc_cmd_do dji_uds_v4 my_func arg1` |
| `<srv> <cmd> <args>` | 简写形式 (当未匹配 PAL 命令时) | `dji_uds_v4 my_func arg1` |

**对 `dsar`/`dsar_mcu` 设备的命令分发** (优先级从高到低)：

| 步骤 | 函数 | 说明 |
|------|------|------|
| 1 | `shell_pal_try_exec_cmd()` | PAL 参数表命令 |
| 2 | `dshell_soc_try_exec_cmd()` | DCOS RPC |
| 3 | UDP/IPC 透传 | 把原始字符串发给 MCU shell server |

### 3.6 已注册的 SOC APP Shell 命令一览

| 命令名 | 所属服务 | 源文件 | 功能 |
|--------|---------|--------|------|
| `x_dom_can_rt_dbg` | dji_ad_app / dji_uds_v4 / com | x_dom_can_rt_cfg.cpp | CAN 信号路由调试 |
| `com_someip_debug` | dji_ad_app | dmm_someip_service.cpp | SOMEIP 通信调试 |
| `diag_someip_debug` | dji_ad_app | diagnosis_someip_service.cpp | 诊断 SOMEIP 调试 |
| `uss_someip_debug` | dji_ad_app | sensor_someip_service.cpp | 超声波 SOMEIP 调试 |
| `uss_dlog` | dji_ad_app | sensor_someip_service.cpp | 超声波数据日志 |
| `lidar_proxy` | lidar_proxy | lidar_proxy.cpp | 激光雷达代理调试 |
| `someip_sd_debug` | dji_ad_app | someip_monitor_product.cpp | SOMEIP SD 调试 |
| `dtc_someip_detect_product` | dji_ad_app | someip_detect_product.cpp | DTC 检测调试 |
| `fusa_someip_detect_product` | dji_ad_app | someip_detect_product.cpp | 功能安全检测调试 |
| `driving_condition_shell` | dji_ad_app | shell_cmd.cpp | 行车条件调试 |
| `do_heater_monitor` | dji_ad_app | heater_app.cpp | 加热器监控 |
| `do_road_monitor` | dji_ad_app | RoadPreview.cpp | 路面预览监控 |
| `do_dvr_monitor` | dji_ad_app | dvrapp.cpp | DVR 监控 |

### 3.7 PAL 命令 (参数抽象层)

PAL 是 `dshell` 内置的本地参数读写工具，不依赖 DCOS RPC。命令实现在 [shell_pal.cpp](dsar-hq-plat/dsar-plat-bf/dsar_app/tool/export/dsar_shell/shell_pal.cpp)。

| 命令 | 说明 | 示例 |
|------|------|------|
| `pal_help` | 打印 PAL 使用帮助 | `pal_help` |
| `pal_tbl_list` | 列出所有配置表 | `pal_tbl_list` |
| `pal_tbl_read <name> [filter]` | 读取一张配置表 | `pal_tbl_read can_matrix` |
| `pal_tbl_reset <name>` | 重置配置表为默认值 | `pal_tbl_reset can_matrix` |
| `pal_item_read <table> <item>` | 读取单个配置项 | `pal_item_read can_matrix baudrate` |
| `pal_item_write <table> <item> <value>` | 写入单个配置项 | `pal_item_write can_matrix baudrate 500000` |
| `pal_item_get <item>` | 跨表搜索配置项 | `pal_item_get baudrate` |
| `pal_item_set <item> <value>` | 跨表设置配置项 | `pal_item_set baudrate 500000` |

---

## 四、跨核 (SOC ↔ MCU) ECU Shell 通信

### 4.1 通信机制

SOC 和 MCU 之间通过 DCMS Topic (发布/订阅) 实现跨核通信。这是 ECU Shell 体系中最复杂也最重要的部分。

```
SOC (HPC, QNX/Linux)                          MCU (SA8650 RTOS)
══════════════════════                        ══════════════════

shell 命令调用者 (发起方)                     shell 命令执行者 (实际干活的一方)

[dshell → MCU shell]
  dshell -d dsar
    └─ UDP/IPC ──────────────────────────→ mc_shell 接收原始字符串
                                            └─ 遍历 .shellcmd 段匹配命令
                                               └─ do_xxx() 直接执行

[MCU shell → SOC NVM 写入]  (工厂参数场景)
  do_factory_param_nvm_set()
    └─ dcms_mcu_topic_send_msg(              ──DCMS Topic──→  cross_factory_param_set_callback()
         DCMS_FACTORY_PARAM_SET, ...)          [IPC/DCOS]      ├─ 写 NvM
                                                               ├─ Fls_Sync()
                                                               └─ dcms_mcu_topic_send_msg(
                                                                    DCMS_FACTORY_PARAM_SET_RESULT, ...)
                                                                      │
  factory_param_nvm_set_callback() ←──────────DCMS Topic──────────┘
    └─ 缓存结果, 等待 factory_param_nvm_get 查询
```

### 4.2 DCMS Topic 配置

**SOC 侧** (`dcms_mcu_config_uds.h`):

```c
// SOC 订阅 (接收 MCU 发来的写请求)
{0, DCMS_FACTORY_PARAM_SET, "/sys/product_param_set/v1",
 0u, 1u, 0u, 0u, MINIDCOS_DOMAIN}      // subscribe_en=1

// SOC 发布 (向 MCU 返回写结果)
{0, DCMS_FACTORY_PARAM_SET_RESULT, "/sys/product_param_set_result/v1",
 1u, 0u, 0u, 0u, MINIDCOS_DOMAIN}      // publish_en=1
```

**MCU 侧** 对应配置 (在 `product_configs.h` 中):

```c
// MCU 发布 (向 SOC 发写请求)
DCMS_FACTORY_PARAM_SET

// MCU 订阅 (接收 SOC 返回的写结果)
DCMS_FACTORY_PARAM_SET_RESULT
```

### 4.3 跨核 Shell 通信使用的 DCMS Topic ID 段

来自 [dcms_mcu_api.h](dsar-hq-plat/dsar-plat-bf/dsar_app/include/dsar_plat_bf/cdd/dcms_mcu_api.h):

| Topic ID 段 | 用途 |
|-------------|------|
| `DCMS_ADP_TOPIC_ECU_DEBUG` (0x18000) | ECU 调试通用 |
| `DCMS_ADP_TOPIC_GE` (0x20000) | GE 进程 (含 DSSAD) |
| `DCMS_ADP_TOPIC_UDS_V4` (0x30000) | UDS V4 诊断 (含工厂参数) |
| `DCMS_ADP_TOPIC_SHELL_DJI_UDS_V4` | Shell SOC 服务 topic `/sys/shell/dji_uds_v4/v2` |

Shell SOC 服务 topic 命名规则: `/sys/shell/<service_name>/v2`

---

## 五、实战示例 1 — 工厂参数读写

### 5.1 背景

工厂参数（VIN、零件号、软件版本、序列号等）持久化存储在 **SOC 侧的 NVM** (NVRAM) 中。MCU 侧没有直接写 NVM 的能力，需要通过 DCMS topic 委托 SOC 侧执行。

### 5.2 数据流

```
MCU Shell 终端                             SOC APP 服务
══════════════                             ════════════

> factory_param_nvm_set 5 FAW-001
        │
        ▼
do_factory_param_nvm_set()                 cross_factory_param_set_callback()
  ├─ param_id  = 5 (Debug_vin)               ├─ 校验 param_id 范围
  ├─ 构造 msg_factory_param_request_t        ├─ 校验 param_len
  │    .param_id  = 5                        ├─ 拷贝数据到 NVM 缓冲区
  │    .param_len = 17                       ├─ NvM_WriteAll() + Fls_Sync()
  │    .param_data = "FAW-001"               ├─ 构造返回信息字符串
  └─ dcms_mcu_topic_send_msg(                └─ dcms_mcu_topic_send_msg(
       DCMS_FACTORY_PARAM_SET)                    DCMS_FACTORY_PARAM_SET_RESULT)
                                                      │
        ←─────────── 返回结果 ────────────────────────┘

> factory_param_nvm_get
        │
        ▼
do_factory_param_nvm_get()
  └─ shell_print(factory_param_respond_info)
     "write result vin[17] = [hex]46 41 57 2d 30 30 31 ..."
```

### 5.3 关键代码

**MCU 侧 — 发送写请求** ([Diag_CrossEcuShell.c:120-208](dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/DcmApp/Diag_CrossEcuShell.c#L120-L208)):

```c
static int32_t do_factory_param_nvm_set(const struct mc_console *csl,
                                         const char *const *const arg,
                                         int32_t arg_size)
{
    // 1. 解析参数
    uint8_t param_id = simple_strtoul(arg[1], NULL, 0);

    // 2. 拷贝用户输入到参数缓冲区
    for (i = 2; i < arg_size; i++)
    {
        str2uint8(arg[i],
                  &factory_param_info[param_id].param_addr[base_addr],
                  strlen(arg[i]));
        base_addr += strlen(arg[i]);
    }

    // 3. 构造 DCMS 消息
    memcpy(&msg_factory_param_request.param_data[0],
           &factory_param_info[param_id].param_addr[0],
           factory_param_info[param_id].param_len);
    msg_factory_param_request.param_id  = param_id;
    msg_factory_param_request.param_len = factory_param_info[param_id].param_len;

    // 4. 通过 DCMS Topic 发送给 SOC
    dcms_mcu_msg.data    = (uint8_t *)&msg_factory_param_request;
    dcms_mcu_msg.datalen = sizeof(msg_factory_param_request);
    int send_ret = dcms_mcu_topic_send_msg(DCMS_FACTORY_PARAM_SET, &dcms_mcu_msg);

    shell_print("topic send ret:%d. use factory_param_nvm_get to get result!\r\n",
                send_ret);
    return 0;
}
DECLEAR_CMD(factory_param_nvm_set, 50, 0, do_factory_param_nvm_set,
            "factory_param_nvm_set",
            "factory_param_nvm_set [param_id] [datastr1] [datastr2] ...");
```

**SOC 侧 — 接收并执行写入** ([cross_ecu_shell.c:73-140](dsar-hq/src/dsar_app/product/faw/oem_feature/autosar_adapter/microsar_config_uds_v4/Appl/Source/cross_ecu_shell.c#L73-L140)):

```c
int32_t cross_factory_param_set_callback(const uint8_t *data, uint32_t len)
{
    // 1. 反序列化请求
    memcpy(&msg_factory_param_request, &data[0], len);
    param_id = msg_factory_param_request.param_id;

    // 2. 校验
    if ((param_id == 0) || (param_id >= Debug_factory_param_invalid))
        return -22;

    // 3. 写入 NVM 缓冲区
    memcpy(&factory_param_info[param_id].param_addr[0],
           &msg_factory_param_request.param_data[0],
           factory_param_info[param_id].param_len);

    // 4. 特殊处理: VIN/SN 同步标志
    if (param_id == Debug_ecu_serial_number)
        FactoryParam_OemSnSyncFlagSet();
    if (param_id == Debug_vin)
        FactoryParam_OemVinSyncFlagSet();

    // 5. 持久化
    NvM_WriteAll();
    Fls_Sync();

    // 6. 返回结果给 MCU
    cross_factory_param_set_response();  // → dcms_mcu_topic_send_msg(DCMS_FACTORY_PARAM_SET_RESULT)
    return 0;
}
```

### 5.4 可写的工厂参数列表

| param_id | 参数名 | 长度 | 说明 |
|----------|--------|------|------|
| 1 | boot_sw_version | 12 | Boot 软件版本 |
| 2 | part_number | 16 | 零件号 |
| 3 | ecu_sw_version | 16 | ECU 软件版本 |
| 4 | ecu_serial_number | 34 | ECU 序列号 |
| 5 | vin | 17 | 车辆识别码 |
| 6 | ecu_hd_version | 16 | ECU 硬件版本 |
| 7 | repair_shop_code | 10 | 维修店代码 |
| 8 | program_date | 4 | 编程日期 |
| 9 | ecu_install_date | 4 | ECU 安装日期 |
| 10 | ecu_sw_version_reserve | 16 | 软件版本保留 |
| 11 | sys_configure_1 | 16 | 系统配置1 |
| 12 | sys_configure_2 | 16 | 系统配置2 |
| 13 | sys_configure_3 | 16 | 系统配置3 |
| 14 | faw_reserved_f1a7 | 16 | 保留字段 |
| 15 | configure_form | 1 | 配置表单 |

### 5.5 使用示例

```bash
# === MCU Shell 终端 ===

# 写入 VIN (param_id=5, 17字节)
factory_param_nvm_set 5 FAW_0000000000001
# 输出: topic send ret:0.please use factory_param_nvm_get to get write result!

# 查询写入结果
factory_param_nvm_get
# 输出: write result vin[17] = [hex]46 41 57 5f 30 30 30 30 30 30 30 30 30 30 30 31 ...

# 写入零件号 (param_id=2, 16字节)
factory_param_nvm_set 2 DJI_PART_0000001

# 写入序列号 (param_id=4, 34字节)
factory_param_nvm_set 4 SN12345678901234567890123456789012

# 用十六进制方式分批写入 (适合长参数)
hexfactory_param_nvm_set 4 0 0x53 0x4E 0x31 0x32 0x33 0x34 0x35 0x36
hexfactory_param_nvm_set 4 8 0x37 0x38 0x39 0x30 0x31 0x32 0x33 0x34

# 清除参数 (写入全0)
factory_param_nvm_set 5 /
```

---

## 六、实战示例 2 — DSSAD 事件调试注入

### 6.1 背景

DSSAD (Data Storage System for Automated Driving) 是国标 GB/T XXXXX 要求的自动驾驶数据记录系统。正常工作时由真实 CAN 信号和 ADAS 状态驱动事件触发。调试时需要绕过真实信号，手动注入事件。

### 6.2 调试注入机制

DSSAD 内置了调试开关 `dssad_event_debug_flag`（[dssad_event_trigger.c:33](dsar-hq-plat/v2.38.1240-p301-hotfix7-step4/dsar-plat-bf/dsar_app/ge/gb_dssad/dssad_event_trigger.c#L33)）：

- **调试模式关闭** (`dssad_event_debug_flag = 0`)：通过 `gDssadConfig` 函数指针读取真实信号
- **调试模式开启** (`dssad_event_debug_flag = 1`)：从 `dssad_event_debug_data[32]` 数组读取所有事件状态，不读真实信号

### 6.3 完整链路

```
MCU Shell                               SOC DSSAD (GE进程)
════════                                 ══════════════════

> dssad_set_event_status 0xFF 1
        │                                      │
        ▼                                      │
do_dssad_set_event_status()                    │
  ├─ data[0] = 0xFF  (event=0xFF 表示开关)      │
  ├─ data[1] = 0x01  (status=1 表示使能)        │
  └─ dcms_mcu_topic_send_msg(        ──→     dssad_trigger_debug_callback()
       DCMS_DSSAD_TRIGGER_DEBUG)      [DCMS]    ├─ dssad_event_debug_flag = 1
                                                └─ 打印调试位映射表

> dssad_set_event_status 6 1
        │                                      │
        ▼                                      ▼
  data = {0x06, 0x01}                dssad_event_debug_data[6] = 1
  dcms_mcu_topic_send_msg(           (Debug_Collision_V0 = 1)
    DCMS_DSSAD_TRIGGER_DEBUG)
                                                │
                                   dssad_event_status_update()
                                     ├─ [调试模式] 从 dssad_event_debug_data[] 取值
                                     ├─ collision = debug_data[6] = 1
                                     └─ → 后续边沿检测触发 DSSAD 记录

> dssad_set_event_status 0xFF 0
        │                                      │
        ▼                                      ▼
  data = {0xFF, 0x00}                dssad_event_debug_flag = 0
                                                └─ 退出调试, 恢复真实信号
```

### 6.4 关键代码

**MCU 侧 — Shell 命令** ([Diag_CrossEcuShell.c:315-389](dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/DcmApp/Diag_CrossEcuShell.c#L315-L389)):

```c
static int32_t do_dssad_set_event_status(const struct mc_console *csl,
                                          const char *const *const arg,
                                          int32_t arg_size)
{
    uint8_t event  = simple_strtoul(arg[1], NULL, 0);
    uint8_t status = simple_strtoul(arg[2], NULL, 0);

    uint8_t data[2] = {event, status};

    dcms_mcu_msg_t dcms_mcu_msg = {0};
    dcms_mcu_msg.data    = data;
    dcms_mcu_msg.datalen = sizeof(data);

    // 通过 DCMS topic 发送到 SOC 侧的 DSSAD
    dcms_mcu_topic_send_msg(DCMS_DSSAD_TRIGGER_DEBUG, &dcms_mcu_msg);
    return 0;
}
DECLEAR_CMD(dssad_set_event_status, 6, 0, do_dssad_set_event_status,
            "dssad_set_event_status",
            "dssad_set_event_status {event} {status}");
```

**SOC 侧 — DCMS Topic 回调** ([dssad_event_trigger.c:860-958](dsar-hq-plat/v2.38.1240-p301-hotfix7-step4/dsar-plat-bf/dsar_app/ge/gb_dssad/dssad_event_trigger.c#L860-L958)):

```c
// 初始化时注册回调
static void dssad_trigger_init(void)
{
    dcms_mcu_topic_setup_callback(DCMS_DSSAD_TRIGGER_DEBUG,
                                   dssad_trigger_debug_callback,
                                   NULL, 0u);
}

// 接收 MCU 发来的调试指令
int32_t dssad_trigger_debug_callback(const uint8_t* data, uint32_t len)
{
    uint8_t event  = data[0];
    uint8_t status = data[1];

    if (event == 0xFF)  // 开关调试模式
    {
        dssad_event_debug_flag = status;
    }
    else if (event < Debug_Max_Event_Num)  // 注入事件状态
    {
        dssad_event_debug_data[event] = status;
    }
    return 0;
}
```

### 6.5 调试 Byte Index 映射表 (V0 版本)

> **注意**: 此表仅适用于 V0 项目 (如 p301, `product_version_num = DSSAD_VERSION_ORIGINAL_V0`)。V1 项目请参见 6.6 节。

| Index | 枚举名 | 对应 DSSAD 事件 | 取值 |
|-------|--------|----------------|------|
| 0 | Debug_None | (保留) | — |
| 1 | Debug_ADS_Activate_V0 | ADS 自动激活 | bit 位对应 32 个子事件 |
| 2 | Debug_ADS_Exit_V0 | ADS 自动退出 | bit 位对应 32 个子事件 |
| 3 | Debug_InterventionRequest_V0 | 介入请求 | 0/1/2/3 递增 |
| 4 | Debug_MinimumRiskStrategy_V0 | 最小风险策略 | 0/1 |
| 5 | Debug_ADS_SeriousFailure_V0 | ADS 严重失效 | bit 位对应 64 个 DTC |
| 6 | Debug_Collision_V0 | 碰撞 | 0/1 |
| 7 | Debug_RiskOfCollision_V0 | 碰撞风险 | 0/1 |
| 8 | Debug_ADS_ActivatedByDriver_V0 | 驾驶员操作激活 | 0/1 |
| 9 | Debug_ADS_ExitedByDriver_V0 | 驾驶员操作退出 | 0/1 |
| 10 | Debug_Lock_V0 | 锁车 | 0/1 |
| 0xFF | (特殊) | 开关调试模式 | 0=关, 1=开 |

### 6.6 DSSAD V1 (新国标) 版本的调试映射表

V1 版本 (如 p567, `product_version_num = DSSAD_VERSION_V1_01`) 的 index 与 V0 **完全不同**，不仅新增了事件，已有事件的 index 也全部重新编号。

> **关键**: V1 的 index 不连续（7,8,10,20-29），不是 V0 的 1-10。用错 index 会导致数据写入 `dssad_event_debug_data[错误的index]`，V1 代码读取的是正确的 index → 永远读不到你设置的值 → 永远不会触发。

| Index | 枚举名 | 对应 DSSAD V1 事件 | 等效 V0 |
|-------|--------|-------------------|---------|
| 0 | Debug_None_V1 | (保留) | 0 |
| **7** | Debug_Collision_V1 | 碰撞 (时间段+时间戳) | 6 |
| **8** | Debug_RiskOfCollision_V1 | 碰撞风险 (时间段+时间戳) | 7 |
| **10** | Debug_Lock_V1 | 锁车 (时间段+时间戳) | 10 |
| **20** | Debug_ADS_Activate_V1 | ADS 自动激活 | 1 |
| **21** | Debug_ADS_Exit_V1 | ADS 自动退出 | 2 |
| **22** | Debug_ADS_ExitedByDriver_Extend_V1 | ADS 驾驶员退出扩展 (新增) | — |
| **23** | Debug_HOR_Activate_V1 | HOR 激活 (新增) | — |
| **24** | Debug_HOR_Exit_V1 | HOR 退出 (新增) | — |
| **25** | Debug_EOR_Activate_V1 | EOR 激活 (新增) | — |
| **26** | Debug_EOR_Exit_V1 | EOR 退出 (新增) | — |
| **27** | Debug_DCA_Activate_V1 | DCA 激活 (新增) | — |
| **28** | Debug_RMF_Activate_V1 | RMF 激活 (新增) | — |
| **29** | Debug_ADS_SeriousFailure_V1 | ADS 严重失效 | 5 |
| 0xFF | (特殊) | 开关调试模式 | 0xFF |

> **已知平台 bug**: `dssad_trigger_debug_callback` 中开启调试时打印的字节序映射表硬编码为 V0 名称（如 `[1]:Debug_ADS_Activate_V0`），**不根据 `product_version_num` 动态切换**。这在 V1 项目上会严重误导——打印说 index=1 是 ADS_Activate，但 V1 代码实际读取的是 index=20。判断版本应看 `dssad_func_register.c` 中的 `product_version_num`。

### 6.7 使用示例

#### 通用: 开启/关闭调试模式

```bash
# === MCU Shell 终端 ===

# 开启调试模式
dssad_set_event_status 0xFF 1
# SOC 日志: "set dssad_event_debug_flag success ret:1"
# SOC 日志: "start dssad debug. Byte order list as follow ..."
# ⚠️ 打印的字节序是硬编码的 V0 映射，V1 项目不要照抄

# 关闭调试模式
dssad_set_event_status 0xFF 0
# SOC 日志: "exit dssad debug"
```

#### V0 项目 (p301) 示例

```bash
# ADS 激活
dssad_set_event_status 1 1
# SOC 日志: "<DSSAD>dssad_time_stamp_event trigger, event_id:0x01
#            (EventId_ADS_Activate_V0) subevent_id 0(Byte_NOD_Activate) ret 0"

# 碰撞 (时间段事件)
dssad_set_event_status 6 1
# SOC 日志: "---- COLLISON status : 1"
# FL 收到: DCMS_TOPIC_DSSAD_EVENT_CHECK 中 collision=true

# 介入请求 (0→1)
dssad_set_event_status 3 1

# 介入请求递增 (1→2)
dssad_set_event_status 3 2
```

#### V1 项目 (p567) 示例

```bash
# ADS 激活 — 先清零再置1产生上升沿
dssad_set_event_status 20 0
# 等待 ≥100ms (让 50ms 状态更新 + 100ms 边沿检测跑完)
dssad_set_event_status 20 1
# SOC 日志: "<DSSAD>dssad_time_stamp_event trigger, event_id:0x14
#            (EventId_ADS_Activate_V1) subevent_id 0(Byte_NOD_Activate) ret 0"

# ADS 退出 — 必须先清零再置1 (EXEC_ONCE 把 last 初始化为了 true)
dssad_set_event_status 21 0
# 等待 ≥100ms
dssad_set_event_status 21 1
# SOC 日志: "<DSSAD>dssad_time_stamp_event trigger, event_id:0x15
#            (EventId_ADS_Exit_V1) ..."

# 碰撞 (时间段+时间戳)
dssad_set_event_status 7 1
# SOC 日志: "---- COLLISON status : 1"

# 碰撞风险
dssad_set_event_status 8 1
# SOC 日志: "---- COLLISION_RISK : 1"

# 锁车
dssad_set_event_status 10 1
# SOC 日志: "---- LOCK status: 1"

# DCA 激活
dssad_set_event_status 27 1
# SOC 日志: "<DSSAD>dssad_time_stamp_event trigger, event_id:0x1b
#            (EventId_DCA_Activate_V1) ret 0"

# HOR 激活 (手离方向盘)
dssad_set_event_status 23 1

# EOR 激活 (眼离路面)
dssad_set_event_status 25 1

# RMF 激活
dssad_set_event_status 28 1

# ADS 驾驶员退出扩展
dssad_set_event_status 22 1

# ADS 严重失效
dssad_set_event_status 29 1
```

> **重要提示**:
> - **时间戳事件需要上升沿** (false→true)。如果当前状态已经是 true 而你设置 status=1，last==true && cur==true → 无边沿 → 不触发。正确做法是先 `status=0` 再 `status=1`。
> - **ADS_Exit 特殊处理**: 代码中有 `EXEC_ONCE(memset(last.ADS_Exit, true, ...))`，首次边沿检测后 last 全为 true。必须先置0再置1产生上升沿。
> - **时间戳 vs 时间段**: 时间戳事件 (ADS_Activate/DCA/...) 发送二进制快照到 `DCMS_TOPIC_FL_OPT_SEND`，FL 立即落盘但**不会打印 "trigger package success"**（那是时间段数据包特有的日志）。时间段事件 (collision/lock/risk) 触发 FL 窗口留存决策，产生大文件包。
> - **版本判断**: 如果日志中调试模式打印的是 V0 字节序但 `dssad_set_event_status 1 1` (V0 index) 不触发，说明固件实际是 V1，应使用 V1 index。

---

## 七、MCU Shell vs SOC Shell 对比总结

| | MCU/FW 侧 ECU Shell | SOC/APP 侧 ECU Shell |
|---|---|---|
| **运行环境** | RTOS (SA8650 安全岛) | QNX/Linux (HPC) |
| **终端程序** | `mc_shell` (固件内置) | `dshell` (独立可执行文件) |
| **命令注册宏** | `DECLEAR_CMD` → `.shellcmd` 段 | `DECLEAR_CMD` → `.shell_soc_cmd` 段 |
| **C 函数签名** | `int32_t(csl, argv[], argc)` | `int32_t(vector<string>&, string&)` |
| **C++ 函数签名** | (不支持 C++) | `int32_t(vector<string>&, string&)` |
| **输出方式** | `shell_print` → `printf` → 串口/UDP | `shell_print` → `output +=` → DCOS RPC 回传 |
| **命令发现** | 链接器自动收集 `.shellcmd` 段 | 启动时解析自身 ELF + `dlopen`/`dlsym` |
| **命令调用** | 进程内直接函数调用 | DCOS RPC (SOMEIP client→server) |
| **发 DCMS Topic** | 直接调 `dcms_mcu_topic_send_msg()` | 同左, 在 handler 中直接调用 |
| **编译开关** | `#ifdef MC_SHELL_SUPPORT` | 无 (命令代码始终编译) |
| **连接 MCU** | — | `dshell -d dsar` → UDP/IPC |
| **连接 SOC APP** | DCMS topic → | `dshell -d dsar_app` → DCOS RPC |

---

## 八、如何新增一个 Shell 命令

### 8.1 在 MCU/FW 侧新增

参考 `do_dssad_set_event_status` 的模式：

```c
// 文件: 放在 dsar_fw 下合适的模块目录

#ifdef MC_SHELL_SUPPORT
#include "mc_shell/shell.h"

static int32_t do_my_new_command(const struct mc_console *csl,
                                  const char *const *const arg,
                                  int32_t arg_size)
{
    if (arg_size < 2)
    {
        shell_print("usage: my_new_command <param>\r\n");
        return -22;
    }

    uint32_t param = simple_strtoul(arg[1], NULL, 0);

    // === 可以做的操作 ===
    // 1. 读写内部全局变量
    // 2. 调用固件内的任何函数
    // 3. 发送 DCMS topic 到 SOC
    // 4. 触发硬件操作

    shell_print("my_new_command executed, param=%d\r\n", param);
    return 0;
}

DECLEAR_CMD(my_new_command, 5, 0, do_my_new_command,
            "my_new_command",
            "my_new_command <param>");
#endif
```

### 8.2 在 SOC/APP 侧新增 (C++版本)

参考 `lidar_proxy` 或 `x_dom_can_rt_dbg` 的模式：

```cpp
// 文件: 放在某个 APP .so 的源文件中 (会被编译进该 .so)

#include "dsar_plat_bf/cdd/shell_soc_cmd.h"
#include "dsar_plat_bf/cdd/dcms_mcu_api.h"

// 命令处理函数 — 必须是 EXPORT_SHELL_SYMBOL
EXPORT_SHELL_SYMBOL int32_t my_soc_command(std::vector<std::string>& argv,
                                            std::string& output)
{
    if (argv.size() < 2)
    {
        shell_print("usage: my_soc_command <param>\r\n");
        return -1;
    }

    std::string param = argv[1];
    // === 可以做的操作 ===
    // 1. 读写模块内部变量
    // 2. 发送 DCMS topic
    // 3. 调用本 .so 内的任何函数

    shell_print("my_soc_command: param=%s\r\n", param.c_str());
    return 0;
}

// 注册命令 — 会被编译进 .shell_soc_cmd 段
DECLEAR_CMD(my_soc_command, 5, 0, my_soc_command,
            "my_soc_command",
            "my_soc_command <param>");
```

无需修改任何注册代码或 CMakeLists.txt。重新编译对应 `.so` 后，`dshell` 会自动发现新命令。

**通过 dshell 调用**：

```bash
dshell -d dsar_app
# 进入交互界面, 格式: <服务名> <命令名> <参数>
dji_ad_app my_soc_command hello_world
```

### 8.3 在 SOC/APP 侧新增 (C 版本)

```c
// 文件: 放在 APP .so 的 C 源文件中

#include "dsar_plat_bf/cdd/shell_soc_cmd.h"

// C 版本函数签名: 与 MCU 侧完全相同
static int32_t do_my_c_command(const struct mc_console *csl,
                                const char *const *const arg,
                                int32_t arg_size)
{
    if (arg_size < 2)
    {
        shell_print("usage: my_c_command <param>\r\n");
        return -22;
    }
    shell_print("my_c_command: arg[1]=%s\r\n", arg[1]);
    return 0;
}

// 注册 — func_type 自动设为 SHELL_FUNC_C
DECLEAR_CMD(my_c_command, 5, 0, do_my_c_command,
            "my_c_command",
            "my_c_command <param>");
```

---

## 九、关键文件索引

```
=== SOC/APP 侧 (dshell 终端 + Shell SOC 服务) ===

dsar-hq-plat/dsar-plat-bf/dsar_app/
├── tool/export/dsar_shell/
│   ├── dsar_shell_simple_history.cpp   dshell 主程序入口 (main, arg_parse, 命令分派)
│   ├── shell_pal.cpp / shell_pal.h     PAL 参数命令实现
│   ├── shell_config_simple.cpp         配置文件解析 (ecu_shell_cfg_ab)
│   ├── cmd_history.c/h                 命令历史 (上下箭头)
│   ├── CMakeLists.txt                  编译配置
│   └── config/sa86xx/
│       ├── ctrl_ecu_shell.json         MCU 设备 UDP 地址配置
│       └── ecu_shell_cfg_ab            设备名→配置文件映射
│
├── cdd/dsar_shell/
│   └── dshell_soc.cpp                  dshell_soc_init / dshell_soc_try_exec_cmd
│
├── cdd/shell_soc/
│   ├── shellsoc_server.cpp             shellsoc_s_init + shell_req_cb (服务端)
│   ├── shellsoc_client.cpp             shellsoc_c_init / shellsoc_c_do_cmd (客户端)
│   ├── shell_soc_register.cpp          RegisterShell → ELF 解析 → dlsym
│   └── shell_soc_internal.h            ShellSocTool 工具函数
│
├── include/dsar_plat_bf/cdd/
│   ├── dshell_soc.h                    dshell_soc_init/cmd_add/try_exec_cmd API
│   ├── shell_soc.h                     RegisterShell 宏 + shellsoc_s/c 接口
│   ├── shell_soc_cmd.h                 DECLEAR_CMD 宏 + t_shell_cmd_info + shell_print
│   └── dcms_mcu_api.h                  DCMS Topic 发送/订阅 API

=== MCU/FW 侧 ===

dsar-hq/src/dsar_fw/
├── product/faw/oem_feature/diagnosis/DcmApp/
│   ├── Diag_CrossEcuShell.c/h         工厂参数 + DSSAD shell 命令 (MCU 侧)
│   ├── DemProcess.c                    DEM 事件 shell 命令
│   └── Diag_DTCEnable.c               DTC 使能 shell 命令
│
├── app_core/fusa/degrade_table/ut/dependance/mc_shell/
│   └── shell.h                         MCU 侧 DECLEAR_CMD 宏定义 + shell_cmd 结构体
│
├── xwire/acu_shell/acu_shell.c        执行器旁路控制 shell 命令
├── app_core/appl_worklist.c           工作列表 shell 命令
├── app_core/version_app/version_app.c 版本信息 shell 命令
└── xwire/vehicle_info/                 车辆信息 shell 命令

=== SOC/APP 侧 (车型适配代码) ===

dsar-hq/src/dsar_app/
├── product/faw/oem_feature/autosar_adapter/
│   └── microsar_config_uds_v4/Appl/Source/
│       └── cross_ecu_shell.c/h         工厂参数 NVM 写入 + DCMS 回调 (SOC 侧)
│
├── app_core/app_ge_core/gb_dssad/
│   ├── dssad_event_trigger.c/h         DSSAD 事件检测 + dssad_trigger_debug_callback
│   └── dssad_data_pack.c/h             DSSAD 数据打包 + DCMS topic 发送
│
└── product/faw/fawhq_p301/
    └── proxy/x_dom_can_rt_gen_*/       x_dom_can_rt_dbg shell 命令

=== 平台仓 SIP 侧 (FW 侧 shell 命令) ===

dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/
├── bsw_manager_v2/                     网络管理 / ESH / 远程代理 shell 命令
├── secoc/secoc_manager/                SecOC / FVM shell 命令
├── can_gateway/                        CAN 网关监控 shell 命令
├── can_com_platform/                   CAN/FL 通信 shell 命令
├── x_dom_can_rt/                       CAN 信号路由 shell 命令
└── diag/SysDiag/                       车辆诊断 shell 命令
```

---

## 十、常用操作速查表

### 10.1 dshell 终端操作

```bash
# 启动 SOC APP shell
dshell -d dsar_app

# 启动 MCU shell
dshell -d dsar

# 单次执行
dshell -d dsar_app -s "list dji_uds_v4"

# dshell 交互界面内:
list dji_uds_v4              # 列出 dji_uds_v4 服务的所有命令
dji_uds_v4 x_dom_can_rt_dbg status  # 执行命令
help                         # Shell SOC 帮助
exit                         # 退出
Tab                          # 命令补全
↑↓                           # 历史命令
```

### 10.2 工厂参数操作

```bash
# MCU Shell:
factory_param_nvm_set 5 FAW_0000000000001    # 写入 VIN
factory_param_nvm_get                         # 查询结果
hexfactory_param_nvm_set 5 0 0x01 0x02 ...   # 按十六进制写
```

### 10.3 DSSAD 调试操作

```bash
# MCU Shell:
dssad_set_event_status 0xFF 1     # 开启调试
dssad_set_event_status 1 0x01     # 触发 ADS 激活
dssad_set_event_status 6 1        # 触发碰撞
dssad_set_event_status 0xFF 0     # 关闭调试
```

### 10.4 SecOC 安全调试

```bash
# MCU Shell:
secoc                             # 查看 SecOC 统计
fvm                               # 查看 FVM 新鲜度值
```

### 10.5 CAN 信号调试

```bash
# MCU Shell:
x_dom_can_rt_dbg status                          # 查看 CAN 路由状态
x_dom_can_rt_dbg read_rx_sig_list               # 读取接收信号列表
x_dom_can_rt_dbg write_to_mcu_sig_set <sig> <val>  # 写 MCU 信号值

# SOC Shell (SOC 侧的信号调试):
dji_ad_app x_dom_can_rt_dbg status
```

### 10.6 网络管理调试

```bash
# MCU Shell:
nm_monitor                        # 网络管理监控
nm_channel                        # NM 通道控制
dataflow                          # 数据流监控
comcontrol                        # 通信控制
```
