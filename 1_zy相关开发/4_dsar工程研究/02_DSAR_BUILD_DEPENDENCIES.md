# DSAR-HQ 构建系统与依赖关系

> 基于 FAW 项目 fawhq_e001_10，SA8650 芯片，双构建系统（CMake + GNU Make），Conan 包管理器驱动。
> 
> 上一轮文档中的错误已在本版纠正：
> - `libdji_uds_v4.so` 实际为 `libuds_v4.so`（无 dji_ 前缀）
> - `vehicle_proxy`/`adapter`/`param` 在 COM.so 和 AD.so 中是**不同 target**（不同源码目录）
> - `gateway_hmicom` 只在 AD.so 中，不在 COM.so 中
> - `plat_common`/`plat_vehicle_proxy` 只在 AD.so 中，不在 COM.so 中
> - dji_bf_app 不加载 uds .so

---

## 一、构建系统总览

```
conan build (conanfile.py: build() 方法)
│
├── Step 1: Conan 依赖解析 → 生成 conanbuildinfo.cmake + conanbuildinfo.mak
│
├── Step 2: dsar_fw 编译 (Makefile, MCU侧)
│     sailsw3 (FAW) + sailsw1 (QCOM)
│
├── Step 3: dsar_app 编译 (CMake, SOC侧)
│     各车型 .so + 可执行程序
│
└── Step 4: 镜像打包 (qnx6fsimg + .bin + tar.xz)
```

MCU 侧和 SOC 侧是**完全独立的编译过程**，互不依赖，可以并行执行。

---

## 二、Conan 依赖解析

### 2.1 解析入口

`conanfile.py: requirements()` 方法根据环境变量按条件组装依赖字典：

```python
# 关键环境变量 (fawhq_e001_10 典型值)
DJI_OS          = "qnx"
DJI_BOARD       = "sa8650"
DJI_SDKVERSION  = "sdk8"
DJI_PRODUCTLIST = "fawhq_e001_10"
DJI_MULTIPKG    = "dsar"          # 非 VIP 单包模式
```

### 2.2 完整依赖列表

| # | Conan 包 | 版本 | Channel | 用途 |
|---|---------|------|---------|------|
| 1 | dji-message | 2.2.0-alpha.4 | TPV1/release | DJI 消息定义 |
| 2 | daf-dins | >=1.18.0 | daf-cicd/stable | DAF 诊断服务框架 |
| 3 | protobuf | >=0.1.1 | infra-cpp-cicd/stable | Protocol Buffers |
| 4 | daf-proto-rhp | >=2.0.0 | daf-cicd/stable | DAF RHP Proto |
| 5 | daf-binxray | >=3.0.0 | daf-cicd/stable | DAF 二进制序列化 |
| 6 | middleware | 8.9.0-alpha.10 | RHP/release | DCOS/MINI_DCOS/SOMEIP |
| 7 | bsp-qcomm | 8.0.0 | RHP/release | Qualcomm BSP (SA8775) |
| 8 | **dsar-plat** | v2.1261.0-beta.87 | RHP/release | AD 平台 (dsar_plat_config等) |
| 9 | **dsar-plat-bf** | v2.31.0-beta.6 | RHP/release | BF 平台 (plat_bf_cdd/ge/udsonip等) |
| 10 | **dsar-plat-ad** | 同 dsar-plat | RHP/release | AD 平台 (plat_ad_app_core等) |
| 11 | ztrace | 1.0.0-alpha.10 | aep/stable | QNX 平台工具 (仅qnx) |

### 2.3 产品覆盖机制

`conan_product_requirements.py` 允许按车型覆盖依赖版本。寻找第一个与 `DJI_PRODUCTLIST` 匹配的 key，将其依赖字典合并覆盖到默认依赖上（空值表示删除该依赖）。FAW 项目默认无覆盖，使用全默认版本。

### 2.4 后缀规则

```
DJI_OS=qnx → suffix = "@RHP/release"
DJI_OS=Linux → suffix = "@TPV1/release"

所有 dsar-plat / dsar-plat-bf / dsar-plat-ad 的版本号如果不含 "/debug" 路径，
则自动追加 suffix。
```

### 2.5 生成的桥接文件

Conan 安装后生成两个关键文件：

```
conanbuildinfo.cmake  → SOC 侧 CMake 使用
  CONAN_INCLUDE_DIRS_DSAR-PLAT-BF   BF平台头文件路径
  CONAN_INCLUDE_DIRS_DSAR-PLAT-AD   AD平台头文件路径
  CONAN_INCLUDE_DIRS_MIDDLEWARE      中间件头文件路径
  CONAN_LIBS_DSAR-PLAT-BF            BF平台预编译库列表
  CONAN_LIB_DIRS_DSAR-PLAT-BF        BF平台库搜索路径
  ... (14个包各自的变量)

conanbuildinfo.mak  → MCU 侧 Makefile 使用
  CONAN_INCLUDE_DIRS      所有头文件路径 (-I)
  CONAN_LIBS              所有库 (-l)
  CONAN_LIB_DIRS          所有库路径 (-L)
  CONAN_DEFINES           宏定义 (-D)
```

---

## 三、APP 侧（SOC）构建链

### 3.1 CMake 顶层入口

`src/dsar_app/CMakeLists.txt`：
```cmake
# 关键变量 (第43-45行)
set(DSAR_APP_LIB_CDD     plat_bf_cdd)         # CDD库 = dsar-plat-bf Conan包
set(DSAR_APP_LIB_UDSONIP plat_bf_udsonip)      # UDS库 = dsar-plat-bf Conan包

# 导入构建宏 (第53-55行)
include(consys/consys_utility.cmake)
include(consys/consys_ad.cmake)
include(consys/consys_bf.cmake)

# 解析产品参数 (第58行)
parse_dpro_param()    # PROJECT_TYPE=FAW → pro_type=faw, 解析 PRODUCT=fawhq_e001_10

# 导入产品配置 (第269-270行)
include(product/faw/faw_config.cmake)       # FAW 全局 CONFIG_*
include(product/faw/fawhq_e001_10/fawhq_e001_10_config.cmake)  # 车型 CONFIG_*

# 构建入口 (file entry/CMakeLists.txt 第316行)
add_subdirectory(entry)
```

### 3.2 可执行文件编译 (entry/CMakeLists.txt)

FAW 项目 `CONFIG_APP_GE_PROCESS=ON`（在 `faw_config.cmake` 第29行设置）：

```cmake
# dji_ad_app — AD主进程
add_executable(dji_ad_app dji_ad_app.cpp)
target_link_libraries(dji_ad_app PRIVATE plat_bf_cdd xml2 mini_dcos dsar_plat_config)

# dji_bf_app — BF主进程
add_executable(dji_bf_app dji_bf_app.cpp)
target_link_libraries(dji_bf_app PRIVATE plat_bf_cdd xml2 mini_dcos dsar_plat_config)

# dji_doip_service_ipv4 — UDS v4 诊断服务
add_executable(dji_doip_service_ipv4 dji_uds_service_v4.cpp)
target_link_libraries(dji_doip_service_ipv4 PRIVATE plat_bf_cdd xml2 mini_dcos dsar_plat_config)

# dji_doip_service_ipv6 — UDS v6 诊断服务
add_executable(dji_doip_service_ipv6 dji_uds_service_v6.cpp)
target_link_libraries(dji_doip_service_ipv6 PRIVATE plat_bf_cdd xml2 mini_dcos dsar_plat_config)
```

**所有可执行文件编译时只链接 4 个基础库**，不链接任何 .so。.so 是运行时通过 `dlopen` 加载的。

| 可执行文件 | 源码 | 运行时 dlopen | 调用的入口函数 | 加载的 .so |
|---|---|---|---|---|
| dji_bf_app | dji_bf_app.cpp | `dll_load_init("com")` | `app_com_core_init`, `app_ge_core_init` | `libfawhq_e001_10_com.so` |
| dji_ad_app | dji_ad_app.cpp | `dll_load_init("app")` | `app_ad_core_init` | `libfawhq_e001_10_app.so` |
| dji_doip_service_ipv4 | dji_uds_service_v4.cpp | `dll_load_init("uds_v4")` | `app_uds_service_init` | `libuds_v4.so` |
| dji_doip_service_ipv6 | dji_uds_service_v6.cpp | `dll_load_init("uds_v6")` | `app_uds_service_init` | `libfawhq_e001_10_uds_v6.so` |

dji_ad_app 在 FAW 分离模式下**只加载 app.so**，不加载 com.so（`#ifndef APP_GE_PROCESS` 块被跳过）。

### 3.3 libfawhq_e001_10_com.so（COM域 + GE域）链接依赖树

由 `consys_bf.cmake: product_build_marcos` 宏构建。

**触发的 CONFIG_* 开关**（来源：fawhq_e001_10_config.cmake）：

| CONFIG 宏 | 值 | 效果 |
|---|---|---|
| CONFIG_APP_COM_CORE | ON | 编译 app_com_core → 加入 APP_CORE_LIB |
| CONFIG_APP_GE_CORE | ON | 编译 app_ge_core → 加入 APP_CORE_LIB |
| CONFIG_X_DOM_CAN_RT_COM | ON | 链接 CAN信号路由(COM侧) |
| CONFIG_X_DOM_SOMEIP_RT_COM | ON | 链接 SOME/IP信号路由(COM侧) |
| CONFIG_SIP_AUTOSAR_COM | ON | 链接 AUTOSAR COM协议栈 |
| CONFIG_BF_OEM_FEATURE | ON | 链接 BF OEM特性 |
| CONFIG_OEM_FEATURE | ON | 链接 OEM特性 |
| CONFIG_OEM_PRODUCT_FEATURE | ON | 链接 OEM产品特性 |
| CONFIG_APP_CALIB | ON | GE子模块: 标定 |
| CONFIG_APP_VPM | ON | GE子模块: VPM |
| CONFIG_APP_DSSAD | ON | GE子模块: DSSAD国标 |
| CONFIG_APP_REMOTE_PARM | ON | GE子模块: 远程参数 |
| CONFIG_SOC_RADAR_PROXY_PLATFORM | ON | GE子模块: 雷达代理 |
| CONFIG_APP_COM_PLATFORM | ON | GE子模块: COM平台 |
| CONFIG_LOG_EXPORT | ON | GE子模块: 日志导出 |
| CONFIG_GE_DIAGPROXY | ON | diagproxy (通过 product_build_marcos) |

```
libfawhq_e001_10_com.so
│
├── [--whole-archive 静态打包]  — 第212行: target_link_libraries
│   │
│   ├── plat_bf_cdd                     ← conan: dsar-plat-bf
│   │   └── 内容: CDD复杂驱动 (DCMS适配/SOC Shell/系统管理/日志/转储/x_dom_can_rt框架)
│   │
│   ├── dcos_dcms                       ← conan: middleware
│   │   └── 内容: DCOS中间件SOC端 (发布/订阅话题总线)
│   │
│   ├── app_com_core_fawhq_e001_10      ← 源码: app_core/app_com_core/app_com_core.cpp
│   │   ├── 链接: dsar_plat_config      ← conan: dsar-plat
│   │   └── 编译宏: APP_COM_CORE, X_DOM_CAN_RT_COM_EN, X_DOM_SOMEIP_RT_COM_EN
│   │
│   ├── app_ge_core_fawhq_e001_10       ← 源码: app_core/app_ge_core/app_ge_core.cpp
│   │   ├── 链接: mini_dcos             ← conan: middleware
│   │   ├── 链接: plat_bf_ge            ← conan: dsar-plat-bf (GE平台层)
│   │   ├── 链接: plat_bf_ge_calib      ← CONFIG_APP_CALIB=ON
│   │   ├── 链接: plat_bf_ge_vpm        ← CONFIG_APP_VPM=ON
│   │   ├── 链接: plat_bf_radar_proxy_platform ← CONFIG_SOC_RADAR_PROXY_PLATFORM=ON
│   │   ├── 链接: plat_bf_ge_com_platform      ← CONFIG_APP_COM_PLATFORM=ON
│   │   ├── 链接: plat_bf_ge_remote_param_manage ← CONFIG_APP_REMOTE_PARM=ON
│   │   ├── 链接: prod_remote_param_cfg_fawhq_e001_10 ← 车型远程参数配置
│   │   └── 链接: app_ge_dssad_fawhq_e001_10     ← CONFIG_APP_DSSAD=ON
│   │
│   ├── bf_oem_feature_fawhq_e001_10    ← CONFIG_BF_OEM_FEATURE=ON
│   ├── oem_feature_fawhq_e001_10       ← CONFIG_OEM_FEATURE=ON (FAW项目第157行)
│   │
│   ├── x_dom_can_rt_gen_lib_com_fawhq_e001_10   ← CONFIG_X_DOM_CAN_RT_COM=ON
│   │   └── 源码: dsar-plat-bf conan包 (sig_route_soc.cpp + x_dom_can_rt_soc.cpp)
│   │       + product/faw/fawhq_e001_10/proxy/x_dom_can_rt_gen_com/
│   │
│   ├── x_dom_someip_rt_gen_lib_com_fawhq_e001_10 ← CONFIG_X_DOM_SOMEIP_RT_COM=ON
│   │   └── 源码: product/faw/fawhq_e001_10/proxy/x_dom_someip_rt_gen_com/
│   │
│   ├── autosar_com_fawhq_e001_10       ← CONFIG_SIP_AUTOSAR_COM=ON
│   │   └── 源码: dsar-sip submodule dsar_app/sip_autosar/sip_2000702/BSW/
│   │       + product/faw/fawhq_e001_10/autosar_adapter/microsar_config_com/
│   │
│   └── oem_proxy_fawhq_e001_10         ← FAW项目使用 (第213行)
│       └── 源码: product/faw/fawhq_e001_10/proxy/
│
└── (无动态链接的外部 .so)
```

### 3.4 libuds_v4.so（UDS诊断域 v4）链接依赖树

由 `consys_bf.cmake: product_build_marcos` 宏构建。注意：**target 名直接写死为 `uds_v4`**（第234行），不加产品前缀。

| CONFIG 宏 | 值 | 效果 |
|---|---|---|
| CONFIG_APP_UDSONIP | ON | 使能 UDS DoIP |
| CONFIG_APP_UDSONIP_IPV4 | ON | 支持 IPv4 |
| CONFIG_APP_UDSONIP_TRANSFER_V4 | ON | 数据传输v4 |
| CONFIG_APP_UDSONIP_INNERUPDATE_V4 | ON | 内部升级v4 |
| CONFIG_APP_UDSONIP_UPDATE | ON | 升级支持 |
| CONFIG_APP_UDSONIP_PROXY | ON | UDS代理 |
| CONFIG_SIP_AUTOSAR_UDS_V4 | ON | 链接 AUTOSAR UDS v4 协议栈 |
| CONFIG_X_DOM_SOMEIP_RT_UDS_V4 | ON | 链接 SOME/IP 路由(UDS v4侧) |
| CONFIG_X_DOM_CAN_RT_UDS_V4 | OFF | 不链接 CAN 路由 |
| CONFIG_OEM_FEATURE_UDS_V4 | OFF | 不链接 OEM UDS v4 特性 |

```
libuds_v4.so
│
├── [--whole-archive 静态打包]  — 第237行
│   │
│   ├── plat_bf_cdd                     ← conan: dsar-plat-bf
│   ├── dcos_dcms                       ← conan: middleware
│   │
│   ├── app_udsonip_fawhq_e001_10_v4    ← 源码: app_core/app_udsonip/
│   │   ├── 源码文件:
│   │   │   ├── appl_ipv4/*             IPv4应用层入口
│   │   │   ├── uds_com_src_list        通用UDS (诊断会话/安全访问/例程控制等)
│   │   │   ├── uds_proxy/              UDS代理
│   │   │   ├── uds_transfer_v4/        数据传输
│   │   │   ├── uds_innerupdate_v4/     内部升级
│   │   │   └── uds_interdiag_v4/       内部诊断
│   │   ├── 链接: plat_bf_udsonip       ← conan: dsar-plat-bf
│   │   └── 编译宏: UDSONIP_USE_IPV4, APP_UDSONIP_IPV4_EN, _UDSONIP_FAW_ENABLE
│   │
│   ├── x_dom_someip_rt_gen_lib_uds_fawhq_e001_10_v4 ← CONFIG_X_DOM_SOMEIP_RT_UDS_V4=ON
│   │   └── 源码: product/faw/fawhq_e001_10/proxy/x_dom_someip_rt_gen_uds/
│   │
│   ├── diagproxy_ge_fawhq_e001_10      ← CONFIG_GE_DIAGPROXY=ON (第138-139行)
│   │
│   └── autosar_uds_fawhq_e001_10_v4    ← CONFIG_SIP_AUTOSAR_UDS_V4=ON
│       └── 源码: dsar-sip submodule dsar_app/sip_autosar/sip_2200_oldsoad/BSW/
│           + product/faw/fawhq_e001_10/autosar_adapter/microsar_config_uds_v4/
│           (fallback: product/faw/oem_feature/autosar_adapter/microsar_config_uds_v4/)
│
├── [动态链接]  — 第238行
│   └── mini_dcos                       ← conan: middleware
```

### 3.5 libfawhq_e001_10_uds_v6.so

结构同 v4 但使用 IPv6 源码路径和 SIP 版本。fawhq_e001_10 中 `CONFIG_APP_UDSONIP_IPV6=OFF` 且 `CONFIG_SIP_AUTOSAR_UDS_V6=OFF`，实际不编译。

### 3.6 libfawhq_e001_10_app.so（AD域）链接依赖树

由 `consys_ad.cmake: app_product_build_marcos` 宏构建。

| CONFIG 宏 | 值 | 效果 |
|---|---|---|
| CONFIG_APP_AD_CORE | ON | 链接 AD 核心框架 |
| CONFIG_X_DOM_CAN_RT_AD | ON | 链接 CAN 信号路由(AD侧) |
| CONFIG_X_DOM_SOMEIP_RT_AD | ON | 链接 SOME/IP 信号路由(AD侧) |
| CONFIG_OEM_FEATURE | ON | 链接 OEM特性(AD侧) |
| CONFIG_OEM_FEATURE_IA | ON | 链接 IA OEM特性 |
| CONFIG_APP_GATEWAY_HMICOM | (默认) | 包含 gateway_hmicom 头文件 |
| CONFIG_APP_GATEWAY_MQTTCOM | (默认OFF) | 需确认 |

```
libfawhq_e001_10_app.so
│
├── [--whole-archive 静态打包]  — 第118行: target_link_libraries
│   │
│   ├── plat_bf_cdd                     ← conan: dsar-plat-bf
│   ├── mini_dcos                       ← conan: middleware
│   ├── dsar_plat_config                ← conan: dsar-plat
│   │
│   ├── vehicle_proxy_fawhq_e001_10     ← 源码: app_core/app_ad_core/product/faw/fawhq_e001_10/proxy/
│   │   └── (注意: 与com.so中的proxy是不同的源码目录！)
│   │
│   ├── adapter_fawhq_e001_10          ← 源码: app_core/app_ad_core/product/faw/fawhq_e001_10/adapter/
│   │   ├── common_adapter             通用适配
│   │   ├── parking_adapter            泊车适配
│   │   ├── driving_adapter            行车适配
│   │   ├── as_adapter                 辅助驾驶适配
│   │   └── hmiapp_adapter             HMI应用适配 (含OEM版本)
│   │
│   ├── product_param_*                ← PAL参数编译产物 (第55行 add_subdirectory)
│   ├── product_fusa_fawhq_e001_10     ← 功能安全 (第58行 add_subdirectory)
│   ├── product_runnable_fawhq_e001_10 ← 可运行实体 (第59行 add_subdirectory)
│   │
│   ├── plat_ad_app_core               ← conan: dsar-plat-ad (AD核心框架)
│   ├── plat_ad_demo                   ← conan: dsar-plat-ad
│   ├── plat_ad_fusa_app               ← conan: dsar-plat-ad
│   ├── plat_vehicle_proxy             ← conan: dsar-plat-ad
│   ├── plat_common                    ← conan: dsar-plat-ad
│   ├── plat_ad_tlv                    ← conan: dsar-plat-ad (TLV编解码)
│   ├── plat_ad_cndtm                  ← conan: dsar-plat-ad (状态机)
│   │
│   ├── app_ia_plat                    ← IA(智能助手)平台层
│   ├── qpc                            ← QPC状态机框架
│   │
│   ├── oem_feature_app_fawhq_e001_10  ← CONFIG_OEM_FEATURE=ON (第94-96行)
│   │   └── 源码: app_core/app_ad_core/product/faw/fawhq_e001_10/oem_feature/
│   │
│   ├── x_dom_can_rt_gen_lib_app_fawhq_e001_10   ← CONFIG_X_DOM_CAN_RT_AD=ON
│   ├── x_dom_someip_rt_gen_lib_app_fawhq_e001_10 ← CONFIG_X_DOM_SOMEIP_RT_AD=ON
│   │
│   ├── gateway_hmicom                 ← 网关HMI通信 (第88行)
│   ├── adc-protobuf                   ← ADC Proto (第91行)
│   ├── dji_gateway-proto-dependency   ← 网关Proto依赖 (第92行)
│   │
│   ├── app_ia_plat                    ← (第76行)
│   │
│   └── [条件链接]:
│       ├── gateway_mqttcom            ← CONFIG_APP_GATEWAY_MQTTCOM (默认OFF, 第88-90行)
│       ├── ia_fawhq_e001_10_app       ← CONFIG_OEM_MODEL_IA (第107-109行)
│       ├── ipsignal_proxy_lib_app_*   ← CONFIG_IPSIGNAL_SAIL2SOC (第65-66行)
│       └── plat_ad_sim                ← CONFIG_DSAR_PLAT_SIM (第79-81行)
│
├── [动态链接]  — 第119行
│   ├── gateway                        ← conan 外部库
│   ├── znet                           ← conan 外部库
│   └── dji_stream_media_protocol      ← conan 外部库
```

---

## 四、--whole-archive 机制解释

每个 .so 都使用此模式链接静态库：

```cmake
target_link_libraries(${PRODUCT_NAME}_com PRIVATE
    -Wl,--whole-archive
        ${product_static_lib_list}
        autosar_com_${PRODUCT_NAME}
        oem_proxy_${PRODUCT_NAME}
    -Wl,--no-whole-archive
)
```

**为什么需要**：DSAR 使用大量**自动注册机制**（链接器段 attribute），模块的初始化函数通过 `__attribute__((section(...)))` 放在特定段中，没有显式的函数调用链。正常的 `--start-group` 只会链接被引用的 .o 文件，会丢弃这些通过段引用的初始化函数。`--whole-archive` 强制将静态库中**所有 .o 文件全部打包进** .so，确保注册机制不丢失。

**代价**：每个 .so 都自带一份 `plat_bf_cdd` 和 `dcos_dcms` 等公共库的完整副本。这些副本在运行时各自独立，不共享内存。

---

## 五、四个 .so 的依赖对比

| 静态库 / 组件 | lib*_com.so | libuds_v4.so | lib*_uds_v6.so | lib*_app.so |
|---|---|---|---|---|
| plat_bf_cdd | ✓ | ✓ | ✓ | ✓ |
| dcos_dcms | ✓ | ✓ | ✓ | — |
| mini_dcos | — (GE通过ge_core间接链接) | ✓ (动态链接) | — | ✓ |
| dsar_plat_config | ✓ (via com_core) | — | — | ✓ |
| app_com_core | ✓ | — | — | — |
| app_ge_core | ✓ | — | — | — |
| app_udsonip | — | ✓ (v4) | ✓ (v6) | — |
| plat_bf_ge + 子模块 | ✓ (via ge_core) | — | — | — |
| plat_bf_udsonip | — | ✓ | — | — |
| plat_ad_* | — | — | — | ✓ |
| autosar_com | ✓ | — | — | — |
| autosar_uds | — | ✓ (v4) | ✓ (v6) | — |
| oem_proxy | ✓ | — | — | — |
| x_dom_can_rt (COM侧) | ✓ | — | — | ✓ (AD侧) |
| x_dom_someip_rt (COM侧) | ✓ | — | — | ✓ (AD侧) |
| x_dom_can_rt (UDS v4侧) | — | OFF | — | — |
| x_dom_someip_rt (UDS v4侧) | — | ✓ | — | — |
| gateway / znet / stream_media | — | — | — | ✓ (动态链接) |
| gateway_hmicom | — | — | — | ✓ |
| adc-protobuf | — | — | — | ✓ |

---

## 六、x_dom_* 代码生成机制

### 6.1 三种代码生成方式对比

| 类型 | 生成方式 | 所在位置 | 目标 | 变量名 |
|------|---------|---------|------|--------|
| **预生成 (平台)** | dsar-plat-bf Conan 包发布时预编译好静态库 | `dsar-plat-bf/dsar_app/cdd/x_dom_can_rt/` | SOC (BF线) | X_DOM_CAN_RT_SRC |
| **PAL 自动生成** | PAL 工具从 JSON 信号描述文件生成 C++ 代码 | `dsar_app/product/<车型>/proxy/x_dom_can_rt_gen_com/` | SOC (BF线+AD线) | x_dom_can_rt_gen_lib |
| **SIP 子模块编译** | dsar-sip Submodule 中的源码直接编译 | `dsar-sip/dsar_fw/sip_extend/x_dom_can_rt/` | MCU | — |

### 6.2 PAL 代码生成流程

PAL (Parameter Abstraction Layer) 工具从 JSON 信号描述文件自动生成 C++ 信号读写代码：

```
JSON 信号描述文件 (平台仓提供)
  │  dsar-hq-plat/dsar-plat-ad/.../pal_param/
  │  dsar-hq-plat/dsar-plat-bf/.../pal_param/
  │
  ├── PAL 工具 (pal_gen / pal_build)
  │     ├── 读取 JSON 信号定义
  │     ├── 解析信号属性 (名称/类型/ID/周期/方向/初始值)
  │     └── 生成 C++ 代码
  │
  └── 生成产物:
        product/<车型>/proxy/x_dom_can_rt_gen_com/
        ├── SigIf_Cfg.h            信号接口配置头文件
        ├── SigIf_Get_*.cpp        CAN 信号读取函数 (如 SigIf_Get_VehicleSpeed)
        ├── SigIf_Set_*.cpp        CAN 信号写入函数
        └── CMakeLists.txt         编译为 x_dom_can_rt_gen_lib_com_* 静态库
```

PAL 工具路径由平台桥接 cmake 文件设置：
- `dsar_plat_ad.cmake` → `PAL_TOOL_PATH = DSAR_PLAT_AD_INSTALL_DIR/tools/pal_gen`
- `dsar_plat_bf.cmake` → PAL 参数表路径配置

### 6.3 三者的交叉引用关系

```
SOC 侧 (dsar_app):
  ┌──────────────────────────────────────────────────────┐
  │ lib${product}_com.so                                  │
  │                                                       │
  │  dsar-plat-bf/cdd/x_dom_can_rt/ (预生成 .a)          │
  │    └→ 提供: 信号路由框架 + DSAR_Plat_CAN_RT 接口      │
  │                                                       │
  │  x_dom_can_rt_gen_lib_com (PAL 自动生成)              │
  │    └→ 提供: 每个信号的 SigIf_Get_*/SigIf_Set_* 实现   │
  │    └→ 调用: ↑ 上面的 DSAR_Plat_CAN_RT 接口            │
  │                                                       │
  │  DSSAD 车型代码 (oem_feature)                         │
  │    └→ 调用: SigIf_Get_VehicleSpeed_*() 等函数         │
  │              │                                        │
  │              └→ 实际实现在 x_dom_can_rt_gen_lib 中     │
  └──────────────────────────────────────────────────────┘

MCU 侧 (dsar_fw):
  ┌──────────────────────────────────────────────────────┐
  │ MCU 固件                                              │
  │                                                       │
  │  dsar-sip/dsar_fw/sip_extend/x_dom_can_rt/ (源码)     │
  │    └→ CAN 信号路由的 MCU 侧实现                       │
  │    └→ 与 SOC 侧通过 DCMS Topic 通信                   │
  └──────────────────────────────────────────────────────┘
```

**关键设计**：SOC 和 MCU 两侧的信号ID、CAN ID、E2E参数定义来自同一套 JSON 描述文件，由 SIP 工具分别生成 SOC 和 MCU 版本的配置代码，保证两侧信号定义一致。

---

## 七、FW 侧（MCU）构建链

### 7.1 构建入口

```
dsar_fw/Makefile
  └→ include build/main.mk
       ├── 解析 oem=faw → PROJECT_TYPE=faw
       ├── 解析 arch=sa8650 → PLATFORM=sa8650
       ├── include conan conanbuildinfo.mak
       └→ include build/main_sa8xx.mk
            ├── include product/faw/sailsw3/sailsw3.config  (Kconfig)
            ├── include build/clang/clang.mk                  (工具链)
            └→ make -C src -f Makefile2 p=sailsw3            (递归编译)
```

### 7.2 头文件包含路径 (common_include_all.mk)

MCU 侧的全局 include 路径在 `common_include_all.mk` 中配置，使所有模块都能引用公共头文件：

```
CINCLUDE_FILE (MCU 侧头文件搜索路径):
  ├── 配置类
  │     ${PRODUCT_DIR}/$(p)/config           车型配置
  │     ${PRODUCT_DIR}/$(p)/vehicle_config    车辆配置
  │     product/${PROJECT_TYPE}/$(p)/config   产品类型配置
  │     include/entry_config/app_config       应用配置入口
  │     include/entry_config/param_config     参数配置入口
  │
  ├── 代理通信
  │     proxy_comm/comm                       代理通信公共
  │     include/proxy                         代理头文件
  │     include/archive                       归档
  │
  ├── BSW 基础软件 (dsar-bsw submodule)
  │     $(DSAR_BSW_LIB_INC_REL_DIR)           BSW 主目录
  │     $(DSAR_BSW_LIB_INC_REL_DIR)/dji_types   DJI 类型定义
  │     $(DSAR_BSW_LIB_INC_REL_DIR)/service     服务层
  │     $(DSAR_BSW_LIB_INC_REL_DIR)/cdd         复杂驱动
  │     $(DSAR_BSW_LIB_INC_REL_DIR)/arch        架构层
  │     $(DSAR_BSW_LIB_INC_REL_DIR)/autosar     AUTOSAR 协议栈
  │     $(DSAR_BSW_LIB_INC_REL_DIR)/bsw         BSW 层
  │     $(DSAR_BSW_LIB_REL_DIR)/bsp_al/bsp_core BSP 核心
  │
  ├── SIP 信号接口协议
  │     $(SIP_EXTEND_LIB_PATH)/x_dom_can_rt    CAN 信号路由 (跨域)
  │
  ├── DJI 消息定义
  │     ../include/local_dji_msgs/local_dji_msg_dsar_fw
  │
  └── 平台特定
        $(DSAR_BSW_LIB_INC_REL_DIR)/platform/sa86xx  SA8650 平台
```

### 7.3 模块组织（Kbuild 风格）

每个目录的 `Makefile` 用 `subdir-y` / `obj-y` 控制条件和必编模块：

```makefile
# app_core/Makefile
subdir-$(CONFIG_APP_FUSA)            += fusa
subdir-y                             += dmm
subdir-$(CONFIG_APP_DIAG)            += diag
subdir-$(CONFIG_APP_VAR_DBG)         += var_dbg
subdir-$(CONFIG_COMPONENT_NM_PLATFORM) += bsw_manager_v2
subdir-$(CONFIG_CAN_COM_PLATFORM)    += can_com_platform
subdir-$(CONFIG_CYBER_SECOC)         += secoc
subdir-$(CONFIG_RTMON)               += rtmon
subdir-y                             += dsar_plat_init
obj-y                                += appl_worklist.o
```

CONFIG_* 宏来自产品的 `.config` 文件（Kconfig 格式），由 `main_sa8xx.mk` include。

### 7.4 FW 二进制链接

链接阶段（main_sa8xx.mk 第76行）：
```makefile
$(LD) $(LIST_MAP_FILE) $(LINK_FLAG) $(LINK_LDSCRIPT) \
    $(shell find $(OUT_PROJECT) -name *.o) \           # 所有自编译的 .o
    $(STATIC_LINK_FILES) \                              # conan 预编译库
    -o $@                                                # → .elf
```

然后：
```makefile
objcopy -O binary → .bin    # ARM 编译器
```

### 7.5 平台特定的构建文件

| 芯片 | 编译入口 | 工具链 |
|---|---|---|
| SA8650/SA8775 | `build/main_sa8xx.mk` | `build/clang/clang.mk` |
| TDA4 | `build/main_tda4.mk` | `build/ti-cgt-arm_15.12.7.LTS/` |
| RH850 | `build/main_rh850.mk` | `build/ccrh850/ccrh850.mk` |
| J6x | `build/main_j6e.mk` | `build/armclang/armclang.mk` |
| x86 | `build/main_x86.mk` | 系统 GCC |

---

## 八、平台包（dsar-hq-plat）提供的静态库到 .so 映射

### 8.1 dsar-plat-bf（BF平台，Conan包）

| Conan 提供的库 | 被链接到 | 功能 |
|---|---|---|
| `plat_bf_cdd` | com.so + uds_v4.so + app.so + 所有ELF | CDD复杂驱动层 |
| `plat_bf_ge` | com.so (via ge_core) | GE通用事件基础 |
| `plat_bf_ge_calib` | com.so (via ge_core) | 标定 |
| `plat_bf_ge_vpm` | com.so (via ge_core) | 车辆参数管理 |
| `plat_bf_ge_remote_param_manage` | com.so (via ge_core) | 远程参数管理 |
| `plat_bf_ge_com_platform` | com.so (via ge_core) | COM平台 |
| `plat_bf_radar_proxy_platform` | com.so (via ge_core) | 雷达代理 |
| `plat_bf_ge_sys_self_diag` | com.so (via ge_core, OFF) | 系统自诊断 |
| `plat_bf_ge_misc` | com.so (via ge_core, OFF) | 杂项 |
| `plat_bf_ge_param_manager` | com.so (via ge_core, OFF) | 参数管理器 |
| `plat_bf_ge_param_sync_server` | com.so (via ge_core, OFF) | 参数同步 |
| `plat_bf_ge_fl_efence_manage` | com.so (via ge_core, OFF) | FL围栏 |
| `plat_bf_udsonip` | uds_v4.so | UDS DoIP 平台层 |
| `plat_bf_log_export` | dji_log_export | 日志导出平台层 |

源码也通过 Conan 包提供：
- `src/cdd/x_dom_can_rt/sig_route_soc.cpp` → 编译进 `x_dom_can_rt_gen_lib_com_*`
- `src/cdd/x_dom_can_rt/x_dom_can_rt_soc.cpp` → 编译进 `x_dom_can_rt_gen_lib_com_*`

### 8.2 dsar-plat-ad（AD平台，Conan包）

| Conan 提供的库 | 被链接到 | 功能 |
|---|---|---|
| `dsar_plat_config` | com.so + app.so + 所有ELF | 平台配置 |
| `plat_ad_app_core` | app.so | AD应用核心框架 |
| `plat_ad_demo` | app.so | AD示例 |
| `plat_ad_fusa_app` | app.so | AD功能安全应用 |
| `plat_vehicle_proxy` | app.so | AD侧车辆信号代理公共代码 |
| `plat_common` | app.so | AD平台公共代码 |
| `plat_ad_tlv` | app.so | AD TLV编解码 |
| `plat_ad_cndtm` | app.so | AD 状态机 |

### 8.3 dsar-sip（SIP协议栈，Git Submodule）

**APP 侧**（编译进 .so）：
- `dsar_app/sip_autosar/sip_2000702/BSW/` → `autosar_com_*` 静态库 → `lib*_com.so`
- `dsar_app/sip_autosar/sip_2200_oldsoad/BSW/` → `autosar_uds_*_v4` 静态库 → `libuds_v4.so`

**FW 侧**（编译进固件）：
- `dsar_fw/sip_extend/x_dom_can_rt/` → MCU 侧 CAN 信号路由 .o
- `dsar_fw/sip_extend/bsw_manager_v2/` → BSW 管理器 .o
- `dsar_fw/sip_extend/can_com_platform/` → CAN 通信平台 .o
- `dsar_fw/sip_extend/can_gateway/` → CAN 网关 .o
- `dsar_fw/sip_extend/secoc/` → SecOC 安全通信 .o
- `dsar_fw/sip_extend/osal/` → OS 抽象层 .o
- `dsar_fw/sip_extend/diag/` → 系统诊断 + UDS 传输层 .o

### 8.4 middleware（中间件，Conan包）

| 提供的库 | 被链接到 | 功能 |
|---|---|---|
| `dcos_dcms` | com.so + uds_v4.so + uds_v6.so | DCOS 中间件 (话题总线) |
| `mini_dcos` | com.so (via ge_core) + uds_v4.so + app.so + 所有ELF | Mini DCOS 轻量版 |

---

## 九、编译产出一览

### 9.1 SOC 侧

| 产物 | 类型 | 所属域 | 典型大小 |
|---|---|---|---|
| `libfawhq_e001_10_com.so` | 动态库 | COM域 + GE域 | ~15-30MB |
| `libuds_v4.so` | 动态库 | UDS域 v4 | ~5-10MB |
| `libfawhq_e001_10_uds_v6.so` | 动态库 | UDS域 v6 (FAW不编译) | — |
| `libfawhq_e001_10_app.so` | 动态库 | AD域 | ~20-40MB |
| `dji_bf_app` | ELF可执行 | BF进程 | ~100KB |
| `dji_ad_app` | ELF可执行 | AD进程 | ~100KB |
| `dji_doip_service_ipv4` | ELF可执行 | UDS v4进程 | ~100KB |
| `dji_doip_service_ipv6` | ELF可执行 | UDS v6进程 (FAW不编译) | — |
| `dji_log_export` | ELF可执行 | 日志导出 | ~500KB |
| `dshell` | ELF工具 | Shell调试 | ~1MB |

### 9.2 MCU 侧

| 产物 | 说明 | 典型大小 |
|---|---|---|
| `sailsw3.bin` | FAW 变体 MCU 固件 | ~6MB |
| `sailsw1.bin` | QCOM 变体 MCU 固件 | ~6MB |

### 9.3 镜像打包 (QNX6 文件系统)

最终发布时，SOC 侧产物和 MCU 固件被打包为 QNX6 文件系统镜像：

```
mkqnx6fsimg
  │
  ├── 输入: SOC 侧所有产物 + 运行时配置
  │     ├── bin/          可执行程序 (dji_bf_app, dji_ad_app, dji_doip_service_ipv4, dji_log_export)
  │     ├── lib/          .so 动态库 (lib*_com.so, libuds_v4.so, lib*_app.so)
  │     ├── etc/          运行时配置 (JSON + 脚本 + PAL参数)
  │     └── scripts/      运行时脚本
  │
  └── 输出:
        ├── qnx6fsimg (SOC 文件系统镜像)
        ├── sailsw3.bin (MCU FAW 固件)
        ├── sailsw1.bin (MCU QCOM 固件)
        └── dsar_xxx.tar.xz (整体发布包)
```

镜像打包配置在 `conan_config.json` 中定义。Conan 的 `package()` 方法负责将编译产物组织到正确的目录结构后调用 `mkqnx6fsimg` 生成镜像。

### 9.4 多产品支持

DSAR-HQ 支持多个车型产品并行编译。产品列表由环境变量 `DJI_PRODUCTLIST` 指定：

```
DJI_PRODUCTLIST = "fawhq_e001_10"   ← FAW e001_10 车型
                = "fawhq_p301"       ← FAW P301 车型
```

每个产品有独立的 CONFIG_* 开关集（在 `<product>_config.cmake` 中）和独立的 `proxy/`、`adapter/`、`param/` 目录。Conan 的 `build()` 方法遍历产品列表，对每个产品执行相同的编译流程。

SOC 侧：每个产品生成独立的 .so 文件（通过 `${PRODUCT_NAME}` 变量区分）。
MCU 侧：每个变体（sailsw3/sailsw1）生成独立的 .bin 固件。

UDS v4 是唯一的例外——`check_autosar_uds_v4_build_first_run` 函数确保多产品编译时 `libuds_v4.so` 只生成一次（所有车型共享同一个 v4 诊断协议栈）。

---

## 十、关键纠正汇总

以下为上一轮文档（03/04/05）中的错误及在本版中的纠正：

| 错误表述 (旧文档) | 正确表述 | 证据 |
|---|---|---|
| `libdji_uds_v4.so` | `libuds_v4.so` (无 dji_ 前缀) | `consys_bf.cmake:234` 直接写死 `add_library(uds_v4 SHARED ...)` |
| dji_bf_app 加载 uds .so | dji_bf_app **只加载 com.so** | `dji_bf_app.cpp:71` 只有 `dll_load_init("com")` |
| `vehicle_proxy`/`adapter`/`param` 在 com.so 和 app.so 中共享 | 它们是**不同的 CMake target**，来自不同源码目录 | com侧: `product/.../proxy/`; ad侧: `app_ad_core/product/.../proxy/` |
| `gateway_hmicom` 在 com.so 和 app.so | **只在 app.so** 中 | `consys_ad.cmake:88` 仅在 `app_product_build_marcos` 中 |
| `plat_common`/`plat_vehicle_proxy` 在 com.so 和 app.so | **只在 app.so** 中 | 来自 dsar-plat-ad，仅在 AD 构建宏中使用 |
| `autosar_com` 和 `autosar_uds` 都在 com.so | `autosar_com` 在 com.so，`autosar_uds` 在 uds_v4/uds_v6.so | `consys_bf.cmake:213,237,260` 分别链接 |
| COM域和GE域是独立进程 | 它们在同一个 .so、同一个进程中 | `dji_bf_app.cpp` 同一个 handle 调用两个入口函数 |
| FW 模块通过 DCMS 作为主要通信方式 | FW 内部主要用直接函数调用，DCMS 用于松耦合和跨芯片 | 全局 include 路径 + `#include` 头文件直接调函数 |

---

## 十一、产品编译配置速查

### FAW fawhq_e001_10 关键 CONFIG_* 开关

| CONFIG 宏 | 值 | 作用域 | 说明 |
|---|---|---|---|
| CONFIG_APP_GE_PROCESS | ON | FAW 全局 | 分离模式 (dji_bf_app + dji_ad_app 独立进程) |
| CONFIG_APP_COM_CORE | ON | COM域 | 编译 COM 通信核心 |
| CONFIG_APP_GE_CORE | ON | GE域 | 编译 GE 通用事件核心 |
| CONFIG_APP_UDSONIP | ON | UDS域 | 使能 UDS DoIP 诊断 |
| CONFIG_APP_UDSONIP_IPV4 | ON | UDS域 | 支持 IPv4 |
| CONFIG_APP_UDSONIP_IPV6 | OFF | UDS域 | 不支持 IPv6 |
| CONFIG_APP_AD_CORE | ON | AD域 | 编译 AD 应用核心 |
| CONFIG_X_DOM_CAN_RT_COM | ON | COM域 | CAN 信号路由 (COM侧) |
| CONFIG_X_DOM_SOMEIP_RT_COM | ON | COM域 | SOME/IP 路由 (COM侧) |
| CONFIG_X_DOM_CAN_RT_UDS_V4 | OFF | UDS域 | CAN 路由 (UDS侧, 不使用) |
| CONFIG_X_DOM_SOMEIP_RT_UDS_V4 | ON | UDS域 | SOME/IP 路由 (UDS侧) |
| CONFIG_SIP_AUTOSAR_COM | ON | COM域 | AUTOSAR COM 协议栈 |
| CONFIG_SIP_AUTOSAR_UDS_V4 | ON | UDS域 | AUTOSAR UDS v4 协议栈 |
| CONFIG_APP_CALIB | ON | GE域 | 标定功能 |
| CONFIG_APP_VPM | ON | GE域 | 车辆参数管理 |
| CONFIG_APP_DSSAD | ON | GE域 | 国标 DSSAD 数据记录 |
| CONFIG_APP_REMOTE_PARM | ON | GE域 | 远程参数管理 |
| CONFIG_GE_DIAGPROXY | ON | GE域 | 诊断代理 |

---

## 十二、关键文件索引

```
=== Conan 依赖管理 ===
dsar-hq/conanfile.py                             构建总入口 (requirements + build + package)
dsar-hq/conan_version_dsar_plat.py               dsar-plat / dsar-vip-plat 版本声明
dsar-hq/conan_version_dsar_plat_bf.py            dsar-plat-bf 版本映射表
dsar-hq/conan_version_dsar_plat_ad.py            dsar-plat-ad 版本映射表
dsar-hq/conan_version_dsar_bsw.py                dsar-bsw 版本声明
dsar-hq/conan_product_requirements.py            各车型依赖版本覆盖表
dsar-hq/conan_config.json                        产品列表 + 镜像打包配置

=== MCU 侧编译 (Makefile) ===
dsar-hq/src/dsar_fw/build/main.mk                主 Makefile 入口 (解析 oem/arch)
dsar-hq/src/dsar_fw/build/main_sa8xx.mk          SA8650 MCU 编译 Makefile
dsar-hq/src/dsar_fw/build/main_tda4.mk           TDA4 MCU 编译
dsar-hq/src/dsar_fw/build/main_rh850.mk          RH850 MCU 编译
dsar-hq/src/dsar_fw/build/main_x86.mk            x86 模拟编译
dsar-hq/src/dsar_fw/build/clang/clang.mk         工具链定义
dsar-hq/src/dsar_fw/include/common_include_all.mk  全局 include 路径配置
dsar-hq/src/dsar_fw/include/common_include_autosar_bsw.mk  AUTOSAR BSW 头文件
dsar-hq/src/dsar_fw/app_core/Makefile            条件编译子模块 (subdir-y)
dsar-hq/src/dsar_fw/app_core/appl_main.c          任务创建入口 (4个RTOS任务)
dsar-hq/src/dsar_fw/app_core/appl_worklist.c      工作链表调度

=== SOC 侧编译 (CMake) ===
dsar-hq/src/dsar_app/CMakeLists.txt              SOC 编译顶层 CMake
dsar-hq/src/dsar_app/consys/consys_utility.cmake  工具宏
dsar-hq/src/dsar_app/consys/consys_bf.cmake        BF 产品编译宏 (product_build_marcos)
dsar-hq/src/dsar_app/consys/consys_ad.cmake        AD 产品编译宏 (app_product_build_marcos)
dsar-hq/src/dsar_app/entry/CMakeLists.txt          可执行程序 CMake
dsar-hq/src/dsar_app/app_core/CMakeLists.txt       应用核心 CMake (条件编译)

=== 可执行程序入口 ===
dsar-hq/src/dsar_app/entry/dji_ad_app.cpp         AD 主进程 (dlopen app.so)
dsar-hq/src/dsar_app/entry/dji_bf_app.cpp         BF 主进程 (dlopen com.so, COM+GE)
dsar-hq/src/dsar_app/entry/dji_uds_service_v4.cpp UDS v4 诊断服务 (dlopen uds_v4.so)
dsar-hq/src/dsar_app/entry/dji_uds_service_v6.cpp UDS v6 诊断服务 (dlopen uds_v6.so)

=== 车型配置 (FAW fawhq_e001_10) ===
dsar-hq/src/dsar_app/product/faw/faw_config.cmake           FAW 全局 CONFIG_*
dsar-hq/src/dsar_app/product/faw/fawhq_e001_10/fawhq_e001_10_config.cmake  车型级 CONFIG_*
dsar-hq/src/dsar_app/product/faw/fawhq_e001_10/proxy/       信号代理 (BF侧, PAL生成)
dsar-hq/src/dsar_app/product/faw/fawhq_e001_10/adapter/     信号适配器 (BF侧)
dsar-hq/src/dsar_app/product/faw/fawhq_e001_10/autosar_adapter/  AUTOSAR 配置
dsar-hq/src/dsar_app/app_core/app_ad_core/product/faw/fawhq_e001_10/  AD侧产品代码

=== 平台仓桥接 ===
dsar-hq-plat/dsar-plat-ad/src/dsar_app/dsar_plat_ad.cmake   AD 平台桥接 cmake
dsar-hq-plat/dsar-plat-bf/dsar_app/dsar_plat_bf.cmake       BF 平台桥接 cmake
dsar-hq-plat/dsar-plat-bf/dsar_app/tool/export/dsar_shell/  dshell 源代码
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/                   MCU 侧 SIP 协议栈源码
dsar-hq-plat/dsar-sip/dsar_app/sip_autosar/                 SOC 侧 SIP AUTOSAR 源码

=== 构建过程分析 ===
dsar-hq-buildlog/buildlog.txt                     完整编译日志 (Conan + 编译 + 打包)
```

以下为上一轮文档（03/04/05）中的错误及在本版中的纠正：

| 错误表述 (旧文档) | 正确表述 | 证据 |
|---|---|---|
| `libdji_uds_v4.so` | `libuds_v4.so` (无 dji_ 前缀) | `consys_bf.cmake:234` 直接写死 `add_library(uds_v4 SHARED ...)` |
| dji_bf_app 加载 uds .so | dji_bf_app **只加载 com.so** | `dji_bf_app.cpp:71` 只有 `dll_load_init("com")` |
| `vehicle_proxy`/`adapter`/`param` 在 com.so 和 app.so 中共享 | 它们是**不同的 CMake target**，来自不同源码目录 | com侧: `product/.../proxy/`; ad侧: `app_ad_core/product/.../proxy/` |
| `gateway_hmicom` 在 com.so 和 app.so | **只在 app.so** 中 | `consys_ad.cmake:88` 仅在 `app_product_build_marcos` 中 |
| `plat_common`/`plat_vehicle_proxy` 在 com.so 和 app.so | **只在 app.so** 中 | 来自 dsar-plat-ad，仅在 AD 构建宏中使用 |
| `autosar_com` 和 `autosar_uds` 都在 com.so | `autosar_com` 在 com.so，`autosar_uds` 在 uds_v4/uds_v6.so | `consys_bf.cmake:213,237,260` 分别链接 |

---
