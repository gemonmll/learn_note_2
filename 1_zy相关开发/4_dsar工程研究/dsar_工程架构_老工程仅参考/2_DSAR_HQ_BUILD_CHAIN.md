# DSAR-HQ 完整编译链路与依赖关系

---

## 一、编译系统概览

DSAR-HQ 使用**双构建系统**：MCU 侧用 Makefile，SOC 侧用 CMake。整个编译由 **Conan 包管理器**驱动，conanfile.py 中的 `build()` 方法是总入口。

```
conan build
  │
  ├── Step 1: Conan 依赖解析 (requirements)
  │     ├── 读取 conan_version_*.py 获取版本
  │     ├── 根据 DJI_BOARD=sa8650, DJI_OS=qnx 选择 suffix
  │     ├── 解析 14 个 Conan 包 → 生成 conanbuildinfo.cmake / conanbuildinfo.mak
  │     └── clone dji_build_script 构建脚本仓库
  │
  ├── Step 2: dsar_fw 编译 (Makefile, MCU 侧)
  │     ├── sailsw3 (FAW, soc1_sail_app2) — MCU固件变体1
  │     └── sailsw1 (QCOM, soc1_sail_app1) — MCU固件变体2
  │
  ├── Step 3: dsar_app 编译 (CMake, SOC 侧)
  │     ├── 编译各车型的 .so 动态库
  │     ├── 编译可执行程序
  │     └── 编译 dshell 调试工具
  │
  └── Step 4: 镜像打包
        ├── SOC: mkqnx6fsimg → qnx6fsimg
        ├── MCU: 固件 objcopy → .bin
        └── 整体: tar.xz 发布包
```

---

## 二、Conan 依赖解析（14 个包）

### 2.1 依赖解析流程

```
conanfile.py: requirements()
  │
  ├── 环境变量读取:
  │     DJI_BOARD     = "sa8650"     ← 硬件平台
  │     DJI_OS        = "qnx"        ← 操作系统 (决定 suffix = @RHP/release)
  │     DJI_SDKVERSION = "sdk2.0.0"  ← SDK 版本 (影响 J6 平台的 bsw 版本)
  │     DJI_PRODUCTLIST = "fawhq_e009" ← 产品车型
  │     DJI_MULTIPKG  = "dsar"       ← 单包/多包模式 (dsar / dsar-app / dsar-vip)
  │
  ├── 基础依赖 (所有项目通用):
  │     dji-message       = "dji-message/2.2.0-alpha.4@TPV1/release"
  │     daf-dins          = "daf-dins/[>=1.18.0 <2.0.0]@daf-cicd/stable"
  │     protobuf          = "protobuf/[>=0.1.1 <1.0.0]@infra-cpp-cicd/stable"
  │     daf-proto-rhp     = "daf-proto-rhp/[>=2.0.0 <3.0.0]@daf-cicd/stable"
  │     daf-binxray       = "daf-binxray/[>=3.0.0 <4.0.0]@daf-cicd/stable"
  │
  ├── QNX 特有依赖:
  │     ztrace            = "bsp_platform_utils/1.0.0-alpha.10@aep/stable"
  │
  ├── 平台依赖 (按版本文件):
  │     dsar-plat         = dsar_plat.version          (v2.1261.0-beta.87)
  │     dsar-plat-bf      = dsar_plat_bf.conan_versions["default"]  (v2.31.0-beta.6)
  │     dsar-plat-ad      = dsar_plat_ad.conan_versions["default"]  (同上)
  │     middleware         = "middleware/8.9.0-alpha.10@RHP/release"
  │
  ├── SA8650 特有依赖:
  │     bsp-qcomm         = "bsp-qcomm/8.0.0@RHP/release"
  │
  ├── 车型产品特殊依赖 (从 conan_product_requirements.py):
  │     product_req_map["fawhq_e009"] → 可能覆盖上述版本
  │
  └── 后缀处理:
        dsar-plat  += suffix (@RHP/release)  (如果版本字符串不包含 /debug)
        dsar-plat-bf += suffix
        dsar-plat-ad += suffix
```

### 2.2 完整 Conan 依赖列表

| # | Conan 包名 | 版本 | Channel | 用途 |
|---|-----------|------|---------|------|
| 1 | dji-message | 2.2.0-alpha.4 | TPV1/release | DJI 消息定义（proto/serialization） |
| 2 | daf-dins | >=1.18.0 <2.0.0 | daf-cicd/stable | DAF 诊断服务框架 |
| 3 | protobuf | >=0.1.1 <1.0.0 | infra-cpp-cicd/stable | Protocol Buffers 序列化 |
| 4 | daf-proto-rhp | >=2.0.0 <3.0.0 | daf-cicd/stable | DAF RHP Proto 定义 |
| 5 | daf-binxray | >=3.0.0 <4.0.0 | daf-cicd/stable | DAF 二进制序列化/反序列化 |
| 6 | middleware | 8.9.0-alpha.10 | RHP/release | 中间件 (DCOS/SOMEIP/MINIDCOS) |
| 7 | bsp-qcomm | 8.0.0 | RHP/release | Qualcomm BSP 板级支持包 |
| 8 | dsar-plat | v2.1261.0-beta.87 | RHP/release | ★ AD 平台公共代码 |
| 9 | dsar-plat-bf | v2.31.0-beta.6 | RHP/release | ★ BF 基础功能平台代码 |
| 10 | dsar-plat-ad | 同上 | RHP/release | ★ AD 平台代码 (与 plat 同步) |
| 11 | adc-proto | (继承) | RHP/release | ADC Proto 定义 (tda4-vh 平台独立声明) |
| 12 | bsp_platform_utils | 1.0.0-alpha.10 | aep/stable | QNX 平台工具 (ztrace) |
| 13 | visualization | (项目特定) | — | 可视化工具 |
| 14 | chais | (项目特定) | — | CHAIS 框架 |

### 2.3 Conan 生成的构建信息文件

Conan 依赖解析完成后，自动生成两个关键文件：

```
conanbuildinfo.cmake     ← SOC 侧 CMake 使用
  ├── CONAN_INCLUDE_DIRS_*   头文件搜索路径
  ├── CONAN_LIBS_*           库文件列表
  ├── CONAN_LIB_DIRS_*       库搜索路径
  └── CONAN_DEFINES_*        编译宏定义

conanbuildinfo.mak        ← MCU 侧 Makefile 使用
  ├── CONAN_INCLUDE_DIRS_*   -I 头文件路径
  ├── CONAN_LIBS_*           -l 库名
  └── CONAN_LIB_DIRS_*       -L 库路径
```

---

## 三、dsar_fw (MCU 侧) 编译链路

### 3.1 编译入口

```
conanfile.py: build()
  └─ cb.execute_build_process()
       └─ dsar_fw/build/main_sa8xx.mk    ← SA8650 MCU 编译入口
            ├── include conanbuildinfo.mak   (Conan 依赖的头文件/库路径)
            ├── -DPLATFORM_SA86XX            (芯片平台宏)
            ├── -D__SA8650__                 (芯片型号宏)
            └── 两个变体:
                  sailsw3 → FAW  (soc1_sail_app2)
                  sailsw1 → QCOM (soc1_sail_app1)
```

### 3.2 头文件包含路径 (common_include_all.mk)

MCU 侧的 include 路径配置在 [common_include_all.mk](dsar-hq/src/dsar_fw/include/common_include_all.mk) 中：

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
  │     $(DSAR_BSW_LIB_INC_REL_DIR)/archive     归档
  │     $(DSAR_BSW_LIB_REL_DIR)/bsp_al/bsp_core BSP 核心
  │
  ├── SIP 信号接口协议
  │     $(SIP_EXTEND_LIB_PATH)/x_dom_can_rt    CAN 信号路由 (跨域)
  │
  ├── DJI 消息定义
  │     ../include/local_dji_msgs/local_dji_msg_dsar_fw
  │
  ├── 控制模块
  │     ../appl                                应用层
  │
  └── 平台特定
        $(DSAR_BSW_LIB_INC_REL_DIR)/platform/sa86xx  SA8650 平台
```

### 3.3 MCU 固件编译流程

```
main_sa8xx.mk
  │
  ├── 编译所有 C/ASM 源文件 → .o 目标文件
  │     源文件来源:
  │     ├── dsar_fw/ 自身代码 (proxy_comm, xwire, app_core, product/)
  │     ├── dsar-sip submodule (dsar_fw/sip_extend/*)
  │     ├── dsar-bsw submodule (AUTOSAR BSW)
  │     └── control submodule (控制算法)
  │
  ├── 链接 → .elf 固件
  │     链接:
  │     ├── Conan 依赖的预编译库 (通过 conanbuildinfo.mak)
  │     ├── dsar-plat-bf 中的 MCU 侧预编译代码
  │     └── 所有编译好的 .o 文件
  │
  ├── objcopy → .bin 固件镜像
  │     arm-none-eabi-objcopy -O binary
  │
  └── 两个变体 → 三个固件镜像:
        ┌─────────────────────────────────────────────┐
        │ sailsw3:   FAW 变体  (soc1_sail_app2)       │
        │ sailsw1:   QCOM 变体 (soc1_sail_app1)       │
        │ sailhyp:   预编译 Hypervisor 镜像 (不编译)     │
        └─────────────────────────────────────────────┘
```

### 3.4 MCU 固件代码组织

```
dsar_fw/
├── proxy_comm/              → 代理通信层 (SOC-MCU 信号转发)
│   └── comm/                公共通信模块
│
├── xwire/                   → 跨总线通信
│   ├── acu_shell/           执行器旁路控制
│   ├── vehicle_info/        车辆信息
│   └── ...
│
├── app_core/                → 应用核心模块
│   ├── appl_worklist/       工作列表
│   ├── version_app/         版本信息
│   ├── fusa/                功能安全 (降级表等)
│   └── ...
│
├── product/faw/             → 车型适配 (FAW)
│   ├── oem_feature/
│   │   └── diagnosis/
│   │       ├── DcmApp/      DCM诊断应用 (工厂参数/DSSAD shell)
│   │       ├── DemProcess.c  DEM事件管理 shell
│   │       ├── Diag_DTCEnable.c  DTC 使能 shell
│   │       └── DiagCalib_Main.c  标定 shell
│   └── ...
│
├── include/                 → 头文件
│   ├── entry_config/        入口配置
│   ├── proxy/               代理接口
│   ├── common_include_all.mk  全局 include 路径
│   └── ...
│
└── build/                   → Makefile 编译入口
    └── main_sa8xx.mk        SA8650 MCU 编译
```

---

## 四、dsar_app (SOC 侧) 编译链路

### 4.1 编译入口

```
conanfile.py: build()
  └─ cb.execute_build_process()
       └─ dsar_app/CMakeLists.txt              ← SOC 编译顶层 CMake
            ├── include(conanbuildinfo.cmake)    (Conan 依赖信息)
            ├── include(consys/consys_ad.cmake)  (AD 产品编译宏)
            ├── include(consys/consys_bf.cmake)  (BF 产品编译宏)
            ├── include(dsar_plat_ad.cmake)      (AD 平台桥接)
            ├── include(dsar_plat_bf.cmake)      (BF 平台桥接)
            │
            ├── add_subdirectory(product/faw/fawhq_e009/)
            │     ├── fawhq_e009_config.cmake     (产品级编译选项: CONFIG_APP_xxx)
            │     ├── param/                      (PAL 自动生成代码)
            │     ├── proxy/                      (信号代理)
            │     ├── adapter/                    (信号适配器)
            │     ├── fusa/                       (功能安全)
            │     ├── runnable/                   (可运行实体)
            │     ├── domain_function/            (领域功能)
            │     └── oem_feature/                (OEM 特性)
            │
            └── add_subdirectory(entry/)          (可执行程序)
```

### 4.2 SOC 编译的两条产品线

```
dsar_app/CMakeLists.txt
  │
  ├── BF 产品线 (consys_bf.cmake → product_build_marcos)
  │     对应: 基础功能 + 诊断通信
  │     产出: libfawhq_e009_com.so           ← BF 通信 SO
  │           libdji_uds_v4.so               ← UDS V4 SO
  │           libfawhq_e009_uds_v6.so        ← UDS V6 SO
  │
  └── AD 产品线 (consys_ad.cmake → app_product_build_marcos)
        对应: 自动驾驶应用
        产出: libfawhq_e009_app.so           ← AD 应用 SO
```

### 4.3 BF 产品 SO 编译链路 (`lib${product}_com.so`)

```
product_build_marcos() 宏 [consys_bf.cmake]
  │
  ├── 读取 ${product}_config.cmake → CONFIG_APP_* 开关
  │
  ├── add_subdirectory(param/)       → PAL 自动生成代码 (JSON→C++)
  ├── add_subdirectory(proxy/)       → 信号代理
  │     ├── x_dom_can_rt_gen_com/    → CAN 信号路由生成代码
  │     └── x_dom_someip_rt_gen_com/ → SOMEIP 信号路由生成代码
  ├── add_subdirectory(adapter/)     → 信号适配器
  ├── add_subdirectory(oem_feature/) → OEM 特性
  │
  └── add_library(${PRODUCT_NAME}_com SHARED ...)
       └── target_link_libraries(... --whole-archive
             ├── 产品自有静态库:
             │     vehicle_proxy_${product}         ← 车型信号代理
             │     adapter_${product}               ← 车型信号适配
             │     param_${product}                 ← PAL 参数
             │     oem_feature_com_${product}       ← OEM 特性
             │     ${APP_CORE_LIB}                  ← 应用核心 (含 DSSAD 等)
             │
             ├── 平台公共静态库:
             │     autosar_com                      ← AUTOSAR 通信栈
             │     autosar_uds                      ← UDS 诊断栈
             │     dcos_dcms                        ← DCMS 中间件
             │     plat_bf_cdd                      ← BF CDD 复杂驱动
             │     plat_bf_ge                       ← BF GE 通用事件
             │     plat_common                      ← 平台公共
             │     plat_vehicle_proxy               ← 车辆代理公共
             │
             ├── 预生成的信号路由代码:
             │     x_dom_can_rt_gen_lib_com_${product}   ← CAN 路由
             │     x_dom_someip_rt_gen_lib_com_${product} ← SOMEIP 路由
             │
             └── 第三方库:
                   mini_dcos                        ← Mini DCOS
                   gateway                          ← 网关
                   znet                             ← ZNet 网络
             --no-whole-archive)
```

**关键设计**: `--whole-archive` 确保所有静态库的符号都被链接进 SO，因为许多符号通过自动注册机制（链接器段）被引用，没有显式函数调用。

### 4.4 AD 产品 SO 编译链路 (`lib${product}_app.so`)

```
app_product_build_marcos() 宏 [consys_ad.cmake]
  │
  ├── 读取 ${product}_config.cmake → CONFIG_APP_AD_CORE 开关
  │
  ├── add_subdirectory(param/)       → PAL 参数
  ├── add_subdirectory(proxy/)       → 信号代理
  ├── add_subdirectory(adapter/)     → 信号适配器
  ├── add_subdirectory(fusa/)        → 功能安全
  ├── add_subdirectory(runnable/)    → 可运行实体
  ├── add_subdirectory(domain_function/) → 领域功能 (如果存在)
  ├── add_subdirectory(oem_feature/) → OEM 特性
  │
  └── add_library(${PRODUCT_NAME}_app SHARED ...)
       └── target_link_libraries(... --whole-archive
             ├── 车型特有静态库:
             │     vehicle_proxy_${product}           ← 信号代理
             │     adapter_${product}                 ← 信号适配
             │     param_${product}                   ← PAL 参数
             │     fusa_${product}                    ← 功能安全
             │     product_runnable_${product}        ← 可运行实体
             │     domain_function_${product}         ← 领域功能
             │     oem_feature_app_${product}         ← OEM 特性
             │     gateway_hmicom                     ← 网关人机交互
             │
             ├── 平台公共静态库:
             │     dsar_plat_config                   ← AD 平台配置
             │     plat_ad_demo                       ← AD 平台示例
             │     plat_ad_fusa_app                   ← AD 功能安全
             │     plat_ad_app_core                   ← AD 应用核心
             │     plat_ad_cndtm                      ← AD 状态机
             │     plat_ad_tlv                        ← AD TLV 编解码
             │     plat_vehicle_proxy                 ← 车辆代理公共
             │     plat_common                        ← 平台公共
             │     app_ia_plat                       ← IA 平台
             │     qpc                                ← QPC 框架
             │     mini_dcos                          ← Mini DCOS
             │
             ├── 通信相关:
             │     gateway_hmicom                     ← 网关人机交互
             │     gateway_mqttcom                    ← MQTT 网关 (可选)
             │     adc-protobuf                       ← ADC Proto
             │     dji_gateway-proto-dependency       ← 网关 Proto
             │
             ├── 预生成的信号路由:
             │     x_dom_can_rt_gen_lib_app_${product}
             │     x_dom_someip_rt_gen_lib_app_${product}
             │
             ├── 可选 OA 模块:
             │     ipsignal_proxy_lib_app_${product}  ← IP 信号代理 (CONFIG_IPSIGNAL_SAIL2SOC)
             │     oem_feature_app_${product}         ← OEM 特性 (CONFIG_OEM_FEATURE)
             │     ia_${product}_app                  ← IA 特性 (CONFIG_OEM_MODEL_IA)
             │
             └── 外部链接:
                   gateway
                   znet
                   dji_stream_media_protocol
             --no-whole-archive)
```

### 4.5 可执行程序

SOC 侧编译以下可执行程序（定义在 [entry/CMakeLists.txt](dsar-hq/src/dsar_app/entry/CMakeLists.txt)）：

```
可执行程序:
  ├── dji_ad_app               ★ AD 主进程 (自动驾驶应用)
  │     链接: ${DSAR_APP_LIB_CDD}, xml2, mini_dcos, dsar_plat_config
  │            + fdbus, zytfdbus, protobuf (CONFIG_FDBUS_ENABLE)
  │     运行时加载: lib${product}_app.so
  │
  ├── dji_bf_app               ★ BF 主进程 (基础功能 + 诊断)
  │     链接: 同 dji_ad_app
  │     运行时加载: lib${product}_com.so, libdji_uds_v4.so, lib${product}_uds_v6.so
  │     (仅在 CONFIG_APP_GE_PROCESS 开启时独立编译)
  │
  ├── dji_doip_service_ipv4    ★ DoIP IPv4 服务 (诊断)
  │     链接: ${DSAR_APP_LIB_CDD}, xml2, mini_dcos, dsar_plat_config
  │
  ├── dji_doip_service_ipv6    ★ DoIP IPv6 服务 (诊断)
  │     链接: ${DSAR_APP_LIB_CDD}, xml2, mini_dcos, dsar_plat_config
  │
  └── doip_replay              DoIP 回放工具 (可选)
```

---

## 五、SOC SO 文件的符号解析路径

### 5.1 BF 线 — `lib${product}_com.so` 的符号解析

```
libfawhq_e009_com.so
  │
  ├─ --whole-archive
  │   ├── vehicle_proxy_fawhq_e009        ← product/faw/fawhq_e009/proxy/
  │   │     └── 内部引用: PAL 自动生成的 SigIf_Get_* 函数
  │   │                   (由 x_dom_can_rt_gen_com 提供实现)
  │   │
  │   ├── adapter_fawhq_e009             ← product/faw/fawhq_e009/adapter/
  │   │     └── 内部引用: DCMS topic 回调, UDS 服务实现
  │   │
  │   ├── param_fawhq_e009               ← PAL 自动生成代码
  │   │     └── 提供: 参数表定义 + 读写接口
  │   │
  │   ├── ${APP_CORE_LIB}                ← app_core/app_ge_core + app_bf_core
  │   │     ├── gb_dssad/                 ★ DSSAD 数据记录
  │   │     ├── app_calib/                应用标定
  │   │     ├── fl_eFenceDownload/       FL 围栏下载
  │   │     ├── sys_self_diag/           系统自诊断
  │   │     └── ...                      其他 GE/BF 模块
  │   │
  │   ├── oem_feature_com_fawhq_e009     ← OEM 特性
  │   │     └── dssad_cfg/               ★ DSSAD 车型适配
  │   │           ├── dssad_data_get.c     信号获取实现
  │   │           ├── dssad_time_segment_cfg.c  5张JSON表定义
  │   │           ├── dssad_time_stamp_cfg.c    时间戳+事件发送
  │   │           ├── dssad_func_register.c     函数指针绑定
  │   │           └── DssadBaseData_20.h        协议头
  │   │
  │   ├── autosar_com                    ← SIP/AUTOSAR 通信栈
  │   ├── autosar_uds                    ← UDS 诊断栈
  │   ├── dcos_dcms                      ← DCMS 中间件 (跨核通信 SOC端)
  │   ├── plat_bf_cdd                    ← BF CDD (系统管理/日志/转储/Shell)
  │   ├── plat_bf_ge                     ← BF GE (通用事件)
  │   │
  │   ├── x_dom_can_rt_gen_lib_com_fawhq_e009    ← CAN 信号路由实现
  │   │     └── 提供: SigIf_Get_*() 函数 (20+个 CAN 信号读取)
  │   │
  │   └── x_dom_someip_rt_gen_lib_com_fawhq_e009 ← SOMEIP 信号路由实现
  │         └── 提供: SOMEIP 信号收发
  │
  ├── gateway                           ← 网关库
  ├── znet                              ← ZNet 网络库
  └── mini_dcos                         ← Mini DCOS 库
```

### 5.2 AD 线 — `lib${product}_app.so` 的符号解析

```
libfawhq_e009_app.so
  │
  ├─ --whole-archive
  │   ├── vehicle_proxy_fawhq_e009        ← 信号代理 (AD 侧重用了 BF 侧的 proxy)
  │   ├── adapter_fawhq_e009             ← 信号适配
  │   ├── param_fawhq_e009               ← PAL 参数
  │   ├── fusa_fawhq_e009                ← 功能安全
  │   ├── product_runnable_fawhq_e009    ← 可运行实体 (周期任务调度)
  │   ├── domain_function_fawhq_e009     ← 领域功能
  │   │     └── gateway_hmicom/          网关人机交互
  │   │
  │   ├── dsar_plat_config               ← AD 平台配置
  │   ├── plat_ad_demo                   ← AD 示例
  │   ├── plat_ad_fusa_app               ← AD 功能安全应用
  │   ├── plat_ad_app_core               ← ★ AD 应用核心
  │   │     └── 来源: dsar-hq-plat/dsar-plat-ad/src/dsar_app/app_core/app_ad_core/
  │   │           ├── dsar_plat/         平台层 (proxy/business/sim/extension)
  │   │           ├── asw_app_core/      应用软件核心
  │   │           ├── mode_manager/      模式管理器
  │   │           ├── dmm_common/        DMM 公共
  │   │           └── plat_app_interface/ 平台应用接口
  │   │
  │   ├── plat_ad_cndtm                  ← AD 状态机
  │   ├── plat_ad_tlv                    ← AD TLV 编解码
  │   ├── plat_vehicle_proxy             ← 车辆代理公共
  │   ├── plat_common                    ← 平台公共
  │   ├── app_ia_plat                   ← IA 平台
  │   ├── qpc                            ← QPC 框架 (状态机)
  │
  │   ├── x_dom_can_rt_gen_lib_app_fawhq_e009    ← CAN 信号路由
  │   └── x_dom_someip_rt_gen_lib_app_fawhq_e009 ← SOMEIP 信号路由
  │
  ├── gateway
  ├── znet
  └── dji_stream_media_protocol
```

---

## 六、三种 x_dom_* 代码生成机制

### 6.1 类型对比

| 类型 | 生成方式 | 所在位置 | 目标 | 变量名 |
|------|---------|---------|------|--------|
| **预生成 (平台)** | dsar-plat-bf Conan 包发布时预编译好静态库 | dsar-plat-bf/dsar_app/cdd/x_dom_can_rt/ | SOC (BF线) | X_DOM_CAN_RT_SRC |
| **PAL 自动生成** | PAL 工具从 JSON 信号描述文件生成 C++ 代码 | dsar_app/product/<车型>/proxy/x_dom_can_rt_gen_com/ | SOC (BF线+AD线) | x_dom_can_rt_gen_lib |
| **SIP 子模块编译** | dsar-sip Submodule 中的源码直接编译 | dsar-sip/dsar_fw/sip_extend/x_dom_can_rt/ | MCU | — |

### 6.2 PAL 代码生成流程

```
JSON 信号描述文件 (平台仓提供)
  │  dsar-hq-plat/dsar-plat-ad/.../pal_param/
  │  dsar-hq-plat/dsar-plat-bf/.../pal_param/
  │
  ├── PAL 工具 (pal_gen / pal_build)
  │     ├── 读取 JSON
  │     ├── 解析信号定义 (名称/类型/ID/周期/方向)
  │     └── 生成 C++ 代码
  │
  └── 生成产物:
        product/<车型>/proxy/x_dom_can_rt_gen_com/
        ├── SigIf_Cfg.h           信号接口配置
        ├── SigIf_Get_*.cpp       CAN 信号读取函数
        ├── SigIf_Set_*.cpp       CAN 信号写入函数
        └── CMakeLists.txt
```

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

---

## 七、编译产物总览

### 7.1 MCU 侧产物

| 产物 | 说明 | 路径 |
|------|------|------|
| `sailsw3.bin` | FAW 变体 MCU 固件 | `dsar_fw/output/sailsw3/` |
| `sailsw1.bin` | QCOM 变体 MCU 固件 | `dsar_fw/output/sailsw1/` |
| `sailhyp.bin` | Hypervisor 镜像 (预编译，不参与编译) | 外部提供 |

### 7.2 SOC 侧产物

| 产物 | 类型 | 说明 |
|------|------|------|
| `libfawhq_e009_com.so` | 动态库 | BF 通信 SO (含 DSSAD + GE 模块) |
| `libdji_uds_v4.so` | 动态库 | UDS IPv4 诊断 SO |
| `libfawhq_e009_uds_v6.so` | 动态库 | UDS IPv6 诊断 SO |
| `libfawhq_e009_app.so` | 动态库 | AD 自动驾驶应用 SO |
| `dji_ad_app` | 可执行程序 | AD 主进程 |
| `dji_bf_app` | 可执行程序 | BF 主进程 (GE_PROCESS 模式) |
| `dji_doip_service_ipv4` | 可执行程序 | DoIP IPv4 服务 |
| `dji_doip_service_ipv6` | 可执行程序 | DoIP IPv6 服务 |
| `dshell` | 可执行工具 | ECU Shell 终端 (来自 dsar-plat-bf 编译) |

### 7.3 最终镜像打包 (QNX6 文件系统)

```
mkqnx6fsimg
  │
  ├── 输入: SOC 侧所有产物 + 运行时配置
  │     ├── bin/        可执行程序
  │     ├── lib/        .so 动态库
  │     ├── etc/        运行时配置 (JSON + 脚本 + PAL参数)
  │     └── scripts/    运行时脚本
  │
  └── 输出:
        ├── qnx6fsimg (70MB)     → SOC 文件系统镜像
        ├── sailsw3.bin (6.3MB)  → MCU 固件
        └── dsar_xxx.tar.xz (137MB) → 整体发布包
```

---

## 八、依赖关系速查表

### 8.1 静态库 → SO 归属

| 静态库 | 所属 SO | 来源 |
|--------|---------|------|
| `vehicle_proxy_${product}` | com.so + app.so (共享) | dsar-hq: product/*/proxy/ |
| `adapter_${product}` | com.so + app.so | dsar-hq: product/*/adapter/ |
| `param_${product}` | com.so + app.so | dsar-hq: product/*/param/ (PAL生成) |
| `fusa_${product}` | app.so | dsar-hq: product/*/fusa/ |
| `product_runnable_${product}` | app.so | dsar-hq: product/*/runnable/ |
| `domain_function_${product}` | app.so | dsar-hq: product/*/domain_function/ |
| `oem_feature_com_${product}` | com.so | dsar-hq: product/*/oem_feature/ |
| `oem_feature_app_${product}` | app.so | dsar-hq: product/*/oem_feature/ |
| `autosar_com` | com.so | dsar-hq-plat: dsar-sip + dsar-plat-bf |
| `autosar_uds` | uds_v4.so + uds_v6.so | dsar-hq-plat: dsar-plat-bf |
| `dcos_dcms` | com.so | dsar-hq-plat: dsar-plat-bf/cdd/dcms_adapt/ |
| `plat_bf_cdd` | com.so | dsar-hq-plat: dsar-plat-bf/dsar_app/cdd/ |
| `plat_bf_ge` | com.so | dsar-hq-plat: dsar-plat-bf/dsar_app/ge/ |
| `plat_ad_app_core` | app.so | dsar-hq-plat: dsar-plat-ad/.../app_ad_core/ |
| `plat_ad_demo` | app.so | dsar-hq-plat: dsar-plat-ad |
| `plat_ad_fusa_app` | app.so | dsar-hq-plat: dsar-plat-ad/.../fusa_app/ |
| `plat_ad_cndtm` | app.so | dsar-hq-plat: dsar-plat-ad |
| `plat_ad_tlv` | app.so | dsar-hq-plat: dsar-plat-ad |
| `plat_vehicle_proxy` | com.so + app.so | dsar-hq-plat: dsar-plat-ad |
| `plat_common` | com.so + app.so | dsar-hq-plat |
| `gateway_hmicom` | com.so + app.so | dsar-hq-plat: dsar-plat-ad/domain_function/ |
| `x_dom_can_rt_gen_lib_com` | com.so | PAL 自动生成: product/*/proxy/ |
| `x_dom_someip_rt_gen_lib_com` | com.so | PAL 自动生成: product/*/proxy/ |
| `x_dom_can_rt_gen_lib_app` | app.so | PAL 自动生成: product/*/proxy/ |
| `x_dom_someip_rt_gen_lib_app` | app.so | PAL 自动生成: product/*/proxy/ |

### 8.2 编译顺序依赖

```
Stage 1: Conan 依赖下载 + conanbuildinfo 生成 (所有编译的前置条件)
  │
  ├──→ Stage 2: dsar_fw MCU 编译 (独立，不依赖 SOC)
  │       ├── sailsw3
  │       └── sailsw1
  │
  └──→ Stage 3: dsar_app SOC 编译 (独立，不依赖 MCU)
          ├── 各产品的 .so 编译
          ├── 可执行程序编译
          └── dshell 编译
```

---

## 九、关键文件索引

```
=== Conan 依赖管理 ===
dsar-hq/conanfile.py                           构建总入口 (requirements + build + package)
dsar-hq/conan_version_dsar_plat.py             dsar-plat / dsar-vip-plat 版本声明
dsar-hq/conan_version_dsar_plat_bf.py          dsar-plat-bf 版本映射表
dsar-hq/conan_version_dsar_plat_ad.py          dsar-plat-ad 版本映射表
dsar-hq/conan_version_dsar_bsw.py              dsar-bsw 版本声明
dsar-hq/conan_product_requirements.py          各车型依赖版本覆盖表
dsar-hq/conan_config.json                      产品列表 + 镜像打包配置

=== MCU 侧编译 (Makefile) ===
dsar-hq/src/dsar_fw/build/main_sa8xx.mk        SA8650 MCU 编译入口 Makefile
dsar-hq/src/dsar_fw/include/common_include_all.mk  全局 include 路径配置
dsar-hq/src/dsar_fw/proxy_comm/comm/            MCU 代理通信层
dsar-hq/src/dsar_fw/product/faw/                FAW 车型 MCU 代码

=== SOC 侧编译 (CMake) ===
dsar-hq/src/dsar_app/CMakeLists.txt             SOC 编译顶层 CMake
dsar-hq/src/dsar_app/consys/consys_bf.cmake     BF 产品编译宏 (product_build_marcos)
dsar-hq/src/dsar_app/consys/consys_ad.cmake     AD 产品编译宏 (app_product_build_marcos)
dsar-hq/src/dsar_app/entry/CMakeLists.txt       可执行程序 CMake

=== 车型适配代码 (FAW e009 示例) ===
dsar-hq/src/dsar_app/product/faw/fawhq_e009/
├── fawhq_e009_config.cmake                    产品级 CONFIG_APP_* 开关
├── proxy/                                      信号代理 (x_dom_can_rt_gen + vehicle_proxy)
├── adapter/                                    信号适配器
├── param/                                      PAL 自动生成参数代码
├── fusa/                                       功能安全
├── runnable/                                   可运行实体
├── domain_function/                            领域功能
└── oem_feature/                                OEM 特性 (DSSAD/工厂参数/)

=== 平台代码 (Conan 包导入) ===
dsar-hq-plat/dsar-plat-ad/src/dsar_app/         AD 平台源码
dsar-hq-plat/dsar-plat-bf/dsar_app/             BF 平台源码
dsar-hq-plat/dsar-sip/dsar_fw/sip_extend/       SIP 协议栈 MCU 侧源码
dsar-hq-plat/dsar-sip/dsar_app/sip_extend/      SIP 协议栈 SOC 侧源码

=== 编译过程分析 ===
dsar-hq-buildlog/buildlog.txt                   完整编译日志
```
