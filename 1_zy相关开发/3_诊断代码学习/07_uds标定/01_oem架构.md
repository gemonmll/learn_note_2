我看了一遍工程结构：这不是一个完整单体仓库，更像是 DSAR 平台下的“一汽 E001_10 产品 overlay”。底层构建、BSW、SIP AUTOSAR、平台库大量通过外部变量引入，比如 `DSAR_APP_SRC_DIR`、`TOP_DIR`、`DSAR_BSW_*`、`dsar-sip`。当前目录主要放产品适配、配置、生成代码和 OEM 特性。

**整体分层**
- `fawhq_e001_10_app`：SoC/App 侧产品适配，CMake 构建。主干是 SOME/IP COM、UDS/DoIP、DSSAD、GE/IA 产品特性、x_dom 信号生成代码。
- `oem_feature_app`：App 侧通用 OEM 诊断/UDS 特性包。包含安全访问、OTA、工厂参数、标定、订阅、重置、证书等。
- `fawhq_e001_10_fw`：MCU/FW 侧产品适配，Makefile 构建。主干是 CAN/xwire/proxy/诊断配置/降级表/Fee/DCMS topic。
- `oem_feature_fw`：FW 侧 OEM 诊断扩展。包含 DcmApp、DemApp、SecurityAccess、DiagCalib、FactoryParam、DTC 上报、CAN 检测等。

**App 侧架构**
`fawhq_e001_10_app` 的配置开关集中在 [fawhq_e001_10_config.cmake](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/fawhq_e001_10_config.cmake:13)，当前开启了 COM SOME/IP、UDS v4、OEM feature、DSSAD、remote param、dump CAN/ETH、SIT3、degtbl2 等；自动驾驶/泊车/OTA 独立 App 等是关闭的。

核心模块：
- `autosar_adapter/microsar_config_com`：COM/SOMEIP 的 MICROSAR 配置和 BSW 导入，生成 `autosar_com_${PRODUCT_NAME}`，见 [microsar_config_com.cmake](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/autosar_adapter/microsar_config_com/microsar_config_com.cmake:43)。
- `oem_product_feature`：产品 SOME/IP 服务库，入口是 [oem_feature_com_init](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/oem_product_feature/oem_product_feature.cpp:10)，它调用 `faw_someip_init()`。
- `someip_service`：业务 SOME/IP 层。`faw_someip_init()` 配网络/ARP/route，然后初始化服务并启动 `dsar_adaptive_init()`，见 [faw_someip_main.cpp](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/oem_product_feature/someip_service/faw_someip_main.cpp:276)。
- `RTE` 周期调用：COM 的 `Default_BSW_Asyn_Task` 每 5ms 调用 [SomeIp_Service_MainFunction](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/autosar_adapter/microsar_config_com/Appl/GenData/Rte.c:1812)，里面跑 diagnosis/map/dmm/sensor/someip detect 等逻辑。
- `proxy`：x_dom CAN/SOMEIP 生成代码和传感器 proxy，构建入口在 [proxy/CMakeLists.txt](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/proxy/CMakeLists.txt:2)。`radar_proxy_adapt.cpp` 是明显的自动生成适配层，并通过 [add_task_init_stage0](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/proxy/SensorGroup/radar_proxy_adapt.cpp:6511) 注册初始化。
- `dssad_cfg`：DSSAD 产品配置库，接入 GE topic、DMM 数据、DTC 信息和 CAN x_dom。
- `DiagProxy`：GE 诊断代理，连接 UDS 侧 x_dom SOME/IP/CAN 生成接口。

**oem_feature_app 架构**
这里有两条线：`oem_feature.cmake` 只构了一个很薄的 `bf_oem_feature`，实际大头在 UDS AUTOSAR 配置里。`microsar_config_uds.cmake` 把 Dcm/Dem/DoIP/NvM/Fee/Fls 等 BSW 和 OEM 模块一起打进 `autosar_uds_${PRODUCT_NAME}_v4`，见 [microsar_config_uds.cmake](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/microsar_config_uds.cmake:87)。

主要模块：
- `SecurityAccess`：27 服务 seed/key，支持不同车型 cipher/SM4 路径。
- `UpdateAdapter` + `uds_proxy_ota`：刷写/OTA、下载状态、脚本、网关下载服务。
- `FactoryParam`：工厂参数读写同步。
- `DiagCalib`：静态/动态/HMI/DMM 标定。
- `SubscribeProcess`：订阅状态、NVM 持久化和 DCMS service。
- `DcmApp_Reset`：诊断复位请求 topic/service。
- UDS RTE 入口在 [DiagApp.c](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/Source/DiagApp.c:191)，主循环在 [DiagApp_MainFunction](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/Source/DiagApp.c:224)，会跑安全访问、复位检查、订阅初始化、OTA 初始化等。

**FW 侧架构**
`fawhq_e001_10_fw` 是 Makefile 子目录式构建，顶层纳入 adapter、comm_config、proxy、com_if、x_dom_can_rt_config、degrade_table、DiagProxy、xwire 等，见 [Makefile](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_fw/Makefile:1)。当前 FW 配置开启了 x_dom CAN、proxy 全组、COM_IF/SOMEIP_IF、CANGateway、诊断 OEM、CAN 检测等，见 [fawhq_e001_10.config](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_fw/fawhq_e001_10.config:58)。

核心模块：
- `adapter`：产品 DCMS topic 注册在 [dcms_adapter.c](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_fw/adapter/dcms_adapter.c:12)，另有 CAN gateway/pm sync 适配。
- `proxy`：Actuator/BCM/Chassis/HMI/Partner/Sensor 分组。`proxy_main_function()` 100Hz 被调，内部每 50ms 跑各组 20Hz proxy，见 [proxy.c](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_fw/proxy/proxy.c:39)。
- `xwire`：车辆抽象层，`vehicle_init()` 初始化 lat/lon/time/planning/perception/sensor/app 等，`vehicle_main_function()` 100Hz 调用各子模块，见 [vehicle.c](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_fw/xwire/vehicle.c:17)。
- `x_dom_can_rt_config`：MCU 侧新的 CAN signal route，Makefile 中接入外部 `x_dom_can_rt` SDK。
- `Sysdiag_Config`、`DiagInvalidSignal_Config`、`fee_config`、`degrade_table`：诊断、无效信号、NVM/Fee、降级表配置。

**FW OEM 诊断**
`oem_feature_fw` 根据开关纳入 `diagnosis` 和 `can_detect_oem`，见 [oem_feature_fw/Makefile](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/Makefile:4)。诊断包包括 DcmApp、DemApp、SecurityAccess、DiagDtcReport、DiagCalib、FactoryParam 等，见 [diagnosis/Makefile](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/Makefile:2)。

FW 诊断主线：
- `DiagApp_Init()` 注册跨 ECU 诊断、PHM DID、session/reset 回调，并初始化工厂参数同步。
- `DiagApp_MainFunction()` 10ms 跑复位、安全访问、DSSAD DTC 上报、标定、工厂参数，见 [DiagApp.c](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/DiagApp.c:430)。
- `DemProcess_MainFunction()` 跑 SysDiag、休眠存储、快照、NRC22、OTA mode、DTC enable、通信诊断，见 [DemProcess.c](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DemApp/DemProcess.c:443)。

**关键数据流**
- SOME/IP 入站：MICROSAR SoAd/Sd/SomeIpTp -> RTE -> `SomeIp_Service_MainFunction()` -> `sensor/map/diagnosis/dmm` 业务 -> DCMS topic。
- UDS/DoIP：DCM/RTE -> `DiagApp`/`DiagnosticService`/`dji_rte` -> SecurityAccess、UpdateAdapter、FactoryParam、Reset、OTA -> DCMS/minidcos。
- MCU CAN：x_dom CAN RT + proxy 分组 -> xwire/vehicle/proxy data -> DCMS topic -> App/GE/诊断消费。
- 平台通信骨架基本是 DCMS：App COM topic 配置在 [dcms_mcu_config_com.h](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/include/dcms_mcu_config_com.h:57)，UDS topic 在 [dcms_mcu_config_uds.h](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/include/dcms_mcu_config_uds.h:82)，FW topic 在 [fawhq_e001_10_dcms_config.h](c:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_fw/config/app_config/fawhq_e001_10_dcms_config.h:378)。

一个小结论：这个工程的“业务代码”其实被三类胶水串起来了：CMake/Makefile 配置、x_dom 生成信号接口、DCMS topic/service。后续如果要改功能，先判断它属于 COM SOME/IP、UDS 诊断、MCU CAN/proxy，基本就能定位到对应模块。未改文件，也没有跑编译。