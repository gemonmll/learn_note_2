## 问题  理解下udstp，就是fw和app之间uds是如何传输的，各个核之间负责哪些服务

**核心结论**

这里的 `UDSTP` 要分两段理解：

```text
诊断仪
  |
  |  UDS over CAN / CanTp
  v
FW/MCU核：CAN + CanTp + PduR + DCM
  |
  |  DCOS/DCMS 跨核通道，透传完整 UDS APDU
  v
App/SoC核：App DCM / DiagProxy / dji_rte / 标定 engine
```

也就是说，**外部 CAN 这一段是真正的 UDS TP/CanTp**；但 **FW 和 App 跨核这一段不是 CAN TP，而是把完整 UDS 报文当作 payload 通过 DCOS/DCMS 传过去**。跨核传的是类似：

```text
Request:  [SID][sub-function / DID / RID / data...]
Response: [SID+0x40][data...]
Negative: [0x7F][SID][NRC]
Pending:  [0x7F][SID][0x78]
```

所以更准确地说，跨核是 **UDS APDU tunneling**，不是重新跑一套 CanTp。

**FW 侧职责**

FW 是入口和出口，主要负责：

1. 接收诊断仪 CAN UDS 请求。
2. 由 CanTp/PduR/DCM 完成 CAN TP 重组。
3. 判断服务是否本核处理，还是跨核转给 App。
4. 把 App 返回的 UDS 响应重新通过 CAN TP 发回诊断仪。
5. 负责一部分 MCU/底层相关服务，比如 CAN 通信控制、DTC、网络管理、部分前置条件/状态检查。

你当前看到的 FW 侧跨核诊断入口在 [DiagApp.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/DiagApp.c:388)：

```c
dcms_mcu_topic_setup_callback(
    DCMS_TOPIC_DIAG_CROS_UDS_REQ,
    cross_diag_request_cb,
    NULL,
    0);

dcms_mcu_service_client_setup_resp_callback(
    DCMS_TOPIC_DIAG_CROS_UDS_RESP,
    cross_diag_response_cb);
```

对应 topic 在 [fawhq_e001_10_dcms_config.h](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_fw/config/app_config/fawhq_e001_10_dcms_config.h:493)：

```c
/sys/cross_domain_req/uds
/sys/cross_domain_resq/uds
```

这两个就是跨核 UDS 请求/响应通道。

**跨核请求如何进入 DCM**

FW 收到跨核 UDS 后，会进入 [Diag_CrossDiag.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/Diag_CrossDiag.c:249) 的：

```c
cross_diag_request_cb(...)
```

这里有个很关键的动作：它不是自己手写处理所有 UDS，而是把收到的 UDS payload 重新灌进 DCM TP 接口：

```c
Dcm_StartOfReception(...)
Dcm_CopyRxData(...)
Dcm_TpRxIndication(...)
```

也就是让 DCM 觉得“我收到了一帧完整的 TP UDS 请求”。后续 DCM 处理完响应后，会走 [Diag_CrossDiag.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/Diag_CrossDiag.c:388) 的：

```c
PduR_UpTransmit_doip_cross_diag(...)
```

这里把 DCM 输出响应缓存到 `doip_cross_diag_resp`，再通过：

```c
DCMS_TOPIC_DIAG_CROS_UDS_RESP
```

发回另一个核。

注意：`doip_cross_diag_request_ack = {0xFF, 0xFF, 0xFF}` 只是 DCMS 层服务调用 ack，不是 UDS 正响应。真正的 UDS 响应还是 `0x6x...` 或 `0x7F...`。

**App 侧职责**

App 更像业务诊断核，负责：

1. 产品/OEM 诊断逻辑。
2. 标定相关服务，比如 `0x31 RoutineControl`、部分 `0x22 DID`、`0x2E WriteDataByIdentifier`。
3. 调用真正的标定 engine。
4. 把标定结果包装成 UDS response。
5. 对接 `DiagProxy`、`dji_rte`、DMM、HMI、SomeIP 等业务模块。

你之前说的标定链路，可以理解为：

```text
诊断仪发 0x31 / 0x22 / 0x2E
        |
FW 收 CAN UDS
        |
FW 判断这是 App/标定服务
        |
通过 DCOS/DCMS 跨核发给 App Diag
        |
App Diag / DiagProxy / dji_rte 分发到标定 engine
        |
标定 engine 返回结果
        |
App 组 UDS response
        |
DCMS 回 FW
        |
FW 通过 CanTp 回诊断仪
```

App 这边能看到不少标定 topic，例如 [dcms_mcu_config_uds.h](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/include/dcms_mcu_config_uds.h:96) 里的：

```c
/sys/dmm_calib/v1
/sys/start_calib_rtos
/sys/eol_result_rtos
/sys/inquire_parameter_service
/sys/send_stedelta
/sys/calib_precondition_check
```

这些更接近“App diag 到标定/业务 engine”的内部链路。

**为什么还会看到 App 转 FW**

还有一个容易混淆的点：代码里也能看到 App 把 UDS 转给 FW，例如 [Dcm_Mcu_Proxy.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/Source/Dcm_Mcu_Proxy.c:698) 的：

```c
cross_uds_req_msg_context(...)
```

它会调用：

```c
call_uds_service_async_cross_domain(...)
get_resp_data_cross_domain(...)
```

这说明系统不是单向的，而是**双向代理**：

```text
CAN 诊断仪 -> FW -> App
用于 App-owned 服务，比如标定。

DoIP/SomeIP/App 侧诊断 -> App -> FW
用于 FW-owned 服务，比如 0x14、0x19、0x28、0x85 等。
```

比如 App 侧的 `0x14 ClearDiagnosticInformation`、`0x19 ReadDTCInformation`、`0x28 CommunicationControl`、`0x85 ControlDTCSetting` 都有 proxy 包装，典型是 App DCM 收到请求，但真正 owner 在 FW，所以 App 再跨核问 FW。

**各核服务边界**

我建议你按这个原则记：

```text
FW/MCU 核：
CAN 入口、CanTp、PduR、底层 DCM、网络/通信控制、MCU 侧 DTC、基础状态、防止非法直接请求。

App/SoC 核：
业务诊断、标定 engine、产品功能、复杂 DID/RID、SomeIP/DoIP 侧业务、DiagProxy/dji_rte 分发。

DCOS/DCMS：
跨核运输层，负责把 UDS APDU 和业务 topic 在两个核之间送达。

DiagProxy_Cfg：
App 侧诊断服务到具体 handler/业务模块的配置表或路由关系。

dji_rte：
更靠近 App 内部运行时接口，把 diag/proxy 请求转成具体业务或标定 engine 调用。
```

这里最重要的一点是：**UDS 的语义没有在跨核时丢失**。跨核虽然不用 CAN TP，但依然保留 UDS 的 SID、正响应、负响应、`0x78 response pending` 这些语义。FW 和 App 只是把“谁负责处理这个服务”拆开了。

----
## 问题二 我的理解是fw是can uds， app侧有doipuds,基于这个结论再帮我分析下
你的这个理解我觉得更准确：

```text
FW/MCU 核：CAN UDS 入口
App/SoC 核：DoIP UDS 入口
两边通过 DCOS/DCMS 做跨核 UDS 代理
```

可以把整个诊断架构看成“双入口、按服务归属转发”。

**整体架构**

```text
                 CAN 诊断仪
                    |
                    v
        FW: CAN / CanTp / PduR / DCM
                    |
        App-owned 服务时跨核转发
                    |
                    v
        App: DCM / DiagProxy / dji_rte / DiagCalib
                    ^
                    |
        FW-owned 服务时跨核转发
                    |
                    v
        App: DoIP / SoAd / TcpIp / PduR / DCM
                    ^
                    |
              DoIP 诊断仪
```

也就是说：

```text
CAN UDS 请求先进 FW。
DoIP UDS 请求先进 App。
但最终服务不一定在入口核处理，而是看这个 SID/DID/RID 的 owner 在哪一边。
```

App 侧确实有完整 DoIP UDS 栈。`microsar_config_uds.cmake` 明确把 `Dcm`、`DoIP`、`PduR`、`SoAd`、`TcpIp` 编进 App 侧 UDS 库：[microsar_config_uds.cmake](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/microsar_config_uds.cmake:10)。

App 的 PduR 配置也能看到典型链路：

```text
DoIP_Tester_to_Phys / DoIP_Tester_to_Func -> DCM
DCM response -> DoIP_Phys_to_Tester
```

对应在 [PduR_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/PduR_Lcfg.c:190) 和 [PduR_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/PduR_Lcfg.c:277)。

**App 侧 UDS 服务分布**

App 的 DCM 服务表里能看到这些 SID：

```text
0x10, 0x11, 0x14, 0x19, 0x22, 0x27, 0x28, 0x2E,
0x31, 0x34, 0x35, 0x36, 0x37, 0x38, 0x3E, 0x85
```

在 [Dcm_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/Dcm_Lcfg.c:1002)。

这里很关键：

```text
0x22 -> Dcm_Service22Processor
0x2E -> Dcm_Service2EProcessor
0x31 -> Dcm_Service31Processor
```

这些是 App 本地 DCM 直接处理，再通过 DID/RID 配置进入业务函数。

而这些：

```text
0x14 -> ClearDiagnosticInformation_Proxy
0x19 -> ReadDTCInformation_Proxy
0x28 -> CommunicationControl_Proxy
0x85 -> ControlDTCSetting_Proxy
```

在 App 侧是 proxy，说明 **DoIP 进来的这些服务虽然先进 App，但实际 owner 在 FW**。App 会通过跨核 UDS 转给 FW。实现入口在 [Dcm_Mcu_Proxy.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/Source/Dcm_Mcu_Proxy.c:704)，里面调用：

```c
call_uds_service_async_cross_domain(...)
get_resp_data_cross_domain(...)
```

**FW 侧 UDS 服务分布**

FW 是 CAN UDS 入口，同时也是 MCU/车身底层能力 owner。FW 初始化跨核 UDS topic 在 [DiagApp.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/DiagApp.c:396)：

```c
DCMS_TOPIC_DIAG_CROS_UDS_REQ
DCMS_TOPIC_DIAG_CROS_UDS_RESP
```

对应 DCMS topic 是：

```text
/sys/cross_domain_req/uds
/sys/cross_domain_resq/uds
```

在 [fawhq_e001_10_dcms_config.h](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_fw/config/app_config/fawhq_e001_10_dcms_config.h:493)。

FW 收到 App 转过来的 DoIP UDS 请求后，会进 [Diag_CrossDiag.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/Diag_CrossDiag.c:249) 的：

```c
cross_diag_request_cb(...)
```

里面会把跨核收到的 UDS payload 注入 FW DCM：

```c
Dcm_StartOfReception(...)
Dcm_CopyRxData(...)
Dcm_TpRxIndication(...)
```

响应再由 [PduR_UpTransmit_doip_cross_diag](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/Diag_CrossDiag.c:390) 捕获，通过 DCMS 回给 App。

所以 `doip_cross_diag` 这个名字容易误导，它不是说 FW 有 DoIP 栈，而是说：**App DoIP 收到某些 FW-owned 服务后，跨核转给 FW 处理**。

**标定链路放到这个结论里**

标定大概率是 App-owned 服务，尤其是这些 RID/DID：

```text
RID: 0x5100, 0x5101, 0x5102, 0x5103
DID: 0x2000, 0x2001, 0x2003, 0x2004, 0x2006, 0x2008, 0x200A...
```

它们在 [DiagProxy_Cfg.h](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/DiagProxy/DiagProxy_Cfg.h:67) 和 [dji_rte.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/dji_rte.c:118) 里都能看到映射。

所以标定会有两种入口：

```text
1. CAN 诊断仪 -> FW CAN UDS -> 跨核到 App -> App 标定服务
2. DoIP 诊断仪 -> App DoIP UDS -> App 标定服务
```

两条入口最后都会落到 App 侧同一套业务：

```text
App DCM
  -> dji_rte
  -> DiagCalib_Static / DiagCalib_Dynamic
  -> DCMS topic
  -> 标定 / 感知 / DMM engine
```

例如 `dji_rte.c` 里把 `0x5100/0x5101/0x5102/0x5103` 映射到静态/动态标定函数；而 `DiagCalib_Dynamic.c` 里 `all_afterSales_dynCalib_requestStart` 会做前置条件检查、DMM 模式设置、再发起标定。

**一句话总结**

这套架构不是“FW 诊断”和“App 诊断”二选一，而是：

```text
FW 负责 CAN UDS 入口和 MCU-owned 服务。
App 负责 DoIP UDS 入口和 App-owned 业务服务。
跨核 UDS 代理负责把请求送到真正 owner 的核。
标定属于 App 业务，但 CAN 入口来的标定请求要先经过 FW。
```