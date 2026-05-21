可以把 UDS 链路分成三层看，最清楚：

```text
传输入口层：CAN / DoIP / DCMS 跨核
DCM 诊断层：根据 SID / DID / RID 找服务
业务层：DiagSwc_Callback -> dji_rte -> DiagCalib / DiagnosticService
```

**App 路：DoIP UDS**

App 侧入口是 DoIP：

```text
DoIP 诊断仪
  -> TcpIp / SoAd / DoIP
  -> PduR
  -> DCM
  -> DCM 服务表
  -> DiagSwc_Callback
  -> dji_rte
  -> 业务函数
  -> DCM 组响应
  -> PduR
  -> DoIP
  -> 诊断仪
```

App 侧 DoIP 和 DCM 靠 PduR 连接。关键表在 [PduR_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/PduR_Lcfg.c:188)：

```c
Dcm_CopyRxData
Dcm_CopyTxData
Dcm_StartOfReception
Dcm_TpRxIndication
Dcm_TpTxConfirmation
DoIP_TpTransmit
```

也就是：

```text
DoIP 收到诊断数据
  -> PduR_DoIPTpStartOfReception / CopyRxData / RxIndication
  -> DCM
```

回复时反过来：

```text
DCM
  -> PduR_DcmTransmit
  -> DoIP_TpTransmit
  -> 诊断仪
```

App 侧真正找 SID 的表是 [Dcm_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/Dcm_Lcfg.c:572)：

```c
Dcm_CfgDiagSvcIdLookUpTable
```

SID 找到后，再看 [Dcm_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/Dcm_Lcfg.c:998)：

```c
Dcm_CfgDiagServiceInfo
```

比如：

```text
0x22 -> Dcm_Service22Processor
0x2E -> Dcm_Service2EProcessor
0x31 -> Dcm_Service31Processor

0x14 -> DiagnosticService_0x14_ClearDiagnosticInformation_Proxy
0x19 -> DiagnosticService_0x19_ReadDTCInformation_Proxy
0x28 -> DiagnosticService_0x28_CommunicationControl_Proxy
0x85 -> DiagnosticService_0x85_ControlDTCSetting_Proxy
```

所以 App 侧有两类服务：

```text
App 自己处理：0x22 / 0x2E / 0x31 / 标定 DID/RID
App 转 FW 处理：0x14 / 0x19 / 0x28 / 0x85
```

以 `31 01 51 02` 为例：

```text
SID 0x31
  -> Dcm_Service31Processor
  -> 查 RID 表 Dcm_CfgRidMgrRidLookUpTable
  -> 找到 RID 0x5102
  -> Dcm_CfgRidMgrOpInfo
  -> DiagRid_0x5102_After_sales_calibration...
  -> DiagSwc_Callback.c
  -> rte_config.RoutineControl_ScvGRop.DcmDspRoutine_0x5102_Start
  -> all_afterSales_dynCalib_requestStart()
```

`0x5102` 的 RID 表在 [Dcm_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/Dcm_Lcfg.c:685)，业务映射在 [dji_rte.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/dji_rte.c:122)。

**FW 路：CAN UDS**

FW 侧入口是 CAN：

```text
CAN 诊断仪
  -> CAN Driver / CanIf
  -> CanTp
  -> PduR
  -> DCM
  -> FW 本地服务 或 跨核转 App
  -> DCM 组响应
  -> PduR
  -> CanTp
  -> CAN
  -> 诊断仪
```

这条路里，真正的 UDS TP 是 `CanTp`。FW 的 DCM 收到的已经是完整 UDS PDU，不是单帧 CAN 数据。

FW 侧有一个重要判断点：[DiagApp.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/DiagApp.c:255) 的：

```c
DcmDslServiceRequestSupplierNotification_Indication(...)
```

它会对一些 SID 做前置检查和拒绝。例如源码里能看到：

```text
0x14 / 0x19：DoCAN 直接请求时拒绝
0x31 / 0x34-0x38 / 0x28 / 0x85：FW 本地直接处理会拒绝
```

这说明很多业务不应该落在 FW 本地 DCM 里处理，而是要走跨核或者由 App 侧 DoIP 处理。

**跨核链路**

跨核不是 CAN TP，也不是 DoIP TP，而是 DCMS 透传完整 UDS 报文：

```text
[SID][payload...]
[SID + 0x40][response payload...]
[0x7F][SID][NRC]
```

App 收到 DoIP 请求但服务属于 FW 时，会走 [Dcm_Mcu_Proxy.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/Source/Dcm_Mcu_Proxy.c:704)：

```c
cross_uds_req_msg_context(...)
```

它会：

```text
1. 拼出完整 UDS 请求：SID + reqData
2. call_uds_service_async_cross_domain 发给 FW
3. get_resp_data_cross_domain 等 FW 响应
4. 正响应：Dcm_UtiProvideResDataAsUN 填回 DCM
5. 负响应：设置 ErrorCode
6. 0x78：继续 pending
```

FW 收到 App 跨核请求后，在 [Diag_CrossDiag.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/Diag_CrossDiag.c:249)：

```c
cross_diag_request_cb(...)
```

它把跨核收到的 UDS 重新灌进 FW DCM：

```c
Dcm_StartOfReception(...)
Dcm_CopyRxData(...)
Dcm_TpRxIndication(...)
```

FW DCM 处理完响应后，通过 [PduR_UpTransmit_doip_cross_diag](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/Diag_CrossDiag.c:390) 把响应取出来，再用 DCMS 发回 App。

**DiagProxy_Cfg 和 dji_rte 的位置**

这两个都不属于 TP 层。

```text
TP 层：CanTp / DoIP / PduR / DCMS
DCM 表层：Dcm_Lcfg.c + DiagProxy_Cfg.h
业务跳转层：DiagSwc_Callback.c + dji_rte.c
业务实现层：DiagCalib_Static / DiagCalib_Dynamic / DiagnosticService
```

`DiagProxy_Cfg.h` 的作用是给 DCM 表提供产品级配置，比如：

```text
DID 有哪些、长度是多少、读写函数是谁
RID 有哪些、Start/RequestResult 函数是谁
标定结果、错误码、delta 数据怎么映射
```

例如 `DCM_CFGRIDMGROPINFO`、`DCM_CFGDIDMGRSIGNALOPCLASSINFO` 定义在 [DiagProxy_Cfg.h](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/DiagProxy/DiagProxy_Cfg.h:67)。

`dji_rte.c` 的作用是运行时函数表，把 generated callback 转到真实业务函数：

```text
DiagRid_0x5102...
  -> rte_config.RoutineControl_ScvGRop.DcmDspRoutine_0x5102_Start
  -> all_afterSales_dynCalib_requestStart
```

所以你学习链路时可以按这个顺序查：

```text
1. 入口：App 看 DoIP/PduR，FW 看 CanTp/PduR/DCM
2. SID：Dcm_CfgDiagSvcIdLookUpTable / Dcm_CfgDiagServiceInfo
3. DID：Dcm_CfgDidMgrDidLookUpTable / Dcm_CfgDidMgrSignalOpClassInfo
4. RID：Dcm_CfgRidMgrRidLookUpTable / Dcm_CfgRidMgrOpInfo
5. 中转：DiagSwc_Callback.c
6. 业务：dji_rte.c -> DiagCalib_xxx.c
7. 回复：业务返回值 -> DCM 组包 -> PduR -> DoIP 或 CanTp
```

一句话总结：**App 是 DoIP UDS 入口，FW 是 CAN UDS 入口；PduR 把 TP 和 DCM 接起来，DCM 表找到服务，`DiagProxy_Cfg` 提供表配置，`dji_rte` 把 DCM 回调转到真正业务函数。**