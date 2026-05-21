可以这样记：**UDSTP 本身不负责找业务函数，它只负责把 UDS 报文送到 DCM；真正“找服务”的是 DCM 生成表，`DiagProxy_Cfg` 和 `dji_rte` 是 DCM 到业务函数之间的两级映射。**

**1. UDSTP 靠什么联系起来**

在 App 的 DoIP UDS 入口里，链路是：

```text
DoIP
  -> PduR
  -> DCM
  -> SID 服务表
  -> DID/RID 表
  -> DiagSwc_Callback
  -> dji_rte
  -> DiagCalib / DiagnosticService / Proxy
```

DoIP 和 DCM 是靠 PduR 配置表接起来的。关键表在 [PduR_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/PduR_Lcfg.c:188)：

```c
PduR_MmRom[0] -> DCM: Dcm_CopyRxData / Dcm_StartOfReception / Dcm_TpRxIndication
PduR_MmRom[1] -> DoIP: DoIP_TpTransmit
```

然后路由表在 [PduR_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/PduR_Lcfg.c:237)：

```text
DoIP_Tester_to_Phys / DoIP_Tester_to_Func -> DCM
DCM -> DoIP_Phys_to_Tester
```

所以 App 侧 DoIP UDS 的 TP 连接核心就是：

```text
DoIP PDU ID <-> PduR route table <-> DCM Rx/Tx PDU ID
```

跨核那段不是标准 CanTp/DoIP TP，而是 DCMS 透传 UDS APDU。比如 App 要把 DoIP 收到的 FW-owned 服务转给 FW，会走 [Dcm_Mcu_Proxy.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/Source/Dcm_Mcu_Proxy.c:704) 的 `cross_uds_req_msg_context()`；FW 收到后在 [Diag_CrossDiag.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_fw/diagnosis/DcmApp/Diag_CrossDiag.c:249) 里重新灌入 DCM：

```c
Dcm_StartOfReception(...)
Dcm_CopyRxData(...)
Dcm_TpRxIndication(...)
```

**2. 从哪张表找到对应服务**

先看 SID 表。

App 侧支持哪些 SID，看 [Dcm_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/Dcm_Lcfg.c:572)：

```c
Dcm_CfgDiagSvcIdLookUpTable
```

里面有：

```text
0x10, 0x11, 0x14, 0x19, 0x22, 0x27, 0x28, 0x2E,
0x31, 0x34, 0x35, 0x36, 0x37, 0x38, 0x3E, 0x85
```

SID 对应哪个 processor，看 [Dcm_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/Dcm_Lcfg.c:998)：

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

也就是说，`0x22/0x2E/0x31` 这种 App 本地业务继续查 DID/RID 表；`0x14/0x19/0x28/0x85` 这种会走 App -> FW 跨核代理。

**3. DID / RID 从哪张表继续找**

`0x22/0x2E` 查 DID：

```text
Dcm_CfgDidMgrDidLookUpTable
Dcm_CfgDidMgrDidInfo
Dcm_CfgDidMgrDidOpInfo
Dcm_CfgDidMgrSignalOpClassInfo
```

入口在 [Dcm_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/Dcm_Lcfg.c:627)。比如 `0x2008` 在 DID lookup 里，然后操作函数来自 [DiagProxy_Cfg.h](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/DiagProxy/DiagProxy_Cfg.h:95) 的：

```c
DCM_CFGDIDMGRSIGNALOPCLASSINFO
```

`0x31` 查 RID：

```text
Dcm_CfgRidMgrRidLookUpTable
Dcm_CfgRidMgrRidInfo
Dcm_CfgRidMgrOpInfo
```

入口在 [Dcm_Lcfg.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/GenData/Dcm_Lcfg.c:685)。比如 `0x5102` 在 RID lookup 里，再通过 [DiagProxy_Cfg.h](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/fawhq_e001_10_app/DiagProxy/DiagProxy_Cfg.h:67) 的：

```c
DCM_CFGRIDMGROPINFO
```

找到：

```c
DiagRid_0x5102_After_sales_calibration_0_1_Bytes_Start
DiagRid_0x5102_After_sales_calibration_0_2_Bytes_RequestResults
```

**4. DiagProxy_Cfg 的作用**

`DiagProxy_Cfg.h` 不在 TP 层，它不负责传输。它的作用是：**给 DCM 生成表提供产品级 DID/RID 配置**。

它主要干三件事：

```text
1. 定义 DID/RID 的长度、权限、操作函数。
2. 把 RID 0x5100/0x5101/0x5102/0x5103 映射到 DiagRid_xxx 回调。
3. 定义标定结果解析用的 camera/error/delta 映射表。
```

所以它是在这一级生效：

```text
DCM Service22/2E/31
  -> DCM DID/RID 表
  -> DiagProxy_Cfg 里的函数指针/长度配置
```

**5. dji_rte 的作用**

`dji_rte` 是下一层：**把 DCM 回调函数再转到真正业务函数**。

例如 `RID 0x5102`：

```text
0x31
 -> Dcm_Service31Processor
 -> Dcm_CfgRidMgrRidLookUpTable 找到 0x5102
 -> Dcm_CfgRidMgrOpInfo 找到 DiagRid_0x5102...
 -> DiagSwc_Callback.c
 -> rte_config.RoutineControl_ScvGRop.DcmDspRoutine_0x5102_Start
 -> dji_rte.c
 -> all_afterSales_dynCalib_requestStart
```

对应代码在 [DiagSwc_Callback.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/Appl/Source/DiagSwc_Callback.c:747) 和 [dji_rte.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/autosar_adapter/microsar_config_uds_v4/dji_rte.c:122)。

所以一句话：

```text
DiagProxy_Cfg 是 DCM 表配置。
dji_rte 是 DCM 回调到真实业务函数的跳板。
UDSTP/PduR 只负责把报文送进 DCM，不负责业务分发。
```

以 `31 01 51 02` 为例，最终会落到 [DiagCalib_Dynamic.c](C:/Users/c-ziming.jiang/Desktop/bak_code/code_bak/oem_feature_app/DiagCalib/DiagCalib_Dynamic.c:321) 的 `all_afterSales_dynCalib_requestStart()`，再通过 DCMS topic 拉起真正标定流程。