下面按一条完整 UDS 请求来串起来：**接收侧是“CanTp 推数据给 DCM”，响应侧是“CanTp 从 DCM 拉数据发送”，通知靠回调，DCM 内部再靠事件位调度。**

**1. CAN 请求进入 ECU**

诊断请求 CAN ID 在 CanIf 配置里：

- 功能寻址：`0x7DF`
- 物理寻址：`0x766`

见 [CanIf_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/CanIf_Lcfg.c:284)。

CanIf 收到 CAN Driver 上报后，会按 CAN ID 找 `CanIf_RxPduConfig`，再通过 `RxIndicationFctListIdx = 2` 找到：

```c
CanTp_RxIndication
```

所以第一段是：

```text
CAN Driver
-> CanIf_RxIndication
-> CanTp_RxIndication(CanTpRxPduId, PduInfo)
```

这个回调函数表在 [CanIf_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/CanIf_Lcfg.c:252)。

**2. CanTp 解析 ISO-TP 并请求 DCM 接收**

CanTp 收到后先判断是：

```text
SF 单帧
FF 首帧
CF 连续帧
FC 流控帧
```

如果是诊断请求的 SF/FF，CanTp 不直接找 DCM，而是问 PduR：

```text
CanTp
-> PduR_CanTpStartOfReception()
-> PduR_LoTpStartOfReception()
-> Dcm_StartOfReception()
```

CanTp 到 PduR 的宏在 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:179)。PduR 根据 `PduR_Lcfg.c` 查到目标模块是 DCM，DCM 回调配置在 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:151)。

这一步的语义是：

```text
CanTp：我收到一个长度 N 的诊断请求。
PduR：查路由，目标上层是 DCM。
PduR -> DCM：你能收 N 字节吗？
Dcm：可以，返回 BUFREQ_OK，并给出 bufferSize。
```

**3. 请求数据怎么进 DCM**

后续每收到一段 payload，CanTp 调：

```text
CanTp
-> PduR_CanTpCopyRxData()
-> PduR_LoTpCopyRxData()
-> Dcm_CopyRxData()
```

PduR 转发 `CopyRxData` 的地方在 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:741)。DCM 真正拷贝数据的位置在 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:16814)。

数据流是：

```text
CanTp 当前 payload buffer
-> PduR 原样转发 PduInfoType
-> DCM Rx buffer
```

所以接收侧不是 DCM 主动取数据，而是 **CanTp 通过 CopyRxData 把数据推给 DCM**。

如果是多帧请求，ECU 还会通过 CanTp 给 tester 发 FlowControl，比如 `FC.CTS`，这属于 CanTp 自己的 ISO-TP 状态机行为。

**4. 完整请求接收完成后的通知**

当 CanTp 判断整个 UDS 请求已经收完，会通知 PduR：

```text
CanTp_RxInit(... NotifyCode = OK)
-> CanTp_PduRRxIndication()
-> PduR_CanTpRxIndication()
-> PduR_LoTpRxIndication()
-> Dcm_TpRxIndication(E_OK)
```

CanTp 通知 PduR 的位置在 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:4256)。PduR 再通知 DCM 的位置在 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:923)。

DCM 收到 `Dcm_TpRxIndication(E_OK)` 后，会把请求交给 DCM 内部任务系统。这里不是信号量，而是事件位：

[Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:32512)

```c
Dcm_TskSetEventByThread(
    DCM_TSK_ID_DIAG_RX,
    DCM_TSK_EV_DIAG_RX_NEW_REQ,
    ...
);
```

然后 `Dcm_MainFunction` 里的 scheduler 处理这个事件，解析 SID，根据 [Dcm_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/Dcm_Lcfg.c:435) 找服务函数，比如：

```text
0x19 -> Dcm_Service19Processor
0x14 -> Dcm_Service14Processor
0x22 -> Dcm_Service22Processor
```

DTC 相关服务再由 DCM 调 DEM API。

**5. DCM 准备响应**

服务处理完成后，DCM 把正响应或负响应写进自己的响应 buffer。

然后 DCM 请求发送：

```text
Dcm_NetTransmitUsdtResponse()
-> PduR_DcmTransmit()
-> PduR_UpTransmit()
-> CanTp_Transmit()
```

DCM 发起位置在 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:16002)。PduR wrapper 在 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:460)。

这一步只是启动发送，告诉 CanTp：

```text
我要发一个长度 N 的响应。
```

不是把所有响应数据一次性塞给 CanTp。

**6. 响应数据怎么从 DCM 出来**

CanTp 发送响应时，需要组 SF/FF/CF。每次需要 payload，就通过 PduR 反向向 DCM 拉数据：

```text
CanTp
-> PduR_CanTpCopyTxData()
-> PduR_LoTpCopyTxData()
-> Dcm_CopyTxData()
```

CanTp 拉数据的位置在 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:4565)。PduR 转给 DCM 的地方在 [PduR_RmTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR_RmTp.c:923)。DCM 拷响应数据的位置在 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:16958)。

响应数据流是：

```text
DCM response buffer
-> Dcm_CopyTxData
-> PduR
-> CanTp payload buffer
-> CanIf_Transmit
-> CAN Driver
```

响应 CAN ID 是 `0x706`，见 [CanIf_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/CanIf_Lcfg.c:433)。

**7. 响应发送完成后的通知**

每个 CAN TP 帧发出去后，底层会确认：

```text
CanIf_TxConfirmation()
-> CanTp_TxConfirmation()
```

这里 CanTp 会用 `CanTp_TxSemaphores` 反查这个 `TxPduId` 属于哪个 CanTp 状态机。它不是通知 DCM 的信号量，只是 CanTp 内部的“TxPduId 占用和确认归属”记录。

设置 semaphore 的地方在 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:4126)，释放并反查的地方在 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:6021)。

如果是最后一帧发送完成，CanTp 才最终通知上层：

```text
CanTp_TxInit(... NotifyCode = OK)
-> CanTp_PduRTxConfirmation()
-> PduR_CanTpTxConfirmation()
-> PduR_LoTpTxConfirmation()
-> Dcm_TpTxConfirmation()
```

CanTp 通知 PduR 的位置在 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:4320)。PduR 转 DCM 的位置在 [PduR_RmTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR_RmTp.c:845)。DCM 入口在 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:36149)。

DCM 收到最终发送确认后，会释放 buffer，并设置内部事件：

[Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:32583)

```c
Dcm_TskSetEventByThread(
    DCM_TSK_ID_DIAG_WORK,
    DCM_TSK_EV_DIAG_WORK_TX_END_FINAL,
    ...
);
```

**总图**

```text
请求接收：
CAN Driver
-> CanIf
-> CanTp_RxIndication
-> PduR_CanTpStartOfReception
-> Dcm_StartOfReception
-> PduR_CanTpCopyRxData
-> Dcm_CopyRxData
-> PduR_CanTpRxIndication
-> Dcm_TpRxIndication
-> DCM 内部事件
-> DCM 服务处理
-> DEM / DID / Routine 等

响应发送：
DCM 服务写响应 buffer
-> PduR_DcmTransmit
-> CanTp_Transmit
-> PduR_CanTpCopyTxData
-> Dcm_CopyTxData
-> CanIf_Transmit
-> CAN Driver
-> CanIf_TxConfirmation
-> CanTp_TxConfirmation
-> PduR_CanTpTxConfirmation
-> Dcm_TpTxConfirmation
-> DCM 内部事件
```

最核心的两句话：

```text
请求侧：CanTp 通过 PduR 把数据推给 DCM，完成后用 RxIndication 通知 DCM。
响应侧：DCM 通过 PduR 启动发送，CanTp 通过 PduR 从 DCM 拉数据，完成后用 TxConfirmation 通知 DCM。
```

通知机制不是靠跨模块信号量，而是靠 AUTOSAR 标准回调；`CanTp_TxSemaphore` 只是 CanTp 内部用来关联 `TxPduId -> CanTp 状态机` 的占用标记。