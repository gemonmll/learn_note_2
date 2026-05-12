可以，把 PduR 这一层单独拆开看，核心是四张表：`RmSrcRom -> RmDestRom -> RmGDestRom -> MmRom`。

**PduR 角色**

这份工程里诊断 Rx 是 **TP unbuffered API forwarding**：

`CanTp -> PduR -> Dcm`

也就是说，PduR **不缓存、不解析、不重组 UDS 数据**。重组是 CanTp 做的，buffer 是 DCM 管的，PduR 只根据 PDU ID 查表，然后把 `StartOfReception / CopyRxData / RxIndication` 转调给 DCM。

**1. CanTp 传给 PduR 的 id**

CanTp 配置里，两个诊断 RxSdu 的 `PduRRxSduId` 分别是：

| 请求 | CanTp RxSdu | 传给 PduR 的 id |
|---|---|---|
| 功能寻址 | `CanTpRxNSdu_1062ea40` | `PduRConf_PduRSrcPdu_PduRSrcPdu_dc13d0b4 = 0` |
| 物理寻址 | `CanTpRxNSdu_9c54fe06` | `PduRConf_PduRSrcPdu_PduRSrcPdu_eac1f0e9 = 1` |

配置在 [CanTp_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/CanTp_Lcfg.c:117)，宏值在 [PduR_Cfg.h](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Cfg.h:204)。

CanTp 运行时会用这个 `PduRRxSduId` 调 PduR：

- 首次申请接收 buffer：`PduR_CanTpStartOfReception`，见 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:2131)
- 拷贝接收数据：`PduR_CanTpCopyRxData`，见 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:2824)
- 接收完成通知：`PduR_CanTpRxIndication`，见 [CanTp.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/CanTp/CanTp.c:4256)

**2. PduR wrapper 入口**

生成文件里这几个函数只是薄包装：

- `PduR_CanTpStartOfReception -> PduR_LoTpStartOfReception`
- `PduR_CanTpCopyRxData -> PduR_LoTpCopyRxData`
- `PduR_CanTpRxIndication -> PduR_LoTpRxIndication`

位置在 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:528)。

**3. id 如何变成内部路由源**

PduR 先检查 `id < PduR_GetSizeOfRxTp2Dest()`，这份工程大小是 `2`，见 [PduR_Lcfg.h](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.h:743)。

然后关键宏是：

```c
PduR_GetRmSrcRomIdxOfRxTp2Dest(id) = id + 4
```

见 [PduR_Lcfg.h](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.h:816)。

所以：

| CanTp 传入 id | PduR 内部 `rmSrcRomIdx` |
|---|---|
| `0` 功能诊断 | `4` |
| `1` 物理诊断 | `5` |

这里要注意：`PduRConf_PduRSrcPdu_* = 0/1` 是 **TpRxSrc handle space** 里的 ID，不是直接等于 `PduR_RmSrcRom[]` 全局数组下标。PduR 通过宏把它偏移到 `RmSrcRom[4]/[5]`。

**4. 功能寻址路由展开**

功能诊断请求：

`id=0 -> RmSrcRom[4] -> RmDestRom[8] -> RmGDestRom[11] -> MmRom[3] -> Dcm`

对应配置：

- `RmSrcRom[4]` 是 `PduRSrcPdu_dc13d0b4`，源模块 `MmRomIdx=1`，也就是 CanTp；它的目标范围最终指到 `RmDestRom[8]`，见 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:251)。
- `RmDestRom[8]` 的路由类型是 `PDUR_TP_UNBUFFERED_RX_API_FWD_ROUTINGTYPEOFRMDESTROM`，目标全局 PDU 是 `RmGDestRom[11]`，见 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:174)。
- `RmGDestRom[11]` 的 `DestHnd` 是 `DcmConf_DcmDslProtocolRx_Meg_FunctionDiag... = 0`，目标模块 `MmRomIdx=3`，也就是 DCM，见 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:213)。
- `MmRom[3]` 里挂的函数指针就是 `Dcm_CopyRxData / Dcm_StartOfReception / Dcm_TpRxIndication`，见 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:146)。

最终实际调用类似：

```c
Dcm_StartOfReception(0, info, TpSduLength, bufferSizePtr);
Dcm_CopyRxData(0, info, bufferSizePtr);
Dcm_TpRxIndication(0, result);
```

这个 `0` 就是 DCM 的功能诊断 RxPduId，见 [Dcm_Cfg.h](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/Dcm_Cfg.h:497)。

**5. 物理寻址路由展开**

物理诊断请求：

`id=1 -> RmSrcRom[5] -> RmDestRom[10] -> RmGDestRom[12] -> MmRom[3] -> Dcm`

最终调用类似：

```c
Dcm_StartOfReception(1, info, TpSduLength, bufferSizePtr);
Dcm_CopyRxData(1, info, bufferSizePtr);
Dcm_TpRxIndication(1, result);
```

这个 `1` 是 DCM 的物理诊断 RxPduId，见 [Dcm_Cfg.h](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/Dcm_Cfg.h:498)。

**6. 三个运行时函数怎么转发**

`PduR_LoTpStartOfReception` 在 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:605)：

```c
rmSrcRomIdx = PduR_GetRmSrcRomIdxOfRxTp2Dest(id);
rmDestRomIdx = PduR_GetRmDestRomStartIdxOfRmSrcRom(rmSrcRomIdx);
rmGDestRomIdx = PduR_GetRmGDestRomIdxOfRmDestRom(rmDestRomIdx);

retVal =
  PduR_GetUpTpStartOfReceptionFctPtrOfMmRom(
    PduR_GetMmRomIdxOfRmGDestRom(rmGDestRomIdx)
  )(
    PduR_GetDestHndOfRmGDestRom(rmGDestRomIdx),
    info,
    TpSduLength,
    bufferSizePtr
  );
```

这行实际就是查到 `MmRom[3].UpTpStartOfReceptionFctPtr = Dcm_StartOfReception`，然后把 `DestHnd` 当 DCM RxPduId 传进去，核心调用在 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:659)。

`PduR_LoTpCopyRxData` 逻辑一样，只是取 `UpTpCopyRxDataFctPtr`，实际调 `Dcm_CopyRxData`，见 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:741) 和 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:786)。

`PduR_LoTpRxIndication` 也是同一套路，实际调 `Dcm_TpRxIndication`，见 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:923) 和 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:968)。

**7. 数据和返回值怎么走**

数据方向：

`CanTp 接收/重组 -> PduR_CanTpCopyRxData -> Dcm_CopyRxData`

PduR 不碰 UDS 内容，只把 `PduInfoType *info` 原样转交给 DCM。DCM 在 `Dcm_CopyRxData` 里把数据拷进自己的 200 字节诊断 buffer，buffer 配置见 [Dcm_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/Dcm_Lcfg.c:297)。

返回值方向：

`Dcm 返回 BufReq_ReturnType -> PduR 原样返回 -> CanTp`

例如 `Dcm_StartOfReception` 返回 `BUFREQ_OK`，CanTp 才继续接收；`Dcm_CopyRxData` 返回 `BUFREQ_OK / BUSY / NOT_OK`，CanTp 据此继续、等待或中止。最后 `CanTp` 调 `PduR_CanTpRxIndication(id, E_OK/E_NOT_OK)`，PduR 再把结果通知 `Dcm_TpRxIndication(DcmRxPduId, result)`。

最适合下断点的一条线是：

`PduR_CanTpStartOfReception` -> `PduR_LoTpStartOfReception` -> `Dcm_StartOfReception` -> `PduR_CanTpCopyRxData` -> `Dcm_CopyRxData` -> `PduR_CanTpRxIndication` -> `Dcm_TpRxIndication`。

---
## 问题二 四张表具体作用

因为 PduR 是一个**通用路由器**，不是专门为 `CanTp -> Dcm` 写死的。它要同时支持：

`CanIf / CanTp / Com / Dcm`，`IF/TP`，`Rx/Tx`，`1:1/1:n`，`buffered/unbuffered`，gateway，routing path group，post-build 配置等。

所以 Vector 生成的 PduR 会把“路由”拆成几层表。你看到的这条：

```text
id=0 -> RmSrcRom[4] -> RmDestRom[8] -> RmGDestRom[11] -> MmRom[3] -> Dcm
```

可以理解成：

```text
外部 API ID -> 内部源 PDU -> 路由边 -> 全局目标 PDU -> 目标模块函数表
```

**1. id 是什么**

这里的 `id=0` 是 CanTp 调 PduR 时传进来的 `PduRRxSduId`。

在 [CanTp_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/CanTp_Lcfg.c:119)：

```c
PduRConf_PduRSrcPdu_PduRSrcPdu_dc13d0b4
```

它在 [PduR_Cfg.h](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Cfg.h:204) 里定义为：

```c
#define PduRConf_PduRSrcPdu_PduRSrcPdu_dc13d0b4 0u
```

这个 `0` 是 **PduR 的 TpRxSrc handle space** 里的 ID，不是 `RmSrcRom[0]`。

PduR 内部先通过这个宏换算：

```c
PduR_GetRmSrcRomIdxOfRxTp2Dest(id) = id + 4
```

见 [PduR_Lcfg.h](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.h:816)。

所以：

```text
CanTp 传 id=0
=> PduR 内部源路由索引 RmSrcRom[4]
```

**2. RmSrcRom：源 PDU 表**

`RmSrcRom` 表示“这条路由从哪里来”。

功能诊断请求对应 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:255)：

```c
{ /* 4 */ 1u, 9u, 1u, PDUR_NO_SRCHND... } /* PduRSrcPdu_dc13d0b4 */
```

含义是：

```text
RmSrcRom[4]
- 源模块 MmRomIdx = 1，也就是 CanTp
- 这个源 PDU 有 1 个目标
- 目标范围结束位置 RmDestRomEndIdx = 9
- 因为长度是 1，所以目标就是 RmDestRom[8]
```

所以 `RmSrcRom` 主要干这件事：**把一个源 PDU 映射到它的一组目的 PDU**。如果一个源要路由到多个目标，这里就会指向多个 `RmDestRom` 项。

**3. RmDestRom：路由边表**

`RmDestRom` 表示“从某个源到某个目标的一条边”。

功能诊断对应 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:184)：

```c
{ /* 8 */ 11u, 4u, PDUR_TP_UNBUFFERED_RX_API_FWD_ROUTINGTYPEOFRMDESTROM }
```

含义是：

```text
RmDestRom[8]
- 源是 RmSrcRom[4]
- 目标是 RmGDestRom[11]
- 路由类型是 TP_UNBUFFERED_RX_API_FWD
```

这个路由类型很关键：`TP_UNBUFFERED_RX_API_FWD` 表示 **TP 接收方向，不经过 PduR 内部 buffer，直接 API 转发给上层模块**。

也就是说 PduR 不存 UDS 数据，只把 CanTp 的调用转给 DCM：

```text
CanTp StartOfReception -> Dcm_StartOfReception
CanTp CopyRxData       -> Dcm_CopyRxData
CanTp RxIndication     -> Dcm_TpRxIndication
```

**4. RmGDestRom：全局目标 PDU 表**

`RmGDestRom` 表示“目标 PDU 到底是谁”。

功能诊断对应 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:226)：

```c
{ /* 11 */
  DcmConf_DcmDslProtocolRx_Meg_FunctionDiag...,
  PDUR_RX_DIRECTIONOFRMGDESTROM,
  3u,
  PDUR_IMMEDIATE_PDURDESTPDUPROCESSINGOFRMGDESTROM,
  8u
}
```

含义是：

```text
RmGDestRom[11]
- DestHnd = DCM 的功能诊断 RxPduId
- 方向 = Rx
- 目标模块 MmRomIdx = 3，也就是 Dcm
- 处理方式 = immediate
- 反指向 RmDestRom[8]
```

所以 `RmGDestRom` 解决两个问题：

```text
1. 目标模块是谁？这里是 DCM
2. 调 DCM 时传哪个 PduId？这里是 Dcm 的 functional RxPduId = 0
```

**5. MmRom：模块函数表**

`MmRom` 表示“模块管理表”，里面放的是每个模块的能力和函数指针。

DCM 对应 [PduR_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/PduR_Lcfg.c:151)：

```c
{ /* 3 */
  ...
  Dcm_CopyRxData,
  Dcm_CopyTxData,
  ...
  Dcm_StartOfReception,
  Dcm_TpRxIndication,
  Dcm_TpTxConfirmation,
  ...
} /* Dcm */
```

所以当 PduR 走到 `MmRom[3]`，它就知道：

```text
上层 TP StartOfReception 函数 = Dcm_StartOfReception
上层 TP CopyRxData 函数       = Dcm_CopyRxData
上层 TP RxIndication 函数     = Dcm_TpRxIndication
```

最终在 [PduR.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/PduR/PduR.c:659) 这种调用：

```c
PduR_GetUpTpStartOfReceptionFctPtrOfMmRom(
  PduR_GetMmRomIdxOfRmGDestRom(rmGDestRomIdx)
)(
  PduR_GetDestHndOfRmGDestRom(rmGDestRomIdx),
  info,
  TpSduLength,
  bufferSizePtr
);
```

对功能诊断展开后就是：

```c
Dcm_StartOfReception(0, info, TpSduLength, bufferSizePtr);
```

对物理诊断展开后就是：

```c
Dcm_StartOfReception(1, info, TpSduLength, bufferSizePtr);
```

**为什么不直接一张表**

因为 PduR 要支持复杂配置。拆成这些表后，同一套运行时代码可以处理很多情况：

```text
一个源 PDU -> 一个目标
一个源 PDU -> 多个目标
CanIf -> Com
CanTp -> Dcm
Dcm -> CanTp
Com -> CanIf
TP buffered gateway
TP unbuffered API forwarding
Rx/Tx 不同方向
不同模块的不同函数指针
不同模块自己的 PduId handle space
```

这份工程刚好很简单，所以看起来绕：

```text
CanTp id 0 -> DCM functional RxPduId 0
CanTp id 1 -> DCM physical RxPduId 1
```

但 PduR 生成器不会为这个简单场景手写特殊代码，它仍然用通用表模型。

一句话记忆：

```text
RmSrcRom   = 从哪里来
RmDestRom  = 走哪条路由边
RmGDestRom = 到哪个目标 PDU、传哪个目标 PduId
MmRom      = 目标模块是谁、调用哪个函数
```

所以这条链：

```text
id=0 -> RmSrcRom[4] -> RmDestRom[8] -> RmGDestRom[11] -> MmRom[3] -> Dcm
```

本质就是：

```text
CanTp 的功能诊断请求
-> 找到 PduR 内部源路由
-> 找到这条 TP Rx 转发路由
-> 找到 DCM 的功能诊断 RxPduId
-> 找到 DCM 的 TP 回调函数
-> 调 Dcm_StartOfReception / Dcm_CopyRxData / Dcm_TpRxIndication
```