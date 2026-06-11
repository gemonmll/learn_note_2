不是“UDS 报文通知 DCM 后，MainFunction 再索引到 DCM”，更准确地说：

**PduR/CanTp 的 Rx 回调先把请求放进 DCM 内部状态里，并置一个事件；`Dcm_MainFunction()` 是周期任务，下一次被 OS/RTE 调用时扫描这些事件，然后推进 DCM 内部状态机。**

**1. MainFunction 谁来调**

这份工程里 `Dcm_MainFunction()` 是在 BSW 5ms 周期任务里被 RTE 调的，位置在 [Rte_SystemApplication_OsCore0.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/Rte_SystemApplication_OsCore0.c:1561)，其中 5ms 分支里先跑 `CanTp_MainFunction()`，再跑 `Dcm_MainFunction()`，再跑 `Dem_MasterMainFunction()` / `Dem_SatelliteMainFunction()`，见 [Rte_SystemApplication_OsCore0.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/Rte_SystemApplication_OsCore0.c:1606)。

本工程 `DCM_SPLIT_TASKS_ENABLED == STD_OFF`，所以只有一个普通 `Dcm_MainFunction()`，不是 `Dcm_MainFunctionTimer/Worker` 分离模式，见 [Dcm_Cfg.h](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/Dcm_Cfg.h:232)。

**2. 报文收完时先发生什么**

完整 UDS 请求经 CanTp/PduR 到 DCM 后，会进：

`Dcm_TpRxIndication -> Dcm_NetTpRxIndication -> Dcm_NetTpRxIndicationOk`

关键位置在 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:36028) 和 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:16920)。

在 `Dcm_NetTpRxIndicationOk()` 里，它会检查这条请求是否可以被 DCM 接收，然后调用 `Dcm_DiagRxIndication(pTranspObj)`，见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:16184)。

`Dcm_DiagRxIndication()` 做两件关键事：

- 保存当前请求对应的 transport object：`Dcm_DiagSetQueuedTranspObj(...)`
- 置位 DCM 内部事件：`Dcm_TskSetEventByThread(DCM_TSK_ID_DIAG_RX, DCM_TSK_EV_DIAG_RX_NEW_REQ, ...)`

位置在 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:32471)。

所以报文收完时，DCM 并不是马上在回调里完整处理 UDS 服务，而是先“挂号”：**请求对象存起来，事件置起来，等 MainFunction 调度。**

**3. MainFunction 怎么调到内部任务**

`Dcm_MainFunction()` 本身很薄，只做初始化检查，然后调用：

```c
Dcm_TskScheduler(DCM_TSK_PRIO_NONE);
```

见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:36675)。

真正的调度在 `Dcm_TskScheduler()`，它会：

- 看 scheduler 是否被激活：`SchdIsActive`
- 遍历 `Dcm_TskTaskInfo[]`
- 对每个 task/thread 读取 `TaskEvents`
- 如果事件不为空，就调用 `Dcm_TskExecuteActiveTask(...)`

核心代码在 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:32198)。

而 `Dcm_TskSetEventByThread()` 会把事件 OR 到对应 task context 里，同时把 `SchdIsActive = TRUE`，见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:38500)。这就是 Rx 回调和 MainFunction 之间的桥。

**4. 哪个任务处理新请求**

内部任务表 `Dcm_TskTaskInfo[]` 定义了 DCM 有哪些“小任务”，见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:11326)。

跟一条 UDS 请求最相关的是：

- `Dcm_DiagTaskRx`
- `Dcm_DiagTaskWorker`
- `Dcm_DiagTaskTx`

当 `DCM_TSK_EV_DIAG_RX_NEW_REQ` 被置位后，`Dcm_DiagTaskRx()` 被 scheduler 调到。它会把 transport object 状态改成 ready，然后再置位：

```c
DCM_TSK_ID_DIAG_WORK, DCM_TSK_EV_DIAG_WORK_NEW_REQ
```

也就是把请求转交给真正的诊断 worker，见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:32876)。

**5. Worker 怎么索引到具体 UDS 服务**

`Dcm_DiagTaskWorker()` 收到 `DCM_TSK_EV_DIAG_WORK_NEW_REQ` 后，会调用：

```c
Dcm_DiagWorkerProcessNewRequest(...)
```

见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:32993)。

然后流程是：

1. `Dcm_DiagSwitchProcessingContext()`  
   把 queued transport object 切成当前正在处理的 object。

2. `Dcm_DiagInitiateServiceProcessing()`  
   初始化 `MsgContext`，把 DCM buffer 里的请求组织成 `reqData/reqDataLen/resData/resDataLen`，见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:19750)。

3. `Dcm_DiagValidateAndDispatchService()`  
   取第一个字节 SID，例如 `0x19`，先写入正响应 SID `0x59`，再查服务表，见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:19942)。

4. `Dcm_UtiLookUpUint8(Dcm_CfgDiagSvcIdLookUpTable, lSid)`  
   用 SID 查生成表，查到的 index 写入 `pContext->Thread->Diag.SidIndex`。

5. `Dcm_RepeaterSetCallee(pContext, SidIndex)`  
   把当前要执行的服务处理器 index 存到 repeater 里，见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:20030)。

6. 设置 `DCM_TSK_EV_DIAG_WORK_REPEAT`，让 worker 立刻进入服务处理器执行阶段。

服务表在 [Dcm_Lcfg.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Gen/GenData/Dcm_Lcfg.c:435)。比如：

- `SID 0x10 -> Dcm_Service10Processor`
- `SID 0x14 -> Dcm_Service14Processor`
- `SID 0x19 -> Dcm_Service19Processor`
- `SID 0x22 -> Dcm_Service22Processor`
- `SID 0x85 -> Dcm_Service85Processor`

**6. Repeater 真正调用服务处理器**

`Dcm_DiagWorkerDoRepeat()` 会调用：

```c
Dcm_RepeaterExecute(pContext, &lNrc);
```

见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:20164)。

`Dcm_RepeaterExecute()` 根据刚才保存的 `CalleeIdx` 取：

```c
pServiceInfo = Dcm_CfgDiagServiceInfo[pContext->Thread->Repeater.CalleeIdx];
pServiceInfo->ProcFunc(...)
```

也就是最终调到 `Dcm_Service19Processor()`、`Dcm_Service14Processor()` 这些服务函数，见 [Dcm.c](d:/workspace/autosar_learn/Proj_Code/_01_BSW/Src/Dcm/Dcm.c:31158)。

一句话串起来就是：

```text
Dcm_TpRxIndication
  -> Dcm_DiagRxIndication
  -> set DIAG_RX_NEW_REQ event
  -> Dcm_MainFunction
  -> Dcm_TskScheduler
  -> Dcm_DiagTaskRx
  -> set DIAG_WORK_NEW_REQ event
  -> Dcm_DiagTaskWorker
  -> Dcm_DiagValidateAndDispatchService
  -> SID lookup
  -> Dcm_RepeaterSetCallee(SidIndex)
  -> Dcm_RepeaterExecute
  -> Dcm_CfgDiagServiceInfo[SidIndex].ProcFunc()
```

所以你看 DCM 主流程时，最值得下断点的是：`Dcm_TpRxIndication`、`Dcm_DiagRxIndication`、`Dcm_TskSetEventByThread`、`Dcm_MainFunction`、`Dcm_TskScheduler`、`Dcm_DiagTaskRx`、`Dcm_DiagTaskWorker`、`Dcm_DiagValidateAndDispatchService`、`Dcm_RepeaterExecute`。