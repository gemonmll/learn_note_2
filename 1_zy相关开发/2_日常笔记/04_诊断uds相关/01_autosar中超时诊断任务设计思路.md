可以把它理解成 **DCM 的 10ms 调度只是“轮询诊断服务状态”**，而不是让 0x31 回调函数连续运行 10s。

假设：

```text
Dcm_MainFunction 周期 = 10ms
0x31 Routine 实际耗时 = 10s
```

正确设计应该是：

```text
Tester 发 31 01 RID
        ↓
DCM 调用 Xxx_Start(OpStatus = DCM_INITIAL)
        ↓
应用启动长任务，然后立刻返回 DCM_E_PENDING 或 DCM_E_FORCE_RCRRP
        ↓
DCM 后续每 10ms 左右再次调用 Xxx_Start(OpStatus = DCM_PENDING / DCM_FORCE_RCRRP_OK)
        ↓
应用每次只检查任务是否完成，不阻塞
        ↓
10s 后任务完成
        ↓
应用返回 E_OK
        ↓
DCM 回复 71 01 RID ...
```

AUTOSAR DCM 规范里，0x31 RoutineControl 由 `Xxx_Start()`、`Xxx_Stop()`、`Xxx_RequestResults()` 这类接口处理；如果使用 port，DCM 调 `RoutineServices_{RoutineName}` 的 C/S operation，如果不使用 port，则调用配置的 callout 函数。规范也定义了 RoutineServices 的可能返回值包括 `E_OK`、`E_NOT_OK`、`DCM_E_PENDING` 和 `DCM_E_FORCE_RCRRP`。([AUTOSAR][1])

---

## 1. 最核心的模型

你的 0x31 回调不要这样：

```c
Std_ReturnType Xxx_Start(...)
{
    DoSomethingFor10s();   /* 错误：阻塞 DCM task */
    return E_OK;
}
```

而是这样：

```c
Std_ReturnType Xxx_Start(Dcm_OpStatusType OpStatus, ...)
{
    if (OpStatus == DCM_INITIAL)
    {
        StartLongJob();    /* 只启动，不等待 */
        return DCM_E_FORCE_RCRRP;
    }

    if (JobFinished())
    {
        FillResponse();
        return E_OK;
    }

    return DCM_E_PENDING;
}
```

真正耗时的 10s 工作，由另一个周期函数、OS task、CDD、Fls/NvM job、应用状态机去推进。

---

## 2. DCM 10ms 调度时，实际会发生什么

举个例子，Routine 需要 10s，Dcm_MainFunction 10ms。

### t = 0ms：收到请求

Tester 发：

```text
31 01 F1 00
```

DCM 检查 session、security、RID、subfunction、长度等。检查通过后，调用你的 Routine Start 回调：

```c
Xxx_Start(OpStatus = DCM_INITIAL, ...)
```

你在这里只做三件事：

```c
保存输入参数
启动后台任务
返回 pending 类状态
```

例如：

```c
return DCM_E_FORCE_RCRRP;
```

这样 DCM 会尽快发：

```text
7F 31 78
```

AUTOSAR 规范说明：当 DCM 调用 operation 得到 `DCM_E_FORCE_RCRRP` 时，DSL 会触发 NRC `0x78` 的发送；这个机制用于应用需要立即请求 ResponsePending，而不是等到 P2/P2* 快超时时才发。([AUTOSAR][1])

---

### t = 10ms / 20ms / 30ms ...

DCM 后续继续调度。

如果前面返回的是 `DCM_E_FORCE_RCRRP`，DCM 通常会等 `7F 31 78` 发送确认之后，再调用：

```c
Xxx_Start(OpStatus = DCM_FORCE_RCRRP_OK, ...)
```

之后如果还没完成，你返回：

```c
return DCM_E_PENDING;
```

然后 DCM 后续周期继续调用：

```c
Xxx_Start(OpStatus = DCM_PENDING, ...)
```

每次调用时，你只是检查：

```c
任务完成了吗？
任务失败了吗？
还在运行吗？
```

不要在回调里真的执行 10s。

---

### t = 10s：任务完成

某一次 DCM 再调用：

```c
Xxx_Start(OpStatus = DCM_PENDING, ...)
```

你发现后台任务已经完成，于是填充响应数据：

```c
OutData[0] = ROUTINE_OK;
*OutLength = 1;
return E_OK;
```

DCM 发送最终正响应：

```text
71 01 F1 00 00
```

---

## 3. 完整时序图

假设 P2 = 50ms，P2* = 5s，Routine 总耗时 10s。

```text
t=0ms
Tester -> ECU : 31 01 F1 00

t=0~10ms
DCM -> App : Xxx_Start(DCM_INITIAL)
App -> DCM : DCM_E_FORCE_RCRRP

t≈10ms
ECU -> Tester : 7F 31 78

t≈20ms
DCM -> App : Xxx_Start(DCM_FORCE_RCRRP_OK)
App -> DCM : DCM_E_PENDING

t=30ms ~ 4990ms
DCM -> App : Xxx_Start(DCM_PENDING)
App -> DCM : DCM_E_PENDING

t≈5000ms
ECU -> Tester : 7F 31 78

t=5010ms ~ 9990ms
DCM -> App : Xxx_Start(DCM_PENDING)
App -> DCM : DCM_E_PENDING

t≈10000ms
DCM -> App : Xxx_Start(DCM_PENDING)
App -> DCM : E_OK

t≈10010ms
ECU -> Tester : 71 01 F1 00 ...
```

注意：**DCM 不是每 10ms 发一次 0x78**。
10ms 是 `Dcm_MainFunction` 的调度周期，`0x78` 是根据 P2 / P2* 计时或者 `DCM_E_FORCE_RCRRP` 触发的。

---

## 4. 推荐的接口分层设计

建议分成两层。

### 第一层：DCM Routine 回调接口

这是给 DCM 调的，必须很快返回。

```c
Std_ReturnType App_Routine_Start(
    Dcm_OpStatusType OpStatus,
    const uint8* InData,
    uint16 InLength,
    uint8* OutData,
    uint16* OutLength,
    Dcm_NegativeResponseCodeType* ErrorCode
);
```

实际 AUTOSAR 生成的函数签名会根据你的 `DcmDspRoutineFncSignature`、输入输出 signal、是否 variable length 等变化，不一定完全长这样，但核心一定有：

```c
Dcm_OpStatusType OpStatus
Dcm_NegativeResponseCodeType* ErrorCode
return Std_ReturnType
```

---

### 第二层：应用内部长任务接口

这是你自己设计的。

```c
typedef enum
{
    LONG_JOB_IDLE,
    LONG_JOB_RUNNING,
    LONG_JOB_DONE,
    LONG_JOB_FAILED,
    LONG_JOB_CANCELLED
} LongJob_StateType;

typedef struct
{
    LongJob_StateType state;
    uint16 rid;
    uint8  input[16];
    uint16 inputLen;
    uint8  result[16];
    uint16 resultLen;
    uint32 elapsedMs;
} LongJob_ContextType;
```

内部接口可以设计成：

```c
void LongJob_Start(const uint8* input, uint16 len);
void LongJob_MainFunction(void);
LongJob_StateType LongJob_GetState(void);
void LongJob_GetResult(uint8* out, uint16* outLen);
void LongJob_Cancel(void);
```

---

## 5. 0x31 回调示例

```c
static LongJob_ContextType g_LongJob;

Std_ReturnType App_Routine_Start(
    Dcm_OpStatusType OpStatus,
    const uint8* InData,
    uint16 InLength,
    uint8* OutData,
    uint16* OutLength,
    Dcm_NegativeResponseCodeType* ErrorCode
)
{
    Std_ReturnType ret = DCM_E_PENDING;

    switch (OpStatus)
    {
        case DCM_INITIAL:
        {
            if (g_LongJob.state == LONG_JOB_RUNNING)
            {
                *ErrorCode = DCM_E_CONDITIONSNOTCORRECT;
                ret = E_NOT_OK;
                break;
            }

            /*
             * 重要：
             * 第一次调用时保存输入参数。
             * 后续 OpStatus = DCM_PENDING 时，不要再依赖 InData 仍然有效。
             */
            LongJob_Start(InData, InLength);

            /*
             * 希望 DCM 立即发 7F 31 78
             */
            ret = DCM_E_FORCE_RCRRP;
            break;
        }

        case DCM_FORCE_RCRRP_OK:
        case DCM_PENDING:
        {
            switch (LongJob_GetState())
            {
                case LONG_JOB_RUNNING:
                    ret = DCM_E_PENDING;
                    break;

                case LONG_JOB_DONE:
                    LongJob_GetResult(OutData, OutLength);
                    ret = E_OK;
                    break;

                case LONG_JOB_FAILED:
                    *ErrorCode = DCM_E_GENERALPROGRAMMINGFAILURE;
                    ret = E_NOT_OK;
                    break;

                default:
                    *ErrorCode = DCM_E_CONDITIONSNOTCORRECT;
                    ret = E_NOT_OK;
                    break;
            }
            break;
        }

        case DCM_CANCEL:
        {
            LongJob_Cancel();
            ret = E_OK;
            break;
        }

        default:
        {
            *ErrorCode = DCM_E_GENERALREJECT;
            ret = E_NOT_OK;
            break;
        }
    }

    return ret;
}
```

---

## 6. 后台任务示例

这个函数可以由 OS task 每 10ms 或 20ms 调一次，也可以挂在某个 SWC runnable 里。

```c
void LongJob_MainFunction(void)
{
    if (g_LongJob.state != LONG_JOB_RUNNING)
    {
        return;
    }

    /*
     * 不要一次做完整 10s 工作。
     * 每次只推进一点点状态机。
     */
    switch (g_LongJob.step)
    {
        case 0:
            StartHardwareOrFlashOperation();
            g_LongJob.step = 1;
            break;

        case 1:
            if (HardwareOrFlashOperationFinished())
            {
                g_LongJob.step = 2;
            }
            break;

        case 2:
            PrepareResult(g_LongJob.result, &g_LongJob.resultLen);
            g_LongJob.state = LONG_JOB_DONE;
            break;

        default:
            g_LongJob.state = LONG_JOB_FAILED;
            break;
    }
}
```

如果你的底层是 Flash、NvM、Fee、Ea 这类异步模块，Routine 回调里只启动 job，后台靠对应模块的 `MainFunction` 推进，Routine 回调里只查状态。

---

## 7. `DCM_E_PENDING` 和 `DCM_E_FORCE_RCRRP` 怎么用

推荐规则：

```text
第一次调用，知道肯定会很久：
返回 DCM_E_FORCE_RCRRP

后续调用，任务还没完成：
返回 DCM_E_PENDING

任务完成：
返回 E_OK

任务失败：
设置 ErrorCode，返回 E_NOT_OK

请求被取消：
OpStatus = DCM_CANCEL，释放资源
```

也就是：

```c
DCM_INITIAL          -> 启动任务，返回 DCM_E_FORCE_RCRRP
DCM_FORCE_RCRRP_OK  -> 继续查状态
DCM_PENDING         -> 继续查状态
DCM_CANCEL          -> 取消任务
```

---

## 8. 为什么第一次建议返回 `DCM_E_FORCE_RCRRP`

因为 0x31 需要 10s，如果你第一次只返回：

```c
return DCM_E_PENDING;
```

DCM 也可以根据 P2 定时在接近 P2 超时时发送 `0x78`。规范里说，如果应用或 DSP 能处理请求但需要更多时间，DSL 应在达到响应时间边界时发送 NRC `0x78`，且 0x78 次数可由 `DcmDslDiagRespMaxNumRespPend` 限制。([AUTOSAR][1])

但如果你明确知道这个 Routine 很慢，更稳妥是第一次直接：

```c
return DCM_E_FORCE_RCRRP;
```

这样 tester 很快收到：

```text
7F 31 78
```

不会等到 P2 快超时。

---

## 9. 对你的 10s 场景，建议配置/设计

如果你希望一个 0x31 请求一直挂起直到 10s 后返回最终结果：

```text
31 01 RID
7F 31 78
7F 31 78
...
71 01 RID result
```

那么需要：

```text
Routine 回调支持 DCM_E_PENDING / DCM_E_FORCE_RCRRP
P2* 足够覆盖两次 0x78 之间的等待
DcmDslDiagRespMaxNumRespPend 次数足够
应用回调不阻塞
长任务有独立状态机或后台任务
```

如果 10s 只是偶尔，可以这样做。
如果是几十秒甚至几分钟，更推荐：

```text
31 01 RID        -> 立即返回 71 01 RID，表示任务已启动
31 03 RID        -> tester 周期查询结果
```

也就是不要让 DCM 一直 pending。

[1]: https://www.autosar.org/fileadmin/standards/R24-11/CP/AUTOSAR_CP_SWS_DiagnosticCommunicationManager.pdf "Specification of Diagnostic Communication Manager"
