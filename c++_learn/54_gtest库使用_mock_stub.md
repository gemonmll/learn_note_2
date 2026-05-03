**Stub 和 Mock 都是假对象（test double），都是为了在单元测试里替代真实依赖。**

区别核心是：

> **Stub 主要提供“假数据 / 假返回值”；Mock 主要验证“有没有按预期调用”。**

---

## 1. Stub 是什么？

Stub 更像一个“假的被调用方”。

它不关心自己有没有被调用几次，也不重点检查调用顺序，它主要是为了让被测代码能继续跑下去。

例如你要测试：

```cpp
class DiagManager {
public:
    DiagManager(DemIf* dem) : dem_(dem) {}

    bool IsFaultActive(uint16_t eventId) {
        return dem_->GetEventStatus(eventId) == 1;
    }

private:
    DemIf* dem_;
};
```

真实 `DemIf` 可能会访问 DEM 模块、数据库、共享内存等。单元测试时可以写一个 stub：

```cpp
class StubDemIf : public DemIf {
public:
    uint8_t GetEventStatus(uint16_t eventId) override {
        return 1; // 固定返回故障激活
    }
};
```

测试：

```cpp
TEST(DiagManagerTest, FaultIsActive) {
    StubDemIf stubDem;
    DiagManager diag(&stubDem);

    EXPECT_TRUE(diag.IsFaultActive(0x1001));
}
```

这里 `StubDemIf` 的作用是：

> 给被测代码提供一个稳定、可控的返回值。

---

## 2. Mock 是什么？

Mock 更像一个“带检查能力的假对象”。

它不只是返回数据，还会检查：

```cpp
某个函数有没有被调用？
参数对不对？
调用了几次？
调用顺序对不对？
```

比如：

```cpp
class DiagManager {
public:
    DiagManager(DemIf* dem) : dem_(dem) {}

    void ReportFault(uint16_t eventId) {
        dem_->SetEventStatus(eventId, 1);
    }

private:
    DemIf* dem_;
};
```

你想验证 `ReportFault()` 是否真的调用了 DEM：

```cpp
TEST(DiagManagerTest, ReportFaultShouldCallDem) {
    MockDemIf mockDem;
    DiagManager diag(&mockDem);

    EXPECT_CALL(mockDem, SetEventStatus(0x1001, 1))
        .Times(1);

    diag.ReportFault(0x1001);
}
```

这里 `MockDemIf` 的作用是：

> 检查被测代码是否按照预期调用了依赖对象。

---

## 3. 一句话区分

| 类型   | 重点    | 例子                                           |
| ---- | ----- | -------------------------------------------- |
| Stub | 提供返回值 | `GetEventStatus()` 固定返回 `FAILED`             |
| Mock | 验证交互  | 检查 `SetEventStatus(0x1001, FAILED)` 是否调用 1 次 |

---

## 4. 放到 AUTOSAR 诊断场景里理解

假设你的诊断模块有两类逻辑：

### 场景一：测试“读取结果后怎么处理”

```cpp
uint8_t status = dem->GetEventStatus(eventId);

if (status == FAILED) {
    functionStatus = DEGRADED;
}
```

这里重点是让 `GetEventStatus()` 返回你想要的值。

这更适合用 **Stub**：

```cpp
class StubDemIf : public DemIf {
public:
    uint8_t GetEventStatus(uint16_t eventId) override {
        return FAILED;
    }
};
```

你关心的是最终结果：

```cpp
EXPECT_EQ(functionStatus, DEGRADED);
```

---

### 场景二：测试“是否正确上报 DEM”

```cpp
if (faultStatusChanged) {
    dem->SetEventStatus(eventId, FAILED);
}
```

这里重点是确认有没有调用 `SetEventStatus()`。

这更适合用 **Mock**：

```cpp
EXPECT_CALL(mockDem, SetEventStatus(eventId, FAILED))
    .Times(1);
```

你关心的是调用行为：

```cpp
SetEventStatus 是否被调用？
参数是不是 eventId 和 FAILED？
调用次数是不是 1 次？
```

---

## 5. gMock 里 Stub 和 Mock 的边界

在 gMock 里，一个 `Mock` 对象也可以当 Stub 用。

比如：

```cpp
EXPECT_CALL(mockDem, GetEventStatus(0x1001))
    .WillOnce(Return(FAILED));
```

这句有两层含义：

1. Stub 行为：调用时返回 `FAILED`
2. Mock 行为：顺便检查 `GetEventStatus(0x1001)` 被调用了

如果你只想提供默认返回值，不想严格验证调用，可以用：

```cpp
ON_CALL(mockDem, GetEventStatus(_))
    .WillByDefault(Return(FAILED));
```

这个更像 Stub。

---

## 6. `ON_CALL` 和 `EXPECT_CALL` 的区别

这个很关键。

### `ON_CALL`

主要用于设置默认行为：

```cpp
ON_CALL(mockDem, GetEventStatus(_))
    .WillByDefault(Return(FAILED));
```

含义：

> 如果有人调用 `GetEventStatus()`，默认返回 `FAILED`。

它不强制要求必须调用。

---

### `EXPECT_CALL`

主要用于设置期望调用：

```cpp
EXPECT_CALL(mockDem, SetEventStatus(0x1001, FAILED))
    .Times(1);
```

含义：

> 我要求这个函数必须被调用 1 次，而且参数必须匹配。

没调用会失败。
调用多了会失败。
参数不对会失败。

---

## 7. 一个完整对比

接口：

```cpp
class DemIf {
public:
    virtual ~DemIf() = default;

    virtual uint8_t GetEventStatus(uint16_t eventId) = 0;
    virtual void SetEventStatus(uint16_t eventId, uint8_t status) = 0;
};
```

业务代码：

```cpp
class Diagnosis {
public:
    Diagnosis(DemIf* dem) : dem_(dem) {}

    void CheckAndReport(uint16_t eventId) {
        uint8_t oldStatus = dem_->GetEventStatus(eventId);

        if (oldStatus == 0) {
            dem_->SetEventStatus(eventId, 1);
        }
    }

private:
    DemIf* dem_;
};
```

测试：

```cpp
using ::testing::_;
using ::testing::Return;

TEST(DiagnosisTest, ShouldReportWhenOldStatusPassed) {
    MockDemIf mockDem;
    Diagnosis diag(&mockDem);

    // Stub：准备输入条件
    ON_CALL(mockDem, GetEventStatus(_))
        .WillByDefault(Return(0));

    // Mock：验证输出行为
    EXPECT_CALL(mockDem, SetEventStatus(0x1001, 1))
        .Times(1);

    diag.CheckAndReport(0x1001);
}
```

这里非常典型：

```cpp
ON_CALL(...).WillByDefault(Return(...));
```

是 **Stub 风格**。

```cpp
EXPECT_CALL(...).Times(...);
```

是 **Mock 风格**。

---

## 8. 什么时候用 Stub？什么时候用 Mock？

### 用 Stub 的情况

你关心的是：

```cpp
给定某个输入，最终结果是什么？
```

比如：

```cpp
GetVehicleSpeed() 返回 80
GetGear() 返回 D
GetEventStatus() 返回 FAILED
GetDtcEnable() 返回 false
```

然后测试你的逻辑是否正确。

典型断言：

```cpp
EXPECT_EQ(result, expected);
EXPECT_TRUE(flag);
EXPECT_FALSE(error);
```

---

### 用 Mock 的情况

你关心的是：

```cpp
被测代码有没有正确调用外部模块？
```

比如：

```cpp
是否调用 DEM 上报故障？
是否调用 PduR 发送报文？
是否调用 NvM 写入数据？
是否调用 Log 接口记录错误？
是否调用 SOME/IP proxy 发送 event？
```

典型断言：

```cpp
EXPECT_CALL(mockDem, SetEventStatus(...)).Times(1);
EXPECT_CALL(mockPdur, Transmit(...)).Times(0);
EXPECT_CALL(mockNvm, WriteBlock(...)).Times(1);
```

---

## 9. 更工程化的理解

在嵌入式 / AUTOSAR / SOME/IP 单元测试里，经常是这样：

```cpp
输入依赖：用 Stub
输出依赖：用 Mock
```

例如：

```cpp
Diagnosis::MainFunction()
```

内部可能会：

1. 从传感器 / 信号缓存读取状态
2. 判断是否有故障
3. 更新内部 DTC mask
4. 调用 DEM 上报
5. 调用 SOME/IP 通知功能状态

测试时可以这样分：

| 依赖               | 更适合  |
| ---------------- | ---- |
| 读取车速             | Stub |
| 读取电源状态           | Stub |
| 读取故障使能条件         | Stub |
| 上报 DEM           | Mock |
| 发送诊断响应           | Mock |
| 发送 SOME/IP event | Mock |
| 写 NvM            | Mock |

---

## 10. 最容易混淆的点

很多人会说：

> Mock 也能返回值，那它不也是 Stub 吗？

是的，**Mock 可以承担 Stub 的职责**。

区别不是对象本身，而是你的测试意图：

```cpp
你只是想让它返回一个值
```

这就是 Stub 用法。

```cpp
你想验证它是否被调用
```

这就是 Mock 用法。

所以可以这样记：

> Stub 关注“状态结果”；Mock 关注“交互行为”。

---

## 11. 对你之前诊断代码的映射

你之前那种代码里有：

```cpp
diag_dem.set_event_status(event_id, status);
```

如果你要测试：

```cpp
fault_status 改变时，是否调用 set_event_status
```

就用 Mock：

```cpp
EXPECT_CALL(mockDem, set_event_status(event_id, FAILED))
    .Times(1);
```

如果你要测试：

```cpp
某个 DTC enable flag 是 true 时，逻辑怎么走
```

就用 Stub：

```cpp
ON_CALL(mockCfg, GetDtcEnable(_))
    .WillByDefault(Return(true));
```

如果你要测试：

```cpp
某个 fault 映射到哪个 DTC
```

也可以用 Stub：

```cpp
ON_CALL(mockCfg, GetDTCByIndex(_))
    .WillByDefault(Return(0xD47C8A));
```

---

## 12. 记忆口诀

可以这样记：

```text
Stub：我给你数据，你自己跑。
Mock：你必须按我要求调用我。
```

或者：

```text
Stub 是“输入条件”。
Mock 是“交互断言”。
```

再或者：

```text
Stub 关心返回什么。
Mock 关心调用了什么。
```
