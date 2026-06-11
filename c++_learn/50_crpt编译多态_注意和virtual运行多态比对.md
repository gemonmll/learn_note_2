好，我用**一个能跑的最小例子**把 CRTP 从“为什么要这么写”到“它到底怎么调用到子类”讲透。

---

## 1) 先用一句话把 CRTP 说清楚

CRTP 就是：

> **父类是模板类 `Base<Derived>`，子类继承它并把自己当作模板参数传进去。父类里用 `static_cast<Derived*>(this)` 去调用子类函数，实现“编译期多态”。**

---

## 2) 最小可运行例子：事件分发（跟你代码非常像）

### ✅ Base：框架提供的通用 Handler（没有 virtual）

```cpp
#include <cstdint>
#include <iostream>

namespace FRAMEWORK {

template <typename Derived>
class CHandler {
public:
    // 框架对外统一入口：收到消息后调用这个
    void Dispatch(int32_t event, void* obj, uint64_t tick)
    {
        // 关键：把“我(父类部分)”强转成真正的派生类
        // 然后调用派生类实现的 OnRecv
        static_cast<Derived*>(this)->OnRecv(event, obj, tick);
    }
};

} // namespace FRAMEWORK
```

### ✅ Derived：业务侧实现（相当于你的 FusionResHandler）

```cpp
class FusionResHandler : public FRAMEWORK::CHandler<FusionResHandler>
{
public:
    void OnRecv(int32_t event, void* obj, uint64_t tick)
    {
        std::cout << "FusionResHandler::OnRecv, event=" << event
                  << ", tick=" << tick << "\n";
    }
};
```

### ✅ main：框架模拟收到事件

```cpp
int main()
{
    FusionResHandler h;

    int dummy = 123;
    h.Dispatch(1001, &dummy, 9999);  // 注意：调用的是父类的 Dispatch

    return 0;
}
```

运行输出会是：

```
FusionResHandler::OnRecv, event=1001, tick=9999
```

---

## 3) 关键点：为什么父类能调用到子类？

看这句：

```cpp
static_cast<Derived*>(this)->OnRecv(...)
```

这里的 `this` 在 `CHandler<Derived>` 里，类型是：

* `CHandler<FusionResHandler>*`

但实际上它指向的是一个完整对象：

* `FusionResHandler` 对象（里面包含了父类那一部分）

所以强转成 `FusionResHandler*` 是合法的（前提：你确实按 CRTP 方式继承了它）。

👉 编译器在编译 `CHandler<FusionResHandler>` 的时候，`Derived` 已经确定就是 `FusionResHandler`，所以这次调用在编译期就绑定到：

```cpp
FusionResHandler::OnRecv(...)
```

**没有 virtual，没有 vtable，没有运行时查表。**

---

## 4) 跟 virtual 的对比（同样功能）

### virtual 版本（运行期多态）

```cpp
class HandlerBase {
public:
    virtual void OnRecv(int32_t, void*, uint64_t) = 0;
    void Dispatch(int32_t e, void* o, uint64_t t) { OnRecv(e, o, t); }
    virtual ~HandlerBase() = default;
};

class FusionResHandler2 : public HandlerBase {
public:
    void OnRecv(int32_t e, void*, uint64_t t) override {
        std::cout << "virtual OnRecv\n";
    }
};
```

区别：

* `virtual`：`Dispatch -> vtable -> FusionResHandler2::OnRecv`
* `CRTP`：`Dispatch -> 直接编译期绑定 -> FusionResHandler::OnRecv`

在你这种**高频数据流/感知回调**场景，CRTP 常被用来减少开销并促进内联。

---

## 5) 再贴回你那段代码：对应关系

你的是：

```cpp
class FusionResHandler : public FRAMEWORK::CHandler<FusionResHandler>
{
public:
    void OnRecvRes(...) { TriggerEvent(...); }
    void OnRecvHmiRes(...) { TriggerEvent(...); }
    void OnRecvVisionObjectPerception(...) { TriggerEvent(...); }
};
```

那框架 `CHandler` 很可能内部有类似：

```cpp
static_cast<FusionResHandler*>(this)->OnRecvRes(...);
static_cast<FusionResHandler*>(this)->OnRecvHmiRes(...);
...
```

所以回调能“直接落到”你的 OnRecvXXX 上。

---

## 6) CRTP 使用时最常见的坑（你写代码时要注意）

1. **子类必须真的传自己**

   ```cpp
   class A : public CHandler<A> ✅
   class A : public CHandler<B> ❌（强转会出事）
   ```

2. **父类调用的函数必须在子类里存在**
   否则编译报错（这是好事，编译期就暴露问题）

3. **CRTP 不支持“运行时随便换子类”**
   如果你要 `CHandlerBase* p = new A/B;` 这种动态多态，那还是 virtual 更合适。

---

如果你愿意，我可以再给你一个**更贴近你工程的例子**：
`CHandler` 根据 `event` 分发到 `OnRecvRes/OnRecvHmiRes/...`（switch-case + CRTP），完全模拟你现在的 FusionResHandler 结构。这样你看一眼就能对上你项目里的代码。
