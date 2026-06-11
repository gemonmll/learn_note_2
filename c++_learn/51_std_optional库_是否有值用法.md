好，我们系统地学习一下 **`optional` 怎么用**。
你这份代码实现的是 `senseAD::base::optional<T>`，用法和 `std::optional` 基本一致。

---

# 一、optional 是干什么的？

一句话：

> 表示“这个值可能存在，也可能不存在”。

---

# 二、最基础用法

---

## 1️⃣ 定义一个 optional

```cpp
#include "optional.h"

using senseAD::base::optional;

optional<int> a;      // 没有值
optional<int> b = 10; // 有值
```

---

## 2️⃣ 判断是否有值

```cpp
if (a.has_value()) {
    std::cout << "有值";
}

if (b) {  // 推荐写法
    std::cout << "b 有值";
}
```

推荐写法：

```cpp
if (opt) { }
```

因为内部实现了：

```cpp
explicit operator bool()
```

---

# 三、如何取值？

---

## 1️⃣ 使用 *

```cpp
optional<int> x = 5;

std::cout << *x;
```

---

## 2️⃣ 使用 value()

```cpp
std::cout << x.value();
```

⚠ 如果没有值会抛异常：

```cpp
bad_optional_access
```

---

## 3️⃣ 安全方式：value_or()

```cpp
optional<int> x;

int v = x.value_or(100);
std::cout << v;  // 输出 100
```

如果有值就返回原值，否则返回默认值。

---

# 四、实际例子（最常见场景）

---

## 场景1：查找函数

```cpp
optional<int> find(int id) {
    if (id == 1)
        return 42;
    return nullptr;   // 表示没有值
}
```

使用：

```cpp
auto result = find(1);

if (result) {
    std::cout << *result;
} else {
    std::cout << "没找到";
}
```

---

## 场景2：解析函数

```cpp
optional<int> parse_int(const std::string& s) {
    try {
        return std::stoi(s);
    } catch (...) {
        return nullptr;
    }
}
```

---

# 五、构造方式

---

## 1️⃣ 默认构造（空）

```cpp
optional<std::string> s;
```

---

## 2️⃣ 直接赋值构造

```cpp
optional<int> x = 10;
```

---

## 3️⃣ in_place 构造（推荐用于复杂对象）

```cpp
optional<std::string> s(senseAD::base::in_place, 5, 'a');
```

等价于：

```cpp
std::string(5, 'a);
```

构造出 `"aaaaa"`。

---

## 4️⃣ make_optional

```cpp
auto x = senseAD::base::make_optional(10);
```

自动推导类型。

---

# 六、修改值

---

## 1️⃣ 赋值

```cpp
optional<int> x;
x = 5;
```

---

## 2️⃣ emplace（推荐）

```cpp
optional<std::string> s;
s.emplace(3, 'b');  // "bbb"
```

emplace 会：

* 先销毁旧值
* 再原地构造

---

## 3️⃣ 清空值

```cpp
x.reset();
```

或者：

```cpp
x = senseAD::base::nullopt_t(0);
```

（不过一般直接 reset 更清晰）

---

# 七、结构体例子（真实项目常见）

```cpp
struct User {
    std::string name;
    int age;
};

optional<User> get_user(int id) {
    if (id == 1)
        return User{"Tom", 18};
    return nullptr;
}
```

使用：

```cpp
auto user = get_user(1);

if (user) {
    std::cout << user->name;
    std::cout << (*user).age;
}
```

注意：

```cpp
user->name
```

是可以用的，因为实现了：

```cpp
operator->()
```

---

# 八、常见错误

---

## ❌ 错误1：直接解引用空 optional

```cpp
optional<int> x;
std::cout << *x;  // ❌ 未定义行为
```

必须先判断：

```cpp
if (x) {
    std::cout << *x;
}
```

---

## ❌ 错误2：把 optional 当指针用

optional 不是指针，不要这样：

```cpp
optional<int>* p;
```

一般直接传值或 const 引用。

---

# 九、使用模式总结（项目级写法）

推荐写法：

```cpp
auto result = some_function();

if (!result) {
    return;
}

use(*result);
```

或者：

```cpp
int value = result.value_or(default_value);
```

---

# 十、什么时候应该用 optional？

用在：

* 函数可能失败
* 查找可能为空
* 结果不是必须存在
* 替代 nullptr / -1 / magic value

不要用在：

* 一定有值的对象
* 需要表达“错误原因”的情况（那应该用 expected）

---

# 十一、optional 心智模型

可以把它想成：

```cpp
struct {
    bool has_value;
    T value;
}
```

只是实现更复杂一点。

---

# 十二、进阶建议（你现在这个 optional 的注意点）

你这个实现：

* 没有比较运算符
* 没有完整 copy assignment
* 对 trivial 类型有优化
* value() 会抛异常

在工程里用是 OK 的，但要注意不要滥用。

---

如果你愿意，我可以下一步带你学习：

* optional 的底层生命周期图
* 为什么必须用 placement new
* optional 和 shared_ptr 的区别
* optional 和 expected 的区别
* 高级用法（monadic map/and_then）

你想往哪个方向深入？
