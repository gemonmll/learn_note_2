你这个表里的公式，本质是在做 **DTC 显示码 ↔ ECU 3字节DTC码** 的转换。

以截图里的公式为例：

```excel
="0x" & BIN2HEX(
    HEX2BIN(
        IF(LEFT(A125,1)="P",0,
        IF(LEFT(A125,1)="C",1,
        IF(LEFT(A125,1)="B",2,
        IF(LEFT(A125,1)="U",3,"Error")))),2
    )
    & HEX2BIN(MID(A125,2,1),2)
) & RIGHT(A125,5)
```

## 1. DTC码转换规则

比如：

```text
B16A054
```

可以拆成：

```text
B    1    6A054
│    │      └── 后5位，原样保留
│    └──────── 第2位，占2bit，只能是 0~3
└───────────── DTC类型，占2bit
```

DTC类型映射如下：

| 显示字母 | 含义              | 二进制 |
| ---- | --------------- | --- |
| P    | Powertrain 动力总成 | 00  |
| C    | Chassis 底盘      | 01  |
| B    | Body 车身         | 10  |
| U    | Network 网络通信    | 11  |

第二位数字也转成2bit：

| 第二位 | 二进制 |
| --- | --- |
| 0   | 00  |
| 1   | 01  |
| 2   | 10  |
| 3   | 11  |

所以：

```text
B16A054
```

转换过程：

```text
B  -> 10
1  -> 01

10 + 01 = 1001 = 0x9

后5位 6A054 原样保留

所以：
B16A054 -> 0x96A054
```

再看你截图里的另一个：

```text
U1A50A8
```

```text
U -> 11
1 -> 01

11 + 01 = 1101 = 0xD

后5位 A50A8 原样保留

所以：
U1A50A8 -> 0xDA50A8
```

反过来也是一样：

```text
0x96A054
```

先看第一位 `9`：

```text
9 = 1001
```

拆成两部分：

```text
10 01
│  │
│  └── 1
└───── B
```

所以：

```text
0x96A054 -> B16A054
```

---

## 2. Python脚本：DTC显示码和ECU十六进制码互转

保存为：

```text
dtc_convert.py
```

代码如下：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import sys


GROUP_TO_BITS = {
    "P": 0b00,
    "C": 0b01,
    "B": 0b10,
    "U": 0b11,
}

BITS_TO_GROUP = {
    0b00: "P",
    0b01: "C",
    0b10: "B",
    0b11: "U",
}


def dtc_text_to_hex(dtc: str) -> str:
    """
    显示DTC -> ECU 3字节DTC码

    例：
        B16A054 -> 0x96A054
        U1A50A8 -> 0xDA50A8
    """
    s = dtc.strip().upper()

    if not re.fullmatch(r"[PCBU][0-3][0-9A-F]{5}", s):
        raise ValueError(
            f"非法DTC显示码: {dtc}，格式应类似 B16A054、U1A50A8，第二位只能是0~3"
        )

    group_char = s[0]
    second_digit = int(s[1], 16)
    rest = s[2:]

    group_bits = GROUP_TO_BITS[group_char]

    # 第一位十六进制 = DTC类型2bit + 第二位2bit
    first_nibble = (group_bits << 2) | second_digit

    return f"0x{first_nibble:X}{rest}"


def dtc_hex_to_text(value: str) -> str:
    """
    ECU 3字节DTC码 -> 显示DTC

    例：
        0x96A054 -> B16A054
        0xDA50A8 -> U1A50A8
    """
    s = value.strip().upper()

    if s.startswith("0X"):
        s = s[2:]

    s = s.replace(" ", "")

    if not re.fullmatch(r"[0-9A-F]{6}", s):
        raise ValueError(
            f"非法ECU DTC码: {value}，格式应类似 0x96A054 或 96A054"
        )

    first_nibble = int(s[0], 16)

    group_bits = first_nibble >> 2
    second_digit = first_nibble & 0b11

    group_char = BITS_TO_GROUP[group_bits]

    rest = s[1:]

    return f"{group_char}{second_digit:X}{rest}"


def auto_convert(value: str) -> str:
    """
    自动判断输入类型并转换
    """
    s = value.strip().upper()

    if re.match(r"^[PCBU]", s):
        return dtc_text_to_hex(s)
    else:
        return dtc_hex_to_text(s)


def main():
    if len(sys.argv) < 2:
        print("DTC显示码和ECU十六进制码互转工具")
        print()
        print("示例：")
        print("  python dtc_convert.py B16A054")
        print("  python dtc_convert.py 0x96A054")
        print()
        print("进入交互模式，输入 q 退出")
        print()

        while True:
            value = input("请输入DTC: ").strip()

            if value.lower() in ("q", "quit", "exit"):
                break

            if not value:
                continue

            try:
                result = auto_convert(value)
                print(f"{value} -> {result}")
            except Exception as e:
                print(f"错误: {e}")

        return

    for value in sys.argv[1:]:
        try:
            result = auto_convert(value)
            print(f"{value} -> {result}")
        except Exception as e:
            print(f"{value} -> 错误: {e}")


if __name__ == "__main__":
    main()
```

---

## 3. 使用示例

### 显示DTC转ECU码

```powershell
python .\dtc_convert.py B16A054
```

输出：

```text
B16A054 -> 0x96A054
```

再比如：

```powershell
python .\dtc_convert.py U1A50A8
```

输出：

```text
U1A50A8 -> 0xDA50A8
```

---

### ECU码转显示DTC

```powershell
python .\dtc_convert.py 0x96A054
```

输出：

```text
0x96A054 -> B16A054
```

```powershell
python .\dtc_convert.py 0xDA50A8
```

输出：

```text
0xDA50A8 -> U1A50A8
```

---

## 4. 对应关系速查表

第一位十六进制和DTC前两位的关系是：

| ECU首位 | 显示前两位 |
| ----- | ----- |
| 0     | P0    |
| 1     | P1    |
| 2     | P2    |
| 3     | P3    |
| 4     | C0    |
| 5     | C1    |
| 6     | C2    |
| 7     | C3    |
| 8     | B0    |
| 9     | B1    |
| A     | B2    |
| B     | B3    |
| C     | U0    |
| D     | U1    |
| E     | U2    |
| F     | U3    |

所以你看到：

```text
B16A054 -> 0x96A054
U1A50A8 -> 0xDA50A8
```

核心就是：

```text
B1 -> 9
U1 -> D
```

后面的5位直接保留。
