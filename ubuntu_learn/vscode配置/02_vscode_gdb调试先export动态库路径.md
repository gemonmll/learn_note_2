在 VSCode 使用 gdb 启动程序时，如果你需要 **先 export 动态库路径（如 `LD_LIBRARY_PATH`）**，常见的做法有三种，推荐程度从高到低是：**在 launch.json 里设置环境变量** > **写 wrapper 脚本** > **修改系统环境变量**。

下面给你最实用、可直接复制的配置方式。

---

# ✅ **方法一：在 VSCode 的 launch.json 中设置环境变量（推荐）**

VSCode 的 gdb 调试使用的是 `launch.json`。
你可以在该启动配置中加上 `"env"` 字段，预先设置 `LD_LIBRARY_PATH`。

### 示例：给你的动态库目录 `/home/xxx/lib` 进行 export：

```jsonc
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Launch with gdb",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/your_program",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [
                {
                    "name": "LD_LIBRARY_PATH",
                    "value": "/home/xxx/lib:${env:LD_LIBRARY_PATH}"
                }
            ],
            "externalConsole": false,
            "MIMode": "gdb",
            "miDebuggerPath": "/usr/bin/gdb"
        }
    ]
}
```

### 说明：

* 这里的 `"environment"` 设置会在 **调试程序启动之前自动执行 environment export**。
* `:${env:LD_LIBRARY_PATH}` 表示在原有路径后追加，确保不会覆盖系统默认搜索路径。
* 这是最干净、最推荐的方式。

---

# ✅ **方法二：使用 wrapper shell 脚本（适合复杂环境）**

如果你调试的程序需要大量环境变量，或者你感觉在 launch.json 写太乱，可以写一个脚本：

创建脚本：`run_with_env.sh`：

```bash
#!/bin/bash
export LD_LIBRARY_PATH=/home/xxx/lib:$LD_LIBRARY_PATH
exec "$@"
```

给执行权限：

```bash
chmod +x run_with_env.sh
```

然后 VSCode 中这样写：

```jsonc
"program": "${workspaceFolder}/run_with_env.sh",
"args": ["${workspaceFolder}/your_program"]
```

---

# ✅ **方法三：在 debug 控制台启动前设置（不推荐）**

例如：

```bash
export LD_LIBRARY_PATH=/home/xxx/lib:$LD_LIBRARY_PATH
```

但 **VSCode 的 Debug 启动不会继承你终端里的环境变量**（除非你从终端启动 VSCode），
所以你如果从菜单或图标启动 VSCode，这种方式大部分情况下会失败。

---

# 🔍 最常见的问题

## ❗ VSCode 找不到动态库但我 export 了？

你大概率是：

* 在系统终端里 export 的（不会被 VSCode 继承）
* 在 launch.json 中写成了 `env` 而不是 `environment`
* 少写了 `"MIMode": "gdb"`

如果你需要，我可以帮你根据你的项目生成一个完整可用的 launch.json。

---

如果你愿意，把你当前的 **launch.json 内容发我**，我可以根据你的情况给你补全并优化配置。
