# -*- coding: utf-8 -*-

"""
DKIT 指令配置文件

维护内容：
1. IP_OPTIONS：目标 IP
2. CMD_GROUPS：指令集

说明：
- 命令中的 {ip} 会在运行时替换成用户选择的 IP
- 命令中的 {file} 会在运行时等待用户输入文件路径
"""

PROJ = "faw"

IP_OPTIONS = {
    "1": {
        "name": "172环境",
        "ip": "172.20.0.68",
    },
    "2": {
        "name": "192环境",
        "ip": "192.168.1.101",
    },
}


CMD_GROUPS = {
    "1": {
        "name": "切换板子 debug_online 模式",
        "steps": [
            {
                "desc": "导入证书",
                "cmd": (
                    "dkit cert install "
                    "--cnt -1 "
                    "--mode compatible "
                    "--start 2020-01-10 "
                    "--end 2024-12-30 "
                    "--proj {proj} "
                    "--ip {ip}"
                ),
            },
            {
                "desc": "切换 Debug_online 模式",
                "cmd": (
                    "dkit mode set "
                    "--system Debug_online "
                    "--proj {proj} "
                    "--ip {ip}"
                ),
            },
        ],
    },

    "2": {
        "name": "切换板子 normal 模式",
        "steps": [
            {
                "desc": "切换 normal 模式",
                "cmd": (
                    "dkit mode set "
                    "--system normal "
                    "--proj {proj} "
                    "--ip {ip}"
                ),
            },
        ],
    },

    "3": {
        "name": "更新 dkit 大包",
        "steps": [
            {
                "desc": "更新 dkit 大包",
                "need_file_input": True,
                "file_prompt": "请输入大包 tar.gz 文件路径: ",
                "cmd": (
                    "dkit pkg install "
                    "--ip {ip} "
                    "--proj {proj} "
                    "--file {file}"
                ),
            },
        ],
    },

    "4": {
        "name": "更新 dkit 小包",
        "steps": [
            {
                "desc": "更新 dkit 小包",
                "need_file_input": True,
                "multi_file_input": True,
                "file_prompt": (
                    "请输入小包 img 文件路径，每次输入一个；空回车结束。\n"
                    "也支持一次输入多个，多个文件用英文逗号连接。\n"
                    "请输入: "
                ),
                "cmd": (
                    "dkit firmware install "
                    "--force "
                    "--ip {ip} "
                    "--proj {proj} "
                    "--file {file}"
                ),
            },
        ],
    },

     "5": {
        "name": "查询大小包版本号",
        "steps": [
            {
                "desc": "查询大小包版本号",
                "cmd": (
                    "dkit ecu query "
                    "--ip {ip} "
                    "--proj {proj} "
                ),
            },
        ],
    },

    "6": {
        "name": "查询板子模式",
        "steps": [
            {
                "desc": "查询板子模式",
                "cmd": (
                    "dkit mode query "
                    "--ip {ip} "
                    "--proj {proj} "
                ),
            },
        ],
    },

    "7": {
        "name": "发送诊断指令",
        "steps": [
            {
                "desc": "发送诊断指令",
                "need_file_input": True,
                "file_prompt": (
                    "请输入诊断指令，不需要空格:\n"
                    "例如: 1003\n"
                    "请输入: "
                ),
                "cmd": (
                    "dkit diag send "
                    "--ip {ip} "
                    "--proj {proj} "
                    "--msg {file}"
                ),
            },
        ],
    },
}
