#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import shlex
import subprocess
import sys
import re
import stat
from datetime import datetime
from pathlib import Path

from carmodel_config import (
    SSH_USER,
    KEY_PATH,
    SSH_TIMEOUT_SEC,
    IP_OPTIONS as CARMODEL_IP_OPTIONS,
    CAR_MODELS,
    FACTORY_PARAM_CTL,
)
from dkit_cmd_config import PROJ, IP_OPTIONS as DKIT_IP_OPTIONS, CMD_GROUPS

try:
    import readline  # noqa: F401
except ImportError:
    pass

FILE_ARG_PLACEHOLDER = "__DKIT_FILE_ARG__"
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
MAIN_OPTIONS = {
    "1": {
        "name": "DKIT 指令",
        "mode": "dkit",
    },
    "2": {
        "name": "车型参数设置",
        "mode": "carmodel",
    },
}


def read_user_input(prompt):
    try:
        return input(prompt)
    except KeyboardInterrupt:
        print("\n用户取消执行。")
        sys.exit(130)


def choose_from_menu(title, options):
    while True:
        print("\n" + "=" * 80)
        print(title)
        print("=" * 80)

        for key, item in options.items():
            extra = ""
            if "ip" in item:
                extra = f" -> {item['ip']}"
            elif "car_model" in item:
                extra = (
                    f" -> car_model={item['car_model']}, "
                    f"sensor_id={item['sensor_id']}, "
                    f"sku={item['sku']}"
                )

            print(f"{key}. {item['name']}{extra}")

        choice = read_user_input("请输入序号: ").strip()

        if choice in options:
            return options[choice]

        print(f"输入无效: {choice}，请重新输入。")


def normalize_file_arg(file_value):
    """
    清理用户输入的文件参数，执行命令时不额外添加引号。
    支持：
    1. 单文件：xxx.tar.gz
    2. 多文件：a.img,b.img
    """
    file_value = ANSI_ESCAPE_RE.sub("", file_value)
    return file_value.strip().strip('"').strip("'")


def split_file_args(file_value):
    return [
        normalize_file_arg(item)
        for item in file_value.split(",")
        if normalize_file_arg(item)
    ]


def input_file_arg(step):
    if not step.get("multi_file_input"):
        while True:
            file_value = read_user_input("\n" + step.get("file_prompt", "请输入文件路径: ")).strip()

            if file_value:
                return normalize_file_arg(file_value)

            print("文件路径不能为空，请重新输入。")

    file_values = []
    prompt = "\n" + step.get("file_prompt", "请输入文件路径: ")

    while True:
        file_value = read_user_input(prompt).strip()

        if not file_value:
            if file_values:
                return ",".join(file_values)

            print("至少需要输入一个文件路径，请重新输入。")
            continue

        new_values = split_file_args(file_value)
        if not new_values:
            print("文件路径不能为空，请重新输入。")
            continue

        file_values.extend(new_values)
        print(f"已添加 {len(file_values)} 个文件。继续输入下一个文件，或直接回车结束。")
        prompt = "继续输入: "


def build_command_items(step, ip, file_value):
    cmd_template = step["cmd"]
    input_values = [file_value]

    if step.get("expand_multi_input") and file_value:
        input_values = split_file_args(file_value)

    items = []
    for input_value in input_values:
        cmd = cmd_template.format(
            ip=ip,
            proj=PROJ,
            file=input_value,
        )
        argv_cmd = cmd

        if step.get("need_file_input"):
            argv_cmd = cmd_template.format(
                ip=ip,
                proj=PROJ,
                file=FILE_ARG_PLACEHOLDER,
            )

        argv = [
            input_value if arg == FILE_ARG_PLACEHOLDER else arg
            for arg in shlex.split(argv_cmd)
        ]

        items.append({
            "desc": step["desc"],
            "cmd": cmd,
            "argv": argv,
        })

    return items


def build_commands(ip, cmd_group):
    commands = []

    for step in cmd_group["steps"]:
        file_value = ""

        if step.get("need_file_input"):
            file_value = input_file_arg(step)

        commands.extend(build_command_items(step, ip, file_value))

    return commands


def run_cmd_realtime(cmd, argv=None):
    """
    在 Ubuntu 下实时执行命令。

    关键点：
    - 不使用 capture_output=True
    - 不使用 stdout=subprocess.PIPE
    - 让 dkit 的 stdout/stderr 直接输出到当前终端
    - 直接调用 Ubuntu PATH 中的 dkit
    """

    print("\n" + "-" * 80)
    print(f"[{datetime.now()}]")
    print(f"执行指令: {cmd}")
    print("-" * 80)

    process = None

    try:
        if argv is None:
            argv = shlex.split(cmd)

        process = subprocess.Popen(
            argv,
            stdin=sys.stdin,
            stdout=sys.stdout,
            stderr=sys.stderr,
        )

        return_code = process.wait()

        print("\n" + "-" * 80)
        print(f"指令执行结束，返回码: {return_code}")
        print("-" * 80)

        return return_code == 0

    except KeyboardInterrupt:
        print("\n用户按下 Ctrl+C，正在终止当前命令...")
        if process is not None:
            try:
                process.terminate()
            except Exception:
                pass
        return False

    except FileNotFoundError as e:
        print(f"未找到命令: {e.filename}。请确认已在 Ubuntu 中安装 dkit，并且命令在 PATH 中。")
        return False

    except Exception as e:
        print("执行异常:", e)
        return False


def ensure_private_key_permissions(key_path):
    key_file = Path(key_path)
    if not key_file.exists():
        print(f"SSH 私钥不存在: {key_file}")
        return False

    mode = stat.S_IMODE(key_file.stat().st_mode)
    if mode & (stat.S_IRWXG | stat.S_IRWXO):
        key_file.chmod(0o600)
        print(f"已将 SSH 私钥权限修正为 600: {key_file}")

    return True


def build_carmodel_commands(car_config):
    car_model = car_config["car_model"]
    sensor_id = car_config["sensor_id"]
    sku = car_config["sku"]

    return [
        f"params_set rw.config.car_model {car_model}",
        f"params_set rw.config.sensor {sensor_id}",
        f"{FACTORY_PARAM_CTL} set rw.sys.sku {sku}",
        "sync",
    ]


def run_remote_command(host, cmd):
    if not ensure_private_key_permissions(KEY_PATH):
        return False

    ssh_cmd = [
        "ssh",
        "-i", KEY_PATH,
        "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=10",
        f"{SSH_USER}@{host}",
        cmd,
    ]

    print("\n" + "-" * 80)
    print(f"[{datetime.now()}]")
    print(f"目标主机: {host}")
    print(f"执行远程命令: {cmd}")
    print("-" * 80)

    try:
        result = subprocess.run(
            ssh_cmd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=SSH_TIMEOUT_SEC,
        )

        print("返回码:", result.returncode)

        if result.stdout:
            print("stdout:")
            print(result.stdout)

        if result.stderr:
            print("stderr:")
            print(result.stderr)

        return result.returncode == 0

    except subprocess.TimeoutExpired:
        print(f"执行超时，超过 {SSH_TIMEOUT_SEC} 秒未返回。")
        return False
    except FileNotFoundError:
        print("未找到 ssh 命令，请确认系统已安装 OpenSSH Client，并且 ssh 在 PATH 中。")
        return False
    except Exception as e:
        print("执行异常:", e)
        return False


def confirm_before_run(ip_config, cmd_group, commands):
    print("\n" + "=" * 80)
    print("请确认本次即将执行的配置")
    print("=" * 80)
    print(f"目标 IP : {ip_config['ip']}")
    print(f"指令集  : {cmd_group['name']}")

    print("\n即将顺序执行:")
    for index, item in enumerate(commands, start=1):
        print(f"{index}. [{item['desc']}]")
        print(f"   {item['cmd']}")

    answer = read_user_input("\n确认执行请输入 y，其它输入取消: ").strip().lower()
    return answer == "y"


def confirm_carmodel_before_run(host, car_config, commands):
    print("\n" + "=" * 80)
    print("请确认本次即将执行的配置")
    print("=" * 80)
    print(f"目标 IP     : {host}")
    print(f"车型        : {car_config['name']}")
    print(f"car_model  : {car_config['car_model']}")
    print(f"sensor_id  : {car_config['sensor_id']}")
    print(f"sku        : {car_config['sku']}")

    print("\n即将顺序执行:")
    for index, cmd in enumerate(commands, start=1):
        print(f"{index}. {cmd}")

    answer = read_user_input("\n确认执行请输入 y，其它输入取消: ").strip().lower()
    return answer == "y"


def run_dkit_flow():
    ip_config = choose_from_menu("请选择目标 IP", DKIT_IP_OPTIONS)

    while True:
        cmd_group = choose_from_menu("请选择要执行的指令集", CMD_GROUPS)

        ip = ip_config["ip"]
        commands = build_commands(ip, cmd_group)

        # if not confirm_before_run(ip_config, cmd_group, commands):
        #     print("\n用户取消执行。")
        #     sys.exit(0)

        for index, item in enumerate(commands, start=1):
            print(f"\n开始执行第 {index} 条指令: {item['desc']}")

            success = run_cmd_realtime(item["cmd"], item["argv"])

            if not success:
                print(f"\n第 {index} 条指令执行失败，停止后续执行。")
                print(f"失败指令: {item['cmd']}")
                sys.exit(1)

        print("\n全部指令执行完成。")


def run_carmodel_flow():
    ip_config = choose_from_menu("请选择目标 IP", CARMODEL_IP_OPTIONS)
    car_config = choose_from_menu("请选择车型", CAR_MODELS)

    host = ip_config["ip"]
    commands = build_carmodel_commands(car_config)

    # if not confirm_carmodel_before_run(host, car_config, commands):
    #     print("\n用户取消执行。")
    #     sys.exit(0)

    for index, cmd in enumerate(commands, start=1):
        print(f"\n开始执行第 {index} 条命令")
        success = run_remote_command(host, cmd)

        if not success:
            print(f"\n第 {index} 条命令执行失败，停止后续执行。")
            print(f"失败命令: {cmd}")
            sys.exit(1)

    print("\n全部命令执行完成。")


def main():
    main_option = choose_from_menu("请选择功能", MAIN_OPTIONS)

    if main_option["mode"] == "carmodel":
        run_carmodel_flow()
        return

    run_dkit_flow()


if __name__ == "__main__":
    main()
