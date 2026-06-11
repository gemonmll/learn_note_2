# -*- coding: utf-8 -*-
"""
车型/IP 公共配置文件

只需要维护这里：
1. IP_OPTIONS：目标板 IP 选项
2. CAR_MODELS：车型对应的 car_model / sensor_id / sku
"""

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

# SSH 配置
SSH_USER = "root"
KEY_PATH = str(BASE_DIR / "id_rsa")
SSH_TIMEOUT_SEC = 20

# 可选 IP
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

# 车型配置
CAR_MODELS = {
    "1": {
        "name": "E001",
        "car_model": "30",
        "sensor_id": "11",
        "sku": "S86-HQ-0000E001-11-000",
    },
    "2": {
        "name": "E202",
        "car_model": "32",
        "sensor_id": "11",
        "sku": "S86-HQ-0000E202-11-000",
    },
    "3": {
        "name": "E007",
        "car_model": "33",
        "sensor_id": "11",
        "sku": "S86-HQ-0000E007-11-000",
    },
    "4": {
        "name": "E009",
        "car_model": "31",
        "sensor_id": "11",
        "sku": "S86-HQ-0000E009-11-000",
    },
    "5": {
        "name": "P301",
        "car_model": "50",
        "sensor_id": "18",
        "sku": "S86-HQ-0000P301-18-000",
    },
    "6": {
        "name": "P567-33 常用",
        "car_model": "33873",
        "sensor_id": "33",
        "sku": "S86-HQ-0000P567-33-000",
    },
    "7": {
        "name": "P567-20",
        "car_model": "33873",
        "sensor_id": "20",
        "sku": "S86-HQ-0000P567-20-000",
    },
    "8": {
        "name": "C801",
        "car_model": "33865",
        "sensor_id": "18",
        "sku": "S86-HQ-0000C801-18-000",
    },
    "9": {
        "name": "C831",
        "car_model": "33877",
        "sensor_id": "11",
        "sku": "S86-HQ-0000C831-11-000",
    },
    "10": {
        "name": "C206",
        "car_model": "33876",
        "sensor_id": "33",
        "sku": "S86-HQ-0000C206-33-000",
    },
    "11": {
        "name": "E001-10",
        "car_model": "33885",
        "sensor_id": "33",
        "sku": "S86-HQ-0E001_10-33-000",
    },
    "12": {
        "name": "E202-10",
        "car_model": "33886",
        "sensor_id": "11",
        "sku": "S86-HQ-0E202_10-11-000",
    },
    "13": {
        "name": "P301-10",
        "car_model": "33923",
        "sensor_id": "33",
        "sku": "S86-HQ-0P301_10-33-000",
    },
}

# SKU 设置工具路径
FACTORY_PARAM_CTL = "/mnt/dji/apps/middleware/bin/factory/test_factory_param_ctl"
