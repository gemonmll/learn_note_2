# 工厂参数完整架构、调用链路与读写路径分析

---

## 一、总体架构

### 1.1 核心设计原则：双端独立存储，无主从同步

工厂参数在 SA8650 芯片的 MCU 侧和 SOC 侧**各自独立存储**，不存在"谁同步谁"的主从关系。

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         SA8650 芯片                                       │
│                                                                          │
│  ┌──────────────────────────────┐  ┌──────────────────────────────────┐  │
│  │ MCU 侧 (R52, RTOS)           │  │ SOC 侧 (HPC, QNX/Linux)          │  │
│  │                              │  │                                  │  │
│  │ 存储: PM (Parameter Manager) │  │ 存储: NvM (NVRAM Manager)        │  │
│  │   └─ NvM (AUTOSAR)           │  │   └─ Flash (Fls_Sync)            │  │
│  │                              │  │                                  │  │
│  │ 诊断入口: UDS CAN (DoCAN)    │  │ 诊断入口: UDS DoIP (以太网)       │  │
│  │   CAN总线 → CanTp → PduR →   │  │   以太网 → DoIP → PduR →         │  │
│  │   Dcm → DID Handler          │  │   Dcm → DID Handler              │  │
│  │                              │  │                                  │  │
│  │ API: read_param_by_id()      │  │ API: memcpy(g_xxx) /             │  │
│  │      write_param_by_id()     │  │      NvM_WriteBlock() +          │  │
│  │                              │  │      Fls_Sync()                  │  │
│  └──────────────┬───────────────┘  └──────────────┬───────────────────┘  │
│                 │                                  │                      │
│                 └──── DCMS IPC (跨核通信) ──────────┘                      │
│                         pub/sub + client/server                            │
└──────────────────────────────────────────────────────────────────────────┘
```

**关键结论**：通过 CAN 写入 MCU 的数据不会自动同步到 SOC，通过 DoIP 写入 SOC 的数据也不会自动同步到 MCU。两侧在车辆出厂时由产线诊断仪分别编程。

### 1.2 五条读写路径总览

```
路径① UDS CAN (DoCAN)
  外部诊断仪 ──CAN──→ MCU Dcm ──→ DID Handler ──→ read_param_by_id / write_param_by_id ──→ PM → NvM

路径② UDS DoIP (以太网)
  外部诊断仪 ──ETH──→ SOC DoIP ──→ Dcm ──→ DID Handler ──→ memcpy(g_xxx) / NvM_WriteBlock + Fls_Sync

路径③ DCMS 跨域请求 (其他模块通过 topic 读写 MCU 侧 PM)
  SOC模块 ──DCMS──→ MCU DCMS Server ──→ read_param_block_by_id / write_param_by_id ──→ PM

路径④ ECU Shell (跨核调试命令)
  终端 dshel -d dsar ──UDP──→ MCU Shell ──DCMS topic──→ SOC cross_factory_param_set_callback ──→ NvM

路径⑤ 系统属性同步 + DSSAD 上报 (SOC 内部定时任务)
  FactoryParam_SyncThread: NvM → rw.oem.sn / rw.oem.vin (系统属性)
  FactoryParam_SendtoDssad: 60s周期 g_xxx → DSSAD (via DCMS topic)
```

---

## 二、数据模型

### 2.1 14个工厂参数字段定义

**文件**: `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/FactoryParam/FactoryParam_Type.h`

```c
typedef struct
{
    uint8_t  arr_part_num[16];           // F187  物料号/零件号 (只读)
    uint8_t  arr_materias_num[16];       // F189  材料号 (只读)
    uint8_t  arr_cecu_sn[34];            // F18C  ECU序列号 (只读)
    uint8_t  arr_vin[17];                // F190  VIN码 (可读可写)
    uint8_t  arr_hd_ver[16];             // F191/F193  硬件版本号 (只读)
    uint8_t  arr_repair_shopcode[10];    // F198  维修店代码 (可读可写)
    uint8_t  arr_programming_date[4];    // F199  编程日期 (可读可写)
    uint8_t  arr_installation_date[4];   // F19D  安装日期 (可读可写)
    uint8_t  arr_systemconfigure1[16];   // F1A1  系统配置字1 (可读可写)
    uint8_t  arr_systemconfigure2[16];   // F1A2  系统配置字2 (可读可写)
    uint8_t  arr_systemconfigure3[16];   // F1A3  系统配置字3 (可读可写)
    uint8_t  arr_fawreserved[16];        // F1A7  预留字段 (可读可写)
    uint8_t  arr_sw_ver[16];             // F1A0/F195  软件版本号 (只读)
    uint8_t  arr_conf_form[1];           // F1A9  配置形式 (可读可写)
} pm_factory_param_block_t;
```

### 2.2 DID 与字段映射表

| DID | 字段名 | 大小 | MCU读写 | SOC读写 | 备注 |
|-----|--------|------|---------|---------|------|
| F187 | arr_part_num | 16B | 只读 | 只读 | SOC侧从ROM常量读取(带车型适配) |
| F189 | arr_materias_num | 16B | 只读 | 只读(跨域) | SOC侧实时查询MCU |
| F18C | arr_cecu_sn | 34B | 只读 | 只读 | |
| F190 | arr_vin | 17B | 可读可写 | 可读可写 | SOC写后触发系统属性同步 |
| F191 | arr_hd_ver | 16B | 只读 | 只读 | |
| F193 | arr_hd_ver | 16B | 只读 | 只读 | 供应商硬件版本号 |
| F195 | arr_sw_ver | 16B | 只读 | 只读(跨域) | SOC侧实时查询MCU |
| F198 | arr_repair_shopcode | 10B | 可读可写 | 可读可写 | |
| F199 | arr_programming_date | 4B | 可读可写 | 可读可写 | |
| F19D | arr_installation_date | 4B | 可读可写 | 可读可写 | |
| F1A0 | arr_sw_ver | 16B | 只读 | 只读(跨域) | SOC侧实时查询MCU |
| F1A1 | arr_systemconfigure1 | 16B | 可读可写 | 可读可写 | SOC写后发送ADAS bit到DMM/FL |
| F1A2 | arr_systemconfigure2 | 16B | 可读可写 | 可读可写 | |
| F1A3 | arr_systemconfigure3 | 16B | 可读可写 | 可读可写 | |
| F1A7 | arr_fawreserved | 16B | 可读可写 | 可读可写 | |
| F1A9 | arr_conf_form | 1B | 可读可写 | 可读可写 | SOC侧校验仅允许0x00/0x01 |

### 2.3 MCU 侧 PM 参数注册

**文件**: `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/FactoryParam/FactoryParam_Cfg.c`

MCU 侧通过 `DEC_ARR` 宏将每个字段注册为 PM 参数项，所有参数统一使用 `PM_OPT_RWSR | AUTH_ASSITANT` 权限（需要安全访问授权）。14个参数注册在 `pm_factory_param_table[]` 数组中，通过 `DECLEAR_PARAM_BLOCK_EXT(pm_factory, CFG_FACTORY, ...)` 声明为一个参数块。

MCU 侧为每个参数定义了静态存储数组（如 `Arr_Part_Num[16]`, `Arr_Vin[17]` 等），这些是 PM 框架内部使用的初始化值和缓冲区。

### 2.4 SOC 侧 NvM 全局变量（RAM 镜像）

**文件**: `dsar-hq/src/dsar_app/product/faw/oem_feature/FactoryParam/FactoryParam_Cfg.c`

```c
/* ref in nvm — 这些全局变量就是 NvM 配置中的 RAM Block Data Address */
uint8_t g_part_number[16];
uint8_t g_ecu_serial_number[34];
uint8_t g_vin[17];
uint8_t g_ecu_hd_version[16];
uint8_t g_repair_shop_code[10];
uint8_t g_program_date[4];
uint8_t g_ecu_install_date[4];
uint8_t g_sys_configure_1[16];
uint8_t g_sys_configure_2[16];
uint8_t g_sys_configure_3[16];
uint8_t g_faw_reserved_f1a7[16];
uint8_t g_configure_form[1];
// 还有 g_boot_sw_version[12], g_ecu_sw_version[16](不使用),
// g_ecu_sw_version_reserve_f1a0[16](不使用), g_supplier_hardware_versnum[16],
// g_supplier_soft_versnum[16]

/* 软件版本号 — 非NvM变量，运行时动态查询MCU获取 */
uint8_t Software_Version[16] = "3629801-XX03XXXX";

/* ROM常量 — 硬件+零件版本信息，作为F187的默认数据源 */
const uint8_t HardAndPart_Version_Info[] = {'3','6','2','9','1','0','0','-','H','X','0','3','0','0','0','0'};
```

---

## 三、NvM RAM 镜像机制 —— 为什么 `memcpy(Data, g_vin, 17)` 就是读存储

### 3.1 核心原理

SOC 侧的 `g_vin`、`g_part_number` 等全局变量**直接配置为 NvM Block 的 RAM 镜像地址**。这是 AUTOSAR NvM 的标准设计模式。

**证据**来自 `NvM_Cfg.c`:

```c
// NvM block 配置中，RamBlockDataAddress 直接指向 g_xxx:
{ .RamBlockDataAddress = (NvM_RamAddressType)&g_vin,             ... }  // line 480
{ .RamBlockDataAddress = (NvM_RamAddressType)&g_part_number,      ... }  // line 366
{ .RamBlockDataAddress = (NvM_RamAddressType)&g_ecu_serial_number,... }  // line 442
{ .RamBlockDataAddress = (NvM_RamAddressType)&g_sys_configure_1,  ... }  // line 784
// ... 其余同理
```

### 3.2 数据生命周期

```
上电启动:
  NvM_ReadAll() ──→ 从 Flash 读取所有 block 到对应的 g_xxx[] RAM 镜像
  (所有 block 配置了 NVM_SELECT_BLOCK_FOR_READALL_ON)

运行时读:
  memcpy(Data, g_vin, 17) — 直接读 RAM 镜像，等同于读存储
  原因: NvM 框架保证 g_xxx 与 Flash 内容一致

运行时写:
  NvM_WriteBlock(nvm_blockid, &Data[0]) ──→ 异步写入 Flash
  Fls_Sync() ──→ 等待 Flash 写入完成
  memcpy(g_xxx, Data, size) ──→ 同步更新 RAM 镜像

下电关机:
  NvM_WriteAll() ──→ 将所有 RAM 镜像写回 Flash
  (所有 block 配置了 NVM_SELECT_BLOCK_FOR_WRITEALL_ON)
```

**结论**: `memcpy(Data, g_vin, 17)` 读的是 NvM 框架维护的 RAM 镜像，该镜像在上电时从 Flash 加载、写操作时同步更新。不需要在每次读 DID 时调 `NvM_ReadBlock`，因为 RAM 镜像就是"当前值"的权威副本。

---

## 四、路径① UDS CAN (DoCAN) —— MCU 侧

### 4.1 调用链总览

```
外部诊断仪
  │
  ▼ CAN 总线
CanIf → CanTp → PduR → Dcm
  │
  ▼ Dcm 根据 DID 查函数指针表 (dji_rte.c)
  │
  ├─ 22 F187 (读零件号)
  │   └─ FactoryParam_FAW_Part_Number_16Bytes_F187_ReadData()
  │       └─ read_param_by_id(CFG_FACTORY, &temp, pm_factory_param_block_t, arr_part_num)
  │           └─ PM 框架从 NvM 读取 pm_factory 块 → 返回 arr_part_num 字段
  │
  ├─ 22 F190 (读VIN)
  │   └─ FactoryParam_FAW_Vehicle_Identification_Number_17Bytes_F190_ReadData()
  │       └─ read_param_by_id(CFG_FACTORY, &temp, pm_factory_param_block_t, arr_vin)
  │
  ├─ 2E F190 (写VIN)
  │   └─ FactoryParam_FAW_Vehicle_Identification_Number_17Bytes_F190_WriteData()
  │       └─ write_param_by_id(CFG_FACTORY, &temp, pm_factory_param_block_t, arr_vin)
  │           └─ PM 框架将 arr_vin 写入 NvM
  │
  └─ ... 其余 DID 同理
```

### 4.2 关键特征

- **全部本地操作**：所有 MCU 侧 DID handler **不涉及任何跨域 DCMS 调用**
- **通过 PM 框架读写**：`read_param_by_id` / `write_param_by_id` 是统一的参数管理接口，内部封装了 NvM 操作
- **F187 无 Write  handler**：零件号只读，`dji_rte.c` 中 Write 函数指针组没有 F187 条目
- **F1A1 Write 无特殊处理**：MCU 侧仅写入 PM，不解析 ADAS bit、不发送 DCMS topic（与 SOC 侧不同）

### 4.3 MCU 侧 DID 路由表

**文件**: `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/DcmApp/dji_rte.c`

```c
// 读函数指针绑定 (完整列表)
.DID_0xF187_ReadData_Svc = FactoryParam_FAW_Part_Number_16Bytes_F187_ReadData,
.DID_0xF189_ReadData_Svc = FactoryParam_FAW_ECU_Materials_Number_16Bytes_F189_ReadData,
.DID_0xF18C_ReadData_Svc = FactoryParam_ECU_Serial_Number_34Bytes_F18C_ReadData,
.DID_0xF190_ReadData_Svc = FactoryParam_FAW_Vehicle_Identification_Number_17Bytes_F190_ReadData,
.DID_0xF191_ReadData_Svc = FactoryParam_FAW_ECU_Hardware_Version_Number_16Bytes_F191_ReadData,
.DID_0xF193_ReadData_Svc = FactoryParam_Supplier_ECU_Hardware_Version_Number_16Bytes_F193_ReadData,
.DID_0xF195_ReadData_Svc = FactoryParam_Supplier_ECU_Software_Version_Number_16Bytes_F195_ReadData,
.DID_0xF198_ReadData_Svc = FactoryParam_Repair_Shop_Code_and_Tester_Serial_Number_10Bytes_F198_ReadData,
.DID_0xF199_ReadData_Svc = FactoryParam_Programming_Date_4Bytes_F199_ReadData,
.DID_0xF19D_ReadData_Svc = FactoryParam_ECU_Installation_Date_4Bytes_F19D_ReadData,
.DID_0xF1A0_ReadData_Svc = FactoryParam_FAW_ECUSoftware_Version_Number_16Bytes_F1A0_ReadData,
.DID_0xF1A1_ReadData_Svc = FactoryParam_System_Configuration_1_16Bytes_F1A1_ReadData,
.DID_0xF1A2_ReadData_Svc = FactoryParam_System_Configuration_2_16Bytes_F1A2_ReadData,
.DID_0xF1A3_ReadData_Svc = FactoryParam_System_Configuration_3_16Bytes_F1A3_ReadData,
.DID_0xF1A7_ReadData_Svc = FactoryParam_FAW_Reserved_16Bytes_F1A7_ReadData,
.DID_0xF1A9_ReadData_Svc = FactoryParam_ConfigureForm_1Bytes_F1A9_ReadData,

// 写函数指针绑定 (仅可写DID)
.DID_0xF190_WriteData_Svc = FactoryParam_FAW_Vehicle_Identification_Number_17Bytes_F190_WriteData,
.DID_0xF198_WriteData_Svc = FactoryParam_Repair_Shop_Code_and_Tester_Serial_Number_10Bytes_F198_WriteData,
.DID_0xF199_WriteData_Svc = FactoryParam_Programming_Date_4Bytes_F199_WriteData,
.DID_0xF19D_WriteData_Svc = FactoryParam_ECU_Installation_Date_4Bytes_F19D_WriteData,
.DID_0xF1A1_WriteData_Svc = FactoryParam_System_Configuration_1_16Bytes_F1A1_WriteData,
.DID_0xF1A2_WriteData_Svc = FactoryParam_System_Configuration_2_16Bytes_F1A2_WriteData,
.DID_0xF1A3_WriteData_Svc = FactoryParam_System_Configuration_3_16Bytes_F1A3_WriteData,
.DID_0xF1A7_WriteData_Svc = FactoryParam_FAW_Reserved_16Bytes_F1A7_WriteData,
.DID_0xF1A9_WriteData_Svc = FactoryParam_ConfigureForm_1Bytes_F1A9_WriteData,
```

---

## 五、路径② UDS DoIP —— SOC 侧

### 5.1 调用链总览

```
外部诊断仪
  │
  ▼ 以太网 (DoIP 端口 13400)
SoAd → DoIP → PduR → Dcm
  │
  ▼ Dcm 根据 DID 查函数指针表 (dji_rte.c)
  │
  ├─ 22 F187 (读零件号) — 特殊: 从ROM常量读取
  │   └─ DataServices_0xF187_FAW_Part_Number_16Bytes_ReadData()
  │       └─ memcpy(Data, HardAndPart_Version_Info, sizeof(g_part_number))
  │       └─ 根据 Diag_GetCarModelValue() 覆写 Data[8..11] (车型适配)
  │       注意: 这里没有读 g_part_number (NvM变量)!
  │
  ├─ 22 F190 (读VIN)
  │   └─ DataServices_0xF190_FAW_Vehicle_Identification_Number_17Bytes_ReadData()
  │       └─ memcpy(Data, g_vin, sizeof(g_vin))   ← 读 NvM RAM 镜像
  │
  ├─ 2E F190 (写VIN)
  │   └─ DataServices_0xF190_FAW_Vehicle_Identification_Number_17Bytes_WriteData()
  │       └─ FactoryParam_Write_ProductParam(0xF190, NvM_VIN, g_vin, ...)
  │           ├─ DCM_INITIAL:  NvM_WriteBlock(nvm_blockid, &Data[0]) → DCM_E_PENDING
  │           ├─ DCM_PENDING:  NvM_GetErrorStatus() + Fls_Sync()
  │           │    成功 → memcpy(g_vin, Data, sizeof(g_vin)) → DCM_E_OK
  │           │    失败 → 重试 (最多1200次) → DCM_E_PENDING
  │           └─ 超时 → E_NOT_OK
  │       └─ 成功后 FactoryParam_OemVinSyncFlagSet()  // 设置系统属性同步标志
  │
  ├─ 22 F189/F195/F1A0 (读软件版本号) — 唯一跨域读DID
  │   └─ FactoryParam_Read_SoftVersion(did, OpStatus, Data)
  │       ├─ DCM_INITIAL:  返回 DCM_E_PENDING
  │       ├─ DCM_PENDING:  每秒调用 SoftWare_Version_Get()
  │       │   └─ dcms_mcu_topic_send(DCMS_TOPIC_GET_SOFTWARE_VERSION) → MCU
  │       │       MCU 返回 → Read_SoftWare_Version_Callback()
  │       │       → Software_Version[12..15] = resp_msg->version[0..3]
  │       │       → Received_Sw_Version = true
  │       └─ 收到后: memcpy(Data, Software_Version, 16) → DCM_E_OK
  │
  ├─ 2E F1A1 (写系统配置字1) — 特殊: 解析ADAS bit并发送DCMS topic
  │   └─ DataServices_0xF1A1_System_Configuration_1_16Bytes_WriteData()
  │       ├─ DCM_INITIAL:
  │       │   解析 Data[0..6] 中 30+ 个 ADAS 功能bit (AEB, FCW, LKA, ...)
  │       │   → driving_param_config / fl_param_config
  │       │   → dcms_mcu_topic_send(DCMS_TOPIC_DMM_PARAM_CONFIG, ...)
  │       │   → dcms_mcu_topic_send(DCMS_TOPIC_DMM_PARAM_CONFIG_FL, ...)
  │       └─ FactoryParam_Write_ProductParam(...) → NvM 写入
  │
  └─ ... 其余 DID 同理
```

### 5.2 关键特征

- **读操作极简**：大部分读操作只需 `memcpy(Data, g_xxx, sizeof(g_xxx))`，因为 g_xxx 就是 NvM RAM 镜像
- **写操作异步**：`FactoryParam_Write_ProductParam()` 实现 DCM_INITIAL → DCM_PENDING → DCM_E_OK 三段式异步写入，包含重试机制（最多1200次）
- **F187 特殊**：SOC 侧 F187 读的是 ROM 常量 `HardAndPart_Version_Info[]`，不是 NvM 变量，且根据车型码适配 Data[8..11]
- **软件版本号跨域**：F189/F195/F1A0 的读需要跨域查询 MCU，是唯一的跨域 DID 读操作
- **F1A1 特殊**：写操作除了 NvM 落盘，还要将 ADAS 配置位通过 DCMS topic 发给 driving 和 FL 模块

### 5.3 SOC 侧 DID 路由表

**文件**: `dsar-hq/src/dsar_app/product/faw/oem_feature/autosar_adapter/microsar_config_uds_v4/dji_rte.c`

```c
// 读函数指针 (工厂参数相关，完整列表)
.DID_0xF18C_ReadData_Svc = DataServices_0xF18C_ECU_Serial_Number_34Bytes_ReadData,
.DID_0xF190_ReadData_Svc = DataServices_0xF190_FAW_Vehicle_Identification_Number_17Bytes_ReadData,
.DID_0xF191_ReadData_Svc = DataServices_0xF191_FAW_ECU_Hardware_Version_Number_16Bytes_ReadData,
.DID_0xF19D_ReadData_Svc = DataServices_0xF19D_ECU_Installation_Date_4Bytes_ReadData,
.DID_0xF1A2_ReadData_Svc = DataServices_0xF1A2_System_Configuration_2_16Bytes_ReadData,
.DID_0xF1A3_ReadData_Svc = DataServices_0xF1A3_System_Configuration_3_16Bytes_ReadData,
.DID_0xF1A9_ReadData_Svc = DataServices_0xF1A9_Configuration_Form_1Bytes_ReadData,
.DID_0xF193_ReadData_Svc = DataServices_0xF193_Supplier_ECU_Hardware_Version_Number_16Bytes_ReadData,
.DID_0xF1A1_ReadData_Svc = DataServices_0xF1A1_System_Configuration_1_16Bytes_ReadData,
.DID_0xF1A7_ReadData_Svc = DataServices_0xF1A7_FAW_Reserved_16Bytes_ReadData,
.DID_0xF187_ReadData_Svc = DataServices_0xF187_FAW_Part_Number_16Bytes_ReadData,
.DID_0xF189_ReadData_Svc = DataServices_0xF189_FAW_ECU_Materials_Number_16Bytes_ReadData,
.DID_0xF1A0_ReadData_Svc = DataServices_0xF1A0_FAW_ECUSoftware_Version_Number_16Bytes_ReadData,
.DID_0xF195_ReadData_Svc = DataServices_0xF195_Supplier_ECU_Software_Version_Number_16Bytes_ReadData,

// 写函数指针 (工厂参数相关)
.DID_0xF190_WriteData_Svc = DataServices_0xF190_FAW_Vehicle_Identification_Number_17Bytes_WriteData,
.DID_0xF19D_WriteData_Svc = DataServices_0xF19D_ECU_Installation_Date_4Bytes_WriteData,
.DID_0xF1A2_WriteData_Svc = DataServices_0xF1A2_System_Configuration_2_16Bytes_WriteData,
.DID_0xF1A3_WriteData_Svc = DataServices_0xF1A3_System_Configuration_3_16Bytes_WriteData,
.DID_0xF1A9_WriteData_Svc = DataServices_0xF1A9_Configuration_Form_1Bytes_WriteData,
.DID_0xF1A1_WriteData_Svc = DataServices_0xF1A1_System_Configuration_1_16Bytes_WriteData,
.DID_0xF1A7_WriteData_Svc = DataServices_0xF1A7_FAW_Reserved_16Bytes_WriteData,
```

### 5.4 FactoryParam_Write_ProductParam 详细流程

这是 SOC 侧所有工厂参数写 DID 的通用函数，实现三段式异步写入：

```
OpStatus == DCM_INITIAL:
  s_TryWriteCnt = 0
  NvM_WriteBlock(nvm_blockid, &Data[0])     // 发起异步写入请求
  失败 → s_TryWriteBlock = TRUE (下次重试)
  返回 DCM_E_PENDING                         // 告诉DCM: 还没完成，稍后再问我

OpStatus == DCM_PENDING:
  s_TryWriteCnt++
  if (s_TryWriteBlock):
    NvM_WriteBlock(nvm_blockid, &Data[0])   // 重试写请求
  NvM_GetErrorStatus(nvm_blockid, &status)  // 查询写入状态
  Fls_Sync()                                 // 等待 Flash 操作完成
  if (status != PENDING && Fls_Sync == OK):
    memcpy(nvm_ramblock, Data, size)         // 同步更新 RAM 镜像
    返回 DCM_E_OK                            // 写入成功
  else if (s_TryWriteCnt <= 1200):
    返回 DCM_E_PENDING                       // 继续等待
  else:
    返回 E_NOT_OK                            // 超时失败
```

---

## 六、路径③ DCMS 跨域请求 —— 其他模块读写 MCU PM

### 6.1 跨域读：SOC → MCU 读工厂参数

```
SOC 侧客户端 (dcms_mcu_topic_send)
  │
  ▼ DCMS_TOPIC_FACTORY_PARAM_SYNC = "/sys/mcu_param_manager/read/v2"
  │  McuParamManagerRdReq { item_name[32] }    // "arr_vin", "arr_part_num", "all" 等
  │
  ▼ MCU 侧 DCMS Server 收到请求
Dcms_Mcu_Service_Factory_Param_Read_Callback(data, len)
  │
  ├─ read_param_block_by_id(CFG_FACTORY, &pm_factory_param_list, ...)
  │   读取整个 pm_factory 块的所有14个字段到 RAM
  │
  ├─ strcmp(item_name, "all")        → 返回整个 pm_factory_param_list (248字节)
  ├─ strcmp(item_name, "arr_vin")    → 返回 pm_factory_param_list.arr_vin (17字节)
  ├─ strcmp(item_name, "arr_part_num") → 返回 .arr_part_num (16字节)
  └─ ... 其余字段同理
  │
  ▼ dcms_mcu_service_server_set_ackdata()
  返回 McuParamManagerRdRsp { item_name[32], result, len, data[200] }
```

**注意**：`/sys/mcu_param_manager/read/v1` (旧版) 仍被 `security_cert_manager` 使用，v2 版供工厂参数使用。

### 6.2 跨域写：SOC → MCU 写工厂参数

```
SOC 侧客户端 (dcms_mcu_topic_send)
  │
  ▼ DCMS_TOPIC_WRITE_FACTORY_PARAM_SYNC = "/sys/mcu_param_manager/write/v2"
  │  FactoryParamManagerwriteReq { item_name[32], len, data[200] }
  │
  ▼ MCU 侧 DCMS Server 收到请求
Dcms_Mcu_Service_Factory_Param_Write_Callback(data, len)
  │
  └─ FactoryParam_WriteOemArgu(item_name, data)
      ├─ strcmp(key, "arr_vin")     → write_param_by_id(CFG_FACTORY, data, ..., arr_vin)
      ├─ strcmp(key, "arr_part_num") → write_param_by_id(CFG_FACTORY, data, ..., arr_part_num)
      └─ ... 其余字段同理
  │
  ▼ dcms_mcu_service_server_set_ackdata()
  返回 FactoryParamManagerwriteRsp { result }
```

### 6.3 注册初始化

**文件**: `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/FactoryParam/FactoryParam.c:327-338`

```c
void Factory_Params_Sync_Init(void)
{
    // 注册读服务 (MCU 作为 server)
    dcms_mcu_topic_setup_callback(DCMS_TOPIC_FACTORY_PARAM_SYNC,
        Dcms_Mcu_Service_Factory_Param_Read_Callback, NULL, 0);
    // 注册写服务 (MCU 作为 server)
    dcms_mcu_topic_setup_callback(DCMS_TOPIC_WRITE_FACTORY_PARAM_SYNC,
        Dcms_Mcu_Service_Factory_Param_Write_Callback, NULL, 0);
}
```

### 6.4 谁调用了这些跨域接口？

跨域读写 MCU PM 的主要是**安全认证模块** (`security_cert_manager`)，使用 v1 路径。直接通过 DCMS topic 读写 MCU 侧工厂参数，不经过 SOC 侧 NvM。

---

## 七、路径④ ECU Shell —— 跨核调试命令

### 7.1 架构概览

```
终端 (dshell -d dsar)
  │
  ▼ UDP/IPC
MCU Shell Server (RTOS)
  │
  ├─ factory_param_nvm_set [param_id] [datastr...]       ← ASCII 字符串写入
  ├─ hexfactory_param_nvm_set [param_id] [addr] [hex...]  ← 十六进制逐字节写入
  └─ factory_param_nvm_get                                 ← 读取上次写入结果
       │
       ▼ 组装 msg_factory_param_request_t { param_id, param_data[128], param_len }
       ▼ dcms_mcu_topic_send_msg(DCMS_FACTORY_PARAM_SET)
       │
       ▼ SOC 侧 (跨核)
cross_factory_param_set_callback(data, len)    // cross_ecu_shell.c:73
  │
  ├─ 校验 param_id 范围
  ├─ 校验 param_len 匹配
  ├─ memcpy(factory_param_info[param_id].param_addr, data, len)
  │   ↑ 注意: SOC侧的 param_addr 指向 g_xxx (真正的 NvM 全局变量)
  ├─ 如果是 SN → FactoryParam_OemSnSyncFlagSet()
  ├─ 如果是 VIN → FactoryParam_OemVinSyncFlagSet()
  ├─ NvM_WriteAll()  // 将所有 NvM block 写回 Flash
  ├─ Fls_Sync()       // 等待 Flash 操作完成
  │
  ▼ 返回结果
cross_factory_param_set_response()
  └─ dcms_mcu_topic_send_msg(DCMS_FACTORY_PARAM_SET_RESULT) → MCU 侧
       │
       ▼ MCU 侧接收
factory_param_nvm_set_callback()    // Diag_CrossEcuShell.c:69
  └─ 设置 factory_param_respond_flag = true
  └─ 保存返回信息到 factory_param_respond_info[512]
       │
       ▼ MCU 终端
do_factory_param_nvm_get()    // 打印 factory_param_respond_info
```

### 7.2 关键设计细节

**MCU 侧 param_addr 指向本地静态缓冲区，SOC 侧指向真正的 NvM 全局变量**：

| 侧 | 文件 | param_addr 指向 | 含义 |
|----|------|-----------------|------|
| MCU | Diag_CrossEcuShell.c | `static uint8_t g_vin1[17]` 等局部静态变量 | 仅作命令参数暂存，写入后通过 DCMS 发给 SOC |
| SOC | cross_ecu_shell.c | `g_vin`, `g_part_number` 等 NvM 全局变量 | 直接操作 NvM RAM 镜像 |

**编译开关**：MCU 侧的 shell 命令和注册在 `#ifdef MC_SHELL_SUPPORT` 下，仅调试版本编译。

### 7.3 DCMS Topic 定义

| Topic | 方向 | 用途 |
|-------|------|------|
| `DCMS_FACTORY_PARAM_SET` = `/sys/product_param_set/v1` | MCU→SOC | MCU Shell 发送写请求到 SOC |
| `DCMS_FACTORY_PARAM_SET_RESULT` = `/sys/product_param_set_result/v1` | SOC→MCU | SOC 返回写入结果给 MCU Shell |

---

## 八、路径⑤ 系统属性同步 + DSSAD 上报

### 8.1 FactoryParam_SyncThread 同步线程

**文件**: `dsar-hq/src/dsar_app/product/faw/oem_feature/FactoryParam/FactoryParam.c:1228-1333`

这是一个独立的后台线程（优先级11），每秒运行一次，负责以下定时任务：

```
t=0~9s:   空闲等待
t=10s:    FactoryParam_System_Configuration1_topic_send()
            → 解析 g_sys_configure_1[] 的ADAS bit → 发送 DMM/FL
            → 带重试 (DMM 3s重试3次, FL 20s重试5次)
t=20s:    读取系统属性
            dji_mini_param_get_system_property("rw.oem.sn", ...)
            dji_mini_param_get_system_property("rw.oem.vin", ...)
t=21s:    同步 SN 到系统属性 (如果与 NvM 不一致)
            dji_mini_param_set_system_property("rw.oem.sn", ...)
t=22s:    同步 VIN 到系统属性 (如果与 NvM 不一致)
            dji_mini_param_set_system_property("rw.oem.vin", ...)

即时触发:
  sOemSnSyncFlag == true  → 立即同步 SN
  sOemVinSyncFlag == true → 立即同步 VIN
  (标志由 DID 写操作和 ECU Shell 写操作触发)
```

### 8.2 FactoryParam_SendtoDssad 周期上报

**文件**: `dsar-hq/src/dsar_app/product/faw/oem_feature/FactoryParam/FactoryParam.c:62-96`

每 60 秒（`time_ms > 30000`，每次+5ms，即 6000 次 × 5ms = 30s，实际约 60s），发送以下数据到 DSSAD：

```c
Dssad_Factory_Param_t {
    VIN[17],           // ← g_vin
    HW_Ver[16],        // ← g_ecu_hd_version
    SN[34],            // ← g_ecu_serial_number
    SW_Ver[16]         // ← Software_Version (含MCU动态查询结果)
}
→ dcms_mcu_topic_send(DCMS_FAW_DSSAD_FACTORY_PARAM, ...)
```

前3次上报（约前180秒）时如果软件版本号还没获取到，会主动尝试重新查询 MCU。

---

## 九、跨域分析

### 9.1 哪些 DID 涉及跨域？

| DID | 操作 | MCU侧 | SOC侧 | 跨域？ |
|-----|------|-------|-------|--------|
| F187 | 读 | read_param_by_id (本地) | memcpy ROM常量 (本地) | **否** |
| F189 | 读 | read_param_by_id (本地) | **实时查询MCU** | **是** (SOC→MCU) |
| F18C | 读 | read_param_by_id (本地) | memcpy(g_ecu_serial_number) (本地) | **否** |
| F190 | 读/写 | read/write_param_by_id (本地) | memcpy(g_vin) / NvM_WriteBlock (本地) | **否** |
| F191 | 读 | read_param_by_id (本地) | memcpy(g_ecu_hd_version) (本地) | **否** |
| F193 | 读 | read_param_by_id (本地) | memcpy(g_ecu_hd_version) (本地) | **否** |
| F195 | 读 | read_param_by_id (本地) | **实时查询MCU** | **是** (SOC→MCU) |
| F198 | 读/写 | read/write_param_by_id (本地) | 本地 (SOC侧待实现) | **否** |
| F199 | 读/写 | read/write_param_by_id (本地) | 本地 (SOC侧待实现) | **否** |
| F19D | 读/写 | read/write_param_by_id (本地) | memcpy / NvM_WriteBlock (本地) | **否** |
| F1A0 | 读 | read_param_by_id (本地) | **实时查询MCU** | **是** (SOC→MCU) |
| F1A1 | 读/写 | read/write_param_by_id (本地) | memcpy / NvM_WriteBlock + **发DMM/FL** (本地域内) | **否** (DMM/FL是SOC域内通信，非跨核) |
| F1A2 | 读/写 | read/write_param_by_id (本地) | memcpy / NvM_WriteBlock (本地) | **否** |
| F1A3 | 读/写 | read/write_param_by_id (本地) | memcpy / NvM_WriteBlock (本地) | **否** |
| F1A7 | 读/写 | read/write_param_by_id (本地) | memcpy / NvM_WriteBlock (本地) | **否** |
| F1A9 | 读/写 | read/write_param_by_id (本地) | memcpy(校验01) / NvM_WriteBlock (本地) | **否** |

### 9.2 跨域总结

**唯一的跨域 DID 读：软件版本号 (F189/F195/F1A0)**

- 原因：软件版本号的最后4字节（如 `1001`）由 MCU 侧在运行时动态提供，SOC 侧无法从 NvM 获取
- 机制：SOC 通过 DCMS_TOPIC_GET_SOFTWARE_VERSION topic 实时请求，异步回调更新 `Software_Version[12..15]`
- 时序：DCM_INITIAL 立即返回 PENDING，DCM 框架每 5ms 重试，SOC 侧每秒发一次请求，直到获取成功

**F1A1 不是跨域**：

- SOC 侧 F1A1 写操作发送的 DCMS_TOPIC_DMM_PARAM_CONFIG 和 DCMS_TOPIC_DMM_PARAM_CONFIG_FL 是发给 SOC 域内的 driving/FL 模块（非跨核到 MCU）
- MCU 侧 F1A1 写操作仅通过 write_param_by_id 写入 PM，不发送任何 DCMS topic

**DCMS 跨域读写 MCU PM（路径③）不是由 DID 触发的**：

- `/sys/mcu_param_manager/read/v2` 和 `/sys/mcu_param_manager/write/v2` 是两个独立的 DCMS 服务
- 主要由 `security_cert_manager`（安全认证模块）调用，用于读写 MCU 侧工厂参数
- 不经过 SOC NvM，直接操作 MCU PM

---

## 十、以 F187 (零件号) 为例的完整端到端对比

### 10.1 DoCAN 侧 (CAN → MCU)

```
外部诊断仪发送: 22 F187 (Read Data By Identifier)

CAN 帧 → CanTp → PduR → Dcm
  │
  ▼ Dcm 查 dji_rte.c:
  .DID_0xF187_ReadData_Svc = FactoryParam_FAW_Part_Number_16Bytes_F187_ReadData
  │
  ▼ FactoryParam.c:463
  read_param_by_id(CFG_FACTORY, &PartNumTemp[0], pm_factory_param_block_t, arr_part_num)
  │
  ▼ PM 框架:
  从 pm_factory 参数块的 NvM 存储中读取 arr_part_num 字段 (16字节)
  │
  ▼ memcpy(Data, PartNumTemp, 16)
  返回 DCM_E_OK
  │
  ▼ Dcm → UDS 响应: 62 F187 + 16字节数据

结论: 纯本地操作，不走跨域
```

### 10.2 DoIP 侧 (以太网 → SOC)

```
外部诊断仪发送: 22 F187 (Read Data By Identifier)

以太网帧 (端口 13400) → SoAd → DoIP → PduR → Dcm
  │
  ▼ Dcm 查 dji_rte.c:
  .DID_0xF187_ReadData_Svc = DataServices_0xF187_FAW_Part_Number_16Bytes_ReadData
  │
  ▼ FactoryParam.c:814
  memcpy(Data, HardAndPart_Version_Info, sizeof(g_part_number))
  // HardAndPart_Version_Info = {'3','6','2','9','1','0','0','-','H','X','0','3','0','0','0','0'}
  │
  ▼ 根据 Diag_GetCarModelValue() 覆写 Data[8..11]:
  E001(30) → Data[8]='H', Data[9]='S'
  E009(31) → Data[8]='H', Data[9]='X', Data[12..15]=0
  E202(32) → Data[8]='Q', Data[9]='F'
  P301(50) → Data[8]='Q', Data[9]='X', Data[12..15]=0
  ... 等12种车型
  │
  ▼ 返回 DCM_E_OK
  │
  ▼ Dcm → UDS 响应: 62 F187 + 16字节数据

结论: 纯本地操作，读取ROM常量而非NvM变量，不走跨域
关键差异: DoCAN侧读PM存储(可被产线编程的零件号)，DoIP侧读ROM常量(固件编译时确定)
```

---

## 十一、以 F190 (VIN) 为例的完整端到端对比

### 11.1 DoCAN 读 (CAN → MCU)

```
22 F190 → Dcm → FactoryParam_FAW_Vehicle_Identification_Number_17Bytes_F190_ReadData()
  → read_param_by_id(CFG_FACTORY, &temp, ..., arr_vin)
  → memcpy(Data, temp, 17) → DCM_E_OK
```

### 11.2 DoCAN 写 (CAN → MCU)

```
2E F190 → Dcm → FactoryParam_FAW_Vehicle_Identification_Number_17Bytes_F190_WriteData()
  → write_param_by_id(CFG_FACTORY, &temp, ..., arr_vin)
  → PM 框架写入 NvM → DCM_E_OK
```

### 11.3 DoIP 读 (以太网 → SOC)

```
22 F190 → Dcm → DataServices_0xF190_FAW_Vehicle_Identification_Number_17Bytes_ReadData()
  → memcpy(Data, g_vin, sizeof(g_vin)) → DCM_E_OK
  (g_vin 是 NvM RAM 镜像，上电时从 Flash 加载)
```

### 11.4 DoIP 写 (以太网 → SOC)

```
2E F190 → Dcm → DataServices_0xF190_FAW_Vehicle_Identification_Number_17Bytes_WriteData()
  → FactoryParam_Write_ProductParam(0xF190, NvM_VIN, g_vin, 17, Data, ...)
    ├─ NvM_WriteBlock → Fls_Sync → memcpy(g_vin, Data, 17)
    └─ DCM_E_OK
  → FactoryParam_OemVinSyncFlagSet()  // 触发系统属性同步
  → FactoryParam_SyncThread 下次循环(22s或即时)同步到 rw.oem.vin
```

### 11.5 VIN 数据流总结

```
产线CAN诊断仪 ──→ MCU PM/NvM (arr_vin)
产线以太网诊断仪 ──→ SOC NvM (g_vin) ──→ rw.oem.vin 系统属性
                                         ──→ DSSAD (60s周期)
```

两条路径各自独立，互不同步。

---

## 十二、关键文件索引

### MCU 侧 (FW)

| 文件 | 作用 |
|------|------|
| `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/FactoryParam/FactoryParam_Type.h` | 数据结构定义：`pm_factory_param_block_t`(14字段)、跨域请求/响应结构体、错误码枚举 |
| `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/FactoryParam/FactoryParam_Cfg.c` | PM参数注册：`DEC_ARR`宏注册14个参数、`DECLEAR_PARAM_BLOCK_EXT`声明参数块 |
| `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/FactoryParam/FactoryParam.c` | 核心实现(1024行)：所有DID读写handler、DCMS跨域读写服务回调、`FactoryParam_WriteOemArgu`通用写函数 |
| `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/FactoryParam/FactoryParam.h` | MCU侧头文件 |
| `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/FactoryParam/FactoryParam_If.h` | 对外接口声明 |
| `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/DcmApp/dji_rte.c` | DID函数指针路由表：读写handler绑定 |
| `dsar-hq/src/dsar_fw/product/faw/oem_feature/diagnosis/DcmApp/Diag_CrossEcuShell.c` | ECU Shell MCU侧：`factory_param_nvm_set`/`hexfactory_param_nvm_set`/`factory_param_nvm_get` shell命令实现 |

### SOC 侧 (APP)

| 文件 | 作用 |
|------|------|
| `dsar-hq/src/dsar_app/product/faw/oem_feature/FactoryParam/FactoryParam_Cfg.c` | NvM全局变量定义：`g_vin[17]`、`g_part_number[16]`等16个全局变量 + ROM常量 + 软件版本号初始值 |
| `dsar-hq/src/dsar_app/product/faw/oem_feature/FactoryParam/FactoryParam.c` | 核心实现(1361行)：所有DID读写handler、`FactoryParam_Write_ProductParam`通用写函数、软件版本跨域查询、`FactoryParam_SyncThread`同步线程、DSSAD上报 |
| `dsar-hq/src/dsar_app/product/faw/oem_feature/FactoryParam/FactoryParam.h` | SOC侧头文件 |
| `dsar-hq/src/dsar_app/product/faw/oem_feature/FactoryParam/FactoryParam_If.h` | 对外接口声明 |
| `dsar-hq/src/dsar_app/product/faw/oem_feature/autosar_adapter/microsar_config_uds_v4/dji_rte.c` | DID函数指针路由表 |
| `dsar-hq/src/dsar_app/product/faw/oem_feature/autosar_adapter/microsar_config_uds_v4/Appl/Source/cross_ecu_shell.c` | ECU Shell SOC侧：`cross_factory_param_set_callback` 接收MCU写请求并写入NvM |
| `dsar-hq/src/dsar_app/product/faw/oem_feature/autosar_adapter/microsar_config_uds_v4/Appl/GenData/NvM_Cfg.c` | NvM配置：RamBlockDataAddress指向g_xxx的证据 |

### DCMS Topic 配置

| Topic 字符串 | 宏定义 | 方向 | 用途 |
|-------------|--------|------|------|
| `/sys/mcu_param_manager/read/v2` | DCMS_TOPIC_FACTORY_PARAM_SYNC | SOC→MCU (client/server) | 跨域读MCU PM |
| `/sys/mcu_param_manager/write/v2` | DCMS_TOPIC_WRITE_FACTORY_PARAM_SYNC | SOC→MCU (client/server) | 跨域写MCU PM |
| `/sys/product_param_set/v1` | DCMS_FACTORY_PARAM_SET | MCU→SOC (pub/sub) | ECU Shell 写请求 |
| `/sys/product_param_set_result/v1` | DCMS_FACTORY_PARAM_SET_RESULT | SOC→MCU (pub/sub) | ECU Shell 写结果 |
| `/sys/update/app_install/firmware_version_query` | DCMS_TOPIC_GET_SOFTWARE_VERSION | SOC→MCU | 软件版本号查询 |
| `/sys/driving/app_param_config/v1` | DCMS_TOPIC_DMM_PARAM_CONFIG | SOC内部 | F1A1→Driving模块 |
| `/sys/fl/app_param_config/v1` | DCMS_TOPIC_DMM_PARAM_CONFIG_FL | SOC内部 | F1A1→FL模块 |
| `/dssad/faw_factory_param/v1` | DCMS_FAW_DSSAD_FACTORY_PARAM | SOC→DSSAD | 60s周期上报 |

---

## 十三、架构图

```
┌──────────────────────────────────────────────────────────────────────────────────────┐
│                              工厂参数 完整架构图                                        │
│                                                                                      │
│  ┌─────────────────────────────────┐  ┌────────────────────────────────────────────┐  │
│  │        MCU 侧 (R52 RTOS)        │  │           SOC 侧 (HPC QNX/Linux)            │  │
│  │                                 │  │                                            │  │
│  │  路径① CAN诊断 (本地)            │  │  路径② DoIP诊断 (本地)                       │  │
│  │  ┌───────────┐                  │  │  ┌───────────┐                             │  │
│  │  │ CAN总线    │                  │  │  │ 以太网     │                             │  │
│  │  │ CanTp/PduR│                  │  │  │ DoIP/PduR │                             │  │
│  │  │ Dcm       │                  │  │  │ Dcm       │                             │  │
│  │  │   ↓       │                  │  │  │   ↓       │                             │  │
│  │  │ dji_rte.c │ 函数指针表        │  │  │ dji_rte.c │ 函数指针表                    │  │
│  │  │   ↓       │                  │  │  │   ↓       │                             │  │
│  │  │ Factory   │ DID Handler      │  │  │ Factory   │ DID Handler                  │  │
│  │  │ Param.c   │                  │  │  │ Param.c   │                              │  │
│  │  │   ↓       │                  │  │  │   ↓       │                              │  │
│  │  │ PM框架    │ read/write       │  │  │ NvM框架   │ memcpy(g_xxx) /              │  │
│  │  │   ↓       │ _param_by_id     │  │  │   ↓       │ NvM_WriteBlock+Fls_Sync      │  │
│  │  │ NvM/Flash │ AUTOSAR NvM      │  │  │ Flash     │ Fls_Sync                     │  │
│  │  └───────────┘                  │  │  └───────────┘                             │  │
│  │                                 │  │                                            │  │
│  │  路径③ DCMS跨域服务 (作为Server) │  │  路径⑤ 系统属性同步 + DSSAD                 │  │
│  │  ┌──────────────────────┐       │  │  ┌────────────────────────────┐            │  │
│  │  │ read/write topic:    │       │  │  │ FactoryParam_SyncThread    │            │  │
│  │  │ /mcu_param_manager/  │       │  │  │ t=10s: F1A1→DMM/FL        │            │  │
│  │  │ read/v2, write/v2    │       │  │  │ t=21s: SN→rw.oem.sn       │            │  │
│  │  └──────────────────────┘       │  │  │ t=22s: VIN→rw.oem.vin     │            │  │
│  │                                 │  │  │ 即时: 标志位触发同步       │            │  │
│  │  路径④ ECU Shell (本地命令)      │  │  │                            │            │  │
│  │  ┌──────────────────────┐       │  │  │ FactoryParam_SendtoDssad   │            │  │
│  │  │ factory_param_nvm_set│       │  │  │ 60s: VIN/HW/SN/SW→DSSAD   │            │  │
│  │  │ hexfactory_param_xxx │       │  │  └────────────────────────────┘            │  │
│  │  │ factory_param_nvm_get│       │  │                                            │  │
│  │  └──────┬───────────────┘       │  │  路径④ ECU Shell SOC侧处理                  │  │
│  │         │ DCMS topic            │  │  ┌────────────────────────────┐            │  │
│  │         │ /product_param_set    │  │  │ cross_factory_param_set    │            │  │
│  │         │              ↓        │  │  │ _callback()                │            │  │
│  │         └───────────────────────┼──┼─→│ → memcpy(g_xxx)            │            │  │
│  │                                 │  │  │ → NvM_WriteAll+Fls_Sync   │            │  │
│  │  跨域软件版本号查询 (SOC→MCU)     │  │  │ → 发回结果                  │  │            │  │
│  │  ┌──────────────────────┐       │  │  └────────────────────────────┘            │  │
│  │  │ firmware_version_    │←──┼──┼──│ SoftWare_Version_Get()                      │  │
│  │  │ query callback       │       │  │   ↑ F189/F195/F1A0 读触发                   │  │
│  │  └──────────────────────┘       │  │                                            │  │
│  └─────────────────────────────────┘  └────────────────────────────────────────────┘  │
│                                                                                      │
│  ═══════════════════════════════════════════════════════════════════════════════════  │
│  关键结论:                                                                           │
│  1. CAN→MCU PM 与 DoIP→SOC NvM 是两条独立路径，互不同步                               │
│  2. 唯一跨域DID读: F189/F195/F1A0 (软件版本号实时查询MCU)                              │
│  3. 读 memcpy(g_xxx) 就是读存储 (g_xxx = NvM RAM 镜像)                                │
│  4. F1A1写触发ADAS bit解析和DCMS发给DMM/FL (SOC域内，非跨核)                           │
│  5. ECU Shell路径: MCU命令 → DCMS topic → SOC NvM (跳过SOC Dcm)                      │
└──────────────────────────────────────────────────────────────────────────────────────┘
```
