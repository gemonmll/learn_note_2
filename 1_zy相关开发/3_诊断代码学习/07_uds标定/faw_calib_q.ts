import { ScriptArg } from 'dkit'
import * as fs from 'fs';
//脚本主体
export const main = async (kit: ScriptArg) => {
    // try {
    //     let app_id: string = "cli_a9f3023a0afa9bc7"
    //     let app_secret: string = "PynID5W7MWvM34AIT7zpzeUgNERCR31D"
    //     let tenant_access_token = await kit.utils.getTenantAccessToken(app_id, app_secret)
    //     let app_token = "Nw86b5pPJaLI1zstdYCcaZAWnyl"
    //     let table_id = "tblCaZGNKCKLGN5y"
    //     let uuid = ""
    //     //获取用户操作系统
    //     let os = kit.utils.getOs()
    //     if (os == 'win32') {
    //         // 获取用户UUID
    //         let uuid_output = kit.utils.execSync(`powershell -c "(Get-CimInstance Win32_ComputerSystemProduct).UUID.Trim()`)
    //         let uuidLines = uuid_output.split('\n').filter((line: string) => line.trim() !== '');
    //         uuid = uuidLines[0]?.trim(); // 取出第二行并去除首尾空格
    //     } else if (os == 'linux') {
    //         // 获取用户UUID
    //         uuid = kit.utils.execSync(`ls -1 /dev/disk/by-uuid/ | head -n1`).trim()

    //     }
    //     //获取dkit版本
    //     let plugin_json_path = kit.path.getAbsolutePath('./plugin.json')
    //     let jsonData = JSON.parse(fs.readFileSync(plugin_json_path, 'utf-8'))
    //     let version = jsonData.version

    //     let fields = {
    //         "任务类型": "工站标定",
    //         "dkit版本": version,
    //         "操作系统": os,
    //         "UUID": uuid,
    //         "日期": Date.now(),
    //     }
    //     kit.utils.addRecord(app_token, table_id, tenant_access_token, fields)
    // } catch (error) {
    //     kit.log('飞书统计异常，不影响正常功能')
    // }

    // 环视标定
    // 函数-读取标定失败原因

    function fail_reason_2005() {
        let ret = kit.send([0x22, 0x20, 0x05])
        for (let i = 1; i < 7; i++) {
            if (ret[2 + i] == 0x00) {
                kit.log(i + '号相机相机正常')
            }
            else if (ret[2 + i] == 0x01) {
                console.error(i + '号相机未收到图像')
            }
            else if (ret[2 + i] == 0x02) {
                console.error(i + '号相机相机内参获取失败')
            }
            else if (ret[2 + i] == 0x03) {
                console.error(i + '号相机视野内未找到对应标定板')
            }
            else if (ret[2 + i] == 0x04) {
                console.error(i + '号相机摄像头内参不符合预期')
            }
            else if (ret[2 + i] == 0x05) {
                console.error(i + '号相机标定板检测失败')
            }
            else if (ret[2 + i] == 0x06) {
                console.error(i + '号相机重投影误差大')
            }
            else if (ret[2 + i] == 0x07) {
                console.error(i + '号相机外参超过预定义阈值')
            }
            else if (ret[2 + i] == 0x08) {
                console.error(i + '号相机域控内部参数系统异常')
            }
            else if (ret[2 + i] == 0x09) {
                console.error(i + '号相机域控内部通讯异常')
            }
            else if (ret[2 + i] == 0x0A) {
                console.error(i + '号相机标定超时')
            }
            else if (ret[2 + i] == 0x0B) {
                console.error(i + '号相机车辆配置获取异常')
            }
            else if (ret[2 + i] == 0x0C) {
                console.error(i + '号相机其它故障')
            }
        }
    }
    // 函数-前后侧视工站前置条件检查
    function fail_reason_2004() {
        let condition = kit.send([0x22, 0x20, 0x04])
        let byte0 = condition[3].toString(2)
        if (condition[0] == 0x7f) {
            console.error('读取前置条件标定结果失败')
            return kit.exitCode.generalFailed
        }
        if (condition[3] != 0x00) {
            if (byte0[0] == '1') {
                console.error('四门两盖未关闭')
            }
            if (byte0[1] == '1') {
                console.error('档位不在N档')
            }
            if (byte0[2] == '1') {
                console.error('车灯未关闭(转向灯、示廓灯、远光灯、近光灯)')
            }
            if (byte0[3] == '1') {
                console.error('胎压检测异常')
            }
            if (byte0[4] == '1') {
                console.error('标定相关软件存在故障')
            }
            if (byte0[5] == '1') {
                console.error('主动悬架档位异常')
            }
        }
        for (let i = 1; i < 7; i++) {
            if (condition[3 + i] != 0x00) {
                if (condition[3 + i] == 0x01) {
                    console.error(i + '号相机视野过亮')
                }
                if (condition[3 + i] == 0x02) {
                    console.error(i + '号相机视野过暗')
                }
                if (condition[3 + i] == 0x03) {
                    console.error(i + '号相机存在脏污')
                }
                if (condition[3 + i] == 0x04) {
                    console.error(i + '号相机存在遮挡')
                }
                if (condition[3 + i] == 0x05) {
                    console.error(i + '号相机存在挂水')
                }
                if (condition[3 + i] == 0x06) {
                    console.error(i + '号相机存在硬件故障')
                }
                if (condition[3 + i] == 0x07) {
                    console.error(i + '号相机存在软件故障')
                }
            }
        }
    }

    //切换为10 01会话，以满足对22 F1 87的请求条件
    let res1 = kit.send([0x10, 0x01])
    //获取车型信息，以此来选择不同的解密算法
    let p567_flag = [0x51, 0x57];
    let p301_flag = [0x51, 0x58];
    let c206_10_flag = [0x51, 0x44];
    let e202_10_flag = [0x51, 0x46];
    let e001_10_flag = [0x48, 0x53];
    let res_car_model_raw = kit.send([0x22, 0xf1, 0x87])
    let car_model = res_car_model_raw.slice(11, 13)
    console.log("车型：", car_model)
    let securityFlag = "faw_p567_L1"
    if (arraysAreEqual(car_model, p567_flag)) {
        console.log("P567 L1解锁")
        securityFlag = "faw_p567_L1"
    }
    else if (arraysAreEqual(car_model, c206_10_flag)) {
        console.log("C206_10 L1解锁")
        securityFlag = "faw_c206_10_L1"
    }
    else if (arraysAreEqual(car_model, e202_10_flag)) {
        console.log("E202_10 L1解锁")
        securityFlag = "faw_e202_e001_10_L1"
    }
    else if (arraysAreEqual(car_model, e001_10_flag)) {
        console.log("E001_10 L1解锁")
        securityFlag = "faw_e202_e001_10_L1"
    }
    else if (arraysAreEqual(car_model, p301_flag)) {
        console.log("P301 L1解锁")
        securityFlag = "faw_p_L1"
    } else {
        console.log("车型未匹配，请检查car_model 参数")
    }

    //前、侧、后视标定======================
    kit.log('\n \
             1号相机:前三目左\n \
             2号相机:前三目中\n \
             3号相机:前三目右\n \
             4号相机:后单目\n \
             5号相机:左侧相机\n \
             6号相机:右侧相机\n ')
    // step 1
    await kit.switchSession(0x03);

    // step 2-3
    let auth_acyclic1 = await kit.securityAcess(0x01, { keygen: securityFlag })
    if (auth_acyclic1 == false) {
        kit.log('权限获取失败', 'error')
        return kit.exitCode.authFailed
    }

    // step 4 开始前置条件检查，激活标定
    let ret_acyclic = kit.send([0x31, 0x01, 0x51, 0x01])
    if (ret_acyclic[0] == 0x71) {
        kit.log('激活前、侧、后视标定工站成功')
        await kit.utils.sleep(1000)
    }
    else if (ret_acyclic[0] == 0x7f) {
        console.error('激活前、侧、后视标定工站失败')
        // 激活标定失败，读取检查前置条件标定结果
        fail_reason_2004()
        return kit.exitCode.generalFailed
    }
    // step 5 激活标定成功，查询标定状态
    let auth_acyclic = await kit.securityAcess(0x01, { keygen: securityFlag })
    if (auth_acyclic == false) {
        kit.log('权限获取失败', 'error')
        return kit.exitCode.authFailed
    }
    //------------
    let Calibration_falg_acyclic = -1    //标定标志位 1失败 0成功
    for (let i = 0; i < 60; i++) {
        // 查询所有相机，有异常就退出
        if (Calibration_falg_acyclic == 1) {
            break
        }
        await kit.utils.sleep(1000)
        let ret = kit.send([0x31, 0x03, 0x51, 0x01])
        if (ret[4] == 0x00) {
            kit.log('前、侧、后视工站标定成功')
            Calibration_falg_acyclic = 0
            break
        }
        else if (ret[4] == 0x01) {
            kit.log('总状态：工站标定中...')
        }
        else if (ret[4] == 0x02) {
            console.error('总状态：工站标定失败')
            Calibration_falg_acyclic = 1
        }
        else if (ret[4] == 0x03) {
            console.error('总状态：工站标定超时')
            Calibration_falg_acyclic = 1
        }
        else if (ret[4] == 0x04) {
            console.error('工站标定异常中止')
            Calibration_falg_acyclic = 1
        }
        for (let i = 1; i < 7; i++) {
            if (ret[4 + i] == 0x01) {
                kit.log(i + '号相机工站标定中...')
            }
            else if (ret[4 + i] == 0x02) {
                console.error(i + '号相机工站标定失败')
                Calibration_falg_acyclic = 1
            }
            else if (ret[4 + i] == 0x03) {
                console.error(i + '号相机工站标定超时')
                Calibration_falg_acyclic = 1
            }
        }
        if (ret[6] == 0x04) {
            console.error('1号相机标定异常中止')
            Calibration_falg_acyclic = 1
        }
        if (ret[6] == 0x05) {
            console.error('2号相机标定异常中止')
            Calibration_falg_acyclic = 1
        }
        if (ret[7] == 0x06) {
            console.error('3号相机标定异常中止')
            Calibration_falg_acyclic = 1
        }
        if (ret[8] == 0x06) {
            console.error('4号相机标定异常中止')
            Calibration_falg_acyclic = 1
        }
        if (ret[9] == 0x06) {
            console.error('5号相机标定异常中止')
            Calibration_falg_acyclic = 1
        }
        if (ret[10] == 0x06) {
            console.error('6号相机标定异常中止')
            Calibration_falg_acyclic = 1
        }
    }
    if (Calibration_falg_acyclic == 1) {
        fail_reason_2004()
        fail_reason_2005()
        kit.send([0x14, 0xFF, 0xFF, 0xFF])
        return kit.exitCode.generalFailed
    }
    else if (Calibration_falg_acyclic == 0) {
        // step 6 读取标定结果
        kit.send([0x22, 0x20, 0x03])
        // step 7 清除DTC
        kit.send([0x14, 0xFF, 0xFF, 0xFF])
        kit.send([0x11, 0x01])
        return kit.exitCode.success
    }
    else if (Calibration_falg_acyclic == -1) {
        console.error('环视工站标定超时(60S)')
        fail_reason_2004()
        fail_reason_2005()
        kit.send([0x14, 0xFF, 0xFF, 0xFF])
        return kit.exitCode.generalFailed
    }
};

/**
 * 深度比较两个数组的值是否相等
 * @param arr1 第一个数组
 * @param arr2 第二个数组
 * @returns 如果两个数组的值完全相等则返回true，否则返回false
 */
function arraysAreEqual<T>(arr1: T[], arr2: T[]): boolean {
    // 如果引用相同，直接返回true
    if (arr1 === arr2) return true;

    // 如果长度不同，直接返回false
    if (arr1.length !== arr2.length) return false;

    // 逐个比较元素
    for (let i = 0; i < arr1.length; i++) {
        const val1 = arr1[i];
        const val2 = arr2[i];

        // 如果两个元素都是数组，递归比较
        if (Array.isArray(val1) && Array.isArray(val2)) {
            if (!arraysAreEqual(val1, val2)) return false;
            continue;
        }

        // 如果两个元素都是对象，使用JSON.stringify比较（简单对象比较）
        if (typeof val1 === 'object' && val1 !== null &&
            typeof val2 === 'object' && val2 !== null) {
            if (JSON.stringify(val1) !== JSON.stringify(val2)) return false;
            continue;
        }

        // 基本类型直接比较
        if (val1 !== val2) return false;
    }

    return true;
}