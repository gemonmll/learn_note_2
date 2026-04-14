#include "vehicle_ota.h"
#include "dsar_plat_bf/cdd/dcms_mcu_api.h"
#include "dsar_plat_bf/cdd/debug.h"
#include "dcms_mcu_config_uds.h"
#include "someip_ota.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "dsar_plat_bf/cdd/system_osal.h"
#include "dsar_plat_bf/diag/base64.h"
#include "dsar_plat_bf/diag/osal.h"
#include "cryptomodule_api.h"
#include "DiagProxy_If.h"
#include "vehicle_ota_If.h"

#define MAX_PATH_LENGTH 128
#define PACKET_DATA_SIZE 10240
#define DATA_BUFFER_SIZE (8 * 1024)
#define UPDATE_TYPE "ota"
#define LIBCRYPTOMODULE_PATH "/mnt/dji/apps/middleware/lib/libcryptomodule.so"

/*
 * First-pass recovery from OCR.
 * Notes:
 * 1) This file is still incomplete.
 * 2) Ambiguous or damaged OCR areas are marked with TODO/RECOVERY_GAP.
 * 3) Symbol names/macros may need to be aligned with your real headers.
 */

RRSelfDownloadCmd_E009_t RRSelfDownloadCmd_E009;
RRSelfDownloadCmd_E001_t RRSelfDownloadCmd_E001;
RRSelfDownloadCmd_P301_t RRSelfDownloadCmd_P301;
PS_SelfDLCmd_strt_P567 RRSelfDownloadCmd_P567;

SelfDLStatus_Req_oRRSelfDownloadStatus_t SelfDLStatus_Req_oRRSelfDownloadStatus;
SelfDLScript_Req_oRRSelfDownloadScript_t SelfDLScript_Req_oRRSelfDownloadScript;
QueryFileDownloadRsp_t QueryFileDownloadRsp;

SelfDLStatus_Out_oRRSelfDownloadStatus_E009_t SelfDLStatus_Out_oRRSelfDownloadStatus_E009;
SelfDLStatus_Out_oRRSelfDownloadStatus_E001_t SelfDLStatus_Out_oRRSelfDownloadStatus_E001;
SelfDLStatus_Out_oRRSelfDownloadStatus_P301_t SelfDLStatus_Out_oRRSelfDownloadStatus_P301;
PS_SelfDLStatus_strt SelfDLStatus_Out_oRRSelfDownloadStatus_P567;

int64_t download_id = 0;
int64_t loop_counter = 0;
int32_t Script_Notify_Ack_flag = 0;
int32_t first_execution_flag = 1;
int32_t car_model = 0;
static int32_t get_hash_flag = 0;
static int32_t last_zip_exit_flag = 0;
static int32_t Status_flag = 0;
static int32_t dec_flag = 0;
static int32_t l_flag = 0;
static int32_t sHashComputerFlag = 0;
static int32_t sGatewayPostDownloadFlag = 0;
static int32_t Downloadn_flag = 0;

char sha256_buffer[32] = {0};
char sha256_buffer_string[64] = {0};
char tar_gz_path[256] = {0};
char xml_path[256] = {0};
uint8_t Last_Sha256_arr[128] = {0};
uint8_t LC_Sha256_arr[128] = {0};
char mv_file_command[256] = {0};
char unzip_file_command[256] = {0};

extern int32_t Diag_GetCarModelValue(void);

static void Binary_To_Hex_String(const unsigned char *data,
                                 size_t data_length,
                                 char *hex_string)
{
    const char *hex_digit = "0123456789abcdef";

    if ((data == NULL) || (hex_string == NULL)) {
        return;
    }

    for (size_t i = 0; i < data_length; i++) {
        hex_string[2 * i]     = hex_digit[(data[i] >> 4) & 0x0F];
        hex_string[2 * i + 1] = hex_digit[data[i] & 0x0F];
    }

    hex_string[2 * data_length] = '\0';
}

static int32_t read_data_from_file(const char *file_path,
                                   const uint32_t buffer_len,
                                   uint8_t *data,
                                   uint32_t *length)
{
    int32_t ret = -1;
    FILE *file = NULL;
    size_t read_len = 0;

    if ((file_path == NULL) || (data == NULL) || (length == NULL)) {
        return -1;
    }

    file = fopen(file_path, "rb");
    if (file != NULL) {
        read_len = fread(data, 1U, buffer_len, file);
        (void)DEBUG_INFO("<faw_ota>read %s data len %ld\r\n", file_path, (long)read_len);

        if (read_len > 0U) {
            *length = (uint32_t)read_len;
            ret = 0;
        }

        fclose(file);
    } else {
        ret = -1;
        (void)DEBUG_ERROR(
            "<faw_ota>read_data_from_file fail fopen:%s, errno:%d, desc:%s\r\n",
            file_path,
            errno,
            strerror(errno));
    }

    return ret;
}

static int32_t write_data_to_file(const char *file_path,
                                  const uint32_t buffer_len,
                                  uint8_t *data)
{
    int32_t ret = -1;
    FILE *file = NULL;
    size_t write_len = 0;

    if ((file_path == NULL) || (data == NULL)) {
        return -1;
    }

    file = fopen(file_path, "wb");
    if (file != NULL) {
        write_len = fwrite(data, 1U, buffer_len, file);
        (void)DEBUG_INFO("<faw_ota>write %s len %ld\r\n", file_path, (long)write_len);
        ret = (write_len == buffer_len) ? 0 : -1;
        fclose(file);
    } else {
        ret = -1;
        (void)DEBUG_ERROR(
            "<faw_ota>write_data_to_file fail fopen:%s, errno:%d, desc:%s\r\n",
            file_path,
            errno,
            strerror(errno));
    }

    return ret;
}

static int32_t vehicle_ota_mkdir(const char *path)
{
    int32_t mkdir_ret = 0;
    struct stat st;

    if (path == NULL) {
        return -1;
    }

    memset(&st, 0, sizeof(st));
    if (stat(path, &st) == -1) {
        mkdir_ret = mkdir(path, 0755);
        if (mkdir_ret != 0) {
            DEBUG_ERROR("<faw_ota>mkdir %s fail, errno:%d, desc:%s\r\n",
                        path,
                        errno,
                        strerror(errno));
        }
    } else {
        DEBUG_INFO("<faw_ota> %s exist!\r\n", path);
    }

    return mkdir_ret;
}

static int32_t Integrity_Verify(void)
{
    int32_t ret = -1;
    uint32_t hash_len = 32;
    uint32_t base64_encode_len = 0;
    struct stat buffer;

    if (stat((const char *)FAW_DECRYPTION_ZIP_PATH, &buffer) != 0) {
        (void)DEBUG_INFO("<faw_ota> FAW_DECRYPTION_ZIP_PATH not exist\r\n");
        return ret;
    }

    /* TODO: 根据 car_model 设置不同平台的下载阶段与错误码 */
    (void)DEBUG_INFO("<faw_ota> crypto_get_file_hash start\r\n");

    crypto_get_file_hash((const char *)FAW_DECRYPTION_ZIP_PATH,
                         CRYPTO_HASH_TYPE_SHA256,
                         sha256_buffer,
                         hash_len,
                         &hash_len);

    Binary_To_Hex_String((const unsigned char *)sha256_buffer, 32, sha256_buffer_string);
    char *hash_base64 = base64_encode((char *)sha256_buffer_string, 64, &base64_encode_len);
    (void)DEBUG_INFO("<faw_ota> hash_base64:%s len=%d\r\n", hash_base64, base64_encode_len);

    /* TODO: 这里 OCR 损坏严重。需要分别与
     * RRSelfDownloadCmd_E009/E001/P301/P567.PS_Sha256_arr 比较，
     * 成功时 ret = 0；失败时更新状态并删除解密包。
     */

    return ret;
}

static int32_t Signature_Verify(void)
{
    int32_t ret = -1;
    int32_t ret_test = -1;
    int32_t ret_p7 = -1;
    int32_t ret_ca = -1;
    int32_t ret_ca_test = -1;
    int32_t retRte = -1;

    char sign_txt_path[256] = {0};
    char sign_p7_path[256] = {0};
    char path[257] = {0};
    char line[DATA_BUFFER_SIZE] = {0};

    uint32_t hash_len = 64;
    uint32_t base64_encode_tar_len = 0;
    uint32_t base64_encode_xml_len = 0;

    uint8_t tar_hash[DATA_BUFFER_SIZE] = {0};
    uint8_t xml_hash[DATA_BUFFER_SIZE] = {0};
    char sha512_buf[64] = {0};

    DIR *dir = opendir(FAW_UNZIP_FILE_PATH);
    struct dirent *term = NULL;
    int readNextLine = 0;

    if (dir == NULL) {
        DEBUG_INFO("<faw_ota>Signature_Verify opendir %s fail errno:%d desc:%s\r\n",
                   FAW_UNZIP_FILE_PATH,
                   errno,
                   strerror(errno));
        return -1;
    }

    while ((term = readdir(dir)) != NULL) {
        if ((term != NULL) && (strstr(term->d_name, ".txt") != NULL)) {
            snprintf(sign_txt_path, sizeof(sign_txt_path), "%s/%s", FAW_UNZIP_FILE_PATH, term->d_name);
        } else if ((term != NULL) && (strstr(term->d_name, ".p7") != NULL)) {
            snprintf(sign_p7_path, sizeof(sign_p7_path), "%s/%s", FAW_UNZIP_FILE_PATH, term->d_name);
        } else if ((term != NULL) && (strstr(term->d_name, "tar.gz") != NULL)) {
            snprintf(tar_gz_path, sizeof(tar_gz_path), "%s/%s", FAW_UNZIP_FILE_PATH, term->d_name);
        } else if ((term != NULL) && (strstr(term->d_name, ".xml") != NULL)) {
            snprintf(xml_path, sizeof(xml_path), "%s/%s", FAW_UNZIP_FILE_PATH, term->d_name);
        }
    }
    closedir(dir);

    {
        uint8_t p7[4096] = {0};
        uint32_t p7len = 0;
        uint8_t indata[512] = {0};
        uint32_t inlen = 0;
        uint8_t root_crt[4096] = {0};
        uint32_t root_crt_len = 0;
        uint8_t root_crt_test[4096] = {0};
        uint32_t root_crt_test_len = 0;

        ret_p7 = read_data_from_file(sign_p7_path, sizeof(p7), p7, &p7len);
        read_data_from_file(sign_txt_path, sizeof(indata), indata, &inlen);
        ret_ca = read_data_from_file(CA_FILE_PATH, sizeof(root_crt), root_crt, &root_crt_len);
        ret_ca_test = read_data_from_file(CA_TEST_FILE_PATH,
                                          sizeof(root_crt_test),
                                          root_crt_test,
                                          &root_crt_test_len);

        if ((ret_p7 != 0) || ((ret_ca != 0) && (ret_ca_test != 0))) {
            DEBUG_ERROR("<faw_ota>read cert/sign file failed\r\n");
            return -1;
        }

        ret = securty_cerify_pkcs7_sign(p7,
                                        p7len,
                                        indata,
                                        inlen,
                                        root_crt,
                                        root_crt_len,
                                        NULL,
                                        0);
        ret_test = securty_cerify_pkcs7_sign(p7,
                                             p7len,
                                             indata,
                                             inlen,
                                             root_crt_test,
                                             root_crt_test_len,
                                             NULL,
                                             0);

        if ((ret != 0) && (ret_test != 0)) {
            DEBUG_ERROR("<faw_ota>securty_cerify_pkcs7_sign fail!\r\n");
            memset(Last_Sha256_arr, 0, sizeof(Last_Sha256_arr));
            sGatewayPostDownloadFlag = 0;
            last_zip_exit_flag = 0;
            return -1;
        }
    }

    crypto_get_file_hash(tar_gz_path, CRYPTO_HASH_TYPE_SHA512, sha512_buf, 64, &hash_len);
    char *tar_hash_base64 = base64_encode((char *)sha512_buf, 64, &base64_encode_tar_len);

    crypto_get_file_hash(xml_path, CRYPTO_HASH_TYPE_SHA512, sha512_buf, 64, &hash_len);
    char *xml_hash_base64 = base64_encode((char *)sha512_buf, 64, &base64_encode_xml_len);

    FILE *file = fopen(sign_txt_path, "r");
    if (file == NULL) {
        DEBUG_ERROR("<faw_ota>open %s fail!\r\n", sign_txt_path);
        return -1;
    }

    while (fgets(line, sizeof(line), file) != NULL) {
        line[strcspn(line, "\n")] = 0;

        if (readNextLine == 1) {
            strncpy((char *)tar_hash, line, sizeof(tar_hash) - 1);
            tar_hash[sizeof(tar_hash) - 1] = '\0';
            readNextLine = 0;
        } else if (readNextLine == 2) {
            strncpy((char *)xml_hash, line, sizeof(xml_hash) - 1);
            xml_hash[sizeof(xml_hash) - 1] = '\0';
            readNextLine = 0;
        } else if (strstr(line, ".tar.gz") != NULL) {
            readNextLine = 1;
        } else if (strstr(line, ".xml") != NULL) {
            readNextLine = 2;
        }
    }
    fclose(file);

    if ((strncmp(tar_hash_base64, (char *)tar_hash, base64_encode_tar_len) == 0) &&
        (strncmp(xml_hash_base64, (char *)xml_hash, base64_encode_xml_len) == 0)) {
        DEBUG_INFO("<faw_ota>SHA512 compare success!\r\n");
        retRte = 0;
    } else {
        DEBUG_ERROR("<faw_ota>SHA512 compare fail!\r\n");
        memset(Last_Sha256_arr, 0, sizeof(Last_Sha256_arr));
        sGatewayPostDownloadFlag = 0;
        last_zip_exit_flag = 0;
        retRte = -1;
    }

    return retRte;
}

static int32_t mv_lastzip_to_deczip(void)
{
    int32_t ret = system(mv_file_command);

    if (ret != 0) {
        DEBUG_ERROR("<faw_ota>mv: %s fail\r\n", mv_file_command);
        return ret;
    }

    return ret;
}

static int32_t unzip_deczip_to_file(void)
{
    int32_t ret = -1;

    for (int8_t i = 0; i < 3; i++) {
        ret = system(unzip_file_command);
        if (ret == 0) {
            break;
        }
    }

    if (ret != 0) {
        char rm_zip_path[256] = {0};
        up_snprintf(rm_zip_path, sizeof(rm_zip_path), "rm %s", FAW_DECRYPTION_ZIP_PATH);
        (void)system(rm_zip_path);
        memset(Last_Sha256_arr, 0, sizeof(Last_Sha256_arr));
        sGatewayPostDownloadFlag = 0;
        last_zip_exit_flag = 0;
        DEBUG_ERROR("<faw_ota>unzip: %s fail\r\n", unzip_file_command);
    }

    return ret;
}

static int32_t HashComputer_E009(void)
{
    PostFileDownloadReq_t PostFileDownloadReq;
    uint32_t hash_len = 32;
    uint32_t base64_encode_len = 0;

    memset(&PostFileDownloadReq, 0, sizeof(PostFileDownloadReq));
    crypto_get_file_hash((const char *)FAW_LAST_TIME_ZIP_PATH,
                         CRYPTO_HASH_TYPE_SHA256,
                         sha256_buffer,
                         hash_len,
                         &hash_len);

    Binary_To_Hex_String((const unsigned char *)sha256_buffer, 32, sha256_buffer_string);
    char *filehash_base64 = base64_encode((char *)sha256_buffer_string, 64, &base64_encode_len);

    if (strncmp(filehash_base64,
                (const char *)RRSelfDownloadCmd_E009.PS_Sha256_arr,
                base64_encode_len) == 0) {
        last_zip_exit_flag = 1;
    } else {
        last_zip_exit_flag = 0;
    }

    /* TODO: 保留原始业务逻辑，OCR 在 mode/resource_location 等处有缺损。 */
    return 0;
}

static int32_t HashComputer_E001(void)
{
    /* TODO: 与 HashComputer_E009 基本同构，仅结构体和枚举不同 */
    return 0;
}

static int32_t HashComputer_P301(void)
{
    /* TODO: 与 HashComputer_E009 基本同构，仅结构体和枚举不同 */
    return 0;
}

static int32_t HashComputer_P567(void)
{
    /* TODO: 与 HashComputer_E009 基本同构，仅结构体和枚举不同 */
    return 0;
}

int32_t DiagProxy_SelfDownloadCmd_cb(const PS_SelfDLCmd_strt_P567 *P567_SelfDLCmd)
{
    PostFileDownloadReq_t PostFileDownloadReq;
    CancelFileDownloadReq_t CancelFileDownloadReq;
    PS_OTAresponse_t RRSelfDownloadCmd_Response_P567;
    int32_t ret = -1;
    struct stat buffer;
    struct stat buffer_Sha256;
    char rm_file_path[256] = {0};

    memset(&PostFileDownloadReq, 0, sizeof(PostFileDownloadReq));
    memset(&CancelFileDownloadReq, 0, sizeof(CancelFileDownloadReq));
    memset(&RRSelfDownloadCmd_Response_P567, 0, sizeof(RRSelfDownloadCmd_Response_P567));

    DEBUG_INFO("<faw_ota>vehicle_ota_RRSelfDownloadCmd_cb start!\r\n");

    if (P567_SelfDLCmd == NULL) {
        DEBUG_ERROR("<faw_ota>vehicle_ota_RRSelfDownloadCmd_cb SelfDLCmd is NULL\r\n");
        return ret;
    }

    memcpy(&RRSelfDownloadCmd_P567, P567_SelfDLCmd, sizeof(PS_SelfDLCmd_strt_P567));

    /* TODO:
     * 1) 根据 OCR 还原 DownloadCmd=1/0 分支。
     * 2) 还原目录创建、历史 hash 比较、续传/重传逻辑。
     * 3) 还原网关下载 topic send 与状态机上报。
     */

    return ret;
}

static int32_t vehicle_ota_RRSelfDownloadCmd_cb(const uint8_t *data, uint32_t len)
{
    PostFileDownloadReq_t PostFileDownloadReq;
    CancelFileDownloadReq_t CancelFileDownloadReq;
    RRSelfDownloadCmd_Response_t RRSelfDownloadCmd_Response;
    RRSelfDownloadCmd_Response_P301_t RRSelfDownloadCmd_Response_P301;
    int32_t ret = -1;
    struct stat buffer;
    struct stat buffer_Sha256;
    char rm_file_path[256] = {0};

    memset(&PostFileDownloadReq, 0, sizeof(PostFileDownloadReq));
    memset(&CancelFileDownloadReq, 0, sizeof(CancelFileDownloadReq));
    memset(&RRSelfDownloadCmd_Response, 0, sizeof(RRSelfDownloadCmd_Response));
    memset(&RRSelfDownloadCmd_Response_P301, 0, sizeof(RRSelfDownloadCmd_Response_P301));

    DEBUG_INFO("<faw_ota>vehicle_ota_RRSelfDownloadCmd_cb start!\r\n");

    if (data == NULL) {
        return ret;
    }

    /* TODO:
     * 这里是 E009/E001/P301 等车型共用入口。
     * OCR 可看出流程大致为：
     *   - memcpy 到对应命令结构体
     *   - 打印日志
     *   - DownloadCmd == 1 时创建目录、处理历史包/hash、决定 kTrunc/kAppend
     *   - DownloadCmd == 0 时发送 Cancel_Download
     *   - 更新 SelfDLStatus_Out_*
     */

    return ret;
}
