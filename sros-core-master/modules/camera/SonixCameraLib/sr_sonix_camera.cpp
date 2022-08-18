#include "SonixCamera.h"
#include "util.h"
#include "LUOptDev.h"
#include "sr_sonix_camrea.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>


#define SF_ADDR_START	0xF000
#define SF_ADDR_END	0xF800

//#ifdef __cplusplus
//extern "C" {
//#endif
namespace usb {

int32_t sr_sonix_cam_init(int first_search_id, int *founded_usb_id, int *dev_number) {
    int32_t i = 0;

    char *vidpid = "0c4564ab";  // 05a39331  2bc5050b 05a39230 0c4564ab 2bc50509
    if (TRUE != SonixCam_Init(vidpid, first_search_id, founded_usb_id, dev_number)) {
        printf("cannot init camera!!!!!%d\n", first_search_id);
        vidpid = "0c456362";
        if (TRUE != SonixCam_Init(vidpid, first_search_id, founded_usb_id, dev_number)) {
            fprintf(stderr, "could not find/open sonix device\n");
            return -1;
        }
    }
    fprintf(stderr, "warning: find device\n");
    return 0;
}

int32_t sr_sonix_deinit(void) { return (SonixCam_UnInit() ? 0 : -1); }

/**
 * 读/写flash
 *
 * @param[in]	mode:读/写
 * @param[in]	addr:flash地址,起始0xF000,最大偏移2kB
 * @param[in]	data:数据
 * @param[in]	data_len:数据长度
 * @param[out]	None
 *
 * @return		0成功, < 0失败
 */
int32_t sr_serial_flash_rw(uint8_t mode, uint16_t addr, uint8_t *data, uint16_t data_len) {
    int32_t ret = 0;
    static sonix_bool flag = FALSE;
    DSP_ROM_TYPE romType;
    ret = SonixCam_GetAsicRomType(&romType);
    int32_t sfAddr = 0x1f000;

    if (romType == DRT_64K) {
        sfAddr = 0xF000;  // default 0xE000
        if (!flag) {
            printf("romType=%d - DRT_64K, sfAddr=0x%X, ret=%d\n", romType, sfAddr, ret);
        }
    } else if (romType == DRT_128K) {
        sfAddr = 0x1f000;
        if (!flag) {
            printf("romType=%d - DRT_128K, sfAddr=0x%X, ret=%d\n", romType, sfAddr, ret);
        }
    }
    flag = TRUE;
    printf("read mode:%d,%d\n", mode, data_len);
    if (0 == mode && (addr >= SF_ADDR_START && addr < SF_ADDR_END)) {
        if (!SonixCam_SerialFlashRead(addr, data, data_len)) {
            printf("read the serial flash error, sfAddr=0x%X\n", addr);
        }
    } else if (1 == mode && (addr >= SF_ADDR_START && addr < SF_ADDR_END)) {
        SERIAL_FLASH_TYPE sft;

        ret = SonixCam_GetSerialFlashType(&sft);
        if (sft == SFT_UNKNOW) {
            // sft = SFT_MXIC;
            printf("warning: the flash is SFT_UNKNOW\n");
        } else {
            printf("FlashType=%d, ret=%d\n", sft, ret);
        }
        if (!SonixCam_SerialFlashSectorWrite(addr, data, data_len, SFT_MXIC)) {  // test SFT_MAIX
            printf("write the serial flash error, sfAddr=0x%X\n", addr);
        }
    }
    return 0;
}

int32_t sr_serial_num_get(void) {
    unsigned char sn[100] = {0};

    if (!SonixCam_GetSerialNumber(sn, sizeof(sn))) {
        printf("get serial num error\n");
        return -1;
    }
    printf("sn: %s\n", (char *)sn);
    return 0;
}

}
//#ifdef __cplusplus
//}
//#endif