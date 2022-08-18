#ifndef __SR_SONIX_CAMERA_H__
#define __SR_SONIX_CAMERA_H__

#include <stdint.h>
//#ifdef __cplusplus
//extern "C" {
//#endif
namespace usb {


/**
 * 初始化摄像头
 *
 * @param[in]	None
 * @param[out]	None
 *
 * @return		0成功, < 0失败
 */
int32_t sr_sonix_cam_init(int first_search_id, int *founded_usb_id, int *dev_number);

/**
 * 关闭摄像头
 *
 * @param[in]	None
 * @param[out]	None
 *
 * @return		0成功, < 0失败
 */
int32_t sr_sonix_deinit(void);

/**
 * 读/写flash
 *
 * @param[in]	mode:读/写
 * @param[in]	addr:flash地址
 * @param[in]	data:数据
 * @param[in]	data_len:数据长度
 * @param[out]	None
 *
 * @return		0成功, < 0失败
 */
int32_t sr_serial_flash_rw(uint8_t mode, uint16_t addr, uint8_t *data, uint16_t data_len);

//#ifdef __cplusplus
//}
//#endif
}
#endif /* __SR_SONIX_CAMERA_H__ */ 