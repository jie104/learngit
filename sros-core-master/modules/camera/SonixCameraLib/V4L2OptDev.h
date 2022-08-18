#ifndef __V4L2OPTDEV_H__
#define __V4L2OPTDEV_H__

#include "libusb/libusb.h"
#include "util.h"
#include "ROMData.h"
#include <linux/videodev2.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
#include <linux/uvcvideo.h>
#endif
namespace usb{

/*
 * Dynamic controls
 */
//copy from uvcvideo.h
#define UVC_CTRL_DATA_TYPE_RAW		0
#define UVC_CTRL_DATA_TYPE_SIGNED	1
#define UVC_CTRL_DATA_TYPE_UNSIGNED	2
#define UVC_CTRL_DATA_TYPE_BOOLEAN	3
#define UVC_CTRL_DATA_TYPE_ENUM		4
#define UVC_CTRL_DATA_TYPE_BITMASK	5

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

#define V4L2_CID_BASE_EXTCTR_SONIX				0x0A0c4501
#define V4L2_CID_BASE_SONIX						V4L2_CID_BASE_EXTCTR_SONIX
#define V4L2_CID_ASIC_RW_SONIX					V4L2_CID_BASE_SONIX+1
#define V4L2_CID_FLASH_CTRL					    V4L2_CID_BASE_SONIX+2
#define V4L2_CID_I2C_CTRL					    V4L2_CID_BASE_SONIX+3

/* ---------------------------------------------------------------------------- */
#define UVC_GUID_SONIX_SYS_HW_CTRL_292	{0x94,0x73,0xDF,0xDD,0x3E,0x97,0x27,0x47,0xBE,0xD9,0x04,0xED,0x64,0x26,0xDC,0x67}  //292
#define UVC_GUID_SONIX_SYS_HW_CTRL_283	{0x28,0xf0,0x33,0x70,0x63,0x11,0x4a,0x2e,0xba,0x2c,0x68,0x90,0xeb,0x33,0x40,0x16}  //283
#define UVC_GUID_SONIX_SYS_HW_CTRL_259	{0x28,0xf0,0x33,0x70,0x63,0x11,0x4a,0x2e,0xba,0x2c,0x68,0x90,0xeb,0x33,0x40,0x16}  //259A
#define XU_SONIX_SYS_ID							0x03
#define XU_SONIX_USR_ID							0x03
// ----------------------------- XU Control Selector ------------------------------------

#define XU_SONIX_SYS_ASIC_RW	      			0x01
#define XU_SONIX_SYS_I2C_RW	      			0x02
#define XU_SONIX_SYS_FLASH_CTRL				0x03

// ----------------------------- XU Control Selector ------------------------------------
//copy from uvcvideo.h
#define UVC_CONTROL_SET_CUR	(1 << 0)
#define UVC_CONTROL_GET_CUR	(1 << 1)
#define UVC_CONTROL_GET_MIN	(1 << 2)
#define UVC_CONTROL_GET_MAX	(1 << 3)
#define UVC_CONTROL_GET_RES	(1 << 4)
#define UVC_CONTROL_GET_DEF	(1 << 5)
/* Control should be saved at suspend and restored at resume. */
#define UVC_CONTROL_RESTORE	(1 << 6)
/* Control can be updated by the camera. */
#define UVC_CONTROL_AUTO_UPDATE	(1 << 7)

#define UVC_CONTROL_GET_RANGE   (UVC_CONTROL_GET_CUR | UVC_CONTROL_GET_MIN | \
                                 UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | \
                                 UVC_CONTROL_GET_DEF)

struct uvc_xu_control_info {
	__u8 entity[16];
	__u8 index;
	__u8 selector;
	__u16 size;
	__u32 flags;
};

#if LINUX_VERSION_CODE <= KERNEL_VERSION (3, 0, 36)
struct uvc_xu_control_mapping {
	__u32 id;
	__u8 name[32];
	__u8 entity[16];
	__u8 selector;

	__u8 size;
	__u8 offset;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,36) || LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,35)
	__u32 v4l2_type;
	__u32 data_type;

	struct uvc_menu_info __user *menu_info;
	__u32 menu_count;

	__u32 reserved[4];
#else    
	enum v4l2_ctrl_type v4l2_type;
	__u32 data_type;
#endif    
};
#endif

struct uvc_xu_control {
	__u8 unit;
	__u8 selector;
	__u16 size;
	__u8 *data;
};

typedef enum{
	CHIP_NONE = -1,
	CHIP_SNC291A = 0,
	CHIP_SNC291B,
	CHIP_SNC292A,
	CHIP_SNC259A,
	CHIP_SNC283
}CHIP_SNC29X;

#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
#define UVC_SET_CUR					0x01
#define UVC_GET_CUR					0x81
#define UVCIOC_CTRL_MAP		_IOWR('u', 0x20, struct uvc_xu_control_mapping)
#define UVCIOC_CTRL_QUERY	_IOWR('u', 0x21, struct uvc_xu_control_query)
#else
#define UVCIOC_CTRL_ADD		_IOW('U', 1, struct uvc_xu_control_info)
#define UVCIOC_CTRL_MAP		_IOWR('U', 2, struct uvc_xu_control_mapping)
#define UVCIOC_CTRL_GET		_IOWR('U', 3, struct uvc_xu_control)
#define UVCIOC_CTRL_SET		_IOW('U', 4, struct uvc_xu_control)
#endif

#define MAX_DEVICE_NUM 10

extern int CameraCaptureFD;

BOOL V4L2_EnumDevice(char *vidpid, ULONG *ulvidpid, BYTE *chipID, DSP_ARCH_TYPE *dspArchTYpe, USHORT *unitID);
BOOL V4L2_CloseDevice();

DSP_ROM_TYPE V4L2_GetDspRomType(BYTE *chipID, DSP_ARCH_TYPE *dspArchType, USHORT *dspUnitID);

BOOL V4L2_GetChipID(USHORT addr, BYTE *dspID);
BOOL V4L2_ReadFormAsic(USHORT addr, BYTE *pValue);
BOOL V4L2_WriteToAsic(USHORT addr, BYTE value);

}



#endif





