
#include "V4L2OptDev.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <zconf.h>

namespace usb{

typedef struct{
        BYTE bLength;
        BYTE bDescriptorType;
        USHORT bcdUSB;
        BYTE bDeviceClass;
        BYTE bDeviceSubClass;
        BYTE bDeviceProtocol;
        BYTE bMaxPacketSize0;
        USHORT idVendor;
        USHORT idProduct;
        USHORT bcdDevice;
        BYTE iManufacturer;
        BYTE iProduct;
        BYTE iSerialNumber;
        BYTE bNumConfigurations;
} usb_device_descriptor;

int g_iCameraCaptureFD = -1;

BOOL V4L2_EnumDevice(char *vidpid, ULONG *ulvidpid, BYTE *chipID, DSP_ARCH_TYPE *dspArchTYpe, USHORT *unitID)
{
	BOOL ret = FALSE;
	LONG i = 0;
	char deviceName[100];
	usb_device_descriptor devDes;
	for(i = 0; i < MAX_DEVICE_NUM; i++)
	{
		struct v4l2_capability cap;
		memset(deviceName, 0, sizeof(deviceName));
		sprintf(deviceName, "/dev/video%d", (int)i);
		//printf(deviceName);
		printf("\n");
		if((g_iCameraCaptureFD = open(deviceName, O_RDWR)) < 0)
		{
			printf("Fail to open device\n");
			continue;
		} 

		if(read(g_iCameraCaptureFD, (void *)(&devDes), sizeof(devDes)) > 0)
		{
			memset(deviceName, 0, sizeof(deviceName));
			sprintf(deviceName, "/dev/video%d vid: %02x pid: %02x \n", (int)i, devDes.idVendor, devDes.idProduct);
            //printf(deviceName);
		}
	
		memset(&cap,0,sizeof cap);
		if(ioctl(g_iCameraCaptureFD,VIDIOC_QUERYCAP,&cap)<0)
		{
			printf("Error opening device /dev/video0 : unable to query device.\n");
			close(g_iCameraCaptureFD);
			continue;
		}

		if(DRT_Unknow == V4L2_GetDspRomType(chipID, dspArchTYpe, unitID))
			return FALSE;
		
		ret = TRUE;
		break;
	}

	
	return TRUE;
}

BOOL V4L2_CloseDevice()
{
	if(g_iCameraCaptureFD > 0)
		close(g_iCameraCaptureFD);
	else
		return FALSE;
	return TRUE;
}

int XU_Set_Cur(__u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data)
{
	int err=0;
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
	struct uvc_xu_control_query xctrl;
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.query = UVC_SET_CUR;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(g_iCameraCaptureFD, UVCIOC_CTRL_QUERY, &xctrl);
#else
	struct uvc_xu_control xctrl;	
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(g_iCameraCaptureFD, UVCIOC_CTRL_SET, &xctrl);
#endif		
	return err;
}

int XU_Get_Cur(__u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data)
{
	int err=0;
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
	struct uvc_xu_control_query xctrl;
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.query = UVC_GET_CUR;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(g_iCameraCaptureFD, UVCIOC_CTRL_QUERY, &xctrl);
#else
	struct uvc_xu_control xctrl;	
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(g_iCameraCaptureFD, UVCIOC_CTRL_GET, &xctrl);
#endif	
	return err;
}

BOOL V4L2_ReadFormAsic(USHORT addr, BYTE *pValue)
{
	int ret = 0;
	__u8 ctrldata[4];

	//uvc_xu_control parmeters
	__u8 xu_unit= 3; 
	__u8 xu_selector= XU_SONIX_SYS_ASIC_RW;
	__u16 xu_size= 4;
	__u8 *xu_data= ctrldata;

	xu_data[0] = (addr & 0xFF);
	xu_data[1] = ((addr >> 8) & 0xFF);
	xu_data[2] = 0x0;
	xu_data[3] = 0xFF;		/* Dummy Write */
	
	/* Dummy Write */
	if ((ret=XU_Set_Cur(xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		printf("xu demmy write fail\n");
		return ret;
	}
	/* Asic Read */
	xu_data[3] = 0x00;
	if ((ret=XU_Get_Cur(xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		printf("xu asic read fail\n");
		printf("Fail to open device\n");
		return ret;
	}
	*pValue = xu_data[2];
	return TRUE;
}

DSP_ROM_TYPE V4L2_GetDspRomType(BYTE *chipID, DSP_ARCH_TYPE *dspArchType, USHORT *dspUnitID)
{
	DSP_ROM_TYPE drt = DRT_Unknow;
	DSP_ARCH_TYPE dspArchIndex = DAT_UNKNOW;
	USHORT unitId = 0x400;
	BYTE idIndex = 0;
	BYTE chipId = 0;
	for (idIndex = 0; idIndex < DSP_ARCH_COUNT; idIndex++)
	{
		V4L2_GetChipID(g_DspArchInfo[idIndex].dspIdAddr, &chipId);
		switch (chipId)
		{
		case 0x15:
		case 0x16:
		case 0x22:
		case 0x23:
		case 0x25:
		case 0x32:
		case 0x33:
		case 0x56:
			unitId = 0x400;
			drt = DRT_64K;
			dspArchIndex = DAT_FIRST;
			break;
		case 0x70:
		case 0x71:
		case 0x75:
		case 0x88:
		case 0x90:
			unitId = 0x300;
			drt = DRT_64K;
			dspArchIndex = DAT_FIRST;
			break;
		case 0x85:
			unitId = 0x300;
			dspArchIndex = DAT_SECOND;
			drt = DRT_128K;
			break;
		case 0x76:
		case 0x83:
		case 0x87:
		case 0x92:
			unitId = 0x300;
			drt = DRT_128K;
			dspArchIndex = DAT_FIRST;
			break;
		case 0x67:
			unitId = 0x300;
			drt = DRT_32K;
			dspArchIndex = DAT_FIRST;
			break;
		default:
			break;
		}
		if(dspArchIndex != DAT_UNKNOW)
			break;
	}
	*chipID = chipId;
	*dspArchType = dspArchIndex;
	*dspUnitID = unitId;
	return drt;
}

BOOL V4L2_GetChipID(USHORT addr, BYTE *chipID)
{
	if(TRUE != V4L2_ReadFormAsic(addr, chipID))
		return FALSE;

	return TRUE;
}

}

