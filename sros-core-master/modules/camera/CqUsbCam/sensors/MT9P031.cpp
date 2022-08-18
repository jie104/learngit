/****************************************************************************************\
 * This is the file for the Caiqi usb camera for Linux/Mac						*
 *                                                                                	*
 * Author              :        alan     											* 												
 * License             :        GPL Ver 2.0                                       	*
 * Copyright           :        Caiqi Electronics Inc.								*
 * Date written        :        Jan 24 2021                                   	*
 * Modification Notes  :                                                          	*
 *    1. alan, Jan 24 2021                                 						*
 *       Add documentation.                                          				*
 *                      															*
 *                                                                                	*
 \***************************************************************************************/

#include <string.h>

#include "MT9P031.h"
#include "../SensorCapbablity.h"



static cq_int32_t MT9P031_InitSensor(cyusb_handle *pUsbHandle)
{
	cq_uint8_t data[10]={'0'};//no use, just to make firmware happy
	cq_uint8_t tempAddr= 0;
	cq_uint16_t tempValue= 0;
    //return cyusb_control_write(pUsbHandle, 0x40, 0xf0, tempValue, tempAddr, data, 2, 100);
	return 0;
}
static cq_int32_t MT9P031_WrSensorReg(cyusb_handle *pUsbHandle, const cq_uint32_t iAddr, const cq_uint32_t iValue)
{
	cq_uint8_t data[10]={'0'};//no use, just to make firmware happy
	cq_uint8_t tempAddr= iAddr&0xff;
	cq_uint16_t tempValue= iValue&0xffff;
    return cyusb_control_write(pUsbHandle, 0x40, 0xf1, tempValue, tempAddr, data, 2, 100);
}


static cq_int32_t MT9P031_RdSensorReg(cyusb_handle *pUsbHandle, const cq_uint32_t iAddr, cq_uint32_t &iValue)
{
	cq_uint8_t tempAddr= iAddr&0xff;
	cq_uint8_t tempValue[2]={'0'};
	cq_int32_t r=cyusb_control_read(pUsbHandle, 0x40, 0xf2,0x0, tempAddr, tempValue, 2, 100);
	iValue = tempValue[0] << 8;
	iValue += tempValue[1];
	return r;
}


#if 0
static cq_int32_t MT9P031_WrDevID(cyusb_handle *pUsbHandle, cq_uint8_t* chIdBuf, cq_uint32_t &length )
{

}
static cq_int32_t MT9P031_RdDevID(cyusb_handle *pUsbHandle, cq_uint8_t *chIdBuf, cq_uint32_t &length)
{

}

static cq_int32_t MT9P031_WrDevSN(cyusb_handle *pUsbHandle, cq_uint8_t* chSnBuf, cq_uint32_t &length )
{

}
static cq_int32_t MT9P031_RdDevSN(cyusb_handle *pUsbHandle, cq_uint8_t *chSnBuf, cq_uint32_t &length)
{

}
#endif

static cq_int32_t MT9P031_WrFpgaReg(cyusb_handle *pUsbHandle, const cq_uint32_t iAddr, const cq_uint32_t iValue)
{
	cq_uint8_t data[10]={'0'};//no use, just to make firmware happy
	cq_uint8_t tempAddr= iAddr&0xff;
	cq_uint8_t tempValue= iValue&0xff;
    return cyusb_control_write(pUsbHandle, 0x40, 0xf3, tempValue, tempAddr, data, 1, 100);
}


static cq_int32_t MT9P031_RdFpgaReg(cyusb_handle *pUsbHandle, const cq_uint32_t iAddr, cq_uint32_t &iValue)
{
cq_uint8_t tempAddr= iAddr&0xff;
	cq_uint8_t tempValue[1]={'0'};
    cq_int32_t r=cyusb_control_read(pUsbHandle, 0x40, 0xf4, 0x0, tempAddr, tempValue, 1, 100);
	iValue=tempValue[0];
	return r;
}

static cq_int32_t MT9P031_WrEeprom(cyusb_handle *pUsbHandle, const cq_uint32_t iAddr, const cq_uint8_t iValue)
{
	cq_int32_t len=2;
	cq_uint8_t data[10]={'0'};//no use, just to make firmware happy
	cq_uint8_t tempAddr= iAddr&0xff;
	cq_uint8_t tempValue= iValue&0xff;
    
	return cyusb_control_write(pUsbHandle, 0x40, 0xf5, tempValue, tempAddr, data, 1, 100);
}
static cq_int32_t MT9P031_RdEeprom(cyusb_handle *pUsbHandle, const cq_uint32_t iAddr, cq_uint8_t * buffer, cq_int32_t &length)
{
	cq_int32_t transferred=length;
	cq_uint16_t tempAddr= iAddr&0xffff;
    cq_int32_t r=cyusb_control_read(pUsbHandle,0x40,0xf6,0x0, tempAddr, buffer, transferred,100);
	return r;
}
/*
static cq_int32_t MT9P031_InitSensor(cyusb_handle *pUsbHandle)
{    
 
    
    return 0;
}

*/
/*
static cq_int32_t MT9P031_SetAnalogGain(cyusb_handle *pUsbHandle, const cq_uint32_t chTrigType, const cq_uint32_t chGainType)
{
	cq_int32_t r=0;
	switch(chTrigType) 
	{   
		case TRIGMODE_AUTO:
		{
			switch(chGainType)
			{
				case ANALOGGAIN_1X:
					r=MT9P031_WrSensorReg(pUsbHandle, 0x30B0,0x0080);
					break;
				case ANALOGGAIN_2X:
					r=MT9P031_WrSensorReg(pUsbHandle, 0x30B0,0x0090);
					break;
				case ANALOGGAIN_4X:
					r=MT9P031_WrSensorReg(pUsbHandle, 0x30B0,0x00A0);
					break;
				case ANALOGGAIN_8X:
					r=MT9P031_WrSensorReg(pUsbHandle, 0x30B0,0x00B0);
					break;
				default:
					r=-99;
					break;
			}
			break;	
		}
		case TRIGMODE_FPGA:
		{
			switch(chGainType)
			{
				case ANALOGGAIN_1X:
					r=MT9P031_WrSensorReg(pUsbHandle, 0x30B0,0x0480);
					break;
				case ANALOGGAIN_2X:
					r=MT9P031_WrSensorReg(pUsbHandle, 0x30B0,0x0490);
					break;
				case ANALOGGAIN_4X:
					r=MT9P031_WrSensorReg(pUsbHandle, 0x30B0,0x04A0);
					break;
				case ANALOGGAIN_8X:
					r=MT9P031_WrSensorReg(pUsbHandle, 0x30B0,0x04B0);
					break;
				default:
					r=-99;
					break;	
			}
			break;
		}	
		default:
			r=-99;
			break;	
	}
	return r;
}
*/

static cq_int32_t MT9P031_SetFpgaTrigFreq(cyusb_handle *pUsbHandle, const cq_uint32_t iFreqVal)
{
     cq_int32_t r=MT9P031_WrFpgaReg(pUsbHandle, 0x05, iFreqVal&0xff);
     return r;
}

static cq_int32_t MT9P031_SetTrigMode(cyusb_handle *pUsbHandle, const cq_uint32_t chTrigType)
{
	switch(chTrigType)
	{
		case TRIGMODE_AUTO:
			MT9P031_WrSensorReg(pUsbHandle, 0x1e, 0x4006);
			MT9P031_WrFpgaReg(pUsbHandle, 0x00, 0x00);
			break;
		case TRIGMODE_FPGA:
			MT9P031_WrSensorReg(pUsbHandle, 0x1e, 0x4106);
			MT9P031_WrFpgaReg(pUsbHandle, 0x00, 0x01);
			MT9P031_WrFpgaReg(pUsbHandle, 0x05, 0x04);// 0x01 by default
			break;
		case TRIGMODE_SOFT:
			MT9P031_WrSensorReg(pUsbHandle, 0x1e, 0x4106);
			
			MT9P031_WrFpgaReg(pUsbHandle, 0x00, 0x02);
			break;
		case TRIGMODE_SIGNAL:
			MT9P031_WrSensorReg(pUsbHandle, 0x1e, 0x4106);
			MT9P031_WrFpgaReg(pUsbHandle, 0x00, 0x03);
		default:
			break;
	}
	return 0;

}
static cq_int32_t MT9P031_SoftTrig(cyusb_handle* pUsbHandle)
{	 
	  MT9P031_WrFpgaReg(pUsbHandle, 0x0a, 0x00);
	  MT9P031_WrFpgaReg(pUsbHandle, 0x0a, 0x01);
	  return 0;

}

static cq_int32_t MT9P031_SetExpoValue(cyusb_handle *pUsbHandle, const cq_uint32_t iExpoVal)
{
	cq_int32_t r=MT9P031_WrSensorReg(pUsbHandle, 0x09, iExpoVal&0xffff);
	return r;
}

static cq_int32_t MT9P031_SetGainValue(cyusb_handle *pUsbHandle, const cq_uint32_t iGainVal)
{
    cq_int32_t r=MT9P031_WrSensorReg(pUsbHandle, 0x35, iGainVal&0xffff);
    return r;
}

static cq_int32_t MT9P031_SetAutoGainExpo(cyusb_handle *pUsbHandle, const cq_bool_t bIsAutoGain, const cq_bool_t bIsAutoExpo)
{
	return 1;
}

static cq_int32_t MT9P031_SetResolution(cyusb_handle *pUsbHandle, const cq_uint32_t chResoluType)
{
	//int mode = (1 << 31) & chResoluType;
	//if (mode == 0)
	{
		//2592,1944
		//cq_uint32_t setWidth_H = (chResoluType & 0xFF000000) >> 24;//0A
		//cq_uint32_t setWidth_L = (chResoluType & 0x00FF0000) >> 16;//20
		//cq_uint32_t setHight_H = (chResoluType & 0x0000FF00) >> 8;//07
		//cq_uint32_t setHight_L = (chResoluType & 0x000000FF);//98
		/*
		MT9P031_WrSensorReg(pUsbHandle, 0x01, 0x1);
		MT9P031_WrSensorReg(pUsbHandle, 0x02, 0x4);
		MT9P031_WrSensorReg(pUsbHandle, 0x03, );
		MT9P031_WrSensorReg(pUsbHandle, 0x04, );
	
		MT9P031_WrFpgaReg(pUsbHandle, 0x01, 0x0A);
		MT9P031_WrFpgaReg(pUsbHandle, 0x02, 0x20);
		MT9P031_WrFpgaReg(pUsbHandle, 0x03, 0x07);
		MT9P031_WrFpgaReg(pUsbHandle, 0x04, 0x98);
		*/
	}
	return 0;
		

}


static cq_int32_t MT9P031_SetMirrorType(cyusb_handle *pUsbHandle, const cq_uint32_t chMirrorType)
{
	if(MIRROR_NORMAL==chMirrorType)	//normal
		
		return MT9P031_WrSensorReg(pUsbHandle, 0x20, 0x0040);

	if(MIRROR_X==chMirrorType)//X
		
		return MT9P031_WrSensorReg(pUsbHandle, 0x20, 0x6040);

	if(MIRROR_Y==chMirrorType)//Y
		
		return MT9P031_WrSensorReg(pUsbHandle, 0x20, 0x8040);

	if(MIRROR_XY==chMirrorType)//XY
		
		return MT9P031_WrSensorReg(pUsbHandle, 0x20, 0xC040);

	return -99;//should never reach here

}


static cq_int32_t MT9P031_SetBitDepth(cyusb_handle *pUsbHandle, const cq_uint32_t chBitDepthType)
{
	if(chBitDepthType==0||chBitDepthType==1)
	{
		return MT9P031_WrFpgaReg(pUsbHandle, 0x7, chBitDepthType);
	}
	return 0;
}


static cq_int32_t MT9P031_StartCap(cyusb_handle *pUsbHandle)
{
	MT9P031_WrFpgaReg(pUsbHandle, 0x09, 0x00);
	MT9P031_WrFpgaReg(pUsbHandle, 0x09, 0x01);
	return 0;	
}

static cq_int32_t MT9P031_StopCap(cyusb_handle *pUsbHandle)
{
	MT9P031_WrFpgaReg(pUsbHandle, 0x09, 0x00);
	return 0;
}

static cq_int32_t MT9P031_SendUsbSpeed2Fpga(cyusb_handle *pUsbHandle, const cq_uint32_t chSpeedType)
{
	MT9P031_WrFpgaReg(pUsbHandle, 0x08, chSpeedType);
	return 0;
}


/*
static tagSensor sensor_MT9P031=
{
	.name="MT9P031",
	.iicAddr=0xf0,
	.regAddrLen=1,
	.regValueLen=2,

	.InitSensor		=	NULL,
	.WrSensorReg	=	MT9P031_WrSensorReg,
	.RdSensorReg	=	MT9P031_RdSensorReg,
	.WrFpgaReg		=	MT9P031_WrFpgaReg,
	.RdFpgaReg		=	MT9P031_RdFpgaReg,
	.SetAnalogGain	=	NULL,
	.SetFpgaTrigFreq=	MT9P031_SetFpgaTrigFreq,
	.SetTrigMode	=	MT9P031_SetTrigMode,
	.SetExpoValue	=	MT9P031_SetExpoValue,
	.SetGainValue	=	MT9P031_SetGainValue,
	.SetAutoGainExpo=	MT9P031_SetAutoGainExpo,
	.SetResolution	=	MT9P031_SetResolution,
	.SetMirrorType	=	MT9P031_SetMirrorType,
	.SetBitDepth	=	NULL,
	.StartCap 		=	MT9P031_StartCap,
	.StopCap 		=	MT9P031_StopCap,
	.SendUsbSpeed2Fpga=	MT9P031_SendUsbSpeed2Fpga,

	.WrEeprom		=	MT9P031_WrEeprom,
	.RdEeprom		=	MT9P031_RdEeprom,
	.WrDevID		=	MT9P031_WrDevID,
	.RdDevID		=	MT9P031_RdDevID,
	.WrDevSN		=	MT9P031_WrDevSN,
	.RdDevSN		=	MT9P031_RdDevSN,
	.SoftTrig		=	MT9P031_SoftTrig
};
*/
static tagSensor sensor_MT9P031=
{
	"MT9P031",
	0xf0,
	1,
	2,

	MT9P031_InitSensor,
	MT9P031_WrSensorReg,
	MT9P031_RdSensorReg,
	MT9P031_WrFpgaReg,
	MT9P031_RdFpgaReg,
	NULL,
	MT9P031_SetFpgaTrigFreq,
	MT9P031_SetTrigMode,
	MT9P031_SetExpoValue,
	MT9P031_SetGainValue,
	MT9P031_SetAutoGainExpo,
	MT9P031_SetResolution,
	MT9P031_SetMirrorType,
	MT9P031_SetBitDepth,
	MT9P031_StartCap,
	MT9P031_StopCap,
	MT9P031_SendUsbSpeed2Fpga,

	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,

	MT9P031_SoftTrig
};
void RegisterSensor_MT9P031(list<tagSensor>& sensorList)
{
	sensorList.push_back(sensor_MT9P031);

}

