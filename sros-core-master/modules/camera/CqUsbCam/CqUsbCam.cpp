/****************************************************************************************\
 * This is the main file for the Caiqi usb camera for Linux/Mac						*
 *                                                                                	*
 * Author              :        nxb     											* 												
 * License             :        GPL Ver 2.0                                       	*
 * Copyright           :        Caiqi Electronics Inc.								*
 * Date written        :        Oct 12, 2017                                    	*
 * Modification Notes  :                                                          	*
 *    1. nxb, Oct 12, 2017                                  						*
 *       Add documentation.                                          				*
 *                      															*
 *                                                                                	*
 \***************************************************************************************/

#include "CqUsbCam.h"
#include "Types.h"
#include <assert.h>
#include "./sensors/AR0135.h"
#include "./sensors/MT9V034.h"
#include "./sensors/AR0144.h"
#include "./sensors/SC130GS.h"
#include "./sensors/MT9P031.h"



#define ERR_ITF_NOT_CLAIMED 		-0x80
#define ERR_ITF_IS_ALREADY_CLAIMED	-0x81
#define ERR_NULL_FUNC_POINTER		-0x82

#define ERR_IS_CAPTURING		-0x83
#define ERR_IS_NOT_CAPTURING	-0x84

CCqUsbCam::CCqUsbCam()
{
	m_pUsbHandle=NULL;

	m_pImgQueue=NULL;
	m_pDataCap=NULL;
	m_pDataProc=NULL;

	m_bIsInterfaceClaimed=false; 		
	m_bIsCapturing=false;

	RegisterSensor_AR0135(m_sensorList);
	RegisterSensor_MT9V034(m_sensorList);
	RegisterSensor_AR0144(m_sensorList);
	RegisterSensor_SC130GS(m_sensorList);
	RegisterSensor_MT9P031(m_sensorList);

}

cq_int32_t  CCqUsbCam::SelectSensor(string strSensorName)
{

	list<tagSensor>::iterator i;      

	if(0==m_sensorList.size())
		return -1;

	for (i = m_sensorList.begin(); i != m_sensorList.end(); ++i)   
	{
		if(strSensorName==(*i).name)
		{
			m_sensorInUse=(*i);
			return 0;
		}
	}
	return -2;
}

cq_int32_t CCqUsbCam::OpenUSB()
{

	cq_int32_t usbCnt=cyusb_open();

	if(usbCnt<0)
		return usbCnt;
	if(usbCnt==0)
	{
		cyusb_close();
		return usbCnt;
	}  

	return usbCnt;

}

cq_int32_t CCqUsbCam::CloseUSB()
{
	cyusb_close();
	return 0;  
}

cq_int32_t CCqUsbCam::ClaimInterface(const cq_int32_t usbNum)
{
	if(true == m_bIsInterfaceClaimed)
		return ERR_ITF_IS_ALREADY_CLAIMED;

	m_pUsbHandle=NULL;
	m_pUsbHandle=cyusb_gethandle(usbNum);
	if(m_pUsbHandle==NULL)
	{
		cyusb_close();
		return -3;
	}

	cq_int32_t r=cyusb_kernel_driver_active(m_pUsbHandle,0);
	if(r!=0)
	{
		cyusb_close();
		return -4;
	}

	r = cyusb_claim_interface(m_pUsbHandle, 0);
	if ( r != 0 )
	{
		cyusb_close();
		return -5;
	}

	m_bIsInterfaceClaimed=true;
	return 0;
}

cq_int32_t CCqUsbCam::ReleaseInterface()
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	StopCap();//in case StopCap has not been called

	cq_int32_t r = cyusb_release_interface(m_pUsbHandle, 0);
	if(r!=0)
		return -1;

	m_bIsInterfaceClaimed=false;
	return 0;  

}

cq_int32_t  CCqUsbCam::InitSensor()
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.InitSensor)
		return ERR_NULL_FUNC_POINTER;

	return m_sensorInUse.InitSensor(m_pUsbHandle);
}


cq_int32_t  CCqUsbCam::StartCap(const cq_uint32_t iHeight, const cq_uint32_t iWidth, callback_t CallBackFunc)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(true ==m_bIsCapturing)
		return ERR_IS_CAPTURING;
cq_uint32_t h,w,temp;
	RdFpgaReg(1, temp);
	h = temp << 8;
	RdFpgaReg(2, temp);
	h = temp + h;
	RdFpgaReg(3, temp);
	w = temp << 8;
	RdFpgaReg(4, temp);
	w = temp + w;
	m_pImgQueue=new wqueue<CImgFrame*>;
	m_pDataCap=new CDataCapture(w, h);
	m_pDataProc=new CDataProcess();

	assert(NULL!=m_pImgQueue);
	assert(NULL!=m_pDataCap);
	assert(NULL!=m_pDataProc);


	m_pDataCap->SetUsbHandle(m_pUsbHandle);
	m_pDataCap->SetImgQueue(m_pImgQueue);

	m_pDataProc->SetCallBackFunc(CallBackFunc);
	m_pDataProc->SetImgQueue(m_pImgQueue);


	m_pDataCap->Open();
	m_pDataProc->Open();

	if(NULL==m_sensorInUse.StartCap)
		return ERR_NULL_FUNC_POINTER;

	m_sensorInUse.StartCap(m_pUsbHandle);
	m_bIsCapturing=true;
	return 0;
}


cq_int32_t  CCqUsbCam::StopCap()
{
	if(true!=m_bIsCapturing)
		return ERR_IS_NOT_CAPTURING;

	m_pDataCap->Close();
	m_pDataProc->Close();

	if(NULL!=m_pImgQueue)	delete m_pImgQueue;
	if(NULL!=m_pDataCap)	delete m_pDataCap;
	if(NULL!=m_pDataProc)	delete m_pDataProc;

	if(NULL==m_sensorInUse.StopCap)
		return ERR_NULL_FUNC_POINTER;
	m_sensorInUse.StopCap(m_pUsbHandle);

	m_bIsCapturing=false;
	return 0;
}



cq_int32_t CCqUsbCam::GetUsbSpeed(cq_uint32_t &chSpeedType)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	cyusb_device *h_dev = NULL;
	h_dev=libusb_get_device(m_pUsbHandle);
	if(h_dev==NULL)
		return -1;
	chSpeedType=libusb_get_device_speed(h_dev);
	//unsigned char buf[100];
	//memset(buf,0,100);
	//int ret=libusb_get_descriptor(m_pUsbHandle,	/*uint8_t desc_type*/LIBUSB_DT_ENDPOINT, /*uint8_t desc_index*/0, /*unsigned char *data*/buf, /*int length*/100);
	//ret=libusb_get_descriptor(m_pUsbHandle,	/*uint8_t desc_type*/LIBUSB_DT_ENDPOINT, /*uint8_t desc_index*/1, /*unsigned char *data*/buf, /*int length*/100);
	//ret=libusb_get_descriptor(m_pUsbHandle,	/*uint8_t desc_type*/LIBUSB_DT_ENDPOINT, /*uint8_t desc_index*/2, /*unsigned char *data*/buf, /*int length*/100);

	return 0;
}


cq_int32_t CCqUsbCam::SetAnalogGain(const cq_uint32_t chTrigType, const cq_uint32_t chGainType)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetAnalogGain)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetAnalogGain(m_pUsbHandle, chTrigType, chGainType);
}

cq_int32_t CCqUsbCam::SetFpgaTrigFreq(const cq_uint32_t iFreqVal)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetFpgaTrigFreq)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetFpgaTrigFreq(m_pUsbHandle, iFreqVal);
}

cq_int32_t CCqUsbCam::SetTrigMode(const cq_uint32_t chTrigType)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetTrigMode)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetTrigMode(m_pUsbHandle, chTrigType);
}

cq_int32_t CCqUsbCam::SetExpoValue(const cq_uint32_t iExpoVal)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetExpoValue)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetExpoValue(m_pUsbHandle, iExpoVal);
}

cq_int32_t CCqUsbCam::SetGainValue(const cq_uint32_t iGainVal)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetGainValue)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetGainValue(m_pUsbHandle, iGainVal);
}

cq_int32_t CCqUsbCam::SetAutoGainExpo(const cq_bool_t bIsAutoGain, const cq_bool_t bIsAutoExpo)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetAutoGainExpo)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetAutoGainExpo(m_pUsbHandle, bIsAutoGain, bIsAutoExpo);
}

cq_int32_t CCqUsbCam::SetResolution(const cq_uint32_t chResoluType)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetResolution)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetResolution(m_pUsbHandle, chResoluType);
}

cq_int32_t CCqUsbCam::SetMirrorType(const cq_uint32_t chMirrorType)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetMirrorType)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetMirrorType(m_pUsbHandle, chMirrorType);
}

cq_int32_t CCqUsbCam::SetBitDepth(const cq_uint32_t chBitDepthType)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetBitDepth)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SetBitDepth(m_pUsbHandle, chBitDepthType);
}

cq_int32_t CCqUsbCam::SendUsbSpeed2Fpga(const cq_uint32_t chSpeedType)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SendUsbSpeed2Fpga)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.SendUsbSpeed2Fpga(m_pUsbHandle, chSpeedType);
}

cq_int32_t CCqUsbCam::WrSensorReg(const cq_uint32_t iAddr, const cq_uint32_t iValue)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.WrSensorReg)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.WrSensorReg(m_pUsbHandle, iAddr, iValue);
}

cq_int32_t CCqUsbCam::RdSensorReg(const cq_uint32_t iAddr, cq_uint32_t &iValue)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.RdSensorReg)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.RdSensorReg(m_pUsbHandle, iAddr, iValue);
}

cq_int32_t CCqUsbCam::WrFpgaReg(const cq_uint32_t 	iAddr, const cq_uint32_t iValue)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.WrFpgaReg)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.WrFpgaReg(m_pUsbHandle, iAddr, iValue);
}

cq_int32_t CCqUsbCam::RdFpgaReg(const cq_uint32_t 	iAddr, cq_uint32_t &iValue)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.RdFpgaReg)
		return ERR_NULL_FUNC_POINTER;
	return m_sensorInUse.RdFpgaReg(m_pUsbHandle, iAddr, iValue);
}
cq_int32_t CCqUsbCam::SoftTrigOnce()
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.SetMirrorType)
		return ERR_NULL_FUNC_POINTER;
		return m_sensorInUse.SoftTrig(m_pUsbHandle);

}
cq_int32_t CCqUsbCam::ArbFunc(void *p)
{
	if(false == m_bIsInterfaceClaimed)
		return ERR_ITF_NOT_CLAIMED;

	if(NULL==m_sensorInUse.ArbitrFunc)
		return ERR_NULL_FUNC_POINTER;
		return m_sensorInUse.ArbitrFunc(m_pUsbHandle,p);

}
void CCqUsbCam::GetRecvByteCnt(cq_uint64_t& byteCnt)
{
	float temp=0;
	temp=m_pDataCap->m_lRecvByteCnt*1.2;
	byteCnt=(cq_uint64_t)temp;
}
void CCqUsbCam::ClearRecvByteCnt()
{
	m_pDataCap->m_lRecvByteCnt=0;
}
void CCqUsbCam::GetRecvFrameCnt(cq_uint64_t& frameCnt)
{
	float temp=0;
	temp=m_pDataCap->m_lRecvFrameCnt*1.2;
	frameCnt=(cq_uint64_t)temp;

}
void CCqUsbCam::ClearRecvFrameCnt()
{
	m_pDataCap->m_lRecvFrameCnt=0;
}

cq_int32_t CCqUsbCam::WrEeprom(const cq_uint32_t iAddr, const uint8_t* buffer,cq_uint32_t length)
{
	int32_t ret = 0;
	uint32_t count = 0;

	if(length>32)
		length=32;

	cq_int32_t len=length;
	static cq_uint8_t data[32]={'0'};//no use, just to make firmware happy
	uint8_t check_data[32] = {0};

	memset(data, 0, 32);
	memcpy(data,buffer,len);
	cq_uint16_t tempAddr= (iAddr+0x100)&0xffff;
	cq_uint16_t tempValue= len&0xff;
	ret  = cyusb_control_write(m_pUsbHandle,0x40,0xf5,tempValue,tempAddr,data,len,1000);
	if(ret <= 0){
		printf("error, WrEeprom cyusb_control_write ret=%d\n", ret);
		return ret;
	}
	usleep(50000);
	ret = cyusb_control_read(m_pUsbHandle,0x40,0xf6,tempValue, tempAddr, check_data, len, 1000);
	if(ret <= 0){
		printf("error, WrEeprom cyusb_control_read ret=%d\n", ret);
		return ret;
	}

	while (count < len) {
		if (data[count] != check_data[count]){
			printf("error, data[%d]=0x%x, check_data[%d]=0x%x\n", count, data[count], count, check_data[count] );
			break;
		}
		count++;
	}
	return count;
}

cq_int32_t CCqUsbCam::RdEeprom(const cq_uint32_t iAddr, cq_uint8_t * buffer, cq_uint32_t &length)
{
	if(length>32)
		length=32;

	cq_int32_t transferred=length;
	cq_uint16_t tempAddr= (iAddr+0x100)&0xffff;
	cq_uint16_t tempValue= length&0xff;
    cq_int32_t r=cyusb_control_read(m_pUsbHandle,0x40,0xf6,tempValue, tempAddr, buffer, transferred,1000);
	return r;
}

int32_t CCqUsbCam::ResetDevice(const int32_t usbNum)
{
	int32_t ret = 0;
	cyusb_handle *usbHandle = NULL;

	usbHandle = cyusb_gethandle(usbNum);
	if(usbHandle){
		ret = cyusb_reset_device(usbHandle);
		if(ret != 0){
			cyusb_close();
			printf("error, cyusb_reset_device svc, ret=%d\n", ret);
		}
	}
	return 0;
}

int32_t CCqUsbCam::ResetFlag(void)
{
	m_bIsInterfaceClaimed = false;
	return 0;
}
