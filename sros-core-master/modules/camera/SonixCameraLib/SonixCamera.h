#ifndef __SONIXCAMERA_H__
#define __SONIXCAMERA_H__
#include "util.h"

//#ifdef __cplusplus
//extern "C" {
//#endif
namespace usb{

// lib init
sonix_bool SonixCam_Init(char *vidpid, int first_usb_id, int *find_id, int *dev_number);

//SonixCam_UnInit函数里面会调用CoUninitialize函数
sonix_bool SonixCam_UnInit();

//reset device
sonix_bool SonixCam_RestartDevice();

//error code
ERROR_CODE SonixCam_GetErrorCode();

sonix_bool SonixCam_Test();


// dsp register read 
sonix_bool SonixCam_AsicRegisterRead(unsigned short addr, unsigned char pData[], long len);

// * dsp register write
sonix_bool SonixCam_AsicRegisterWrite(unsigned short addr, unsigned char pData[], long len);

//sensor register read
sonix_bool SonixCam_SensorRegisterRead(unsigned char slaveId, unsigned short addr, unsigned char pData[], long len);

sonix_bool SonixCam_SensorRegisterCustomRead(unsigned char slaveId, long addr, unsigned char pData[], long len);

//* sensor register write
sonix_bool SonixCam_SensorRegisterWrite(unsigned char slaveId, unsigned short addr, unsigned char pData[], long len);

sonix_bool SonixCam_SensorRegisterCustomWrite(unsigned char slaveId, long addr, unsigned char pData[], long len);


//serial flash read
sonix_bool SonixCam_SerialFlashRead(long addr, unsigned char pData[], long len);

sonix_bool SonixCam_SerialFlashCustomRead(long addr, unsigned char pData[], long len);

//* serial flash write
sonix_bool SonixCam_SerialFlashSectorWrite(long addr, unsigned char pData[], long len, SERIAL_FLASH_TYPE sft);

sonix_bool SonixCam_SerialFlashSectorCustomWrite(long addr, unsigned char pData[], long len, SERIAL_FLASH_TYPE sft);

// get information	
sonix_bool SonixCam_GetSerialFlashType(SERIAL_FLASH_TYPE *sft);

sonix_bool SonixCam_GetFwVersion(unsigned char pData[], long len);

sonix_bool SonixCam_GetManufacturer(unsigned char pData[], long len);

sonix_bool SonixCam_GetProduct(unsigned char pData[], long len);

sonix_bool SonixCam_GetVidPid(unsigned char pData[], long len);

sonix_bool SonixCam_GetSerialNumber(unsigned char pData[], long len);

// * set information
//BOOL  SonixCam_SetVidPid(long addr, unsigned char pData[], long len);
//BOOL  SonixCam_SetSerialNumber(long addr, unsigned char pData[], long len);

typedef void(*SonixCam_SetProgress)(void *ptrClass, float fProcess);

// get dsp rom type
sonix_bool SonixCam_GetAsicRomType(DSP_ROM_TYPE *romType);

// * burner fw
sonix_bool SonixCam_BurnerFW(unsigned char pFwBuffer[], LONG lFwLength, SonixCam_SetProgress setProgress, void *ptrClass,
                       SERIAL_FLASH_TYPE sft);

// export fw
sonix_bool SonixCam_ExportFW(unsigned char pFwBuffer[], LONG lFwLength, SonixCam_SetProgress setProgress, void *ptrClass);

//extend burner fw
typedef struct {
    unsigned int SerialNumberLength;
    unsigned int ProductLength;
    unsigned int ManufactureLength;
    unsigned int FWVersionLength;
    unsigned int VidPidLength;
    unsigned int InterfaceLength;
    unsigned int String3Length;
    char *pSerialNumber;
    char *pProduct;
    char *pManufacture;
    char *pFWVersion;
    char *pVidPid;
    char *pString3;
    char *pInterface;
} ChangeParamInfo;

sonix_bool SonixCam_SetParamTableFormFWFile(unsigned char pFW[], long lFwLength, const ChangeParamInfo *pParamInfo,
                                      SonixCam_SetProgress setProgress, void *ptrClass, SERIAL_FLASH_TYPE sft,
                                      char *pLogFilePath);

//#ifdef __cplusplus
}
//#endif

#endif
