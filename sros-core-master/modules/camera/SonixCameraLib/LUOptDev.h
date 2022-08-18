#ifndef __LUOPTDEV_H__
#define __LUOPTDEV_H__

#include "libusb/libusb.h"
#include "util.h"
#include "ROMData.h"

#define USB_RQ			0x85
#define CTRL_IN			0xa1
#define CTRL_OUT		0x21
#define ASIC_RW			0x0100
#define I2C_RW			0x0200
#define FLASH_RW		0x0300
#define ROM_RW 			0x0400

#define		MAX_CAM_NUM		8
namespace usb{

BOOL VidPidStringToInt(char *vidpid, USHORT *vid, USHORT *pid);

BOOL LibUsb_EnumDevice(char *vidpid, int first_usb_id, int *find_id, int *dev_number);
BOOL LibUsb_CloseDevice();
BOOL LibUsb_RestartDevice();

BOOL LibUsb_TEST_READ();

/*
BOOL LibUsb_IsValidID(struct usb_device *pDev);
BOOL LibUsb_BusFindCam(struct usb_device *pDev, int level);
BOOL LibUsb_IsSonixDevice(libusb_device *dev);
BOOL LibUsb_EnumCamera();
*/

BOOL LibUsb_GetChipID(LONG idAddr, BYTE *pChipID);
DSP_ROM_TYPE LibUsb_GetChipRomType(BYTE *chipID, DSP_ARCH_TYPE *dspArchType);

BOOL LibUsb_EnableAsicRegisterBit(LONG addr, BYTE bit);
BOOL LibUsb_DisableAsicRegisterBit(LONG addr, BYTE bit);
BOOL LibUsb_SFWaitReady();

BOOL LibUsb_ReadFromASIC(USHORT addr, BYTE *pValue);
BOOL LibUsb_WriteToASIC(USHORT addr, BYTE value);

BOOL LibUsb_ReadFromROM(LONG addr, BYTE data[]);

BOOL LibUsb_ReadFromSensor(BYTE slaveID, USHORT addr, BYTE pData[], LONG len);
BOOL LibUsb_WriteToSensor(BYTE slaveID, USHORT addr, BYTE pData[], LONG len);
BOOL LibUsb_CustomReadFromSensor(BYTE slaveID, USHORT addr, BYTE pData[], LONG len);
BOOL LibUsb_CustomWriteToSensor(BYTE slaveID, USHORT addr, BYTE pData[], LONG len);

BOOL LibUsb_GetSerialFlashType(SERIAL_FLASH_TYPE *sft);

BOOL LibUsb_ReadFormSF(LONG addr, BYTE pData[], LONG len);
BOOL LibUsb_WriteToSF(LONG addr, BYTE pData[], LONG le, SERIAL_FLASH_TYPE sft);

BOOL LibUsb_ReadDataFormFlash(LONG addr, BYTE pData[], BYTE dataLen);
BOOL LibUsb_WriteDataToFlash(LONG addr, BYTE pData[], BYTE dataLen);

BOOL LibUsb_DisableSerialFlashWriteProtect(SERIAL_FLASH_TYPE sft);
BOOL LibUsb_EraseSectorForSerialFlash(LONG addr, SERIAL_FLASH_TYPE sft);

BOOL LibUsb_SerialFlashErase(SERIAL_FLASH_TYPE sft);
BOOL LibUsb_DefGetAsicRomVersion(BYTE romVersion[]);
BOOL LibUsb_GetAsicRomVersion(BYTE romVersion[]);
BOOL LibUsb_DefGetAsicRomVersion(BYTE data[]);
BOOL LibUsb_SFCMDreadStatus();

BOOL LibUsb_GetParaTableAndCRCAddrFormSF(ULONG *paraTableStartAddr, ULONG *paraTableEndAddr, ULONG *crcAddr);
BOOL LibUsb_GetStringSettingFormSF(BYTE* pbyString, DWORD stringSize, DWORD StringOffset, BOOL bIsCRCProtect);

}
#endif





