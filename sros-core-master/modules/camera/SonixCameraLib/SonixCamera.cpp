#include "SonixCamera.h"
#include "LUOptDev.h"
#include "util.h"
#include "V4L2OptDev.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <zconf.h>

//#ifdef __cplusplus
//extern "C" {
//#endif
namespace usb {

BOOL svc100_camera_init = FALSE;
extern ERROR_CODE svc100_gEC;
extern unsigned int svc100_uiRomID;

sonix_bool SonixCam_Init(char *vidpid, int first_usb_id, int *find_id, int *dev_number) {
    if (svc100_camera_init) return TRUE;

    if (libusb_init(NULL) < 0) {
        fprintf(stderr, "failed to initialise libusb\n");
        return FALSE;
    }

    if (TRUE != LibUsb_EnumDevice(vidpid, first_usb_id, find_id, dev_number)) {
        svc100_gEC = EC_NoFindDevice;
        return FALSE;
    }
    svc100_camera_init = TRUE;
    return TRUE;
}

sonix_bool SonixCam_UnInit() {
    if (!svc100_camera_init) return FALSE;

    LibUsb_CloseDevice();
    V4L2_CloseDevice();

    svc100_camera_init = FALSE;
    return TRUE;
}

sonix_bool SonixCam_RestartDevice() {
    if (!svc100_camera_init) return FALSE;

    return LibUsb_RestartDevice();
}

sonix_bool SonixCam_GetAsicRomType(DSP_ROM_TYPE *romType) {
    static sonix_bool flag = FALSE;

    if (!svc100_camera_init) {
        printf("svc100_camera_init false\n");
        return FALSE;
    }

    BYTE chipID;
    DSP_ARCH_TYPE dspArchType;
    *romType = LibUsb_GetChipRomType(&chipID, &dspArchType);
    if (!flag) {
        printf("LibUsb_GetChipID ,chipID=0x%x\n", chipID);
        flag = TRUE;
    }
    return TRUE;
}

ERROR_CODE SonixCam_GetErrorCode() {
    if (!svc100_camera_init) return EC_UnKnow;

    return svc100_gEC;
}

sonix_bool SonixCam_Test() { return LibUsb_TEST_READ(); }

sonix_bool SonixCam_AsicRegisterRead(unsigned short addr, unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    BYTE data = 0;
    USHORT startAddr = addr;
    LONG i = 0;
    for (i = 0; i < len; i++) {
        if (TRUE != LibUsb_ReadFromASIC((USHORT)startAddr++, &data)) return FALSE;
        memcpy(pData + i, &data, 1);
    }
    return TRUE;
}

sonix_bool SonixCam_AsicRegisterWrite(unsigned short addr, unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    BYTE data = 0;
    LONG startAddr = addr;
    LONG i = 0;
    for (i = 0; i < len; i++) {
        data = pData[i];
        if (TRUE != LibUsb_WriteToASIC((USHORT)startAddr++, data)) return FALSE;
    }
    return TRUE;
}

sonix_bool SonixCam_SensorRegisterRead(unsigned char slaveId, unsigned short addr, unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    LONG loop = len / 3;
    LONG ram = len % 3;
    BYTE tempData[3] = {0};
    LONG i = 0;
    for (i = 0; i < loop; i++) {
        if (TRUE != LibUsb_ReadFromSensor(slaveId, (USHORT)addr, tempData, 3)) return FALSE;
        memcpy(&pData[i * 3], tempData, 3);
    }
    if (ram) {
        if (TRUE != LibUsb_ReadFromSensor(slaveId, (USHORT)addr, tempData, ram)) return FALSE;
        memcpy(&pData[loop * 3], tempData, ram);
    }
    return TRUE;
}

sonix_bool SonixCam_SensorRegisterWrite(unsigned char slaveId, unsigned short addr, unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    LONG loop = len / 3;
    LONG ram = len % 3;
    LONG i = 0;
    for (i = 0; i < loop; i++) {
        if (TRUE != LibUsb_WriteToSensor(slaveId, (USHORT)addr, &pData[i * 3], 3)) return FALSE;
    }
    if (ram) {
        if (TRUE != LibUsb_WriteToSensor(slaveId, (USHORT)addr, &pData[loop * 3], ram)) return FALSE;
    }
    return TRUE;
}

sonix_bool SonixCam_SensorRegisterCustomRead(unsigned char slaveId, long addr, unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    return LibUsb_CustomReadFromSensor(slaveId, addr, pData, len);
}

sonix_bool SonixCam_SensorRegisterCustomWrite(unsigned char slaveId, long addr, unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    return LibUsb_CustomWriteToSensor(slaveId, addr, pData, len);
}

sonix_bool SonixCam_GetSerialFlashType(SERIAL_FLASH_TYPE *sft) {
    if (!svc100_camera_init) return FALSE;

    return LibUsb_GetSerialFlashType(sft);
}

sonix_bool SonixCam_SerialFlashRead(long addr, unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    return LibUsb_ReadFormSF(addr, pData, len);
}

sonix_bool SonixCam_SerialFlashSectorWrite(long addr, unsigned char pData[], long len, SERIAL_FLASH_TYPE sft) {
    if (!svc100_camera_init) return FALSE;

    return LibUsb_WriteToSF(addr, pData, len, sft);
}

sonix_bool SonixCam_GetFwVersion(unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    BYTE RomValue[10] = {0};
    BOOL hr = LibUsb_GetAsicRomVersion(RomValue);

    BYTE chipID;
    DSP_ARCH_TYPE dspArchType;
    if (DRT_Unknow == LibUsb_GetChipRomType(&chipID, &dspArchType)) return FALSE;

    BYTE FlashCodeVer[56] = {0};
    BYTE CusVer[31];
    BYTE DayVer[31];
    BYTE Customer[31];

    switch (chipID) {
        case 0x15:
            LibUsb_ReadFormSF(0x3E00, Customer + 2, 10);
            LibUsb_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
            break;
        case 0x16:
            LibUsb_ReadFormSF(0x0FD0, Customer + 2, 10);
            LibUsb_ReadFormSF(0x0FD0 + 10, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x0FD0 + 20, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
            break;
        case 0x20:
            LibUsb_ReadFormSF(0x5E00, Customer + 2, 10);
            LibUsb_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x0 + 11, FlashCodeVer, 4);
            break;
        case 0x25:
            LibUsb_ReadFormSF(0x5E00, Customer + 2, 10);
            LibUsb_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
            break;
        case 0x30:
            LibUsb_ReadFormSF(0x1E00, Customer + 2, 10);
            LibUsb_ReadFormSF(0x1E00 + 10, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x1E00 + 20, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
        case 0x31:
            if (!hr) {
                return FALSE;
            }
            if (RomValue[5] == 1) {
                LibUsb_ReadFormSF(0x1E00, Customer + 2, 10);
                LibUsb_ReadFormSF(0x1E00 + 10, CusVer + 2, 10);
                LibUsb_ReadFormSF(0x1E00 + 20, DayVer + 2, 10);
                LibUsb_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
            } else if (RomValue[5] == 2) {
                LibUsb_ReadFormSF(0x6000 + 0x1E0, Customer + 2, 10);
                LibUsb_ReadFormSF(0x6000 + 0x1E00 + 10, CusVer + 2, 10);
                LibUsb_ReadFormSF(0x6000 + 0x1E00 + 20, DayVer + 2, 10);
                LibUsb_ReadFormSF(0x6000 + 0x01e8 + 11, FlashCodeVer, 5);
            } else {
                return FALSE;
            }

            break;
        case 0x32:
            if (!hr) {
                return FALSE;
            }
            if (RomValue[5] == 1) {
                LibUsb_ReadFormSF(0x0FD0, Customer + 2, 10);
                LibUsb_ReadFormSF(0x0FD0 + 10, CusVer + 2, 10);
                LibUsb_ReadFormSF(0x0FD0 + 20, DayVer + 2, 10);
                LibUsb_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
            } else if (RomValue[5] == 2) {
                LibUsb_ReadFormSF(0x0FD0, Customer + 2, 10);
                LibUsb_ReadFormSF(0x0FD0 + 18, CusVer + 2, 10);
                LibUsb_ReadFormSF(0x0FD0 + 28, DayVer + 2, 10);
                LibUsb_ReadFormSF(0x0FD0 + 10, FlashCodeVer, 5);
            } else {
                return FALSE;
            }
            break;
        case 0x36:
            LibUsb_ReadFormSF(0x3E00, Customer + 2, 10);
            LibUsb_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
            break;
        case 0x50:
            LibUsb_ReadFormSF(0x5E00, Customer + 2, 10);
            LibUsb_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
            break;
        case 0x56:
            LibUsb_ReadFormSF(0x3E00, Customer + 2, 10);
            LibUsb_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
            break;
        case 0x70:
            LibUsb_ReadFormSF(0x3E05, Customer + 2, 10);
            LibUsb_ReadFormSF(0x3E0F, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x3E19, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x3E00, FlashCodeVer, 5);
            break;
        case 0x71:
            LibUsb_ReadFormSF(0x3E05, Customer + 2, 10);
            LibUsb_ReadFormSF(0x3E0F, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x3E19, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x3E00, FlashCodeVer, 5);
            break;
        case 0x75:
            if (RomValue[5] == 1) {
                LibUsb_ReadFormSF(0x2000, Customer + 2, 10);
                LibUsb_ReadFormSF(0x2012, CusVer + 2, 10);
                LibUsb_ReadFormSF(0x201C, DayVer + 2, 10);
                LibUsb_ReadFormSF(0x200A, FlashCodeVer, 5);
            } else {
                return FALSE;
            }

            break;
        case 0x76:  // 128K
            if (RomValue[5] == 1) {
                LibUsb_ReadFormSF(0x3FD0, Customer + 2, 10);
                LibUsb_ReadFormSF(0x3FD0 + 10, CusVer + 2, 10);
                LibUsb_ReadFormSF(0x3FD0 + 20, DayVer + 2, 10);
                LibUsb_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
            } else {
                return FALSE;
            }
            break;
        case 0x83:  // 128K
            LibUsb_ReadFormSF(0x2000, Customer + 2, 10);
            LibUsb_ReadFormSF(0x2012, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x201C, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x200A, FlashCodeVer, 5);
        case 0x85:  // 128K
            LibUsb_ReadFormSF(0x2000, Customer + 2, 10);
            LibUsb_ReadFormSF(0x2012, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x201C, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x200A, FlashCodeVer, 5);
            break;
        case 0x90:
            LibUsb_ReadFormSF(0x2000, Customer + 2, 10);
            LibUsb_ReadFormSF(0x2012, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x201C, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x200A, FlashCodeVer, 5);
            break;
        case 0x92:  // 128K
            LibUsb_ReadFormSF(0x2000, Customer + 2, 10);
            LibUsb_ReadFormSF(0x2012, CusVer + 2, 10);
            LibUsb_ReadFormSF(0x201C, DayVer + 2, 10);
            LibUsb_ReadFormSF(0x200A, FlashCodeVer, 5);
            break;
        default:
            return FALSE;
            break;
    }

    // 220
    if (chipID == 0x20) {
        FlashCodeVer[40] = '\0';
    } else {
        FlashCodeVer[41] = '\0';
    }

    memcpy(FlashCodeVer + 5, CusVer + 2, 10);
    memcpy(FlashCodeVer + 15, Customer + 2, 10);
    memcpy(FlashCodeVer + 25, DayVer + 2, 10);
    char Temp[56] = {0};
    memcpy(Temp, FlashCodeVer, 56);
    // memcpy_s(pData, len, Temp, sizeof(Temp));
    memcpy(pData, Temp, sizeof(Temp));
    return TRUE;
}

sonix_bool SonixCam_BurnerFW(unsigned char pFwBuffer[], LONG lFwLength, SonixCam_SetProgress setProgress,
                             void *ptrClass, SERIAL_FLASH_TYPE sft) {
    if (!svc100_camera_init) return FALSE;

    if (sft == SFT_UNKNOW) {
        svc100_gEC = EC_UnKnowSerialFlashType;
        return FALSE;
    }

    if (TRUE != LibUsb_DisableSerialFlashWriteProtect(sft)) {
        svc100_gEC = EC_DisableFlashWriteProtectFail;
        return FALSE;
    }

    if (TRUE != LibUsb_SerialFlashErase(sft)) {
        svc100_gEC = EC_EraseFlashFail;
        return FALSE;
    }

    /************************************************************************
    *
    **************************************************************************/
    BYTE *pBuffer = (unsigned char*)malloc(lFwLength);
    if (!pBuffer) {
        svc100_gEC = EC_MallocMemoryFail;
        return FALSE;
    }
    memcpy(pBuffer, pFwBuffer, lFwLength);
    BYTE intBuffer[8] = {0};
    memcpy(intBuffer, pBuffer + 0x160, 8);
    memset(pBuffer + 0x160, 0xFF, 4);

    float gProgress = 0;
    BYTE data[8];
    BYTE *pFw = pBuffer;
    LONG fwLen = lFwLength;
    BOOL sf_hight_addr = FALSE;
    BYTE addrLow, addrHigh;
    LONG i;
    for (i = 0; i < fwLen; i += 8) {
        if (setProgress && (i % 0x1000 == 0)) {
            gProgress = (float)i / (float)fwLen;
            setProgress(ptrClass, gProgress);
        }
        if (i % 0x1000 == 0) {
            gProgress = (float)i / (float)fwLen;
            printf("gProgress %5f\n", gProgress);
        }
        memset(data, 0xff, 8);
        memcpy(data, pFw + i, 8);
        if (TRUE != LibUsb_WriteDataToFlash(i, data, 8)) return FALSE;
    }
    /******************************************
    *
    ******************************************/
    LibUsb_WriteDataToFlash(0x160, intBuffer, 8);
    // SAFE_DELETE_ARRAY(pBuffer);
    free(pBuffer);
    pBuffer = 0;
    if (setProgress) {
        setProgress(ptrClass, 1);
    }

    pFw = pFwBuffer;
    sf_hight_addr = FALSE;
    for (i = 0; i < fwLen - 1024; i += 1024) {
        memset(&data, 0xff, 8);
        LibUsb_ReadDataFormFlash(i, data, 8);
        if (data[0] != *(pFw + i) || data[1] != *(pFw + i + 1) || data[2] != *(pFw + i + 2) ||
            data[3] != *(pFw + i + 3) || data[4] != *(pFw + i + 4) || data[5] != *(pFw + i + 5) ||
            data[6] != *(pFw + i + 6) || data[7] != *(pFw + i + 7)) {
            svc100_gEC = EC_BurnerCheckFail;
            sf_hight_addr = FALSE;
            return FALSE;
        }
    }
    sf_hight_addr = FALSE;
    return TRUE;
}

sonix_bool SonixCam_ExportFW(unsigned char pFwBuffer[], LONG lFwLength, SonixCam_SetProgress setProgress,
                             void *ptrClass) {
    if (!svc100_camera_init) return FALSE;

    BYTE tempData[8];
    BYTE *pFw = pFwBuffer;
    LONG fwLen = lFwLength;
    BOOL sf_hight_addr = FALSE;
    BYTE addrLow, addrHigh;
    float gProgress = 0.0;  //(0.0 - 1.0)
    LONG i;
    for (i = 0; i < fwLen; i += 8) {
        if (setProgress) {
            gProgress = (float)i / (float)fwLen;
            // setProgress(ptrClass, gProgress);
        }
        memset(&tempData, 0xff, 8);
        if (TRUE != LibUsb_ReadDataFormFlash(i, tempData, 8)) return FALSE;

        *(pFw + i) = tempData[0];
        *(pFw + i + 1) = tempData[1];
        *(pFw + i + 2) = tempData[2];
        *(pFw + i + 3) = tempData[3];
        *(pFw + i + 4) = tempData[4];
        *(pFw + i + 5) = tempData[5];
        *(pFw + i + 6) = tempData[6];
        *(pFw + i + 7) = tempData[7];
    }
    sf_hight_addr = FALSE;
    return TRUE;
}

sonix_bool SonixCam_GetSerialNumber(unsigned char pData[], long len) {
    if (!svc100_camera_init) return FALSE;

    return LibUsb_GetStringSettingFormSF(pData, len, 0xC0, TRUE);
}

BOOL SetParamToParamBuffer(BYTE paramBuffer[], LONG paramAddr, BYTE param[], LONG length) {
    BYTE *pParamTable = paramBuffer + paramAddr;
    memset(pParamTable, 0xFF, 64);
    pParamTable[0] = length * 2 + 2;
    pParamTable[1] = 0x03;
    LONG i = 0;
    for (i = 0; i < length; i++) {
        *(pParamTable + 2 + 2 * i) = param[i];
        *(pParamTable + 2 + 2 * i + 1) = 0x0;
    }
    return TRUE;
}

BOOL CheckCRC(BYTE *pFW, LONG paraTableStartAddr, LONG paraTableLength, LONG crcStartAddr) {
    USHORT crc16 = 0xFFFF;
    BYTE temp;
    LONG i, j;
    BYTE *pParaBuffer = pFW + paraTableStartAddr;
    for (i = 0; i < paraTableLength; i++) {
        temp = pParaBuffer[i];
        crc16 ^= temp;
        for (j = 0; j < 8; j++) {
            if (crc16 & 0x01) {
                crc16 >>= 1;
                crc16 ^= 0xA001;
            } else
                crc16 >>= 1;
        }
    }
    *(pFW + crcStartAddr + 20) = crc16 >> 8;
    *(pFW + crcStartAddr + 21) = crc16;
    return FALSE;
}

sonix_bool SonixCam_SetParamTableFormFWFile(unsigned char pFW[], long lFwLength, const ChangeParamInfo *pParamInfo,
                                            SonixCam_SetProgress setProgress, void *ptrClass, SERIAL_FLASH_TYPE sft,
                                            char *pLogFilePath) {
    if (!svc100_camera_init) return false;

    if (sft == SFT_UNKNOW) {
        svc100_gEC = EC_UnKnowSerialFlashType;
        return false;
    }

    DWORD dwStringAddr = 0;
    ULONG dwParaTableStartAddr = 0;
    ULONG dwParaTableEndAddr = 0;
    ULONG dwCRCStartAddr = 0;
    if (!LibUsb_GetParaTableAndCRCAddrFormSF(&dwParaTableStartAddr, &dwParaTableEndAddr, &dwCRCStartAddr)) return false;

    printf("dwParaTableStartAddr :%x, dwParaTableEndAddr : %x, dwCRCStartAddr: %x", (int)dwParaTableStartAddr,
           (int)dwParaTableEndAddr, (int)dwCRCStartAddr);
    LONG SZ_4K = 4 * 1024;
    LONG SZ_16K = 16 * 1024;
    LONG SEA_4K = dwParaTableStartAddr / SZ_4K * SZ_4K;
    LONG SEA_16K = dwParaTableStartAddr / SZ_16K * SZ_16K;

    LONG startSectorEraseAddr = SEA_4K < SEA_16K ? SEA_4K : SEA_16K;
    LONG sectorEraseEndAddr = startSectorEraseAddr + SZ_16K;

    BYTE *pCopyFW = 0;
    pCopyFW = (unsigned char*)malloc(lFwLength);
    if (!pCopyFW) return false;
    memcpy(pCopyFW, pFW, lFwLength);

    LONG startAddr = startSectorEraseAddr;
    BOOL sf_hight_addr = FALSE;
    BYTE temp[8];
    BYTE addrLow;
    BYTE addrHigh;
    LONG addIndex = 0;
    float gProgress = 0.0;  //(0.0 - 1.0)

    if (pParamInfo->pVidPid) {
        BYTE *pBuffer = pCopyFW + (dwParaTableStartAddr + 0x06);
        memcpy(pBuffer, pParamInfo->pVidPid, 4);
    }
    if (pParamInfo->pProduct) {
        SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x40), (BYTE *)pParamInfo->pProduct,
                              pParamInfo->ProductLength);
    }
    if (pParamInfo->pSerialNumber) {
        SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0xC0), (BYTE *)pParamInfo->pSerialNumber,
                              pParamInfo->SerialNumberLength);
    }
    if (pParamInfo->pManufacture) {
        SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x80), (BYTE *)pParamInfo->pManufacture,
                              pParamInfo->ManufactureLength);
    }
    if (pParamInfo->pString3) {
        SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x100), (BYTE *)pParamInfo->pString3,
                              pParamInfo->String3Length);
    }
    if (pParamInfo->pInterface) {
        SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x140), (BYTE *)pParamInfo->pInterface,
                              pParamInfo->InterfaceLength);
    }

    // CRC Check
    CheckCRC(pCopyFW, dwParaTableStartAddr, dwParaTableEndAddr - dwParaTableStartAddr, dwCRCStartAddr);

    // disable flash write protect
    if (!LibUsb_DisableSerialFlashWriteProtect(sft)) {
        svc100_gEC = EC_DisableFlashWriteProtectFail;
        return false;
    }

    // sector erase
    LibUsb_EraseSectorForSerialFlash(dwParaTableStartAddr, (SERIAL_FLASH_TYPE)sft);
    sleep(1);

    startAddr = startSectorEraseAddr;
    sf_hight_addr = FALSE;
    LONG flashSectorSize = SZ_16K;
    BOOL eraseCheckArr[4] = {0};
    // erase check and get flash sector size
    LONG i = 0;
    for (i = 0; i < SZ_16K; i += 0x50) {
        memset(&temp, 0xff, 8);
        LibUsb_ReadDataFormFlash(startAddr, temp, 8);
        if (temp[0] != 0xFF || temp[1] != 0xFF || temp[2] != 0xFF || temp[3] != 0xFF || temp[4] != 0xFF ||
            temp[5] != 0xFF || temp[6] != 0xFF || temp[7] != 0xFF) {
            BYTE index = i / SZ_4K;
            eraseCheckArr[index] = TRUE;
            i = i / SZ_4K * SZ_4K + SZ_4K - 0x50;
            flashSectorSize -= SZ_4K;
            startAddr = startAddr / SZ_4K * SZ_4K + SZ_4K;
            continue;
        }
        startAddr += 0x50;
    }

    if (!flashSectorSize) {
        svc100_gEC = EC_EraseFlashFail;
        return false;
    }

    // burner sector src date
    BYTE *pData = pCopyFW + startSectorEraseAddr;
    startAddr = startSectorEraseAddr;
    sf_hight_addr = FALSE;
    LONG s = 0;
    for (s = 0; s < 4; s++) {
        if (eraseCheckArr[s]) {
            startAddr += SZ_4K;
            pData += SZ_4K;
            continue;
        }
        for (i = 0; i < SZ_4K; i += 8) {
            LONG addr = s * SZ_4K + i;
            if (setProgress && addr % 0x20) {
                gProgress = ((float)(addr / 2)) / (float)SZ_16K + 0.5f;
                setProgress(ptrClass, gProgress);
            }
            memcpy(temp, pData, 8);
            LibUsb_WriteDataToFlash(startAddr, temp, 8);
            startAddr += 8;
            pData += 8;
        }
    }

    // burn check
    startAddr = startSectorEraseAddr;
    pData = pCopyFW + startSectorEraseAddr;
    sf_hight_addr = FALSE;
    i = 0;
    for (i = 0; i < SZ_16K; i += 0x200) {
        memset(&temp, 0xFF, 8);
        LibUsb_ReadDataFormFlash(startAddr, temp, 8);
        if (temp[0] != *(pData + i) || temp[1] != *(pData + i + 1) || temp[2] != *(pData + i + 2) ||
            temp[3] != *(pData + i + 3) || temp[4] != *(pData + i + 4) || temp[5] != *(pData + i + 5) ||
            temp[6] != *(pData + i + 6) || temp[7] != *(pData + i + 7)) {
            svc100_gEC = EC_BurnerCheckFail;
            return false;
        }
        startAddr += 0x200;
    }
    free(pCopyFW);
    return true;
}

}
//#ifdef __cplusplus
//}
//#endif


