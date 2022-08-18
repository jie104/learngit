
#include "LUOptDev.h"
#include "libusb/libusb.h"
//#include <usb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SFDate.h"
#include "V4L2OptDev.h"
#include "libusb/libusbi.h"
#include "util.h"
namespace usb {

static libusb_device **sv100_devs = NULL;
struct libusb_device_handle *svc100_devh = NULL;
ERROR_CODE svc100_gEC = EC_UnKnow;

long svc100_g_nDevNum = 0;
struct usb_device *svc100_g_pAllDevice[MAX_CAM_NUM];

unsigned char chr2hex(char chr) {
    if (chr >= '0' && chr <= '9') return (chr - '0');
    if (chr >= 'A' && chr <= 'F') return (chr - 'A' + 10);
    if (chr >= 'a' && chr <= 'f') return (chr - 'a' + 10);
    return 0;
}

unsigned int cal_pow(int src, int v) {
    int i;
    unsigned int ret = 1;
    if (v == 0) return 1;
    for (i = 0; i < v; i++) {
        ret *= src;
    }
    return ret;
}

BOOL VidPidStringToInt(char *vidpid, USHORT *vid, USHORT *pid) {
    if (!vidpid) return TRUE;
    LONG len = strlen(vidpid);
    if (len != 8) return FALSE;
    LONG i = 0;
    *vid = 0;
    *pid = 0;
    for (i = len; i > 0; i--) {
        unsigned char var = chr2hex(vidpid[len - i]);
        if (i <= 4)
            *pid += var * cal_pow(16, i - 1);
        else
            *vid += var * cal_pow(16, i - 5);
    }
    return TRUE;
}

USHORT GetUVCExtendUnitID() {
    unsigned char data[0x04];
    int r;
    unsigned int i;
    long addr = 0x101f;
    data[0] = (char)addr;
    data[1] = (char)(addr >> 8);
    data[2] = 00;
    data[3] = 0xff;

    // Dummy Write
    r = libusb_control_transfer(svc100_devh, CTRL_OUT, 0x01, ASIC_RW, 0x300, data, 4, 1000);
    // Read
    r = libusb_control_transfer(svc100_devh, CTRL_IN, 0x81, ASIC_RW, 0x300, data, 4, 1000);
    if (r > 0) {
        return 0x300;
    }

    r = libusb_control_transfer(svc100_devh, CTRL_OUT, 0x01, ASIC_RW, 0x400, data, 4, 1000);
    // Read
    r = libusb_control_transfer(svc100_devh, CTRL_IN, 0x81, ASIC_RW, 0x400, data, 4, 1000);
    if (r > 0) {
        return 0x400;
    }

    return 0;
}

BOOL LibUsb_TEST_READ() {
    unsigned char data[0x04] = {0};
    int r;
    unsigned int i;
    long addr = 0x101f;

    r = libusb_control_transfer(svc100_devh, CTRL_IN, 0x81, 0x04, 0x100, data, 4, 1000);
    if (r > 0) {
        printf("LibUsb_TEST_READ().libusb_control_transfer %d %s\n", r, strerror(-r));
    }

    for (i = 0; i < 4; i++) printf("data[%d]%d\n", i, data[i]);

    data[0] = 0x9c;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    r = libusb_control_transfer(svc100_devh, CTRL_OUT, 0x21, 0x04, 0x100, data, 4, 500);
    if (r > 0) {
        printf("LibUsb_TEST_READ().libusb_control_transfer %d %s\n", r, strerror(-r));
    }

    return TRUE;
}

/*

BOOL LibUsb_IsSonixDevice(libusb_device *dev)
{

        struct libusb_device_descriptor desc;
        if(LIBUSB_SUCCESS != libusb_get_device_descriptor(dev, &desc))
        {
                fprintf(stderr, "failed to get device descriptor");
                return FALSE;
        }

        LONG configIndex = 0;
        LONG ifIndex = 0;
        for(index = 0; index < desc->bNumConfigurations; index++)
        {
                libusb_get_config_descriptor(dev, i, &conf_desc);
                for(ifIndex = 0; ifIndex < conf_desc->bNumInterfaces; ifIndex++)
                {

                }
        }

    return TRUE;
}

BOOL LibUsb_IsValidID(struct usb_device *pDev)
{
        usb_dev_handle 	*udev;
        BOOL bRet = FALSE;

        libusb_open(pDev, &svc100_devh);
        if (udev == NULL)
                goto exit;

        /*
        if(libusb_kernel_driver_active(svc100_devh, 0) != 1)
        {
                printf("driver not ok, reattach\n");
                libusb_attach_kernel_driver(svc100_devh, 0);
        }

        for(i=0; i<3; i++)
        {
                r = libusb_claim_interface(svc100_devh, 0);
                if (r >= 0)
                        break;
                //fprintf(stderr, "usb_claim_interface error %d %s\n", r, strerror(-r));
                r= libusb_detach_kernel_driver(svc100_devh, 0);
                r= libusb_set_configuration(svc100_devh, 1);
        }

        USHORT ExtendUnitID = 0;
        ExtendUnitID = GetUVCExtendUnitID();
        if!(ExtendUnitID)
                bRet = FALSE;
exit:
        libusb_close(svc100_devh);
        return bRet;
}


BOOL LibUsb_BusFindCam(struct usb_device *pDev, int level)
{
        int	i;
        if (LibUsb_IsValidID(pDev))
        {
                svc100_g_pAllDevice[svc100_g_nDevNum] = pDev;
                svc100_g_nDevNum++;
        }

        for (i=0; i < pDev->num_children; i++)
        {
                LibUsb_BusFindCam(pDev->children[i], level+1);
        }
        return TRUE;
}


BOOL LibUsb_EnumCamera()
{
        struct usb_bus		*bus;
        struct usb_device 	*pDev;
        int  i;
        usb_find_busses();
        usb_find_devices();

        svc100_g_nDevNum = 0;
        for (bus = usb_busses; bus; bus = bus->next)
        {
                if (bus->root_dev)
                        LibUsb_BusFindCam(bus->root_dev, 0);
                else
                {
                        for (pDev = bus->devices; pDev; pDev = pDev->next)
                                LibUsb_BusFindCam(pDev, 0);
                }
        }
        return TRUE;
}
*/

BOOL LibUsb_EnumDevice(char *vidpid, int first_usb_id, int *find_id, int *dev_number) {
    ssize_t cnt;
    cnt = libusb_get_device_list(NULL, &sv100_devs);

    printf("usb number is : %d\n", (int)cnt);

    if (cnt < 0) return FALSE;

    libusb_device *dev;
    BOOL iret = FALSE;
    LONG i = 0, j = 0;
    int r = 1;

    USHORT vid, pid;
    if (!VidPidStringToInt(vidpid, &vid, &pid)) return FALSE;
    ULONG ulvidpid;
    USHORT unitID;

    //	if(TRUE != V4L2_EnumDevice(vidpid, &ulvidpid, &videoChipID, &dspArchType, &unitID))
    //		return FALSE;

    // USHORT sVid, sPid;
    // sVid = ulvidpid >> 16;
    // sPid = ulvidpid;

    // printf("video device vid pid :%x %x \n", sVid, sPid);
    // printf("video device chip id :%x\n", videoChipID);

    i = 0 > cnt ? cnt : 0;
    printf("index is:%d\n", (int)i);
    while ((dev = sv100_devs[i++]) != NULL) {
        if (dev->device_address == first_usb_id) {
            struct libusb_device_descriptor desc;
            r = libusb_get_device_descriptor(dev, &desc);
            if (r < 0) {
                fprintf(stderr, "failed to get device descriptor\n");
                // return FALSE;
            }
            if (vidpid) {
                if (desc.idVendor == vid && desc.idProduct == pid) {
                    printf("%04x:%04x (bus %d, device %d)\n", desc.idVendor, desc.idProduct, libusb_get_bus_number(dev),
                           libusb_get_device_address(dev));

                    int open_state = libusb_open(dev, &svc100_devh);
                    if (open_state == 0) {
                        iret = TRUE;
                    } else {
                        printf("cannot open device:%d,have not the athority!\n", first_usb_id);
                    }
                    *find_id = i;
                    *dev_number = libusb_get_device_address(dev);
                    break;
                }
            }
        }
        if (i >= cnt) {
            printf("cannot get device! will break!");
            break;
        }
    }

    if (!iret) {
        libusb_close(svc100_devh);
        libusb_free_device_list(sv100_devs, 1);
        libusb_exit(NULL);
        return FALSE;
    } else {
        libusb_free_device_list(sv100_devs, 1);
        sv100_devs = NULL;
    }

    if (libusb_kernel_driver_active(svc100_devh, 0) == 1) {
        printf("driver not ok, reattach\n");
        libusb_detach_kernel_driver(svc100_devh, 0);
        printf("driver attach ok, reattach\n");
    }

    for (i = 0; i < 3; i++) {
        r = libusb_claim_interface(svc100_devh, 0);
        if (r >= 0) break;
        // fprintf(stderr, "usb_claim_interface error %d %s\n", r, strerror(-r));
        r = libusb_detach_kernel_driver(svc100_devh, 0);
        r = libusb_set_configuration(svc100_devh, 1);
    }
    printf("detach state:%d\n", r);
    if (r < 0) {
        printf("usb claim interface failed\n");
        libusb_close(svc100_devh);
        libusb_free_device_list(sv100_devs, 1);
        libusb_exit(NULL);
        return 0;
    }

    USHORT ExtendUnitID = 0;
    if (!(ExtendUnitID = GetUVCExtendUnitID())) return FALSE;

    g_CurDspExtendUnitID = ExtendUnitID;

    BYTE chipID;
    DSP_ARCH_TYPE dspArchType;
    DSP_ROM_TYPE romType = LibUsb_GetChipRomType(&chipID, &dspArchType);
    return iret;
}

BOOL LibUsb_CloseDevice() {
    libusb_release_interface(svc100_devh, 0);
    if (libusb_kernel_driver_active(svc100_devh, 0) != 1) libusb_attach_kernel_driver(svc100_devh, 0);

    libusb_close(svc100_devh);
    libusb_free_device_list(sv100_devs, 1);
    libusb_exit(NULL);
    svc100_devh = NULL;
    sv100_devs = NULL;
    return TRUE;
}

BOOL LibUsb_RestartDevice() {
    BYTE romVersion[10] = {0};
    if (TRUE != LibUsb_GetAsicRomVersion(romVersion)) {
        svc100_gEC = EC_GetAsicRomVersionFail;
        return FALSE;
    }
    if ((svc100_uiRomID == ROM220) || (svc100_uiRomID == ROM225)) {
        return FALSE;
    } else if ((svc100_uiRomID == ROM283) || (svc100_uiRomID == ROM292)) {
        BYTE TempVar = 0;
        if (TRUE != LibUsb_ReadFromASIC(0x1073, &TempVar)) {
            return FALSE;
        }
        TempVar &= 0xFE;
        LibUsb_WriteToASIC(0x1073, TempVar);  // Always Error because of HW reset, Ignore it.
        usleep(1000);
        return TRUE;
    } else {
        LibUsb_WriteToASIC(0x1073, 0x00);
        return TRUE;
    }
    return TRUE;
}

BOOL LibUsb_GetChipID(LONG idAddr, BYTE *pChipID) {
    BYTE chipID = 0;
    if (!LibUsb_ReadFromASIC(idAddr, &chipID)) {
        return FALSE;
    }

    *pChipID = chipID;

    return TRUE;
}

DSP_ROM_TYPE LibUsb_GetChipRomType(BYTE *dspChipID, DSP_ARCH_TYPE *dspArchType) {
    LONG i = 0;
    DSP_ROM_TYPE ret = DRT_Unknow;
    BYTE chipId = 0;
    DSP_ARCH_TYPE dspArchIndex = DAT_UNKNOW;

    for (i = 0; i < DSP_ARCH_COUNT; i++) {
        if (!LibUsb_GetChipID(g_DspArchInfo[i].dspIdAddr, &chipId)) return ret;
        switch (chipId) {
            case 0x15:
            case 0x16:
            case 0x22:
            case 0x23:
            case 0x25:
            case 0x32:
            case 0x33:
            case 0x56:
            case 0x70:
            case 0x71:
            case 0x75:
            case 0x88:
            case 0x90:
                ret = DRT_64K;
                dspArchIndex = DAT_FIRST;
                break;
            case 0x85:
                dspArchIndex = DAT_SECOND;
                ret = DRT_128K;
                break;
            case 0x76:
            case 0x83:
            case 0x87:
            case 0x92:
                ret = DRT_128K;
                dspArchIndex = DAT_FIRST;
                break;
            case 0x67:
                ret = DRT_32K;
                dspArchIndex = DAT_FIRST;
                break;
            default:
                ret = DRT_Unknow;
                break;
        }
        if (ret >= 0) {
            *dspChipID = chipId;
            *dspArchType = dspArchIndex;
            set_dsp_base_addr(dspArchIndex);

            return ret;
        }
    }
    return ret;
}

BOOL LibUsb_EnableAsicRegisterBit(LONG addr, BYTE bit) { return TRUE; }

BOOL LibUsb_DisableAsicRegisterBit(LONG addr, BYTE bit) { return TRUE; }

BOOL LibUsb_ReadFromASIC(USHORT addr, BYTE *pValue) {
    unsigned char data[0x04];
    int r;
    unsigned int i;
    data[0] = (char)addr;
    data[1] = (char)(addr >> 8);
    data[2] = 00;
    data[3] = 0xff;

    // Dummy Write
    r = libusb_control_transfer(svc100_devh, CTRL_OUT, 0x01, ASIC_RW, g_CurDspExtendUnitID, data, 4, 1000);
    // Read
    r = libusb_control_transfer(svc100_devh, CTRL_IN, 0x81, ASIC_RW, g_CurDspExtendUnitID, data, 4, 1000);

    if (r < 0) {
        fprintf(stderr, "F0 error %d\n", r);
        return FALSE;
    }

    *pValue = data[2];

    return TRUE;
}

BOOL LibUsb_WriteToASIC(USHORT addr, BYTE value) {
    int r;
    unsigned char data[0x04];
    int i = 0;

    data[0] = (char)addr;
    data[1] = (char)(addr >> 8);
    data[2] = value;
    data[3] = 0x00;
    r = libusb_control_transfer(svc100_devh, CTRL_OUT, 0x01, ASIC_RW, g_CurDspExtendUnitID, data, 4, 1000);
    if (r < 0) {
        fprintf(stderr, "F1 error %d\n", r);
        return FALSE;
    }
    return TRUE;
}

BOOL LibUsb_ReadFromROM(LONG addr, BYTE data[]) {
    int r;
    BYTE temp[0x0b];
    int i = 0;

    temp[0] = (char)addr;
    temp[1] = (char)(addr >> 8);
    temp[2] = 0x8;

    // Dummy Write
    r = libusb_control_transfer(svc100_devh, CTRL_OUT, 0x01, ROM_RW, g_CurDspExtendUnitID, temp, 11, 1000);
    // Read
    r = libusb_control_transfer(svc100_devh, CTRL_IN, 0x81, ROM_RW, g_CurDspExtendUnitID, temp, 11, 1000);

    if (r < 0) {
        fprintf(stderr, "F2 error %d\n", r);
        return FALSE;
    }

    memcpy(data, temp + 3, 8);
    return TRUE;
}

BOOL LibUsb_ReadFromSensor(BYTE slaveID, USHORT addr, BYTE pData[], LONG len) { return TRUE; }

BOOL LibUsb_WriteToSensor(BYTE slaveID, USHORT addr, BYTE pData[], LONG len) { return TRUE; }

BOOL LibUsb_CustomReadFromSensor(BYTE slaveID, USHORT addr, BYTE pData[], LONG len) { return TRUE; }

BOOL LibUsb_CustomWriteToSensor(BYTE slaveID, USHORT addr, BYTE pData[], LONG len) { return TRUE; }

SERIAL_FLASH_TYPE LibUsb_SerialFlashSearch() {
    BYTE i, ubID_Num;
    BYTE ubSFType = SF_UNKNOW;
    ubID_Num = ubSFLib_GetIDSize();
    for (i = 1; i < ubID_Num; i++) {
        if (cbSFLib_ID[i][SFCMD_INFO_MFR] == sfManufactureID &&
            (cbSFLib_ID[i][SFCMD_INFO_DEVID1] == sfDeviceID1 || cbSFLib_ID[i][SFCMD_INFO_DEVID1] == 0xFF) &&
            (cbSFLib_ID[i][SFCMD_INFO_DEVID2] == sfDeviceID2 || cbSFLib_ID[i][SFCMD_INFO_DEVID2] == 0xFF)) {
            break;
        }
    }
    if (i == ubID_Num) i = 0;
    ubSFType = cbSFLib_ID[i][SFCMD_INFO_TYPE];
    return (SERIAL_FLASH_TYPE)ubSFType;
}

BOOL LibUsb_ReadBitFormAsic(LONG addr, BYTE bit) {
    BYTE bufs;
    BYTE data;
    LibUsb_ReadFromASIC(addr, &bufs);

    switch (bit) {
        case 0:
            data = bufs & 0x01;
            break;
        case 1:
            data = bufs & 0x02;
            break;
        case 2:
            data = bufs & 0x04;
            break;
        case 3:
            data = bufs & 0x08;
            break;
        case 4:
            data = bufs & 0x10;
            break;
        case 5:
            data = bufs & 0x20;
            break;
        case 6:
            data = bufs & 0x40;
            break;
        case 7:
            data = bufs & 0x80;
            break;
        default:
            break;
    }
    return data;
}

BOOL LibUsb_SFWaitReady() {
    BYTE i;
    LONG data = 0;
    for (i = 0; i < 50; i++)  // pooling ready; 500us timeout
    {
        if (LibUsb_ReadBitFormAsic(sfRdyAddr, 0))  // aISP_RDY
        {
            return TRUE;
        }
        usleep(1000);
    }
    return FALSE;
}

BOOL LibUsb_SFCMDreadStatus() {
    int i;
    unsigned char ucData;
    for (i = 0; i < 10000; ++i) {
        // chip select to low
        if (TRUE != LibUsb_WriteToASIC(sfCSAddr, 0x0)) {
            return FALSE;
        }
        // Read status cmd
        if (TRUE != LibUsb_WriteToASIC(sfWriteDataAddr, 0x05)) {
            return FALSE;
        }
        // Write trig
        if (TRUE != LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01)) {
            return FALSE;
        }
        if (TRUE != LibUsb_SFWaitReady()) {
            return FALSE;
        }
        // Read reg cmd
        if (TRUE != LibUsb_WriteToASIC(sfReadDataAddr, 0x0)) {
            return FALSE;
            ;
        }
        // 2: Trigger for read data to Serial Flash in SF_MODE
        if (TRUE != LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x02)) {
            return FALSE;
        }
        if (TRUE != LibUsb_SFWaitReady()) {
            return FALSE;
        }
        // Data read from Serial Flash
        if (TRUE != LibUsb_ReadFromASIC(sfReadDataAddr, &ucData)) {
            return FALSE;
        }
        // chip select to high
        if ((ucData & 0x01) != 0x01) {
            if (TRUE != LibUsb_WriteToASIC(sfCSAddr, 0x1)) {
                return FALSE;
            }
            return TRUE;
        }
        usleep(1000);
    }
    return FALSE;
}

void LibUsb_ReadSFID(BYTE cmd, BYTE dummyNum, BYTE devIdNum) {
    LONG data;
    data = 0x1;
    LibUsb_WriteToASIC(0x1080, data);
    data = 0x0;
    LibUsb_WriteToASIC(0x1091, data);
    data = cmd;
    LibUsb_WriteToASIC(0x1082, data);
    data = 0x01;
    LibUsb_WriteToASIC(0x1081, data);
    LibUsb_SFWaitReady();

    while (dummyNum > 0) {
        data = 0x00;
        LibUsb_WriteToASIC(0x1082, data);
        data = 0x01;
        LibUsb_WriteToASIC(0x1081, data);
        LibUsb_SFWaitReady();
        dummyNum--;
    }

    data = 0x0;
    LibUsb_WriteToASIC(0x1083, data);
    data = 0x02;
    LibUsb_WriteToASIC(0x1081, data);
    LibUsb_SFWaitReady();
    LibUsb_ReadFromASIC(0x1083, &sfManufactureID);
    if (sfManufactureID == SF_MFRID_CONT) {
        data = 0x0;
        LibUsb_WriteToASIC(0x1083, data);
        data = 0x02;
        LibUsb_WriteToASIC(0x1081, data);
        LibUsb_SFWaitReady();
        LibUsb_ReadFromASIC(0x1083, &sfManufactureID);
    }

    data = 0x0;
    LibUsb_WriteToASIC(0x1083, data);
    data = 0x02;
    LibUsb_WriteToASIC(0x1081, data);
    LibUsb_SFWaitReady();
    LibUsb_ReadFromASIC(0x1083, &sfDeviceID1);
    if (devIdNum == 2) {
        data = 0x0;
        LibUsb_WriteToASIC(0x1083, data);
        data = 0x02;
        LibUsb_WriteToASIC(0x1081, data);
        LibUsb_SFWaitReady();
        LibUsb_ReadFromASIC(0x1083, &sfDeviceID2);
    } else
        sfDeviceID2 = 0xFF;

    data = 0x00;
    LibUsb_WriteToASIC(0x1080, data);
}

SERIAL_FLASH_TYPE LibUsb_SerialFlashIdentify() {
    // MXIC-like series
    LibUsb_ReadSFID(SFCMD_RDID_MXIC, 0, 2);
    SERIAL_FLASH_TYPE sfType = LibUsb_SerialFlashSearch();
    if (sfType != SF_UNKNOW) goto sfIndetifyExit;

    // Atmel AT25F series
    LibUsb_ReadSFID(SFCMD_RDID_AT25F, 0, 1);
    sfType = LibUsb_SerialFlashSearch();
    if (sfType != SF_UNKNOW) goto sfIndetifyExit;

    // SST/Winbond/Other MXIC-like
    LibUsb_ReadSFID(SFCMD_REMS_SST, 3, 1);
    sfType = LibUsb_SerialFlashSearch();
    if (sfType != SF_UNKNOW) goto sfIndetifyExit;

    // ST/PMC
    LibUsb_ReadSFID(SFCMD_RES_ST, 3, 1);
    sfType = LibUsb_SerialFlashSearch();
    if (sfType != SF_UNKNOW) goto sfIndetifyExit;

sfIndetifyExit:
    return sfType;
}

BOOL LibUsb_GetSerialFlashType(SERIAL_FLASH_TYPE *sft) {
    *sft = LibUsb_SerialFlashIdentify();
    return TRUE;
}

BOOL LibUsb_ReadFormSF(LONG addr, BYTE pData[], LONG len) {
    BOOL addr128k = FALSE;
    BYTE tempData[8];
    BYTE AddressLow, AddressHigh;
    LONG startAddr = addr;
    LONG loop = len / 8;
    LONG ram = len % 8;
    BOOL hr = TRUE;
    startAddr = addr;
    LONG addrIndex = 0;
    LONG i = 0;
    printf("the addr is:%d,%d\n", (int)startAddr, (int)loop);
    for (i = 0; i < loop; i++) {
        memset(&tempData, 0xff, 8);
        hr = LibUsb_ReadDataFormFlash(startAddr, tempData, 8);
        if (TRUE != hr) {
            return hr;
        }
        memcpy(pData + addrIndex, tempData, 8);
        addrIndex += 8;
        startAddr += 8;
//        printf("curr loop:%d,total loop:%d\n", i, loop);
    }

    if (ram > 0) {
        memset(&tempData, 0xff, 8);
        hr = LibUsb_ReadDataFormFlash(startAddr, tempData, ram);
        if (TRUE != hr) {
            return hr;
        }
        memcpy(pData + addrIndex, tempData, ram);
    }
    return TRUE;
}

BOOL LibUsb_WriteToSF(LONG addr, BYTE pData[], LONG len, SERIAL_FLASH_TYPE sft) {
    if (sft == SFT_UNKNOW) {
        svc100_gEC = EC_UnKnowSerialFlashType;
        return FALSE;
    }
    // disable serial flash wirte protect
    BOOL hr = LibUsb_DisableSerialFlashWriteProtect(sft);
    if (TRUE != hr) return hr;

    // erase serial flash sector
    hr = LibUsb_EraseSectorForSerialFlash(addr, sft);
    if (TRUE != hr) {
        return hr;
    }
    sleep(1);

    // write data to serial flash
    LONG startAddr = addr;
    LONG loop = len / 8;
    LONG ram = len % 8;
    BYTE tempData[8];
    BOOL addr128k = FALSE;
    BYTE addrHigh;
    BYTE addrLow;
    LONG i = 0;
    for (i = 0; i < loop; i++) {
        memcpy(tempData, pData + i * 8, 8);
        hr = LibUsb_WriteDataToFlash(startAddr, tempData, 8);
        if (!hr) return hr;
        startAddr += 8;
    }

    if (ram > 0) {
        memset(&tempData, 0xFF, 8);
        memcpy(tempData, pData + (loop * 8), ram);
        hr = LibUsb_WriteDataToFlash(startAddr, tempData, ram);
        if (TRUE != hr) {
            return hr;
        }
    }
    return hr;
}

BOOL LibUsb_ReadDataFormFlash(LONG addr, BYTE pData[], BYTE dataLen) {
    unsigned char data[11];
    if (dataLen > 8) dataLen = 8;

    data[1] = (BYTE)(addr >> 8);
    data[0] = (BYTE)((addr << 8) >> 8);

    if (addr < 0x10000)
        data[2] = 0x88;
    else if (addr < 0x20000)
        data[2] = 0x98;
    else if (addr < 0x30000)
        data[2] = 0xA8;
    else
        data[2] = 0xC8;
    data[2] &= 0xF0;
    data[2] |= dataLen;

    int r = libusb_control_transfer(svc100_devh, CTRL_OUT, 0x01, FLASH_RW, g_CurDspExtendUnitID, data, 11, 1000);

    // Read
    r = libusb_control_transfer(svc100_devh, CTRL_IN, 0x81, FLASH_RW, g_CurDspExtendUnitID, data, 11, 1000);
    if (r < 0) {
        fprintf(stderr, "F3 error %d\n", r);
        return 0;
    }

    memcpy(pData, data + 3, dataLen);

    return TRUE;
}

BOOL LibUsb_WriteDataToFlash(LONG addr, BYTE pData[], BYTE dataLen) {
    unsigned char data[11];
    if (dataLen > 8) dataLen = 8;

    data[1] = (BYTE)(addr >> 8);
    data[0] = (BYTE)((addr << 8) >> 8);

    if (addr < 0x10000)
        data[2] = 0x08;
    else if (addr < 0x20000)
        data[2] = 0x18;
    else if (addr < 0x30000)
        data[2] = 0x28;
    else
        data[2] = 0x38;
    data[2] &= 0xf0;
    data[2] |= dataLen;

    memcpy(data + 3, pData, dataLen);
    int r = libusb_control_transfer(svc100_devh, CTRL_OUT, 0x01, FLASH_RW, g_CurDspExtendUnitID, data, 11, 1000);
    if (r < 0) {
        fprintf(stderr, "F4 error %d\n", r);
        return 0;
    }
    return TRUE;
}

BOOL LibUsb_DisableSerialFlashWriteProtect(SERIAL_FLASH_TYPE sft) {
    if (sft == SFT_UNKNOW) {
        svc100_gEC = EC_UnKnowSerialFlashType;
        return FALSE;
    }

    BYTE romVersion[8] = {0};
    if (TRUE != LibUsb_GetAsicRomVersion(romVersion)) {
        svc100_gEC = EC_GetAsicRomTypeFail;
        return FALSE;
    }

    BYTE chipID;
    DSP_ARCH_TYPE dspArchType;
    DSP_ROM_TYPE drt = LibUsb_GetChipRomType(&chipID, &dspArchType);
    if (DRT_Unknow == drt) {
        svc100_gEC = EC_GetAsicRomTypeFail;
        return FALSE;
    }

    // setp 1 : disable hardware wirete protect
    BYTE data[2] = {0};
    if (drt == DRT_64K) {
        LibUsb_ReadFromASIC(gpioOutputAddr, &data[0]);
        data[0] = data[0] | 0x1F;
        LibUsb_WriteToASIC(gpioOutputAddr, data[0]);
        LibUsb_ReadFromASIC(gpioOutputAddr, &data[0]);
        LibUsb_ReadFromASIC(gpioOEAddr, &data[1]);
        data[1] = data[1] | 0x1F;
        LibUsb_WriteToASIC(gpioOEAddr, data[1]);
    } else {
        LibUsb_ReadFromASIC(gpioOutputAddr, &data[0]);
        data[0] = data[0] | 0xFF;
        LibUsb_WriteToASIC(gpioOutputAddr, data[0]);
        LibUsb_ReadFromASIC(gpioOutputAddr, &data[0]);
        LibUsb_ReadFromASIC(gpioOEAddr, &data[1]);
        data[1] = data[1] | 0xFF;
        LibUsb_WriteToASIC(gpioOEAddr, data[1]);
    }

    BYTE wpData = 0;
    switch (chipID) {
        case 0x33:
            LibUsb_ReadFormSF(0x000f, &wpData, 1);
            break;
        case 0x32:
        case 0x76:
            LibUsb_ReadFormSF(0xC034, &wpData, 1);
            break;
        case 0x16:
            LibUsb_ReadFormSF(0x5834, &wpData, 1);
            break;
        case 0x71:
        case 0x85:
            wpData = 0xFF;
            break;
        case 0x75:
            LibUsb_ReadFormSF(0xB034, &wpData, 1);
            break;
        default:
            LibUsb_ReadFormSF(0x8034, &wpData, 1);
            break;
    }

    BYTE byTmpAddr = 0;
    BYTE byMemType = 0;
    BYTE wpGPIO = 0;
    BOOL isNewWPVer = FALSE;
    USHORT wpAddr = 0xFFFF;
    BYTE sfWriteEnable = 0;
    BYTE sfWriteCommand = 0;
    wpGPIO = (wpData >> 4) & 0x7;
    if ((svc100_uiRomID != ROM283) && (svc100_uiRomID != ROM292) && (svc100_uiRomID != ROM275V2) &&
        (svc100_uiRomID != ROM287)) {
        if ((wpData & 0x0C) == 0x08) {
            isNewWPVer = TRUE;
        }
    }
    if (isNewWPVer) {
        if ((wpData & 0x03) == 0x02) {
            wpData = 1;
        } else if ((wpData & 0x03) == 0x03) {
            wpData = 2;
        }
    } else {
        wpData = wpData & 0x03;
    }
    if (RomInfo[svc100_uiRomID].IsDisSFWriteCmd) {
        if (TRUE != LibUsb_ReadFromASIC(0x05F3, &sfWriteEnable)) {
            return FALSE;
        }
        // SF write enable(ubSFWREN)
        if (TRUE != LibUsb_WriteToASIC(0x05F3, 0x06)) {
            return FALSE;
        }
        // Memkey1 low
        if (TRUE != LibUsb_WriteToASIC(0x05F8, 0x12)) {
            return FALSE;
        }
        // MemKey1 high
        if (TRUE != LibUsb_WriteToASIC(0x05F9, 0x12)) {
            return FALSE;
        }
        // memkey2 low
        if (TRUE != LibUsb_WriteToASIC(0x05FA, 0xED)) {
            return FALSE;
        }
        // memkey2 high
        if (TRUE != LibUsb_WriteToASIC(0x05FB, 0xED)) {
            return FALSE;
        }
        // ubsfwrite
        if (TRUE != LibUsb_ReadFromASIC(0x05F5, &sfWriteCommand)) {
            return FALSE;
        }
        if (isNewWPVer) {
            // SF write command
            if (TRUE != LibUsb_WriteToASIC(0x05F5, cbSFLib_Cmd[ubSFLib_CmdID][SFCMD_IDX_PP])) {
                return FALSE;
            }
        } else {
            if (sft == SFT_SST) {
                // SF write command for SST
                if (TRUE != LibUsb_WriteToASIC(0x05F5, 0xAF)) {
                    return FALSE;
                }
            } else {
                if (TRUE != LibUsb_WriteToASIC(0x05F5, 0x02)) {
                    return FALSE;
                }
            }
        }
    }

    if (RomInfo[svc100_uiRomID].IsCompactMode && byMemType == 2) return TRUE;

    // setp 2 :disable write protect @ flash status register (bp0, bp1)
    if (TRUE != LibUsb_WriteToASIC(0x1080, 0x1)) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1091, 0x0)) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1082, 0x06)) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1081, 0x1)) {
        return FALSE;
    }
    if (TRUE != LibUsb_SFWaitReady()) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1091, 0x1)) {
        return FALSE;
    }
    if (TRUE != LibUsb_SFCMDreadStatus()) {
        return FALSE;
    }
    if (sft == SFT_SST) {
        if (TRUE != LibUsb_WriteToASIC(0x1091, 0x0)) {
            return FALSE;
        }
        // Enable-Write-Status-Register(EWSR)
        if (TRUE != LibUsb_WriteToASIC(0x1082, 0x50)) {
            return FALSE;
        }
        if (TRUE != LibUsb_WriteToASIC(0x1081, 0x1)) {
            return FALSE;
        }
        if (TRUE != LibUsb_SFWaitReady()) {
            return FALSE;
        }
        if (TRUE != LibUsb_WriteToASIC(0x1091, 0x1)) {
            return FALSE;
        }
    }
    if (TRUE != LibUsb_WriteToASIC(0x1091, 0x0)) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1082, 0x1)) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1081, 0x1)) {
        return FALSE;
    }
    if (TRUE != LibUsb_SFWaitReady()) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1082, 0x0)) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1081, 0x1)) {
        return FALSE;
    }
    if (TRUE != LibUsb_SFWaitReady()) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1091, 0x1)) {
        return FALSE;
    }
    if (TRUE != LibUsb_SFCMDreadStatus()) {
        return FALSE;
    }
    if (TRUE != LibUsb_WriteToASIC(0x1080, 0x0)) {
        return FALSE;
    }

    if ((svc100_uiRomID == ROM283) || (svc100_uiRomID == ROM275V2) || (svc100_uiRomID == ROM287)) {
        BYTE TempVar = 0;
        LibUsb_ReadFromASIC(0x1006, &TempVar);
        TempVar |= 0x04;
        LibUsb_WriteToASIC(0x1006, TempVar);
        TempVar = 0;
        LibUsb_ReadFromASIC(0x1007, &TempVar);
        TempVar |= 0x04;
        LibUsb_WriteToASIC(0x1007, TempVar);
    }

    return TRUE;
}

BOOL LibUsb_EraseSectorForSerialFlash(LONG addr, SERIAL_FLASH_TYPE sft) {
    // disable serial falsh write protect
    BYTE sectorEraseCode = 0x20;
    switch (sft) {
        case SFT_ST:
            sectorEraseCode = 0xd8;
            break;
        case SFT_SST:
        case SFT_MXIC:
        case SFT_GIGA:
        case SFT_WINBOND:
        case SFT_MXIC_LIKE:
        case SFT_ATMEL_AT25F:
        case SFT_ATMEL_AT25FS:
        case SFT_ATMEL_AT45DB:
            sectorEraseCode = 0x20;
            break;
        case SFT_PMC:
            sectorEraseCode = 0xd7;
            break;
        case SFT_AMIC:
            break;
        case SFT_EON:
            break;
        case SFT_UNKNOW:
            return FALSE;
            break;
            break;
        default:
            break;
    }
    BYTE data = 0x1;
    LibUsb_WriteToASIC(0x1080, data);  // flash mode
    data = 0x0;
    LibUsb_WriteToASIC(0x1091, data);  // CS=0
    data = 0x06;
    LibUsb_WriteToASIC(0x1082, data);  // WREN
    data = 0x01;
    LibUsb_WriteToASIC(0x1081, data);
    LibUsb_SFWaitReady();
    data = 0x1;
    LibUsb_WriteToASIC(0x1091, data);  // CS=1
    // sector erase
    data = 0x0;
    LibUsb_WriteToASIC(0x1091, data);  // CS=0
    data = sectorEraseCode;
    LibUsb_WriteToASIC(0x1082, data);  // for chip sector erase
    // SetRegData(0x1082,0x20);
    data = 0x01;
    LibUsb_WriteToASIC(0x1081, data);
    LibUsb_SFWaitReady();
    // ldata = 0x00;
    data = addr >> 16;
    LibUsb_WriteToASIC(0x1082, data);  // addr
    // SetRegData(0x1082,0x00);
    data = 0x01;
    LibUsb_WriteToASIC(0x1081, data);
    LibUsb_SFWaitReady();
    data = addr >> 8;
    LibUsb_WriteToASIC(0x1082, data);
    // SetRegData(0x1082,SectorNum);
    data = 0x01;
    LibUsb_WriteToASIC(0x1081, data);
    LibUsb_SFWaitReady();
    data = (BYTE)addr;  // ldata = 0x00;
    LibUsb_WriteToASIC(0x1082, data);
    // SetRegData(0x1082,0x00);
    data = 0x01;
    LibUsb_WriteToASIC(0x1081, data);
    LibUsb_SFWaitReady();
    data = 0x1;
    LibUsb_WriteToASIC(0x1091, data);  // CS=1
    // SF_CMDread_Status
    LibUsb_SFWaitReady();  // SF_CMDread_Status(iDevIndex);
    data = 0x0;
    LibUsb_WriteToASIC(0x1080, data);  // flash mode disable
    return TRUE;
}

BOOL LibUsb_SerialFlashErase(SERIAL_FLASH_TYPE sft) {
    BOOL ret = TRUE;
    switch (sft) {
        case SF_WINBOND:
        case SF_PMC:
        case SF_ST:
        case SF_AMIC:
            LibUsb_WriteToASIC(sfModeAddr, 0x1);
            // SF_Set_WEL_Bit
            LibUsb_WriteToASIC(sfCSAddr, 0x0);
            LibUsb_WriteToASIC(sfWriteDataAddr, 0x06);
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
            LibUsb_SFWaitReady();
            LibUsb_WriteToASIC(sfCSAddr, 0x1);
            // chip erase
            LibUsb_WriteToASIC(sfCSAddr, 0x0);
            LibUsb_WriteToASIC(sfWriteDataAddr, 0xc7);  // for PMC chip erase
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
            LibUsb_SFWaitReady();
            LibUsb_WriteToASIC(sfCSAddr, 0x1);
            LibUsb_SFCMDreadStatus();
            ret = LibUsb_WriteToASIC(sfModeAddr, 0x0);
            break;
        case SF_SST:
            LibUsb_WriteToASIC(sfModeAddr, 0x1);  // serial flash mode
            // SF_Set_EWSR_Bit
            LibUsb_WriteToASIC(sfCSAddr, 0x0);                 // chip select
            LibUsb_WriteToASIC(sfWriteDataAddr, 0x50);         // write data
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);  // trigger for write
            LibUsb_SFWaitReady();
            LibUsb_WriteToASIC(sfCSAddr, 0x1);
            // SF_Set_WRSR_Bit
            LibUsb_WriteToASIC(sfCSAddr, 0x0);                 // chip select
            LibUsb_WriteToASIC(sfWriteDataAddr, 0x01);         // write data
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);  // trigger for write
            LibUsb_SFWaitReady();
            LibUsb_WriteToASIC(sfWriteDataAddr, 0x00);
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);  // trigger for write
            LibUsb_SFWaitReady();
            LibUsb_WriteToASIC(sfCSAddr, 0x1);
            // SF_Set_WEL_Bit
            LibUsb_WriteToASIC(sfCSAddr, 0x0);                 // chip select
            LibUsb_WriteToASIC(sfWriteDataAddr, 0x06);         // write data
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);  // trigger for write
            LibUsb_SFWaitReady();
            LibUsb_WriteToASIC(sfCSAddr, 0x1);
            // chip erase
            LibUsb_WriteToASIC(sfCSAddr, 0x0);
            LibUsb_WriteToASIC(sfWriteDataAddr, 0x60);
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
            LibUsb_SFWaitReady();
            LibUsb_WriteToASIC(sfCSAddr, 0x1);
            // SF_CMDread_Status
            LibUsb_SFCMDreadStatus();
            ret = LibUsb_WriteToASIC(sfModeAddr, 0x0);
            break;
        case SF_UNKNOW:
        case SF_MXIC:
        case SF_ATMEL_AT25FS:
        case SF_MXIC_LIKE:
        default:
            LibUsb_WriteToASIC(sfModeAddr, 0x1);
            LibUsb_WriteToASIC(sfCSAddr, 0x0);  // CS#
            // SF_Set_WEL_Bit
            LibUsb_WriteToASIC(sfWriteDataAddr, 0x06);  // write enable cmd
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
            LibUsb_SFWaitReady();
            // disable BP0 BP1
            LibUsb_WriteToASIC(sfCSAddr, 0x1);
            // chip erase
            LibUsb_WriteToASIC(sfCSAddr, 0x0);
            LibUsb_WriteToASIC(sfWriteDataAddr, 0x60);  // chip erase cmd
            // SetRegData(0x1082,0xc7);
            LibUsb_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
            LibUsb_SFWaitReady();
            LibUsb_WriteToASIC(sfCSAddr, 0x1);
            // SF_CMDread_Status
            LibUsb_SFCMDreadStatus();
            ret = LibUsb_WriteToASIC(sfModeAddr, 0x0);
            break;
    }
    return TRUE;
}

BOOL LibUsb_DefGetAsicRomVersion(BYTE romVersion[]) {
    BYTE romString[8];
    memset(romString, 0, 8);
    int i = 0, j = 0;
    for (i = 0; i < ROMSTRADDRCNT; i++) {
        LibUsb_ReadFromROM(RomStringAddr[i], romString);
        for (j = 0; j < ROMCOUNT; j++) {
            if (RomInfo[j].RomStringAddr == RomStringAddr[i]) {
                if (RomInfo[j].IsNewestVer) {
                    if (memcmp(romString, RomInfo[j].RomString, 4) == 0 && romString[5] >= RomInfo[j].RomString[5]) {
                        memcpy(romVersion, romString, 8);
                        svc100_uiRomID = j;  // shawn 2010/05/14 add
                        return TRUE;
                    }
                } else {
                    if (memcmp(romString, RomInfo[j].RomString, 4) == 0 &&  // shawn 2009/06/10 modify
                        romString[5] == RomInfo[j].RomString[5])            // shawn 2010/04/12 only compare 6 bytes
                    {
                        memcpy(romVersion, romString, 8);
                        svc100_uiRomID = j;  // shawn 2010/05/14 add
                        return TRUE;
                    }
                }
            }
        }
    }
    romVersion = 0;
    return FALSE;
}

BOOL LibUsb_GetAsicRomVersion(BYTE romVersion[]) {
    if (TRUE == LibUsb_DefGetAsicRomVersion(romVersion)) {
        if (svc100_uiRomID == ROM276V1) {
            BYTE byTmp1 = 0;
            BYTE byTmp2 = 0;
            LibUsb_ReadFromASIC(0x1185, &byTmp1);
            byTmp1 |= 0x70;
            LibUsb_WriteToASIC(0x1185, byTmp1);
            LibUsb_ReadFromASIC(0x1185, &byTmp2);
            if ((byTmp2 & 0x70) == (byTmp1 & 0x70)) {
                romVersion[4] = 0x31;
            }
        }
        if (svc100_uiRomID == ROM288V1) {
            BYTE byTmp1 = 0;
            BYTE byTmp2 = 0;
            LibUsb_ReadFromASIC(0x101f, &byTmp1);
            if (byTmp1 == 0x89) {
                romVersion[2] = 0x39;
                svc100_uiRomID = ROM289V1;
            } else {
                byTmp1 = 0;
                LibUsb_ReadFromASIC(0x1007, &byTmp1);
                byTmp1 &= 0xDF;
                LibUsb_ReadFromASIC(0x1006, &byTmp2);
                byTmp2 |= 0x20;
                LibUsb_WriteToASIC(0x1007, byTmp1);
                LibUsb_WriteToASIC(0x1006, byTmp2);
                byTmp1 = 0;
                LibUsb_ReadFromASIC(0x1005, &byTmp1);
                if ((byTmp1 & 0x20) == 0x20) {
                    romVersion[4] = 0x31;
                } else {
                    byTmp2 &= 0xDF;
                    LibUsb_WriteToASIC(0x1006, byTmp2);
                }
            }
        }
        if (svc100_uiRomID == ROM271V1) {
            BYTE byTmp1 = 0;
            BYTE byTmp2 = 0;
            BYTE byTmp3 = 0;
            LibUsb_ReadFromASIC(0x100A, &byTmp1);
            byTmp2 = (byTmp1 & 0x10);
            byTmp1 &= 0xEF;
            LibUsb_WriteToASIC(0x100A, byTmp1);
            LibUsb_ReadFromASIC(0x101F, &byTmp3);
            byTmp1 |= byTmp2;
            LibUsb_WriteToASIC(0x100A, byTmp1);
            if (byTmp3 == 0x70) {
                romVersion[2] = 0x30;
                svc100_uiRomID = ROM270V1;
            }
            byTmp1 = 0;
            LibUsb_ReadFromASIC(0x1007, &byTmp1);
            byTmp1 &= 0xF7;
            byTmp2 = 0;
            LibUsb_ReadFromASIC(0x1006, &byTmp2);
            byTmp2 |= 0x08;
            LibUsb_WriteToASIC(0x1007, byTmp1);
            LibUsb_WriteToASIC(0x1006, byTmp2);
            byTmp1 = 0;
            LibUsb_ReadFromASIC(0x1005, &byTmp1);
            if ((byTmp1 & 0x08) == 0x08) {
                romVersion[4] = 0x31;  // 270M, 271M
                byTmp2 &= 0xF7;
                LibUsb_WriteToASIC(0x1006, byTmp2);
            } else {
                byTmp1 = 0;
                LibUsb_ReadFromASIC(0x1007, &byTmp1);
                byTmp1 &= 0xEF;
                byTmp2 = 0;
                LibUsb_ReadFromASIC(0x1006, &byTmp2);
                byTmp2 |= 0x10;
                LibUsb_WriteToASIC(0x1007, byTmp1);
                LibUsb_WriteToASIC(0x1006, byTmp2);
                byTmp1 = 0;
                LibUsb_ReadFromASIC(0x1005, &byTmp1);

                if ((byTmp1 & 0x10) == 0x10) {
                    // 270A, 271A
                    romVersion[4] = 0x30;
                } else {
                    if (svc100_uiRomID == ROM270V1) {
                        romVersion[4] = 0x32;  // 270B
                        byTmp2 &= 0xEF;
                        LibUsb_WriteToASIC(0x1006, byTmp2);
                    }
                }
            }
        }
        if (svc100_uiRomID == ROM281V1) {
            BYTE byTmp1 = 0;
            BYTE byTmp2 = 0;
            BYTE byTmp3 = 0;
            LibUsb_ReadFromASIC(0x100A, &byTmp1);
            byTmp2 = (byTmp1 & 0x10);
            byTmp1 &= 0xEF;
            LibUsb_WriteToASIC(0x100A, byTmp1);
            LibUsb_ReadFromASIC(0x101F, &byTmp3);
            byTmp1 |= byTmp2;
            LibUsb_WriteToASIC(0x100A, byTmp1);
            if (byTmp3 == 0x80) {
                romVersion[2] = 0x30;
                svc100_uiRomID = ROM280V1;
            }
            byTmp1 = 0;
            LibUsb_ReadFromASIC(0x1007, &byTmp1);
            byTmp1 &= 0xF7;
            byTmp2 = 0;
            LibUsb_ReadFromASIC(0x1006, &byTmp2);
            byTmp2 |= 0x08;
            LibUsb_WriteToASIC(0x1007, byTmp1);
            LibUsb_WriteToASIC(0x1006, byTmp2);
            byTmp1 = 0;
            LibUsb_ReadFromASIC(0x1005, &byTmp1);
            if ((byTmp1 & 0x08) == 0x08) {
                romVersion[4] = 0x31;  // 280M, 281M
                byTmp2 &= 0xF7;
                LibUsb_WriteToASIC(0x1006, byTmp2);
            } else {
                romVersion[4] = 0x30;  // 280A, 281A
            }
        }
        return TRUE;
    }
    romVersion = 0;
    return FALSE;
}

BOOL LibUsb_GetParaTableAndCRCAddrFormSF(ULONG *paraTableStartAddr, ULONG *paraTableEndAddr, ULONG *crcAddr) {
    BYTE romVersion[8] = {0};
    if (TRUE != LibUsb_GetAsicRomVersion(romVersion)) return FALSE;

    if (memcmp(romVersion, "231R0", 4) == 0 && romVersion[5] >= 2)  // 231R0_V2
    {
        return FALSE;
    } else if (((memcmp(romVersion, "232R0", 4) == 0 && romVersion[5] == 1)) ||
               ((memcmp(romVersion, "275R0", 4) == 0 && romVersion[5] == 1 && romVersion[6] == 0x30)) ||
               ((memcmp(romVersion, "276R0", 4) == 0 && romVersion[5] == 1)) ||
               ((memcmp(romVersion, "290R0", 4) == 0 && romVersion[5] == 1))) {
        *paraTableStartAddr = 0xC000;
        *paraTableEndAddr = *paraTableStartAddr + 0xF00;
        *crcAddr = *paraTableStartAddr + 0xF00;
    } else if (((memcmp(romVersion, "232R0", 4) == 0 && romVersion[5] == 2)) ||
               ((memcmp(romVersion, "290R0", 4) == 0 && romVersion[5] == 2)) ||
               ((memcmp(romVersion, "286R0", 4) == 0 && romVersion[5] == 1)) ||
               ((memcmp(romVersion, "288R0", 4) == 0 && romVersion[5] == 1)) ||
               ((memcmp(romVersion, "289R0", 4) == 0 && romVersion[5] == 1)) ||
               ((memcmp(romVersion, "270R0", 4) == 0 && romVersion[5] == 1)) ||
               ((memcmp(romVersion, "271R0", 4) == 0 && romVersion[5] == 1)) ||
               ((memcmp(romVersion, "280R0", 4) == 0 && romVersion[5] == 1)) ||
               ((memcmp(romVersion, "281R0", 4) == 0 && romVersion[5] == 1))) {
        BYTE sectorTable[32];
        if (TRUE != LibUsb_ReadFormSF(0x160, sectorTable, sizeof(sectorTable))) return FALSE;

        // pbySectorTable[8] is parameter start address for deivce
        // pbySectorTable[9] is parameter size for deivce
        *paraTableStartAddr = (ULONG)sectorTable[0x08] << 8;
        *paraTableEndAddr = *paraTableStartAddr + ((ULONG)sectorTable[0x09] << 8);
        *crcAddr = (ULONG)sectorTable[0xE] << 8;
    } else if (((memcmp(romVersion, "272R0", 4) == 0) && (romVersion[5] == 1)) ||
               ((memcmp(romVersion, "273R0", 4) == 0) && (romVersion[5] == 1)) ||
               ((memcmp(romVersion, "275R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
               ((memcmp(romVersion, "283R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
               ((memcmp(romVersion, "287R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
               ((memcmp(romVersion, "267R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
               ((memcmp(romVersion, "292R0", 4) == 0) && (romVersion[5] == 1)))  // wei add 292
    {
        // Get Parameter table start address
        // Parameter table start address is stored at 0x16F
        BYTE sectorTable[0x2B];
        if (TRUE != LibUsb_ReadFormSF(0x160, sectorTable, sizeof(sectorTable))) return FALSE;

        *paraTableStartAddr = ((ULONG)sectorTable[0x0F] << 24) + ((ULONG)sectorTable[0x10] << 16) +
                              ((ULONG)sectorTable[0x11] << 8) + sectorTable[0x12];
        ULONG dwParaTableSize = ((ULONG)sectorTable[0x13] << 24) + ((ULONG)sectorTable[0x14] << 16) +
                                ((ULONG)sectorTable[0x15] << 8) + sectorTable[0x16];
        *paraTableEndAddr = *paraTableStartAddr + dwParaTableSize;
        *crcAddr = ((ULONG)sectorTable[0x27] << 24) + ((ULONG)sectorTable[0x28] << 16) +
                   ((ULONG)sectorTable[0x29] << 8) + sectorTable[0x2a];
    } else {
        *paraTableStartAddr = 0x8000;
        *paraTableEndAddr = *paraTableStartAddr + 0x800;
        *crcAddr = 0;
    }
    return TRUE;
}

BOOL LibUsb_GetStringSettingFormSF(BYTE *pbyString, DWORD stringSize, DWORD StringOffset, BOOL bIsCRCProtect) {
    DWORD dwStringAddr = 0;
    ULONG dwParaTableStartAddr = 0;
    ULONG dwParaTableEndAddr = 0;
    ULONG dwCRCStartAddr = 0;

    if (TRUE != LibUsb_GetParaTableAndCRCAddrFormSF(&dwParaTableStartAddr, &dwParaTableEndAddr, &dwCRCStartAddr))
        return FALSE;

    dwStringAddr = StringOffset;
    if (bIsCRCProtect)
        dwStringAddr += dwParaTableStartAddr;
    else
        dwStringAddr += dwCRCStartAddr;

    BYTE pbyStringBuf[0x40] = {0};
    if (TRUE != LibUsb_ReadFormSF(dwStringAddr, pbyStringBuf, sizeof(pbyStringBuf))) return FALSE;

    // Calculate string length
    DWORD dwStringLength = 0;
    if (bIsCRCProtect)
        dwStringLength = (pbyStringBuf[0] - 2) / 2;
    else
        for (; (dwStringLength < 0x40 / 2) && (pbyStringBuf[dwStringLength] != 0xFF); ++dwStringLength)
            ;

    if (stringSize < dwStringLength) return FALSE;

    // Copy string to output buffer
    DWORD i;
    if (bIsCRCProtect) {
        for (i = 0; i < dwStringLength; ++i) pbyString[i] = pbyStringBuf[2 + i * 2];
    } else {
        memcpy(pbyString, pbyStringBuf, dwStringLength);
    }

    return TRUE;
}
}
