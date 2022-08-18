#ifndef __ROMDATA_H__
#define __ROMDATA_H__

#include "util.h"

namespace usb {

enum { ADDR_9FF8, ADDR_D2B2, ADDR_DFF8, ADDR_BFF8, ADDR_C7F8, ADDR_AFF8, ROMSTRADDRCNT };

enum {
    ROM220,
    ROM225,
    ROM250V1,
    ROM250V2,
    ROM230V1,
    ROM230V2,
    ROM230V3,
    ROM231V1,
    ROM216V1,
    ROM215V1,
    ROM236V1,
    ROM236V2,
    ROM256V1,
    ROM231V2,
    ROM266V1,
    ROM276V1,
    ROM275V1,
    ROM290V1,  // shawn 2010/08/11 add
    ROM290V2,  // shawn 2011/01/31 add
    ROM286V1,  // shawn 2010/12/30 add
    ROM288V1,  // shawn 2011/06/10 add
    ROM289V1,  // shawn 2011/08/23 add
    ROM270V1,  // shawn 2012/02/08 add
    ROM271V1,  // shawn 2012/02/08 add
    ROM280V1,  // Daniel 2012/09/13 add
    ROM281V1,  // Daniel 2012/09/13 add
    ROM232V1,
    ROM232V2,
    ROM272V1,  // Jyh 2015/2/5
    ROM273V0,
    ROM275V2,
    ROM283,  // wei add 283
    ROM292,  // wei add 292
    ROM287,  // wei add 287
    ROM267,  // yiling add 267
    ROMCOUNT
};

typedef struct {
    USHORT RomStringAddr;
    unsigned char RomString[8];
    USHORT SFTypeAddr;
    USHORT CodeVerAddr1;
    USHORT CodeVerAddr2;
    unsigned long Bypass;
    sonix_bool IsNewestVer;      // for Lib compatibility
    sonix_bool IsSWErase;        // shawn 2009/04/03 add
    sonix_bool IsDisSFWriteCmd;  // shawn 2009/04/03 add
    sonix_bool IsGeneric;        // shawn 2010/04/14 add
    sonix_bool IsCompactMode;    // shawn 2010/05/11 add
    sonix_bool IsLCVer;          // sha
} ROM_INFO;

extern USHORT RomStringAddr[ROMSTRADDRCNT];

extern ROM_INFO RomInfo[ROMCOUNT];

extern unsigned int svc100_uiRomID;

typedef enum { DAT_UNKNOW = -1, DAT_FIRST, DAT_SECOND, DSP_ARCH_COUNT } DSP_ARCH_TYPE;

typedef struct {
    USHORT dspIdAddr;
    USHORT sfRdyAddr;
    USHORT usbResetAddr;
    USHORT gpioInputAddr;
    USHORT gpioOutputAddr;
    USHORT gpioOEAddr;
    USHORT sfModeAddr;
    USHORT sfCSAddr;
    USHORT sfWriteDataAddr;
    USHORT sfReadDataAddr;
    USHORT sfReadWriteTriggerAddr;

} DSP_ARCH_INFO;

extern USHORT dspIdAddr;
extern USHORT sfRdyAddr;
extern USHORT usbResetAddr;
extern USHORT gpioInputAddr;
extern USHORT gpioOutputAddr;
extern USHORT gpioOEAddr;
extern USHORT sfModeAddr;
extern USHORT sfCSAddr;
extern USHORT sfWriteDataAddr;
extern USHORT sfReadDataAddr;
extern USHORT sfReadWriteTriggerAddr;

extern DSP_ARCH_INFO g_DspArchInfo[DSP_ARCH_COUNT];
extern USHORT g_CurDspExtendUnitID;
extern DSP_ARCH_INFO g_CurDspArchInfo;

void set_dsp_base_addr(DSP_ARCH_TYPE dat);
}
#endif
