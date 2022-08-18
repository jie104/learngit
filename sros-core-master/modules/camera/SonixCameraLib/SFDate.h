#ifndef __SFDATE_H__
#define __SFDATE_H__
#include "util.h"

#define SF_OPT_NA		0x00
#define SF_OPT_SE		0x01
#define SF_OPT_BIGSZ	0x02		//Big size  > 64KB


#define SF_UNKNOW		0
#define SF_MXIC			1
#define SF_ST			2
#define SF_SST			3
#define SF_ATMEL_AT25F		4
#define SF_ATMEL_AT25FS		5
#define SF_ATMEL_AT2DBF		6
#define SF_WINBOND		7
#define SF_PMC			8
#define SF_MXIC_LIKE		9
#define SF_AMIC			10
#define SF_EON			11
#define SF_ESMT			12
#define SF_GIGA			13


#define SFCMD_WREN_MXIC				0x06
#define SFCMD_WRDI_MXIC				0x04
#define SFCMD_RDID_MXIC				0x9f
#define SFCMD_RDSR_MXIC				0x05
#define SFCMD_WRSR_MXIC				0x01
#define SFCMD_READ_MXIC				0x03
#define SFCMD_SE_MXIC				0x20
#define SFCMD_BE_MXIC				0x52
#define SFCMD_CE_MXIC				0x60
#define SFCMD_PP_MXIC				0x02
#define SFCMD_DP_MXIC				0xb9
#define SFCMD_RDP_MXIC				0xab
#define SFCMD_RES_MXIC				0xab
#define SFCMD_REMS_MXIC				0x90

#define SFCMD_WREN_ST				0x06
#define SFCMD_WRDI_ST				0x04
#define SFCMD_RDID_ST				0x9f
#define SFCMD_RDSR_ST				0x05
#define SFCMD_WRSR_ST				0x01
#define SFCMD_READ_ST				0x03
#define SFCMD_PP_ST				0x02
#define SFCMD_SE_ST				0xd8					//different with MXIC
#define SFCMD_BE_ST				0xc7					//different with MXIC
#define SFCMD_DP_ST				0xb9
#define SFCMD_RES_ST				0xab

#define SFCMD_WREN_SST				0x06
#define SFCMD_WRDI_SST				0x04
#define SFCMD_RDSR_SST				0x05
#define SFCMD_WRSR_SST				0x01
#define SFCMD_EWSR_SST				0x50
#define SFCMD_READ_SST				0x03
#define SFCMD_SE_SST				0x20
#define SFCMD_BE_SST				0x52
#define SFCMD_CE_SST				0x60
#define SFCMD_BP_SST				0x02					//byte program
#define SFCMD_AAIP_SST				0xaf					//not support in MXIC
#define SFCMD_REMS_SST				0x90

#define SFCMD_WREN_AT25F			0x06
#define SFCMD_WRDI_AT25F			0x04
#define SFCMD_RDSR_AT25F			0x05
#define SFCMD_WRSR_AT25F			0x01
#define SFCMD_READ_AT25F			0x03
#define SFCMD_PP_AT25F				0x02
#define SFCMD_SE_AT25F				0x52					//different with MXIC
#define SFCMD_CE_AT25F				0x62					//different with MXIC
#define SFCMD_RDID_AT25F			0x15					//different with MXIC

#define SFCMD_WREN_AT25FS			0x06
#define SFCMD_WRDI_AT25FS			0x04
#define SFCMD_RDSR_AT25FS			0x05
#define SFCMD_WRSR_AT25FS			0x01
#define SFCMD_READ_AT25FS			0x03
#define SFCMD_PP_AT25FS				0x02
#define SFCMD_SE_AT25FS				0x20
#define SFCMD_BE_AT25FS				0x52
#define SFCMD_CE_AT25FS				0x60
#define SFCMD_RDID_AT25FS			0x9f

#define SFCMD_WREN_WINBOND			0x06
#define SFCMD_WRDI_WINBOND			0x04
#define SFCMD_RDSR_WINBOND			0x05
#define SFCMD_WRSR_WINBOND			0x01
#define SFCMD_READ_WINBOND			0x03
#define SFCMD_PP_WINBOND			0x02
#define SFCMD_BE_WINBOND			0xd8					//different with MXIC
#define SFCMD_CE_WINBOND			0xc7					//different with MXIC
#define SFCMD_SE_WINBOND			0x20
#define SFCMD_RES_WINBOND			0xab
#define SFCMD_REMS_WINBOND			0x90
#define SFCMD_RDID_WINBOND			0x9f					//W25P80, W25P16, W25P32

#define SFCMD_WREN_PMC				0x06
#define SFCMD_WRDI_PMC				0x04
#define SFCMD_RDSR_PMC				0x05
#define SFCMD_WRSR_PMC				0x01
#define SFCMD_READ_PMC				0x03
#define SFCMD_PP_PMC				0x02
#define SFCMD_SE_PMC				0xd7					//different with MXIC
#define SFCMD_BE_PMC				0xd8					//different with MXIC
#define SFCMD_CE_PMC				0xc7					//different with MXIC
#define SFCMD_RES_PMC				0xab
#define SFCMD_RDID_PMC				0x9f					//Pm25LV010, Pm25LV020, Pm25LV040

#define SFCMD_WREN_AMIC				0x06
#define SFCMD_WRDI_AMIC				0x04
#define SFCMD_RDSR_AMIC				0x05
#define SFCMD_WRSR_AMIC				0x01
#define SFCMD_READ_AMIC				0x03
#define SFCMD_PP_AMIC				0x02
#define SFCMD_SE_AMIC				0xd8					//different with MXIC
#define SFCMD_BE_AMIC				0xc7					//different with MXIC
#define SFCMD_CE_AMIC				0xc7					//different with MXIC
#define SFCMD_RES_AMIC				0xab
#define SFCMD_RDID_AMIC				0x9f					//Pm25LV010, Pm25LV020, Pm25LV040

#define SFCMD_WREN_EON				0x06
#define SFCMD_WRDI_EON				0x04
#define SFCMD_RDSR_EON				0x05
#define SFCMD_WRSR_EON				0x01
#define SFCMD_READ_EON				0x03
#define SFCMD_PP_EON				0x02
#define SFCMD_SE_EON				0x20
#define SFCMD_BE_EON				0x52
#define SFCMD_CE_EON				0x60
#define SFCMD_DP_EON				0xB9
#define SFCMD_RDP_EON				0xAB
#define SFCMD_REMS_EON				0x90
#define SFCMD_RDID_EON				0x9F

// special type
#define SFCMD_RDSR_AT45DB			0xd7
#define SFCMD_READ_AT45DB			0xe8

// manufacturer ID
#define SF_MFRID_CONT				0x7f
#define SF_MFRID_MXIC				0xc2
#define SF_MFRID_ST				0x20
#define SF_MFRID_SST				0xbf
#define SF_MFRID_ATMEL				0x1f
#define SF_MFRID_WINBOND			0xef
#define SF_MFRID_PMC				0x9d
#define SF_MFRID_AMIC				0x37
#define SF_MFRID_EON				0x1C
#define SF_MFRID_ESMT				0x8C
#define SF_MFRID_GIGA				0xC8
//special manufacturer id 
#define SF_MFRID_FH				0x5E
// product type
//#define SF_UNKNOWN				0x00

#define MX25L512					0x01
#define MX25L1005					0x02
#define MX25L2005					0x03
#define MX25L4005					0x04
#define MX25L8005					0x05
#define MX25L1605					0x06
#define MX25L3205					0x07
#define MX25L6405					0x08

#define STM25P05					0x10
#define STM25P10					0x11

#define AT25F512					0x20
#define AT25F1024					0x21
#define AT25F2048					0x22
#define AT25F4096					0x23
#define AT25FS040					0x30
#define AT45DB011					0x40
#define AT45DB041					0x41

#define SST25VF512					0x50
#define SST25VF010					0x51

#define W25P10						0x70
#define W25P20						0x71
#define W25P40						0x72
#define W25P80						0x73
#define W25P16						0x74
#define W25P32						0x75
#define W25B40_BB					0x76
#define W25B40_TB					0x77
#define W25X10						0x78
#define W25X20						0x79
#define W25X40						0x7a
#define W25X80						0x7b

#define PM25LV512					0x80
#define PM25LV010					0x81
#define PM25LV020					0x82
#define PM25LV040					0x83

#define A25L05PT					0x90
#define A25L05PU					0x91
#define A25L10PT					0x92
#define A25L10PU					0x93
#define A25L20PT					0x94
#define A25L20PU					0x95

#define EN25F05						0xA0

#define SF_OPT_NA					0x00
#define SF_OPT_SE					0x01
#define SF_OPT_BIGSZ					0x02		//Big size  > 64KB


namespace usb{

enum
{
	SFCMD_IDX_READ = 0,
	SFCMD_IDX_WREN,
	SFCMD_IDX_PP,
	SFCMD_IDX_WRSR,
	SFCMD_IDX_CE,
	SFCMD_IDX_SE,
	SFCMD_IDX_WRDI
};

enum
{
	SFCMD_INFO_MFR = 0,
	SFCMD_INFO_DEVID1,
	SFCMD_INFO_DEVID2,
	SFCMD_INFO_TYPE,
	SFCMD_INFO_CMD_ID,
	SFCMD_INFO_SF_OPT,
};


extern  BYTE cbSFLib_Cmd[][7];
extern  BYTE cbSFLib_ID[][6];
extern  BYTE cbSFLib_Ver[];

extern BYTE ubSFLib_CmdID;

extern LONG ubSFLib_GetIDSize();

extern BYTE sfManufactureID;
extern BYTE sfDeviceID1;
extern BYTE sfDeviceID2;

}
#endif
