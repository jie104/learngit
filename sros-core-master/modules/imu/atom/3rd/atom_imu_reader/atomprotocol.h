/******************** (C) COPYRIGHT 2015 Atom Corporation *********************
* File Name          : ATOM_PROTOCOL.h
* Author             : Stephen
* Date First Issued  : 2016.10.20
* Description        : Header file
********************************************************************************
* History:

* 2016.10.20: V0.1

********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
#ifndef ATOMPROTOCOL_H
#define ATOMPROTOCOL_H

/*
 ------------------------------------------------------------------------------
 ------------------------------------------------------------------------------
 - MACRO definition start here
 ------------------------------------------------------------------------------
 ------------------------------------------------------------------------------
 */
/*
 ************************************************************************
 * Definition for Protocol
 ************************************************************************
 */

#define MADDR_OUT 0xFF

#define RING_BUFFER_SIZE (1024 * 192)
#define FILE_BUFFER_SIZE (1024 * 192)

#define MAX_RECV_BYTES (2048 * 2)
#define PRINT_SIZE (2 * MAX_RECV_BYTES)

#define PACKET_PL_INDEX 2
#define PACKET_DATA_INDEX 3

#include "atom_macro.h"
enum mode {
    NOT_CONNNECT_MODE,
    CONNECTED_MODE,
    WAKEUP_MODE,
    CONFIG_MODE,
    MEASURE_MODE,
    DEBUG_MODE,
    MAG_CALIBRATE_MODE,
};

enum parserCode {
    ERROR_NOT_ENOUGH_LENGTH = 10,
    ERROR_CRC_FAIL,
    ERROR_DECODE_FAIL,
    ERROR_DUPLICATED_FRAME,
};

enum frameType {
    FRAME_COMPLETE = 1,
    FRAME_ERROR,
    NOT_FRAME,
};
extern void DataPacketParser(u8* pBuffer, u16 dataLen);
extern int AtomCmd_SearchingFrame(u8** pCurrent, u32 rBufferLen, u32* punProcessedLen, u32* pPacketLength);
extern int AtomCmd_Processer(u8* pHeader, u8** ppCustomer, u8* ringBuffer, u32 ringBufferLen, u32* pBytes);
extern void Atom_switchModeReq(char mode);
extern void sendPacket(u8 MADDR, u8 classID, u8 msgID, u8 res, u8* payloadData, u16 payloadLen);
extern void Atom_setDataPacketConfigReq(u8* pData, u8 dataLen);
extern void Atom_setPacktUpdateRateReq(u8* setRate, u8 dataLen_2);

#endif  // ATOMPROTOCOL_H
