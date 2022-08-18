#ifndef _TYPES_H_
#define _TYPES_H_

/****************************************************************************************\
 * This is the header file for the Caiqi usb camera for Linux/Mac						*
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
#include "functional"
#define USB_ORDER_OUT 0
#define USB_ORDER_IN 1
 typedef unsigned long cq_uint64_t;
 typedef unsigned int cq_uint32_t;
 typedef unsigned short cq_uint16_t;
 typedef unsigned char cq_uint8_t;
 
 typedef long cq_int64_t;
 typedef int cq_int32_t;
 typedef short cq_int16_t;
 typedef char cq_int8_t;
 
 typedef char cq_byte_t;
 
 typedef bool cq_bool_t;

// typedef void (*callback_t)(void *);
typedef std::function<void (void *)> callback_t;
 typedef struct USB_ORDER_S
{
	cq_uint8_t			Target;		//0:TGT_DEVICE;1:TGT_INIFC;2:TGT_ENDPT;3:TGT_OTHER
	cq_uint8_t			ReqType;	//0:REQ_STD;1:REQ_CLASS;2:REQ_VENDOR
	cq_uint8_t			Direction;	//0:DIR_TO_DEVICE;1:DIR_FROM_DEVICE
	cq_uint8_t          ReqCode;
	cq_uint16_t         Value;
	cq_uint16_t			Index;

	cq_uint8_t*			pData;
	cq_uint16_t			DataBytes;

	USB_ORDER_S()
	{
		Target=0;
		ReqType=2;
		Direction=0;
		ReqCode=0;
		Value=0;
		Index=0;
		pData=nullptr;
		DataBytes=0;
	}
}USB_ORDER,*PUSB_ORDER;

 struct arbFuncStruct
 {
	 int FuncNum;

	 USB_ORDER order;
 };

 
#endif // _TYPES_H_
