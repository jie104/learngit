#ifndef _MT9P031_H_
#define _MT9P031_H_

/****************************************************************************************\
 * This is the header file for the Caiqi usb camera for Linux/Mac						*
 *                                                                                	*
 * Author              :        alanliu											* 												
 * License             :        GPL Ver 2.0                                       	*
 * Copyright           :        Caiqi Electronics Inc.								*
 * Date written        :        Jan 24, 2021                                    	*
 * Modification Notes  :                                                          	*
 *    1.  Jan 24, 2021                                  						*
 *                                                 				*
 *                      															*
 *                                                                                	*
 \***************************************************************************************/

#include <stdio.h>
#include <string>
#include <list>

#include "../Types.h"
#include "../cyusb.h"
#include "../tagSensor.h"


void RegisterSensor_MT9P031(list<tagSensor>& sensorList);


#endif
