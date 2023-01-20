/*********************************************************************
*
* Software License Agreement ()
*
*  Copyright (c) 2020, HINS, Inc.
*  All rights reserved.
*
* Author: Hu Liankui
* Create Date: 8/9/2020
*********************************************************************/

/**
 * @brief useful tool
 */

#pragma once
#include <inttypes.h>
#include <sys/time.h>
#include <string>

namespace hins {

/**
 * @brief Now ms timestamp
 * @return
 */
static uint64_t Now()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

} // end of namespace hins
