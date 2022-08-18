/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact: Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtx.c
 * \brief Main libdmtx source file
 */

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <ctype.h>
#include <limits.h>
#include <float.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include "dmtx.h"
#include "dmtxstatic.h"

/**
 * Use #include to merge the individual .c source files into a single combined
 * file during preprocessing. This allows the project to be organized in files
 * of like-functionality while still keeping a clean namespace. Specifically,
 * internal functions can be static without losing the ability to access them
 * "externally" from the other source files in this list.
 */

//#include "dmtxencode.c"
//#include "dmtxencodestream.c"
//#include "dmtxencodescheme.c"
//#include "dmtxencodeoptimize.c"
//#include "dmtxencodeascii.c"
//#include "dmtxencodec40textx12.c"
//#include "dmtxencodeedifact.c"
//#include "dmtxencodebase256.c"
//
//#include "dmtxdecode.c"
//#include "dmtxdecodescheme.c"
//
//#include "dmtxmessage.c"
//#include "dmtxregion.c"
//#include "dmtxsymbol.c"
//#include "dmtxplacemod.c"
//#include "dmtxreedsol.c"
//#include "dmtxscangrid.c"
//
//#include "dmtximage.c"
//#include "dmtxbytelist.c"
//#include "dmtxtime.c"
//#include "dmtxvector2.c"
//#include "dmtxmatrix3.c"

char *
dmtxVersion(void)
{
   return DmtxVersion;
}
