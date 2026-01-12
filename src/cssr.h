/*------------------------------------------------------------------------------
* cssr.h : Compact SSR constants, types and function prototypes
*
*          Copyright (C) 2015- by Mitsubishi Electric Corporation, All rights reserved.
*-----------------------------------------------------------------------------*/
#ifndef CSSR_H
#define CSSR_H
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#if !defined(__CYGWIN__)
#include <stdint.h>
#endif

/* constants -----------------------------------------------------------------*/

#define VER_CSSR  "v10"             /* library version */

#define CSSR_MAX_GNSS       16
#define CSSR_MAX_GP         64
#define CSSR_MAX_SV         32
#define CSSR_MAX_NET        32
#define CSSR_MAX_SV_GNSS    40
#define CSSR_MAX_SIG        16
#define CSSR_MAX_LOCAL_SV   32

/* cssr */
#define CSSR_SYS_GPS    0
#define CSSR_SYS_GLO    1
#define CSSR_SYS_GAL    2
#define CSSR_SYS_BDS    3
#define CSSR_SYS_QZS    4
#define CSSR_SYS_SBS    5
#define CSSR_SYS_IRN    6
#define CSSR_SYS_BDS3   7
#define CSSR_SYS_QZS2   8
#define CSSR_SYS_NONE   -1

#define CSSR_TYPE_NUM   14
#define CSSR_TYPE_MASK  1
#define CSSR_TYPE_OC    2
#define CSSR_TYPE_CC    3
#define CSSR_TYPE_CB    4
#define CSSR_TYPE_PB    5
#define CSSR_TYPE_BIAS  6
#define CSSR_TYPE_URA   7
#define CSSR_TYPE_STEC  8
#define CSSR_TYPE_GRID  9
#define CSSR_TYPE_SI    10
#define CSSR_TYPE_OCC   11
#define CSSR_TYPE_ATMOS 12
#define CSSR_TYPE_AUTH  13
#define CSSR_TYPE_AUX   15

#define CSSR_TYPE_GD    0x1f    /* grid definition message */

#ifndef INVALID_VALUE
#define INVALID_VALUE -10000
#endif

typedef struct {
	int iod;
} cssr_t;

#define CSSR_OTYPE_RTCM3	1
#define CSSR_OTYPE_L6		2

#define L6MSG_LENGTH 	256
#define LEN_L6MSG_DATA	1695

#endif /* CSSR_H */
