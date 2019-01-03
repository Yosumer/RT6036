/****************************************************************************
 *
 * FILENAME
 *     UPD78def.h
 *
 * VERSION
 *     1.0
 *
 * DESCRIPTION
 *     This file contains basic macros and inline functions.
 *
 * DATA STRUCTURES
 *     None
 *
 * FUNCTIONS
 *     None
 *
 * HISTORY
 *     10/09/26       Ver 1.0 Created by Liugt
  *
 * REMARK
 *     None
 **************************************************************************/

 

#ifndef _UPD78DEF_H_
#define _UPD78DEF_H_

#ifndef _UPD78TYPES_H_
#include "EFM32_types.h"
#endif


#define PORT_OUT  0
#define PORT_IN   1
#define PORT_HIGH 1
#define PORT_LOW  0


#ifndef CONST
  #define CONST             const
#endif

#ifndef FALSE
  #define FALSE             0
#endif

#ifndef TRUE
  #define TRUE              1
#endif

#ifndef FAIL
  #define FAIL      -1
#endif

#ifndef SUCCEFULL
  #define SUCCEFULL  1
#endif

//----------------------------------------------------------------------------------
//  Macros
//----------------------------------------------------------------------------------

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#define Maximum(a,b)          (a>b ? a : b)
#define Minimum(a,b)          (a<b ? a : b)
#define Middle(a,b)           ((a+b)/2)
#define Equal(a,b)            (a==b ? 1 : 0)
#define NotEqual(a,b)         (a==b ? 0 : 1)
#define GreaterEqual(a,b)     (a>=b ? 1 : 0)
#define LittleEqual(a,b)      (a<=b ? 1 : 0)
	
#define BIT0  0x01
#define BIT1  0x02
#define BIT2  0x04
#define BIT3  0x08
#define BIT4  0x10
#define BIT5  0x20
#define BIT6  0x40
#define BIT7  0x80	
	
/*
static inline UINT16 Swap16(UINT16 val)
{
    return (val<<8) | (val>>8);
}

static inline UINT32 Swap32(UINT32 val)
{
    return (val<<24) | ((val<<8)&0xff0000) | ((val>>8)&0xff00) | (val>>24);
}

static inline UINT16 Get16(PUINT8 addr)
{
    return ((addr[1]<<8) | addr[0]);
}
#ifdef ST_STM32
static inline UINT32 Get32(PUINT8 addr)
{
    return (addr[3]<<24) | (addr[2]<<16) | (addr[1]<<8) | addr[0];
}
#endif
static inline void Set16(UINT16 val, PUINT8 addr)
{
    addr[0] = (UINT8) val;
    addr[1] = (UINT8)(val >> 8);
}

static inline void Set32(UINT32 val, PUINT8 addr)
{
    addr[0] = (UINT8) val;
    addr[1] = (UINT8)(val >> 8);
    addr[2] = (UINT8)(val >> 16);
    addr[3] = (UINT8)(val >> 24);
}
*/
#endif /*  _UPD78DEF_H_ */
