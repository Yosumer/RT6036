/****************************************************************************
*
* FILENAME
*     UPD78types.h
*
* VERSION
*     2.0
*
* DESCRIPTION
*     This PreDefined data types
*
* DATA STRUCTURES
*     1
*
* FUNCTIONS
*     None
*
* HISTORY
*     10/09/26       Ver 1.0 Created by Liugt
*     10/10/26       Ver 1.1 added BYTE_ST
* REMARK
*     None
**************************************************************************/

#ifndef _UPD78TYPES_H_
#define _UPD78TYPES_H_

typedef void              VOID;
typedef void *            PVOID;

typedef signed char       INT8;
typedef signed char *     PINT8;
typedef unsigned char     UINT8;
typedef unsigned char *   PUINT8;

typedef signed   char  INT;
typedef signed   int*  PINT;
typedef unsigned   int  UINT;
typedef unsigned   int*  PUINT;

typedef signed char *     PCHAR;
typedef unsigned char     UCHAR;
typedef unsigned char *   PUCHAR;
typedef signed char *            PSTR;
typedef const signed char *      PCSTR;

typedef short             INT16;
typedef short *           PINT16;
typedef unsigned short    UINT16;
typedef unsigned short *  PUINT16;


typedef signed int        INT32;
typedef signed int *      PINT32;
typedef unsigned int      UINT32;
typedef unsigned int *    PUINT32;

typedef float             FLOAT;
typedef float *           PFLOAT;

typedef double            DOUBLE;
typedef double *          PDOUBLE;

typedef unsigned char       BYTE;
typedef unsigned char *     PBYTE;
//typedef unsigned short      WORD;
typedef unsigned short *    PWORD;
typedef unsigned long       DWORD;
typedef unsigned long *     PDWORD;

typedef union
{
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
    }B;
    unsigned char Byte;
}BYTE_ST;


typedef union
{
	struct
	{
		unsigned bD0:1 ;
		unsigned bD1:1 ;
		unsigned bD2:1 ;
		unsigned bD3:1 ;
		unsigned bD4:1 ;
		unsigned bD5:1 ;
		unsigned bD6:1 ;
		unsigned bD7:1 ;
	} ;
	unsigned char nByte ;
}BITS ;




#endif /* _UPD78TYPES_H_ */
