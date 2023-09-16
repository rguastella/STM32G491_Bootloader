/**
 * @file   DrvTypeDefs.h
 * @brief  Type definitions
 *
 * Contains the type definitions
 */

#ifndef _DRV_TYPE_DEFS_H_
#define _DRV_TYPE_DEFS_H_

////////////////////////////////////////////////////////////////////////////////
//
// Required include files.
//
////////////////////////////////////////////////////////////////////////////////
#include <stm32g4xx_hal.h>
////////////////////////////////////////////////////////////////////////////////
//
// Definitions.
//
////////////////////////////////////////////////////////////////////////////////

#define		SOFTWARE_VERSION	"SW_01_00A, 28-6-2023"
#define		CONFIG_USB		1

#ifndef false
   #define false  0                    //!< define false equal 0
#endif

#ifndef true
   #define true   1                    //!< define true equal 1
#endif

#ifndef FALSE
   #define FALSE  0                    //!< define FALSE equal 0
#endif

#ifndef TRUE
   #define TRUE   1                    //!< define TRUE equal 1
#endif

#ifndef NULL
#include <stddef.h>
#endif

// ToDo: Need to comment out this section due to build errors when this header file is included in various other modules.
//#ifndef SUCCESS
//   #define SUCCESS   1                 //!< define SUCCESS equal 1
//#endif

#ifndef FAILURE
   #define FAILURE   0                 //!< define FAILURE equal 0
#endif

//
typedef unsigned char         BOOL;   //!< Boolean definition
typedef unsigned char         UINT8;   //!< unsigned char -> 8 bits
typedef unsigned short int    UINT16;  //!< unsigned short -> 16 bits
typedef unsigned long int     UINT32;  //!< unsigned long -> 32 bits

typedef signed char           INT8;    //!< signed char -> 8 bits
typedef signed short int      INT16;   //!< signed short -> 16 bits
typedef signed long int       INT32;   //!< signed long -> 32 bits
typedef unsigned short			 sys_char_t;
// Floating point types.
typedef  float                FLT32;   //!< float 
typedef  double               FLT64;   //!< double

// Character types. These types are used for data that represent text characters.
typedef  char 				  CHAR;    //!< unsigned char -> 8 bits
typedef  unsigned short int   WCHAR;   //!< unsigned short -> 16 bits

// Untyped data types of known sizes. These are to be used for data whose type 
// is unknown or has no meaning in the context where it is used.
typedef  unsigned char        BIT;     //!< unsigned char -> 8 bits

// Untyped data types of unknow size used to define bit fields.
typedef  unsigned long int    BITS;    //!< unsigned long -> 32 bits
typedef  unsigned char        BOOL;    //!< unsigned char -> 8 bits

// Pointer types.
typedef  void*                ROM_PTR; //!< void pointer
typedef  void*                DATA_PTR;//!< void pointer

// Variant types.
typedef  UINT32               VARIANT; //!< unsigned long -> 32 bits       

/* Function pointer definitions */
typedef void (*pFVoidVoid)(void);      //!< void Function(void)
typedef void (*pFVoidChar)(CHAR data); //!< void Function(CHAR data)
typedef void (*pFVoidInt)(INT16 data); //!< void Function(INT16 data)
typedef void (*pFVoidLong)(INT32 data);//!< void Function(INT32 data)
typedef UINT8 (*pFUINT8Void)(void);    //!< UINT8 Function(void)
                                       
                                       //! void Function(UINT32 data)
typedef void (*pFVoidULong)(UINT32 data);    
                                       //! UINT8 Function(UINT16 data)
typedef UINT8 (*pFUINT8UINT16)(UINT16 Value);
                                       //! UINT8 Function(UINT16* pAddress)
typedef UINT8 (*pFUINT8pUINT16)(UINT16* pAddress);
                                       //! UINT8 Function(UINT16* pAddress, UINT16 Data)
typedef UINT8 (*pFUINT8pUINT16UINT16)(UINT16* pAddress, UINT16 Data);
                                       //! UINT32 Function(UINT32 Data)
typedef UINT32 (*pFULongULong)(UINT32 data);

#define __optimize_level0__      __attribute__((optimize("O0"))) 
#define __optimize_for_size__    __attribute__((optimize("Os"))) 
#define __ext_flash__            __attribute__((section(".PFlashCodeSection"), long_call))
//#define __ext_flash__   __attribute__((section(".PFlashCodeSection")))


//
// Date/time types.
//

typedef struct
{
   UINT8    Seconds;
   UINT8    Minutes;
   UINT8    Hours;
   UINT8    Day;
   UINT8    Date;
   UINT8    Month;
   UINT16   Year;
}_DateTime, *_DateTimePtr;

typedef struct DWordBytes {
    UINT8  Low;   
    UINT8  LowMid;
    UINT8  HighMid;
    UINT8  High;
}_DWordBytes;

typedef struct DWordHalves {
    UINT16  Low;
    UINT16  High;
}_DWordHalves;

typedef union PackedUDWord {
    UINT32       UDWord;
    _DWordHalves Word;
    _DWordBytes  Byte;
}_PackedUDWord;

typedef union PackedDWord {
    INT32        DWord;
    _DWordHalves Word;
    _DWordBytes  Byte;
}_PackedDWord;

typedef struct WordBytes {
    UINT8 Low; 
    UINT8 High;
}_WordBytes;

typedef union PackedUWord {
    UINT16     UWord;
    _WordBytes Byte;
}_PackedUWord;

typedef union PackedWord {
    INT16      Word;
    _WordBytes Byte;
}_PackedWord;

typedef struct Nibbles {
   UINT8 Low  : 4;
   UINT8 High : 4;
}_Nibbles;

typedef union PackedUByte {
    UINT8    UByte;
    _Nibbles Nibble;
}_PackedUByte;

typedef union PackedByte {
    INT8    Byte;
    _Nibbles Nibble;
}_PackedByte;

////////////////////////////////////////////////////////////////////////////////
//
// Macro's
//
////////////////////////////////////////////////////////////////////////////////

                                       //! Return byte 0 of multibyte register
#define GET_BYTE_0( Value )      ((UINT8)((UINT32)Value >> 0) & 0xFF)
                                       //! Return byte 1 of multibyte register
#define GET_BYTE_1( Value )      ((UINT8)((UINT32)Value >> 8) & 0xFF)
                                       //! Return byte 2 of multibyte register
#define GET_BYTE_2( Value )      ((UINT8)((UINT32)Value >> 16) & 0xFF)
                                       //! Return byte 3 of multibyte register
#define GET_BYTE_3( Value )      ((UINT8)((UINT32)Value >> 24) & 0xFF)

                                       //! Return byte 0 of 2 byte register
#define GET_LOW_BYTE( Value )    GET_BYTE_0( Value )
                                       //! Return byte 1 of 2 byte register
#define GET_HIGH_BYTE( Value )   GET_BYTE_1( Value )
 
                                       //! Return low nibble of byte
#define NIBBLE_0(Value) (Value & 0x0F) 
                                       //! Return high nibble of byte
#define NIBBLE_1(Value) ((Value >> 4) & 0x0F)

                                       //! Swap bytes order for 2 byte register
#define BYTE_SWAP_UINT16(a) (UINT16)((((UINT16)(a)&0xFF00)>>8) | \
                                    (((UINT16)(a)&0x00FF)<<8))

                                       //! Swap byte order for 4 byte register
#define BYTE_SWAP_UINT32(a) (UINT32)((((UINT32)(a)&0xFF000000)>>24) | \
                                     (((UINT32)(a)&0x00FF0000)>>8) | \
                                     (((UINT32)(a)&0x0000FF00)<<8) | \
                                     (((UINT32)(a)&0x000000FF)<<24))
////////////////////////////////////////////////////////////////////////////////
//
// Global variables.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Function prototypes.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// End of include file.
//
////////////////////////////////////////////////////////////////////////////////

#endif

