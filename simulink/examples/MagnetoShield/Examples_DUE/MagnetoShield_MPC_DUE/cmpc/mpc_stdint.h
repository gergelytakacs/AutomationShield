#ifndef STDINT_H
#define STDINT_H

/* MISRA C 2004 integer numeric typedef */

typedef char char_t;

/* Probably already defined by inttypes.h or types.h*/
# ifndef __int8_t_defined
#  define __int8_t_defined
typedef signed char int8_t;
typedef unsigned char uint8_t;
#endif
# ifndef __int16_t_defined
#  define __int16_t_defined
typedef signed short int16_t;
typedef unsigned short uint16_t;
#endif
# ifndef __int32_t_defined
#  define __int32_t_defined
typedef signed int int32_t;
typedef unsigned int uint32_t;
#endif

#ifndef _SYS_TYPES_H
#if !defined(_INT64_T) && !defined(INT64_MAX)
#define _INT64_T
typedef signed long int64_t;
#endif
#endif

#if !defined(_UINT64_T) && !defined(UINT64_MAX)
#define _UINT64_T
typedef unsigned long uint64_t;
#endif

#endif
