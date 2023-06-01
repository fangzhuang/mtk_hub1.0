/*
Copyright (c) 2003-2010, Mark Borgerding

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the author nor the names of any contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _KISS_FFT_GUTS_H
#define _KISS_FFT_GUTS_H

/* kiss_fft.h
   defines kiss_fft_scalar as either short or a float type
   and defines
   typedef struct { kiss_fft_scalar r; kiss_fft_scalar i; }kiss_fft_cpx; */
#include "kiss_fft.h"
#include <limits.h>

/*
*  Explanation of macros dealing with complex math:
*
*   C_MUL(m,a,b)         : m = a*b
*   C_FIXDIV( c , div )  : if a fixed point impl., c /= div. noop otherwise
*   C_SUB( res, a,b)     : res = a - b
*   C_SUBFROM( res , a)  : res -= a
*   C_ADDTO( res , a)    : res += a
*/

/* define the fractional bit, bound of fixed point*/
#ifdef FIXED_POINT
#if (FIXED_POINT==32u)
#define FRACBITS 31u /* fractional component will occupy 31 bit & remain one bit for +/- */
#define SAMPPROD int64_t
#define SAMP_MAX 2147483647u /* 2 ^ 31 - 1*/
typedef int32_t fix32_t;
#else
#define FRACBITS 15u
#define SAMPPROD int32_t
#define SAMP_MAX 32767u
typedef int32_t fix32_t;
#endif

#define SAMP_MIN -SAMP_MAX

/* if the product of two point exceeds the bound, report it */
#if defined(CHECK_OVERFLOW)
#define CHECK_OVERFLOW_OP(a,op,b)  \
  if ( (SAMPPROD)(a) op (SAMPPROD)(b) > SAMP_MAX || (SAMPPROD)(a) op (SAMPPROD)(b) < SAMP_MIN ) { \
        fprintf(stderr,"WARNING:overflow @ " __FILE__ "(%d): (%d " #op" %d) = %ld\n",__LINE__,(a),(b),(SAMPPROD)(a) op (SAMPPROD)(b) );  }
#endif

/* scale multiplication */
#define smul(a, b) ( (SAMPPROD) (a) * (b) )

/* e.g. (0.4 * 2^31 + 1 * 2^30) / 2 ^ 31 */
#define sround(x)  (kiss_fft_scalar) (((x) + (1 << (FRACBITS - 1))) >> FRACBITS )

#define S_MUL(a,b) sround( smul(a,b) )

/* complex multiplication*/
#define C_MUL(m,a,b) \
      do{ \
          (m).r = sround( smul((a).r,(b).r) - smul((a).i,(b).i) ); \
          (m).i = sround( smul((a).r,(b).i) + smul((a).i,(b).r) ); \
      }while(0)

/* scalar division: x = x / divs */
#define DIVSCALAR(x,divs) (x) = sround(smul(x, divs)) /* #define DIVSCALAR(x,k) (x) = sround(smul(x, SAMP_MAX / k)) */

/* complex division */
# define C_FIXDIV(c, div) \
    do {\
        DIVSCALAR((c).r, div);  \
        DIVSCALAR((c).i, div); \
    }while (0)

/* e.g., 5 * (2 + 3i) = 5 * 2 + 5 * 3i */
#define C_MULBYSCALAR( c, s ) \
    do{\
        (c).r =  sround(smul((c).r , s));\
        (c).i =  sround(smul((c).i , s)) ;\
    }while(0)

#else  /* not FIXED_POINT*/

#define S_MUL(a,b) ((a) * (b))

#define C_MUL(m,a,b) \
    do{ \
        (m).r = (a).r*(b).r - (a).i*(b).i;\
        (m).i = (a).r*(b).i + (a).i*(b).r;\
    }while(0)

#define C_FIXDIV(c,div) /* NOOP */
#define C_MULBYSCALAR( c, s ) \
    do{ (c).r *= (s);\
        (c).i *= (s);\
    }while(0)

#endif

#ifndef CHECK_OVERFLOW_OP
#define CHECK_OVERFLOW_OP(a,op,b) /* noop */
#endif

#define  C_ADD(res, a, b)\
    do { \
        CHECK_OVERFLOW_OP((a).r, +, (b).r)\
        CHECK_OVERFLOW_OP((a).i, +, (b).i)\
        (res).r=(a).r + (b).r;\
        (res).i=(a).i + (b).i; \
    }while(0)

#define  C_SUB(res, a, b)\
    do { \
        CHECK_OVERFLOW_OP((a).r, -, (b).r)\
        CHECK_OVERFLOW_OP((a).i, -, (b).i)\
        (res).r=(a).r-(b).r;\
        (res).i=(a).i-(b).i; \
    }while(0)

#define C_ADDTO(res , a)\
    do { \
        CHECK_OVERFLOW_OP((res).r, +, (a).r)\
        CHECK_OVERFLOW_OP((res).i, +, (a).i)\
        (res).r += (a).r;\
        (res).i += (a).i;\
    }while(0)

#define C_SUBFROM(res, a)\
    do {\
        CHECK_OVERFLOW_OP((res).r, -, (a).r)\
        CHECK_OVERFLOW_OP((res).i, -, (a).i)\
        (res).r -= (a).r;\
        (res).i -= (a).i; \
    }while(0)


#ifdef FIXED_POINT
/* NOTE: the default setting of the two functions will receive double in phase */
/*e.g., 0.5 (for rounding) + 2 ^ 31 * cos_result */
/* in default, int * double = double, however it is dangerous */
#define KISS_FFT_COS(phase)  floor(.5 + SAMP_MAX * cos(phase))
#define KISS_FFT_SIN(phase)  floor(.5 + SAMP_MAX * sin(phase))
#define HALF_OF(x) ((x) >> 1)
/* remove #elif defined(USE_SIMD) */
#else
#define KISS_FFT_COS(phase) (kiss_fft_scalar) cos(phase)
#define KISS_FFT_SIN(phase) (kiss_fft_scalar) sin(phase)
#define HALF_OF(x) ((x) * .5f)
#endif

#define  kf_cexp(x, phase) \
    do{ \
        (x)->r = KISS_FFT_COS(phase);\
        (x)->i = KISS_FFT_SIN(phase);\
    }while(0)


/* a debugging function */
#define pcpx(c) fprintf(stderr,"%g + %gi\n",(double)((c)->r),(double)((c)->i) )

/* remove  #define KISS_FFT_USE_ALLOCA */

#endif
