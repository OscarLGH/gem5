/*
 * Copyright (c) 2009 The University of Edinburgh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_POWER_INSTS_FLOATING_HH__
#define __ARCH_POWER_INSTS_FLOATING_HH__

#include "arch/power/insts/static_inst.hh"
#include "base/bitfield.hh"
#include "base/cprintf.hh"
#include <cmath>
#include <cfenv>
#include <quadmath.h>
#include "../regs/misc.hh"

namespace gem5
{

namespace PowerISA
{

/**
 * Base class for floating point operations.
 */
class FloatOp : public PowerStaticInst
{
  protected:

    bool rc;

    /// Constructor
    FloatOp(const char *mnem, MachInst _machInst, OpClass __opClass)
      : PowerStaticInst(mnem, _machInst, __opClass),
        rc(machInst.rc)
    {
    }
  public:
    // Test for NaN (maximum biased exponent & non-zero fraction)
    inline bool
    isNan(uint32_t val_bits) const
    {
        return ((bits(val_bits, 30, 23) == 0xFF) && bits(val_bits, 22, 0));
    }

    inline bool
    isNan(uint64_t val_bits) const
    {
        return ((bits(val_bits, 62, 52) == 0x7FF) && bits(val_bits, 51, 0));
    }

    inline bool
    isNan(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return isNan(val_bits);
    }

    inline bool
    isNan(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return isNan(val_bits);
    }
    
    inline bool
    isNan(__float128 val) const
    {
        void *val_ptr = &val;
        __uint128_t val_bits = *(__uint128_t *) val_ptr;
        return ((((val_bits >> 112) & 0x7FFF) == 0x7FFF) && 
            ((val_bits & 0xffffffffffffffffULL) || 
            (bits((val_bits >> 64), 126 - 64, 112 - 64)))) ;
    }

    // Test for SNaN (NaN with high order bit of fraction set to 0)
    inline bool
    isSnan(uint32_t val_bits) const
    {
        return ((bits(val_bits, 30, 22) == 0x1FE) && bits(val_bits, 22, 0));
    }

    inline bool
    isSnan(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return ((bits(val_bits, 30, 22) == 0x1FE) && bits(val_bits, 22, 0));
    }
    
    inline bool
    isSnan(uint64_t val_bits) const
    {
        return ((bits(val_bits, 62, 51) == (0x7FF7 >> 3)) && bits(val_bits, 51, 0));
    }

    inline bool
    isSnan(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return ((bits(val_bits, 62, 51) == (0x7FF7 >> 3)) && bits(val_bits, 51, 0));
    }
    
    inline bool
    isSnan(__float128 val) const
    {
		void *val_ptr = &val;
        __uint128_t val_bits = *(__uint128_t *) val_ptr;
        return (((val_bits >> 111) & 0x7FFFF) == (0x7FFF7 >> 3));
    }

    // Test for QNaN (NaN with high order bit of fraction set to 1)
    inline bool
    isQnan(uint32_t val_bits) const
    {
        return (bits(val_bits, 30, 22) == 0x1FF);
    }
    
    inline bool
    isQnan(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return (bits(val_bits, 30, 22) == 0x1FF);
    }
    
    inline bool
    isQnan(uint64_t val_bits) const
    {
        return ((bits(val_bits, 62, 51) == (0x7FF8 >> 3)));
    }
    
    inline bool
    isQnan(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return ((bits(val_bits, 62, 51) == (0x7FF8 >> 3)));
    }
    
    // convert SNaN to QNaN
    inline float
    SnantoQnan(float val) const
    {
        void *val_ptr = &val;
        *(uint32_t *) val_ptr |= (0x1 << 22);
        return val;
    }
    
    inline double
    SnantoQnan(double val) const
    {
        void *val_ptr = &val;
        *(uint64_t *) val_ptr |= (0x1ULL << 51);
        return val;
    }
    
    inline double
    SnantoQnan(__float128 val) const
    {
        void *val_ptr = &val;
        __uint128_t mask = 0x1;
        mask <<= 111;
        *(__uint128_t *)val_ptr |= mask;
        return val;
    }

    // Test for infinity (maximum biased exponent and zero fraction)
    inline bool
    isInfinity(uint32_t val_bits) const
    {
        return ((bits(val_bits, 30, 23) == 0xFF) && !bits(val_bits, 22, 0));
    }
    
    inline bool
    isInfinity(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return ((bits(val_bits, 30, 23) == 0xFF) && !bits(val_bits, 22, 0));
    }
    
    inline bool
    isInfinity(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return ((bits(val_bits, 62, 52) == 0x7FF) && !bits(val_bits, 51, 0));
    }
    
    inline bool
    isPosInfinity(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return ((bits(val_bits, 62, 52) == 0x7FF) && !bits(val_bits, 51, 0)) && !bits(val_bits, 63, 63);
    }


    // Test for normalized numbers (biased exponent in the range 1 to 254)
    inline bool
    isNormalized(uint32_t val_bits) const
    {
        return ((bits(val_bits, 30, 23) != 0xFF) && (bits(val_bits, 30, 23) != 0));
    }
    
    inline bool
    isNormalized(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return ((bits(val_bits, 30, 23) != 0xFF) && (bits(val_bits, 30, 23) != 0));
    }

    inline bool
    isNormalized(uint64_t val_bits) const
    {
        return ((bits(val_bits, 62, 52) != 0) && (bits(val_bits, 62, 52) != 0x7FF));
    }

    inline bool
    isNormalized(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return ((bits(val_bits, 62, 52) != 0) && (bits(val_bits, 62, 52) != 0x7FF));
    }

    // Test for denormalized numbers (biased exponent of zero and
    // non-zero fraction)
    inline bool
    isDenormalized(uint32_t val_bits) const
    {
        return (!bits(val_bits, 30, 23) && bits(val_bits, 22, 0));
    }
    
    inline bool
    isDenormalized(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return (!bits(val_bits, 30, 23) && bits(val_bits, 22, 0));
    }
    
    inline bool
    isDenormalized(uint64_t val_bits) const
    {
        return (!bits(val_bits, 62, 52) && bits(val_bits, 51, 0));
    }
    
    inline bool
    isDenormalized(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return (!bits(val_bits, 62, 52) && bits(val_bits, 51, 0));
    }

    // Test for zero (biased exponent of zero and fraction of zero)
    inline bool
    isZero(uint32_t val_bits) const
    {
        return (!bits(val_bits, 30, 23) && !bits(val_bits, 22, 0));
    }
    
    inline bool
    isZero(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return (!bits(val_bits, 30, 23) && !bits(val_bits, 22, 0));
    }
    
    inline bool
    isZero(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return (!bits(val_bits, 62, 52) && !bits(val_bits, 51, 0));
    }

    // Test for negative
    inline bool
    isNegative(uint32_t val_bits) const
    {
        return (bits(val_bits, 31));
    }
    
    inline bool
    isNegative(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return (bits(val_bits, 31));
    }
    
    inline bool
    isNegative(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return (bits(val_bits, 63));
    }

    inline std::tuple<int32_t, bool> si32_CONVERT_FROM_BFP32(float x, int32_t y) const
    {
        void *val_ptr = &x;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        //printf("####output val_bits = %x\n", val_bits);
        bool sign = !!(val_bits & 0x80000000);
        int32_t exp = bits(val_bits, 30, 23);
        uint32_t frac = (bits(val_bits, 22, 0) << 8) | 0x00;
        //printf("####output sign = %x exp = %x frac = %x\n", sign, exp, frac);
        bool sat_flag = false;
        int64_t ret;
        int64_t significand = 0;
        
        if (exp == 255 && frac != 0) {
            ret = 0;
            return std::make_tuple(ret, sat_flag);
        }
        if (exp == 255 && frac == 0) {
            sat_flag = true;
            if (sign == 0x1)
                ret = 0x80000000;
            else
                ret = 0x7fffffff;
            return std::make_tuple(ret, sat_flag);
        }
        
        if ((exp + y - 127) > 30) {
            sat_flag = true;
            if (sign == 0x1)
                ret = 0x80000000;
            else
                ret = 0x7fffffff;
            return std::make_tuple(ret, sat_flag);
        }
        
        if ((exp + y - 127) < 0) {
            return std::make_tuple(0x00000000, false);
        }
        
        significand = (1 << 31) | frac;
        significand = (significand << (31 - (exp + y - 127) - 1));
        
        if (sign == 0)
            return std::make_tuple((int32_t)significand, false);
        else
            return std::make_tuple((int32_t)-significand, false);
    }

    inline std::tuple<uint32_t, bool> ui32_CONVERT_FROM_BFP32(float x, int32_t y) const
    {
        void *val_ptr = &x;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        bool sign = !!(val_bits & 0x80000000);
        int32_t exp = bits(val_bits, 30, 23);
        uint32_t frac = (bits(val_bits, 22, 0) << 8) | 0x00;
        bool sat_flag = false;
        int64_t ret;
        int64_t significand = 0;
        
        if (exp == 255 && frac != 0) {
            return std::make_tuple(0x00000000, sat_flag);
        }
        if (exp == 255 && frac == 0) {
            sat_flag = true;
            if (sign == 0x1)
                ret = 0x00000000;
            else
                ret = 0xffffffff;
            return std::make_tuple(ret, sat_flag);
        }
        
        if ((exp + y - 127) > 31) {
            sat_flag = true;
            if (sign == 0x1)
                ret = 0x00000000;
            else
                ret = 0xffffffff;
            return std::make_tuple(ret, sat_flag);
        }
        
        if ((exp + y - 127) < 0) {
            return std::make_tuple(0x00000000, false);
        }
        
        significand = (1 << 31) | frac;
        significand = (significand << (31 - (exp + y - 127) - 1));
    
        return std::make_tuple((uint32_t)significand, false);
    }

    inline float bfp32_CONVERT_FROM_SI32(int32_t x, uint32_t y) const
    {
    	bool sign = !!(x & 0x80000000);
        int exp = 32 + 127;
        int64_t frac = (x | ((int64_t)sign << 32));
        float ret;
        uint32_t *val_bits = (uint32_t *)&ret;
        if (frac == 0)
            return 0.0;
            
        if (sign == 1) {
            frac = -frac;
        }
        
        while((frac & 0x100000000ULL) == 0) {
            frac = (frac << 1);
            exp--;
        }
        
        bool lsb = bits(frac, 9, 9);
        bool gbit = bits(frac, 8, 8);
        bool xbit = (bits(frac, 7, 0) != 0);
        bool inc = (lsb & gbit) | (gbit & xbit);
        
        int64_t frac_1 = bits(frac, 32, 9);
        frac_1 = frac_1 + inc;
        bool carry_out = !!(frac_1 & (1 << 24));
        frac_1 = (bits(frac_1, 23, 0) << 9) | bits(frac, 8, 0);
        
        if (carry_out)
            exp++;
        
        *val_bits = (sign << 31) | ((exp - y) << 23) | bits(frac_1, 31, 9);
        return ret;
    }

    inline float bfp32_CONVERT_FROM_UI32(uint32_t x, uint32_t y) const
    {
        int exp = 31 + 127;
        int64_t frac = x;
        float ret;
        uint32_t *val_bits = (uint32_t *)&ret;
        if (frac == 0)
            return 0.0;
            
        while((frac & 0x80000000) == 0) {
            frac = (frac << 1);
            exp--;
        }
        
        bool lsb = bits(frac, 8, 8);
        bool gbit = bits(frac, 7, 7);
        bool xbit = (bits(frac, 6, 0) != 0);
        bool inc = (lsb & gbit) | (gbit & xbit);
        
        int64_t frac_1 = bits(frac, 31, 8);
        frac_1 = frac_1 + inc;
        bool carry_out = !!(frac_1 & (1 << 24));
        frac_1 = (bits(frac_1, 23, 0) << 8) | bits(frac, 7, 0);
        
        if (carry_out)
            exp++;
        
        *val_bits = ((exp - y) << 23) | bits(frac_1, 30, 8);
        return ret;
    }
    
    static float bfp32_CONVERT_FROM_BFP64(double x)
    {
        void *val_ptr = &x;
        uint32_t y;
        float *ret_ptr = (float *)&y;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        uint64_t sign = bits(val_bits, 63, 63);
        uint64_t exponent = bits(val_bits, 62, 52);
        uint64_t fraction = ((1ULL << 52) | bits(val_bits, 51, 0));
        
        if ((exponent == 0) && (bits(fraction, 51, 0) != 0)) {
			exponent = 1;
			fraction &= ~(1ULL << 52);
		}
		
		if ((exponent < 897) && (fraction != 0)) {
			int shift = (897 - exponent);
			if (shift > 64)
				fraction = 0;
			else
				fraction = (fraction >> (897 - exponent));
			exponent = 896;
		}
		
		y = ((sign << 31) | (bits(exponent, 10, 10) << 30) | (bits(exponent, 6, 0) << 23) | bits(fraction, 51, 29));   
		return *ret_ptr;
    }
    
    inline double CONVERT_FROM_BFP32(float x) const
    {
        uint64_t y;
        double *ret_ptr = (double *)&y;
        void *val_ptr = &x;
        uint32_t val_bits = *(uint32_t *)val_ptr;
        bool sign = !!(val_bits & 0x80000000);
        int32_t exp = bits(val_bits, 30, 23);
        uint32_t frac = bits(val_bits, 22, 0);

        if (isNan(x)) {
			y = (((uint64_t)sign) << 63) | (((uint64_t)frac) << 40); 
		} else {
			*ret_ptr = (double)x;
		}

		return *ret_ptr;
    }
    
    inline double ConvertSPtoDP_NS(float x) const
    {
        void *val_ptr = &x;
        uint64_t y;
        double *ret_ptr = (double *)&y;
        uint64_t val_bits = *(uint32_t *) val_ptr;
        uint64_t sign = bits(val_bits, 31, 31);
        uint64_t exponent = (bits(val_bits, 30, 30) << 10) | ((~bits(val_bits, 30, 30) & 0x1) << 9) | ((~bits(val_bits, 30, 30) & 0x1) << 8) | ((~bits(val_bits, 30, 30) & 0x1) << 7) | bits(val_bits, 29, 23);
        uint64_t fraction = (bits(val_bits, 22, 0) << 29);

        if (bits(val_bits, 30, 23) == 255) {
			exponent = 2047;
		}

		if ((bits(val_bits, 30, 23) == 0) && (fraction == 0)) {
			exponent = 0;
		}

		if ((bits(val_bits, 30, 23) == 0) && (fraction != 0)) {
			exponent = 897;
			do {
				fraction = fraction << 1;
				exponent--;
			} while (!(fraction & (1ULL << 52)));
		}
		
		y = ((sign << 63) | (bits(exponent, 11, 0) << 52) | bits(fraction, 51, 0));   
		return *ret_ptr;
    }

    inline float bfp32_ROUND_TO_INTEGER_FLOOR(float x) const
    {
        return floor(x);
    }

    inline float bfp32_ROUND_TO_INTEGER_NEAR(float x) const
    {
        return round(x);
    }

    inline float bfp32_ROUND_TO_INTEGER_CEIL(float x) const
    {
        return ceil(x);
    }

    inline float bfp32_ROUND_TO_INTEGER_TRUNC(float x) const
    {
        return trunc(x);
    }

    inline float bfp32_ROUND_TO_INTEGER_NEAR_AWAY(float x) const
    {
        if (x > 0)
            return trunc(x + 0.5);
        else
            return trunc(x - 0.5);
    }

    inline float bfp32_POWER2_ESTIMATE(float x) const
    {
        return exp2(x);
    }

    inline float bfp32_LOG_BASE2_ESTIMATE(float x) const
    {
        return log2(x);
    }

    inline float bfp32_RECIPROCAL_ESTIMATE(float x) const
    {
        double real = (double)1.0 / (double)x;
        return real * (double)257 / (double)256;
    }

    inline float bfp32_RECIPROCAL_SQRT_ESTIMATE(float x) const
    {
        return 1.0 / sqrt(x);
    }
    
    inline double bfp64_ROUND_TO_INTEGER_FLOOR(double x) const
    {
        return floor(x);
    }

    inline double bfp64_ROUND_TO_INTEGER_NEAR(double x) const
    {
        return round(x);
    }

    inline double bfp64_ROUND_TO_INTEGER_CEIL(double x) const
    {
        return ceil(x);
    }

    inline double bfp64_ROUND_TO_INTEGER_TRUNC(double x) const
    {
        return trunc(x);
    }

    inline double bfp64_ROUND_TO_INTEGER_NEAR_AWAY(double x) const
    {
        if (x > 0)
            return trunc(x + 0.5);
        else
            return trunc(x - 0.5);
    }

    inline double bfp64_POWER2_ESTIMATE(double x) const
    {
        return exp2(x);
    }

    inline double bfp64_LOG_BASE2_ESTIMATE(double x) const
    {
        return log2(x);
    }

    inline double bfp64_RECIPROCAL_ESTIMATE(double x) const
    {
        double real = (double)1.0 / x;
        return real * (double)257 / (double)256;
    }

    inline double bfp64_RECIPROCAL_SQRT_ESTIMATE(double x) const
    {
        return (double)1.0 / sqrt(x);
    }

    std::tuple<int64_t, Fpscr> bfp64_CONVERT_TO_INT64(double x, Fpscr fpscr) const
    {
        Fpscr fpscr_flag = 0;
        __int128_t tmp;
        int64_t ret;
        if (isNan(x)) {
            ret = 0x8000000000000000LL;
            fpscr_flag.vxcvi = 1;
            if (isSnan(x))
                fpscr_flag.vxsnan = 1;
        } else {
            tmp = x;
            ret = x;
            
            if (tmp > 0x7fffffffffffffffLL) {
                ret = 0x7fffffffffffffffLL;
                fpscr_flag.vxcvi = 1;
                printf("convert si64:%f to %llx\n", x, ret);
            }

            __int128_t val = 0xffffffffffffffffULL;
            val = val << 64;
            val |= 0x8000000000000000ULL;
            if (tmp < val) {
                ret = 0x8000000000000000LL;
                fpscr_flag.vxcvi = 1;
                printf("convert si64:%f to %llx\n", x, ret);
            }
        }
        fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
    }

    std::tuple<uint64_t, Fpscr> bfp64_CONVERT_TO_UINT64(double x, Fpscr fpscr) const
    {
        Fpscr fpscr_flag = 0;
        __int128_t tmp;
        uint64_t ret;
        if (isNan(x)) {
            ret = 0x0000000000000000;
            fpscr_flag.vxcvi = 1;
            if (isSnan(x))
                fpscr_flag.vxsnan = 1;
        } else {
            tmp = x;
            ret = x;
            if (tmp > 0xffffffffffffffffULL) {
                ret = 0xffffffffffffffffULL;
                fpscr_flag.vxcvi = 1;
                printf("convert ui64:%f to %llx\n", x, ret);
            }

            if (tmp < 0) {
                ret = 0;
                fpscr_flag.vxcvi = 1;
                printf("convert ui64:%f to %llx\n", x, ret);
            }
        }
        fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
    }

    std::tuple<int32_t, Fpscr> bfp64_CONVERT_TO_INT32(double x, Fpscr fpscr) const
    {
        Fpscr fpscr_flag = 0;
        int64_t tmp;
        int32_t ret;
        if (isNan(x)) {
            ret = 0x80000000;
            fpscr_flag.vxcvi = 1;
            if (isSnan(x))
                fpscr_flag.vxsnan = 1;
        } else {
            tmp = x;
            ret = x;
            if (tmp > 0x7fffffff) {
                ret = 0x7fffffff;
                fpscr_flag.vxcvi = 1;
                printf("convert si32:%f to %llx\n", x, ret);
            }

            if (tmp < -0x80000000LL) {
                ret = 0x80000000;
                fpscr_flag.vxcvi = 1;
                printf("convert si32:%f to %llx\n", x, ret);
            }
        }
        fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
    }

    std::tuple<uint32_t, Fpscr> bfp64_CONVERT_TO_UINT32(double x, Fpscr fpscr) const
    {
        Fpscr fpscr_flag = 0;
        int64_t tmp;
        uint32_t ret;
        if (isNan(x)) {
            ret = 0x00000000;
            fpscr_flag.vxcvi = 1;
            if (isSnan(x))
                fpscr_flag.vxsnan = 1;
        } else {
            tmp = x;
            ret = x;
           
            if (tmp > 0xffffffff) {
                ret = 0xffffffff;
                fpscr_flag.vxcvi = 1;
                printf("convert ui32:%f to %llx\n", x, ret);
            }

            if (tmp < 0) {
                ret = 0;
                fpscr_flag.vxcvi = 1;
                printf("convert ui32:%f to %llx\n", x, ret);
            }
        }
        fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
    }

	std::tuple<__float128, Fpscr> bfp128_ADD(__float128 x, __float128 y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		fp_before_cal(fpscr);
		__float128 ret = x + y;
		std::tie(ret, fpscr_flag) = get_floating_exception(x, y, fpscr_flag, ret);

		if ((isInfinity((double)x)) && isInfinity((double)y) && (isNegative((double)x) ^ isNegative((double)y)))
			fpscr_flag.vxisi = 1;

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp64_ADD(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		uint64_t dQNan = 0x7ff8000000000000;
		fp_before_cal(fpscr);
		__float128 tmp = (__float128)x + (__float128)y;
		double ret = tmp;
		std::tie(ret, fpscr_flag) = get_floating_exception(x, y, fpscr_flag, ret);

		if ((isInfinity((double)x)) && isInfinity((double)y) && (isNegative((double)x) ^ isNegative((double)y))) {
			fpscr_flag.vxisi = 1;
			ret = *(double *)&dQNan;
		}

		if (fpscr_flag.fi) {
			if (ret > 0.0) {
				if ((__float128) ret - tmp > 0.0)
					fpscr_flag.fr = 1;
				if ((ret == x && y < 0.0) || (ret == y && x < 0.0)) {
					fpscr_flag.fr = 1;
				}
			}
			
			if (ret < 0.0) {
				if ((__float128) ret - tmp < 0.0)
					fpscr_flag.fr = 1;
				if ((ret == x && y > 0.0) || (ret == y && x > 0.0)) {
					fpscr_flag.fr = 1;
				}
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp32_ADD(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		uint64_t dQNan = 0x7ff8000000000000;
		fp_before_cal(fpscr);
		__float128 tmp = (__float128)x + (__float128)y;
		double ret = (float)tmp;
		std::tie(ret, fpscr_flag) = get_floating_exception(x, y, fpscr_flag, ret);

		if ((isInfinity((double)x)) && isInfinity((double)y) && (isNegative((double)x) ^ isNegative((double)y))) {
			fpscr_flag.vxisi = 1;
			ret = *(double *)&dQNan;
		}
		
		if (fpscr_flag.fi) {
			if (ret > 0.0) {
				if ((__float128) ret - tmp > 0.0)
					fpscr_flag.fr = 1;
				if ((ret == x && y < 0.0) || (ret == y && x < 0.0)) {
					fpscr_flag.fr = 1;
				}
			}
			
			if (ret < 0.0) {
				if ((__float128) ret - tmp < 0.0)
					fpscr_flag.fr = 1;
				if ((ret == x && y > 0.0) || (ret == y && x > 0.0)) {
					fpscr_flag.fr = 1;
				}
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		ret = (float)ret;
		return std::make_tuple(ret, fpscr);
	}
    
    std::tuple<__float128, Fpscr> bfp128_MULTIPLY(__float128 x, __float128 y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		fp_before_cal(fpscr);
		__float128 ret = x * y;
		std::tie(ret, fpscr_flag) = get_floating_exception(x, y, fpscr_flag, ret);
		uint64_t dQNan = 0x7ff8000000000000;

		if ((isInfinity((double)x) && isZero((double)y)) || (isInfinity((double)y) && isZero((double)x))) {
			fpscr_flag.vximz = 1;
			ret = *(double *)&dQNan;
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp64_MULTIPLY(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		fp_before_cal(fpscr);
		__float128 tmp = (__float128)x * (__float128)y;
		double ret = tmp;
		std::tie(ret, fpscr_flag) = get_floating_exception(x, y, fpscr_flag, ret);
		uint64_t dQNan = 0x7ff8000000000000;

		if ((isInfinity((double)x) && isZero((double)y)) || (isInfinity((double)y) && isZero((double)x))) {
			fpscr_flag.vximz = 1;
			ret = *(double *)&dQNan;
		}
		
		if (fpscr_flag.fi) {
			if (ret > 0.0) {
				if ((__float128) ret - tmp > 0.0)
					fpscr_flag.fr = 1;
			}
			
			if (ret < 0.0) {
				if ((__float128) ret - tmp < 0.0)
					fpscr_flag.fr = 1;
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp32_MULTIPLY(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		fp_before_cal(fpscr);
		__float128 tmp = (__float128)x * (__float128)y;
		double ret = (float)tmp;
		std::tie(ret, fpscr_flag) = get_floating_exception(x, y, fpscr_flag, ret);
		uint64_t dQNan = 0x7ff8000000000000;

		if ((isInfinity((double)x) && isZero((double)y)) || (isInfinity((double)y) && isZero((double)x))) {
			fpscr_flag.vximz = 1;
			ret = *(double *)&dQNan;
		}
		
		if (fpscr_flag.fi) {
			if (ret > 0.0) {
				if ((__float128) ret - tmp > 0.0)
					fpscr_flag.fr = 1;
			}
			
			if (ret < 0.0) {
				if ((__float128) ret - tmp < 0.0)
					fpscr_flag.fr = 1;
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		ret = (float)ret;
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp32_DIV(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		uint64_t dQNan = 0x7ff8000000000000;

		fp_before_cal(fpscr);
		__float128 tmp = (__float128)x / (__float128)y;
		double ret = (float)tmp;
		std::tie(ret, fpscr_flag) = get_floating_exception(x, y, fpscr_flag, ret);

		if (isInfinity(x) && isInfinity(y)) {
			fpscr_flag.vxidi = 1;
			ret = *(double *)&dQNan;
		}
			
		if (isZero(x) && isZero(y)) {
			fpscr_flag.vxzdz = 1;
			ret = *(double *)&dQNan;
		}
		
		if (fpscr_flag.fi) {
			if (ret > 0.0) {
				if ((__float128) ret - tmp > 0.0)
					fpscr_flag.fr = 1;
			}
			
			if (ret < 0.0) {
				if ((__float128) ret - tmp < 0.0)
					fpscr_flag.fr = 1;
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		ret = (float)ret;
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp64_DIV(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		uint64_t dQNan = 0x7ff8000000000000;

		fp_before_cal(fpscr);
		__float128 tmp = (__float128)x / (__float128)y;
		double ret = tmp;
		std::tie(ret, fpscr_flag) = get_floating_exception(x, y, fpscr_flag, ret);

		if (isInfinity(x) && isInfinity(y)) {
			fpscr_flag.vxidi = 1;
			ret = *(double *)&dQNan;
		}
			
		if (isZero(x) && isZero(y)) {
			fpscr_flag.vxzdz = 1;
			ret = *(double *)&dQNan;
		}
		
		if (fpscr_flag.fi) {
			if (ret > 0.0) {
				if ((__float128) ret - tmp > 0.0)
					fpscr_flag.fr = 1;
			}
			
			if (ret < 0.0) {
				if ((__float128) ret - tmp < 0.0)
					fpscr_flag.fr = 1;
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
	
	
	
	std::tuple<__float128, Fpscr> bfp128_MULTIPLY_ADD(__float128 x, __float128 y, __float128 z, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		fp_before_cal(fpscr);
		__float128 ret;
		std::tie(ret, fpscr_flag) = bfp128_MULTIPLY(x, y, fpscr_flag);
		std::tie(ret, fpscr_flag) = bfp128_ADD(ret, z, fpscr_flag);
		fpscr = update_fpscr(fpscr, fpscr_flag);

		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp64_MULTIPLY_ADD(double x, double y, double z, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		double ret;
		uint64_t dQNan = 0x7ff8000000000000;
		__float128 tmp;
		
		if (isSnan(x) || isSnan(y) || isSnan(z)) {
			fpscr_flag.vxsnan = 1;
		} 
		if (isInfinity(x) && isZero(y)) {
			fpscr_flag.vximz = 1;
		} else if (isInfinity(y) && isZero(x)) {
			fpscr_flag.vximz = 1;
		} else if (isInfinity(x * y) && isInfinity(z) && (isNegative(x * y) ^ isNegative(z))) {
			fpscr_flag.vxisi = 1;
		}
		
		if (isQnan(x)) {
			ret = x;
		} else if (isSnan(x)) {
			ret = SnantoQnan(x);
		} else if (isQnan(z)) {
			ret = z;
		} else if (isSnan(z)) {
			ret = SnantoQnan(z);
		} else if (isQnan(y)) {
			ret = y;
		} else if (isSnan(y)) {
			ret = SnantoQnan(y);
		} else if (isInfinity(x) && isZero(y)) {
			ret = *(double *)&dQNan;
		} else if (isInfinity(y) && isZero(x)) {
			ret = *(double *)&dQNan;
		} else if (isInfinity(x * y) && isInfinity(z) && (isNegative(x * y) ^ isNegative(z))) {
			ret = *(double *)&dQNan;
		} else {
			fp_before_cal(fpscr);
			tmp = (__float128)x * (__float128)y + (__float128)z;
			ret = tmp;
			fpscr_flag = get_floating_exception_0(fpscr_flag);

            if (fpscr_flag.fi) {
                if (ret > 0.0) {
                    if ((__float128) ret - tmp > 0.0)
                        fpscr_flag.fr = 1;
                    if ((ret == x * y && z < 0.0) || (ret == z && x * y < 0.0)) {
                        fpscr_flag.fr = 1;
                    }
                }
                
                if (ret < 0.0) {
                    if ((__float128) ret - tmp < 0.0)
                        fpscr_flag.fr = 1;
                    if ((ret == x * y && z > 0.0) || (ret == z && x * y > 0.0)) {
                        fpscr_flag.fr = 1;
                    }
                }
            }
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp32_MULTIPLY_ADD(double x, double y, double z, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		double ret;
		uint64_t dQNan = 0x7ff8000000000000;
		__float128 tmp;
		
		if (isSnan(x) || isSnan(y) || isSnan(z)) {
			fpscr_flag.vxsnan = 1;
		} 
		if (isInfinity(x) && isZero(y)) {
			fpscr_flag.vximz = 1;
		} else if (isInfinity(y) && isZero(x)) {
			fpscr_flag.vximz = 1;
		} else if (isInfinity(x * y) && isInfinity(z) && (isNegative(x * y) ^ isNegative(z))) {
			fpscr_flag.vxisi = 1;
		}
		
		if (isQnan(x)) {
			ret = x;
		} else if (isSnan(x)) {
			ret = SnantoQnan(x);
		} else if (isQnan(z)) {
			ret = z;
		} else if (isSnan(z)) {
			ret = SnantoQnan(z);
		} else if (isQnan(y)) {
			ret = y;
		} else if (isSnan(y)) {
			ret = SnantoQnan(y);
		} else if (isInfinity(x) && isZero(y)) {
			ret = *(double *)&dQNan;
		} else if (isInfinity(y) && isZero(x)) {
			ret = *(double *)&dQNan;
		} else if (isInfinity(x * y) && isInfinity(z) && (isNegative(x * y) ^ isNegative(z))) {
			ret = *(double *)&dQNan;
		} else {
			fp_before_cal(fpscr);
			tmp = (__float128)x * (__float128)y + (__float128)z;
			ret = (float)tmp;
			fpscr_flag = get_floating_exception_0(fpscr_flag);
            if (fpscr_flag.fi) {
                if (ret > 0.0) {
                    if ((__float128) ret - tmp > 0.0)
                        fpscr_flag.fr = 1;
                    if ((ret == x * y && z < 0.0) || (ret == z && x * y < 0.0)) {
                        fpscr_flag.fr = 1;
                    }
                }
                
                if (ret < 0.0) {
                    if ((__float128) ret - tmp < 0.0)
                        fpscr_flag.fr = 1;
                    if ((ret == x * y && z > 0.0) || (ret == z && x * y > 0.0)) {
                        fpscr_flag.fr = 1;
                    }
                }
            }
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		ret = (float)ret;
		return std::make_tuple(ret, fpscr);
	}
	
	double bfp_NEGATE(double x) const
	{
		if (isNan(x))
			return x;
		return -x;
	}
	
	std::tuple<double, Fpscr> bfp64_MAX(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		double ret;

		if (isSnan(x) || isSnan(y)) {
			fpscr_flag.vxsnan = 1;
			fpscr_flag.vx = 1;
			fpscr_flag.fx = 1;
		}
		
		if (isQnan(x) && !isNan(y)) {
			ret = y;
		} else if (isQnan(x)) {
			ret = x;
		} else if (isSnan(x)) {
			ret = SnantoQnan(x);
		} else if (isQnan(y)) {
			ret = x;
		} else if (isSnan(y)) {
			ret = SnantoQnan(y);
		} else {
			if (isZero(x) && isZero(y)) {
				if ((isNegative(x) && !isNegative(y))) {
					ret = y;
				} else if ((!isNegative(x) && isNegative(y))) {
					ret = x;
				} else {
					ret = x;
				}
			} else {
				ret = x > y ? x : y;
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<double, Fpscr> bfp64_MIN(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		double ret;

		if (isSnan(x) || isSnan(y)) {
			fpscr_flag.vxsnan = 1;
			fpscr_flag.vx = 1;
			fpscr_flag.fx = 1;
		}
		
		if (isQnan(x) && !isNan(y)) {
			ret = y;
		} else if (isQnan(x)) {
			ret = x;
		} else if (isSnan(x)) {
			ret = SnantoQnan(x);
		} else if (isQnan(y)) {
			ret = x;
		} else if (isSnan(y)) {
			ret = SnantoQnan(y);
		} else {
			if (isZero(x) && isZero(y)) {
				if ((isNegative(x) && !isNegative(y))) {
					ret = x;
				} else if ((!isNegative(x) && isNegative(y))) {
					ret = y;
				} else {
					ret = y;
				}
			} else {
				ret = x < y ? x : y;
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}

    std::tuple<Fpscr, uint32_t> bfp64_COMPARE_ORDER(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
        uint32_t c;

        c = makeCRField(x, y);
        fpscr.fprf.fpcc = c;

		if (isSnan(x) || isSnan(y)) {
			fpscr_flag.vxsnan = 1;
            fpscr_flag.vxvc = 1;
			fpscr_flag.vx = 1;
			fpscr_flag.fx = 1;
		} else if (isQnan(x) || isQnan(y)) {
			fpscr_flag.vxvc = 1;
			fpscr_flag.fx = 1;
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(fpscr, c);
	}

    std::tuple<Fpscr, uint32_t> bfp64_COMPARE_UNORDER(double x, double y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
        uint32_t c;

        c = makeCRField(x, y);
        fpscr.fprf.fpcc = c;

		if (isSnan(x) || isSnan(y)) {
			fpscr_flag.vxsnan = 1;
			fpscr_flag.vx = 1;
			fpscr_flag.fx = 1;
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(fpscr, c);
	}
	
	std::tuple<float, Fpscr> bfp32_MAX(float x, float y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		float ret;

		if (isSnan(x) || isSnan(y)) {
			fpscr_flag.vxsnan = 1;
			fpscr_flag.vx = 1;
			fpscr_flag.fx = 1;
		}
		
		if (isQnan(x) && !isNan(y)) {
			ret = y;
		} else if (isQnan(x)) {
			ret = x;
		} else if (isSnan(x)) {
			ret = SnantoQnan(x);
		} else if (isQnan(y)) {
			ret = x;
		} else if (isSnan(y)) {
			ret = SnantoQnan(y);
		} else {
			if (isZero(x) && isZero(y)) {
				if ((isNegative(x) && !isNegative(y))) {
					ret = y;
				} else if ((!isNegative(x) && isNegative(y))) {
					ret = x;
				} else {
					ret = x;
				}
			} else {
				ret = x > y ? x : y;
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<float, Fpscr> bfp32_MIN(float x, float y, Fpscr fpscr) const
    {
		Fpscr fpscr_flag = 0;
		float ret;

		if (isSnan(x) || isSnan(y)) {
			fpscr_flag.vxsnan = 1;
			fpscr_flag.vx = 1;
			fpscr_flag.fx = 1;
		}
		
		if (isQnan(x) && !isNan(y)) {
			ret = y;
		} else if (isQnan(x)) {
			ret = x;
		} else if (isSnan(x)) {
			ret = SnantoQnan(x);
		} else if (isQnan(y)) {
			ret = x;
		} else if (isSnan(y)) {
			ret = SnantoQnan(y);
		} else {
			if (isZero(x) && isZero(y)) {
				if ((isNegative(x) && !isNegative(y))) {
					ret = x;
				} else if ((!isNegative(x) && isNegative(y))) {
					ret = y;
				} else {
					ret = y;
				}
			} else {
				ret = x < y ? x : y;
			}
		}

		fpscr = update_fpscr(fpscr, fpscr_flag);
		return std::make_tuple(ret, fpscr);
	}
    
    Fpscr fp_before_cal(Fpscr fpscr) const
    {
		//fpscr.fi = 0;
        //fpscr.fx = 0;
        //fpscr.ox = 0;
        //fpscr.zx = 0;
        //fpscr.vx = 0;
        //fpscr.vxsnan = 0;
        fpscr.fr = 0;
        fpscr.fi = 0;

        switch (fpscr.rn) {
            case 0x0:
                fesetround(FE_TONEAREST);
                break;
            case 0x1:
                fesetround(FE_TOWARDZERO);
                break;
            case 0x2:
                fesetround(FE_UPWARD);
                break;
            case 0x3:
                fesetround(FE_DOWNWARD);
                break;
        }
        std::feclearexcept(FE_ALL_EXCEPT);
        return fpscr;
    }
    
    std::tuple<double, Fpscr> round_current(double val, Fpscr fpscr) const
    {
		double ret;
		switch (fpscr.rn) {
            case 0x0:
                ret = bfp64_ROUND_TO_INTEGER_NEAR(val);
                break;
            case 0x1:
                ret = bfp64_ROUND_TO_INTEGER_TRUNC(val);
                break;
            case 0x2:
                ret = bfp64_ROUND_TO_INTEGER_CEIL(val);
                break;
            case 0x3:
                ret = bfp64_ROUND_TO_INTEGER_FLOOR(val);
                break;
        }
        
        if (!isNan(val) && !isInfinity(val)) {
			if (fabs(ret) > fabs(val)) {
				fpscr.fr = 1;
			} else {
				fpscr.fr = 0;
			}
			if (ret != val) {
				fpscr.fi = 1;
			} else {
				fpscr.fi = 0;
			}
		}
        return std::make_tuple(ret, fpscr);
	}
	
	std::tuple<float, Fpscr> round_current(float val, Fpscr fpscr) const
    {
		double ret;
		switch (fpscr.rn) {
            case 0x0:
                ret = bfp32_ROUND_TO_INTEGER_NEAR(val);
                break;
            case 0x1:
                ret = bfp32_ROUND_TO_INTEGER_TRUNC(val);
                break;
            case 0x2:
                ret = bfp32_ROUND_TO_INTEGER_CEIL(val);
                break;
            case 0x3:
                ret = bfp32_ROUND_TO_INTEGER_FLOOR(val);
                break;
        }
        
        if (!isNan(val) && !isInfinity(val)) {
			if (fabs(ret) > fabs(val)) {
				fpscr.fr = 1;
			} else {
				fpscr.fr = 0;
			}
			if (ret != val) {
				fpscr.fi = 1;
			} else {
				fpscr.fi = 0;
			}
		}
        return std::make_tuple(ret, fpscr);
	}
    
    std::tuple<float, Fpscr> get_floating_exception(float src1, float src2, Fpscr fpscr, float tmp) const
    {
        int except = fetestexcept(FE_ALL_EXCEPT);

        //fpscr.fi = 0;
        //fpscr.fx = 0;
        //fpscr.ox = 0;
        //fpscr.zx = 0;
        //fpscr.vx = 0;
        //fpscr.xx = 0;
        //fpscr.vxsnan = 0;
        
        //exception bits are sticky, only set but DO NOT clear them.
		if (fpscr.fi == 1) {
			fpscr.xx = fpscr.xx | fpscr.fi;
		}

        if (except & FE_INEXACT) {
             fpscr.fi = 1;
             fpscr.xx = fpscr.xx | fpscr.fi;
        }
        if (except & FE_UNDERFLOW) {
             fpscr.ux = 1;
        }
        if (except & FE_OVERFLOW) {
             fpscr.ox = 1;
        }
        if (except & FE_DIVBYZERO) {
             fpscr.zx = 1;
        }
        if (except & FE_INVALID) {
             fpscr.vx = 1;
        }
        if (isSnan(src1) || isSnan(src2)) {
             fpscr.vxsnan = 1;
        }
        if (isNan(src1) && isNan(src2)) {
             tmp = src1;
        }
        if (isSnan(tmp))
            tmp = SnantoQnan(tmp);
            
        fpscr.vx = (fpscr.vxisi | fpscr.vxzdz | fpscr.vxidi | fpscr.vximz | fpscr.vxvc | fpscr.vxsqrt | fpscr.vxcvi | fpscr.vxsnan);
		fpscr.fx = (fpscr.ux | fpscr.ox | fpscr.zx | fpscr.vx | fpscr.xx);
        return std::make_tuple(tmp, fpscr);
    }
    
    std::tuple<double, Fpscr> get_floating_exception(double src1, double src2, Fpscr fpscr, double tmp) const
    {
        //fpscr.fi = 0;
        //fpscr.fx = 0;
        //fpscr.ox = 0;
        //fpscr.zx = 0;
        //fpscr.vx = 0;
        //fpscr.xx = 0;
        //fpscr.vxsnan = 0;
        
        int except = fetestexcept(FE_ALL_EXCEPT);
        //int except  = except0;

		//exception bits are sticky, only set but DO NOT clear them.
		if (fpscr.fi == 1) {
			fpscr.xx = fpscr.xx | fpscr.fi;
		}

        if (except & FE_INEXACT) {
             fpscr.fi = 1;
             fpscr.xx = fpscr.xx | fpscr.fi;
        }
        if (except & FE_UNDERFLOW) {
             fpscr.ux = 1;
        }
        if (except & FE_OVERFLOW) {
             fpscr.ox = 1;
        }
        if (except & FE_DIVBYZERO) {
             fpscr.zx = 1;
        }
        if (except & FE_INVALID) {
             fpscr.vx = 1;
        }
        if (isSnan(src1) || isSnan(src2)) {
             fpscr.vxsnan = 1;
        }
        if (isNan(src1) && isNan(src2)) {
             tmp = src1;
        }
        if (isSnan(tmp))
            tmp = SnantoQnan(tmp);
            
        fpscr.vx = (fpscr.vxisi | fpscr.vxzdz | fpscr.vxidi | fpscr.vximz | fpscr.vxvc | fpscr.vxsqrt | fpscr.vxcvi | fpscr.vxsnan);
		fpscr.fx = (fpscr.ux | fpscr.ox | fpscr.zx | fpscr.vx | fpscr.xx);
        return std::make_tuple(tmp, fpscr);
    }
    
    std::tuple<__float128, Fpscr> get_floating_exception(__float128 src1, __float128 src2, Fpscr fpscr, __float128 tmp) const
    {
        //fpscr.fi = 0;
        //fpscr.fx = 0;
        //fpscr.ox = 0;
        //fpscr.zx = 0;
        //fpscr.vx = 0;
        //fpscr.xx = 0;
        //fpscr.vxsnan = 0;
        
        int except = fetestexcept(FE_ALL_EXCEPT);
        //int except  = except0;

		//exception bits are sticky, only set but DO NOT clear them.
		if (fpscr.fi == 1) {
			fpscr.xx = fpscr.xx | fpscr.fi;
		}

        if (except & FE_INEXACT) {
             fpscr.fi = 1;
             fpscr.xx = fpscr.xx | fpscr.fi;
        }
        if (except & FE_UNDERFLOW) {
             fpscr.ux = 1;
        }
        if (except & FE_OVERFLOW) {
             fpscr.ox = 1;
        }
        if (except & FE_DIVBYZERO) {
             fpscr.zx = 1;
        }
        if (except & FE_INVALID) {
             fpscr.vx = 1;
        }
        //if (isSnan((double)src1) || isSnan(src2)) {
        //     fpscr.vxsnan = 1;
        //}
        if (isNan((double)src1) && isNan((double)src2)) {
             tmp = src1;
        }
        //if (isSnan(tmp))
        //    tmp = SnantoQnan(tmp);
            
        fpscr.vx = (fpscr.vxisi | fpscr.vxzdz | fpscr.vxidi | fpscr.vximz | fpscr.vxvc | fpscr.vxsqrt | fpscr.vxcvi | fpscr.vxsnan);
		fpscr.fx = (fpscr.ux | fpscr.ox | fpscr.zx | fpscr.vx | fpscr.xx);
        return std::make_tuple(tmp, fpscr);
    }
    
    Fpscr get_floating_exception_0(Fpscr fpscr) const
    {
        
        int except = fetestexcept(FE_ALL_EXCEPT);
        //int except  = except0;

		//exception bits are sticky, only set but DO NOT clear them.
		if (fpscr.fi == 1) {
			fpscr.xx = fpscr.xx | fpscr.fi;
		}

        if (except & FE_INEXACT) {
             fpscr.fi = 1;
             fpscr.xx = fpscr.xx | fpscr.fi;
        }
        if (except & FE_UNDERFLOW) {
             fpscr.ux = 1;
        }
        if (except & FE_OVERFLOW) {
             fpscr.ox = 1;
        }
        if (except & FE_DIVBYZERO) {
             fpscr.zx = 1;
        }
        if (except & FE_INVALID) {
             fpscr.vx = 1;
        }
            
        fpscr.fx = (fpscr.ux | fpscr.ox | fpscr.zx | fpscr.vx | fpscr.xx | fpscr.vxsnan);
        return fpscr;
    }

    Fpscr update_fpscr(Fpscr fpscr, Fpscr fpscr_flags) const
    {
		if (fpscr_flags.ux)
			fpscr.ux = 1;
		if (fpscr_flags.ox)
			fpscr.ox = 1;
		if (fpscr_flags.zx)
			fpscr.zx = 1;
		if (fpscr_flags.vx)
			fpscr.vx = 1;
		if (fpscr_flags.xx)
			fpscr.xx = 1;
		if (fpscr_flags.vxsnan)
			fpscr.vxsnan = 1;
		if (fpscr_flags.vxisi)
			fpscr.vxisi = 1;
		if (fpscr_flags.vxidi)
			fpscr.vxidi = 1;
		if (fpscr_flags.vxzdz)
			fpscr.vxzdz = 1;
		if (fpscr_flags.vximz)
			fpscr.vximz = 1;
		if (fpscr_flags.vxvc)
			fpscr.vxvc = 1;
		if (fpscr_flags.vxsqrt)
			fpscr.vxsqrt = 1;
		if (fpscr_flags.vxcvi)
			fpscr.vxcvi = 1;
			
		fpscr.fi = fpscr_flags.fi;
		fpscr.fr = fpscr_flags.fr;
		fpscr.vx = (fpscr.vxisi | fpscr.vxzdz | fpscr.vxidi | fpscr.vximz | fpscr.vxvc | fpscr.vxsqrt | fpscr.vxcvi | fpscr.vxsnan);
		fpscr.fx = (fpscr.ux | fpscr.ox | fpscr.zx | fpscr.vx | fpscr.xx);
		return fpscr;
	}
    
    uint8_t calculate_grs(double src1, double src2)
    {
		void *val_ptr1 = &src1;
        uint64_t val_bits1 = *(uint64_t *) val_ptr1;
        int exp1 = bits(val_bits1, 62, 52);
        uint64_t frac1 = (1ULL << 52) | bits(val_bits1, 51, 0);
        void *val_ptr2 = &src2;
        uint64_t val_bits2 = *(uint64_t *) val_ptr2;
        int exp2 = bits(val_bits2, 62, 52);
        uint64_t frac2 = (1ULL << 52) | bits(val_bits2, 51, 0);
        int shift = 0;
        uint64_t tmp;
        uint8_t grs = 0;
        if (exp1 > exp2) {
			shift = exp1 - exp2;
			tmp = frac2 << 3;
			
		} else if (exp1 < exp2) {
			shift = exp2 - exp1;
			tmp = frac1 << 3;
		}
		
		for (int i = 0; i < shift; i++) {
			tmp = tmp > 1;
			grs = grs ^ (tmp & 0x1);
		}
		grs |= (tmp & 0x6);
		return grs;
	}
    
    Fpscr vsx_set_class(float vrt_val, Fpscr fpscr) const
    {
        bool vx_flag = fpscr.vxsnan | fpscr.vxisi;
        bool vex_flag = vx_flag & fpscr.ve;
        if (vex_flag == 0) {
            //modify by outside
        } else {
            fpscr.fr = 0;
            fpscr.fi = 0;
        }
    	if (isQnan(vrt_val)) {
    	    fpscr.fprf.c = 1;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 1;
    	}
    	
    	if (isInfinity(vrt_val) && isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 1;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 1;
    	}
    	
    	if (isNormalized(vrt_val) && isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 1;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isDenormalized(vrt_val) && isNegative(vrt_val)) {
    	    fpscr.fprf.c = 1;
    	    fpscr.fprf.fpcc.fl = 1;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isZero(vrt_val) && isNegative(vrt_val)) {
    	    fpscr.fprf.c = 1;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 1;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isZero(vrt_val) && !isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 1;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isDenormalized(vrt_val) && !isNegative(vrt_val)) {
    	    fpscr.fprf.c = 1;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 1;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isNormalized(vrt_val) && !isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 1;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isInfinity(vrt_val) && !isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 1;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 1;
    	}
    	
    	return fpscr;
    }
    
    Fpscr vsx_set_class(double vrt_val, Fpscr fpscr) const
    {
        bool vx_flag = fpscr.vxsnan | fpscr.vxisi;
        bool vex_flag = vx_flag & fpscr.ve;
        //fpscr.fprf.c = 0;
    	//fpscr.fprf.fpcc = 0;
        if (vex_flag == 0) {
			//modify by outside
        } else {
            fpscr.fr = 0;
            fpscr.fi = 0;
        }
        
    	if (isQnan(vrt_val)) {
    	    fpscr.fprf.c = 1;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 1;
    	}
    	
    	if (isInfinity(vrt_val) && isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 1;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 1;
    	}
    	
    	if (isNormalized(vrt_val) && isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 1;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isDenormalized(vrt_val) && isNegative(vrt_val)) {
    	    fpscr.fprf.c = 1;
    	    fpscr.fprf.fpcc.fl = 1;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isZero(vrt_val) && isNegative(vrt_val)) {
    	    fpscr.fprf.c = 1;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 1;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isZero(vrt_val) && !isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 0;
    	    fpscr.fprf.fpcc.fe = 1;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isDenormalized(vrt_val) && !isNegative(vrt_val)) {
    	    fpscr.fprf.c = 1;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 1;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isNormalized(vrt_val) && !isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 1;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 0;
    	}
    	
    	if (isInfinity(vrt_val) && !isNegative(vrt_val)) {
    	    fpscr.fprf.c = 0;
    	    fpscr.fprf.fpcc.fl = 0;
    	    fpscr.fprf.fpcc.fg = 1;
    	    fpscr.fprf.fpcc.fe = 0;
    	    fpscr.fprf.fpcc.fu = 1;
    	}
    	
    	return fpscr;
    }

    // Compute the CR field
    inline uint32_t
    makeCRField(double a, double b) const
    {
        uint32_t c = 0;
        if (isNan(a) || isNan(b)) { c = 0x1; }
        else if (a < b)           { c = 0x8; }
        else if (a > b)           { c = 0x4; }
        else                      { c = 0x2; }
        return c;
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

} // namespace PowerISA
} // namespace gem5

#endif //__ARCH_POWER_INSTS_FLOATING_HH__
