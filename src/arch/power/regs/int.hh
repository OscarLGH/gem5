/*
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2021 IBM Corporation
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

#ifndef __ARCH_POWER_REGS_INT_HH__
#define __ARCH_POWER_REGS_INT_HH__

namespace gem5
{

namespace PowerISA
{

// Constants Related to the number of registers
const int NumIntArchRegs = 32;

// CR, XER, LR, CTR, TAR, FPSCR, MSR, RSV, RSV-LEN, RSV-ADDR
// and zero register, which doesn't actually exist but needs a number
const int NumIntSpecialRegs = 13;

const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs;

// Semantically meaningful register indices
const int ReturnValueReg = 3;
const int ArgumentReg0 = 3;
const int ArgumentReg1 = 4;
const int ArgumentReg2 = 5;
const int ArgumentReg3 = 6;
const int ArgumentReg4 = 7;
const int ArgumentReg5 = 8;
const int StackPointerReg = 1;
const int TOCPointerReg = 2;
const int ThreadPointerReg = 13;

enum MiscIntRegNums
{
    INTREG_CR = NumIntArchRegs,
    INTREG_XER,
    INTREG_LR,
    INTREG_CTR,
    INTREG_TAR,
    INTREG_FPSCR,
    INTREG_RSV,
    INTREG_RSV_LEN,
    INTREG_RSV_ADDR,
    INTREG_VSCR,

    INTREG_DSCR,
    INTREG_DSISR,
    INTREG_DAR,
    INTREG_DEC,
    INTREG_SRR0,
    INTREG_SRR1,
    INTREG_CFAR,
    INTREG_CTRL,
    INTREG_VRSAVE,
    INTREG_TB,
    INTREG_TBL,
    INTREG_TBU,
    INTREG_PVR,
    INTREG_PPR,
    INTREG_PPR32,
    INTREG_MSR,
    INTREG_SPRG0,
    INTREG_SPRG1,
    INTREG_SPRG2,
    INTREG_SPRG3,
    INTREG_LPCR,
    INTREG_PTCR,
    INTREG_FSCR,
    INTREG_MMCRA,
    INTREG_MMCR0,
    INTREG_MMCR1,
    INTREG_MMCR2,
    INTREG_PSSCR,
    INTREG_LPIDR,
    INTREG_PIDR,
    INTREG_HFSCR,
    INTREG_HSPRG0,
    INTREG_HIR0,
    INTREG_MMCRC,
    INTREG_AMOR,
    INTREG_IAMR,
    INTREG_HSRR0,
    INTREG_HSRR1,
    INTREG_HDEC,
    INTREG_PCR,
    INTREG_AMR,
    INTREG_TFHAR,
    INTREG_TFIAR,
    INTREG_TEXASR,
    INTREG_TIDR,
    INTREG_UAMOR,
    INTREG_DPDES,
    INTREG_DAWR0,
    INTREG_RPR,
    INTREG_CIABR,
    INTREG_CIR,
    INTREG_TBU40,
    INTREG_SPURR,
    INTREG_PURR,
    INTREG_HDSISR,
    INTREG_HDAR,
    INTREG_HRMOR,
    INTREG_HMER,
    INTREG_HMEER,
    INTREG_SIER,
    INTREG_PMC1,
    INTREG_PMC2,
    INTREG_PMC3,
    INTREG_PMC4,
    INTREG_PMC5,
    INTREG_PMC6,
    INTREG_SDAR,
    INTREG_BESCRS,
    INTREG_BESCRSU,
    INTREG_BESCRR,
    INTREG_BESCRRU,
    INTREG_EBBHR,
    INTREG_EBBRR,
    INTREG_BESCR,
    INTREG_ASDR,
    INTREG_IC,
    INTREG_VTB,
    INTREG_HSPRG1,
    INTREG_PIR,
    INTREG_DUMMY
};

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_REGS_INT_HH__
