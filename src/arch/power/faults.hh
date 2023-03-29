/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __ARCH_POWER_FAULTS_HH__
#define __ARCH_POWER_FAULTS_HH__

#include "arch/power/regs/int.hh"
#include "arch/power/regs/misc.hh"
#include "cpu/thread_context.hh"
#include "sim/faults.hh"

namespace gem5
{

namespace PowerISA
{

#define SRR1_TRAP_BIT     16
#define SRR1_PRI_BIT      17
#define SRR1_ILLEGAL_INSTR_BIT 18

#define setbit(shift, mask) ( (uint64_t)1 << shift | mask)
#define unsetbit(shift,mask) ( ~((uint64_t)1 << shift) & mask)
#define setBitMask(shift) ( (uint64_t)1 << shift)
#define unsetMask(start ,end)(~((setBitMask(start))-1) | ((setBitMask(end))-1))

enum pcSet
{   DecrementerPCSet = 0x900,
    SystemCallPCSet = 0xC00,
    ProgramPCSet = 0x700,
    DataStoragePCSet = 0x300,
    InstrStoragePCSet = 0x400,
    PriDoorbellPCSet = 0xA00,
    HypDoorbellPCSet = 0xe80,
    DirectExternalPCSet = 0x500
};

class PowerFault : public FaultBase
{
  protected:
    FaultName _name;

    PowerFault(FaultName name)
        : _name(name)
    {
    }

    FaultName
    name() const
    {
        return _name;
    }
};


class UnimplementedOpcodeFault : public PowerFault
{
  public:
    UnimplementedOpcodeFault()
        : PowerFault("Unimplemented Opcode")
    {
    }

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
};


class MachineCheckFault : public PowerFault
{
  public:
    MachineCheckFault()
        : PowerFault("Machine Check")
    {
    }
};


class AlignmentFault : public PowerFault
{
  private:
    Addr vaddr;
  public:
    AlignmentFault(Addr va)
        : PowerFault("Alignment"), vaddr(va)
    {
    }

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
};


class TrapFault : public PowerFault
{
  public:
    TrapFault()
        : PowerFault("Trap")
    {
    }

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
                nullStaticInstPtr) override;
};

class PowerInterrupt : public PowerFault
{
  public:
    PowerInterrupt()
        : PowerFault("Interrupt")
    {
    }
    virtual void updateMsr(ThreadContext * tc)
      {
        Msr msr = tc->readIntReg(INTREG_MSR);
        msr.tm = 0;
        msr.vec = 0;
        msr.vsx = 0;
        msr.fp = 0;
        msr.pr = 0;
        msr.pmm = 0;
        msr.ir = 0;
        msr.dr = 0;
        msr.fe1 = 0;
        msr.fe0 = 0;
        msr.ee = 0;
        msr.ri = 0;
        msr.te = 0;
        msr.sf = 1;
        msr = unsetbit(5, msr);
        tc->setIntReg(INTREG_MSR, msr);
      }

    virtual void updateSRR1(ThreadContext *tc, uint64_t BitMask=0x0000000)
    {
      Msr msr = tc->readIntReg(INTREG_MSR);
      uint64_t srr1 = ((msr & unsetMask(31, 27)) & unsetMask(22,16)) | BitMask;
      tc->setIntReg(INTREG_SRR1, srr1);
    }

    virtual void updateHSRR1(ThreadContext *tc, uint64_t BitMask=0x0000000)
    {
      Msr msr = tc->readIntReg(INTREG_MSR);
      uint64_t hsrr1 = ((msr & unsetMask(31, 27)) & unsetMask(22,16)) |
                                                    BitMask;
      tc->setIntReg(INTREG_HSRR1, hsrr1);
    }
};

class DirectExternalInterrupt : public PowerInterrupt
{
  public:
    DirectExternalInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                        StaticInst::nullStaticInstPtr)
    {
      printf("Direct External Interrupt invoked\n");
      // Refer Power ISA Manual v3.0B Book-III, section 6.5.7.1
      Lpcr lpcr = tc->readIntReg(INTREG_LPCR);

      if (lpcr.lpes){
        tc->setIntReg(INTREG_SRR0 , tc->instAddr());
        PowerInterrupt::updateSRR1(tc);
        PowerInterrupt::updateMsr(tc);
        Msr msr =  tc->readIntReg(INTREG_MSR);
        msr.ri = 0;
        tc->setIntReg(INTREG_MSR, msr);
      }
      else{
        tc->setIntReg(INTREG_HSRR0 , tc->instAddr());
        PowerInterrupt::updateHSRR1(tc);
        PowerInterrupt::updateMsr(tc);
        Msr msr =  tc->readIntReg(INTREG_MSR);
        msr.hv = 1;
        tc->setIntReg(INTREG_MSR, msr);
      }
      tc->pcState(DirectExternalPCSet);
    }
};

class PriDoorbellInterrupt : public PowerInterrupt
{
  public:
    PriDoorbellInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerInterrupt::updateSRR1(tc);
      PowerInterrupt::updateMsr(tc);
      tc->pcState(PriDoorbellPCSet);
    }
};

class HypDoorbellInterrupt : public PowerInterrupt
{
  public:
    HypDoorbellInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      printf("In Hypervisor interrupt\n");
      tc->setIntReg(INTREG_HSRR0 , tc->instAddr());
      PowerInterrupt::updateHSRR1(tc);
      PowerInterrupt::updateMsr(tc);
      Msr msr = tc->readIntReg(INTREG_MSR);
      msr.hv = 1;
      tc->setIntReg(INTREG_MSR, msr);
      tc->pcState(HypDoorbellPCSet);
    }
};

//SRR1 value is correctly set by the entity raising
//Instruction Storage Interrupt. So, no need to
//Update here.

class InstrStorageInterrupt : public PowerInterrupt
{
public:
  InstrStorageInterrupt()
  {
  }
  virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerInterrupt::updateMsr(tc);
      tc->pcState(InstrStoragePCSet);
    }
};


class DataStorageInterrupt :public PowerInterrupt
{
public:
  DataStorageInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerInterrupt::updateSRR1(tc);
      PowerInterrupt::updateMsr(tc);
      tc->pcState(DataStoragePCSet);
    }
};

//TODO: Need to add Floating point and TM Bad thing fault handler
class ProgramInterrupt : public PowerInterrupt
{
  public:
    ProgramInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr ,uint64_t bitSet = 0)
    {
      tc->setIntReg(INTREG_SRR0, tc->instAddr());
      PowerInterrupt::updateSRR1(tc, bitSet);
      PowerInterrupt::updateMsr(tc);
      tc->pcState(ProgramPCSet);
    }
};

class ProgramIllegalInterrupt : public ProgramInterrupt
{
  public:
    ProgramIllegalInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      ProgramInterrupt::invoke(tc, inst ,setBitMask(SRR1_ILLEGAL_INSTR_BIT));
    }
};

class ProgramTrapInterrupt : public ProgramInterrupt
{
  public:
    ProgramTrapInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      ProgramInterrupt::invoke(tc, inst ,setBitMask(SRR1_TRAP_BIT));
    }
};

class ProgramPriInterrupt : public ProgramInterrupt
{
  public:
    ProgramPriInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      ProgramInterrupt::invoke(tc, inst, setBitMask(SRR1_PRI_BIT));
    }
};

class SystemCallInterrupt : public PowerInterrupt
{
  public:
    SystemCallInterrupt();
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst);
    /* SystemCallInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      //TODO: Right now it not handle case when LEV=0.
      tc->setIntReg(INTREG_SRR0 , tc->instAddr() + 4);
      PowerInterrupt::updateSRR1(tc);
      PowerInterrupt::updateMsr(tc);
      tc->pcState(SystemCallPCSet);
    }
    */
};

class DecrementerInterrupt : public PowerInterrupt
{
  public:
    DecrementerInterrupt()
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                        StaticInst::nullStaticInstPtr)
    {
      // Refer Power ISA Manual v3.0B Book-III, section 6.5.11
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerInterrupt::updateSRR1(tc);
      PowerInterrupt::updateMsr(tc);
      tc->pcState(DecrementerPCSet);
    }
};


} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_FAULTS_HH__
