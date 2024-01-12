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
#include "debug/Fault.hh"
#include "debug/Interrupt.hh"
#include "enums/ByteOrder.hh"
#include "sim/faults.hh"


namespace gem5
{

namespace PowerISA
{

#define SRR1_TRAP_BIT     17
#define SRR1_PRI_BIT      18
#define SRR1_ILLEGAL_INSTR_BIT 18

#define setbit(shift, mask) ( (uint64_t)1 << shift | mask)
#define unsetbit(shift,mask) ( ~((uint64_t)1 << shift) & mask)
#define setBitMask(shift) ( (uint64_t)1 << shift)
#define unsetMask(start ,end)(~((setBitMask(start))-1) | ((setBitMask(end))-1))

enum pcSet
{
    SystemResetPCSet = 0x100,
    ProgramPCSet = 0x700,
    DataStoragePCSet = 0x300,
    DataSegmentPCSet = 0x380,
    InstrStoragePCSet = 0x400,
    InstrSegmentPCSet = 0x480,
    DirectExternalPCSet = 0x500,
    DecrementerPCSet = 0x900,
    PriDoorbellPCSet = 0xA00,
    HypDoorbellPCSet = 0xe80,
    SystemCallPCSet = 0xC00,
};

/* SRR1[42:45] wakeup fields for System Reset Interrupt */

#define SRR1_WAKEMASK           0x003c0000 /* reason for wakeup */

#define SRR1_WAKEHMI            0x00280000 /* Hypervisor maintenance */
#define SRR1_WAKEHVI            0x00240000 /* Hypervisor Virt. Interrupt (P9) */
#define SRR1_WAKEEE             0x00200000 /* External interrupt */
#define SRR1_WAKEDEC            0x00180000 /* Decrementer interrupt */
#define SRR1_WAKEDBELL          0x00140000 /* Privileged doorbell */
#define SRR1_WAKERESET          0x00100000 /* System reset */
#define SRR1_WAKEHDBELL         0x000c0000 /* Hypervisor doorbell */
#define SRR1_WAKESCOM           0x00080000 /* SCOM not in power-saving mode */

/* SRR1[46:47] power-saving exit mode */

#define SRR1_WAKESTATE          0x00030000 /* Powersave exit mask */

#define SRR1_WS_HVLOSS          0x00030000 /* HV resources not maintained */
#define SRR1_WS_GPRLOSS         0x00020000 /* GPRs not maintained */
#define SRR1_WS_NOLOSS          0x00010000 /* All resources maintained */

class PowerFault : public FaultBase
{
  protected:
    FaultName _name;
    const PCState pcState;

    PowerFault(FaultName name)
        : _name(name)
    {
    }

    FaultName
    name() const
    {
        return _name;
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
      //printf("set srr1:%llx\n", srr1);
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


class MachineCheckInterrupt : public PowerFault
{
  public:
    MachineCheckInterrupt()
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

class ResetInterrupt : public PowerFault
{
  public:
    ResetInterrupt()
      : PowerFault("ResetInterrupt")
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      DPRINTF(Fault, "System Reset Interrupt invoked\n");
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      uint64_t powersaving = tc->readIntReg(INTREG_PMC6);
      uint64_t cause = tc->readIntReg(INTREG_PMC5);
      if (powersaving == 1) {
        PowerFault::updateSRR1(tc, SRR1_WS_NOLOSS | cause);
        PowerFault::updateMsr(tc);
        tc->setIntReg(INTREG_PMC6, 0);
      } else {
        PowerFault::updateSRR1(tc);
        PowerFault::updateMsr(tc);
      }
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(SystemResetPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};

class DirectExternalInterrupt : public PowerFault
{
  public:
    DirectExternalInterrupt()
      : PowerFault("DirectExternalInterrupt")
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                        StaticInst::nullStaticInstPtr)
    {
      DPRINTF(Fault, "Direct External Interrupt invoked\n");
      // Refer Power ISA Manual v3.0B Book-III, section 6.5.7.1
      Lpcr lpcr = tc->readIntReg(INTREG_LPCR);

      if (lpcr.lpes){
        tc->setIntReg(INTREG_SRR0 , tc->instAddr());
        PowerFault::updateSRR1(tc);
        PowerFault::updateMsr(tc);
        Msr msr =  tc->readIntReg(INTREG_MSR);
        msr.ri = 0;
        tc->setIntReg(INTREG_MSR, msr);
      }
      else{
        tc->setIntReg(INTREG_HSRR0 , tc->instAddr());
        PowerFault::updateHSRR1(tc);
        PowerFault::updateMsr(tc);
        Msr msr =  tc->readIntReg(INTREG_MSR);
        msr.hv = 1;
        tc->setIntReg(INTREG_MSR, msr);
      }
      //tc->pcState(DirectExternalPCSet);
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(DirectExternalPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};

class PriDoorbellInterrupt : public PowerFault
{
  public:
    PriDoorbellInterrupt()
      : PowerFault("PriDoorbellInterrupt")
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      DPRINTF(Fault, "Doorbell Interrupt invoked, tid = %d.\n", tc->threadId());
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerFault::updateSRR1(tc);
      PowerFault::updateMsr(tc);
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(PriDoorbellPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};

class HypDoorbellInterrupt : public PowerFault
{
  public:
    HypDoorbellInterrupt()
      : PowerFault("HyperDoorbellInterrupt")
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      DPRINTF(Fault, "Hyper Doorbell Interrupt invoked, tid = %d.\n", tc->threadId());
      tc->setIntReg(INTREG_HSRR0 , tc->instAddr());
      PowerFault::updateHSRR1(tc);
      PowerFault::updateMsr(tc);
      Msr msr = tc->readIntReg(INTREG_MSR);
      msr.hv = 1;
      tc->setIntReg(INTREG_MSR, msr);
      //tc->pcState(HypDoorbellPCSet);
      PCState *pc = new PCState(HypDoorbellPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};

//SRR1 value is correctly set by the entity raising
//Instruction Storage Interrupt. So, no need to
//Update here.

class InstrStorageFault : public PowerFault
{
private:
  uint64_t srr;
public:
  InstrStorageFault(uint64_t _srr)
     : PowerFault("InstrStorageFault"), srr(_srr)
  {
  }
  virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      DPRINTF(Fault, "InstrStorageFault.pc = %llx tid = %d.\n",
        tc->pcState().instAddr(), tc->threadId());
      //tc->pcState(InstrStoragePCSet);
      Msr msr = tc->readIntReg(INTREG_MSR);
      //here unsetting SRR1 bits 33-36 and 42-47 according to ISA
      uint64_t srr1 = ((msr & unsetMask(31, 27)) & unsetMask(22,16)) | srr;
      tc->setIntReg(INTREG_SRR1, srr1);
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerFault::updateMsr(tc);
      PCState *pc = new PCState(InstrStoragePCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};

class InstrSegmentFault : public PowerFault
{
public:
  InstrSegmentFault()
    : PowerFault("InstrSegmentFault")
  {
  }
  virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      DPRINTF(Fault, "InstrSegmentFault.pc = %llx tid = %d.\n",
        tc->pcState().instAddr(), tc->threadId());
      PowerFault::updateMsr(tc);
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(InstrSegmentPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};


class DataStorageFault :public PowerFault
{
private:
  uint64_t dar;
  uint64_t dsisr;
public:
  DataStorageFault(uint64_t _dar, uint64_t _dsisr)
    : PowerFault("DataStorageFault"), dar(_dar), dsisr(_dsisr)
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      DPRINTF(Fault, "DataStorageFault.pc = %llx tid = %d addr = %llx DSISR = %llx\n",
        tc->pcState().instAddr(), tc->threadId(), tc->readIntReg(INTREG_DAR), tc->readIntReg(INTREG_DSISR));
      //tc->pcState(DataStoragePCSet);
      tc->setIntReg(INTREG_DSISR, dsisr);
      tc->setIntReg(INTREG_DAR, dar);
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerFault::updateSRR1(tc);
      PowerFault::updateMsr(tc);
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(DataStoragePCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};

class DataSegmentFault :public PowerFault
{
private:
  uint64_t dar;
public:
  DataSegmentFault(uint64_t _dar)
    : PowerFault("DataSegmentFault"), dar(_dar)
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      tc->setIntReg(INTREG_DAR, dar);
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerFault::updateSRR1(tc);
      PowerFault::updateMsr(tc);
      DPRINTF(Fault, "DataSegmentFault.pc = %llx tid = %d addr = %llx\n",
        tc->pcState().instAddr(), tc->threadId(), tc->readIntReg(INTREG_DAR));
      //tc->pcState(DataStoragePCSet);
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(DataSegmentPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};

//TODO: Need to add Floating point and TM Bad thing fault handler
class ProgramInterrupt : public PowerFault
{
  public:
    ProgramInterrupt()
      : PowerFault("ProgramFault")
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr ,uint64_t bitSet = 0)
    {
      tc->setIntReg(INTREG_SRR0, tc->instAddr());
      PowerFault::updateSRR1(tc, bitSet);
      PowerFault::updateMsr(tc);
      //tc->pcState(ProgramPCSet);
      //printf("Program Fault! pc = %llx\n", tc->pcState().instAddr());
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(ProgramPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
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

class SystemCallInterrupt : public PowerFault
{
  public:
    SystemCallInterrupt()
      : PowerFault("SyscallFault")
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr)
    {
      //TODO: Right now it not handle case when LEV=0.
      tc->setIntReg(INTREG_SRR0 , tc->instAddr() + 4);
      PowerFault::updateSRR1(tc);
      PowerFault::updateMsr(tc);
      //tc->pcState(SystemCallPCSet);
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(SystemCallPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};

class DecrementerInterrupt : public PowerFault
{
  public:
    DecrementerInterrupt()
      : PowerFault("DecrementerInterrupt")
    {
    }
    virtual void invoke(ThreadContext * tc, const StaticInstPtr &inst =
                        StaticInst::nullStaticInstPtr)
    {
      DPRINTF(Interrupt, "DecrementerInterrupt.pc = %llx\n",
        tc->pcState().instAddr());
      // Refer Power ISA Manual v3.0B Book-III, section 6.5.11
      tc->setIntReg(INTREG_SRR0 , tc->instAddr());
      PowerFault::updateSRR1(tc);
      PowerFault::updateMsr(tc);
      //tc->pcState(DecrementerPCSet);
      Msr msr = tc->readIntReg(INTREG_MSR);
      PCState *pc = new PCState(DecrementerPCSet,
        msr.le ? ByteOrder::little : ByteOrder::big);
      tc->pcState(*pc);
      delete pc;
    }
};


} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_FAULTS_HH__
