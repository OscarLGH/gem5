/*
 * Copyright (c) 2011 Google
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

#ifndef __ARCH_POWER_INTERRUPT_HH__
#define __ARCH_POWER_INTERRUPT_HH__

#include "arch/generic/interrupts.hh"
#include "arch/power/faults.hh"
#include "arch/power/registers.hh"
#include "base/logging.hh"
#include "debug/Interrupt.hh"
#include "params/PowerInterrupts.hh"
#include "sim/sim_object.hh"

#define NumInterruptLevels 8

#define SystemReset 0 //System Reset Interrupt(Highest Priority)
#define MachineCheck 1 //Machine Check Interrupt
#define DirectExt 2 //Direct External Interrupt
#define MediatedExt 3 //Mediated External Interrupt
#define Decrementer 4 //Decrementer Interrupt
#define PerfMoniter 5 //Performance Monitor Interrupt
#define DirPriDoorbell 6 //Directed Privileged Doorbell Interrupt
#define DirHypDoorbell 7 //Directed Hypervisor Doorbell Interrupt

class BaseCPU;
class ThreadContext;

namespace PowerISA {

class Interrupts : public BaseInterrupts
{
  private:
    BaseCPU * cpu;

  protected:
    bool interrupts[NumInterruptLevels];

  public:
    using Params = PowerInterruptsParams;

    Interrupts(const Params &p) : BaseInterrupts(p) {}

    void
    setCPU(BaseCPU * _cpu)
    {
        cpu = _cpu;
    }

    void
    post(int int_num, int index)
    {
       DPRINTF(Interrupt, "Interrupt %d: posted\n", int_num);
       if (int_num < 0 || int_num >= NumInterruptLevels)
         panic("int_num out of bounds for fun POST%d\n",int_num);
       interrupts[int_num] = 1;
    }

    void
    clear(int int_num, int index)
    {
       DPRINTF(Interrupt, "Interrupt %d:\n", int_num);
       if (int_num < 0 || int_num >= NumInterruptLevels)
         panic("int_num out of bounds for fun CLEAR%d\n",int_num);
       interrupts[int_num] = 0;
    }

    void
    clearAll()
    {
       memset(interrupts, 0, sizeof(interrupts));
    }

    bool
    checkInterrupts() const override
    {
        return false;
    }

    bool
    checkInterrupts()
    {
       Msr msr = tc->readIntReg(INTREG_MSR);
       tc->setIntReg(INTREG_TB , tc->readIntReg(INTREG_TB)+1);
       if (tc->readIntReg(INTREG_DEC) != 0)
         tc->setIntReg(INTREG_DEC , tc->readIntReg(INTREG_DEC)-1);
       else
         interrupts[Decrementer] = 1;
       if (msr.ee)
       {
         if (interrupts[2] == 1)
           return true;
         for (int i = 0; i < NumInterruptLevels; i++) {
             if (interrupts[i] == 1)
               return true;
         }
       }
       if (interrupts[DirHypDoorbell] && (!msr.hv || msr.pr))
         return true;
       return false;
    }

    Fault
    getInterrupt() override
    {
        assert(checkInterrupts());
        if (interrupts[Decrementer]) {
            clear(Decrementer,0);
            return std::make_shared<DecrementerInterrupt>();
        }
        else if (interrupts[DirPriDoorbell]) {
            clear(DirPriDoorbell,0);
            return std::make_shared<PriDoorbellInterrupt>();
        }
        else if (interrupts[DirHypDoorbell]) {
            clear(DirHypDoorbell,0);
            return std::make_shared<HypDoorbellInterrupt>();
        }
        else if (interrupts[DirectExt]){
            clear(DirectExt,0);
            return std::make_shared<DirectExternalInterrupt>();
        }

        else return NoFault;
    }

    void
    updateIntrInfo()
    {
        tc->setIntReg(INTREG_DEC , 0xffffffffffffffff);
    }
};

} // namespace PowerISA

#endif // __ARCH_POWER_INTERRUPT_HH__

