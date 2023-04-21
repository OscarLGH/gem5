/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * Copyright (c) 2018 TU Dresden
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "arch/power/bare_metal/fs_workload.hh"

#include "arch/power/regs/int.hh"
#include "arch/power/regs/misc.hh"
#include "base/loader/object_file.hh"
#include "cpu/thread_context.hh"
#include "sim/system.hh"

namespace gem5
{

namespace PowerISA
{

BareMetal::BareMetal(const Params &p) : Workload(p),
_isBareMetal(p.bare_metal),
bootloader(loader::createObjectFile(p.bootloader))
{

}

BareMetal::~BareMetal()
{
    delete bootloader;
}

void
BareMetal::initState()
{
    Workload::initState();
    printf("PowerSystem::initState: No of thread contexts %d\n" ,
                    (int)system->threads.size());

    ThreadContext *tc = system->threads[0];
    //tc->pcState(tc->getSystemPtr()->kernelEntry);
    tc->pcState(0x10); // For skiboot
    //Sixty Four, little endian,Hypervisor bits are enabled.
    // IR and DR bits are disabled.
    Msr msr = 0x9000000000000001;
    tc->setIntReg(INTREG_DEC , 0xffffffffffffffff);
    // This PVR is specific to power9
    // Setting TB register to 0
    tc->setIntReg(INTREG_TB , 0x0);
    //tc->setIntReg(INTREG_PVR , 0x004e1100);
    tc->setIntReg(INTREG_PVR , 0x004e0200);
    tc->setIntReg(INTREG_MSR , msr);
    //ArgumentReg0 is initialized with 0xc00000 because in linux/system.cc
    //dtb is loaded at 0xc00000
    tc->setIntReg(ArgumentReg0, 0x1800000);

    /*
    ThreadID tid = 1;
    ThreadContext *tc1 = system->threads[1];
    tc1->pcState(0x10);
    tc1->setIntReg(INTREG_PVR , 0x004e0200);
    tc1->setIntReg(INTREG_MSR , msr);
    tc1->setIntReg(ArgumentReg0, 0x1800000);
    //tc1->pcState(0xc00000000000a840);
    //tc1->setIntReg(ArgumentReg0, 0x1);
    tc1->setIntReg(INTREG_PIR,0x1);
    */
}

} // namespace PowerISA
} // namespace gem5
