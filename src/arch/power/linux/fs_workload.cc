/*
 * Copyright (c) 2021 Huawei International
 * All rights reserved
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

#include "arch/power/linux/fs_workload.hh"

#include "arch/power/faults.hh"
#include "base/loader/dtb_file.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "sim/kernel_workload.hh"
#include "sim/system.hh"

namespace gem5
{

namespace PowerISA
{

void
FsLinux::initState()
{
    KernelWorkload::initState();

    if (params().dtb_filename != "") {
        inform("Loading DTB file: %s at address %#x\n", params().dtb_filename,
                params().dtb_addr);

        auto *dtb_file = new loader::DtbFile(params().dtb_filename);

        if (!dtb_file->addBootCmdLine(
                    commandLine.c_str(), commandLine.size())) {
            warn("couldn't append bootargs to DTB file: %s\n",
                 params().dtb_filename);
        }

        dtb_file->buildImage().offset(params().dtb_addr)
            .write(system->physProxy);
        delete dtb_file;

        for (auto *tc: system->threads) {
            //tc->setIntReg(11, params().dtb_addr);
        }
    } else {
        warn("No DTB file specified\n");
    }

    printf("PowerSystem::initState: No of thread contexts %d\n" ,
                    (int)system->threads.size());
    //printf("bootloader entry point: %p\n" ,
    //                bootloader->entryPoint());
    ByteOrder byteOrder = kernelObj->getByteOrder();
    bool is64bit = (kernelObj->getArch() == loader::Power64);
    bool isLittleEndian = (byteOrder == ByteOrder::little);

    int cpu_idx = 0;
    for (auto *tc: system->threads) {
        auto pc = tc->pcState();
        pc.set(0x10);    // For skiboot
        pc.byteOrder(byteOrder);
        tc->pcState(pc);
        //Sixty Four, little endian,Hypervisor bits are enabled.
        // IR and DR bits are disabled.
        //Msr msr = 0x9000000000000000;
        //Set the machine status for a typical userspace
        Msr msr = 0;
        msr.sf = is64bit;
        msr.hv = 1;
        msr.ee = 0;
        msr.pr = 0;
        msr.me = 0;
        msr.ir = 0;
        msr.dr = 0;
        msr.ri = 0;
        msr.le = isLittleEndian;
        tc->setIntReg(INTREG_MSR, msr);
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
        tc->setIntReg(INTREG_PIR , cpu_idx++);
        tc->activate();
    }
}

} // namespace PowerISA
} // namespace gem5
