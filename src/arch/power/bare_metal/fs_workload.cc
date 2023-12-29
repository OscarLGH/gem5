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
#include "base/loader/dtb_file.hh"
#include "base/loader/raw_image.hh"
#include "base/loader/object_file.hh"
#include "cpu/thread_context.hh"
#include "sim/system.hh"

namespace gem5
{

namespace PowerISA
{

BareMetal::BareMetal(const Params &p) : KernelWorkload(p),
_isBareMetal(p.bare_metal),
bootloader(loader::createObjectFile(p.bootloader)),
kernel(loader::createObjectFile(p.kernel_filename))
{
    fatal_if(!bootloader, "Could not load bootloader file %s.", p.bootloader);
    _resetVect = bootloader->entryPoint();
    bootloaderSymtab = bootloader->symtab();
    kernelSymtab = kernel->symtab();
    loader::debugSymbolTable.insert(kernelSymtab);
    kernel->updateBias(p.kernel_addr);
}

BareMetal::~BareMetal()
{
    delete bootloader;
}

void
BareMetal::initState()
{
    KernelWorkload::initState();
    printf("PowerSystem::initState: No of thread contexts %d\n" ,
                    (int)system->threads.size());
    //printf("bootloader entry point: %p\n" ,
    //                bootloader->entryPoint());
    ByteOrder byteOrder = bootloader->getByteOrder();
    bool is64bit = (bootloader->getArch() == loader::Power64);
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
        //tc->setIntReg(INTREG_PVR , 0x004e0200);
        tc->setIntReg(INTREG_PVR , 0x004d0200); //power8
        tc->setIntReg(INTREG_MSR , msr);
        //ArgumentReg0 is initialized with 0xc00000 because in linux/system.cc
        //dtb is loaded at 0xc00000
        tc->setIntReg(ArgumentReg0, 0x1800000);
        tc->setIntReg(INTREG_PIR , cpu_idx++);
        tc->activate();
    }

    warn_if(!bootloader->buildImage().write(system->physProxy),
            "Could not load sections to memory.");

    warn_if(!kernel->buildImage().write(system->physProxy),
            "Could not load kernel sections to memory.");

    if (params().dtb_filename != "") {
        inform("Loading DTB file: %s at address %#x\n", params().dtb_filename,
                params().dtb_addr);

        auto *dtb_file = new loader::DtbFile(params().dtb_filename);

        dtb_file->addBootCmdLine(params().command_line.c_str(), params().command_line.size());
        dtb_file->buildImage().offset(params().dtb_addr)
            .write(system->physProxy);
        delete dtb_file;

        for (auto *tc: system->threads) {
            //tc->setIntReg(11, params().dtb_addr);
        }
    } else {
        warn("No DTB file specified\n");
    }

    if (params().initramfs_filename != "") {
        inform("Loading initramfs file: %s at address %#x\n", params().initramfs_filename,
                params().initramfs_addr);
        auto *initramfs_file = new loader::RawImage(params().initramfs_filename);
        initramfs_file->buildImage().offset(params().initramfs_addr)
            .write(system->physProxy);
        delete initramfs_file;
    }

    for (auto *tc: system->threads) {
        tc->activate();
    }
}

} // namespace PowerISA
} // namespace gem5
