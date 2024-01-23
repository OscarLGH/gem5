# Copyright (c) 2007-2008 The Hewlett-Packard Development Company
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.params import *
from m5.objects.Workload import *

class PowerBareMetal(KernelWorkload):
    type = 'PowerBareMetal'
    cxx_class = 'gem5::PowerISA::BareMetal'
    cxx_header = 'arch/power/bare_metal/fs_workload.hh'

    bare_metal = Param.Bool(True, "Using Bare Metal Application?")
    bootloader = Param.String("dist/m5/system/binaries/skiboot.elf", \
                              "File, that contains the bootloader code")
    reset_vect = Param.Addr(0x0, 'Reset vector')

    dtb_filename = Param.String("dist/m5/system/binaries/gem5-power8-fs.dtb",
        "File that contains the Device Tree Blob. Don't use DTB if empty.")
    dtb_addr = Param.Addr(0x1800000, "DTB address")
    skiboot = Param.String("dist/m5/system/binaries/skiboot.elf",
            "File that contains the OPAL firmware.");
    kernel_filename = Param.String("dist/m5/system/binaries/vmlinux",
            "File that contains the kernel.");
    kernel_addr = Param.Addr(0x20000000, "kernel address")
    initramfs_filename = Param.String("dist/m5/system/binaries/initramfs.cpio",
            "File that contains the initramfs image");
    initramfs_addr = Param.Addr(0x40000000, "initramfs address")

class PowerLinux(KernelWorkload):
    type = 'PowerLinux'
    cxx_header = 'arch/power/linux/fs_workload.hh'
    cxx_class = 'gem5::PowerISA::FsLinux'
    dtb_filename = Param.String("",
        "File that contains the Device Tree Blob. Don't use DTB if empty.")
    dtb_addr = Param.Addr(0x1800000, "DTB address")
    skiboot = Param.String("",
            "File that contains the OPAL firmware.");
    initramfs = Param.String("",
            "File that contains the initramfs image");
    early_kernel_symbols = Param.Bool(False,
        "enable early kernel symbol tables before MMU")