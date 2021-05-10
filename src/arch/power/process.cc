/*
 * Copyright (c) 2007-2008 The Florida State University
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

#include "arch/power/process.hh"

#include "arch/power/decoder.hh"
#include "arch/power/isa_traits.hh"
#include "arch/power/types.hh"
#include "arch/power/utility.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/page_table.hh"
#include "params/Process.hh"
#include "sim/aux_vector.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_return.hh"
#include "sim/system.hh"

using namespace PowerISA;

PowerProcess::PowerProcess(
       const ProcessParams &params, ::Loader::ObjectFile *objFile)
    : Process(params,
              new EmulationPageTable(params.name, params.pid, PageBytes),
              objFile)
{
    fatal_if(params.useArchPT, "Arch page tables not implemented.");
    // Set up break point (Top of Heap)
    Addr brk_point = image.maxAddr();
    brk_point = roundUp(brk_point, PageBytes);

    Addr stack_base = 0x3ffffb100000L;

    Addr max_stack_size = 8 * 1024 * 1024;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    Addr next_thread_stack_base = stack_base - max_stack_size;

    // Set up region for mmaps. For now, start at bottom of kuseg space.
    Addr mmap_end = 0x70000000L;

    memState = std::make_shared<MemState>(
            this, brk_point, stack_base, max_stack_size,
            next_thread_stack_base, mmap_end);
}

void
PowerProcess::initState()
{
    Process::initState();

    argsInit(sizeof(uint64_t), PageBytes);
}

void
PowerProcess::argsInit(int intSize, int pageSize)
{
    typedef AuxVector<uint64_t> auxv_t;
    std::vector<auxv_t> auxv;

    pageSize = 0x1000;

    std::string filename;
    if (argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    //We want 16 byte alignment
    uint64_t align = 16;

    // Patch the ld_bias for dynamic executables.
    updateBias();

    // load object file into target memory
    image.write(*initVirtMem);
    interpImage.write(*initVirtMem);

    enum PowerCpuFeature {
        Power_32 = ULL(1) << 31,            // Always set for powerpc64
        Power_64 = ULL(1) << 30,            // Always set for powerpc64
        Power_HAS_ALTIVEC = ULL(1) << 28,
        Power_HAS_FPU = ULL(1) << 27,
        Power_HAS_MMU = ULL(1) << 26,
        Power_UNIFIED_CACHE = ULL(1) << 24,
        Power_NO_TB = ULL(1) << 20,         // 601/403gx have no timebase
        Power_POWER4 = ULL(1) << 19,        // POWER4 ISA 2.00
        Power_POWER5 = ULL(1) << 18,        // POWER5 ISA 2.02
        Power_POWER5_PLUS = ULL(1) << 17,   // POWER5+ ISA 2.03
        Power_CELL_BE = ULL(1) << 16,       // CELL Broadband Engine
        Power_BOOKE = ULL(1) << 15,         // ISA Category Embedded
        Power_SMT = ULL(1) << 14,           // Simultaneous Multi-Threading
        Power_ICACHE_SNOOP = ULL(1) << 13,
        Power_ARCH_2_05 = ULL(1) << 12,     // ISA 2.05
        Power_PA6T = ULL(1) << 11,          // PA Semi 6T Core
        Power_HAS_DFP = ULL(1) << 10,       // Decimal FP Unit
        Power_POWER6_EXT = ULL(1) << 9,     // P6 + mffgpr/mftgpr
        Power_ARCH_2_06 = ULL(1) << 8,      // ISA 2.06
        Power_HAS_VSX = ULL(1) << 7,        // P7 Vector Extension
        Power_PSERIES_PERFMON_COMPAT = ULL(1) << 6,
        Power_TRUE_LE = ULL(1) << 1,
        Power_PPC_LE = ULL(1) << 0
    };

    //Setup the auxilliary vectors. These will already have endian conversion.
    //Auxilliary vectors are loaded only for elf formatted executables.
    auto *elfObject = dynamic_cast<::Loader::ElfObject *>(objFile);
    if (elfObject) {
        uint64_t features = Power_32 | Power_64 | Power_PPC_LE;

        //Bits which describe the system hardware capabilities
        //XXX Figure out what these should be
        auxv.emplace_back(M5_AT_HWCAP, features);
        //The system page size
        auxv.emplace_back(M5_AT_PAGESZ, PowerISA::PageBytes);
        //Frequency at which times() increments
        auxv.emplace_back(M5_AT_CLKTCK, 0x64);
        // For statically linked executables, this is the virtual address of
        // the program header tables if they appear in the executable image
        auxv.emplace_back(M5_AT_PHDR, elfObject->programHeaderTable());
        // This is the size of a program header entry from the elf file.
        auxv.emplace_back(M5_AT_PHENT, elfObject->programHeaderSize());
        // This is the number of program headers from the original elf file.
        auxv.emplace_back(M5_AT_PHNUM, elfObject->programHeaderCount());
        // This is the base address of the ELF interpreter; it should be
        // zero for static executables or contain the base address for
        // dynamic executables.
        auxv.emplace_back(M5_AT_BASE, getBias());
        //XXX Figure out what this should be.
        auxv.emplace_back(M5_AT_FLAGS, 0);
        //The entry point to the program
        auxv.emplace_back(M5_AT_ENTRY, objFile->entryPoint());
        //Different user and group IDs
        auxv.emplace_back(M5_AT_UID, uid());
        auxv.emplace_back(M5_AT_EUID, euid());
        auxv.emplace_back(M5_AT_GID, gid());
        auxv.emplace_back(M5_AT_EGID, egid());
        //Whether to enable "secure mode" in the executable
        auxv.emplace_back(M5_AT_SECURE, 0);
        //The filename of the program
        auxv.emplace_back(M5_AT_EXECFN, 0);
        //The string "v51" with unknown meaning
        auxv.emplace_back(M5_AT_PLATFORM, 0);
        //The address of 16 bytes in the data section containing a random
        //value; it is required for stack protection using a canary value.
        auxv.emplace_back(M5_AT_RANDOM, 0);
    }

    //Figure out how big the initial stack nedes to be

    // A sentry NULL void pointer at the top of the stack.
    int sentry_size = intSize;

    std::string platform = "v51";
    int platform_size = platform.size() + 1;

    // The aux vectors are put on the stack in two groups. The first group are
    // the vectors that are generated as the elf is loaded. The second group
    // are the ones that were computed ahead of time and include the platform
    // string.
    int aux_data_size = filename.size() + 1;

    const int numRandomBytes = 16;
    aux_data_size += numRandomBytes;

    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }

    int info_block_size =
        sentry_size + env_data_size + arg_data_size +
        aux_data_size + platform_size;

    //Each auxilliary vector is two 4 byte words
    int aux_array_size = intSize * 2 * (auxv.size() + 1);

    int envp_array_size = intSize * (envp.size() + 1);
    int argv_array_size = intSize * (argv.size() + 1);

    int argc_size = intSize;

    //Figure out the size of the contents of the actual initial frame
    int frame_size =
        info_block_size +
        aux_array_size +
        envp_array_size +
        argv_array_size +
        argc_size;

    //There needs to be padding after the auxiliary vector data so that the
    //very bottom of the stack is aligned properly.
    int partial_size = frame_size;
    int aligned_partial_size = roundUp(partial_size, align);
    int aux_padding = aligned_partial_size - partial_size;

    int space_needed = frame_size + aux_padding;

    Addr stack_min = memState->getStackBase() - space_needed;
    stack_min = roundDown(stack_min, align);

    memState->setStackSize(memState->getStackBase() - stack_min);

    // map memory
    memState->mapRegion(roundDown(stack_min, pageSize),
                        roundUp(memState->getStackSize(), pageSize), "stack");

    // map out initial stack contents
    uint64_t sentry_base = memState->getStackBase() - sentry_size;
    uint64_t aux_data_base = sentry_base - aux_data_size;
    uint64_t env_data_base = aux_data_base - env_data_size;
    uint64_t arg_data_base = env_data_base - arg_data_size;
    uint64_t platform_base = arg_data_base - platform_size;
    uint64_t auxv_array_base = platform_base - aux_array_size - aux_padding;
    uint64_t envp_array_base = auxv_array_base - envp_array_size;
    uint64_t argv_array_base = envp_array_base - argv_array_size;
    uint64_t argc_base = argv_array_base - argc_size;

    DPRINTF(Stack, "The addresses of items on the initial stack:\n");
    DPRINTF(Stack, "0x%x - aux data\n", aux_data_base);
    DPRINTF(Stack, "0x%x - env data\n", env_data_base);
    DPRINTF(Stack, "0x%x - arg data\n", arg_data_base);
    DPRINTF(Stack, "0x%x - platform base\n", platform_base);
    DPRINTF(Stack, "0x%x - auxv array\n", auxv_array_base);
    DPRINTF(Stack, "0x%x - envp array\n", envp_array_base);
    DPRINTF(Stack, "0x%x - argv array\n", argv_array_base);
    DPRINTF(Stack, "0x%x - argc \n", argc_base);
    DPRINTF(Stack, "0x%x - stack min\n", stack_min);

    // write contents to stack
    //uint64_t data = 1;
    //initVirtMem->writeBlob(memState->getStackBase(), &data, 8);

    // figure out argc
    uint64_t argc = argv.size();
    ThreadContext *tc = system->threads[contextIds[0]];
    uint64_t guestArgc = htog<uint64_t>(argc, byteOrder(tc));

    //Write out the sentry void *
    uint64_t sentry_NULL = 0;
    initVirtMem->writeBlob(sentry_base,
            (uint8_t*)&sentry_NULL, sentry_size);

    //Fix up the aux vectors which point to other data
    for (int i = auxv.size() - 1; i >= 0; i--) {
        if (auxv[i].type == M5_AT_PLATFORM) {
            auxv[i].val = platform_base;
            initVirtMem->writeString(platform_base, platform.c_str());
        } else if (auxv[i].type == M5_AT_EXECFN) {
            auxv[i].val = aux_data_base + numRandomBytes;
            initVirtMem->writeString(aux_data_base, filename.c_str());
        } else if (auxv[i].type == M5_AT_RANDOM) {
            auxv[i].val = aux_data_base;
        }
    }

    Msr msr = tc->readIntReg(INTREG_MSR);

    if (elfObject->elf_endian() == ByteOrder::little) {
        msr.le = 1;
    } else {
        msr.le = 0;
    }
    tc->getDecoderPtr()->fetchByteOrder(elfObject->elf_endian());
    tc->setIntReg(INTREG_MSR, msr);

    //Copy the aux stuff
    Addr auxv_array_end = auxv_array_base;
    for (const auto &aux: auxv) {
        initVirtMem->write(auxv_array_end, aux, byteOrder(tc));
        auxv_array_end += sizeof(aux);
    }
    //Write out the terminating zeroed auxilliary vector
    const AuxVector<uint64_t> zero(0, 0);
    initVirtMem->write(auxv_array_end, zero);
    auxv_array_end += sizeof(zero);

    copyStringArray(envp, envp_array_base, env_data_base,
                    byteOrder(tc), *initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base,
                    byteOrder(tc), *initVirtMem);

    initVirtMem->writeBlob(argc_base, &guestArgc, intSize);

    //Set the stack pointer register
    tc->setIntReg(StackPointerReg, stack_min);

    if (elfObject->elf_flags() == 0x1) {
        /* abi v1 */
        uint64_t entry_point;
        uint64_t vaddr = getStartPC();
        PortProxy &proxy = tc->getVirtProxy();
        int size = 2;
        uint64_t is[size];
        proxy.readBlob(vaddr, &is, sizeof(is));
        entry_point = gtoh(is[0], elfObject->elf_endian());
        tc->pcState(entry_point);
    } else {
        /* abi v2 and above */
        tc->pcState(getStartPC());
    }

    //Align the "stack_min" to a page boundary.
    memState->setStackMin(roundDown(stack_min, pageSize));

    // write contents to stack
    uint64_t data = 1;
    initVirtMem->writeBlob(stack_min, &data, 8);
}
