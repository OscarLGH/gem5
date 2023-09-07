/*
 * Copyright (c) 2012 ARM Limited
 * Copyright (c) 2020 Barkhausen Institut
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
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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

#include "arch/power/pagetable_walker.hh"

#include <memory>

#include "arch/power/faults.hh"
#include "arch/power/pagetable.hh"
#include "arch/power/tlb.hh"
#include "base/bitfield.hh"
#include "base/trie.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/PageTableWalker.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"

namespace gem5
{

namespace PowerISA {

uint64_t
Walker::readPhysMem(uint64_t addr, uint64_t dataSize)
{
    uint64_t ret;
    Request::Flags flags = Request::PHYSICAL;

    RequestPtr request =
        std::make_shared<Request>(addr, dataSize, flags, this->requestorId);
    Packet *read = new Packet(request, MemCmd::ReadReq);
    read->allocate();
    this->port.sendAtomic(read);
    ret = read->getLE<uint64_t>();

    read->deleteData();
    delete read;

    return ret;
}

uint64_t
Walker::writePhysMem(uint64_t addr, uint64_t data, uint64_t dataSize)
{
    uint64_t ret;
    Request::Flags flags = Request::PHYSICAL;

    RequestPtr request =
        std::make_shared<Request>(addr, dataSize, flags, this->requestorId);
    Packet *write = new Packet(request, MemCmd::WriteReq);
    write->allocate();
    write->setBE<uint64_t>(data);
    this->port.sendAtomic(write);
    ret = write->getLE<uint64_t>();

    write->deleteData();
    delete write;

    return ret;
}

Fault
Walker::prepareDSI(ThreadContext * tc, RequestPtr req,
                    BaseMMU::Mode mode, uint64_t BitMask)
{
    uint64_t dsisr = tc->readIntReg(INTREG_DSISR);
    dsisr = (dsisr & (~(DSISR_MASK))) | BitMask;
    if (mode == BaseMMU::Write)
      dsisr = dsisr | ISSTORE;
    tc->setIntReg(INTREG_DSISR, dsisr);
    tc->setIntReg(INTREG_DAR, req->getVaddr());
    return std::make_shared<DataStorageInterrupt>();
}

Fault
Walker::prepareISI(ThreadContext * tc, RequestPtr req,
                     uint64_t BitMask)
{
    Msr msr = tc->readIntReg(INTREG_MSR);
    //here unsetting SRR1 bits 33-36 and 42-47 according to ISA
    uint64_t srr1 = ((msr & unsetMask(31, 27)) & unsetMask(22,16)) | BitMask;
    tc->setIntReg(INTREG_SRR1, srr1);
    return std::make_shared<InstrStorageInterrupt>();
}

Fault
Walker::prepareSI(ThreadContext * tc, RequestPtr req,
                    BaseMMU::Mode mode, uint64_t BitMask)
{
    if (mode != BaseMMU::Execute)
      return prepareDSI(tc, req, mode, BitMask);
    else
      return prepareISI(tc, req, BitMask);
}

uint32_t geteffLPID(ThreadContext *tc)
{
    Msr msr = tc->readIntReg(INTREG_MSR);

    if (msr.hv)
        return 0;

    return tc->readIntReg(INTREG_LPIDR);
}

uint32_t geteffPID(ThreadContext *tc, Addr vaddr)
{
    Msr msr = tc->readIntReg(INTREG_MSR);
    uint64_t quadrant = vaddr & QUADRANT_MASK;

    if (msr.hv && !msr.pr) { //Hypervisor Kernel
        switch(quadrant) {
        case QUADRANT11:
            return 0;
        case QUADRANT10:
            return 0;
        default:
            return tc->readIntReg(INTREG_PIDR);
        }
    }

    //Hypervisor Userspace or Guest
    switch(quadrant) {
    case QUADRANT11:
        return 0;
    case QUADRANT00:
       return tc->readIntReg(INTREG_PIDR);
    default:
       if (msr.hv)
           panic("Errorenous hypervisor Userspace Quadrant : %lx\n", quadrant);
       else
           panic("Errorenous Guest Quadrant : %lx\n", quadrant);
    }

    return tc->readIntReg(INTREG_PIDR);
}


Fault
Walker::start(ThreadContext * _tc, BaseMMU::Translation *_translation,
              const RequestPtr &_req, BaseMMU::Mode _mode)
{
    Addr vaddr = _req->getVaddr();
    // prte0 ---> Process Table Entry
    DPRINTF(PageTableWalker, "Translating vaddr = 0x%016lx\n", vaddr);
    uint64_t prte0 = getRPDEntry(_tc, vaddr);
    // rpdb, rpds --->  Root Page Directory Base and Size
    uint64_t rpdb = extract(prte0, RPDB_SHIFT, RPDB_MASK);
    uint64_t rpds = extract(prte0, RPDS_SHIFT, RPDS_MASK);
    // rts = Radix Tree Size - 31.
    uint64_t rts = getRTS(prte0);

    uint64_t nextLevelBase = align(rpdb, DIR_BASE_ALIGN);
    uint64_t nextLevelSize = rpds;

    // usefulBits ---> radix tree size + 31
    // These are the useful lower bits of vaddr used for
    // address translation.
    uint64_t usefulBits = rts + 31; //Typically is 52.

    DPRINTF(PageTableWalker, "RPDB: 0x%lx RPDS: 0x%lx usefulBits: %ld\n"
            ,rpdb,rpds,usefulBits);

    std::pair<Addr, Fault> AddrTran;

    Lpcr lpcr = _tc->readIntReg(INTREG_LPCR);
    Msr msr = _tc->readIntReg(INTREG_MSR);
    AddrTran.first = vaddr;

    if (( lpcr.hr == 0 && lpcr.vc <= 3 ) ||
            ( msr.hv == 1 && msr.pr == 0 && msr.dr == 0 )) {
        AddrTran.second = prepareSI(_tc, _req, _mode,
                       PRTABLE_FAULT);
      DPRINTF(PageTableWalker, "Fault Generated due to Process table fault\n");
      return AddrTran.second;
    }

    AddrTran = this->walkRadixTree(vaddr, nextLevelBase, _tc, _mode, _req,
                                nextLevelSize, usefulBits);
    _req->setPaddr(AddrTran.first);
    if (AddrTran.second == NoFault) {

      DPRINTF(PageTableWalker, "Radix Translated 0x%016lx -> 0x%016lx\n",
      vaddr,AddrTran.first);
    }
    return AddrTran.second;
}

uint64_t
Walker::getRPDEntry(ThreadContext * tc, Addr vaddr)
{
    Ptcr ptcr = tc->readIntReg(INTREG_PTCR);
    uint32_t efflpid = geteffLPID(tc);

    DPRINTF(PageTableWalker,"PTCR:0x%016lx\n",(uint64_t)ptcr);
    DPRINTF(PageTableWalker,"effLPID: %d\n",efflpid);

    //Accessing 2nd double word of partition table (pate1)
    //Ref: Power ISA Manual v3.0B, Book-III, section 5.7.6.1
    //           PTCR Layout
    // ====================================================
    // -----------------------------------------------
    // | /// |     PATB                | /// | PATS  |
    // -----------------------------------------------
    // 0     4                       51 52 58 59    63
    // PATB[4:51] holds the base address of the Partition Table,
    // right shifted by 12 bits.
    // This is because the address of the Partition base is
    // 4k aligned. Hence, the lower 12bits, which are always
    // 0 are ommitted from the PTCR.
    //
    // Thus, The Partition Table Base is obtained by (PATB << 12)
    //
    // PATS represents the partition table size right-shifted by 12 bits.
    // The minimal size of the partition table is 4k.
    // Thus partition table size = (1 << PATS + 12).
    //
    //        Partition Table
    //  ====================================================
    //  0    PATE0            63  PATE1             127
    //  |----------------------|----------------------|
    //  |                      |                      |
    //  |----------------------|----------------------|
    //  |                      |                      |
    //  |----------------------|----------------------|
    //  |                      |                      | <-- effLPID
    //  |----------------------|----------------------|
    //           .
    //           .
    //           .
    //  |----------------------|----------------------|
    //  |                      |                      |
    //  |----------------------|----------------------|
    //
    // The effective LPID  forms the index into the Partition Table.
    //
    // Each entry in the partition table contains 2 double words, PATE0, PATE1,
    // corresponding to that partition.
    //
    // In case of Radix, The structure of PATE0 and PATE1 is as follows.
    //
    //     PATE0 Layout
    // -----------------------------------------------
    // |1|RTS1|/|     RPDB          | RTS2 |  RPDS   |
    // -----------------------------------------------
    //  0 1  2 3 4                55 56  58 59      63
    //
    // HR[0] : For Radix Page table, first bit should be 1.
    // RTS1[1:2] : Gives one fragment of the Radix treesize
    // RTS2[56:58] : Gives the second fragment of the Radix Tree size.
    // RTS = (RTS1 << 3 + RTS2) + 31.
    //
    // RPDB[4:55] = Root Page Directory Base.
    // RPDS = Logarithm of Root Page Directory Size right shifted by 3.
    //        Thus, Root page directory size = 1 << (RPDS + 3).
    //        Note: RPDS >= 5.
    //
    //   PATE1 Layout
    // -----------------------------------------------
    // |///|       PRTB             |  //  |  PRTS   |
    // -----------------------------------------------
    // 0  3 4                     51 52  58 59     63
    //
    // PRTB[4:51]   = Process Table Base. This is aligned to size.
    // PRTS[59: 63] = Process Table Size right shifted by 12.
    //                Minimal size of the process table is 4k.
    //                Process Table Size = (1 << PRTS + 12).
    //                Note: PRTS <= 24.
    //
    //                Computing the size aligned Process Table Base:
    //                   table_base = (PRTB  & ~((1 << PRTS) - 1)) << 12
    //                Thus, the lower 12+PRTS bits of table_base will
    //                be zero.

    uint64_t pate1Addr = align(ptcr.patb, TABLE_BASE_ALIGN) +
                         (efflpid*sizeof(uint64_t)*2) + 8;
    uint64_t dataSize = 8;
    uint64_t pate1 = betoh<uint64_t>(this->readPhysMem(pate1Addr, dataSize));
    DPRINTF(PageTableWalker,
            "2nd Double word of partition table entry: 0x%016lx\n",
            pate1);

    uint64_t prtb = extract(pate1, PRTB_SHIFT, PRTB_MASK);
    prtb = align(prtb, TABLE_BASE_ALIGN);

    //Ref: Power ISA Manual v3.0B, Book-III, section 5.7.6.2
    //
    //        Process Table
    // ==========================
    //  0    PRTE0            63  PRTE1             127
    //  |----------------------|----------------------|
    //  |                      |                      |
    //  |----------------------|----------------------|
    //  |                      |                      |
    //  |----------------------|----------------------|
    //  |                      |                      | <-- effPID
    //  |----------------------|----------------------|
    //           .
    //           .
    //           .
    //  |----------------------|----------------------|
    //  |                      |                      |
    //  |----------------------|----------------------|
    //
    // The effective Process id (PID) forms the index into the Process Table.
    //
    // Each entry in the partition table contains 2 double words, PRTE0, PRTE1,
    // corresponding to that process
    //
    // In case of Radix, The structure of PRTE0 and PRTE1 is as follows.
    //
    //     PRTE0 Layout
    // -----------------------------------------------
    // |/|RTS1|/|     RPDB          | RTS2 |  RPDS   |
    // -----------------------------------------------
    //  0 1  2 3 4                55 56  58 59      63
    //
    // RTS1[1:2] : Gives one fragment of the Radix treesize
    // RTS2[56:58] : Gives the second fragment of the Radix Tree size.
    // RTS = (RTS1 << 3 + RTS2) << 31,
    //        since minimal Radix Tree size is 4G.
    //
    // RPDB = Root Page Directory Base.
    // RPDS = Root Page Directory Size right shifted by 3.
    //        Thus, Root page directory size = RPDS << 3.
    //        Note: RPDS >= 5.
    //
    //   PRTE1 Layout
    // -----------------------------------------------
    // |                      ///                    |
    // -----------------------------------------------
    // 0                                            63
    // All bits are reserved.

    uint32_t effPID = geteffPID(tc, vaddr);
    DPRINTF(PageTableWalker,"effPID=%d\n", effPID);
    uint64_t prte0Addr = prtb + effPID*sizeof(prtb)*2 ;
    DPRINTF(PageTableWalker,"Process table base: 0x%016lx\n",prtb);
    uint64_t prte0 = betoh<uint64_t>(this->readPhysMem(prte0Addr, dataSize));
    //prte0 ---> Process Table Entry

    DPRINTF(PageTableWalker,"process table entry: 0x%016lx\n",prte0);
    return prte0;
}

std::pair<Addr, Fault>
Walker::walkRadixTree(Addr vaddr ,uint64_t curBase ,ThreadContext * tc ,
    BaseMMU::Mode mode , RequestPtr req, uint64_t curSize ,uint64_t usefulBits)
{
        uint64_t dataSize = 8;
        std::pair<Addr, Fault> AddrTran;

        if (curSize < 5) {
          DPRINTF(PageTableWalker,"[PANIC] vaddr = %lx,Radix RPDS = %lx,< 5\n",
                       vaddr, curSize);
          AddrTran.first = vaddr;
          AddrTran.second = prepareSI(tc, req, mode,
                          UNSUPP_MMU);
          DPRINTF(PageTableWalker,
            "Fault due to unsupported Radix Tree config\n");
        return AddrTran;
        }
        // vaddr                    64 Bit
        // vaddr |-----------------------------------------------------|
        //       | Unused    |  Used                                   |
        //       |-----------|-----------------------------------------|
        //       | 0000000   | usefulBits = X bits (typically 52)      |
        //       |-----------|-----------------------------------------|
        //       |           |<--Cursize---->|                         |
        //       |           |    Index      |                         |
        //       |           |    into Page  |                         |
        //       |           |    Directory  |                         |
        //       |-----------------------------------------------------|
        //                        |                       |
        //                        V                       |
        // PDE  |---------------------------|             |
        //      |V|L|//|  NLB       |///|NLS|             |
        //      |---------------------------|             |
        // PDE = Page Directory Entry                     |
        // [0] = V = Valid Bit                            |
        // [1] = L = Leaf bit. If 0, then                 |
        // [4:55] = NLB = Next Level Base                 |
        //                right shifted by 8              |
        // [59:63] = NLS = Next Level Size                |
        //            |    NLS >= 5                       |
        //            |                                   V
        //            |                     |--------------------------|
        //            |                     |   usfulBits = X-Cursize  |
        //            |                     |--------------------------|
        //            |---------------------><--NLS-->|                |
        //                                  | Index   |                |
        //                                  | into    |                |
        //                                  | PDE     |                |
        //                                  |--------------------------|
        //                                                    |
        // If the next PDE obtained by                        |
        // (NLB << 8 + 8 * index) is a                        |
        // nonleaf, then repeat the above.                    |
        //                                                    |
        // If the next PDE is a leaf,                         |
        // then Leaf PDE structure is as                      |
        // follows                                            |
        //                                                    |
        //                                                    |
        // Leaf PDE                                           |
        // |------------------------------|           |----------------|
        // |V|L|sw|//|RPN|sw|R|C|/|ATT|EAA|           | usefulBits     |
        // |------------------------------|           |----------------|
        // [0] = V = Valid Bit                                 |
        // [1] = L = Leaf Bit = 1 if leaf                      |
        //                      PDE                            |
        // [2] = Sw = Sw bit 0.                                |
        // [7:51] = RPN = Real Page Number,                    V
        //          real_page = RPN << 12 ------------->  Logical OR
        // [52:54] = Sw Bits 1:3                               |
        // [55] = R = Reference                                |
        // [56] = C = Change                                   V
        // [58:59] = Att =                                Physical Address
        //           0b00 = Normal Memory
        //           0b01 = SAO
        //           0b10 = Non Idenmpotent
        //           0b11 = Tolerant I/O
        // [60:63] = Encoded Access
        //           Authority
        //
        uint64_t shift = usefulBits - curSize;
        uint64_t mask = (1UL << curSize) - 1;
        uint64_t index = extract(vaddr, shift, mask);

        uint64_t entryAddr = curBase + (index * sizeof(uint64_t));
        Rpde rpde = betoh<uint64_t>(this->readPhysMem(entryAddr, dataSize));
        DPRINTF(PageTableWalker,"rpde:0x%016lx\n",(uint64_t)rpde);
        usefulBits = usefulBits - curSize;
        Msr msr = tc->readIntReg(INTREG_MSR);

        if (rpde.valid == 0) {
            AddrTran.first = vaddr;
            Lpcr lpcr = tc->readIntReg(INTREG_LPCR);

            if (lpcr.hr == 0) {
              AddrTran.second = prepareSI(tc, req, mode,
                        PRTABLE_FAULT);
              DPRINTF(PageTableWalker,
                "Fault generated due to invalid pt entry\n");
            }
            else if (msr.dr == 1 || msr.ir == 1) {
              AddrTran.second = prepareSI(tc, req, mode, NOHPTE);
              DPRINTF(PageTableWalker,
                "Fault due to translation not found\n");
            }
        return AddrTran;
           }

        //Ref: Power ISA Manual v3.0B, Book-III, section 5. 7.10.2
        if (rpde.leaf == 1) {
                Rpte  rpte = (uint64_t)rpde;
                uint64_t realpn = rpde & RPN_MASK;
                uint64_t pageMask = (1UL << usefulBits) - 1;
                Addr paddr = (realpn & ~pageMask) | (vaddr & pageMask);
                DPRINTF(PageTableWalker,"paddr:0x%016lx\n",paddr);
                AddrTran.second = NoFault;
                AddrTran.first = paddr;


                //Conditions for checking privileges and permissions
            if (mode == BaseMMU::Execute &&
                    (!rpte.exe || ( rpte.pri && msr.pr ))) {
              AddrTran.second = prepareISI(tc, req,
                            PROTFAULT);
              DPRINTF(PageTableWalker,
                "Fault is due to protection violation\n");
              }

            else if ( ( mode == BaseMMU::Read && !rpte.read ) ||
                      ( mode == BaseMMU::Write && !rpte.r_w ) ||
                      (( mode != BaseMMU::Execute)
                                && (rpte.pri && msr.pr ))) {
              AddrTran.second = prepareDSI(tc, req, mode,
                                  PROTFAULT);
              DPRINTF(PageTableWalker,
                "Fault is due to protection violation\n");
              }

            rpte.ref = 1;
            if (mode == BaseMMU::Write) {
                rpte.c = 1;
            }
            htobe<uint64_t>(rpte);
            this->writePhysMem(entryAddr, rpte, dataSize);

          return AddrTran;
        }

        uint64_t nextLevelBase = align(rpde.NLB, DIR_BASE_ALIGN);
        uint64_t nextLevelSize = rpde.NLS;
        DPRINTF(PageTableWalker,"NLB: 0x%lx\n",(uint64_t)nextLevelBase);
        DPRINTF(PageTableWalker,"NLS: 0x%lx\n",(uint64_t)nextLevelSize);
        DPRINTF(PageTableWalker,"usefulBits: %lx",(uint64_t)usefulBits);
        return walkRadixTree(vaddr, nextLevelBase, tc ,
                              mode, req, nextLevelSize, usefulBits);
}

/*
 * SLB handling
 */

Walker::ppc_slb_t * Walker::slb_lookup(ThreadContext * tc, Addr eaddr)
{
    uint64_t esid_256M, esid_1T;
    int n;

    esid_256M = (eaddr & SEGMENT_MASK_256M) | SLB_ESID_V;
    esid_1T = (eaddr & SEGMENT_MASK_1T) | SLB_ESID_V;

    int slb_size = 64;

    for (n = 0; n < slb_size; n++) {
        ppc_slb_t *slb = &slb[n]; //&env->slb[n]; //TODO

        /*
         * We check for 1T matches on all MMUs here - if the MMU
         * doesn't have 1T segment support, we will have prevented 1T
         * entries from being inserted in the slbmte code.
         */
        if (((slb->esid == esid_256M) &&
             ((slb->vsid & SLB_VSID_B) == SLB_VSID_B_256M))
            || ((slb->esid == esid_1T) &&
                ((slb->vsid & SLB_VSID_B) == SLB_VSID_B_1T))) {
            return slb;
        }
    }

    return NULL;
}

static inline Addr ppc_hash64_hpt_base(ThreadContext * tc)
{
    uint64_t base = tc->readIntReg(INTREG_ASDR);
    return base & SDR_64_HTABORG;
}

static inline Addr ppc_hash64_hpt_mask(ThreadContext * tc)
{
    uint64_t base = tc->readIntReg(INTREG_ASDR);

/*
    if (cpu->vhyp) {
        PPCVirtualHypervisorClass *vhc =
            PPC_VIRTUAL_HYPERVISOR_GET_CLASS(cpu->vhyp);
        return vhc->hpt_mask(cpu->vhyp);
    }
*/

    return (1ULL << ((base & SDR_64_HTABSIZE) + 18 - 7)) - 1;
}

uint64_t ppc_hash64_hpte0(const Walker::ppc_hash_pte64_t *hptes, int i)
{
    return betoh((uint64_t)&(hptes[i].pte0));
}

uint64_t ppc_hash64_hpte1(const Walker::ppc_hash_pte64_t *hptes, int i)
{
    return betoh((uint64_t)&(hptes[i].pte1));
}

Walker::ppc_hash_pte64_t * Walker::ppc_hash64_map_hptes(ThreadContext * tc,
                                             Addr ptex, int n)
{
    Addr pte_offset = ptex * HASH_PTE_SIZE_64;
    Addr base;
    Addr plen = n * HASH_PTE_SIZE_64;
    ppc_hash_pte64_t *hptes;

    base = ppc_hash64_hpt_base(tc);

    if (!base) {
        return NULL;
    }

    hptes = new ppc_hash_pte64_t[32];

    for (int i = 0; i < n; i++) {
        hptes[i].pte0 = readPhysMem(base + pte_offset, 8);
        hptes[i].pte1 = readPhysMem(base + pte_offset + 8, 8);
    }

    return hptes;
}

unsigned Walker::hpte_page_shift(const PPCHash64SegmentPageSizes *sps,
                                uint64_t pte0, uint64_t pte1)
{
    int i;

    if (!(pte0 & HPTE64_V_LARGE)) {
        if (sps->page_shift != 12) {
            /* 4kiB page in a non 4kiB segment */
            return 0;
        }
        /* Normal 4kiB page */
        return 12;
    }

    for (i = 0; i < PPC_PAGE_SIZES_MAX_SZ; i++) {
        const PPCHash64PageSize *ps = &sps->enc[i];
        uint64_t mask;

        if (!ps->page_shift) {
            break;
        }

        if (ps->page_shift == 12) {
            /* L bit is set so this can't be a 4kiB page */
            continue;
        }

        mask = ((1ULL << ps->page_shift) - 1) & HPTE64_R_RPN;

        if ((pte1 & mask) == ((uint64_t)ps->pte_enc << HPTE64_R_RPN_SHIFT)) {
            return ps->page_shift;
        }
    }

    return 0; /* Bad page size encoding */
}


Addr Walker::ppc_hash64_pteg_search(ThreadContext * tc, Addr hash,
                                     const PPCHash64SegmentPageSizes *sps,
                                     Addr ptem,
                                     ppc_hash_pte64_t *pte, unsigned *pshift)
{
    int i;
    const ppc_hash_pte64_t *pteg;
    Addr pte0, pte1;
    Addr ptex;

    ptex = (hash & ppc_hash64_hpt_mask(tc)) * HPTES_PER_GROUP;
    pteg = ppc_hash64_map_hptes(tc, ptex, HPTES_PER_GROUP);
    if (!pteg) {
        return -1;
    }
    for (i = 0; i < HPTES_PER_GROUP; i++) {
        pte0 = ppc_hash64_hpte0(pteg, i);
        /*
         * pte0 contains the valid bit and must be read before pte1,
         * otherwise we might see an old pte1 with a new valid bit and
         * thus an inconsistent hpte value
         */

        pte1 = ppc_hash64_hpte1(pteg, i);

        /* This compares V, B, H (secondary) and the AVPN */
        if (HPTE64_V_COMPARE(pte0, ptem)) {
            *pshift = hpte_page_shift(sps, pte0, pte1);
            /*
             * If there is no match, ignore the PTE, it could simply
             * be for a different segment size encoding and the
             * architecture specifies we should not match. Linux will
             * potentially leave behind PTEs for the wrong base page
             * size when demoting segments.
             */
            if (*pshift == 0) {
                continue;
            }
            /*
             * We don't do anything with pshift yet as qemu TLB only
             * deals with 4K pages anyway
             */
            pte->pte0 = pte0;
            pte->pte1 = pte1;

            return ptex + i;
        }
    }
    delete pteg;
    /*
     * We didn't find a valid entry.
     */
    return -1;
}

Addr Walker::ppc_hash64_htab_lookup(ThreadContext *tc,
                                     ppc_slb_t *slb, Addr eaddr,
                                     ppc_hash_pte64_t *pte, unsigned *pshift)
{
    Addr hash, ptex;
    uint64_t vsid, epnmask, epn, ptem;
    const PPCHash64SegmentPageSizes *sps = slb->sps;

    /*
     * The SLB store path should prevent any bad page size encodings
     * getting in there, so:
     */
    assert(sps);

    Lpcr lpcr = tc->readIntReg(INTREG_LPCR);

    /* If ISL is set in LPCR we need to clamp the page size to 4K */
    // if (lpcr & LPCR_ISL) {
    //     /* We assume that when using TCG, 4k is first entry of SPS */
    //     sps = &cpu->hash64_opts->sps[0];
    //     assert(sps->page_shift == 12);
    // }

    epnmask = ~((1ULL << sps->page_shift) - 1);

    if (slb->vsid & SLB_VSID_B) {
        /* 1TB segment */
        vsid = (slb->vsid & SLB_VSID_VSID) >> SLB_VSID_SHIFT_1T;
        epn = (eaddr & ~SEGMENT_MASK_1T) & epnmask;
        hash = vsid ^ (vsid << 25) ^ (epn >> sps->page_shift);
    } else {
        /* 256M segment */
        vsid = (slb->vsid & SLB_VSID_VSID) >> SLB_VSID_SHIFT;
        epn = (eaddr & ~SEGMENT_MASK_256M) & epnmask;
        hash = vsid ^ (epn >> sps->page_shift);
    }
    ptem = (slb->vsid & SLB_VSID_PTEM) | ((epn >> 16) & HPTE64_V_AVPN);
    ptem |= HPTE64_V_VALID;

    /* Page address translation */
    DPRINTF(PageTableWalker,
            "htab_base " "%llx" " htab_mask " "%llx"
            " hash " "%llx" "\n",
            ppc_hash64_hpt_base(tc), ppc_hash64_hpt_mask(tc), hash);

    /* Primary PTEG lookup */
    DPRINTF(PageTableWalker,
            "0 htab=" "%llx" "/" "%llx"
            " vsid=" "%llx" " ptem=" "%llx"
            " hash=" "%llx" "\n",
            ppc_hash64_hpt_base(tc), ppc_hash64_hpt_mask(tc),
            vsid, ptem,  hash);
    ptex = ppc_hash64_pteg_search(tc, hash, sps, ptem, pte, pshift);

    if (ptex == -1) {
        /* Secondary PTEG lookup */
        ptem |= HPTE64_V_SECONDARY;
        DPRINTF(PageTableWalker,
                "1 htab=" "%llx" "/" "%llx"
                " vsid=" "%llx" " api=" "%llx"
                " hash=" "%llx" "\n", ppc_hash64_hpt_base(tc),
                ppc_hash64_hpt_mask(tc), vsid, ptem, ~hash);

        ptex = ppc_hash64_pteg_search(tc, ~hash, sps, ptem, pte, pshift);
    }

    return ptex;
}

/* Check No-Execute or Guarded Storage */
int Walker::ppc_hash64_pte_noexec_guard(ppc_hash_pte64_t pte)
{
    /* Exec permissions CANNOT take away read or write permissions */
    return (pte.pte1 & HPTE64_R_N) || (pte.pte1 & HPTE64_R_G) ?
            PAGE_READ | PAGE_WRITE : PAGE_READ | PAGE_WRITE | PAGE_EXEC;
}

/* Check Basic Storage Protection */
int Walker::ppc_hash64_pte_prot(int mmu_idx,
                               ppc_slb_t *slb, ppc_hash_pte64_t pte)
{
    unsigned pp, key;
    /*
     * Some pp bit combinations have undefined behaviour, so default
     * to no access in those cases
     */
    int prot = 0;

    key = !!(mmuidx_pr(mmu_idx) ? (slb->vsid & SLB_VSID_KP)
             : (slb->vsid & SLB_VSID_KS));
    pp = (pte.pte1 & HPTE64_R_PP) | ((pte.pte1 & HPTE64_R_PP0) >> 61);

    if (key == 0) {
        switch (pp) {
        case 0x0:
        case 0x1:
        case 0x2:
            prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
            break;

        case 0x3:
        case 0x6:
            prot = PAGE_READ | PAGE_EXEC;
            break;
        }
    } else {
        switch (pp) {
        case 0x0:
        case 0x6:
            break;

        case 0x1:
        case 0x3:
            prot = PAGE_READ | PAGE_EXEC;
            break;

        case 0x2:
            prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
            break;
        }
    }

    return prot;
}

/* Check the instruction access permissions specified in the IAMR */
int Walker::ppc_hash64_iamr_prot(ThreadContext * tc, int key)
{
    uint64_t iamr = tc->readIntReg(INTREG_IAMR);
    int iamr_bits = (iamr >> 2 * (31 - key)) & 0x3;

    /*
     * An instruction fetch is permitted if the IAMR bit is 0.
     * If the bit is set, return PAGE_READ | PAGE_WRITE because this bit
     * can only take away EXEC permissions not READ or WRITE permissions.
     * If bit is cleared return PAGE_READ | PAGE_WRITE | PAGE_EXEC since
     * EXEC permissions are allowed.
     */
    return (iamr_bits & 0x1) ? PAGE_READ | PAGE_WRITE :
                               PAGE_READ | PAGE_WRITE | PAGE_EXEC;
}

int Walker::ppc_hash64_amr_prot(ThreadContext * tc, ppc_hash_pte64_t pte)
{
    int key, amrbits;
    int prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;

    uint64_t amr = tc->readIntReg(INTREG_AMR);

    key = HPTE64_R_KEY(pte.pte1);
    amrbits = (amr >> 2 * (31 - key)) & 0x3;

    /* fprintf(stderr, "AMR protection: key=%d AMR=0x%" PRIx64 "\n", key, */
    /*         env->spr[SPR_AMR]); */

    /*
     * A store is permitted if the AMR bit is 0. Remove write
     * protection if it is set.
     */
    if (amrbits & 0x2) {
        prot &= ~PAGE_WRITE;
    }
    /*
     * A load is permitted if the AMR bit is 0. Remove read
     * protection if it is set.
     */
    if (amrbits & 0x1) {
        prot &= ~PAGE_READ;
    }

    /*
     * MMU version 2.07 and later support IAMR
     * Check if the IAMR allows the instruction access - it will return
     * PAGE_EXEC if it doesn't (and thus that bit will be cleared) or 0
     * if it does (and prot will be unchanged indicating execution support).
     */
    prot &= ppc_hash64_iamr_prot(tc, key);

    return prot;
}

static inline uint64_t deposit64(uint64_t value, int start, int length,
                                 uint64_t fieldval)
{
    uint64_t mask;
    assert(start >= 0 && length > 0 && length <= 64 - start);
    mask = (~0ULL >> (64 - length)) << start;
    return (value & ~mask) | ((fieldval << start) & mask);
}

static inline int prot_for_access_type(BaseMMU::Mode mode)
{
    switch (mode) {
    case BaseMMU::Execute:
        return PAGE_EXEC;
    case BaseMMU::Read:
        return PAGE_READ;
    case BaseMMU::Write:
        return PAGE_WRITE;
    }
}

void Walker::ppc_hash64_set_r(ThreadContext * tc, Addr ptex, uint64_t pte1)
{
    Addr base, offset = ptex * HASH_PTE_SIZE_64 + HPTE64_DW1_R;

    base = ppc_hash64_hpt_base(tc);


    /* The HW performs a non-atomic byte update */
    //stb_phys(CPU(cpu)->as, base + offset, ((pte1 >> 8) & 0xff) | 0x01);
    writePhysMem(base + offset, ((pte1 >> 8) & 0xff) | 0x01, 1);
}

void Walker::ppc_hash64_set_c(ThreadContext * tc, Addr ptex, uint64_t pte1)
{
    Addr base, offset = ptex * HASH_PTE_SIZE_64 + HPTE64_DW1_C;

    base = ppc_hash64_hpt_base(tc);

    /* The HW performs a non-atomic byte update */
    //stb_phys(CPU(cpu)->as, base + offset, (pte1 & 0xff) | 0x80);
    writePhysMem(base + offset, (pte1 & 0xff) | 0x80, 1);
}

std::pair<Addr, Fault>
Walker::walkHashTable(Addr vaddr ,ThreadContext * tc,
    BaseMMU::Mode mode , RequestPtr req)
{
    ppc_slb_t vrma_slbe;
    ppc_slb_t *slb;
    unsigned apshift;
    Addr ptex;
    ppc_hash_pte64_t pte;
    int exec_prot, pp_prot, amr_prot, prot;
    int need_prot;
    Addr raddr;

    int mmu_idx = 0;

    std::pair<Addr, Fault> AddrTran;

    Lpcr lpcr = tc->readIntReg(INTREG_LPCR);
    Msr msr = tc->readIntReg(INTREG_MSR);

    /*
     * Note on LPCR usage: 970 uses HID4, but our special variant of
     * store_spr copies relevant fields into env->spr[SPR_LPCR].
     * Similarly we filter unimplemented bits when storing into LPCR
     * depending on the MMU version. This code can thus just use the
     * LPCR "as-is".
     */

    /* 1. Handle real mode accesses */

    /* 2. Translation is on, so look up the SLB */
    slb = slb_lookup(tc, vaddr);
    if (!slb) {
        /* No entry found, check if in-memory segment tables are in use */

        /* Segment still not found, generate the appropriate interrupt */
        AddrTran.first = vaddr;
        if (lpcr.hr == 0) {
            AddrTran.second = prepareSI(tc, req, mode,
                    PRTABLE_FAULT);
            DPRINTF(PageTableWalker,
            "Fault generated due to invalid pt entry\n");
        }
        else if (msr.dr == 1 || msr.ir == 1) {
            AddrTran.second = prepareSI(tc, req, mode, NOHPTE);
            DPRINTF(PageTableWalker,
            "Fault due to translation not found\n");
        }
    }

 skip_slb_search:

    /* 3. Check for segment level no-execute violation */
    if (mode == BaseMMU::Execute && (slb->vsid & SLB_VSID_N)) {
        AddrTran.second = prepareSI(tc, req, mode, PRTABLE_FAULT);
        return AddrTran;
    }

    /* 4. Locate the PTE in the hash table */
    ptex = ppc_hash64_htab_lookup(tc, slb, vaddr, &pte, &apshift);
    if (ptex == -1) {
        if (mode == BaseMMU::Execute) {
            AddrTran.second = prepareISI(tc, req, NOHPTE);
        } else {
            AddrTran.second = prepareDSI(tc, req, mode, NOHPTE);
        }
        return AddrTran;
    }
    DPRINTF(PageTableWalker,
                  "found PTE at index %08" "%llx" "\n", ptex);

    /* 5. Check access permissions */

    exec_prot = ppc_hash64_pte_noexec_guard(pte);
    pp_prot = ppc_hash64_pte_prot(mmu_idx, slb, pte);
    amr_prot = ppc_hash64_amr_prot(tc, pte);
    prot = exec_prot & pp_prot & amr_prot;

    need_prot = prot_for_access_type(mode);
    if (need_prot & ~prot) {
        /* Access right violation */
        DPRINTF(PageTableWalker, "PTE access rejected\n");

        if (mode == BaseMMU::Execute) {
            int srr1 = 0;
            if (PAGE_EXEC & ~exec_prot) {
                /* Access violates noexec or guard */
                srr1 |= SRR1_NOEXEC_GUARD;
            } else if (PAGE_EXEC & ~pp_prot) {
                /* Access violates access authority */
                srr1 |= SRR1_PROTFAULT;
            }
            if (PAGE_EXEC & ~amr_prot) {
                /* Access violates virt pg class key prot */
                srr1 |= SRR1_IAMR;
            }
            //ppc_hash64_set_isi(cs, mmu_idx, slb->vsid, srr1);
            AddrTran.second = prepareISI(tc, req, srr1);
        } else {
            int dsisr = 0;
            if (need_prot & ~pp_prot) {
                dsisr |= DSISR_PROTFAULT;
            }
            if (mode == BaseMMU::Write) {
                dsisr |= DSISR_ISSTORE;
            }
            if (need_prot & ~amr_prot) {
                dsisr |= DSISR_AMR;
            }
            //ppc_hash64_set_dsi(cs, mmu_idx, slb->vsid, eaddr, dsisr);
            AddrTran.second = prepareDSI(tc, req, mode, dsisr);
        }
        return AddrTran;
    }

    DPRINTF(PageTableWalker, "PTE access granted !\n");

    /* 6. Update PTE referenced and changed bits if necessary */

    if (!(pte.pte1 & HPTE64_R_R)) {
        ppc_hash64_set_r(tc, ptex, pte.pte1);
    }
    if (!(pte.pte1 & HPTE64_R_C)) {
        if (mode == BaseMMU::Write) {
            ppc_hash64_set_c(tc, ptex, pte.pte1);
        } else {
            /*
             * Treat the page as read-only for now, so that a later write
             * will pass through this function again to set the C bit
             */
            prot &= ~PAGE_WRITE;
        }
    }

    /* 7. Determine the real address from the PTE */

    //*raddrp = deposit64(pte.pte1 & HPTE64_R_RPN, 0, apshift, vaddr);
    //*protp = prot;
    //*psizep = apshift;

    AddrTran.second = NoFault;
    AddrTran.first = deposit64(pte.pte1 & HPTE64_R_RPN, 0, apshift, vaddr);
    DPRINTF(PageTableWalker,"paddr:0x%016lx\n",AddrTran.first);
    return AddrTran;
}

Fault
Walker::startFunctional(ThreadContext * _tc, Addr &addr, unsigned &logBytes,
              BaseMMU::Mode _mode)
{
    funcState.initState(_tc, _mode);
    return funcState.startFunctional(addr, logBytes);
}

bool
Walker::WalkerPort::recvTimingResp(PacketPtr pkt)
{
    return walker->recvTimingResp(pkt);
}

bool
Walker::recvTimingResp(PacketPtr pkt)
{
    WalkerSenderState * senderState =
        dynamic_cast<WalkerSenderState *>(pkt->popSenderState());
    WalkerState * senderWalk = senderState->senderWalk;
    bool walkComplete = senderWalk->recvPacket(pkt);
    delete senderState;
    if (walkComplete) {
        std::list<WalkerState *>::iterator iter;
        for (iter = currStates.begin(); iter != currStates.end(); iter++) {
            WalkerState * walkerState = *(iter);
            if (walkerState == senderWalk) {
                iter = currStates.erase(iter);
                break;
            }
        }
        delete senderWalk;
        // Since we block requests when another is outstanding, we
        // need to check if there is a waiting request to be serviced
        if (currStates.size() && !startWalkWrapperEvent.scheduled())
            // delay sending any new requests until we are finished
            // with the responses
            schedule(startWalkWrapperEvent, clockEdge());
    }
    return true;
}

void
Walker::WalkerPort::recvReqRetry()
{
    walker->recvReqRetry();
}

void
Walker::recvReqRetry()
{
    std::list<WalkerState *>::iterator iter;
    for (iter = currStates.begin(); iter != currStates.end(); iter++) {
        WalkerState * walkerState = *(iter);
        if (walkerState->isRetrying()) {
            walkerState->retry();
        }
    }
}

bool Walker::sendTiming(WalkerState* sendingState, PacketPtr pkt)
{
    WalkerSenderState* walker_state = new WalkerSenderState(sendingState);
    pkt->pushSenderState(walker_state);
    if (port.sendTimingReq(pkt)) {
        return true;
    } else {
        // undo the adding of the sender state and delete it, as we
        // will do it again the next time we attempt to send it
        pkt->popSenderState();
        delete walker_state;
        return false;
    }

}

Port &
Walker::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return ClockedObject::getPort(if_name, idx);
}

void
Walker::WalkerState::initState(ThreadContext * _tc,
        BaseMMU::Mode _mode, bool _isTiming)
{
    assert(state == Ready);
    started = false;
    tc = _tc;
    mode = _mode;
    timing = _isTiming;
}

void
Walker::startWalkWrapper()
{

}

Fault
Walker::WalkerState::startWalk()
{
    Fault fault = NoFault;
    return fault;
}

Fault
Walker::WalkerState::startFunctional(Addr &addr, unsigned &logBytes)
{
    Fault fault = NoFault;
    return fault;
}

Fault
Walker::WalkerState::stepWalk(PacketPtr &write)
{
    Fault fault = NoFault;
    return fault;
}

void
Walker::WalkerState::endWalk()
{

}

void
Walker::WalkerState::setupWalk(Addr vaddr)
{

}

bool
Walker::WalkerState::recvPacket(PacketPtr pkt)
{
    assert(pkt->isResponse());
    assert(inflight);
    assert(state == Waiting);
    inflight--;
    if (squashed) {
        // if were were squashed, return true once inflight is zero and
        // this WalkerState will be freed there.
        return (inflight == 0);
    }
    if (pkt->isRead()) {
        // should not have a pending read it we also had one outstanding
        assert(!read);

        // @todo someone should pay for this
        pkt->headerDelay = pkt->payloadDelay = 0;

        state = nextState;
        nextState = Ready;
        PacketPtr write = NULL;
        read = pkt;
        timingFault = stepWalk(write);
        state = Waiting;
        assert(timingFault == NoFault || read == NULL);
        if (write) {
            writes.push_back(write);
        }
        sendPackets();
    } else {
        sendPackets();
    }
    if (inflight == 0 && read == NULL && writes.size() == 0) {
        state = Ready;
        nextState = Waiting;
        if (timingFault == NoFault) {
            /*
             * Finish the translation. Now that we know the right entry is
             * in the TLB, this should work with no memory accesses.
             * There could be new faults unrelated to the table walk like
             * permissions violations, so we'll need the return value as
             * well.
             */

            // Let the CPU continue.
            translation->finish(timingFault, req, tc, mode);
        } else {
            // There was a fault during the walk. Let the CPU know.
            translation->finish(timingFault, req, tc, mode);
        }
        return true;
    }

    return false;
}

void
Walker::WalkerState::sendPackets()
{
    //If we're already waiting for the port to become available, just return.
    if (retrying)
        return;

    //Reads always have priority
    if (read) {
        PacketPtr pkt = read;
        read = NULL;
        inflight++;
        if (!walker->sendTiming(this, pkt)) {
            retrying = true;
            read = pkt;
            inflight--;
            return;
        }
    }
    //Send off as many of the writes as we can.
    while (writes.size()) {
        PacketPtr write = writes.back();
        writes.pop_back();
        inflight++;
        if (!walker->sendTiming(this, write)) {
            retrying = true;
            writes.push_back(write);
            inflight--;
            return;
        }
    }
}

unsigned
Walker::WalkerState::numInflight() const
{
    return inflight;
}

bool
Walker::WalkerState::isRetrying()
{
    return retrying;
}

bool
Walker::WalkerState::isTiming()
{
    return timing;
}

bool
Walker::WalkerState::wasStarted()
{
    return started;
}

void
Walker::WalkerState::squash()
{
    squashed = true;
}

void
Walker::WalkerState::retry()
{
    retrying = false;
    sendPackets();
}

Fault
Walker::WalkerState::pageFault(bool present)
{
    DPRINTF(PageTableWalker, "Raising page fault.\n");
    //return walker->tlb->createPagefault(entry.vaddr, mode);
    return NoFault;
}

} // namespace PowerISA
} // namespace gem5
