/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
#include "arch/power/tlb.hh"

#include <fcntl.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include "arch/power/faults.hh"
#include "arch/power/pagetable_walker.hh"
#include "arch/power/pagetable.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Power.hh"
#include "debug/TLB.hh"
#include "mem/page_table.hh"
#include "params/PowerTLB.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

namespace gem5
{

using namespace PowerISA;

///////////////////////////////////////////////////////////////////////
//
//  POWER TLB
//

#define MODE2MASK(X) (1 << (X))

TLB::TLB(const Params &p) : BaseTLB(p), size(p.size), nlu(0)
{
    table = new PowerISA::PTE[size];
    memset(table, 0, sizeof(PowerISA::PTE[size]));
    smallPages = 0;
    walker = p.walker;
    walker->setTLB(this);
}

TLB::~TLB()
{
    if (table)
        delete [] table;
}

// look up an entry in the TLB
PowerISA::PTE *
TLB::lookup(Addr vpn, uint8_t tid) const
{
    // assume not found...
    PowerISA::PTE *retval = NULL;
    for (int i = 0; i < size; i++) {
        PowerISA::PTE *pte = &table[i];
        Addr Mask = pte->Mask;
        Addr VPN  = pte->VPN;
        if (((vpn & Mask) == (VPN & Mask))
            && pte->V && (tid == pte->thread_id)) {

            // We have a VPN + ASID Match
            retval = pte;
            break;
        }
    }

    DPRINTF(TLB, "lookup %#x, tid %#x -> %s ppn %#x\n", vpn, (int)tid,
            retval ? "hit" : "miss", retval ? retval->PFN : 0);
    return retval;
}

// insert a new TLB entry
void
TLB::insert(Addr vfn, Addr pfn, int prot, bool c, int page_shift, int tid)
{
    PowerISA::PTE *retval = NULL;
    Tick min_tick = curTick();
    int min_tick_idx = -1;
    int free_idx = -1;

    for (int index = 0; index < size; index++) {
        PowerISA::PTE *pte = &table[index];

        if (!pte->Valid()) {
            free_idx = index;
        } else {
            if (pte->age < min_tick) {
                min_tick = pte->age;
                min_tick_idx = index;
            }
        }
    }

    // fill free slot.
    if (free_idx != -1) {
        PowerISA::PTE *pte = &table[free_idx];
        pte->Mask = ~((1ULL << page_shift) - 1);
        pte->VPN = vfn & pte->Mask;
        pte->PFN = pfn & pte->Mask;
        pte->age = curTick();
        pte->prot = prot;
        pte->c = c;
        pte->V = 1;
        pte->thread_id = tid;

        DPRINTF(TLB, "insert free:%llx, tid %#x PTE->Mask = %llx PTE->VPN = %llx\n", vfn, (int)tid,
            pte->Mask, pte->VPN);

        return;
    }

    // replace oldest pte
    if (min_tick_idx != -1) {
        PowerISA::PTE *pte = &table[min_tick_idx];
        pte->Mask = ~((1ULL << page_shift) - 1);
        pte->VPN = vfn & pte->Mask;
        pte->PFN = pfn & pte->Mask;
        pte->age = curTick();
        pte->prot = prot;
        pte->c = c;
        pte->V = 1;
        pte->thread_id = tid;

        DPRINTF(TLB, "insert oldest:%llx, tid %#x PTE->Mask = %llx PTE->VPN = %llx\n", vfn, (int)tid,
            pte->Mask, pte->VPN);

    }
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "flushAll\n");
    memset(table, 0, sizeof(PowerISA::PTE[size]));
    nlu = 0;
}

void
TLB::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(size);
    SERIALIZE_SCALAR(nlu);

    for (int i = 0; i < size; i++) {
        ScopedCheckpointSection sec(cp, csprintf("PTE%d", i));
        table[i].serialize(cp);
    }
}

void
TLB::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(size);
    UNSERIALIZE_SCALAR(nlu);

    for (int i = 0; i < size; i++) {
        ScopedCheckpointSection sec(cp, csprintf("PTE%d", i));
        if (table[i].V || table[i].V) {
        }
    }
}

Fault
TLB::translateInst(const RequestPtr &req, ThreadContext *tc)
{
    Addr vaddr = req->getVaddr();

    // Instruction accesses must be word-aligned
    if (vaddr & 0x3) {
        DPRINTF(TLB, "Alignment Fault on %#x, size = %d\n", vaddr,
                req->getSize());
        return std::make_shared<AlignmentFault>(vaddr);
    }

    return tc->getProcessPtr()->pTable->translate(req);
}

Fault
TLB::translateData(const RequestPtr &req, ThreadContext *tc, bool write)
{
    return tc->getProcessPtr()->pTable->translate(req);
}

Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Mode mode)
{
    Addr paddr;
    Addr vaddr = req->getVaddr();
    //DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
    //vaddr &= 0x0fffffffffffffff;
    if (FullSystem) {
        Msr msr = tc->readIntReg(INTREG_MSR);
        if (mode == BaseMMU::Execute) {
            if (msr.ir) {
                PowerISA::PTE *tlb_entry = lookup(vaddr, tc->threadId());
                if (!tlb_entry ||
                    (tlb_entry && !(tlb_entry->prot & PAGE_EXEC))
                    ) {
                    //printf("MSR: %lx\n",(uint64_t)msr);
                    Fault fault = walker->start(tc, NULL, req, mode);
                    if (fault != NoFault) {
                        DPRINTF(TLB, "translation fault:%s.\n", fault->name());
                    } else {
                        fault = NoFault;
                        DPRINTF(TLB, "TID:%d PageTableWalker Translated %#x -> %#x.\n", tc->threadId(), req->getVaddr(), req->getPaddr());
                    }
                    return fault;
                } else {
                    paddr = tlb_entry->PFN + (req->getVaddr() & (~tlb_entry->Mask));
                    if (tlb_entry->c) {
                        req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
                    }
                    req->setPaddr(paddr);
                    DPRINTF(TLB, "TID:%d TLB Translated %#x -> %#x.\n", tc->threadId(), req->getVaddr(), req->getPaddr());
                }

                return NoFault;
            }
            else{
                //DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
                paddr = vaddr & 0x0fffffffffffffff;
                //DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
                req->setPaddr(paddr);

                return NoFault;
            }
        }
        else{
            if (msr.dr){
                PowerISA::PTE *tlb_entry = lookup(vaddr, tc->threadId());
                if (!tlb_entry ||
                    (tlb_entry && mode == BaseMMU::Read && !(tlb_entry->prot & PAGE_READ)) ||
                    (tlb_entry && mode == BaseMMU::Write && !(tlb_entry->prot & PAGE_WRITE))
                    ) {
                    //printf("MSR: %lx\n",(uint64_t)msr);
                    Fault fault = walker->start(tc, NULL, req, mode);
                    if (fault != NoFault) {
                        DPRINTF(TLB, "translation fault:%s.\n", fault->name());
                    } else {
                        fault = NoFault;
                        DPRINTF(TLB, "TID:%d PageTableWalker Translated %#x -> %#x.\n", tc->threadId(), req->getVaddr(), req->getPaddr());
                    }
                    return fault;
                } else {
                    paddr = tlb_entry->PFN + (req->getVaddr() & (~tlb_entry->Mask));
                    if (tlb_entry->c) {
                        req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
                    }
                    req->setPaddr(paddr);
                    DPRINTF(TLB, "TID:%d TLB Translated %#x -> %#x.\n", tc->threadId(), req->getVaddr(), req->getPaddr());
                }

                return NoFault;
            }
            else{
                //DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
                paddr = vaddr & 0x0fffffffffffffff;
                //DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
                req->setPaddr(paddr);
                return NoFault;
            }
        }
    } else {
        if (mode == BaseMMU::Execute)
            return translateInst(req, tc);
        else
            return translateData(req, tc, mode == BaseMMU::Write);
    }
}

Fault
TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc,
                         BaseMMU::Mode mode)
{
    panic_if(FullSystem,
            "translateFunctional not implemented for full system.");
    return tc->getProcessPtr()->pTable->translate(req);
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Translation *translation, BaseMMU::Mode mode)
{
    Addr paddr;
    Addr vaddr = req->getVaddr();
    //DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
    vaddr &= 0x0fffffffffffffff;
    if (FullSystem) {
        Msr msr = tc->readIntReg(INTREG_MSR);
        if (mode == BaseMMU::Execute) {
            if (msr.ir){
                //printf("MSR: %lx\n",(uint64_t)msr);
                Fault fault = walker->start(tc, translation, req, mode);
                //Fault fault = NoFault;
                if (fault != NoFault) {
                    DPRINTF(TLB, "translation fault:%s.\n", fault->name());
                    translation->finish(fault, req, tc, mode);
                } else {
                    fault = NoFault;
                    translation->markDelayed();
                }

                return;
            }
            else{
                //DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
                paddr = vaddr;
                //DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
                req->setFlags(Request::STRICT_ORDER);
                req->setPaddr(paddr);
                translation->finish(NoFault, req, tc, mode);

                //trySnoopKernConsole(paddr, tc);
                //trySnoopOpalConsole(paddr, tc);

                return;
            }
        }
        else{
            if (msr.dr){
                Fault fault = walker->start(tc, translation, req, mode);

                if (fault != NoFault) {
                    DPRINTF(TLB, "translation fault:%s.\n", fault->name());
                    translation->finish(fault, req, tc, mode);
                } else {
                    fault = NoFault;
                    translation->markDelayed();
                }
                return;
            }
            else{
                //DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
                paddr = vaddr;
                //DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
                req->setFlags(Request::STRICT_ORDER);
                req->setPaddr(paddr);
                translation->finish(NoFault, req, tc, mode);
                return;
            }
        }
    }
}

Fault
TLB::finalizePhysical(const RequestPtr &req,
                      ThreadContext *tc, BaseMMU::Mode mode) const
{
    return NoFault;
}

PowerISA::PTE &
TLB::index(bool advance)
{
    PowerISA::PTE *pte = &table[nlu];

    if (advance)
        nextnlu();

    return *pte;
}

Port *
TLB::getTableWalkerPort()
{
    return &walker->getPort("port");
}

} // namespace gem5
