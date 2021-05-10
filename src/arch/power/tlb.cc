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
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "arch/power/faults.hh"
#include "arch/power/miscregs.hh"
#include "arch/power/pagetable.hh"
#include "arch/power/radixwalk.hh"
#include "arch/power/registers.hh"
#include "arch/power/utility.hh"
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

using namespace std;
using namespace PowerISA;

long stdout_buf_length=0;
long stdout_buf_addr=0;

namespace PowerISA {

SystemCallInterrupt::SystemCallInterrupt(){
}
void
SystemCallInterrupt::invoke(ThreadContext * tc, const StaticInstPtr &inst =
                       StaticInst::nullStaticInstPtr){

      tc->setIntReg(INTREG_SRR0 , tc->instAddr() + 4);
      PowerInterrupt::updateSRR1(tc);
      PowerInterrupt::updateMsr(tc);
      tc->pcState(SystemCallPCSet);
      //std::printf("System call number = %lu\n", tc->readIntReg(0));
      if (tc->readIntReg(0) == 4){
        stdout_buf_length = (int)tc->readIntReg(5);
        stdout_buf_addr = tc->readIntReg(4);
     }
}
///////////////////////////////////////////////////////////////////////
//
//  POWER TLB
//

#define MODE2MASK(X) (1 << (X))

void
TLB::initConsoleSnoop(void)
{
    int nameStart, addrLen;
    string symName, symHexAddr;
    ifstream in;
    string line;

    /* Find console snoop point for kernel */
    in.open("dist/m5/system/binaries/objdump_vmlinux");
    if (!in.is_open()) {
        panic("Could not find kernel objdump");
    }

    while (getline(in, line)) {
        /* Find ".log_store" and the first call to ".memcpy" inside it */
        nameStart = line.find("<.log_store>:");

        /* Sometimes, optimizations introduce ISRA symbols */
        if (nameStart == string::npos) {
            nameStart = line.find("<.log_store.isra.1>:");
        }

        if (nameStart != string::npos) {
            while (getline(in, line)) {
                if (line.find("<.memcpy>") != string::npos &&
                    (*(line.rbegin())) != ':') {
                    addrLen = line.find(":");
                    istringstream hexconv(line.substr(0, addrLen));
                    hexconv >> hex >> kernConsoleSnoopAddr;

                    /* Use previous instruction and remove quadrant bits */
                    kernConsoleSnoopAddr -= 4;
                    kernConsoleSnoopAddr &= (-1ULL >> 2);
                    break;
                }
            }
        }
    }

    in.close();

    if (!kernConsoleSnoopAddr) {
        panic("Could not determine kernel console snooping address");
    }

    /* Find console snoop point for skiboot */
    in.open("dist/m5/system/binaries/objdump_skiboot");
    if (!in.is_open()) {
        panic("Could not find skiboot objdump");
    }

    while (getline(in, line)) {
        /* Find ".console_write" and the first call to ".write" inside it */
        nameStart = line.find("<.console_write>:");

        if (nameStart != string::npos) {
            addrLen = line.find(":");
            istringstream hexconv(line.substr(0, addrLen));
            hexconv >> hex >> opalConsoleSnoopAddr;

            /* Add OPAL load offset */
            opalConsoleSnoopAddr += 0x30000000ULL;
            break;
        }
    }

    inform("Snooping kernel console at 0x%016lx", kernConsoleSnoopAddr);
    inform("Snooping skiboot console at 0x%016lx", opalConsoleSnoopAddr);

    in.close();
    if (!opalConsoleSnoopAddr) {
        panic("Could not determine skiboot console snooping address");
    }
}

void
TLB::trySnoopKernConsole(uint64_t paddr, ThreadContext *tc)
{
    uint64_t addr;
    int len, i;
    char *buf;

    if (paddr != kernConsoleSnoopAddr) {
        return;
    }

    len = (int) tc->readIntReg(5);
    buf = new char[len + 1];
    addr = (uint64_t) tc->readIntReg(4) & (-1ULL >> 4);

    for (i = 0; i < len; i++) {
        buf[i] = (char) rwalk->readPhysMem(addr + i, 8);
    }

    buf[i] = '\0';
    printf("%lu [KERN LOG] %s\n", curTick(), buf);
    delete buf;
}

void
TLB::trySnoopOpalConsole(uint64_t paddr, ThreadContext *tc)
{
    uint64_t addr;
    int len, i;
    char *buf;

    if (paddr != opalConsoleSnoopAddr) {
        return;
    }

    len = (int) tc->readIntReg(5);
    buf = new char[len + 1];
    addr = (uint64_t) tc->readIntReg(4) & (-1ULL >> 4);

    for (i = 0; i < len; i++) {
        buf[i] = (char) rwalk->readPhysMem(addr + i, 8);
    }

    buf[i] = '\0';
    printf("%lu [OPAL LOG] %s\n", curTick(), buf);
    delete buf;
}

TLB::TLB(const Params &p)
    : BaseTLB(p), size(p.size), nlu(0)
{
    table = new PowerISA::PTE[size];
    memset(table, 0, sizeof(PowerISA::PTE[size]));
    smallPages = 0;
    rwalk = p.walker;
    //initConsoleSnoop();
}

TLB::~TLB()
{
    if (table)
        delete [] table;
}

// look up an entry in the TLB
PowerISA::PTE *
TLB::lookup(Addr vpn, uint8_t asn) const
{
    // assume not found...
    PowerISA::PTE *retval = NULL;
    PageTable::const_iterator i = lookupTable.find(vpn);
    if (i != lookupTable.end()) {
        while (i->first == vpn) {
            int index = i->second;
            PowerISA::PTE *pte = &table[index];
            Addr Mask = pte->Mask;
            Addr InvMask = ~Mask;
            Addr VPN  = pte->VPN;
            if (((vpn & InvMask) == (VPN & InvMask))
               && (pte->G  || (asn == pte->asid))) {

                // We have a VPN + ASID Match
                retval = pte;
                break;
            }
            ++i;
        }
    }

    DPRINTF(TLB, "lookup %#x, asn %#x -> %s ppn %#x\n", vpn, (int)asn,
            retval ? "hit" : "miss", retval ? retval->PFN1 : 0);
    return retval;
}

PowerISA::PTE*
TLB::getEntry(unsigned Index) const
{
    // Make sure that Index is valid
    assert(Index<size);
    return &table[Index];
}

int
TLB::probeEntry(Addr vpn,uint8_t asn) const
{
    // assume not found...
    int Ind = -1;
    PageTable::const_iterator i = lookupTable.find(vpn);
    if (i != lookupTable.end()) {
        while (i->first == vpn) {
            int index = i->second;
            PowerISA::PTE *pte = &table[index];
            Addr Mask = pte->Mask;
            Addr InvMask = ~Mask;
            Addr VPN  = pte->VPN;
            if (((vpn & InvMask) == (VPN & InvMask))
                && (pte->G  || (asn == pte->asid))) {

                // We have a VPN + ASID Match
                Ind = index;
                break;
            }
            ++i;
        }
    }

    DPRINTF(Power, "VPN: %x, asid: %d, Result of TLBP: %d\n", vpn, asn, Ind);
    return Ind;
}

inline Fault
TLB::checkCacheability(const RequestPtr &req)
{
    Addr VAddrUncacheable = 0xA0000000;
    if ((req->getVaddr() & VAddrUncacheable) == VAddrUncacheable) {

        // mark request as uncacheable
        req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
    }
    return NoFault;
}

void
TLB::insertAt(PowerISA::PTE &pte, unsigned Index, int _smallPages)
{
    smallPages=_smallPages;
    if (Index > size){
        warn("Attempted to write at index (%d) beyond TLB size (%d)",
             Index, size);
    } else {

        // Update TLB
        if (table[Index].V0 || table[Index].V1) {

            // Previous entry is valid
            PageTable::iterator i = lookupTable.find(table[Index].VPN);
            lookupTable.erase(i);
        }
        table[Index]=pte;

        // Update fast lookup table
        lookupTable.insert(make_pair(table[Index].VPN, Index));
    }
}

// insert a new TLB entry
void
TLB::insert(Addr addr, PowerISA::PTE &pte)
{
    fatal("TLB Insert not yet implemented\n");
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "flushAll\n");
    memset(table, 0, sizeof(PowerISA::PTE[size]));
    lookupTable.clear();
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
        if (table[i].V0 || table[i].V1) {
            lookupTable.insert(make_pair(table[i].VPN, i));
        }
    }
}

RadixWalk *
TLB::getWalker()
{
    return rwalk;
}

void
TLB::regStats()
{
    BaseTLB::regStats();

    read_hits
        .name(name() + ".read_hits")
        .desc("DTB read hits")
        ;

    read_misses
        .name(name() + ".read_misses")
        .desc("DTB read misses")
        ;


    read_accesses
        .name(name() + ".read_accesses")
        .desc("DTB read accesses")
        ;

    write_hits
        .name(name() + ".write_hits")
        .desc("DTB write hits")
        ;

    write_misses
        .name(name() + ".write_misses")
        .desc("DTB write misses")
        ;


    write_accesses
        .name(name() + ".write_accesses")
        .desc("DTB write accesses")
        ;

    hits
        .name(name() + ".hits")
        .desc("DTB hits")
        ;

    misses
        .name(name() + ".misses")
        .desc("DTB misses")
        ;

    accesses
        .name(name() + ".accesses")
        .desc("DTB accesses")
        ;

    hits = read_hits + write_hits;
    misses = read_misses + write_misses;
    accesses = read_accesses + write_accesses;
}

Fault
TLB::translateInst(const RequestPtr &req, ThreadContext *tc)
{
    // Instruction accesses must be word-aligned
    if (req->getVaddr() & 0x3) {
        DPRINTF(TLB, "Alignment Fault on %#x, size = %d\n", req->getVaddr(),
                req->getSize());
        return std::make_shared<AlignmentFault>();
    }

    return tc->getProcessPtr()->pTable->translate(req);
}

Fault
TLB::translateData(const RequestPtr &req, ThreadContext *tc, bool write)
{
    return tc->getProcessPtr()->pTable->translate(req);
}

Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode)
{
    Addr paddr;
    Addr vaddr = req->getVaddr();
    DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
    vaddr &= 0x0fffffffffffffff;
    if (FullSystem){
        /*
        if (stdout_buf_length){
            RequestPtr ptr = new Request();
            Msr msr = tc->readIntReg(INTREG_MSR);
            msr.dr = 1;
            tc->setIntReg(INTREG_MSR,msr);
            ptr->setVirt(req->_asid,stdout_buf_addr,8,
                         req->_flags,req->_masterId,req->_pc);
            rwalk->start(tc,ptr,BaseTLB::Read);
            char stdout_buf[stdout_buf_length + 1];
            Addr stdout_paddr = ptr->getPaddr();
            int i = 0;
            char read;
            for (i=0; i<stdout_buf_length; i++){
                read =  (char)rwalk->readPhysMem(stdout_paddr + i, 8);
               stdout_buf[i] = read;
            }
            stdout_buf[i] = '\0';
            //DPRINTF(TLB, "[STDOUT LOG] %s",stdout_buf);
            std::printf("%lu [STDOUT] %s",curTick(),stdout_buf);
            std::fflush(stdout);
            stdout_buf_length = 0;
        }
        */
        Msr msr = tc->readIntReg(INTREG_MSR);
        if (mode == Execute){
                //0x20000000
           /* if (vaddr == 0x30005214){
                int fd = open("device_tree.dtb", O_CREAT | O_WRONLY,0644);
                //uint64_t i;
                int index = 0;
                //uint64_t start_addr = (uint64_t)tc->readIntReg(3);
                std::printf("r3(device tree start) 0x%016lx\n",
                                tc->readIntReg(4));
                uint64_t start_addr = (uint64_t)tc->readIntReg(4); //- 0x1000;
                uint8_t buf[10745];
                for (index=0; index<10745; index++){
                    buf[index] = (uint8_t)rwalk->readPhysMem(
                                        start_addr + index,8);
                    if (index < 8){
                            std::printf("buf[0x%016lx] = %02x\n",
                            start_addr + index,(unsigned int)buf[index]);
                    }
                }
                int len = write(fd,buf,10745);
                std::printf("Written to the device_tree.dtb :: len %d\n",len);
            }*/
            if (msr.ir){
                //printf("MSR: %lx\n",(uint64_t)msr);
                Fault fault = rwalk->start(tc,req, mode);
                paddr = req->getPaddr();

                trySnoopKernConsole(paddr, tc);
                trySnoopOpalConsole(paddr, tc);

                return fault;
            }
            else{
                DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
                paddr = vaddr;
                DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
                req->setPaddr(paddr);

                trySnoopKernConsole(paddr, tc);
                trySnoopOpalConsole(paddr, tc);

                return NoFault;

            }
        }
        else{
            if (msr.dr){
                Fault fault = rwalk->start(tc,req, mode);
                paddr = req->getPaddr();
                return fault;
            }
            else{
                DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);
                paddr = vaddr;
                DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
                req->setPaddr(paddr);
                return NoFault;
            }
        }
    }
    if (mode == Execute)
         return translateInst(req, tc);
    else{
        return translateData(req, tc, mode == Write);
   }
}

Fault
TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc, Mode mode)
{
    panic_if(FullSystem,
            "translateFunctional not implemented for full system.");
    return tc->getProcessPtr()->pTable->translate(req);
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
                     Translation *translation, Mode mode)
{
    assert(translation);
    translation->finish(translateAtomic(req, tc, mode), req, tc, mode);
}

Fault
TLB::finalizePhysical(const RequestPtr &req,
                      ThreadContext *tc, Mode mode) const
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

}

Port *
TLB::getTableWalkerPort()
{
    return &rwalk->getRequestPort("port");
}
