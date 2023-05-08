/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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

#ifndef __ARCH_POWER_TABLE_WALKER_HH__
#define __ARCH_POWER_TABLE_WALKER_HH__

#include <vector>

#include "arch/generic/mmu.hh"
#include "arch/power/pagetable.hh"
#include "arch/power/tlb.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "params/PowerPagetableWalker.hh"
#include "sim/clocked_object.hh"
#include "sim/faults.hh"
#include "sim/system.hh"

namespace gem5
{

class ThreadContext;

namespace PowerISA
{
    class Walker : public ClockedObject
    {
      protected:
        // Port for accessing memory
        class WalkerPort : public RequestPort
        {
          public:
            WalkerPort(const std::string &_name, Walker * _walker) :
                  RequestPort(_name, _walker), walker(_walker)
            {}

          protected:
            Walker *walker;

            bool recvTimingResp(PacketPtr pkt);
            void recvReqRetry();
        };

        friend class WalkerPort;
        WalkerPort port;

        // State to track each walk of the page table
        class WalkerState
        {
          friend class Walker;
          private:
            enum State
            {
                Ready,
                Waiting,
                Translate,
            };

          protected:
            Walker *walker;
            ThreadContext *tc;
            RequestPtr req;
            State state;
            State nextState;
            int level;
            unsigned inflight;
            TlbEntry entry;
            PacketPtr read;
            std::vector<PacketPtr> writes;
            Fault timingFault;
            BaseMMU::Translation * translation;
            BaseMMU::Mode mode;
            bool functional;
            bool timing;
            bool retrying;
            bool started;
            bool squashed;
          public:
            WalkerState(Walker * _walker, BaseMMU::Translation *_translation,
                        const RequestPtr &_req, bool _isFunctional = false) :
                walker(_walker), req(_req), state(Ready),
                nextState(Ready), level(0), inflight(0),
                translation(_translation),
                functional(_isFunctional), timing(false),
                retrying(false), started(false), squashed(false)
            {
            }
            void initState(ThreadContext * _tc, BaseMMU::Mode _mode,
                           bool _isTiming = false);
            Fault startWalk();
            Fault startFunctional(Addr &addr, unsigned &logBytes);
            bool recvPacket(PacketPtr pkt);
            unsigned numInflight() const;
            bool isRetrying();
            bool wasStarted();
            bool isTiming();
            void retry();
            void squash();
            std::string name() const {return walker->name();}

          private:
            void setupWalk(Addr vaddr);
            Fault stepWalk(PacketPtr &write);
            void sendPackets();
            void endWalk();
            Fault pageFault(bool present);
        };

        friend class WalkerState;
        // State for timing and atomic accesses (need multiple per walker in
        // the case of multiple outstanding requests in timing mode)
        std::list<WalkerState *> currStates;
        // State for functional accesses (only need one of these per walker)
        WalkerState funcState;

        struct WalkerSenderState : public Packet::SenderState
        {
            WalkerState * senderWalk;
            WalkerSenderState(WalkerState * _senderWalk) :
                senderWalk(_senderWalk) {}
        };

      public:
        // Kick off the state machine.
        Fault start(ThreadContext * _tc, BaseMMU::Translation *translation,
                const RequestPtr &req, BaseMMU::Mode mode);
        Fault startFunctional(ThreadContext * _tc, Addr &addr,
                unsigned &logBytes, BaseMMU::Mode mode);
        Port &getPort(const std::string &if_name,
                      PortID idx=InvalidPortID) override;

      protected:
        // The TLB we're supposed to load.
        PowerISA::TLB * tlb;
        System * sys;
        RequestorID requestorId;

        // The number of outstanding walks that can be squashed per cycle.
        unsigned numSquashable;

        // Wrapper for checking for squashes before starting a translation.
        void startWalkWrapper();

        /**
         * Event used to call startWalkWrapper.
         **/
        EventFunctionWrapper startWalkWrapperEvent;

        // Functions for dealing with packets.
        bool recvTimingResp(PacketPtr pkt);
        void recvReqRetry();
        bool sendTiming(WalkerState * sendingState, PacketPtr pkt);

      public:

        void setTLB(PowerISA::TLB * _tlb)
        {
            tlb = _tlb;
        }

        using Params = PowerPagetableWalkerParams;

        Walker(const Params &params) :
            ClockedObject(params), port(name() + ".port", this),
            funcState(this, NULL, NULL, true), tlb(NULL), sys(params.system),
            requestorId(sys->getRequestorId(this)),
            numSquashable(params.num_squash_per_cycle),
            startWalkWrapperEvent([this]{ startWalkWrapper(); }, name())
        {
        }

      public:
        uint64_t readPhysMem(uint64_t addr, uint64_t dataSize);
        uint64_t writePhysMem(uint64_t addr, uint64_t dataSize);

        Addr getRPDEntry(ThreadContext * tc, Addr vaddr);
        Fault prepareSI(ThreadContext * tc,
              RequestPtr req, BaseMMU::Mode mode, uint64_t BitMask);

        Fault prepareISI(ThreadContext * tc,
              RequestPtr req, uint64_t BitMask);

        Fault prepareDSI(ThreadContext * tc, RequestPtr req,
                         BaseMMU::Mode mode,uint64_t BitMask);
        BitUnion64(Rpde)
                Bitfield<63> valid;
                Bitfield<62> leaf;
                Bitfield<59, 8>  NLB;
                Bitfield<4, 0> NLS;
        EndBitUnion(Rpde)

        BitUnion64(Rpte)
                Bitfield<63> valid;
                Bitfield<62> leaf;
                Bitfield<61> sw1;
                Bitfield<56,12> rpn;
                Bitfield<11,9> sw2;
                Bitfield<8> ref;
                Bitfield<7> c;
                Bitfield<5,4> att;
                Bitfield<3> pri;
                Bitfield<2> read;
                Bitfield<1> r_w;
                Bitfield<0> exe;
        EndBitUnion(Rpte)

        std::pair<Addr,Fault> walkTree(Addr vaddr ,uint64_t curBase ,
            ThreadContext * tc ,BaseMMU::Mode mode , RequestPtr req,
                         uint64_t curSize ,uint64_t usefulBits);
    };

#define PRTB_SHIFT     12
#define PRTB_MASK      0x0ffffffffffff
#define PRTB_ALIGN     4
#define TABLE_BASE_ALIGN     PRTB_SHIFT
#define DSISR_MASK    0x00000000ffffffff

#define RPDB_SHIFT     8
#define RPDB_MASK      0x0fffffffffffff

#define RPDS_SHIFT     0
#define RPDS_MASK      0x1f

#define NLB_SHIFT      RPDB_SHIFT
#define NLB_MASK       RPDB_MASK

#define NLS_SHIFT      RPDS_SHIFT
#define NLS_MASK       RPDS_MASK

#define DIR_BASE_ALIGN  RPDB_SHIFT

#define RTS1_SHIFT     61
#define RTS1_MASK      0x3
#define RTS2_BITS      3
#define RTS2_SHIFT     5
#define RTS2_MASK      ((1 << RTS2_BITS) - 1)

#define   NOHPTE      0x0000000040000000
            /*Bit-33 Acc to ISA: no translation found */
#define  PROTFAULT   0x0000000008000000
           /* Bit-36 Acc to ISA:protection fault */
#define  ISSTORE     0x0000000002000000
           /* Bit-38 Acc to ISA:access was a store */
#define  UNSUPP_MMU  0x0000000000080000
           /*Bit-44 P9: Unsupported MMU config */
#define  PRTABLE_FAULT 0x0000000000020000
          /*Bit-46  P9: Fault on process table */

#define RPN_MASK      0x01fffffffffff000

#define QUADRANT_MASK 0xc000000000000000
#define QUADRANT00   0x0000000000000000
#define QUADRANT01   0x4000000000000000
#define QUADRANT10   0x8000000000000000
#define QUADRANT11   0xc000000000000000

#define extract(x, shift, mask)   ((x >> shift) & mask)
#define align(x, bits) (x << bits)
#define setBitMask(shift) ( (uint64_t)1 << shift)
#define unsetMask(start ,end)(~((setBitMask(start))-1) | ((setBitMask(end))-1))

#define getRTS(x)      ((extract(x, RTS1_SHIFT, RTS1_MASK) << RTS2_BITS) | \
                        (extract(x, RTS2_SHIFT, RTS2_MASK)))

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_RISCV_PAGE_TABLE_WALKER_HH__
