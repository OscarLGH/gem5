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
        uint64_t writePhysMem(uint64_t addr, uint64_t data, uint64_t dataSize);

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

        std::pair<Addr,Fault> walkRadixTree(Addr vaddr ,uint64_t curBase ,
            ThreadContext * tc ,BaseMMU::Mode mode , RequestPtr req,
                         uint64_t curSize ,uint64_t usefulBits);

        public:
        /*
        * SLB definitions
        */

        /* Bits in the SLB ESID word */
        #define SLB_ESID_ESID           0xFFFFFFFFF0000000ULL
        #define SLB_ESID_V              0x0000000008000000ULL /* valid */

        /* Bits in the SLB VSID word */
        #define SLB_VSID_SHIFT          12
        #define SLB_VSID_SHIFT_1T       24
        #define SLB_VSID_SSIZE_SHIFT    62
        #define SLB_VSID_B              0xc000000000000000ULL
        #define SLB_VSID_B_256M         0x0000000000000000ULL
        #define SLB_VSID_B_1T           0x4000000000000000ULL
        #define SLB_VSID_VSID           0x3FFFFFFFFFFFF000ULL
        #define SLB_VSID_VRMA           (0x0001FFFFFF000000ULL | SLB_VSID_B_1T)
        #define SLB_VSID_PTEM           (SLB_VSID_B | SLB_VSID_VSID)
        #define SLB_VSID_KS             0x0000000000000800ULL
        #define SLB_VSID_KP             0x0000000000000400ULL
        #define SLB_VSID_N              0x0000000000000200ULL /* no-execute */
        #define SLB_VSID_L              0x0000000000000100ULL
        #define SLB_VSID_L_SHIFT        PPC_BIT_NR(55)
        #define SLB_VSID_C              0x0000000000000080ULL /* class */
        #define SLB_VSID_LP             0x0000000000000030ULL
        #define SLB_VSID_LP_SHIFT       PPC_BIT_NR(59)
        #define SLB_VSID_ATTR           0x0000000000000FFFULL
        #define SLB_VSID_LLP_MASK       (SLB_VSID_L | SLB_VSID_LP)
        #define SLB_VSID_4K             0x0000000000000000ULL
        #define SLB_VSID_64K            0x0000000000000110ULL
        #define SLB_VSID_16M            0x0000000000000100ULL
        #define SLB_VSID_16G            0x0000000000000120ULL

        /*
        * Hash page table definitions
        */

        #define SDR_64_HTABORG         0x0FFFFFFFFFFC0000ULL
        #define SDR_64_HTABSIZE        0x000000000000001FULL

        #define PATE0_HTABORG           0x0FFFFFFFFFFC0000ULL
        #define PATE0_PS                PPC_BITMASK(56, 58)
        #define PATE0_GET_PS(dw0)       (((dw0) & PATE0_PS) >> PPC_BIT_NR(58))

        #define HPTES_PER_GROUP         8
        #define HASH_PTE_SIZE_64        16
        #define HASH_PTEG_SIZE_64       (HASH_PTE_SIZE_64 * HPTES_PER_GROUP)

        #define HPTE64_V_SSIZE          SLB_VSID_B
        #define HPTE64_V_SSIZE_256M     SLB_VSID_B_256M
        #define HPTE64_V_SSIZE_1T       SLB_VSID_B_1T
        #define HPTE64_V_SSIZE_SHIFT    62
        #define HPTE64_V_AVPN_SHIFT     7
        #define HPTE64_V_AVPN           0x3fffffffffffff80ULL
        #define HPTE64_V_AVPN_VAL(x)    (((x) & HPTE64_V_AVPN) >> HPTE64_V_AVPN_SHIFT)
        #define HPTE64_V_COMPARE(x, y)  (!(((x) ^ (y)) & 0xffffffffffffff83ULL))
        #define HPTE64_V_BOLTED         0x0000000000000010ULL
        #define HPTE64_V_LARGE          0x0000000000000004ULL
        #define HPTE64_V_SECONDARY      0x0000000000000002ULL
        #define HPTE64_V_VALID          0x0000000000000001ULL

        #define HPTE64_R_PP0            0x8000000000000000ULL
        #define HPTE64_R_TS             0x4000000000000000ULL
        #define HPTE64_R_KEY_HI         0x3000000000000000ULL
        #define HPTE64_R_RPN_SHIFT      12
        #define HPTE64_R_RPN            0x0ffffffffffff000ULL
        #define HPTE64_R_FLAGS          0x00000000000003ffULL
        #define HPTE64_R_PP             0x0000000000000003ULL
        #define HPTE64_R_N              0x0000000000000004ULL
        #define HPTE64_R_G              0x0000000000000008ULL
        #define HPTE64_R_M              0x0000000000000010ULL
        #define HPTE64_R_I              0x0000000000000020ULL
        #define HPTE64_R_W              0x0000000000000040ULL
        #define HPTE64_R_WIMG           0x0000000000000078ULL
        #define HPTE64_R_C              0x0000000000000080ULL
        #define HPTE64_R_R              0x0000000000000100ULL
        #define HPTE64_R_KEY_LO         0x0000000000000e00ULL
        #define HPTE64_R_KEY(x)         ((((x) & HPTE64_R_KEY_HI) >> 57) | \
                                        (((x) & HPTE64_R_KEY_LO) >> 9))

        #define HPTE64_V_1TB_SEG        0x4000000000000000ULL
        #define HPTE64_V_VRMA_MASK      0x4001ffffff000000ULL

        /* PTE offsets */
        #define HPTE64_DW1              (HASH_PTE_SIZE_64 / 2)
        #define HPTE64_DW1_R            (HPTE64_DW1 + 6)
        #define HPTE64_DW1_C            (HPTE64_DW1 + 7)

        /* Format changes for ARCH v3 */
        #define HPTE64_V_COMMON_BITS    0x000fffffffffffffULL
        #define HPTE64_R_3_0_SSIZE_SHIFT 58
        #define HPTE64_R_3_0_SSIZE_MASK (3ULL << HPTE64_R_3_0_SSIZE_SHIFT)

        #define PPC_PAGE_SIZES_MAX_SZ   8



        static inline bool mmuidx_pr(int idx) { return !(idx & 1); }
        static inline bool mmuidx_real(int idx) { return idx & 2; }
        static inline bool mmuidx_hv(int idx) { return idx & 4; }

        #define MAX_SLB_ENTRIES         64
        #define SEGMENT_SHIFT_256M      28
        #define SEGMENT_MASK_256M       (~((1ULL << SEGMENT_SHIFT_256M) - 1))

        #define SEGMENT_SHIFT_1T        40
        #define SEGMENT_MASK_1T         (~((1ULL << SEGMENT_SHIFT_1T) - 1))

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

        /* DSISR */
        #define DSISR_NOPTE              0x40000000
        /* Not permitted by access authority of encoded access authority */
        #define DSISR_PROTFAULT          0x08000000
        #define DSISR_ISSTORE            0x02000000
        /* Not permitted by virtual page class key protection */
        #define DSISR_AMR                0x00200000
        /* Unsupported Radix Tree Configuration */
        #define DSISR_R_BADCONFIG        0x00080000
        #define DSISR_ATOMIC_RC          0x00040000
        /* Unable to translate address of (guest) pde or process/page table entry */
        #define DSISR_PRTABLE_FAULT      0x00020000

        /* SRR1 error code fields */

        #define SRR1_NOPTE               DSISR_NOPTE
        /* Not permitted due to no-execute or guard bit set */
        #define SRR1_NOEXEC_GUARD        0x10000000
        #define SRR1_PROTFAULT           DSISR_PROTFAULT
        #define SRR1_IAMR                DSISR_AMR

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
        struct PPCHash64PageSize
        {
            uint32_t page_shift;  /* Page shift (or 0) */
            uint32_t pte_enc;     /* Encoding in the HPTE (>>12) */
        };
        typedef struct PPCHash64PageSize PPCHash64PageSize;

        struct PPCHash64SegmentPageSizes
        {
            uint32_t page_shift;  /* Base page shift of segment (or 0) */
            uint32_t slb_enc;     /* SLB encoding for BookS */
            PPCHash64PageSize enc[PPC_PAGE_SIZES_MAX_SZ];
        };

        struct PPCHash64Options
        {
        #define PPC_HASH64_1TSEG        0x00001
        #define PPC_HASH64_AMR          0x00002
        #define PPC_HASH64_CI_LARGEPAGE 0x00004
            unsigned flags;
            unsigned slb_size;
            PPCHash64SegmentPageSizes sps[PPC_PAGE_SIZES_MAX_SZ];
        };

        typedef struct PPCHash64Options PPCHash64Options;

        /* same as PROT_xxx */
        #define PAGE_READ      0x0001
        #define PAGE_WRITE     0x0002
        #define PAGE_EXEC      0x0004
        #define PAGE_BITS      (PAGE_READ | PAGE_WRITE | PAGE_EXEC)
        #define PAGE_VALID     0x0008

        struct ppc_slb_t
        {
            uint64_t esid;
            uint64_t vsid;
            const PPCHash64SegmentPageSizes *sps;
        };
        typedef struct ppc_slb_t ppc_slb_t;

        struct ppc_hash_pte64
        {
            uint64_t pte0, pte1;
        };

        typedef struct ppc_spr_t ppc_spr_t;
        typedef union ppc_tlb_t ppc_tlb_t;
        typedef struct ppc_hash_pte64 ppc_hash_pte64_t;
        std::pair<Addr,Fault> walkHashTable(Addr vaddr ,
            ThreadContext * tc ,BaseMMU::Mode mode , RequestPtr req);

        public:

          const PPCHash64Options ppc_hash64_opts_POWER7 = {
              .flags = PPC_HASH64_1TSEG | PPC_HASH64_AMR | PPC_HASH64_CI_LARGEPAGE,
              .slb_size = 32,
              .sps = {
                  {
                      .page_shift = 12, /* 4K */
                      .slb_enc = 0,
                      .enc = { { .page_shift = 12, .pte_enc = 0 },
                              { .page_shift = 16, .pte_enc = 0x7 },
                              { .page_shift = 24, .pte_enc = 0x38 }, },
                  },
                  {
                      .page_shift = 16, /* 64K */
                      .slb_enc = SLB_VSID_64K,
                      .enc = { { .page_shift = 16, .pte_enc = 0x1 },
                              { .page_shift = 24, .pte_enc = 0x8 }, },
                  },
                  {
                      .page_shift = 24, /* 16M */
                      .slb_enc = SLB_VSID_16M,
                      .enc = { { .page_shift = 24, .pte_enc = 0 }, },
                  },
                  {
                      .page_shift = 34, /* 16G */
                      .slb_enc = SLB_VSID_16G,
                      .enc = { { .page_shift = 34, .pte_enc = 0x3 }, },
                  },
              }
          };
          ppc_slb_t slb_table[64];
          PPCHash64PageSize sp_table[8];

          ppc_slb_t * slb_lookup(ThreadContext * tc, Addr eaddr);
          ppc_hash_pte64_t * ppc_hash64_map_hptes(ThreadContext * tc,
                                             Addr ptex, int n);
          unsigned hpte_page_shift(const PPCHash64SegmentPageSizes *sps,
                                uint64_t pte0, uint64_t pte1);
          Addr ppc_hash64_pteg_search(ThreadContext * tc, Addr hash,
                                     const PPCHash64SegmentPageSizes *sps,
                                     Addr ptem,
                                     ppc_hash_pte64_t *pte, unsigned *pshift);
          Addr ppc_hash64_htab_lookup(ThreadContext * tc,
                                     ppc_slb_t *slb, Addr eaddr,
                                     ppc_hash_pte64_t *pte, unsigned *pshift);

          int ppc_hash64_pte_noexec_guard(ppc_hash_pte64_t pte);
          int ppc_hash64_pte_prot(int mmu_idx,
                               ppc_slb_t *slb, ppc_hash_pte64_t pte);
          int ppc_hash64_iamr_prot(ThreadContext * tc, int key);
          int ppc_hash64_amr_prot(ThreadContext * tc, ppc_hash_pte64_t pte);
          void ppc_hash64_set_r(ThreadContext * tc, Addr ptex, uint64_t pte1);
          void ppc_hash64_set_c(ThreadContext * tc, Addr ptex, uint64_t pte1);
          int ppc_store_slb(int slot, Addr esid, Addr vsid);
          void slbie_helper(ThreadContext * tc, Addr eaddr);
          Fault prepareSegInt(ThreadContext * tc, RequestPtr req,
                    BaseMMU::Mode mode);
    };
} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_RISCV_PAGE_TABLE_WALKER_HH__
