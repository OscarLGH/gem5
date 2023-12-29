/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "dev/power/icp/Icp.hh"

#include "base/bitfield.hh"
#include "base/trace.hh"
#include "debug/Icp.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

namespace gem5
{

PowerISA::Icp::Icp(const Params &p)
    : BasicPioDevice(p, 0x1000),
      latency(p.pio_latency)
{

}

void
PowerISA::Icp::init()
{
    BasicPioDevice::init();

}

Tick
PowerISA::Icp::read(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;
    int thread = (daddr >> 12) & 0x7;
    int core = (daddr >> 15) & 0xf;
    int offset = daddr & 0xfff;

    DPRINTF(Icp, "Read register %#x\n", daddr);
    //registers.read(daddr, pkt->getPtr<void>(), pkt->getSize());

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
PowerISA::Icp::write(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;
    int thread = (daddr >> 12) & 0x7;
    int core = (daddr >> 15) & 0xf;
    int offset = daddr & 0xfff;

    DPRINTF(Icp, "Write register %#x value %#x\n", daddr,
            pkt->getRaw<uint8_t>());

    //registers.write(daddr, pkt->getPtr<void>(), pkt->getSize());

    pkt->makeAtomicResponse();
    return pioDelay;
}


void
PowerISA::Icp::requestInterrupt(int line)
{
}

void
PowerISA::Icp::serialize(CheckpointOut &cp) const
{

}

void
PowerISA::Icp::unserialize(CheckpointIn &cp)
{

}

} // namespace gem5
