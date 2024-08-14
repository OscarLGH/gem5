#include "dev/riscv/noc_device.hh"

#include "debug/NocDevice.hh"
#include "mem/packet_access.hh"
#include "params/NocDevice.hh"

#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>

namespace gem5
{

namespace RiscvISA
{

static int dev_cnt = 0;

NocDevice::NocDevice(const NocDeviceParams &params)
    : PlicDmaDevice(params),interrupt_id(params.interrupt_id),
    pollingEvent([this]{ polling_event_callback(); }, "polling event")
{
    node_index = dev_cnt++;
    this->schedule(pollingEvent, nextCycle());
}

NocDevice::~NocDevice()
{
}

Tick
NocDevice::read(PacketPtr pkt)
{
    const Addr offset = pkt->getAddr() - pio_addr;
    const unsigned size(pkt->getSize());

    DPRINTF(NocDevice, "Reading %u bytes @ 0x%x:\n", size, offset);

    const uint64_t value = read(offset);
    DPRINTF(NocDevice, "    value: 0x%llx\n", value);
    pkt->makeResponse();
    size == 8 ? pkt->setLE<uint64_t>(value) : pkt->setLE<uint32_t>(value);

    return 0;
}

uint32_t
NocDevice::read(Addr offset)
{
    switch(offset) {
      case REG0:
        return 0xffffffff;
      case REG1:
        return 0x12345678;
      default:
        warn("Unhandled read offset (0x%x)\n", offset);
        return 0;
    }
}

Tick
NocDevice::write(PacketPtr pkt)
{
    const Addr offset = pkt->getAddr() - pio_addr;
    const unsigned size(pkt->getSize());

    DPRINTF(NocDevice, "Writing %u bytes @ 0x%x:\n", size, offset);

    DPRINTF(NocDevice, "    value: 0x%llx\n", size == 8 ? pkt->getLE<uint64_t>() : pkt->getLE<uint32_t>());
    pkt->makeResponse();
    write(offset, pkt->getLE<uint32_t>());
    return 0;
}

void
NocDevice::write(Addr offset, uint32_t value)
{
    switch(offset) {
      case REG0:
      {
        //TODO dma opeartions.
        //void
        //dmaRead(Addr addr, int size, Event *event, uint8_t *data,
        //    uint32_t sid, uint32_t ssid, Tick delay=0)
        //dmaWrite(Addr addr, int size, Event *event, uint8_t *data,
        //     uint32_t sid, uint32_t ssid, Tick delay=0)
        //struct mem_data data;

        //dmaRead((Addr)inst_info.src1, inst_info.src1_len, NULL, (uint8_t *)data.src1, 0, 0, 0);
        //dmaRead((Addr)inst_info.src2, inst_info.src1_len, NULL, (uint8_t *)data.src2, 0, 0, 0);
        //dmaRead((Addr)inst_info.src3, inst_info.src1_len, NULL, (uint8_t *)data.src3, 0, 0, 0);
        reg[0] = value;
        kick();
        break;
      }
      case REG1:
      {
        DPRINTF(NocDevice, "get REG1 write.\n");
        reg[1] = value;
        break;
      }
      default:
        warn("Unhandled read offset (0x%x)\n", offset);
        break;
    }
}

void
NocDevice::polling_event_callback()
{
    DPRINTF(NocDevice, "node %d polling external access...\n", node_index);
    this->schedule(pollingEvent, nextCycle());
}

void
NocDevice::kick()
{
    DPRINTF(NocDevice, "kick(): Sending interrupt...\n");
    setInterrupts(1);
}

void
NocDevice::setInterrupts(uint32_t value)
{
    platform->postPciInt(interrupt_id);
}

AddrRangeList
NocDevice::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pio_addr, pio_size));
    return ranges;
}

} // namespace RiscvISA

} // namespace gem5
