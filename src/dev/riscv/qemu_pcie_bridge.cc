#include "dev/riscv/qemu_pcie_bridge.hh"

#include "debug/QemuPcieBridge.hh"
#include "mem/packet_access.hh"
#include "params/QemuPcieBridge.hh"

#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>

namespace gem5
{

namespace RiscvISA
{

QemuPcieBridge::QemuPcieBridge(const QemuPcieBridgeParams &params)
    : PlicDmaDevice(params),interrupt_id(params.interrupt_id),
    pollingEvent([this]{ polling_event_callback(); }, "polling event")
{
    this->schedule(pollingEvent, curTick() + 1);
}

QemuPcieBridge::~QemuPcieBridge()
{
}

Tick
QemuPcieBridge::read(PacketPtr pkt)
{
    const Addr offset = pkt->getAddr() - pio_addr;
    const unsigned size(pkt->getSize());

    DPRINTF(QemuPcieBridge, "Reading %u bytes @ 0x%x:\n", size, offset);

    const uint64_t value = read(offset);
    DPRINTF(QemuPcieBridge, "    value: 0x%llx\n", value);
    pkt->makeResponse();
    size == 8 ? pkt->setLE<uint64_t>(value) : pkt->setLE<uint32_t>(value);

    return 0;
}

uint32_t
QemuPcieBridge::read(Addr offset)
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
QemuPcieBridge::write(PacketPtr pkt)
{
    const Addr offset = pkt->getAddr() - pio_addr;
    const unsigned size(pkt->getSize());

    DPRINTF(QemuPcieBridge, "Writing %u bytes @ 0x%x:\n", size, offset);

    DPRINTF(QemuPcieBridge, "    value: 0x%llx\n", size == 8 ? pkt->getLE<uint64_t>() : pkt->getLE<uint32_t>());
    pkt->makeResponse();
    write(offset, pkt->getLE<uint32_t>());
    return 0;
}

void
QemuPcieBridge::write(Addr offset, uint32_t value)
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
        DPRINTF(QemuPcieBridge, "get REG1 write.\n");
        reg[1] = value;
        break;
      }
      default:
        warn("Unhandled read offset (0x%x)\n", offset);
        break;
    }
}

void
QemuPcieBridge::polling_event_callback()
{
    DPRINTF(QemuPcieBridge, "polling QEMU access...\n");
    this->schedule(pollingEvent, curTick() + 1);
}

void
QemuPcieBridge::kick()
{
    DPRINTF(QemuPcieBridge, "kick(): Sending interrupt...\n");
    setInterrupts(1);
}

void
QemuPcieBridge::setInterrupts(uint32_t value)
{
    platform->postPciInt(interrupt_id);
}

AddrRangeList
QemuPcieBridge::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pio_addr, pio_size));
    return ranges;
}

} // namespace RiscvISA

} // namespace gem5
