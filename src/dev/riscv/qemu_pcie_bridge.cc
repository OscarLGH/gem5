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

static int dev_cnt = 0;

QemuPcieBridge::QemuPcieBridge(const QemuPcieBridgeParams &params)
    : PlicDmaDevice(params),interrupt_id(params.interrupt_id),
    pollingEvent([this]{ polling_event_callback(); }, "polling event")
{
    node_index = dev_cnt++;
    this->schedule(pollingEvent, nextCycle());
    qemu_fd_req = open("/tmp/qemu-fifo-req", O_RDONLY | O_NONBLOCK, 0644);
    qemu_fd_resp = open("/tmp/qemu-fifo-resp", O_RDONLY | O_NONBLOCK, 0644);
    qemu_fd_irq = open("/tmp/qemu-fd-irq", O_RDWR, 0644);
    memset(reg, 0, sizeof(reg));
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

uint64_t
QemuPcieBridge::read(Addr offset)
{
    switch(offset) {
      case PCIE_DOORBELL_H2D:
        return 0xffffffffffffffff;
      case PCIE_MSG_TYPE_H2D:
      case PCIE_MSG_DATA_H2D:
        return reg[offset / 8];
      case PCIE_DOORBELL_D2H:
        return 0xffffffffffffffff;
      case PCIE_MSG_TYPE_D2H:
      case PCIE_MSG_DATA_D2H:
        return reg[offset / 8];
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
    write(offset, pkt->getLE<uint64_t>());
    return 0;
}

void
QemuPcieBridge::write(Addr offset, uint64_t value)
{
    ssize_t num_write;
    switch(offset) {
      case PCIE_DOORBELL_H2D:
        kick();
        break;
      case PCIE_MSG_TYPE_H2D:
        reg[offset / 8] = value;
        break;
      case PCIE_MSG_DATA_H2D:
        reg[offset / 8] = value;
      case PCIE_DOORBELL_D2H:
        num_write = ::write(qemu_fd_irq, (void *)&value, (size_t)sizeof(value));
        break;
      case PCIE_MSG_TYPE_D2H:
        reg[offset / 8] = value;
        break;
      case PCIE_MSG_DATA_D2H:
        reg[offset / 8] = value;
        break;
      default:
        warn("Unhandled write offset (0x%x)\n", offset);
        break;
    }
}

void
QemuPcieBridge::polling_event_callback()
{
    //DPRINTF(QemuPcieBridge, "node %d polling QEMU access...\n", node_index);
    qemuMmioCmd cmd = {0};
    ssize_t num_read = ::read(qemu_fd_req, (void *)&cmd, (size_t)sizeof(cmd));
    if (num_read != sizeof(cmd)) {
      //DPRINTF(QemuPcieBridge, "no qemu data.\n");
    } else {
      //DPRINTF(QemuPcieBridge, "Got qemu cmd:addr = %llx size = %d, %s.\n", cmd.addr, cmd.length, cmd.rw ? "W" : "R");
      if (!cmd.rw) {
        dmaRead((Addr)cmd.addr, cmd.length, NULL, (uint8_t *)&cmd.data, 0, 0, 0);
        ssize_t num_write = ::write(qemu_fd_resp, (void *)&cmd, (size_t)sizeof(cmd));
      } else {
        dmaWrite((Addr)cmd.addr, cmd.length, NULL, (uint8_t *)&cmd.data, 0, 0, 0);
      }
    }
    this->schedule(pollingEvent, nextCycle());
}

void
QemuPcieBridge::kick()
{
    DPRINTF(QemuPcieBridge, "kick(): Sending interrupt...\n");
    setInterrupts(interrupt_id);
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
