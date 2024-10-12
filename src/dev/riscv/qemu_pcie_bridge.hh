#ifndef __DEV_RISCV_QEMU_PCIE_BRIDGE_HH__
#define __DEV_RISCV_QEMU_PCIE_BRIDGE_HH__

#include <semaphore.h>

#include "dev/riscv/hifive.hh"
#include "dev/riscv/plic_dma_device.hh"
#include "dev/virtio/base.hh"

namespace gem5
{

struct QemuPcieBridgeParams;

namespace RiscvISA
{

class QemuPcieBridge : public PlicDmaDevice
{
  public:
    QemuPcieBridge(const QemuPcieBridgeParams &params);
    ~QemuPcieBridge();

    AddrRangeList getAddrRanges() const override;

    void polling_event_callback();

    int node_index;
    int interrupt_id;

    typedef struct qemuMmioCmd {
      bool rw;
      int length;
      uint64_t addr;
      uint64_t data;
    } qemuMmioCmd;

    int qemu_fd_req;
    int qemu_fd_resp;
    int qemu_fd_irq;

  protected: // BasicPioDevice
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  protected:
    

    enum : Addr
    {
        PCIE_DOORBELL_H2D = 0x0,
        PCIE_MSG_TYPE_H2D = 0x8,
        PCIE_MSG_DATA_H2D = 0x10,
        PCIE_DOORBELL_D2H = 0x100,
        PCIE_MSG_TYPE_D2H = 0x108,
        PCIE_MSG_DATA_D2H = 0x110,
    };

    uint64_t read(Addr offset);
    void write(Addr offset, uint64_t value);

    void kick();
    void setInterrupts(uint32_t value);

    EventFunctionWrapper pollingEvent;

    uint32_t pageSize;
    uint32_t interruptStatus;

    uint64_t reg[0x200];
};

} // namespace RiscvISA

} // namespace gem5

#endif // __DEV_RISCV_QEMU_PCIE_BRIDGE_HH__
