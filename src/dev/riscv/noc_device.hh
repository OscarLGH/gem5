#ifndef __DEV_RISCV_NOC_DEVICE_HH__
#define __DEV_RISCV_NOC_DEVICE_HH__

#include "dev/riscv/hifive.hh"
#include "dev/riscv/plic_dma_device.hh"
#include "dev/virtio/base.hh"

namespace gem5
{

struct NocDeviceParams;

namespace RiscvISA
{

class NocDevice : public PlicDmaDevice
{
  public:
    NocDevice(const NocDeviceParams &params);
    ~NocDevice();

    AddrRangeList getAddrRanges() const override;

    void polling_event_callback();

    int node_index;
    int interrupt_id;

  protected: // BasicPioDevice
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  protected:
    

    enum : Addr
    {
        REG0 = 0x0,
        REG1 = 0x4,
        REG2 = 0x8
    };

    uint32_t read(Addr offset);
    void write(Addr offset, uint32_t value);

    void kick();
    void setInterrupts(uint32_t value);

    EventFunctionWrapper pollingEvent;

    uint32_t pageSize;
    uint32_t interruptStatus;

    uint32_t reg[16];
};

} // namespace RiscvISA

} // namespace gem5

#endif // __DEV_RISCV_NOC_DEVICE_HH__
