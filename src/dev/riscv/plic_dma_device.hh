#ifndef __DEV_RISCV_PLIC_DMA_DEVICE_HH__
#define __DEV_RISCV_PLIC_DMA_DEVICE_HH__

#include "dev/dma_device.hh"
#include "dev/platform.hh"
#include "params/PlicDmaDevice.hh"
#include "sim/system.hh"

namespace gem5
{

class PlicDmaDevice : public DmaDevice
{
  protected:
    System *system;
    Platform *platform;
    int interrupt_id;
    Addr pio_addr;
    Addr pio_size;

  public:
    typedef PlicDmaDeviceParams Params;

    const Params &
    params() const
    {
        return dynamic_cast<const Params &>(_params);
    }

    PlicDmaDevice(const Params &params);

    const int &
    id()
    {
      return interrupt_id;
    }

};

} // namespace gem5

#endif // __DEV_RISCV_PLIC_DMA_DEVICE_HH__
