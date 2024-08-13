
#include "dev/riscv/plic_dma_device.hh"

namespace gem5
{

PlicDmaDevice::PlicDmaDevice(const Params &params) :
    DmaDevice(params), pio_addr(params.pio_addr),
    pio_size(params.pio_size),
    system(params.system),
    platform(params.platform),
    interrupt_id(params.interrupt_id)
{
}

} // namespace gem5