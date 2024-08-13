from m5.objects.PlicDevice import PlicDmaDevice
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from m5.util.fdthelper import *

class NocDevice(PlicDmaDevice):
    type = "NocDevice"
    cxx_header = "dev/riscv/noc_device.hh"
    cxx_class = "gem5::RiscvISA::NocDevice"
    pio_size = 0x20000000
    interrupt_id = Param.Int("PLIC Interrupt ID")

    def generateDeviceTree(self, state):
        node = self.generatePlicDeviceNode(state, "noc_device,mmio")
        node.appendCompatible(["noc_device,mmio"])

        yield node