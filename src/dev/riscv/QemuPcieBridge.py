from m5.objects.PlicDevice import PlicDmaDevice
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from m5.util.fdthelper import *

class QemuPcieBridge(PlicDmaDevice):
    type = "QemuPcieBridge"
    cxx_header = "dev/riscv/qemu_pcie_bridge.hh"
    cxx_class = "gem5::RiscvISA::QemuPcieBridge"
    pio_size = 0x20000000
    interrupt_id = Param.Int("PLIC Interrupt ID")

    def generateDeviceTree(self, state):
        node = self.generatePlicDeviceNode(state, "qemu_pcie_bridge,mmio")
        node.appendCompatible(["qemu_pcie_bridge,mmio"])

        yield node