from m5.objects.PlicDevice import PlicDmaDevice
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from m5.util.fdthelper import *

class QemuPcieBridge(PlicDmaDevice):
    type = "QemuPcieBridge"
    cxx_header = "dev/riscv/qemu_pcie_bridge.hh"
    cxx_class = "gem5::RiscvISA::QemuPcieBridge"
    pio_size = 0x10000000
    interrupt_id = Param.Int("PLIC Interrupt ID")
    fifo_path = Param.String("")
    tx_fifo_req_name = Param.String("")
    tx_fifo_resp_name = Param.String("")
    rx_fifo_req_name = Param.String("")
    rx_fifo_resp_name = Param.String("")
    remote_shm_name = Param.String("")

    def generateDeviceTree(self, state):
        node = self.generatePlicDeviceNode(state, "qemu_pcie_bridge,mmio")
        node.appendCompatible(["qemu_pcie_bridge,mmio"])

        yield node