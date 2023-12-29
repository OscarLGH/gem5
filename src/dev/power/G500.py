from m5.params import *
from m5.proxy import *
from m5.objects.Device import BasicPioDevice, PioDevice, IsaFake, BadAddr
from m5.objects.Platform import Platform
from m5.objects.Terminal import Terminal
from m5.objects.Uart import Uart8250
from m5.objects.Icp import Icp

class G500(Platform):
    type = 'G500'
    cxx_header = "dev/power/g500.hh"
    cxx_class = 'gem5::PowerISA::G500'
    system = Param.System(Parent.any, "system")
    terminal = Terminal()
    uart = Uart8250(pio_addr=0xFFFF4505)
    icp = Icp(pio_addr=0x3FFFF80008000)

    def attachIO(self,bus):
        self.uart.device = self.terminal
        self.uart.pio = bus.master
        self.icp.device = self.terminal
        self.icp.pio = bus.master
