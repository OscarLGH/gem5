# -*- mode:python -*-
from m5.params import *
from AbstractMemory import *

# A wrapper for Ramulator multi-channel memory controller
class Ramulator(AbstractMemory):
    type = 'Ramulator'
    cxx_header = "mem/ramulator.hh"

    # A single port for now
    port = SlavePort("Slave port")

    config_file = Param.String("", "configuration file")
    num_cpus = Param.Unsigned(1, "Number of cpu")
