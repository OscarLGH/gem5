#include "dev/riscv/qemu_pcie_bridge.hh"

#include "debug/QemuPcieBridge.hh"
#include "mem/packet_access.hh"
#include "params/QemuPcieBridge.hh"

#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

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

    std::string path = params.fifo_path + "/" + std::to_string(node_index) + "/";
    tx_fd_req_name = (path + params.tx_fifo_req_name).c_str();
    tx_fd_resp_name = (char *)(path + params.tx_fifo_resp_name).c_str();

    rx_fd_req_name = (char *)(path + params.rx_fifo_req_name).c_str();
    rx_fd_resp_name = (char *)(path + params.rx_fifo_resp_name).c_str();

    remote_shm_name = (char *)(params.remote_shm_name).c_str();
    remote_shm_size = 0x400000000;

    rx_fd_req = open(rx_fd_req_name, O_RDONLY | O_NONBLOCK, 0644);
    rx_fd_resp = open(rx_fd_resp_name, O_RDWR | O_NONBLOCK, 0644);

    tx_fd_req = open(tx_fd_req_name, O_RDWR | O_NONBLOCK, 0644);
    tx_fd_resp = open(tx_fd_resp_name, O_RDWR | O_NONBLOCK, 0644);

    remote_fd_shm = shm_open(remote_shm_name, O_RDWR, 0666);
    remote_shm_ptr = mmap(0x0, remote_shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, remote_fd_shm, 0);
  
    memset(reg, 0, sizeof(reg));

    this->schedule(pollingEvent, nextCycle());
}

QemuPcieBridge::~QemuPcieBridge()
{
}

Tick
QemuPcieBridge::read(PacketPtr pkt)
{
    const Addr offset = pkt->getAddr() - pio_addr;
    const unsigned size(pkt->getSize());

    //DPRINTF(QemuPcieBridge, "Reading %u bytes @ 0x%x:\n", size, offset);

    const uint64_t value = read(offset);
    //DPRINTF(QemuPcieBridge, "    value: 0x%llx\n", value);
    pkt->makeResponse();
    size == 8 ? pkt->setLE<uint64_t>(value) : pkt->setLE<uint32_t>(value);

    return 0;
}

uint64_t
QemuPcieBridge::read(Addr offset)
{
    switch(offset) {
      case PCIE_DOORBELL_R2L:
        return 0xffffffffffffffff;
      case PCIE_MSG_TYPE_R2L:
      case PCIE_MSG_DATA_R2L:
        return reg[offset / 8];
      case PCIE_DOORBELL_L2R:
        return 0xffffffffffffffff;
      case PCIE_MSG_TYPE_L2R:
      case PCIE_MSG_DATA_L2R:
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
    exPktCmd cmd;
    switch(offset) {
      case PCIE_DOORBELL_R2L:
        kick();
        break;
      case PCIE_MSG_TYPE_R2L:
        reg[offset / 8] = value;
        break;
      case PCIE_MSG_DATA_R2L:
        reg[offset / 8] = value;
      case PCIE_DOORBELL_L2R:
        cmd.type = EX_PKT_IRQ;
        cmd.data = value;
        num_write = ::write(tx_fd_req, (void *)&cmd, (size_t)sizeof(cmd));
        break;
      case PCIE_MSG_TYPE_L2R:
        reg[offset / 8] = value;
        break;
      case PCIE_MSG_DATA_L2R:
        reg[offset / 8] = value;
        break;
      default:
        int chan = DW_HDMA_REG_GET_DMA_CTX_CHANNEL(offset);
        DMAEngine *dma;
        switch (offset % 0x200) {
            case DW_HDMA_REG_DMA_CTX_EN_WR:
                dma = &wr_dma_chs[chan];
                dma->ch_en = value;
                break;
            case DW_HDMA_REG_DMA_CTX_EN_RD:
                dma = &rd_dma_chs[chan];
                dma->ch_en = value;
                break;
            case DW_HDMA_REG_DMA_CTX_DOORBELL_WR:
                dw_edma_dma_doorbell_ring(chan, 1);
                break;
            case DW_HDMA_REG_DMA_CTX_DOORBELL_RD:
                dw_edma_dma_doorbell_ring(chan, 0);
                break;
            case DW_HDMA_REG_DMA_CTX_TRANS_SIZE_WR:
                dw_edma_dma_config_txdesc_len(value, chan, 1);
                break;
            case DW_HDMA_REG_DMA_CTX_TRANS_SIZE_RD:
                dw_edma_dma_config_txdesc_len(value, chan, 0);
                break;
            case DW_HDMA_REG_DMA_CTX_SAR_LO_WR:
                dw_edma_dma_config_txdesc_src_lo(value, chan, 1);
                break;
            case DW_HDMA_REG_DMA_CTX_SAR_HI_WR:
                dw_edma_dma_config_txdesc_src_hi(value, chan, 1);
                break;
            case DW_HDMA_REG_DMA_CTX_DAR_LO_WR:
                dw_edma_dma_config_txdesc_dst_lo(value, chan, 1);
                break;
            case DW_HDMA_REG_DMA_CTX_DAR_HI_WR:
                dw_edma_dma_config_txdesc_dst_hi(value, chan, 1);
                break;
            case DW_HDMA_REG_DMA_CTX_SAR_LO_RD:
                dw_edma_dma_config_txdesc_src_lo(value, chan, 0);
                break;
            case DW_HDMA_REG_DMA_CTX_SAR_HI_RD:
                dw_edma_dma_config_txdesc_src_hi(value, chan, 0);
                break;
            case DW_HDMA_REG_DMA_CTX_DAR_LO_RD:
                dw_edma_dma_config_txdesc_dst_lo(value, chan, 0);
                break;
            case DW_HDMA_REG_DMA_CTX_DAR_HI_RD:
                dw_edma_dma_config_txdesc_dst_hi(value, chan, 0);
                break;
            case DW_HDMA_REG_DMA_CTX_CTRL1_WR:
                if (value & HDMA_V0_LINKLIST_EN) {
                    dma = &wr_dma_chs[chan];
                    dma->list_dma = true;
                } else {
                    dma = &wr_dma_chs[chan];
                    dma->list_dma = false;
                }
                break;
            case DW_HDMA_REG_DMA_CTX_CTRL1_RD:
                if (value & HDMA_V0_LINKLIST_EN) {
                    dma = &rd_dma_chs[chan];
                    dma->list_dma = true;
                } else {
                    dma = &rd_dma_chs[chan];
                    dma->list_dma = false;
                }
                break;
            default:
                break;
        }
        break;
    }
}


/**
 * dw_edma_dma_execute: Execute the DMA operation
 *
 * Effectively executes the DMA operation according to the configurations
 * in the transfer descriptor.
 *
 * @dev: Instance of DW_EDMADevice object being used
 */
void
QemuPcieBridge::dw_edma_dma_execute(int chan, bool rw)
{
    int err = 0;
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    if (!dma->list_dma) {
        if (rw) {
            DPRINTF(QemuPcieBridge, "dma write to remote device: sar = %lx dar = %lx len = %x\n \n", dma->txdesc.sar, dma->txdesc.dar, dma->txdesc.size);
            // dma read from local memory.
            dmaRead(dma->txdesc.dar, dma->txdesc.size, NULL, (uint8_t *)remote_shm_ptr + dma->txdesc.sar, 0, 0, 0);
        } else {
            DPRINTF(QemuPcieBridge, "dma read from device: sar = %lx dar = %lx len = %x\n \n", dma->txdesc.sar, dma->txdesc.dar, dma->txdesc.size);
            dmaWrite(dma->txdesc.sar, dma->txdesc.size, NULL, (uint8_t *)remote_shm_ptr + dma->txdesc.dar, 0, 0, 0);
        }
    } else {
        DPRINTF(QemuPcieBridge, "list dma:\n");
        uint64_t lintlist_addr = dma->linklist_addr;
        uint64_t offset = 0;
        while(1) {
            DMATransferDesc *linklist_desc;
            DPRINTF(QemuPcieBridge, "dma descriptor fetch addr:%lx \n", lintlist_addr + offset);
            //err = pci_dma_read(&dev->pci_dev, lintlist_addr + offset, &linklist_desc,
            //                        sizeof(linklist_desc));
            //if (err) {
            //    qemu_log_mask(LOG_GUEST_ERROR, "pci_dma_read err=%d\n", err);
            //}
            dmaRead(lintlist_addr + offset, sizeof(DMATransferDesc), NULL, (uint8_t *)linklist_desc, 0, 0, 0);
            //linklist_desc = (DMATransferDesc *)&device_buffer[lintlist_addr + offset];
            if  (linklist_desc->size == 0) {
                break;
            }
            DPRINTF(QemuPcieBridge, "dma descriptor: sar:%lx dar:%lx size:%x\n", linklist_desc->sar, linklist_desc->dar, linklist_desc->size);
            // perform dma opeartions.
            if (rw) {
                DPRINTF(QemuPcieBridge, "dma write: dar = %lx sar = %lx len = %x\n \n", linklist_desc->dar, linklist_desc->sar, linklist_desc->size);
                //err = pci_dma_write(&dev->pci_dev, linklist_desc->dar, dev->device_buffer + linklist_desc->sar,
                //                        linklist_desc->size);
                dmaRead(linklist_desc->dar, linklist_desc->size, NULL, (uint8_t *)remote_shm_ptr + linklist_desc->sar);
            } else {
                DPRINTF(QemuPcieBridge, "dma write: dar = %lx sar = %lx len = %x\n \n", linklist_desc->dar, linklist_desc->sar, linklist_desc->size);
                //err = pci_dma_read(&dev->pci_dev,linklist_desc->sar, dev->device_buffer + linklist_desc->dar,
                //                        linklist_desc->size);
                dmaWrite(linklist_desc->sar, linklist_desc->size, NULL, (uint8_t *)remote_shm_ptr + linklist_desc->dar);
            }

            // LINK NODE
            if (linklist_desc->ctrl & (1 << 2)) {
                if (linklist_desc->sar == dma->linklist_addr) {
                    break;
                }

                lintlist_addr = linklist_desc->sar;
                offset = 0;
                continue;
            }

            offset += sizeof(*linklist_desc);
        }
    }
    //dw_edma_irq_raise(dev, chan * 2 + (!rw));
}

void QemuPcieBridge::dw_edma_dma_config_txdesc_src_lo(uint64_t src, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->txdesc.sar = src;
}

void QemuPcieBridge::dw_edma_dma_config_txdesc_src_hi(uint64_t src, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->txdesc.sar &= 0xffffffff;
    dma->txdesc.sar |= (src << 32);
}

void QemuPcieBridge::dw_edma_dma_config_txdesc_src(uint64_t src, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->txdesc.sar = src;
}

/**
 * dw_edma_dma_config_txdesc_dst: Configure the destination register
 *
 * The destination register inside the transfer descriptor (txdesc)
 * describes destination of the DMA operation. It can be :
 *  - the offset inside the DMA memory area when direction is "to device"
 *  - the bus address pointing to RAM (or other) when direction is "from device"
 *
 * @dev: Instance of DW_EDMADevice object being used
 */
void QemuPcieBridge::dw_edma_dma_config_txdesc_dst_lo(uint64_t dst, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->txdesc.dar = dst;
}

void QemuPcieBridge::dw_edma_dma_config_txdesc_dst_hi(uint64_t dst, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->txdesc.dar &= 0xffffffff;
    dma->txdesc.dar |= (dst << 32);
}

void QemuPcieBridge::dw_edma_dma_config_txdesc_dst(uint64_t dst, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->txdesc.dar = dst;
}

void QemuPcieBridge::dw_edma_dma_config_linklist_lo(uint64_t addr, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->linklist_addr = addr;
}

void QemuPcieBridge::dw_edma_dma_config_linklist_hi(uint64_t addr, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->linklist_addr &= 0xffffffff;
    dma->linklist_addr |= (addr << 32);
}

void QemuPcieBridge::dw_edma_dma_config_linklist(uint64_t addr, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->linklist_addr = addr;
}

/**
 * dw_edma_dma_config_txdesc_len: Configure the length register
 *
 * The length register inside the transfer descriptor (txdesc) describes
 * the size of the DMA operation in bytes.
 *
 * @dev: Instance of DW_EDMADevice object being used
 */
void QemuPcieBridge::dw_edma_dma_config_txdesc_len(uint64_t size, int chan, bool rw)
{
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dma->txdesc.size = size;
}

/**
 * dw_edma_dma_doorbell_ring: Reception of a doorbell
 *
 * When the host (or other device) writes to the doorbell register
 * it is signaling to the DMA engine to start executing the DMA.
 * At this point, it is assumed that the host has already (and properly)
 * configured all necessary DMA engine registers.
 *
 * @dev: Instance of DW_EDMADevice object being used
 */
void QemuPcieBridge::dw_edma_dma_doorbell_ring(int chan, bool rw)
{
    /* atomic access of dma.status may not be neeeded as the MMIO access
     * will be normally serialized.
     * Though not really necessary, it can show that we need to think of
     * atomic accessing regions, especially if the device is a bit more
     * complex.
     */
    DMAEngine *dma = rw ? &wr_dma_chs[chan] : &rd_dma_chs[chan];
    dw_edma_dma_execute(chan, rw);
}

void
QemuPcieBridge::polling_event_callback()
{
    //DPRINTF(QemuPcieBridge, "node %d polling QEMU access...\n", node_index);
    exPktCmd cmd;
    ssize_t num_read = ::read(rx_fd_req, (void *)&cmd, (size_t)sizeof(cmd));
    if (num_read != sizeof(cmd)) {
      //DPRINTF(QemuPcieBridge, "no qemu data.\n");
    } else {
      DPRINTF(QemuPcieBridge, "Got qemu cmd:addr = %llx size = %d, type = %d.\n", cmd.addr, cmd.length, cmd.type);
      if (cmd.type == EX_PKT_RD) {
        dmaRead((Addr)cmd.addr, cmd.length, NULL, (uint8_t *)&cmd.data, 0, 0, 0);
        ssize_t num_write = ::write(rx_fd_resp, (void *)&cmd, (size_t)sizeof(cmd));
      } else if (cmd.type == EX_PKT_WR) {
        dmaWrite((Addr)cmd.addr, cmd.length, NULL, (uint8_t *)&cmd.data, 0, 0, 0);
      } else {
        DPRINTF(QemuPcieBridge, "Got qemu cmd:unsupport type = %d.\n", cmd.type);
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
