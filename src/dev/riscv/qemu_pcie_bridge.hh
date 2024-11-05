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

    enum dmaDirection {
      DMA_DIR_REMOTE_TO_LOCAL = 0,
      DMA_DIR_LOCAL_TO_REMOTE = 1
    };

    int node_index;
    int interrupt_id;

    enum exPktType {
      EX_PKT_RD = 0,
      EX_PKT_WR = 1,
      EX_PKT_IRQ = 2
    };

    typedef struct exPktCmd {
      exPktType type;
      int length;
      uint64_t addr;
      uint64_t data;
    } exPktCmd;

    /* transfer descriptor */
    typedef struct DMATransferDesc {
        uint32_t ctrl;
        uint32_t size;
        uint64_t sar;
        uint64_t dar;
    } DMATransferDesc;

    typedef struct DMAEngine {
        bool ch_en;
        bool intr_enable;
        bool remote_intr_enable;
        bool list_dma;

        DMATransferDesc txdesc;
        uint64_t linklist_addr;
        DMATransferDesc *txdesc_head;
        uint8_t *buff;
    } DMAEngine;

    const char *rx_fd_req_name;
    const char *rx_fd_resp_name;
    int rx_fd_req;
    int rx_fd_resp;

    const char *tx_fd_req_name;
    const char *tx_fd_resp_name;

    int tx_fd_req;
    int tx_fd_resp;

    const char *remote_shm_name;
    int remote_fd_shm;
    uint64_t remote_shm_size;
    void *remote_shm_ptr;

  protected: // BasicPioDevice
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  protected:
    

    enum : Addr
    {
        PCIE_DOORBELL_R2L = 0x0,
        PCIE_MSG_TYPE_R2L = 0x8,
        PCIE_MSG_DATA_R2L = 0x10,
        PCIE_DOORBELL_L2R = 0x100,
        PCIE_MSG_TYPE_L2R = 0x108,
        PCIE_MSG_DATA_L2R = 0x110,
    };

    #define HDMA_V0_MAX_NR_CH			8
    #define HDMA_V0_LOCAL_ABORT_INT_EN		BIT(6)
    #define HDMA_V0_REMOTE_ABORT_INT_EN		BIT(5)
    #define HDMA_V0_LOCAL_STOP_INT_EN		BIT(4)
    #define HDMA_V0_REMOTE_STOP_INT_EN		BIT(3)
    #define HDMA_V0_ABORT_INT_MASK			BIT(2)
    #define HDMA_V0_STOP_INT_MASK			BIT(0)
    #define HDMA_V0_LINKLIST_EN			(1 << 0)
    #define HDMA_V0_CONSUMER_CYCLE_STAT		BIT(1)
    #define HDMA_V0_CONSUMER_CYCLE_BIT		BIT(0)
    #define HDMA_V0_DOORBELL_START			BIT(0)

    #define DW_HDMA_REG_BASE_OFFSET 0x1000
    #define DW_HDMA_REG_GET_DMA_CTX_CHANNEL(x) ((x)/0x200)
    #define DW_HDMA_REG_STATUS_WR 0x80
    #define DW_HDMA_REG_STATUS_RD 0x180
    #define DW_HDMA_REG_INTR_STATUS_WR 0x84
    #define DW_HDMA_REG_INTR_STATUS_RD 0x184
    #define DW_HDMA_REG_INTR_SETUP_WR 0x88
    #define DW_HDMA_REG_INTR_SETUP_RD 0x188
    #define DW_HDMA_REG_INTR_CLR_WR 0x8c
    #define DW_HDMA_REG_INTR_CLR_RD 0x18c
    #define DW_HDMA_REG_MSI_STOP_LO_WR 0x90
    #define DW_HDMA_REG_MSI_STOP_LO_RD 0x190
    #define DW_HDMA_REG_MSI_STOP_HI_WR 0x94
    #define DW_HDMA_REG_MSI_STOP_HI_RD 0x194
    #define DW_HDMA_REG_MSI_ABORT_LO_WR 0x98
    #define DW_HDMA_REG_MSI_ABORT_LO_RD 0x198
    #define DW_HDMA_REG_MSI_ABORT_HI_WR 0x9c
    #define DW_HDMA_REG_MSI_ABORT_HI_RD 0x19c
    #define DW_HDMA_REG_MSI_WATERMARK_LO_WR 0xa0
    #define DW_HDMA_REG_MSI_WATERMARK_LO_RD 0x1a0
    #define DW_HDMA_REG_MSI_WATERMARK_HI_WR 0xa4
    #define DW_HDMA_REG_MSI_WATERMARK_HI_RD 0x1a4
    #define DW_HDMA_REG_MSI_MSGD_WR 0xa8
    #define DW_HDMA_REG_MSI_MSGD_RD 0x1a8

    #define DW_HDMA_REG_DMA_CTX_BASE 0x200

    #define DW_HDMA_REG_DMA_CTX_EN_WR 0x0
    #define DW_HDMA_REG_DMA_CTX_EN_RD 0x100
    #define DW_HDMA_REG_DMA_CTX_DOORBELL_WR 0x4
    #define DW_HDMA_REG_DMA_CTX_DOORBELL_RD 0x104
    #define DW_HDMA_REG_DMA_CTX_PREFETCH 0x108
    #define DW_HDMA_REG_DMA_CTX_HANDSHAKE_WR 0xc
    #define DW_HDMA_REG_DMA_CTX_HANDSHAKE_RD 0x10c
    #define DW_HDMA_REG_DMA_CTX_LLP_LO_WR 0x10
    #define DW_HDMA_REG_DMA_CTX_LLP_LO_RD 0x110
    #define DW_HDMA_REG_DMA_CTX_LLP_HI_WR 0x14
    #define DW_HDMA_REG_DMA_CTX_LLP_HI_RD 0x114
    #define DW_HDMA_REG_DMA_CTX_CYCLE_WR 0x18
    #define DW_HDMA_REG_DMA_CTX_CYCLE_RD 0x118
    #define DW_HDMA_REG_DMA_CTX_TRANS_SIZE_WR 0x1c
    #define DW_HDMA_REG_DMA_CTX_TRANS_SIZE_RD 0x11c
    #define DW_HDMA_REG_DMA_CTX_SAR_LO_WR 0x20
    #define DW_HDMA_REG_DMA_CTX_SAR_LO_RD 0x120
    #define DW_HDMA_REG_DMA_CTX_SAR_HI_WR 0x24
    #define DW_HDMA_REG_DMA_CTX_SAR_HI_RD 0x124
    #define DW_HDMA_REG_DMA_CTX_DAR_LO_WR 0x28
    #define DW_HDMA_REG_DMA_CTX_DAR_LO_RD 0x128
    #define DW_HDMA_REG_DMA_CTX_DAR_HI_WR 0x2c
    #define DW_HDMA_REG_DMA_CTX_DAR_HI_RD 0x12c
    #define DW_HDMA_REG_DMA_CTX_WATER_EN_WR 0x30
    #define DW_HDMA_REG_DMA_CTX_WATER_EN_RD 0x130
    #define DW_HDMA_REG_DMA_CTX_CTRL1_WR 0x34
    #define DW_HDMA_REG_DMA_CTX_CTRL1_RD 0x134
    #define DW_HDMA_REG_DMA_CTX_FUNC_NUM_WR 0x38
    #define DW_HDMA_REG_DMA_CTX_FUNC_NUM_RD 0x138
    #define DW_HDMA_REG_DMA_CTX_QOS_WR 0x3c
    #define DW_HDMA_REG_DMA_CTX_QOS_RD 0x13c

    #define DW_EDMA_RD_CHANNELS 8
    #define DW_EDMA_WR_CHANNELS 8

    /* DMAs */
    DMAEngine wr_dma_chs[DW_EDMA_WR_CHANNELS];
    DMAEngine rd_dma_chs[DW_EDMA_RD_CHANNELS];

    uint64_t read(Addr offset);
    void write(Addr offset, uint64_t value);

    void dw_edma_dma_execute(int chan, bool rw);
    void dw_edma_dma_config_txdesc_src_lo(uint64_t src, int chan, bool rw);
    void dw_edma_dma_config_txdesc_src_hi(uint64_t src, int chan, bool rw);
    void dw_edma_dma_config_txdesc_src(uint64_t src, int chan, bool rw);
    void dw_edma_dma_config_txdesc_dst_lo(uint64_t dst, int chan, bool rw);
    void dw_edma_dma_config_txdesc_dst_hi(uint64_t dst, int chan, bool rw);
    void dw_edma_dma_config_txdesc_dst(uint64_t dst, int chan, bool rw);
    void dw_edma_dma_config_linklist_lo(uint64_t addr, int chan, bool rw);
    void dw_edma_dma_config_linklist_hi(uint64_t addr, int chan, bool rw);
    void dw_edma_dma_config_linklist(uint64_t addr, int chan, bool rw);
    void dw_edma_dma_config_txdesc_len(uint64_t size, int chan, bool rw);
    void dw_edma_dma_doorbell_ring(int chan, bool rw);

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
