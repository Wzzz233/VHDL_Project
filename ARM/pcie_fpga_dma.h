/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCIe FPGA DMA Driver Header
 *
 * Driver for RK3568 to access FPGA registers via PCIe and perform DMA transfers
 * for camera frame data (1280x720 RGB565)
 *
 * FPGA: PG2L50H (Pango) - PCIe Endpoint
 * Vendor ID: 0x0755
 * Device ID: 0x0755
 */

#ifndef _PCIE_FPGA_DMA_H
#define _PCIE_FPGA_DMA_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* Driver name and version */
#define FPGA_DMA_DRV_NAME    "fpga_dma"
#define FPGA_DMA_DRV_VERSION "1.0"

/* Device file name */
#define FPGA_DMA_DEV_NAME    "fpga_dma0"

/* Frame buffer parameters (matching FPGA fram_buf.v) */
#define FPGA_FRAME_WIDTH           1280
#define FPGA_FRAME_HEIGHT          720
#define FPGA_FRAME_BPP_BGR565      2U
#define FPGA_FRAME_BPP_BGRX8888    4U
#define FPGA_FRAME_SIZE_BGR565     (FPGA_FRAME_WIDTH * FPGA_FRAME_HEIGHT * FPGA_FRAME_BPP_BGR565)
#define FPGA_FRAME_SIZE_BGRX8888   (FPGA_FRAME_WIDTH * FPGA_FRAME_HEIGHT * FPGA_FRAME_BPP_BGRX8888)
#define FPGA_FRAME_MAX_BPP         FPGA_FRAME_BPP_BGRX8888
#define FPGA_FRAME_MAX_SIZE        FPGA_FRAME_SIZE_BGRX8888
/* Keep a conservative default size for legacy callers. */
#define FPGA_FRAME_SIZE            FPGA_FRAME_MAX_SIZE

/* Pixel format reported by fpga_info.pixel_format */
#define FPGA_PIXEL_FORMAT_BGR565    0U
#define FPGA_PIXEL_FORMAT_BGRX8888  1U

/* BAR1 DMA Control Register Offsets (from ips2l_pcie_dma_controller.v) */
#define BAR1_DMA_CMD_REG     0x100  /* DMA command register */
#define BAR1_DMA_L_ADDR      0x110  /* Lower 32-bit target address */
#define BAR1_DMA_H_ADDR      0x120  /* Upper 32-bit target address (64-bit mode) */

/* DMA Command Register Bit Fields */
#define DMA_CMD_LEN_MASK     0x3FF  /* Bits [9:0] - Transfer length in DWORDs minus 1 */
#define DMA_CMD_64BIT_ADDR   (1 << 16)  /* Bit [16] - 1=64-bit address, 0=32-bit */
#define DMA_CMD_WRITE        (1 << 24)  /* Bit [24] - 1=Write (MWR), 0=Read (MRD) */

/* Maximum DMA transfer size per chunk in DWORDs.
 * cmd_reg[9:0] encodes (length - 1); 0x3FF encodes 1024 DW.
 * The RTL uses 10-bit length where value 0 represents 1024 DW. */
#define DMA_MAX_LEN_DWORDS   1024
#define DMA_MAX_LEN_BYTES    (DMA_MAX_LEN_DWORDS * 4)

/* IOCTL magic number */
#define FPGA_DMA_IOC_MAGIC   'F'

/* IOCTL commands */
#define FPGA_DMA_GET_INFO    _IOR(FPGA_DMA_IOC_MAGIC, 1, struct fpga_info)
#define FPGA_DMA_READ_FRAME  _IOWR(FPGA_DMA_IOC_MAGIC, 2, struct dma_transfer)
#define FPGA_DMA_MAP_BUFFER  _IOWR(FPGA_DMA_IOC_MAGIC, 3, struct buffer_map)

/**
 * struct fpga_info - FPGA device information
 * @vendor_id: PCI vendor ID
 * @device_id: PCI device ID
 * @bar0_size: BAR0 size in bytes
 * @bar1_size: BAR1 size in bytes
 * @link_width: PCIe link width (1, 2, 4, 8)
 * @link_speed: PCIe link speed (1=Gen1, 2=Gen2)
 * @frame_width: Frame width in pixels
 * @frame_height: Frame height in pixels
 * @frame_bpp: Bytes per pixel
 * @frame_stride: Bytes per line
 * @pixel_format: Pixel format enum (FPGA_PIXEL_FORMAT_*)
 */
struct fpga_info {
    __u32 vendor_id;
    __u32 device_id;
    __u32 bar0_size;
    __u32 bar1_size;
    __u32 link_width;
    __u32 link_speed;
    __u32 frame_width;
    __u32 frame_height;
    __u32 frame_bpp;
    __u32 frame_stride;
    __u32 pixel_format;
};

/**
 * struct dma_transfer - DMA transfer parameters
 * @size: Number of bytes to transfer
 * @offset: Offset in FPGA DDR3 memory to read from (for future use)
 * @flags: Transfer flags (reserved for future use)
 * @result: Result code (0=success, negative=error)
 */
struct dma_transfer {
    __u32 size;
    __u32 offset;
    __u32 flags;
    __u32 result;
    __u64 user_buf;   /* Userspace buffer address; driver copies DMA data here */
};

/**
 * struct buffer_map - Buffer mapping for mmap
 * @index: Buffer index (0 for single buffer)
 * @size: Buffer size
 * @offset: Offset for mmap (returned by driver)
 */
struct buffer_map {
    __u32 index;
    __u32 size;
    __u64 offset;
};

#endif /* _PCIE_FPGA_DMA_H */
