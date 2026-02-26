// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe FPGA DMA Driver
 *
 * Driver for RK3568 to access FPGA registers via PCIe and perform DMA transfers
 * for camera frame data (1280x720, format selected at runtime).
 *
 * FPGA: PG2L50H (Pango) - PCIe Endpoint
 * Vendor ID: 0x0755
 * Device ID: 0x0755
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/version.h>

#include "pcie_fpga_dma.h"

#define DRIVER_NAME   "fpga_dma"
#define DRIVER_AUTHOR "RK3568 FPGA DMA Driver"
#define DRIVER_DESC   "PCIe DMA driver for PG2L50H FPGA camera frame transfer"

#define FPGA_PCI_VENDOR_ID  0x0755
#define FPGA_PCI_DEVICE_ID  0x0755

/* Module parameters */
static int major_num;
static int dma_timeout_ms = 5000;  /* 5 seconds default timeout */
static int dma_chunk_delay_us = 0; /* inter-chunk pacing, default 0us */
static int dma_poll_sleep_us = 5;      /* initial sleep in wait loop (0=busy poll) */
static int dma_poll_sleep_max_us = 80; /* adaptive backoff upper bound */
static int dma_poll_backoff_polls = 8; /* bump sleep every N polls */
static bool dma_verbose = false;   /* verbose transfer logs */
static int dma_pixel_format = FPGA_PIXEL_FORMAT_BGRX8888;

module_param(major_num, int, 0);
MODULE_PARM_DESC(major_num, "Major device number (0=dynamic)");
module_param(dma_timeout_ms, int, 0644);
MODULE_PARM_DESC(dma_timeout_ms, "DMA transfer timeout in milliseconds");
module_param(dma_chunk_delay_us, int, 0644);
MODULE_PARM_DESC(dma_chunk_delay_us, "Delay in microseconds after each DMA chunk trigger");
module_param(dma_poll_sleep_us, int, 0644);
MODULE_PARM_DESC(dma_poll_sleep_us, "Initial sleep in microseconds while polling DMA completion (0=busy)");
module_param(dma_poll_sleep_max_us, int, 0644);
MODULE_PARM_DESC(dma_poll_sleep_max_us, "Maximum sleep in microseconds for adaptive DMA polling backoff");
module_param(dma_poll_backoff_polls, int, 0644);
MODULE_PARM_DESC(dma_poll_backoff_polls, "Increase poll sleep every N polls while waiting DMA completion");
module_param(dma_verbose, bool, 0644);
MODULE_PARM_DESC(dma_verbose, "Enable verbose DMA transfer logs");
module_param(dma_pixel_format, int, 0644);
MODULE_PARM_DESC(dma_pixel_format, "Frame format: 0=BGR565, 1=BGRX8888");

/* Per-device structure */
struct fpga_dma_dev {
    struct pci_dev *pdev;
    struct cdev cdev;
    struct device *dev;

    /* BAR mappings */
    void __iomem *bar0;  /* 64KB for PIO/data readback */
    void __iomem *bar1;  /* DMA control registers */
    resource_size_t bar0_size;
    resource_size_t bar1_size;

    /* DMA buffer */
    void *dma_buf;
    dma_addr_t dma_handle;
    size_t dma_buf_size;
    struct mutex dma_lock;

    /* Completion for DMA transfer */
    struct completion dma_done;

    /* Device info */
    struct fpga_info info;

    /* Character device */
    int major;
    int minor;
};

static struct class *fpga_dma_class;
static dev_t fpga_dma_dev_t;

/* Forward declarations */
static int fpga_dma_open(struct inode *inode, struct file *file);
static int fpga_dma_release(struct inode *inode, struct file *file);
static long fpga_dma_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int fpga_dma_mmap(struct file *file, struct vm_area_struct *vma);

static const struct file_operations fpga_dma_fops = {
    .owner = THIS_MODULE,
    .open = fpga_dma_open,
    .release = fpga_dma_release,
    .unlocked_ioctl = fpga_dma_ioctl,
    .compat_ioctl = fpga_dma_ioctl,
    .mmap = fpga_dma_mmap,
    .llseek = no_llseek,
};

/* PCI device ID table */
static const struct pci_device_id fpga_dma_pci_tbl[] = {
    { PCI_DEVICE(FPGA_PCI_VENDOR_ID, FPGA_PCI_DEVICE_ID) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, fpga_dma_pci_tbl);

static u32 fpga_dma_bpp_from_format(u32 pixel_format)
{
    switch (pixel_format) {
    case FPGA_PIXEL_FORMAT_BGRX8888:
        return FPGA_FRAME_BPP_BGRX8888;
    case FPGA_PIXEL_FORMAT_BGR565:
    default:
        return FPGA_FRAME_BPP_BGR565;
    }
}

static void fpga_dma_normalize_info_layout(struct fpga_info *info)
{
    u32 bpp;

    if (info->pixel_format != FPGA_PIXEL_FORMAT_BGR565 &&
        info->pixel_format != FPGA_PIXEL_FORMAT_BGRX8888)
        info->pixel_format = FPGA_PIXEL_FORMAT_BGR565;

    bpp = fpga_dma_bpp_from_format(info->pixel_format);
    info->frame_bpp = bpp;
    info->frame_stride = info->frame_width * bpp;
}

static size_t fpga_dma_default_frame_size(const struct fpga_info *info)
{
    return (size_t)info->frame_stride * (size_t)info->frame_height;
}

/**
 * fpga_dma_read_reg - Read a 32-bit register from BAR1
 */
static u32 __maybe_unused fpga_dma_read_reg(struct fpga_dma_dev *dev, u32 offset)
{
    /* BAR1 is Write-Only in current FPGA RTL. Reading causes Bus Error. */
    /* return ioread32(dev->bar1 + offset); */
    return 0;
}

/**
 * fpga_dma_write_reg - Write a 32-bit register to BAR1
 *
 * Keep writes posted; caller can flush once per chunk when needed.
 */
static void fpga_dma_write_reg(struct fpga_dma_dev *dev, u32 offset, u32 value)
{
    iowrite32(value, dev->bar1 + offset);
}

static inline void fpga_dma_flush_posted_writes(struct fpga_dma_dev *dev)
{
    /* One non-posted read flushes prior posted BAR1 writes on PCIe. */
    (void)ioread32(dev->bar0);
}

/**
 * fpga_dma_perform_transfer - Perform a DMA read from FPGA
 *
 * This initiates a DMA read operation by programming the FPGA's DMA controller.
 * The FPGA will read data from its DDR3 and write it to the RK3568's DMA buffer.
 */
static int fpga_dma_perform_transfer(struct fpga_dma_dev *dev, size_t size)
{
    u32 cmd_reg;
    int ret = 0;
    size_t remaining = size;
    size_t chunk_size;
    dma_addr_t current_addr = dev->dma_handle;
    int chunk_num = 0;

    if (dma_verbose) {
        dev_info(dev->dev, "=== DMA MWR Transfer Start ===\n");
        dev_info(dev->dev, "  Total size: %zu bytes\n", size);
        dev_info(dev->dev, "  DMA handle (bus addr): 0x%llx\n", (u64)dev->dma_handle);
        dev_info(dev->dev, "  DMA buf (virt): %px\n", dev->dma_buf);
        dev_info(dev->dev, "  BAR1 base (virt): %px\n", dev->bar1);
    }

    /* PIO read test: verify TLPs can reach FPGA via BAR0 read (MRd TLP) */
    {
        u32 bar0_val0 = ioread32(dev->bar0 + 0x00);
        u32 bar0_val4 = ioread32(dev->bar0 + 0x04);
        if (dma_verbose)
            dev_info(dev->dev, "  BAR0 PIO read test: [0x00]=0x%08x [0x04]=0x%08x\n",
                     bar0_val0, bar0_val4);
        if (bar0_val0 == 0xFFFFFFFF && bar0_val4 == 0xFFFFFFFF)
            dev_warn(dev->dev, "  >>> BAR0 reads all-F: TLPs may not reach FPGA! <<<\n");
    }

    /* Initialize completion */
    reinit_completion(&dev->dma_done);

    /* Process transfer in chunks to handle 4KB boundary */
    while (remaining > 0) {
        bool verbose_chunk = dma_verbose && ((chunk_num < 3) || ((chunk_num % 100) == 0));
        size_t buf_offset;
        u32 *chunk_tail0;
        u32 *chunk_tail1 = NULL;
        const u32 tail_sentinel0 = 0xDEADBEEF;
        const u32 tail_sentinel1 = 0xA5A55A5A;
        int poll_sleep_cur_us = dma_poll_sleep_us;
        int poll_sleep_max_us = dma_poll_sleep_max_us;
        int poll_backoff_polls = dma_poll_backoff_polls;
        int poll_count = 0;
        unsigned long deadline;

        /* Calculate chunk size (max 1024 DWORDs = 4096 bytes). */
        chunk_size = min(remaining, (size_t)DMA_MAX_LEN_BYTES);

        /* Check for 4KB boundary crossing */
        if ((current_addr & 0xFFF) + chunk_size > 0x1000) {
            /* Reduce chunk size to avoid crossing 4KB boundary */
            chunk_size = 0x1000 - (current_addr & 0xFFF);
            if (chunk_size == 0) {
                /* Address is exactly at boundary, align to next page */
                current_addr = ALIGN(current_addr, 0x1000);
                continue;
            }
        }

        /* Calculate length in DWORDs minus 1 (FPGA expects this format) */
        u32 length_in_dwords = (chunk_size + 3) / 4;  /* Round up to DWORDs */
        u32 cmd_length = length_in_dwords - 1;

        if (verbose_chunk) {
            dev_info(dev->dev, "--- Chunk %d ---\n", chunk_num);
            dev_info(dev->dev, "  Addr: 0x%llx (lo=0x%x hi=0x%x)\n",
                     (u64)current_addr,
                     lower_32_bits(current_addr),
                     upper_32_bits(current_addr));
            dev_info(dev->dev, "  Size: %zu bytes (%u DWORDs)\n",
                     chunk_size, length_in_dwords);
        }

        buf_offset = (size_t)(current_addr - dev->dma_handle);

        if (verbose_chunk) {
            /* Clear first 16 bytes of this chunk area to detect writes */
            u32 *chunk_buf = (u32 *)((u8 *)dev->dma_buf + buf_offset);
            chunk_buf[0] = 0xAAAAAAAA;
            chunk_buf[1] = 0xBBBBBBBB;
            chunk_buf[2] = 0xCCCCCCCC;
            chunk_buf[3] = 0xDDDDDDDD;
            dev_info(dev->dev, "  Pre-fill: %08x %08x %08x %08x\n",
                     chunk_buf[0], chunk_buf[1], chunk_buf[2], chunk_buf[3]);
        }

        /* Set end-of-chunk sentinels and poll host RAM for overwrite completion. */
        chunk_tail0 = (u32 *)((u8 *)dev->dma_buf + buf_offset + chunk_size - sizeof(u32));
        WRITE_ONCE(*chunk_tail0, tail_sentinel0);
        if (chunk_size >= 8) {
            chunk_tail1 = (u32 *)((u8 *)chunk_tail0 - sizeof(u32));
            WRITE_ONCE(*chunk_tail1, tail_sentinel1);
        }
        dma_wmb();

        /* Program DMA address registers */
        if (verbose_chunk)
            dev_info(dev->dev, "  Writing BAR1+0x110 = 0x%x\n", lower_32_bits(current_addr));
        fpga_dma_write_reg(dev, BAR1_DMA_L_ADDR, lower_32_bits(current_addr));

        if (verbose_chunk)
            dev_info(dev->dev, "  Writing BAR1+0x120 = 0x%x\n", upper_32_bits(current_addr));
        fpga_dma_write_reg(dev, BAR1_DMA_H_ADDR, upper_32_bits(current_addr));

        /* Program command register
         * Bits [9:0]: Length in DWORDs - 1
         * Bit [16]: 1 = 64-bit address mode
         * Bit [24]: 1 = Write (MWR) - FPGA writes frame data TO host memory
         */
        cmd_reg = (cmd_length & DMA_CMD_LEN_MASK) | DMA_CMD_64BIT_ADDR | DMA_CMD_WRITE;
        if (verbose_chunk) {
            dev_info(dev->dev, "  Writing BAR1+0x100 = 0x%08x (MWR, len=%u DW)\n",
                     cmd_reg, length_in_dwords);
        }
        fpga_dma_write_reg(dev, BAR1_DMA_CMD_REG, cmd_reg);
        fpga_dma_flush_posted_writes(dev);

        deadline = jiffies + msecs_to_jiffies(dma_timeout_ms);
        while ((READ_ONCE(*chunk_tail0) == tail_sentinel0) ||
               (chunk_tail1 && (READ_ONCE(*chunk_tail1) == tail_sentinel1))) {
            if (time_after(jiffies, deadline)) {
                dev_err(dev->dev,
                        "Chunk %d timeout waiting RAM overwrite (addr=0x%llx size=%zu)\n",
                        chunk_num, (u64)current_addr, chunk_size);
                return -ETIMEDOUT;
            }
            if (poll_sleep_cur_us <= 0) {
                cpu_relax();
                continue;
            }

            if (poll_sleep_max_us > 0 && poll_sleep_cur_us > poll_sleep_max_us)
                poll_sleep_cur_us = poll_sleep_max_us;

            usleep_range(poll_sleep_cur_us, poll_sleep_cur_us + 5);

            if (poll_backoff_polls > 0 && poll_sleep_max_us > 0 &&
                poll_sleep_cur_us < poll_sleep_max_us) {
                poll_count++;
                if (poll_count >= poll_backoff_polls) {
                    poll_count = 0;
                    poll_sleep_cur_us <<= 1;
                    if (poll_sleep_cur_us > poll_sleep_max_us)
                        poll_sleep_cur_us = poll_sleep_max_us;
                }
            }
        }
        dma_rmb();

        /* Optional extra pacing for stress testing */
        if (dma_chunk_delay_us >= 1000)
            usleep_range(dma_chunk_delay_us, dma_chunk_delay_us + 100);
        else if (dma_chunk_delay_us > 0)
            udelay(dma_chunk_delay_us);

        if (verbose_chunk) {
            /* Check if data arrived */
            u32 *chunk_buf = (u32 *)((u8 *)dev->dma_buf + buf_offset);
            dev_info(dev->dev, "  Post-DMA: %08x %08x %08x %08x\n",
                     chunk_buf[0], chunk_buf[1], chunk_buf[2], chunk_buf[3]);
            if (chunk_buf[0] == 0xAAAAAAAA)
                dev_info(dev->dev, "  >>> DATA NOT MODIFIED - FPGA did NOT write! <<<\n");
            else
                dev_info(dev->dev, "  >>> DATA CHANGED - FPGA wrote successfully! <<<\n");
        }

        /* Update remaining and address */
        remaining -= chunk_size;
        current_addr += chunk_size;
        chunk_num++;
    }

    if (dma_verbose) {
        dev_info(dev->dev, "  Processed chunks: %d\n", chunk_num);
        dev_info(dev->dev, "=== DMA Transfer Complete ===\n");
    }
    return ret;
}

/**
 * fpga_dma_open - Open the device file
 */
static int fpga_dma_open(struct inode *inode, struct file *file)
{
    struct fpga_dma_dev *dev;
    struct cdev *cdev = inode->i_cdev;

    dev = container_of(cdev, struct fpga_dma_dev, cdev);
    file->private_data = dev;

    dev_dbg(dev->dev, "Device opened\n");
    return 0;
}

/**
 * fpga_dma_release - Close the device file
 */
static int fpga_dma_release(struct inode *inode, struct file *file)
{
    struct fpga_dma_dev *dev = file->private_data;

    dev_dbg(dev->dev, "Device closed\n");
    return 0;
}

/**
 * fpga_dma_ioctl - Handle IOCTL commands
 */
static long fpga_dma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct fpga_dma_dev *dev = file->private_data;
    void __user *argp = (void __user *)arg;
    int ret = 0;

    switch (cmd) {
    case FPGA_DMA_GET_INFO: {
        struct fpga_info info = dev->info;

        /* Update link status */
        /* TODO: Read actual PCIe link status from capability registers */
        fpga_dma_normalize_info_layout(&info);

        if (copy_to_user(argp, &info, sizeof(info)))
            ret = -EFAULT;
        break;
    }

    case FPGA_DMA_READ_FRAME: {
        struct dma_transfer transfer;
        size_t size;
        size_t default_size;

        if (copy_from_user(&transfer, argp, sizeof(transfer))) {
            ret = -EFAULT;
            break;
        }

        fpga_dma_normalize_info_layout(&dev->info);
        default_size = fpga_dma_default_frame_size(&dev->info);

        /* Use default frame size if not specified */
        size = transfer.size > 0 ? transfer.size : default_size;

        if (size > dev->dma_buf_size) {
            dev_err(dev->dev, "Requested size %zu exceeds buffer size %zu\n",
                    size, dev->dma_buf_size);
            ret = -EINVAL;
            break;
        }

        /* Perform DMA transfer */
        mutex_lock(&dev->dma_lock);
        ret = fpga_dma_perform_transfer(dev, size);
        mutex_unlock(&dev->dma_lock);

        if (!ret) {
            /* Copy DMA buffer data to userspace */
            if (transfer.user_buf) {
                if (copy_to_user((void __user *)(unsigned long)transfer.user_buf,
                                 dev->dma_buf, size)) {
                    ret = -EFAULT;
                    break;
                }
            }
            transfer.result = 0;
            if (copy_to_user(argp, &transfer, sizeof(transfer)))
                ret = -EFAULT;
        }
        break;
    }

    case FPGA_DMA_MAP_BUFFER: {
        struct buffer_map map;

        if (copy_from_user(&map, argp, sizeof(map))) {
            ret = -EFAULT;
            break;
        }

        if (map.index != 0) {
            ret = -EINVAL;
            break;
        }

        map.size = dev->dma_buf_size;
        map.offset = 0;  /* Offset for mmap */

        if (copy_to_user(argp, &map, sizeof(map)))
            ret = -EFAULT;
        break;
    }

    default:
        dev_dbg(dev->dev, "Unknown ioctl cmd=0x%x\n", cmd);
        ret = -ENOTTY;
        break;
    }

    return ret;
}

/**
 * fpga_dma_mmap - Map DMA buffer to userspace
 */
static int fpga_dma_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct fpga_dma_dev *dev = file->private_data;
    size_t size = vma->vm_end - vma->vm_start;
    int ret;

    dev_dbg(dev->dev, "mmap requested: size=%zu\n", size);

    if (size > dev->dma_buf_size) {
        dev_err(dev->dev, "mmap size %zu exceeds DMA buffer size %zu\n",
                size, dev->dma_buf_size);
        return -EINVAL;
    }

    /* Map coherent DMA memory with the DMA API helper to avoid wrong PFN mapping. */
    ret = dma_mmap_coherent(&dev->pdev->dev, vma, dev->dma_buf, dev->dma_handle, size);
    if (ret) {
        dev_err(dev->dev, "dma_mmap_coherent failed: %d\n", ret);
        return ret;
    }

    return 0;
}

/**
 * fpga_dma_probe - PCI device probe callback
 */
static int fpga_dma_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct fpga_dma_dev *dev;
    int ret;
    u16 pcie_cmd;

    dev_info(&pdev->dev, "Probing FPGA DMA device\n");

    /* Allocate device structure */
    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->pdev = pdev;
    dev->dev = &pdev->dev;
    mutex_init(&dev->dma_lock);
    init_completion(&dev->dma_done);

    pci_set_drvdata(pdev, dev);

    /* Enable PCI device */
    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "Cannot enable PCI device\n");
        return ret;
    }

    /* Enable bus mastering */
    pci_read_config_word(pdev, PCI_COMMAND, &pcie_cmd);
    pcie_cmd |= PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
    pci_write_config_word(pdev, PCI_COMMAND, pcie_cmd);

    /* Set DMA mask - RK3568 supports 64-bit DMA */
    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (ret) {
        dev_warn(&pdev->dev, "Cannot set 64-bit DMA mask, trying 32-bit\n");
        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (ret) {
            dev_err(&pdev->dev, "Cannot set DMA mask\n");
            goto err_disable_device;
        }
    }

    /* Map BAR0 (64KB for PIO/data readback) */
    ret = pci_request_region(pdev, 0, DRIVER_NAME);
    if (ret) {
        dev_err(&pdev->dev, "Cannot request BAR0\n");
        goto err_disable_device;
    }

    dev->bar0 = pci_iomap(pdev, 0, 0);
    if (!dev->bar0) {
        dev_err(&pdev->dev, "Cannot map BAR0\n");
        ret = -ENOMEM;
        goto err_release_bar0;
    }
    dev->bar0_size = pci_resource_len(pdev, 0);
    dev_info(&pdev->dev, "BAR0 mapped at %p, size=%pa\n",
             dev->bar0, &dev->bar0_size);

    /* Map BAR1 (DMA control registers) */
    ret = pci_request_region(pdev, 1, DRIVER_NAME);
    if (ret) {
        dev_err(&pdev->dev, "Cannot request BAR1\n");
        goto err_iounmap_bar0;
    }

    dev->bar1 = pci_iomap(pdev, 1, 0);
    if (!dev->bar1) {
        dev_err(&pdev->dev, "Cannot map BAR1\n");
        ret = -ENOMEM;
        goto err_release_bar1;
    }
    dev->bar1_size = pci_resource_len(pdev, 1);
    dev_info(&pdev->dev, "BAR1 mapped at %p, size=%pa\n",
             dev->bar1, &dev->bar1_size);

    /* Allocate enough space for the largest supported frame format. */
    dev->dma_buf_size = PAGE_ALIGN((size_t)FPGA_FRAME_MAX_SIZE);
    dev->dma_buf = dma_alloc_coherent(&pdev->dev, dev->dma_buf_size,
                                      &dev->dma_handle, GFP_KERNEL);
    if (!dev->dma_buf) {
        dev_err(&pdev->dev, "Cannot allocate DMA buffer\n");
        ret = -ENOMEM;
        goto err_iounmap_bar1;
    }

    dev_info(&pdev->dev, "DMA buffer allocated: virt=%p phys=%pad size=%zu\n",
             dev->dma_buf, &dev->dma_handle, dev->dma_buf_size);

    /* Fill device info structure */
    dev->info.vendor_id = FPGA_PCI_VENDOR_ID;
    dev->info.device_id = FPGA_PCI_DEVICE_ID;
    dev->info.bar0_size = dev->bar0_size;
    dev->info.bar1_size = dev->bar1_size;
    dev->info.link_width = 2;  /* PCIe x2 */
    dev->info.link_speed = 2;  /* Gen2 */
    dev->info.frame_width = FPGA_FRAME_WIDTH;
    dev->info.frame_height = FPGA_FRAME_HEIGHT;
    dev->info.pixel_format = (dma_pixel_format == FPGA_PIXEL_FORMAT_BGR565)
        ? FPGA_PIXEL_FORMAT_BGR565
        : FPGA_PIXEL_FORMAT_BGRX8888;
    fpga_dma_normalize_info_layout(&dev->info);
    dev_info(&pdev->dev,
             "Frame layout: %ux%u format=%u bpp=%u stride=%u bytes\n",
             dev->info.frame_width,
             dev->info.frame_height,
             dev->info.pixel_format,
             dev->info.frame_bpp,
             dev->info.frame_stride);

    /* Create character device */
    if (major_num > 0) {
        dev->major = major_num;
        ret = register_chrdev_region(MKDEV(major_num, 0), 1, DRIVER_NAME);
    } else {
        ret = alloc_chrdev_region(&fpga_dma_dev_t, 0, 1, DRIVER_NAME);
        dev->major = MAJOR(fpga_dma_dev_t);
    }

    if (ret) {
        dev_err(&pdev->dev, "Cannot register chrdev region\n");
        goto err_free_dma;
    }

    cdev_init(&dev->cdev, &fpga_dma_fops);
    dev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&dev->cdev, MKDEV(dev->major, 0), 1);
    if (ret) {
        dev_err(&pdev->dev, "Cannot add cdev\n");
        goto err_unregister_chrdev;
    }

    /* Create device node in sysfs */
    dev->dev = device_create(fpga_dma_class, &pdev->dev,
                             MKDEV(dev->major, 0), dev,
                             FPGA_DMA_DEV_NAME);
    if (IS_ERR(dev->dev)) {
        ret = PTR_ERR(dev->dev);
        dev_err(&pdev->dev, "Cannot create device\n");
        goto err_cdev_del;
    }

    dev_info(&pdev->dev, "FPGA DMA driver loaded successfully\n");
    dev_info(&pdev->dev, "Device file: /dev/%s\n", FPGA_DMA_DEV_NAME);

    return 0;

err_cdev_del:
    cdev_del(&dev->cdev);
err_unregister_chrdev:
    unregister_chrdev_region(MKDEV(dev->major, 0), 1);
err_free_dma:
    dma_free_coherent(&pdev->dev, dev->dma_buf_size, dev->dma_buf, dev->dma_handle);
err_iounmap_bar1:
    pci_iounmap(pdev, dev->bar1);
err_release_bar1:
    pci_release_region(pdev, 1);
err_iounmap_bar0:
    pci_iounmap(pdev, dev->bar0);
err_release_bar0:
    pci_release_region(pdev, 0);
err_disable_device:
    pci_disable_device(pdev);

    return ret;
}

/**
 * fpga_dma_remove - PCI device remove callback
 */
static void fpga_dma_remove(struct pci_dev *pdev)
{
    struct fpga_dma_dev *dev = pci_get_drvdata(pdev);

    dev_info(&pdev->dev, "Removing FPGA DMA driver\n");

    /* Remove character device */
    device_destroy(fpga_dma_class, MKDEV(dev->major, 0));
    cdev_del(&dev->cdev);
    unregister_chrdev_region(MKDEV(dev->major, 0), 1);

    /* Free DMA buffer */
    dma_free_coherent(&pdev->dev, dev->dma_buf_size, dev->dma_buf, dev->dma_handle);

    /* Unmap and release BARs */
    pci_iounmap(pdev, dev->bar1);
    pci_release_region(pdev, 1);
    pci_iounmap(pdev, dev->bar0);
    pci_release_region(pdev, 0);

    /* Disable PCI device */
    pci_disable_device(pdev);

    dev_info(&pdev->dev, "FPGA DMA driver removed\n");
}

static struct pci_driver fpga_dma_driver = {
    .name = DRIVER_NAME,
    .id_table = fpga_dma_pci_tbl,
    .probe = fpga_dma_probe,
    .remove = fpga_dma_remove,
};

/**
 * fpga_dma_init - Module initialization
 */
static int __init fpga_dma_init(void)
{
    int ret;

    pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);

    /* Create device class */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    fpga_dma_class = class_create(DRIVER_NAME);
#else
    fpga_dma_class = class_create(THIS_MODULE, DRIVER_NAME);
#endif
    if (IS_ERR(fpga_dma_class)) {
        pr_err("Cannot create class\n");
        return PTR_ERR(fpga_dma_class);
    }

    /* Register PCI driver */
    ret = pci_register_driver(&fpga_dma_driver);
    if (ret) {
        pr_err("Cannot register PCI driver\n");
        class_destroy(fpga_dma_class);
        return ret;
    }

    pr_info("%s: driver loaded\n", DRIVER_NAME);
    return 0;
}

/**
 * fpga_dma_exit - Module cleanup
 */
static void __exit fpga_dma_exit(void)
{
    pr_info("%s: driver unloaded\n", DRIVER_NAME);
    pci_unregister_driver(&fpga_dma_driver);
    class_destroy(fpga_dma_class);
}

module_init(fpga_dma_init);
module_exit(fpga_dma_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION("1.0");
