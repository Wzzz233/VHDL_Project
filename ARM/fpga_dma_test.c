// SPDX-License-Identifier: GPL-2.0
/*
 * FPGA DMA Test Application
 *
 * Userspace test program for the FPGA DMA driver
 * Tests: device info query, DMA frame transfer, data validation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>

#include "pcie_fpga_dma.h"

#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"

/* Global variables for signal handling */
static int g_device_fd = -1;
static volatile sig_atomic_t g_running = 1;

enum ppm_mode {
    PPM_MODE_RGB565 = 0,
    PPM_MODE_BGR565,
    PPM_MODE_RGB565_SWAP16,
    PPM_MODE_BGR565_SWAP16,
};

/**
 * print_color - Print colored output to terminal
 */
static void print_color(const char *color, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    fprintf(stderr, "%s", color);
    vfprintf(stderr, format, args);
    fprintf(stderr, "%s\n", COLOR_RESET);
    va_end(args);
}

/**
 * signal_handler - Handle Ctrl+C gracefully
 */
static void signal_handler(int signo)
{
    if (signo == SIGINT || signo == SIGTERM) {
        print_color(COLOR_YELLOW, "\nReceived signal, exiting...");
        g_running = 0;
    }
}

/**
 * print_usage - Print usage information
 */
static void print_usage(const char *progname)
{
    printf("Usage: %s [OPTIONS]\n", progname);
    printf("\nOptions:\n");
    printf("  --info                 Show FPGA device information\n");
    printf("  --read <filename>      Read frame from FPGA and save to file\n");
    printf("  --continuous           Read frames continuously (Ctrl+C to stop)\n");
    printf("  --count <num>          Number of frames to read (default: 1)\n");
    printf("  --verify               Verify frame data (check for zeros)\n");
    printf("  --dump <bytes>         Dump first N bytes of frame (hex)\n");
    printf("  --save-ppm <filename>  Save frame as PPM image\n");
    printf("  --ppm-mode <mode>      PPM decode mode: rgb565|bgr565|rgb565-swap|bgr565-swap\n");
    printf("  --mmap                 Test mmap buffer access\n");
    printf("  --help                 Show this help message\n");
    printf("\nExamples:\n");
    printf("  %s --info\n", progname);
    printf("  %s --read frame.raw\n", progname);
    printf("  %s --read frame.raw --verify --dump 64\n", progname);
    printf("  %s --read frame.raw --save-ppm frame.ppm\n", progname);
    printf("  %s --continuous --count 100\n", progname);
}

/**
 * print_fpga_info - Display FPGA device information
 */
static int print_fpga_info(int fd)
{
    struct fpga_info info;
    int ret;

    ret = ioctl(fd, FPGA_DMA_GET_INFO, &info);
    if (ret < 0) {
        print_color(COLOR_RED, "Failed to get FPGA info: %s", strerror(errno));
        return -1;
    }

    print_color(COLOR_BLUE, "=== FPGA Device Information ===");
    printf("  Vendor ID:             0x%04x\n", info.vendor_id);
    printf("  Device ID:             0x%04x\n", info.device_id);
    printf("  BAR0 Size:             %u bytes\n", info.bar0_size);
    printf("  BAR1 Size:             %u bytes\n", info.bar1_size);
    printf("  PCIe Link Width:       x%u\n", info.link_width);
    printf("  PCIe Link Speed:       Gen%u\n", info.link_speed);
    printf("  Frame Width:           %u pixels\n", info.frame_width);
    printf("  Frame Height:          %u pixels\n", info.frame_height);
    printf("  Frame Bytes/Pixel:     %u\n", info.frame_bpp);
    printf("  Frame Size:            %u bytes (%.2f MB)\n",
           info.frame_width * info.frame_height * info.frame_bpp,
           (float)(info.frame_width * info.frame_height * info.frame_bpp) / (1024 * 1024));
    print_color(COLOR_BLUE, "===============================");

    return 0;
}

/**
 * read_frame - Read a frame from FPGA via DMA
 */
static int read_frame(int fd, void *buffer, size_t size)
{
    struct dma_transfer transfer;
    int ret;

    memset(&transfer, 0, sizeof(transfer));
    transfer.size = size;
    transfer.flags = 0;
    transfer.result = -1;
    transfer.user_buf = (uint64_t)(unsigned long)buffer;

    ret = ioctl(fd, FPGA_DMA_READ_FRAME, &transfer);
    if (ret < 0) {
        print_color(COLOR_RED, "DMA read failed: %s", strerror(errno));
        return -1;
    }

    if (transfer.result != 0) {
        print_color(COLOR_RED, "DMA read failed with result: %u", transfer.result);
        return -1;
    }

    return 0;
}

/**
 * verify_frame - Check if frame data is valid (not all zeros)
 */
static int verify_frame(const void *buffer, size_t size)
{
    const uint8_t *data = buffer;
    size_t i;
    size_t non_zero_count = 0;

    for (i = 0; i < size; i++) {
        if (data[i] != 0) {
            non_zero_count++;
        }
    }

    print_color(COLOR_BLUE, "Data verification: %zu/%zu bytes are non-zero (%.2f%%)",
               non_zero_count, size, (float)non_zero_count * 100 / size);

    if (non_zero_count == 0) {
        print_color(COLOR_RED, "Warning: Frame data is all zeros!");
        return -1;
    } else if (non_zero_count < size / 10) {
        print_color(COLOR_YELLOW, "Warning: Less than 10%% of data is non-zero");
    } else {
        print_color(COLOR_GREEN, "Frame data looks valid");
    }

    return 0;
}

/**
 * dump_data - Dump hex data to console
 */
static void dump_data(const void *buffer, size_t size)
{
    const uint8_t *data = buffer;
    size_t i;

    print_color(COLOR_BLUE, "First %zu bytes of frame:", size);
    for (i = 0; i < size && i < 256; i++) {
        if (i % 16 == 0) {
            printf("%04zx: ", i);
        }
        printf("%02x ", data[i]);
        if (i % 16 == 15) {
            printf("\n");
        }
    }
    if (i % 16 != 0) {
        printf("\n");
    }
}

/**
 * save_frame - Save frame data to file
 */
static int save_frame(const char *filename, const void *buffer, size_t size)
{
    FILE *fp;

    fp = fopen(filename, "wb");
    if (!fp) {
        print_color(COLOR_RED, "Failed to open file '%s': %s", filename, strerror(errno));
        return -1;
    }

    if (fwrite(buffer, 1, size, fp) != size) {
        print_color(COLOR_RED, "Failed to write frame data: %s", strerror(errno));
        fclose(fp);
        return -1;
    }

    fclose(fp);
    print_color(COLOR_GREEN, "Frame saved to '%s' (%zu bytes)", filename, size);
    return 0;
}

/**
 * save_ppm_rgb565 - Convert RGB565 buffer to PPM (P6)
 */
static int save_ppm_rgb565(const char *filename, const void *buffer,
                           uint32_t width, uint32_t height, enum ppm_mode mode)
{
    const uint8_t *src = (const uint8_t *)buffer;
    uint32_t pixel_count = width * height;
    FILE *fp;
    uint32_t i;

    fp = fopen(filename, "wb");
    if (!fp) {
        print_color(COLOR_RED, "Failed to open PPM file '%s': %s", filename, strerror(errno));
        return -1;
    }

    fprintf(fp, "P6\n%u %u\n255\n", width, height);

    for (i = 0; i < pixel_count; i++) {
        uint8_t lo = src[i * 2];
        uint8_t hi = src[i * 2 + 1];
        uint8_t r5, g6, b5;
        uint8_t rgb[3];
        uint16_t pix;

        if (mode == PPM_MODE_RGB565_SWAP16 || mode == PPM_MODE_BGR565_SWAP16) {
            uint8_t tmp = lo;
            lo = hi;
            hi = tmp;
        }

        pix = (uint16_t)lo | ((uint16_t)hi << 8);

        if (mode == PPM_MODE_RGB565 || mode == PPM_MODE_RGB565_SWAP16) {
            r5 = (pix >> 11) & 0x1F;
            g6 = (pix >> 5) & 0x3F;
            b5 = pix & 0x1F;
        } else {
            r5 = pix & 0x1F;
            g6 = (pix >> 5) & 0x3F;
            b5 = (pix >> 11) & 0x1F;
        }

        rgb[0] = (uint8_t)((r5 << 3) | (r5 >> 2));
        rgb[1] = (uint8_t)((g6 << 2) | (g6 >> 4));
        rgb[2] = (uint8_t)((b5 << 3) | (b5 >> 2));

        if (fwrite(rgb, 1, sizeof(rgb), fp) != sizeof(rgb)) {
            print_color(COLOR_RED, "Failed to write PPM data: %s", strerror(errno));
            fclose(fp);
            return -1;
        }
    }

    fclose(fp);
    print_color(COLOR_GREEN, "PPM image saved to '%s' (%ux%u)", filename, width, height);
    return 0;
}

/**
 * test_mmap - Test mmap buffer access
 */
static int test_mmap(int fd)
{
    struct buffer_map map;
    void *mapped;
    int ret;

    print_color(COLOR_BLUE, "Testing mmap buffer access...");

    /* Get buffer info */
    memset(&map, 0, sizeof(map));
    map.index = 0;

    ret = ioctl(fd, FPGA_DMA_MAP_BUFFER, &map);
    if (ret < 0) {
        print_color(COLOR_RED, "Failed to get buffer info: %s", strerror(errno));
        return -1;
    }

    printf("Buffer size: %u bytes\n", map.size);

    /* Map buffer to userspace */
    mapped = mmap(NULL, map.size, PROT_READ, MAP_SHARED, fd, 0);
    if (mapped == MAP_FAILED) {
        print_color(COLOR_RED, "mmap failed: %s", strerror(errno));
        return -1;
    }

    print_color(COLOR_GREEN, "Buffer mapped at %p", mapped);

    /* Trigger a DMA read */
    ret = read_frame(fd, NULL, FPGA_FRAME_SIZE);
    if (ret < 0) {
        munmap(mapped, map.size);
        return -1;
    }

    /* Access the mapped data */
    printf("First 16 bytes via mmap: ");
    for (ret = 0; ret < 16 && ret < map.size; ret++) {
        printf("%02x ", *((uint8_t*)mapped + ret));
    }
    printf("\n");

    /* Cleanup */
    munmap(mapped, map.size);
    print_color(COLOR_GREEN, "mmap test completed");
    return 0;
}

/**
 * main - Main entry point
 */
int main(int argc, char *argv[])
{
    const char *device_file = "/dev/" FPGA_DMA_DEV_NAME;
    const char *output_file = NULL;
    const char *ppm_file = NULL;
    enum ppm_mode ppm_mode = PPM_MODE_BGR565;
    int do_info = 0;
    int do_read = 0;
    int do_continuous = 0;
    int do_verify = 0;
    int do_mmap = 0;
    int dump_bytes = 0;
    int frame_count = 1;
    int ret = 0;
    int i;

    /* Parse command line arguments */
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--info") == 0) {
            do_info = 1;
        } else if (strcmp(argv[i], "--read") == 0) {
            do_read = 1;
            if (i + 1 < argc) {
                output_file = argv[++i];
            } else {
                fprintf(stderr, "Error: --read requires filename argument\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--continuous") == 0) {
            do_continuous = 1;
            do_read = 1;
            if (!output_file) {
                output_file = "frame.raw";
            }
        } else if (strcmp(argv[i], "--count") == 0) {
            if (i + 1 < argc) {
                frame_count = atoi(argv[++i]);
            } else {
                fprintf(stderr, "Error: --count requires number argument\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--verify") == 0) {
            do_verify = 1;
        } else if (strcmp(argv[i], "--dump") == 0) {
            if (i + 1 < argc) {
                dump_bytes = atoi(argv[++i]);
            } else {
                fprintf(stderr, "Error: --dump requires byte count argument\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--save-ppm") == 0) {
            do_read = 1;
            if (i + 1 < argc) {
                ppm_file = argv[++i];
            } else {
                fprintf(stderr, "Error: --save-ppm requires filename argument\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--ppm-mode") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: --ppm-mode requires mode argument\n");
                return 1;
            }
            i++;
            if (strcmp(argv[i], "rgb565") == 0) {
                ppm_mode = PPM_MODE_RGB565;
            } else if (strcmp(argv[i], "bgr565") == 0) {
                ppm_mode = PPM_MODE_BGR565;
            } else if (strcmp(argv[i], "rgb565-swap") == 0) {
                ppm_mode = PPM_MODE_RGB565_SWAP16;
            } else if (strcmp(argv[i], "bgr565-swap") == 0) {
                ppm_mode = PPM_MODE_BGR565_SWAP16;
            } else {
                fprintf(stderr, "Error: invalid --ppm-mode '%s'\n", argv[i]);
                return 1;
            }
        } else if (strcmp(argv[i], "--mmap") == 0) {
            do_mmap = 1;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    /* Setup signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Open device */
    print_color(COLOR_BLUE, "Opening device: %s", device_file);
    g_device_fd = open(device_file, O_RDWR);
    if (g_device_fd < 0) {
        print_color(COLOR_RED, "Failed to open device '%s': %s", device_file, strerror(errno));
        print_color(COLOR_YELLOW, "Make sure the kernel module is loaded:");
        print_color(COLOR_YELLOW, "  sudo insmod pcie_fpga_dma.ko");
        return 1;
    }

    /* Show device info */
    if (do_info || do_read) {
        ret = print_fpga_info(g_device_fd);
        if (ret < 0) {
            close(g_device_fd);
            return 1;
        }
    }

    /* Test mmap */
    if (do_mmap) {
        ret = test_mmap(g_device_fd);
        if (ret < 0) {
            close(g_device_fd);
            return 1;
        }
    }

    /* Read frame(s) */
    if (do_read) {
        uint8_t *buffer = NULL;

        /* Allocate buffer for frame data */
        buffer = malloc(FPGA_FRAME_SIZE);
        if (!buffer) {
            print_color(COLOR_RED, "Failed to allocate buffer");
            close(g_device_fd);
            return 1;
        }

        print_color(COLOR_BLUE, "Reading %d frame(s)...", do_continuous ? frame_count : 1);

        /* Read frames */
        for (i = 0; i < frame_count && g_running; i++) {
            struct timespec start, end;
            double elapsed;

            clock_gettime(CLOCK_MONOTONIC, &start);

            ret = read_frame(g_device_fd, buffer, FPGA_FRAME_SIZE);
            if (ret < 0) {
                free(buffer);
                close(g_device_fd);
                return 1;
            }

            clock_gettime(CLOCK_MONOTONIC, &end);
            elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;

            print_color(COLOR_GREEN, "Frame %d read successfully in %.3f seconds (%.2f MB/s)",
                       i + 1, elapsed, FPGA_FRAME_SIZE / (elapsed * 1024 * 1024));

            /* Verify data */
            if (do_verify) {
                verify_frame(buffer, FPGA_FRAME_SIZE);
            }

            /* Dump data */
            if (dump_bytes > 0) {
                size_t dump_size = (dump_bytes < FPGA_FRAME_SIZE) ? dump_bytes : FPGA_FRAME_SIZE;
                dump_data(buffer, dump_size);
            }

            /* Save to file */
            if (output_file) {
                char filename[256];

                if (do_continuous && frame_count > 1) {
                    snprintf(filename, sizeof(filename), "%s_%04d.raw", output_file, i);
                } else {
                    snprintf(filename, sizeof(filename), "%s", output_file);
                }

                save_frame(filename, buffer, FPGA_FRAME_SIZE);
            }

            if (ppm_file) {
                char ppm_name[256];

                if (do_continuous && frame_count > 1) {
                    snprintf(ppm_name, sizeof(ppm_name), "%s_%04d.ppm", ppm_file, i);
                } else {
                    snprintf(ppm_name, sizeof(ppm_name), "%s", ppm_file);
                }

                if (save_ppm_rgb565(ppm_name, buffer, FPGA_FRAME_WIDTH, FPGA_FRAME_HEIGHT, ppm_mode) < 0) {
                    free(buffer);
                    close(g_device_fd);
                    return 1;
                }
            }

            if (do_continuous && i < frame_count - 1 && g_running) {
                usleep(33000);  /* ~30 FPS */
            }
        }

        free(buffer);
    }

    /* Cleanup */
    close(g_device_fd);
    print_color(COLOR_GREEN, "Test completed successfully");

    return 0;
}
