# Kernel module + userspace tools
KDIR := /home/hjf/SDK/kernel
PWD := $(shell pwd)

ARCH := arm64
CROSS_COMPILE := aarch64-linux-gnu-

obj-m := pcie_fpga_dma.o

all: module testapp

module:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

testapp: fpga_dma_test.c pcie_fpga_dma.h
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_dma_test fpga_dma_test.c

GST_PKGS := gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 libdrm
GST_CFLAGS ?= $(shell pkg-config --cflags $(GST_PKGS))
GST_LIBS ?= $(shell pkg-config --libs $(GST_PKGS))

displayapp: fpga_hdmi_display.c pcie_fpga_dma.h
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_hdmi_display fpga_hdmi_display.c $(GST_CFLAGS) $(GST_LIBS)

RKNN_CFLAGS ?=
RKNN_LIBS ?= -lrknnrt
HOST_CC ?= gcc
HOST_CFLAGS ?= -Wall -Wextra -O2 -std=c11

lprapp: fpga_lpr_display.c ocr_decode.c ocr_decode.h lpr_special_route.h fpga_lpr_types.h pcie_fpga_dma.h
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_lpr_display fpga_lpr_display.c ocr_decode.c -pthread $(GST_CFLAGS) $(GST_LIBS) $(RKNN_CFLAGS) $(RKNN_LIBS) -lm

pplcnet-green-live: pplcnet_green_live.c ocr_decode.c ocr_decode.h pcie_fpga_dma.h
	$(CROSS_COMPILE)gcc -Wall -O2 -o pplcnet_green_live pplcnet_green_live.c ocr_decode.c $(GST_CFLAGS) $(GST_LIBS) $(RKNN_CFLAGS) $(RKNN_LIBS) -lm

pplcnet-bg-live: pplcnet_bg_live.c ocr_decode.c ocr_decode.h pcie_fpga_dma.h
	$(CROSS_COMPILE)gcc -Wall -O2 -o pplcnet_bg_live pplcnet_bg_live.c ocr_decode.c $(GST_CFLAGS) $(GST_LIBS) $(RKNN_CFLAGS) $(RKNN_LIBS) -lm

# Three-expert (blue/green/police) PPLCNet live driver. Built from
# modular sources under lpr_live/ for maintainability:
#   lpr_common, lpr_dma, lpr_detector, lpr_warp, lpr_color,
#   lpr_ocr, lpr_display, lpr_infer
# Police OCR is optional (--ocr-police-model); if omitted the driver
# silently degrades to blue+green and behaves like pplcnet-bg-live.
#
# Cross-compile environment is provided by the rknn:dev-snapshot Docker
# image with /home/wzzz/RK3568J_SDK mounted at /home/hjf/SDK. The pkg-config
# variables are pointed at the aarch64 sysroot inside the SDK so libdrm /
# gstreamer headers come from there instead of the host x86_64 paths.
#
# To build:
#   docker run --rm -u root \
#     -v /home/wzzz/ARM:/app \
#     -v /home/wzzz/RK3568J_SDK:/home/hjf/SDK \
#     -w /app \
#     cdc81f835218 \
#     make pplcnet-bgp-live
SDK_SYSROOT  ?= /home/hjf/SDK/debian/binary
RKNN_RUNTIME_DIR ?= /home/hjf/SDK/external/rknpu2/runtime/Linux/librknn_api
RKNN_INC_DIR ?= $(RKNN_RUNTIME_DIR)/include
RKNN_LIB_DIR ?= $(RKNN_RUNTIME_DIR)/aarch64

LPR_LIVE_SRCS := \
	lpr_live/lpr_common.c \
	lpr_live/lpr_dma.c \
	lpr_live/lpr_detector.c \
	lpr_live/lpr_warp.c \
	lpr_live/lpr_color.c \
	lpr_live/lpr_ocr.c \
	lpr_live/lpr_display.c \
	lpr_live/lpr_infer.c
LPR_LIVE_HDRS := \
	lpr_live/lpr_common.h \
	lpr_live/lpr_dma.h \
	lpr_live/lpr_detector.h \
	lpr_live/lpr_warp.h \
	lpr_live/lpr_color.h \
	lpr_live/lpr_ocr.h \
	lpr_live/lpr_display.h \
	lpr_live/lpr_infer.h

# Use the SDK's aarch64 pkg-config so headers/libs come from the rootfs
# instead of host paths. We still inherit GST_PKGS from above.
BGP_PKG_CONFIG_PATH := $(SDK_SYSROOT)/usr/lib/aarch64-linux-gnu/pkgconfig:$(SDK_SYSROOT)/usr/share/pkgconfig
BGP_GST_CFLAGS := $(shell PKG_CONFIG_SYSROOT_DIR=$(SDK_SYSROOT) PKG_CONFIG_LIBDIR=$(BGP_PKG_CONFIG_PATH) PKG_CONFIG_PATH=$(BGP_PKG_CONFIG_PATH) pkg-config --cflags $(GST_PKGS))
BGP_GST_LIBS   := $(shell PKG_CONFIG_SYSROOT_DIR=$(SDK_SYSROOT) PKG_CONFIG_LIBDIR=$(BGP_PKG_CONFIG_PATH) PKG_CONFIG_PATH=$(BGP_PKG_CONFIG_PATH) pkg-config --libs $(GST_PKGS))

pplcnet-bgp-live: pplcnet_bgp_live.c $(LPR_LIVE_SRCS) $(LPR_LIVE_HDRS) ocr_decode.c ocr_decode.h pcie_fpga_dma.h
	$(CROSS_COMPILE)gcc -Wall -Wextra -O2 \
		--sysroot=$(SDK_SYSROOT) \
		-Ilpr_live -I. -I$(RKNN_INC_DIR) \
		-o pplcnet_bgp_live \
		pplcnet_bgp_live.c $(LPR_LIVE_SRCS) ocr_decode.c \
		-pthread $(BGP_GST_CFLAGS) $(BGP_GST_LIBS) \
		-L$(RKNN_LIB_DIR) -lrknnrt -lm

# Standalone PPM-input PPLCNet RKNN tester. Cross-compiled for the board (uses libplcnetrt).
# Run on board:  ./test_pplcnet_green model.rknn keys.txt image.ppm [...] | @list.txt
pplcnet-test: test_pplcnet_green.c ocr_decode.c ocr_decode.h
	$(CROSS_COMPILE)gcc -Wall -O2 -o test_pplcnet_green test_pplcnet_green.c ocr_decode.c $(RKNN_CFLAGS) $(RKNN_LIBS) -lm

decode-test: test_ocr_decode.c ocr_decode.c ocr_decode.h
	$(HOST_CC) $(HOST_CFLAGS) -o test_ocr_decode test_ocr_decode.c ocr_decode.c -lm

ocr-layout-test: test_ocr_layout.c
	$(HOST_CC) $(HOST_CFLAGS) -o test_ocr_layout test_ocr_layout.c

special-route-test: test_lpr_special_route.c lpr_special_route.h
	$(HOST_CC) $(HOST_CFLAGS) -o test_lpr_special_route test_lpr_special_route.c

pose-decoder-test: test_pose_decoder.c
	$(HOST_CC) $(HOST_CFLAGS) -o test_pose_decoder test_pose_decoder.c -lm

test: decode-test ocr-layout-test special-route-test pose-decoder-test
	./test_ocr_decode
	./test_ocr_layout
	./test_lpr_special_route
	./test_pose_decoder

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f fpga_dma_test
	rm -f fpga_hdmi_display
	rm -f fpga_lpr_display
	rm -f pplcnet_green_live
	rm -f pplcnet_bg_live
	rm -f pplcnet_bgp_live
	rm -f test_pplcnet_green
	rm -f test_ocr_decode
	rm -f test_ocr_layout
	rm -f test_lpr_special_route
	rm -f test_pose_decoder
	rm -f *.raw
	rm -f Module.symvers modules.order *.mod.c *.o .*.cmd
