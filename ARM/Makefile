# Kernel module + userspace tools
KDIR := /home/hjf/SDK/kernel
PWD := $(shell pwd)

ARCH := arm64
CROSS_COMPILE := aarch64-linux-gnu-

obj-m := pcie_fpga_dma.o

all: module testapp

module:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

testapp: fpga_dma_test.c
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_dma_test fpga_dma_test.c

GST_PKGS := gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 libdrm
GST_CFLAGS ?= $(shell pkg-config --cflags $(GST_PKGS))
GST_LIBS ?= $(shell pkg-config --libs $(GST_PKGS))

displayapp: fpga_hdmi_display.c
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_hdmi_display fpga_hdmi_display.c $(GST_CFLAGS) $(GST_LIBS)

RKNN_CFLAGS ?=
RKNN_LIBS ?= -lrknnrt

lprapp: fpga_lpr_display.c
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_lpr_display fpga_lpr_display.c -pthread $(GST_CFLAGS) $(GST_LIBS) $(RKNN_CFLAGS) $(RKNN_LIBS) -lm

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f fpga_dma_test
	rm -f fpga_hdmi_display
	rm -f fpga_lpr_display
	rm -f *.raw
	rm -f Module.symvers modules.order *.mod.c *.o .*.cmd
