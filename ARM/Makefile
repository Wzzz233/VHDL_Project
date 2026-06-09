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
OPENCV_CFLAGS ?= -I/usr/include/opencv4
OPENCV_LIBS ?= -lopencv_core -lopencv_imgproc

displayapp: fpga_hdmi_display.c pcie_fpga_dma.h
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_hdmi_display fpga_hdmi_display.c $(GST_CFLAGS) $(GST_LIBS)

RKNN_CFLAGS ?=
RKNN_LIBS ?= -lrknnrt
HOST_CC ?= gcc
HOST_CFLAGS ?= -Wall -Wextra -O2 -std=c11

lprapp: fpga_lpr_display.c ocr_decode.c ocr_decode.h lpr_special_route.h fpga_lpr_types.h pcie_fpga_dma.h
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_lpr_display fpga_lpr_display.c ocr_decode.c -pthread $(GST_CFLAGS) $(GST_LIBS) $(RKNN_CFLAGS) $(RKNN_LIBS) -lm

crosswalk-test: crosswalk_ped_cv_test.cpp pcie_fpga_dma.h
	$(CROSS_COMPILE)g++ -Wall -O2 -std=c++17 -o crosswalk_ped_cv_test crosswalk_ped_cv_test.cpp -pthread $(GST_CFLAGS) $(OPENCV_CFLAGS) $(GST_LIBS) $(OPENCV_LIBS) $(RKNN_CFLAGS) $(RKNN_LIBS) -lm

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
	rm -f crosswalk_ped_cv_test
	rm -f test_ocr_decode
	rm -f test_ocr_layout
	rm -f test_lpr_special_route
	rm -f test_pose_decoder
	rm -f *.raw
	rm -f Module.symvers modules.order *.mod.c *.o .*.cmd
