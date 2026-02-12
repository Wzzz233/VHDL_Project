# 1. 定义内核路径（必须确保容器内这个路径存在且是有效的内核源码）
# 如果你的 SDK 挂载在其他地方，请修改这里
KDIR := /home/hjf/SDK/kernel

# 2. 定义当前目录
PWD := $(shell pwd)

# 3. 定义交叉编译参数（这是关键！）
# ARCH 指定目标架构
ARCH := arm64
# CROSS_COMPILE 指定编译器前缀，容器内通常是 aarch64-linux-gnu-
CROSS_COMPILE := aarch64-linux-gnu-

# 4. 定义内核模块对象
obj-m := pcie_fpga_dma.o

# 5. 默认目标
all: module testapp

# 编译内核模块
module:
	# 关键修改：加入了 ARCH 和 CROSS_COMPILE 参数
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

# 编译测试程序
testapp: fpga_dma_test.c
	# 关键修改：直接使用交叉编译器，而不是默认的 cc
	$(CROSS_COMPILE)gcc -Wall -O2 -o fpga_dma_test fpga_dma_test.c

# 清理编译文件
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f fpga_dma_test
	rm -f *.raw
	rm -f Module.symvers modules.order *.mod.c *.o .*.cmd

# --- 以下目标仅在开发板本机编译时有用，交叉编译环境下建议注释掉 ---
# install:
# 	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
# 	depmod -a

# load:
# 	insmod pcie_fpga_dma.ko

# unload:
# 	rmmod pcie_fpga_dma || true
