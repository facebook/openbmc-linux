#
# Makefile for the linux kernel.
#

# Common support (must be linked before board specific support)

# Specific board support
obj-$(CONFIG_ARCH_AST1100) += ast1100.o
obj-$(CONFIG_ARCH_AST2100) += ast2100.o
obj-$(CONFIG_ARCH_AST2300) += ast2300.o gpio.o ast-lpc.o
obj-$(CONFIG_ARCH_AST2400) += ast2400.o gpio.o ast-lpc.o ast-lpc_plus.o
obj-$(CONFIG_ARCH_AST2500) += ast2500.o ast-mctp.o
#PC Ext
obj-$(CONFIG_ARCH_AST1510) += ast1510.o gpio.o
obj-$(CONFIG_ARCH_AST1520) += ast1520.o ast-mctp.o

#RemoteFx Zero client
obj-$(CONFIG_ARCH_AST3100) += ast3100.o gpio.o
obj-$(CONFIG_ARCH_AST3200) += ast3200.o ast-mctp.o

#BMC Comapnion Controller
obj-$(CONFIG_ARCH_AST1070) += ast1070.o
