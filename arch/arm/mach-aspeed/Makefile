#
# Makefile for the linux kernel.
#

# Common support (must be linked before board specific support)

obj-y				+= ast.o

obj-$(CONFIG_AST1070_UART_DMA)	+= ast1070-uart-dma.o

obj-$(CONFIG_AST_UART_SDMA)	+= ast-uart-sdma.o
