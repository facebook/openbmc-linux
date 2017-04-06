#
# Makefile for the linux kernel.
#

obj-y 			+= fiq.o irq.o timer.o devs.o ast-scu.o ast-sdmc.o ast-ahbc.o i2c-irq.o ast-pciarbiter.o sdhci-irq.o

obj-$(CONFIG_AST_RFX)	+= egfx-irq.o

obj-$(CONFIG_ARCH_AST1070) += ast1070-irq.o ast1070-scu.o ast1070-i2c-irq.o dev-ci2c.o dev-cuart.o dev-clpc.o

obj-$(CONFIG_AST_I2C_SLAVE_MODE) += i2c-slave-eeprom.o

ifeq ($(CONFIG_PCI),y)
obj-$(CONFIG_PCIE_AST)	+= ast-p2x.o
endif

obj-$(CONFIG_AST2400_BMC) += ast2400-irq.o ast2400-scu.o dev-ast2400-uart.o #dev-ast2400-i2c.o

#obj-n   := dummy.o
#platform
obj-y			+= dev-vuart.o dev-wdt.o dev-rtc.o dev-gpio.o dev-sgpio.o dev-crypto.o

#Storage
obj-y 	                += dev-nor.o dev-nand.o dev-sdhci.o

#bus
obj-y                   += dev-ehci.o dev-uhci.o dev-lpc.o dev-espi.o dev-peci.o dev-kcs.o dev-bt.o dev-snoop.o dev-mbx.o dev-pcie.o dev-jtag.o dev-mctp.o dev-xdma.o

PLATFB := 0
ifeq ($(CONFIG_CMM),y)
  obj-y += dev-uart.o dev-i2c-cmm.o dev-i2c-common.o
  PLATFB = 1
endif

ifeq ($(CONFIG_WEDGE), y)
  obj-y += dev-uart.o dev-i2c-wedge.o dev-i2c-common.o
  PLATFB = 1
endif

ifeq ($(CONFIG_FBTP), y)
  obj-y += dev-uart-fbtp.o dev-i2c.o
  PLATFB = 1
endif

ifeq ($(CONFIG_FBTTN), y)
  obj-y += dev-uart.o dev-i2c.o
  PLATFB = 1
endif

ifeq ($(CONFIG_LIGHTNING), y)
  obj-y += dev-uart.o  dev-i2c.o
  PLATFB = 1
endif

ifeq ($(CONFIG_FBY2),y)
  obj-y += dev-uart.o dev-i2c-fby2.o
  PLATFB = 1
endif

ifneq ($(PLATFB), 1)
  obj-y += dev-uart.o dev-spi.o dev-i2c.o
else
  obj-y += dev-spi-fb.o
endif

#dev
obj-y			+= dev-udc11.o dev-hid.o dev-udc20.o
#net
obj-y			+= dev-eth.o

#hwmon
obj-y                   += dev-pwm-fan.o dev-adc.o

#video
obj-y			+= dev-fb.o dev-video.o dev-h264.o dev-formatter.o dev-rfx.o

obj-y			+= dev-rng.o

# USB Virtual Hub
obj-y			+= dev-vhub.o
