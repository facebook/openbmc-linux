#
# Makefile for the linux kernel.
#

obj-y 			+= irq.o timer.o devs.o ast-scu.o ast-sdmc.o

obj-$(CONFIG_ARCH_AST1070) += ast1070_irq.o ast1070-scu.o ast1070-uart-dma.o
obj-$(CONFIG_AST_I2C_SLAVE_MODE) += i2c-slave-eeprom.o

#obj-n   := dummy.o
#platform
obj-y			+= dev-uart.o dev-vuart.o dev-wdt.o dev-rtc.o dev-gpio.o dev-sgpio.o  dev-cuart.o

#Storage
obj-y 	                += dev-nor.o dev-nand.o dev-sdhci.o

#bus
obj-y                   += dev-i2c.o dev-ci2c.o dev-spi.o dev-ehci.o dev-uhci.o dev-clpc.o dev-peci.o dev-kcs.o dev-mbx.o dev-snoop.o dev-lpc.o

#dev
#obj-y			+= dev-udc11.o
#net
#obj-y			+= dev-eth.o

#hwmon
obj-y                   += dev-pwm-fan.o dev-adc.o

#video
obj-y			+= dev-fb.o dev-video.o
#obj-m                  :=
#obj-n                  :=
#obj-                   :=

#usbdevicegadgetthing
obj-y			+= dev-virthub.o
