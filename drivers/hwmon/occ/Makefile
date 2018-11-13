occ-hwmon-objs := common.o

ifeq ($(CONFIG_SENSORS_OCC_P9_SBE), y)
occ-hwmon-objs += p9_sbe.o
endif

ifeq ($(CONFIG_SENSORS_OCC_P8_I2C), y)
occ-hwmon-objs += p8_i2c.o
endif

obj-$(CONFIG_SENSORS_OCC) += occ-hwmon.o
