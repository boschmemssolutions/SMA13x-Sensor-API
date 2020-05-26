#
# Makefile for Bosch sensor driver.
#

ifneq ($(CONFIG_BOSCH_DRIVER_LOG_FUNC),n)
	EXTRA_CFLAGS += -DBOSCH_DRIVER_LOG_FUNC
endif


obj-$(CONFIG_SMA13X_DRIVER)    += sma13x-module.o
sma13x-module-objs := sma13x_log.o sma13x.o sma13x_driver.o

ifeq ($(CONFIG_SMA13X_I2C),y)
	sma13x-module-objs += sma13x_i2c_driver.o
else
	sma13x-module-objs += sma13x_spi_driver.o
endif
