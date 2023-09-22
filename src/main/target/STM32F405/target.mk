RX_SRC = \
    rx/cc2500_common.c \
    rx/cc2500_frsky_shared.c \
    rx/cc2500_frsky_d.c \
    rx/cc2500_frsky_x.c \
    rx/cc2500_sfhss.c \
    rx/cc2500_redpine.c \
    rx/a7105_flysky.c \
    rx/cyrf6936_spektrum.c \
    drivers/rx/rx_nrf24l01.c \
    rx/nrf24_custom.c

F405_TARGETS += $(TARGET)

CUSTOM_DEFAULTS_EXTENDED = yes


FEATURES       += VCP SDCARD_SDIO ONBOARDFLASH RX_SPI GPS TELEMETRY
TARGET_SRC = \
	$(addprefix drivers/accgyro/,$(notdir $(wildcard $(SRC_DIR)/drivers/accgyro/*.c))) \
	$(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
	$(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
	$(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
	drivers/max7456.c \
	drivers/vtx_rtc6705.c \
	drivers/vtx_rtc6705_soft_spi.c \
	$(RX_SRC)
