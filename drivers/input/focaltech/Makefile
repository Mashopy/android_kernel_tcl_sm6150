# Early-logging level for ff_hal, ff_ta and driver as well. Note that this
# logging level only takes effect before ff_config module is loaded.
#
# Options:
# 0 - FF_LOG_LEVEL_ALL, All
# 1 - FF_LOG_LEVEL_VBS, Verbose
# 2 - FF_LOG_LEVEL_DBG, Debug
# 3 - FF_LOG_LEVEL_INF, Info
# 4 - FF_LOG_LEVEL_WRN, Warning
# 5 - FF_LOG_LEVEL_ERR, Error
#ccflags-y += -D__FF_EARLY_LOG_LEVEL=3

# Source files.
obj-$(CONFIG_FOCALTECH_FINGERPRINT) := focaltech_fp.o
focaltech_fp-objs     := ff_ctl.o ff_spi.o plat-msm8953.o










