#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/simplelink_msp432e4_sdk_4_20_00_12/source;C:/ti/simplelink_msp432e4_sdk_4_20_00_12/kernel/tirtos/packages;C:/Users/PC/Desktop/LSE/LSE2/sw/RTOS_v1/.config
override XDCROOT = C:/ti/xdctools_3_61_01_25_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/simplelink_msp432e4_sdk_4_20_00_12/source;C:/ti/simplelink_msp432e4_sdk_4_20_00_12/kernel/tirtos/packages;C:/Users/PC/Desktop/LSE/LSE2/sw/RTOS_v1/.config;C:/ti/xdctools_3_61_01_25_core/packages;..
HOSTOS = Windows
endif
