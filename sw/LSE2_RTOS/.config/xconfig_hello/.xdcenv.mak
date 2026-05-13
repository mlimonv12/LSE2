#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/simplelink_msp432e4_sdk_4_20_00_12/source;C:/ti/simplelink_msp432e4_sdk_4_20_00_12/kernel/tirtos/packages;C:/Users/jsieiro/workspace_v12_6/hello_MSP_EXP432E401Y_tirtos_ccs/.config
override XDCROOT = C:/ti/xdctools_3_61_02_27_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/simplelink_msp432e4_sdk_4_20_00_12/source;C:/ti/simplelink_msp432e4_sdk_4_20_00_12/kernel/tirtos/packages;C:/Users/jsieiro/workspace_v12_6/hello_MSP_EXP432E401Y_tirtos_ccs/.config;C:/ti/xdctools_3_61_02_27_core/packages;..
HOSTOS = Windows
endif
