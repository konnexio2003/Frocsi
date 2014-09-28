#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/marci/ti/tirtos_tivac_2_01_00_03/packages;/home/marci/ti/tirtos_tivac_2_01_00_03/products/bios_6_40_03_39/packages;/home/marci/ti/tirtos_tivac_2_01_00_03/products/ndk_2_23_01_01/packages;/home/marci/ti/tirtos_tivac_2_01_00_03/products/uia_2_00_01_34/packages;/home/marci/ti/ccsv6/ccs_base;/home/marci/git/MSE/MSE2_TIVA129X/.config
override XDCROOT = /home/marci/ti/xdctools_3_30_03_47_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/marci/ti/tirtos_tivac_2_01_00_03/packages;/home/marci/ti/tirtos_tivac_2_01_00_03/products/bios_6_40_03_39/packages;/home/marci/ti/tirtos_tivac_2_01_00_03/products/ndk_2_23_01_01/packages;/home/marci/ti/tirtos_tivac_2_01_00_03/products/uia_2_00_01_34/packages;/home/marci/ti/ccsv6/ccs_base;/home/marci/git/MSE/MSE2_TIVA129X/.config;/home/marci/ti/xdctools_3_30_03_47_core/packages;..
HOSTOS = Linux
endif
