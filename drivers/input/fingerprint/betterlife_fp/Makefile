#ccflags-y += -DCOMPATIBLE
#ccflags-y += -DFAST_VERSION
#ccflags-y += -DCONFIG_MTK_CLK
#ccflags-y += -DNEED_OPT_POWER_ON1V8
#ccflags-y += -DNEED_OPT_POWER_ON2V8

#ccflags-y += -DBF_PINCTL

#PLATFORM_MTK PLATFORM_MT6739 PLATFORM_SPRD PLATFORM_QCOM
BL_PLATFORM := PLATFORM_SPRD

#TEE_TK TEE_BEANPOD REE other TEE_PLATFORM add #
#TEE_PLATFORM := TEE_TK

ifeq ($(strip $(TEE_PLATFORM)), TEE_TK)
$(info "************* trustkernel tee ***************")
ccflags-y += -DTEE_TK
endif

ifeq ($(strip $(TEE_PLATFORM)), TEE_BEANPOD)
$(info "**************** beanpod tee ****************")
ccflags-y += -DTEE_BEANPOD
ccflags-y += -DCONFIG_MTK_CLK
endif

ifeq ($(strip $(TEE_PLATFORM)), REE)
$(info "********************** ree ******************")
ccflags-y += -DBF_REE
endif

ifeq ($(strip $(BL_PLATFORM)), PLATFORM_MTK)
ccflags-y += -DPLATFORM_MTK
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat -Wno-unused
endif

ifeq ($(strip $(BL_PLATFORM)), PLATFORM_MT6739)
ccflags-y += -DPLATFORM_MT6739
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat -Wno-unused
endif

ifeq ($(strip $(BL_PLATFORM)), PLATFORM_SPRD)
ccflags-y += -DPLATFORM_SPRD
endif

ifeq ($(strip $(BL_PLATFORM)), PLATFORM_QCOM)
ccflags-y += -DPLATFORM_QCOM
endif

obj-y := bf_fp_spi.o bf_fp_platform.o
