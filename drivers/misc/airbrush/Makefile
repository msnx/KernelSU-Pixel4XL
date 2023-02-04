obj-$(CONFIG_AIRBRUSH_SM) += airbrush.o tmu/

airbrush-y +=	\
		airbrush-sm.o \
		airbrush-sm-ctrl.o \
		airbrush-clk.o \
		airbrush-bootseq.o \
		airbrush-spi.o \
		airbrush-pmu.o \
		airbrush-pmic-ctrl.o \
		airbrush-ddr.o \
		airbrush-ddr-vref.o \
		airbrush-clkout.o \
		airbrush-cooling.o \
		airbrush-thermal.o \
		airbrush-sysfs.o \
		airbrush-lvcc.o \
		airbrush-chip-info.o

airbrush-$(CONFIG_AIRBRUSH_SM_DEBUGFS)	+= airbrush-debugfs.o airbrush-ppmu.o
airbrush-$(CONFIG_AB_DDR_RW_TEST)	+= airbrush-ddr-test.o
airbrush-$(CONFIG_AB_DDR_EYE_MARGIN)	+= airbrush-ddr-eyemargin.o
airbrush-$(CONFIG_AB_DDR_PPC)		+= airbrush-ddr-ppc.o
