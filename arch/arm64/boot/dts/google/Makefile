ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

dtbo-$(CONFIG_BOARD_SUNFISH) += \
		sm7150-sunfish-dev1.0-overlay.dtbo \
		sm7150-sunfish-dev1.1-overlay.dtbo \
		sm7150-sunfish-dev1.2-overlay.dtbo \
		sm7150-sunfish-evt1.0-overlay.dtbo \
		sm7150-sunfish-evt1.1-overlay.dtbo \
		sm7150-sunfish-dvt1.0-overlay.dtbo \
		sm7150-sunfish-pvt1.0-overlay.dtbo \
		sm7150-sunfish-mp1.0-overlay.dtbo

sm7150-sunfish-dev1.0-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-dev1.1-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-dev1.2-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-evt1.0-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-evt1.1-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-dvt1.0-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-pvt1.0-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-mp1.0-overlay.dtbo-base := qcom-base/sdmmagpie.dtb

dtbo-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws-overlay.dtbo \
	sm8150-coral-proto-overlay.dtbo \
	sm8150-coral-evt-overlay.dtbo \
	sm8150-coral-evt1_1-overlay.dtbo \
	sm8150-coral-dvt-overlay.dtbo \
	sm8150-coral-overlay.dtbo \
	sm8150-flame-proto-overlay.dtbo \
	sm8150-flame-notouch-overlay.dtbo \
	sm8150-flame-evt-overlay.dtbo \
	sm8150-flame-evt1_1-overlay.dtbo \
	sm8150-flame-dvt-overlay.dtbo \
	sm8150-flame-overlay.dtbo

sm8150-floral-jaws-overlay.dtbo-base	:= qcom-base/sm8150.dtb
sm8150-coral-proto-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-coral-evt-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-coral-evt1_1-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-coral-dvt-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-coral-overlay.dtbo-base		:= qcom-base/sm8150-v2.dtb
sm8150-flame-proto-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-flame-notouch-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-flame-evt-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-flame-evt1_1-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-flame-dvt-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-flame-overlay.dtbo-base		:= qcom-base/sm8150-v2.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
