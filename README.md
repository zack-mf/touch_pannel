# touch_pannel
cypress input driver &amp;&amp; awa9120 led driver &amp;&amp; dts file


cypress4025lqi_driver.c驱动编译进内核
	修改Makefile，添加：
		obj-$(CONFIG_INPUT_CYPRESS4025LQI)      += cypress4025lqi_driver.o
	修改Kconfig，添加：
		config INPUT_CYPRESS4025LQI
			tristate "CONFIG_INPUT_CYPRESS4025LQI i2c transfer data"
			help
				depends on i2c bug

leds-aw9120.c 驱动编译进内核
	修改Makefile，添加：
		# LED AW9120 Drivers
		obj-$(CONFIG_LEDS_AW9120)       += leds-aw9120.o
	修改Kconfig，添加：
		config  LEDS_AW9120
			tristate "LED Support for AWINIC WA9120 I2C LED controller family"
			depends on LEDS_CLASS && I2C && OF
			select REGMAP_I2C
			help
			  This option enables support for LEDs connected to AWINIC WA9120
			  fancy LED driver chips accessed via the I2C bus.
			  Driver supports individual PWM brightness control for each channel.

			  This driver can also be built as a module. If so the module will be
			  called leds-aw9102.c

编译时候是能CONFIG_INPUT_CYPRESS4025LQI=y, CONFIG_LEDS_AW9120=y 即可编译到内核，或者直接再Makefile写死obj-y      += cypress4025lqi_driver.o