/* linux/arch/arm/mach-msm/board-primoc-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap-7x30.h>
#include <mach/vreg.h>
#include <mach/msm_panel.h>
#include <mach/panel_id.h>

#include "../board-primoc.h"
#include "../devices.h"
#include "../proc_comm.h"
#include "../../../../drivers/video/msm/mdp_hw.h"

#define B(s...) printk(s)
#define LCM_GPIO_CFG(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)

extern int panel_type;

static struct vreg *V_LCMIO_1V8, *V_LCMIO_2V85;

#define PWM_USER_DEF	 	142
#define PWM_USER_MIN		30
#define PWM_USER_DIM		9
#define PWM_USER_MAX		255

#define PWM_NOVATEK_DEF		135
#define PWM_NOVATEK_MIN		7
#define PWM_NOVATEK_MAX		255

#define DEFAULT_BRIGHTNESS      PWM_USER_DEF
#define LCM_CMD(_cmd, _delay, ...)                              \
{                                                               \
        .cmd = _cmd,                                            \
        .delay = _delay,                                        \
        .vals = (u8 []){__VA_ARGS__},                           \
        .len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}

enum {
	GATE_ON = 1 << 0,
};

static uint32_t display_on_gpio_table[] = {
	LCM_GPIO_CFG(PRIMOC_LCD_PCLK, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_DE, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_VSYNC, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_HSYNC, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_G2, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_G3, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_G4, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_G5, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_G6, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_G7, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_B3, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_B4, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_B5, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_B6, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_B7, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_R3, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_R4, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_R5, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_R6, 1),
	LCM_GPIO_CFG(PRIMOC_LCD_R7, 1),
};

static uint32_t display_off_gpio_table[] = {
	LCM_GPIO_CFG(PRIMOC_LCD_PCLK, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_DE, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_VSYNC, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_HSYNC, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_G2, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_G3, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_G4, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_G5, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_G6, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_G7, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_B3, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_B4, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_B5, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_B6, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_B7, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_R3, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_R4, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_R5, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_R6, 0),
	LCM_GPIO_CFG(PRIMOC_LCD_R7, 0),
};

static int panel_gpio_switch(int on)
{
	config_gpio_table(
		!!on ? display_on_gpio_table : display_off_gpio_table,
		!!on ? ARRAY_SIZE(display_on_gpio_table) : ARRAY_SIZE(display_off_gpio_table));

	return 0;
}

static int panel_init_power(void)
{
	int rc;

	V_LCMIO_1V8 = vreg_get(NULL, "lvsw0");
	if (IS_ERR(V_LCMIO_1V8)) {
		pr_err("%s: LCMIO_1V8 get failed (%ld)\n",
			__func__, PTR_ERR(V_LCMIO_1V8));
		return -1;
	}

	V_LCMIO_2V85 = vreg_get(NULL, "gp13");
	if (IS_ERR(V_LCMIO_2V85)) {
		pr_err("%s: LCMIO_2V85 get failed (%ld)\n",
			__func__, PTR_ERR(V_LCMIO_2V85));
		return -1;
	}

	rc = vreg_set_level(V_LCMIO_1V8, 1800);
	if (rc) {
		pr_err("%s: vreg LCMIO_1V8 set level failed(%d)\n",
			__func__, rc);
		return -1;
	}

	return 0;
}

static int panel_lg_power(int on)
{
	int rc = 0;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		rc = vreg_enable(V_LCMIO_1V8);
	}
	if (rc) {
		pr_err("%s: V_LCMIO_1V8 vreg enable failed (%d)\n",
			__func__, rc);
		return -1;
	}

	if (on) {
		rc = vreg_enable(V_LCMIO_2V85);
	}
	if (rc) {
		pr_err("%s: V_LCMIO_2V85 vreg enable failed (%d)\n",
			__func__, rc);
		return -1;
	}

	if (on) {
		hr_msleep(10);
		gpio_set_value(PRIMOC_LCD_RSTz, 1);
		hr_msleep(10);
		gpio_set_value(PRIMOC_LCD_RSTz, 0);
		udelay(500);
		gpio_set_value(PRIMOC_LCD_RSTz, 1);
		hr_msleep(10);
	} else if (!on) {
		hr_msleep(10);
		gpio_set_value(PRIMOC_LCD_RSTz, 0);
		hr_msleep(120);
	}

	if (!on) {
		vreg_disable(V_LCMIO_2V85);
		vreg_disable(V_LCMIO_1V8);
	}

	if (rc) {
		pr_err("%s: V_LCMIO_2V85, 1V8 vreg disable failed (%d)\n",
			__func__, rc);
		return -1;
	}

	return 0;
}

static void lcdc_config_gpios(int on)
{
	printk(KERN_INFO "%s: power goes to %d\n", __func__, on);

	if (panel_lg_power(on))
		printk(KERN_ERR "%s: panel_power failed!\n", __func__);
	if (panel_gpio_switch(on))
		printk(KERN_ERR "%s: panel_gpio_switch failed!\n", __func__);
}


static struct msm_panel_common_pdata lcdc_panel_data = {
	.panel_config_gpio = lcdc_config_gpios,
};

struct platform_device lcdc_primoc_lg_panel_device = {
	.name   = "lcdc_primo_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_panel_data,
	}
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_max_clk = 192000000,
	.mdp_rev = MDP_REV_40,
};

static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	return panel_lg_power(on);
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save = lcdc_panel_power,
};

struct msm_list_device primoc_fb_devices[] = {
	{ "mdp", &mdp_pdata },
	{ "lcdc", &lcdc_pdata }
};

int device_fb_detect_panel(const char *name)
{
	return 0;
}

int __init primoc_init_panel(void)
{
	int ret = 0;

	ret = panel_init_power();
	if(ret)
	return ret;

	msm_fb_add_devices(primoc_fb_devices, ARRAY_SIZE(primoc_fb_devices));

	ret = platform_device_register(&lcdc_primoc_lg_panel_device);
	printk(KERN_ERR "%s is LG panel: %d\n", __func__, panel_type);

	return ret;
}
