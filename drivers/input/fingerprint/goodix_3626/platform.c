// SPDX-License-Identifier: GPL-2.0
/*
 * platform independent driver interface
 *
 * Copyright (c) 2017 Goodix
 */
#define pr_fmt(fmt)	"gf_platform: " fmt

#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int irq;

	/*
	 * Keep the reset/IRQ lines unclaimed until the vendor HAL actually
	 * opens /dev/goodix_fp.  Stock marble enables both the FPC and the
	 * Goodix node, and both share gpio40/41; claiming at probe would make
	 * the other vendor driver fail unconditionally.
	 */
	irq = platform_get_irq(gf_dev->spi, 0);
	if (irq < 0) {
		pr_err("IRQ is invalid: %d\n", irq);
		return irq;
	}

	gf_dev->irq = irq;
	pr_info("gf::irq:%d\n", gf_dev->irq);

	return 0;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);
}

int gf_power_on(struct gf_dev *gf_dev)
{
	msleep(10);
	return 0;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	return 0;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (!gf_dev || !gf_dev->reset_gpio) {
		pr_info("Input buff is NULL.\n");
		return -EPERM;
	}

	gpiod_direction_output(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpiod_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	pr_info("%s\n", __func__);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (!gf_dev) {
		pr_info("Input buff is NULL.\n");
		return -EPERM;
	}

	if (gf_dev->irq_gpio)
		return gpiod_to_irq(gf_dev->irq_gpio);

	return gf_dev->irq ? gf_dev->irq : -EINVAL;
}
