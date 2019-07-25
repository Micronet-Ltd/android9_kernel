/*
 * Copyright (C) 2019 Micronet Inc, All Right Reserved.
 *
 * Athor: vladimir.zatulovsky@micronet-inc.com
 *
 */

#define pr_fmt(fmt) "%s: " fmt, __func__
#include <linux/kconfig.h>
//#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/switch.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/notifier.h>


MODULE_DESCRIPTION("3-wire host interface");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimir.zatulovsky@micronet-inc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tab8-hi-3w");

