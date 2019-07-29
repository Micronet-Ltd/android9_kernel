/* 
 * Lovdream peijian control driver
 * 
 * Copyright  (C)  2016 - 2020 Lovdream. Ltd.
 * 
 * wangqing.yu@lovdream.com
 * 
 * Version: 1.0
 * Release Date: 2017/03/10
 */

#ifndef _LOVDREAM_PEIJIAN_H_
#define _LOVDREAM_PEIJIAN_H_
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#define FORCE_STATUS_DELAY 2500
#define FORCE_REBOOT_DELAY 5000

struct peijian_data {
	struct class *peijian_dev_class;
	struct device *dev;
	//struct regulator *reg_peijian_vio;
	struct pinctrl *peijian_pinctrl;
	int peijian_pogo_irq;
	int peijian_pogo_pin4;
	int peijian_pogo_pin8;
	int peijian_pogo_pin13;
	int peijian_pogo_pin21;
	int peijian_pogo_pin22;
	int peijian_pogo_pin24;
	int peijian_pogo_pin26;
	struct notifier_block peijian_notifier;
	struct input_dev *input_dev;
	struct workqueue_struct *force_reset_wq;
	struct delayed_work force_reset_work;
	struct wake_lock vsm_wake_lock;
	struct power_supply *usb_psy;
	struct workqueue_struct *peijian_wq;
	struct delayed_work work;
	int id_status;
	spinlock_t irq_lock;
	struct workqueue_struct *pogo_wq;
	struct delayed_work pogo_work;
	spinlock_t pogo_irq_lock;
	s32 pogo_irq_is_disable;
	s32 irq_is_disable;
};

#endif /* PEIJIAN */
