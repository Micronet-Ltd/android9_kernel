/*
 * Copyright (C) 2017 Micronet Inc, All Right Reserved.
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
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/notifier.h>
#include <linux/power_supply.h>

#include <linux/syscalls.h>
//#include <linux/file.h>
//#include <linux/fs.h>
//#include <linux/cred.h>
#include <linux/dirent.h>
//#include <linux/string.h>
#include "../misc/hi_3w/hi_3w.h"

#if defined  (CONFIG_PRODUCT_TAB8_FIXED)
extern int32_t gpio_in_register_notifier(struct notifier_block *nb);
#else
static int32_t gpio_in_register_notifier(struct notifier_block *nb)
{
    return 0;
}
#endif

#define SWITCH_DOCK	(1 << 0)
#define SWITCH_IGN  (1 << 1)
#define SWITCH_EDOCK (1 << 2)
#define SWITCH_ODOCK (1 << 3)

#define DEBOUNCE_INTERIM  200 //500
#define PATERN_INTERIM    100
#define BASIC_PATTERN     0
#define SMART_PATTERN     1000
#define IG_HI_PATTERN     500
#define IG_LOW_PATTERN    100

#define VIRT_GPIO_OFF	0
#define VIRT_GPIO_INIT	1
#define VIRT_GPIO_ON	2
#define VGPIO_MAX 8
//////////////////////////////////////////
#define MCU_GPIO_MAX 160
#define J1708_GPIO_OFFSET 64 //GPIO num 0 at group C 32*2 + 0 the driver 
                              //will transfer it to the correct number
                              //I couldn't keep with the MCU convention as GPIO 
                              //nums in the device should be continuos
#define RS48_GPIO_OFFSET 155 //GPIO num 27 at group E 32*4 +27 the driver 
                              //will transfer it to the correct number
                              //I couldn't keep with the MCU convention as GPIO 
                              //nums in the device should be continuos
////////////////////////////////////////////
#define FORBID_EXT_SPKR	9
enum e_dock_type {
    e_dock_type_unspecified = -1,
    e_dock_type_basic,
    e_dock_type_smart
};

struct dock_switch_attr {
    struct device_attribute attr;
    char name[32];
};

struct dock_switch_device {
	struct  switch_dev sdev;
    struct  work_struct work;
	int     dock_pin;
    int 	ign_pin;
    int     usb_switch_pin;
    int     otg_en_pin;
    int     mic_sw_pin;
    int     dock_irq;
    int 	ign_irq;
	int	    dock_active_l;
	int	    ign_active_l;
    int     usb_switch_l;
    int     otg_en_l;
    int     mic_sw_l;
    unsigned sched_irq;
	int	    state;
    struct  wake_lock wlock;
	struct  device*	pdev;
    struct 	notifier_block   ignition_notifier;
    enum    e_dock_type dock_type;
    struct  power_supply *usb_psy;
    int 	ampl_enable;
    struct  pinctrl *pctl;
    struct  dock_switch_attr attr_outs_mask_state;
    struct  dock_switch_attr attr_outs_mask_set;
    struct  dock_switch_attr attr_outs_mask_clr;
    struct  dock_switch_attr attr_dbg_state;
    //////////////barak/////////////////////
    struct  dock_switch_attr attr_J1708_en;
    struct  dock_switch_attr attr_rs485_en;
    ////////////////////////////////////////   
    spinlock_t outs_mask_lock;
    unsigned long lock_flags;
    unsigned outs_mask_state;
    unsigned outs_mask_set;
    unsigned outs_mask_clr;
    unsigned outs_base;
    unsigned outs_num;
    int outs_can_sleep;
    int outs_pins[VGPIO_MAX];
    struct delayed_work	vgpio_init_work;
    /////////////////////////////
    int mcu_outs_pins[MCU_GPIO_MAX];
    unsigned mcu_gpio_base;
    unsigned mcu_gpio_num;
    unsigned j1708en_vgpio_num;
    unsigned rs485en_vgpio_num;
    struct delayed_work	mcu_gpio_init_work;//used to initiate the mcu gpios
    int irq_ack;
    struct mutex lock;
};

#include "../gpio/gpiolib.h"
static DEFINE_MUTEX(ampl_lock);
static void set_aml_enable(struct dock_switch_device *ds, int val)
{
    struct gpio_desc *desc;

    if (!gpio_is_valid(ds->dock_pin)) {
		return;
	}

	mutex_lock(&ampl_lock);

	ds->ampl_enable = val;
//	pr_notice("set %d\n", val);

    desc = gpio_to_desc(ds->dock_pin);
	if(FORBID_EXT_SPKR == val) {
		gpio_direction_input(ds->dock_pin);
        gpio_lock_as_irq(desc->chip, gpio_chip_hwgpio(desc));
	}
	else {//if(0 == val) {
        gpio_unlock_as_irq(desc->chip, gpio_chip_hwgpio(desc));
        gpio_direction_output(ds->dock_pin, !val);
	}

	mutex_unlock(&ampl_lock);

}

static ssize_t ampl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
	struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

	unsigned long val;

	if (!ds || !gpio_is_valid(ds->dock_pin)) {
		return -EINVAL;
	}

	mutex_lock(&ampl_lock);

	if (FORBID_EXT_SPKR != ds->ampl_enable && kstrtol(buf, 10, &val) == 0 && (val == 0 || val == 1)) {
		pr_info("val %lu (%d)\n", val, ds->ampl_enable);
		if(val != ds->ampl_enable) {
			ds->ampl_enable = val;
			gpio_set_value(ds->dock_pin, !val);
		}
	}else {
		mutex_unlock(&ampl_lock);
		return -EINVAL;
	}
	mutex_unlock(&ampl_lock);

	return count;
}

static ssize_t ampl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
	struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

	if (!ds || !gpio_is_valid(ds->dock_pin)) {
		return 0;
	}
	return sprintf(buf, "%d\n", ds->ampl_enable);
}

static DEVICE_ATTR(ampl_enable, S_IRUGO|S_IWUSR|S_IWGRP, ampl_show, ampl_store);

static RAW_NOTIFIER_HEAD(cradle_notify_chain);
static DEFINE_RAW_SPINLOCK(cradle_notify_chain_lock);

void cradle_notify(unsigned long reason, void *arg)
{
    unsigned long flags;

    raw_spin_lock_irqsave(&cradle_notify_chain_lock, flags);
    raw_notifier_call_chain(&cradle_notify_chain, reason, 0);
    raw_spin_unlock_irqrestore(&cradle_notify_chain_lock, flags);
}

int cradle_register_notifier(struct notifier_block *nb)
{
    unsigned long flags;
    int err;

    raw_spin_lock_irqsave(&cradle_notify_chain_lock, flags);
    err = raw_notifier_chain_register(&cradle_notify_chain, nb);
    raw_spin_unlock_irqrestore(&cradle_notify_chain_lock, flags);

    return err;
}
EXPORT_SYMBOL(cradle_register_notifier);

int cradle_unregister_notifier(struct notifier_block *nb)
{
    unsigned long flags;
    int err;

    raw_spin_lock_irqsave(&cradle_notify_chain_lock, flags);
    err = raw_notifier_chain_unregister(&cradle_notify_chain, nb);
    raw_spin_unlock_irqrestore(&cradle_notify_chain_lock, flags);

    return err;
}
EXPORT_SYMBOL(cradle_unregister_notifier);

static int wait_for_stable_signal(int pin, int interim)
{
    long long timer;
    int pulses = 0, state = 0;

//    long long start_time = 0, end_time = 0;

    timer = ktime_to_ms(ktime_get()) + interim;

    if (gpio_is_valid(pin)) {
//        pr_notice("start %d pulses %lld\n", gpio_get_value(pin), ktime_to_ms(ktime_get()));
        do {
            if (state != gpio_get_value(pin)) {
                state ^= 1;
//                pr_notice("detcted %d pulses %lld\n", pulses, ktime_to_ms(ktime_get()));
                pulses++;
                ///temp!!!
//                if(1==pulses)
//                	start_time = ktime_to_ms(ktime_get());
//               	end_time = ktime_to_ms(ktime_get());
            }
        } while (ktime_to_ms(ktime_get()) < timer);
    }
    pulses >>= 1;

    pr_notice("detected %d pulses %lld\n", pulses, ktime_to_ms(ktime_get()));
 //   pr_notice("detcted %d pulses %lld (delta %lld)\n", pulses, ktime_to_ms(ktime_get()), (end_time - start_time));
    return pulses;
}

static inline int pulses2freq(int pulses, int interim)
{
//    pr_notice("%d HZ %lld\n", 1000 * pulses / interim, ktime_to_ms(ktime_get()));
    return 1000 * pulses / interim;
}

#define HYST_PATTERN(p1, p2) ((p2) - (((p2) - (p1)) >> 1))
//#define HYST_PATTERN(p1, p2) ((p1) + (((p2) - (p1)) >> 2))
static inline int freq2pattern(int freq)
{
//    if (freq < IG_LOW_PATTERN - pulses2freq(2, PATERN_INTERIM)) {//HYST_PATTERN(BASIC_PATTERN, IG_LOW_PATTERN)) {
    if (freq < HYST_PATTERN(BASIC_PATTERN, IG_LOW_PATTERN)) {
        return BASIC_PATTERN;
//    } else if (freq <= HYST_PATTERN(IG_LOW_PATTERN, IG_HI_PATTERN) && IG_LOW_PATTERN - pulses2freq(2, PATERN_INTERIM) <= freq) {
    } else if (freq <= HYST_PATTERN(IG_LOW_PATTERN, IG_HI_PATTERN) && HYST_PATTERN(BASIC_PATTERN, IG_LOW_PATTERN) <= freq) {
        return IG_LOW_PATTERN;
    } else if (freq <= HYST_PATTERN(IG_HI_PATTERN, SMART_PATTERN) && HYST_PATTERN(IG_LOW_PATTERN, IG_HI_PATTERN) < freq) {
        return IG_HI_PATTERN;
    }

    return SMART_PATTERN; 
}

inline void enable_switch_irq(int irq, int en)
{
    struct irq_desc *desc;

    desc = irq_to_desc(irq);
    if (en) {
        if(desc->depth > 0) {
            enable_irq(irq);
        }
    } else {
        if (0 == desc->depth) {
            disable_irq_nosync(irq);
        }
    }
}

inline void enable_and_sync_switch_irq(int irq, int en)
{
    struct irq_desc *desc;

    desc = irq_to_desc(irq);
    if (en) {
        if(desc->depth > 0) {
            enable_irq(irq);
        }
    } else {
        if (0 == desc->depth) {
            disable_irq(irq);
        }
    }
}

static void dock_switch_work_func(struct work_struct *work) 
{
	struct dock_switch_device *ds = container_of(work, struct dock_switch_device, work);
    long long timer = ktime_to_ms(ktime_get());
    int val = 0, act = 0;
    union power_supply_propval prop = {0,};

    if (!ds->usb_psy) {
        pr_notice("usb power supply not ready %lld\n", ktime_to_ms(ktime_get()));
        ds->usb_psy = power_supply_get_by_name("usb");
        msleep(200);
        schedule_work(&ds->work);

        return;
    }

    mutex_lock(&ds->lock);
    if (e_dock_type_basic != ds->dock_type) {
        if (ds->sched_irq & SWITCH_DOCK) {
            //enable_and_sync_switch_irq(ds->ign_irq, 0);
            val = wait_for_stable_signal(ds->ign_pin, DEBOUNCE_INTERIM + PATERN_INTERIM);
            if (ds->irq_ack) {
                //val += ds->irq_ack;
                //val = ds->irq_ack;
                pr_notice("%d acknowledged interrupts %lld\n", ds->irq_ack, ktime_to_ms(ktime_get()));
                ds->irq_ack = 0;
            }
            //enable_and_sync_switch_irq(ds->ign_irq, 1);
        } else /*if (gpio_is_valid(ds->dock_pin) && ds->dock_active_l != gpio_get_value(ds->dock_pin))*/ {
            val = 0;
        }
        val = pulses2freq(val, PATERN_INTERIM);
        pr_notice("%d HZ %lld\n", val, ktime_to_ms(ktime_get()));
        val = freq2pattern(val);
        pr_notice("pattern[%d, %d] [%lld]%lld\n", val, gpio_get_value(ds->ign_pin), timer, ktime_to_ms(ktime_get()));
       	if (BASIC_PATTERN == val) {
            if (gpio_is_valid(ds->dock_pin) && gpio_is_valid(ds->ign_pin)) {
				if (e_dock_type_smart == ds->dock_type && ds->ign_active_l == gpio_get_value(ds->ign_pin)) {
					pr_notice("smart cradle unplagged %lld [dock_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->dock_pin));
					if (ds->usb_psy) {
                        pr_notice("notify usb host about unplug cradel %lld\n", ktime_to_ms(ktime_get()));
						prop.intval = 0;
						//ds->usb_psy->set_property(ds->usb_psy, POWER_SUPPLY_PROP_BOOST_CURRENT, &prop);
                        power_supply_set_usb_otg(ds->usb_psy, prop.intval);
					}
					ds->dock_type = e_dock_type_unspecified;
					ds->sched_irq |= SWITCH_IGN;
					act = 1;
				} else if (e_dock_type_smart != ds->dock_type) {
                    pr_notice("probably basic cradle plugging or smart cradle lost power %lld \n", ktime_to_ms(ktime_get())); 
                    if (ds->dock_active_l == gpio_get_value(ds->dock_pin)) {
                        //if (power_supply_is_system_supplied() > 0) {
                        //    pr_notice("basic cradle attempt to be plugged %lld  [dock_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->dock_pin)); 
                        //    ds->dock_type = e_dock_type_basic;
                        //    act = 1;
                        //  }
                        if (ds->ign_active_l != gpio_get_value(ds->ign_pin)) {
                            pr_notice("basic cradle attempt to be plugged %lld  [ign_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->ign_pin)); 
                            ds->dock_type = e_dock_type_basic;
                            act = 1;
                        } else {
                            pr_notice("any cradle hasn't detected %lld  [dock_pin %d, ign_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->dock_pin), gpio_get_value(ds->ign_pin));
                        }
                    } else if (ds->ign_active_l != gpio_get_value(ds->ign_pin)) {
                        pr_notice("basic cradle attempt to be plugged %lld  [ign_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->ign_pin)); 
                        ds->dock_type = e_dock_type_basic;
                        act = 1;
                    }
                }

				if(act) {//e_dock_type_basic == ds->dock_type || ds->ign_active_l != gpio_get_value(ds->ign_pin)) {
		            act = 0;
					val = 0;
					// pin function is basic dock detect
					pr_notice("enable dock detect function %lld\n", ktime_to_ms(ktime_get()));
					set_aml_enable(ds, FORBID_EXT_SPKR);//
					//gpio_direction_input(ds->dock_pin);
					// switch otg connector
					if (gpio_is_valid(ds->usb_switch_pin)) {
						pr_notice("switch usb %s connector %lld\n", (e_dock_type_unspecified == ds->dock_type)?"type-c":"44-pin", ktime_to_ms(ktime_get()));
						gpio_set_value(ds->usb_switch_pin, !!(ds->usb_switch_l == (e_dock_type_unspecified != ds->dock_type)));
					}
                    if (gpio_is_valid(ds->otg_en_pin)) {
                        pr_notice("reverse otg_en %lld\n", ktime_to_ms(ktime_get()));
                        gpio_set_value(ds->otg_en_pin, !!(ds->otg_en_l == (e_dock_type_unspecified != ds->dock_type)));
                    }
				}
				else {//nothing changed
					val = ds->state;
				}
            }
        } else if (e_dock_type_smart == ds->dock_type) {
            if (IG_HI_PATTERN == val) {
                val = (SWITCH_DOCK | SWITCH_IGN | SWITCH_EDOCK);
                pr_notice("ignition plugged %lld\n", ktime_to_ms(ktime_get()));
            } else if (IG_LOW_PATTERN == val) {
                val = SWITCH_DOCK | SWITCH_EDOCK;
                pr_notice("ignition unplugged %lld\n", ktime_to_ms(ktime_get()));
            } else {
                val = ds->state;
            }
        } else if (SMART_PATTERN == val || IG_HI_PATTERN == val || IG_LOW_PATTERN == val) {
            pr_notice("smart cradle attempt to be plugged %lld\n", ktime_to_ms(ktime_get()));
            ds->dock_type = e_dock_type_smart;
            if (ds->usb_psy) {
                pr_notice("notify usb host about plug smart cradel %lld\n", ktime_to_ms(ktime_get()));
                prop.intval = 0x11;
                power_supply_set_usb_otg(ds->usb_psy, prop.intval);
                power_supply_set_current_limit(ds->usb_psy, 1500*1000);
            }
            if (IG_HI_PATTERN == val) {
                val = SWITCH_DOCK | SWITCH_IGN | SWITCH_EDOCK; 
            } else {
                val = SWITCH_DOCK | SWITCH_EDOCK; 
            }

            // disable dock interrupts while smart cradle
            if (ds->dock_irq) {
                enable_switch_irq(ds->dock_irq, 0);
                pr_notice("disable dock irq[%d] %lld\n", ds->dock_irq, ktime_to_ms(ktime_get()));
                ds->sched_irq &= ~SWITCH_IGN;
            }

            if (gpio_is_valid(ds->dock_pin)) {
                // pin function is smart cradle spkr switch
                //pr_notice("enable spkr switch function %lld\n", ktime_to_ms(ktime_get()));
                //canceled as cause of hw design
                //gpio_direction_output(ds->dock_pin, 1);
				set_aml_enable(ds, 0);//
            }

            // switch otg connector
            if (gpio_is_valid(ds->usb_switch_pin)) {
                pr_notice("switch usb 44-pin connector %lld\n", ktime_to_ms(ktime_get()));
                gpio_set_value(ds->usb_switch_pin, ds->usb_switch_l);
            }
            if (gpio_is_valid(ds->otg_en_pin)) {
                pr_notice("enable otg %lld\n", ktime_to_ms(ktime_get()));
                gpio_set_value(ds->otg_en_pin, ds->otg_en_l);
            }
        }
    }

    // Vladimir
    // the pins IGN and CRADLE_DETECT ara swapped in hardware
    //
    if (e_dock_type_basic == ds->dock_type) {
        ds->irq_ack = 0;
        if (gpio_is_valid(ds->ign_pin)) {
            if (ds->ign_active_l != gpio_get_value(ds->ign_pin)) {
                prop.intval = 0x20;
                power_supply_set_usb_otg(ds->usb_psy, prop.intval);
                power_supply_set_current_limit(ds->usb_psy, 1500*1000);
                pr_notice("basic cradle plagged %lld\n", ktime_to_ms(ktime_get()));
                val |= SWITCH_DOCK;
            } else {
                pr_notice("basic cradle unplagged %lld\n", ktime_to_ms(ktime_get()));
                prop.intval = 0x0;
                power_supply_set_usb_otg(ds->usb_psy, prop.intval);
                ds->dock_type = e_dock_type_unspecified;
                // switch otg connector
                if (gpio_is_valid(ds->usb_switch_pin)) {
                    pr_notice("switch usb type-c connector %lld\n", ktime_to_ms(ktime_get()));
                    gpio_set_value(ds->usb_switch_pin, !ds->usb_switch_l);
                }
            }
//        prop.intval = POWER_SUPPLY_TYPE_USB_ACA;
//        prop.intval = POWER_SUPPLY_TYPE_UNKNOWN;
//        ds->usb_psy->set_property(ds->usb_psy, POWER_SUPPLY_PROP_REAL_TYPE, &prop);
        }

        if (gpio_is_valid(ds->dock_pin)) {
            if (ds->dock_active_l != gpio_get_value(ds->dock_pin) ) {
                val |= SWITCH_IGN;
            }
        }
    }

    mutex_unlock(&ds->lock);
    // interrupts handled
    if (ds->sched_irq & SWITCH_IGN) {
        pr_notice("enable ign/dock monitor irq[%d]\n", ds->dock_irq);
        ds->sched_irq &= ~SWITCH_IGN;
        enable_switch_irq(ds->dock_irq, 1);
    }

    if (ds->sched_irq & SWITCH_DOCK) {
        pr_notice("enable dock/ign monitor irq[%d]\n", ds->ign_irq);
        ds->sched_irq &= ~SWITCH_DOCK;
        enable_switch_irq(ds->ign_irq, 1);
    }

	if (ds->state != val) {
        if (val & SWITCH_IGN) {
            wake_lock(&ds->wlock);
        } else {
            wake_unlock(&ds->wlock);
        }
		ds->state = val;
//        pr_notice("notify dock state [%d] %lld\n", ds->state, ktime_to_ms(ktime_get()));
		switch_set_state(&ds->sdev, val);
	}
}

#define SC_IG_HI    110
#define SC_IG_LOW   55

static void dock_switch_work_func_fix(struct work_struct *work)
{
	struct dock_switch_device *ds  = container_of(work, struct dock_switch_device, work);
    int val = 0;
    uint32_t cmd, fd;
    union power_supply_propval prop = {0,};
    char ver[16];
    int transmit_err = -3;
    int err_cnt = 0;

    if (!ds->usb_psy) {
        pr_notice("usb power supply not ready %lld\n", ktime_to_ms(ktime_get()));
        ds->usb_psy = power_supply_get_by_name("usb");
        msleep(200);
        schedule_work(&ds->work);

        return;
    }

    // Vladimir:
    // PATERN_INTERIM should be replaced by correct
    //
    mutex_lock(&ds->lock);
    val = wait_for_stable_signal(ds->dock_pin, DEBOUNCE_INTERIM + PATERN_INTERIM);
    val = pulses2freq(val, PATERN_INTERIM);
    pr_notice("%d HZ %lld\n", val, ktime_to_ms(ktime_get()));

    if (ds->dock_active_l == gpio_get_value(ds->dock_pin) && val < 2) {
        val = 0; //&= ~(SWITCH_IGN | SWITCH_DOCK | SWITCH_ODOCK);
        pr_notice("stm32 detached %lld\n", ktime_to_ms(ktime_get()));
        if (gpio_is_valid(ds->usb_switch_pin)) {
            pr_notice("switch usb to type-c connector %lld\n", ktime_to_ms(ktime_get()));
            gpio_set_value(ds->usb_switch_pin, !ds->usb_switch_l);
        }

        if (gpio_is_valid(ds->mic_sw_pin)) {
            pr_notice("switch mic_in1(mic1) to 28-pin connector %lld\n", ktime_to_ms(ktime_get()));
            gpio_set_value(ds->mic_sw_pin, !ds->mic_sw_l);
        }

        prop.intval = 0x0;
        power_supply_set_usb_otg(ds->usb_psy, prop.intval);
        fd = sys_open("/proc/mcu_version", O_WRONLY, 0);
        if (fd >= 0) {
            sprintf(ver, "unknown");
            pr_notice("stm32 detached %s\n", ver);
            sys_write(fd, ver, strlen(ver));
            sys_close(fd);
        }
    } else {
        pr_notice("stm32 attached %lld\n", ktime_to_ms(ktime_get()));
        if (val > SC_IG_HI) {
            pr_notice("freq to high ignition off %lld\n", ktime_to_ms(ktime_get()));
            val = (SWITCH_DOCK | SWITCH_ODOCK); 
        } else if (val > (SC_IG_HI - SC_IG_HI/4)) {
            pr_notice("ignition on %lld\n", ktime_to_ms(ktime_get()));
            val = (SWITCH_DOCK | SWITCH_ODOCK | SWITCH_IGN); 
        } else if (val > 1 && (val < SC_IG_LOW)) {
            pr_notice("ignition off %lld\n", ktime_to_ms(ktime_get()));
            val = (SWITCH_DOCK | SWITCH_ODOCK); 
        } else {
            val = (ds->state)?ds->state:SWITCH_DOCK | SWITCH_ODOCK;
        }
        cmd = 0;
        cmd = 2<<24;
        pr_notice("request %x\n", (cmd));
        transmit_err = hi_3w_tx_cmd(&cmd, 1);
        pr_notice("answer %x\n", (cmd & 0x00FFFFFF));
        while (transmit_err) {
            pr_notice("transmit error %d\n", transmit_err);
            cmd = 0;
            cmd = 2<<24;
            msleep(10);
            transmit_err = hi_3w_tx_cmd(&cmd, 1);
            if (++err_cnt >= 15) {
                err_cnt = 0;
                break;
            }
        }
        pr_notice("transmit is ok\n");
        fd = sys_open("/proc/mcu_version", O_WRONLY, 0);
        if (fd >= 0) {
            sprintf(ver, "%d.%d.%d", (cmd >> 16) & 0xFF, (cmd >> 8) & 0xFF, cmd & 0xFF);
            pr_notice("stm32 sw ver %s\n", ver);
            sys_write(fd, ver, strlen(ver));
            sys_close(fd);
        }
        prop.intval = 0x20;
        power_supply_set_usb_otg(ds->usb_psy, prop.intval);
        power_supply_set_current_limit(ds->usb_psy, 1500*1000);
    }
    mutex_unlock(&ds->lock);

    if (ds->sched_irq & SWITCH_IGN) {
        pr_notice("enable ign/dock monitor irq[%d]\n", ds->dock_irq);
        ds->sched_irq &= ~SWITCH_IGN;
        enable_switch_irq(ds->dock_irq, 1);
    }

    if (ds->sched_irq & SWITCH_DOCK) {
        pr_notice("enable dock/ign monitor irq[%d]\n", ds->ign_irq);
        ds->sched_irq &= ~SWITCH_DOCK;
    }

    if (ds->state != val) {
        pr_notice("dock state changed to %d\n", val);
        if (val & SWITCH_ODOCK) {
            wake_lock(&ds->wlock);
            if (gpio_is_valid(ds->usb_switch_pin)) {
                pr_notice("switch usb 44-pin connector %lld\n", ktime_to_ms(ktime_get()));
                gpio_set_value(ds->usb_switch_pin, ds->usb_switch_l);
            }
            if (gpio_is_valid(ds->mic_sw_pin)) {
                pr_notice("switch mic_in1(mic4) to 44-pin connector %lld\n", ktime_to_ms(ktime_get()));
                gpio_set_value(ds->mic_sw_pin, ds->mic_sw_l);
            }

        } else {
            wake_unlock(&ds->wlock);
        }
		ds->state = val;
		switch_set_state(&ds->sdev, val);
        cradle_notify(ds->state != 0, 0);
	}
}

static irqreturn_t dock_switch_irq_handler(int irq, void *arg)
{
	struct dock_switch_device *ds = (struct dock_switch_device *)arg;
    int sched = 0;

// Vladimir
// the pins IGN and CRADLE_DETECT ara swapped in hardware
//

    disable_irq_nosync(irq);
//    pr_notice("irq[%d]\n", irq);

    if (irq == ds->dock_irq) {
        //pr_notice("ign/dock state [%d]\n", gpio_get_value(ds->dock_pin));
        ds->sched_irq |= SWITCH_IGN;
        sched = 1;
    }

    if (irq == ds->ign_irq) {
//        pr_notice("dock/ign state [%d]\n", gpio_get_value(ds->ign_pin));
        ds->sched_irq |= SWITCH_DOCK;
        //if (0 == ds->irq_ack) {
            sched = 1;
        //}
        ds->irq_ack++; 
    }

    if (sched) {
        schedule_work(&ds->work); 
    }

	return IRQ_HANDLED;
}

static ssize_t dock_switch_print_state(struct switch_dev *sdev, char *buffer)
{
	struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

	return sprintf(buffer, "%d", ds->state);
}

static int32_t __ref dock_switch_ign_callback(struct notifier_block *nfb, unsigned long reason, void *arg)
{
    struct dock_switch_device *ds = container_of(nfb, struct dock_switch_device, ignition_notifier);

    pr_notice("%ld\n", reason);
    if (0) {
        schedule_work(&ds->work); 
    }

    return NOTIFY_OK;
}

#if 0
//unsigned long long de_buf[(sizeof(struct linux_dirent64) + 260)/sizeof(unsigned long long)];
static int find_dir_by_label(char *root, char *label, char *out, int out_len)
{
	struct linux_dirent64 *de;
	int fd,lfd, des, err = -1, i;
    mm_segment_t prev_fs;
    char de_buf[sizeof(struct linux_dirent64) + 260];
    char lfname[64], lbl[64];

    prev_fs = get_fs();
	set_fs(get_ds());

    fd = sys_open(root, O_DIRECTORY|O_RDONLY, S_IRUSR|S_IRGRP);
    if (fd) {
        do {
            de = (struct linux_dirent64 *)de_buf;
            des = sys_getdents64(fd, de, sizeof(de_buf)); 
            if (des < 0) {
                pr_notice("failure to read %s entry [%lx, %lu]%d\n", root, (unsigned long)de_buf, sizeof(de_buf), des);
                err = -1;
            } else if (0 == des) {
                //pr_notice("last entry\n");
                err = 0;
            } else {
                i = 0;
                do {
                    de = (struct linux_dirent64 *)&de_buf[i];
                    if (DT_DIR == de->d_type || DT_LNK == de->d_type) {
                        err = 0;
                        //pr_notice("found %s entry [%d, %d, %d]\n", de->d_name, de->d_reclen, de->d_type, des);
                        sprintf(lfname, "%s%s/label", root, de->d_name);
                        lfd = sys_open(lfname, O_RDONLY, S_IRUSR|S_IRGRP);
                        if (lfd) {
                            err = sys_read(lfd, lbl, sizeof(lbl) - 1);
                            sys_close(lfd);
                            if (err > 0) {
                                err--;
                                if (err >= sizeof(lbl)) {
                                    err = sizeof(lbl) - 1;
                                }
                                if (err > out_len) {
                                    err = out_len - 1;
                                }
                                lbl[err] = 0; 
                                //pr_notice("is %s [%s] match to %s\n", lfname, lbl, label);
                                if (0 == strncmp(lbl, label, out_len)) {
                                    des = 0;
                                    err = 0;
                                    //pr_notice("found\n");
                                    strncpy(out, de->d_name, out_len);
                                    break;
                                }
                            }
                        }
                    } else {
                        //pr_notice("%s entry [%d, %d, %d] isn't directory\n", de->d_name, de->d_reclen, de->d_type, des);
                    }
                    i += de->d_reclen;
                } while (i < des);
            }
        } while (des > 0);
        sys_close(fd);
    }

    set_fs(prev_fs);

    return err;
}
#endif

static ssize_t dock_switch_outs_mask_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

#if 0
    spin_lock_irqsave(&inf->rfkillpin_lock, inf->lock_flags);
    if (kstrtos32(buf, 10, &val))
    spin_unlock_irqrestore(&inf->rfkillpin_lock, inf->lock_flags);
#endif
//    pr_notice("already initialized %d..%d\n", ds->outs_base, ds->outs_base + ds->outs_num - 1);
//    ds->outs_base = ds->outs_num = -1;
//    schedule_delayed_work(&ds->vgpio_init_work, msecs_to_jiffies(1000));

    return sprintf(buf, "%x\n", ds->outs_mask_state); 
}

static ssize_t dock_switch_outs_mask_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    return sprintf(buf, "%x\n", ds->outs_mask_set); 
}

static ssize_t dock_switch_outs_mask_clr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    return sprintf(buf, "%x\n", ds->outs_mask_clr); 
}

static ssize_t dock_switch_outs_mask_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);
    int err, i;

    if (-1 == ds->outs_num || -1 == ds->outs_base) {
        pr_notice("vgpio still not ready or not exists\n");
        return -EINVAL;
    }

    if (ds->outs_mask_set) {
        pr_notice("busy\n");
        return -EBUSY;
    }

    spin_lock_irqsave(&ds->outs_mask_lock, ds->lock_flags);
    err = kstrtou32(buf, 16, &ds->outs_mask_set);
    spin_unlock_irqrestore(&ds->outs_mask_lock, ds->lock_flags);

    pr_notice("[%d,%d] <-- %x[%d]\n", ds->outs_base, ds->outs_num, ds->outs_mask_set, err);
    if (0 == err) {
        for (i = 0; ds->outs_mask_set; i++) {
            if (ds->outs_mask_set & 1) {
                if (gpio_is_valid(ds->outs_pins[i])) {
                    pr_notice("set out[%d] can%ssleep\n", ds->outs_base + i, (ds->outs_can_sleep)?" ":"'t ");
                    if (ds->outs_can_sleep) {
                        gpio_set_value_cansleep(ds->outs_pins[i], 1);
                    } else {
                        gpio_set_value(ds->outs_pins[i], 1); 
                    }
                    spin_lock_irqsave(&ds->outs_mask_lock, ds->lock_flags);
                    ds->outs_mask_state |= (1 << i);
                    spin_unlock_irqrestore(&ds->outs_mask_lock, ds->lock_flags);
                }
            }
            ds->outs_mask_set >>= 1;
        }
        err = count;
    }

    return err; 
}

static ssize_t dock_switch_outs_mask_clr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);
    int err, i;

    if (-1 == ds->outs_num || -1 == ds->outs_base) {
        pr_notice("vgpio still not ready or not exists\n");
        return -EINVAL;
    }

    if (ds->outs_mask_clr) {
        pr_notice("busy\n");
        return -EBUSY;
    }

    spin_lock_irqsave(&ds->outs_mask_lock, ds->lock_flags);
    err = kstrtou32(buf, 16, &ds->outs_mask_clr);
    spin_unlock_irqrestore(&ds->outs_mask_lock, ds->lock_flags);
    pr_notice("[%d,%d] <-- %x[%d]\n", ds->outs_base, ds->outs_num, ds->outs_mask_clr, err);
    if (0 == err) {
        for (i = 0; ds->outs_mask_clr; i++) {
            if (ds->outs_mask_clr & 1) {
                if (gpio_is_valid(ds->outs_pins[i])) {
                    pr_notice("clear out[%d] can%ssleep\n", ds->outs_base + i, (ds->outs_can_sleep)?" ":"'t ");
                    if (ds->outs_can_sleep) {
                        gpio_set_value_cansleep(ds->outs_pins[i], 0);
                    } else {
                        gpio_set_value(ds->outs_pins[i], 0); 
                    }
                    spin_lock_irqsave(&ds->outs_mask_lock, ds->lock_flags);
                    ds->outs_mask_state &= ~(1 << i);
                    spin_unlock_irqrestore(&ds->outs_mask_lock, ds->lock_flags);
                }
            }
            ds->outs_mask_clr >>= 1;
        }
        err = count;
    }

    return err; 
}

///////////////////////////////////////barak/////////////////////////////////////////////////
static ssize_t rs485_en_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    return sprintf(buf, "%d\n",gpio_get_value_cansleep(ds->mcu_outs_pins[RS48_GPIO_OFFSET]));
}

static ssize_t rs485_en_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    pr_err("rs485en store %d", (*buf-'0'));

    gpio_set_value_cansleep(ds->mcu_outs_pins[RS48_GPIO_OFFSET],(*buf-'0')); 

    return (count);
}

static ssize_t j1708_en_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);
 
    return sprintf(buf, "%d\n", gpio_get_value_cansleep(ds->mcu_outs_pins[J1708_GPIO_OFFSET])); ;
}

static ssize_t j1708_en_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    pr_err("j1708 store %d", (*buf-'0'));

    gpio_set_value_cansleep(ds->mcu_outs_pins[J1708_GPIO_OFFSET],(*buf-'0')); 

    return (count);
}
///////////////////////////////////////////////////////////////////////////////////////

static ssize_t dock_switch_dbg_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    return sprintf(buf, "%d\n", (int)ds->dock_type); 
}

static ssize_t dock_switch_dbg_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);
    int err, i;

    err = kstrtos32(buf, 10, &i);

    if (0 == err) {
        ds->dock_type = i;
        err = count;
    }
    ds->sched_irq = SWITCH_DOCK | SWITCH_IGN;
    schedule_work(&ds->work);

    return err; 
}

static int gpc_lable_match(struct gpio_chip *gpc, void *lbl)
{
	return !strcmp(gpc->label, lbl);
}

////////////////////////////////////////////////////////////////////////////////////////

static void mcu_gpio_init_work(struct work_struct *work)
{
    struct dock_switch_device *ds = container_of(work, struct dock_switch_device, mcu_gpio_init_work.work);
    int  err =0;
//    int fd;
//    char gpiochip_dir[32];
      char gp_file[64];
//    mm_segment_t prev_fs;
    struct gpio_chip *gpc;

    gpc = gpiochip_find("vgpio_mcu", gpc_lable_match);

    if (gpc) {
        ds->mcu_gpio_base = gpc->base;
        ds->mcu_gpio_num  = gpc->ngpio;
        ds->j1708en_vgpio_num = ds->mcu_gpio_base + J1708_GPIO_OFFSET;
        ds->rs485en_vgpio_num = ds->mcu_gpio_base + RS48_GPIO_OFFSET;
        
    pr_err("dock_switch_device %s %d num = %d\n", gpc->label, ds->mcu_gpio_base, ds->mcu_gpio_num);

            if (gpio_is_valid(ds->j1708en_vgpio_num )) {
                
                sprintf(gp_file, "mcu_out_%d",ds->j1708en_vgpio_num);
                pr_err( "%s %d %p", gp_file, ds->j1708en_vgpio_num,ds->pdev);
               err = devm_gpio_request(ds->pdev,ds->j1708en_vgpio_num, gp_file);
                if (err) {
                    pr_err("virtual out [%d] is busy!\n", (ds->j1708en_vgpio_num));
                } else {
                    ds->mcu_outs_pins[J1708_GPIO_OFFSET] = ds->j1708en_vgpio_num;
                    gpio_direction_output(ds->mcu_outs_pins[J1708_GPIO_OFFSET],0);
                    gpio_export(ds->mcu_outs_pins[J1708_GPIO_OFFSET], 0);
                }
            }

            if (gpio_is_valid(ds->rs485en_vgpio_num )) {           
                sprintf(gp_file, "mcu_out_%d",ds->rs485en_vgpio_num);
                pr_err( "%s %d %p", gp_file, ds->rs485en_vgpio_num,ds->pdev);
               err = devm_gpio_request(ds->pdev,ds->rs485en_vgpio_num, gp_file);
                if (err) {
                    pr_err("virtual out [%d] is busy!\n", (ds->rs485en_vgpio_num));
                } else {
                    ds->mcu_outs_pins[RS48_GPIO_OFFSET] = ds->rs485en_vgpio_num;
                    gpio_direction_output(ds->mcu_outs_pins[RS48_GPIO_OFFSET],0);
                    gpio_export(ds->mcu_outs_pins[RS48_GPIO_OFFSET], 0);
                }
            }
        
        return;
    }
    pr_err("dock_switch_device %s %d num = %d\n", gpc->label, ds->mcu_gpio_base, ds->mcu_gpio_num);

    schedule_delayed_work(&ds->mcu_gpio_init_work, msecs_to_jiffies(1000));
}
///////////////////////////////////////////////////////////////////////////////////////////////

static void swithc_dock_outs_init_work(struct work_struct *work)
{
    struct dock_switch_device *ds = container_of(work, struct dock_switch_device, vgpio_init_work.work);
    static int secs = 60;
    int err = -1, i = 0;
//    int fd;
//    char gpiochip_dir[32];
    char gp_file[64];
//    mm_segment_t prev_fs;
    struct gpio_chip *gpc;

#if 1
    if ((-1 != ds->outs_num && -1 != ds->outs_base) || (secs <= 0)) {
        secs = 60;
        return;
    }

    gpc = gpiochip_find("vgpio_out", gpc_lable_match);

    if (gpc) {
        ds->outs_base = gpc->base;
        ds->outs_num  = gpc->ngpio;
        ds->outs_can_sleep  = gpc->can_sleep;
        for (i = 0; i < ds->outs_num; i++) {
            if (gpio_is_valid(ds->outs_base + i)) {
                sprintf(gp_file, "virtual_out_%d", i);
                err = devm_gpio_request(ds->pdev, ds->outs_base + i, gp_file);
                if (err) {
                    pr_err("virtual out [%d] is busy!\n", ds->outs_base + i);
                } else {
                    ds->outs_pins[i] = ds->outs_base + i;
                    gpio_direction_output(ds->outs_pins[i], 0);
                    gpio_export(ds->outs_pins[i], 0);
                }
            }
        }
        pr_notice("%s %d..%d\n", gpc->label, ds->outs_base, ds->outs_base + ds->outs_num - 1);
        return;
    }

#else
    if ((-1 != ds->outs_num && -1 != ds->outs_base) || (secs <= 0)) {
        secs = 60;
        return;
    }

    secs--;
    err = find_dir_by_label("/sys/class/gpio/", "vgpio_out", gpiochip_dir, (sizeof(gpiochip_dir) - sizeof(gpiochip_dir[0])) / sizeof(gpiochip_dir[0])); 

    if (0 == err) {
        prev_fs = get_fs();
        set_fs(get_ds());
        sprintf(gp_file, "/sys/class/gpio/%s/base", gpiochip_dir);
        fd = sys_open(gp_file, O_RDONLY, S_IRUSR|S_IRGRP);
        if (fd) {
            err = sys_read(fd, gp_file, 8); 
            sys_close(fd);
            if (err > 0) {
                err--;
                if (err > 7) {
                    err = 7;
                }
                gp_file[err] = 0; 
                ds->outs_base = simple_strtoul(gp_file, 0, 10);
            }
        }
        sprintf(gp_file, "/sys/class/gpio/%s/ngpio", gpiochip_dir);
        fd = sys_open(gp_file, O_RDONLY, S_IRUSR|S_IRGRP);
        if (fd) {
            err = sys_read(fd, gp_file, 8); 
            sys_close(fd);
            if (err > 0) {
                err--;
                if (err > 7) {
                    err = 7;
                }
                gp_file[err] = 0; 
                ds->outs_num = simple_strtoul(gp_file, 0, 10);
                if (ds->outs_num > VGPIO_MAX) {
                    ds->outs_num = VGPIO_MAX;
                }
            }
        }

        pr_notice("%s %d..%d\n", gpiochip_dir, ds->outs_base, ds->outs_base + ds->outs_num - 1);

        set_fs(prev_fs);

        for (i = 0; i < ds->outs_num; i++) {
            if (gpio_is_valid(ds->outs_base + i)) {
                sprintf(gp_file, "virtual_out_%d", i);
                err = devm_gpio_request(ds->pdev, ds->outs_base + i, gp_file);
                if (err) {
                    pr_err("virtual out [%d] is busy!\n", ds->outs_base + i);
                } else {
                    ds->outs_pins[i] = ds->outs_base + i;
                    gpio_direction_output(ds->outs_pins[i], 0);
                    gpio_export(ds->outs_pins[i], 0);
                }
            }
        }
    }
#endif

    schedule_delayed_work(&ds->vgpio_init_work, msecs_to_jiffies(1000));
}

// Vladimir:
// Portable and fix tab8 otehrwize smart cam that alwys fix
//
#define DOCK_SWITCH_PORTABLE 1

static int dock_switch_probe(struct platform_device *pdev)
{
	int err = -1;
	struct dock_switch_device *ds;
    struct device *dev = &pdev->dev;
    struct device_node *np;
    struct pinctrl_state *pctls;
    int	compatible = DOCK_SWITCH_PORTABLE;
    const char *c;
	
    np = dev->of_node;
    if (!np) {
        pr_err("failure to find device tree\n");
        return -EINVAL;
    }
	
	ds = devm_kzalloc(dev, sizeof(struct dock_switch_device), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

    c = of_get_property(np, "compatible", NULL);
    if (c && 0 == strncmp("mcn,fixed-dock-switch", c, sizeof("mcn,fixed-dock-switch") - sizeof(char))) {
        compatible = !DOCK_SWITCH_PORTABLE;
    }
    pr_notice("TAB8 %s \n", (DOCK_SWITCH_PORTABLE == compatible)?"portable":"fixed");

    ds->state = 0;
    do {
        ds->pctl = devm_pinctrl_get(dev);
        if (IS_ERR(ds->pctl)) {
            if (PTR_ERR(ds->pctl) == -EPROBE_DEFER) {
                dev_err(dev, "pin ctl critical error!\n");
                err = -EPROBE_DEFER;
                break;
            }

            pr_notice("pin control isn't used\n");
            ds->pctl = 0;
        }

        if (ds->pctl) {
            pctls = pinctrl_lookup_state(ds->pctl, "dock_pins_active");
            if (IS_ERR(pctls)) {
                dev_err(dev, "failure to get pinctrl active state\n");
                err = PTR_ERR(pctls);
                break;
            }
            err = pinctrl_select_state(ds->pctl, pctls);
            if (err) {
                dev_err(dev, "failure to set pinctrl active state\n");
                break;
            }
        }

        mutex_init(&ds->lock);
        ds->irq_ack = 0;
        ds->usb_switch_pin = of_get_named_gpio_flags(np,"mcn,usb-switch-pin", 0, (enum of_gpio_flags *)&ds->usb_switch_l);
        if (gpio_is_valid(ds->usb_switch_pin)) {
            ds->usb_switch_l = (OF_GPIO_ACTIVE_LOW != ds->usb_switch_l);
            err = devm_gpio_request(dev, ds->usb_switch_pin, "usb_switch");
            if (err) {
                ds->usb_switch_pin = -1;
                pr_err("usb switch pin is busy!\n");
            } else {
                gpio_direction_output(ds->usb_switch_pin, !!!ds->usb_switch_l);
                gpio_export(ds->usb_switch_pin, 0);
            }
        }
        ds->mic_sw_pin = of_get_named_gpio_flags(np,"mcn,mic-switch-pin", 0, (enum of_gpio_flags *)&ds->mic_sw_l);
        if (gpio_is_valid(ds->mic_sw_pin)) {
            ds->mic_sw_l = (OF_GPIO_ACTIVE_LOW != ds->mic_sw_l);
            err = devm_gpio_request(dev, ds->mic_sw_pin, "mic_switch");
            if (err) {
                ds->mic_sw_pin = -1;
                pr_err("usb switch pin is busy!\n");
            } else {
                gpio_direction_output(ds->mic_sw_pin, !!!ds->mic_sw_l);
                gpio_export(ds->mic_sw_pin, 0);
            }
        }

        err = of_get_named_gpio_flags(np, "mcn,dock-pin", 0, (enum of_gpio_flags *)&ds->dock_active_l);
        if (!gpio_is_valid(err)) {
            pr_err("ivalid docking pin\n");
            err = -EINVAL;
            break;
        }
        ds->dock_pin = err;
        ds->dock_active_l = !ds->dock_active_l;

        if (gpio_is_valid(ds->dock_pin)) {
            err = devm_gpio_request(dev, ds->dock_pin, "dock-state");
            if (err < 0) {
                pr_err("failure to request the gpio[%d]\n", ds->dock_pin);
                break;
            }
            err = gpio_direction_input(ds->dock_pin);
            if (err < 0) {
                pr_err("failure to set direction of the gpio[%d]\n", ds->dock_pin);
                break;
            }
            gpio_export(ds->dock_pin, 1);
        }

        wake_lock_init(&ds->wlock, WAKE_LOCK_SUSPEND, "switch_dock_wait_lock");
		if (DOCK_SWITCH_PORTABLE == compatible) {
            ds->usb_psy = power_supply_get_by_name("usb");

            INIT_WORK(&ds->work, dock_switch_work_func);

			pr_notice("dock active level %s\n", (ds->dock_active_l)?"high":"low");
			err = of_get_named_gpio_flags(np, "mcn,ign-pin", 0, (enum of_gpio_flags *)&ds->ign_active_l);
			if (!gpio_is_valid(err)) {
				pr_err("ivalid ignition pin\n");
				err = -EINVAL;
				break;
			}
			ds->ign_pin = err;
			ds->ign_active_l = !ds->ign_active_l;
			pr_notice("ignition active level %s\n", (ds->ign_active_l)?"high":"low");

            ds->dock_type = e_dock_type_unspecified;

			if (gpio_is_valid(ds->dock_pin)) {
				set_aml_enable(ds, FORBID_EXT_SPKR);
				ds->dock_irq = gpio_to_irq(ds->dock_pin);
				if (ds->dock_irq < 0) {
					pr_err("failure to request gpio[%d] irq\n", ds->dock_pin);
				} else {
					err = devm_request_irq(dev, ds->dock_irq, dock_switch_irq_handler,
										   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_DISABLED,
										   pdev->name, ds);
					if (!err) {
						disable_irq_nosync(ds->dock_irq);
					} else {
						pr_err("failure to request irq[%d] irq -- polling available\n", ds->dock_irq);
					}
				}

                ds->otg_en_pin = of_get_named_gpio_flags(np,"mcn,otg-en-pin", 0, (enum of_gpio_flags *)&ds->otg_en_l);
                if (gpio_is_valid(ds->otg_en_pin)) {
                    ds->otg_en_l = (OF_GPIO_ACTIVE_LOW != ds->otg_en_l);
                    err = devm_gpio_request(dev, ds->otg_en_pin, "otg_en");
                    if (err) {
                        ds->otg_en_pin = -1;
                        pr_err("usb switch pin is busy!\n");
                    } else {
                        gpio_direction_output(ds->otg_en_pin, !!!ds->otg_en_l);
                        gpio_export(ds->otg_en_pin, 0);
                    }
                }
			}

			if (gpio_is_valid(ds->ign_pin)) {
				err = devm_gpio_request(dev, ds->ign_pin, "ignition-state");
				if (err < 0) {
					pr_err("failure to request the gpio[%d]\n", ds->ign_pin);
					break;
				}
				err = gpio_direction_input(ds->ign_pin);
				if (err < 0) {
					pr_err("failure to set direction of the gpio[%d]\n", ds->ign_pin);
					break;
				}
				gpio_export(ds->ign_pin, 0);
				ds->ign_irq = gpio_to_irq(ds->ign_pin);
				if (ds->ign_irq < 0) {
					pr_err("failure to request gpio[%d] irq\n", ds->ign_pin);
				} else {
					err = devm_request_irq(dev, ds->ign_irq, dock_switch_irq_handler,
										   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_DISABLED,
										   pdev->name, ds);
					if (!err) {
						disable_irq_nosync(ds->ign_irq);
					} else {
						pr_err("failure to request irq[%d] irq -- polling available\n", ds->ign_irq);
					}
				}
			}
			if (!(ds->dock_irq < 0 && ds->ign_irq < 0)) {
				device_init_wakeup(dev, 1);
			}
		} else {
            ds->dock_irq = gpio_to_irq(ds->dock_pin);
            if (ds->dock_irq < 0) {
                pr_err("failure to request gpio[%d] irq\n", ds->dock_pin);
            } else {
                err = devm_request_irq(dev, ds->dock_irq, dock_switch_irq_handler,
                                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_DISABLED,
                                       pdev->name, ds);
                if (!err) {
                    disable_irq_nosync(ds->dock_irq);
                } else {
                    pr_err("failure to request irq[%d] irq -- polling available\n", ds->dock_irq);
                }
            }

			INIT_WORK(&ds->work, dock_switch_work_func_fix);

			ds->ignition_notifier.notifier_call = dock_switch_ign_callback;
			err = gpio_in_register_notifier(&ds->ignition_notifier);
			if (err) {
				pr_err("failure to register remount notifier [%d]\n", err);
				err = -EINVAL;
				break;
			}
		}
        ds->sdev.name = "dock";
        ds->sdev.print_state = dock_switch_print_state;
        err = switch_dev_register(&ds->sdev);
        if (err < 0) {
            pr_err("err_register_switch\n");
            break;
        }
    	err = device_create_file((&ds->sdev)->dev, &dev_attr_ampl_enable);
    	if (err < 0) {
            pr_err("err0r create amplifier file\n");
            break;
        }

        ds->pdev = dev;
        dev_set_drvdata(dev, ds);
        ds->sched_irq = SWITCH_DOCK | SWITCH_IGN;
        pr_notice("sched reason[%u]\n", ds->sched_irq);
        schedule_work(&ds->work);

        spin_lock_init(&ds->outs_mask_lock);
        memset(ds->outs_pins, -1, sizeof(ds->outs_pins));
        ds->outs_mask_clr = ds->outs_mask_set = ds->outs_mask_state = 0;
        ds->outs_num = ds->outs_base = -1;
        snprintf(ds->attr_outs_mask_state.name, sizeof(ds->attr_outs_mask_state.name) - 1, "outs_mask_state");
        ds->attr_outs_mask_state.attr.attr.name = ds->attr_outs_mask_state.name;
        ds->attr_outs_mask_state.attr.attr.mode = S_IRUGO;//|S_IWUGO;
        ds->attr_outs_mask_state.attr.show = dock_switch_outs_mask_state_show;
        ds->attr_outs_mask_state.attr.store = 0;
        sysfs_attr_init(&ds->attr_outs_mask_state.attr.attr);
        device_create_file((&ds->sdev)->dev, &ds->attr_outs_mask_state.attr);

        snprintf(ds->attr_outs_mask_set.name, sizeof(ds->attr_outs_mask_set.name) - 1, "outs_mask_set");
        ds->attr_outs_mask_set.attr.attr.name = ds->attr_outs_mask_set.name;
        ds->attr_outs_mask_set.attr.attr.mode = S_IRUGO|S_IWUGO;
        ds->attr_outs_mask_set.attr.show = dock_switch_outs_mask_set_show;
        ds->attr_outs_mask_set.attr.store = dock_switch_outs_mask_set_store;
        sysfs_attr_init(&ds->attr_outs_mask_set.attr.attr);
        device_create_file((&ds->sdev)->dev, &ds->attr_outs_mask_set.attr);

        snprintf(ds->attr_outs_mask_clr.name, sizeof(ds->attr_outs_mask_clr.name) - 1, "outs_mask_clr");
        ds->attr_outs_mask_clr.attr.attr.name = ds->attr_outs_mask_clr.name;
        ds->attr_outs_mask_clr.attr.attr.mode = S_IRUGO|S_IWUGO;
        ds->attr_outs_mask_clr.attr.show = dock_switch_outs_mask_clr_show;
        ds->attr_outs_mask_clr.attr.store = dock_switch_outs_mask_clr_store;
        sysfs_attr_init(&ds->attr_outs_mask_clr.attr.attr);
        device_create_file((&ds->sdev)->dev, &ds->attr_outs_mask_clr.attr);

        snprintf(ds->attr_dbg_state.name, sizeof(ds->attr_dbg_state.name) - 1, "dbg_state");
        ds->attr_dbg_state.attr.attr.name = ds->attr_dbg_state.name;
        ds->attr_dbg_state.attr.attr.mode = S_IRUGO|S_IWUGO;
        ds->attr_dbg_state.attr.show = dock_switch_dbg_state_show;
        ds->attr_dbg_state.attr.store = dock_switch_dbg_state_store;
        sysfs_attr_init(&ds->attr_dbg_state.attr.attr);
        device_create_file((&ds->sdev)->dev, &ds->attr_dbg_state.attr);

        /////////////////////////////////////////////////////////////////////////////////////////////

        snprintf(ds->attr_J1708_en.name, sizeof(ds->attr_J1708_en.name) - 1, "J1708_en");
        ds->attr_J1708_en.attr.attr.name  = ds->attr_J1708_en.name;
        ds->attr_J1708_en.attr.attr.mode = S_IRUGO|S_IWUGO;//666
        ds->attr_J1708_en.attr.show = j1708_en_state_show;
        ds->attr_J1708_en.attr.store = j1708_en_state_store;
        sysfs_attr_init(&ds->attr_J1708_en.attr.attr);
        device_create_file((&ds->sdev)->dev, &ds->attr_J1708_en.attr);

        snprintf(ds->attr_rs485_en.name, sizeof(ds->attr_rs485_en.name) - 1, "rs485_en");
        ds->attr_rs485_en.attr.attr.name = ds->attr_rs485_en.name;
        ds->attr_rs485_en.attr.attr.mode = S_IRUGO|S_IWUGO;//666
        ds->attr_rs485_en.attr.show = rs485_en_state_show;
        ds->attr_rs485_en.attr.store = rs485_en_state_store;
        sysfs_attr_init(&ds->attr_rs485_en.attr.attr);
        device_create_file((&ds->sdev)->dev, &ds->attr_rs485_en.attr);

        INIT_DELAYED_WORK(&ds->mcu_gpio_init_work, mcu_gpio_init_work);
        schedule_delayed_work(&ds->mcu_gpio_init_work, msecs_to_jiffies(100));

        ////////////////////////////////////////////////////

        INIT_DELAYED_WORK(&ds->vgpio_init_work, swithc_dock_outs_init_work);
        schedule_delayed_work(&ds->vgpio_init_work, msecs_to_jiffies(100));

        return 0;
    } while (0);

	if (ds->dock_irq)
        devm_free_irq(&pdev->dev, ds->dock_irq, ds);
    if (ds->ign_irq)
        devm_free_irq(&pdev->dev, ds->ign_irq, ds);
	if (gpio_is_valid(ds->dock_pin))
        devm_gpio_free(&pdev->dev, ds->dock_pin);
    if (gpio_is_valid(ds->ign_pin))
        devm_gpio_free(&pdev->dev, ds->ign_pin);
	devm_kfree(dev, ds);

	pr_err("failure\n");

    return 0;
}

static int dock_switch_remove(struct platform_device *pdev)
{
	struct dock_switch_device *ds = platform_get_drvdata(pdev);
    int i;

    cancel_work_sync(&ds->work);

	device_remove_file((&ds->sdev)->dev, &dev_attr_ampl_enable);
    switch_dev_unregister(&ds->sdev);

    if (device_may_wakeup(&pdev->dev))
        device_wakeup_disable(&pdev->dev);
	if (ds->ign_irq) {
        disable_irq_nosync(ds->ign_irq);
		devm_free_irq(&pdev->dev, ds->ign_irq, ds);
    }
    if (ds->dock_irq) {
        disable_irq_nosync(ds->dock_irq);
        devm_free_irq(&pdev->dev, ds->dock_irq, ds);
    }

	if (gpio_is_valid(ds->dock_pin)) 
		devm_gpio_free(&pdev->dev, ds->dock_pin);
	if (gpio_is_valid(ds->ign_pin))
		devm_gpio_free(&pdev->dev, ds->ign_pin);

    for (i = 0; i < ds->outs_num; i++) {
        if (gpio_is_valid(ds->outs_pins[i])) {
            devm_gpio_free(&pdev->dev, ds->outs_pins[i]);
        }
    }

    wake_lock_destroy(&ds->wlock);
    dev_set_drvdata(&pdev->dev, 0);

	devm_kfree(&pdev->dev, ds);

    return 0;
}

static int dock_switch_suspend(struct device *dev)
{
	struct dock_switch_device *ds = dev_get_drvdata(dev);

	cancel_work_sync(&ds->work);

    if (device_may_wakeup(dev)) {
        if (ds->ign_irq) {
            pr_notice("enable wake source IGN[%d]\n", ds->ign_irq);
            enable_irq_wake(ds->ign_irq);
        }
        if (ds->dock_irq && (e_dock_type_smart != ds->dock_type)) {
            pr_notice("enable wake source DOCK[%d]\n", ds->dock_irq);
            enable_irq_wake(ds->dock_irq);
        }
    }

    return 0;
}

static int dock_switch_resume(struct device *dev)
{
	struct dock_switch_device *ds = dev_get_drvdata(dev);

    if (device_may_wakeup(dev)) {
        if (ds->ign_irq) {
            enable_switch_irq(ds->ign_irq, 0);
            pr_notice("disable wake source IGN[%d]\n", ds->ign_irq);
            disable_irq_wake(ds->ign_irq);
        }
        if (ds->dock_irq && (e_dock_type_smart != ds->dock_type)) {
            enable_switch_irq(ds->dock_irq, 0);
            pr_notice("disable wake source DOCK[%d]\n", ds->dock_irq);
            disable_irq_wake(ds->dock_irq);
        }
    }

    ds->sched_irq = SWITCH_IGN;
    if (e_dock_type_smart != ds->dock_type) {
        ds->sched_irq |= SWITCH_DOCK;
    }
    pr_notice("sched reason[%u]\n", ds->sched_irq); 
	schedule_work(&ds->work);

	return 0;
}

static const struct dev_pm_ops dock_switch_pm_ops = {
	.suspend	= dock_switch_suspend,
	.resume		= dock_switch_resume,
};

static struct of_device_id dock_switch_match[] = {
	{ .compatible = "mcn,dock-switch", },
    { .compatible = "mcn,fixed-dock-switch", },
	{},
};

static struct platform_driver dock_switch_platform_driver = {
	.probe = dock_switch_probe,
	.remove = dock_switch_remove, 
	.driver = {
		.name = "dock",
		.owner = THIS_MODULE, 
        .of_match_table = dock_switch_match,
        .pm = &dock_switch_pm_ops,
	},
};

module_platform_driver(dock_switch_platform_driver);

MODULE_DESCRIPTION("Dock switch Monitor");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimir.zatulovsky@micronet-inc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tab8-dock-switch");

