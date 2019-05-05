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
    int     dock_irq;
    int 	ign_irq;
	int	    dock_active_l;
	int	    ign_active_l;
    int     usb_switch_l;
    int     otg_en_l;
    unsigned sched_irq;
	int	    state;
    struct  wake_lock wlock;
	struct  device*	pdev;
    struct 	notifier_block   ignition_notifier;
    int		virt_gpio_offset;
    int     virt_init;
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
    int mcu_pins[MCU_GPIO_MAX];
    unsigned mcu_gpio_base;
    unsigned mcu_gpio_num;
    unsigned j1708en_vgpio_num;
    unsigned rs485en_vgpio_num;
    struct delayed_work	mcu_gpio_init_work;//used to initiate the mcu gpios

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

/////////////////////////////////////////////////////////////////////////////////////////////////
static RAW_NOTIFIER_HEAD(cradle_connected_chain);
static DEFINE_RAW_SPINLOCK(cradle_connected_chain_lock);

enum NOTIFICATION_REASONS
{
    CONNETED_TO_BASIC_CRADLE = 1,
    DISCONNECTED_FROM_BASIC_CRADLE = 2,
    CONNETED_TO_SMART_CRADLE = 3,
    DISCONNECTED_FROM_SMART_CRADLE = 4,
};

void cradle_connect_notify(unsigned long reason, void *arg)
{
    unsigned long flags;

    raw_spin_lock_irqsave(&cradle_connected_chain_lock, flags);
    raw_notifier_call_chain(&cradle_connected_chain, reason, 0);
    raw_spin_unlock_irqrestore(&cradle_connected_chain_lock, flags);
}

int cradle_register_notifier(struct notifier_block *nb)
{
    unsigned long flags;
    int err;

    raw_spin_lock_irqsave(&cradle_connected_chain_lock, flags);
    err = raw_notifier_chain_register(&cradle_connected_chain, nb);
    raw_spin_unlock_irqrestore(&cradle_connected_chain_lock, flags);

    return err;
}
EXPORT_SYMBOL(cradle_register_notifier);

int cradle_unregister_notifier(struct notifier_block *nb)
{
    unsigned long flags;
    int err;

    raw_spin_lock_irqsave(&cradle_connected_chain_lock, flags);
    err = raw_notifier_chain_unregister(&cradle_connected_chain, nb);
    raw_spin_unlock_irqrestore(&cradle_connected_chain_lock, flags);

    return err;
}
EXPORT_SYMBOL(cradle_unregister_notifier);
/////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    pr_notice("%d HZ %lld\n", 1000 * pulses / interim, ktime_to_ms(ktime_get()));
    return 1000 * pulses / interim;
}

#define HYST_PATTERN(p1, p2) ((p2) - (((p2) - (p1)) >> 1))
//#define HYST_PATTERN(p1, p2) ((p1) + (((p2) - (p1)) >> 2))
static inline int freq2pattern(int freq)
{
    if (freq < IG_LOW_PATTERN - pulses2freq(2, PATERN_INTERIM)) {//HYST_PATTERN(BASIC_PATTERN, IG_LOW_PATTERN)) {
        return BASIC_PATTERN;
    } else if (freq <= HYST_PATTERN(IG_LOW_PATTERN, IG_HI_PATTERN) && IG_LOW_PATTERN - pulses2freq(2, PATERN_INTERIM) <= freq) {
        return IG_LOW_PATTERN;
    } else if (freq <= HYST_PATTERN(IG_HI_PATTERN, SMART_PATTERN) && HYST_PATTERN(IG_LOW_PATTERN, IG_HI_PATTERN) < freq) {
        return IG_HI_PATTERN;
    }

    return SMART_PATTERN; 
}

static void dock_switch_work_func(struct work_struct *work) 
{
	struct dock_switch_device *ds = container_of(work, struct dock_switch_device, work);
    long long timer = ktime_to_ms(ktime_get());
    int val = 0, act = 0;
    union power_supply_propval prop = {0,};
    struct irq_desc *desc;

    if (!ds->usb_psy) {
        pr_notice("usb power supply not ready %lld\n", ktime_to_ms(ktime_get()));
        ds->usb_psy = power_supply_get_by_name("usb");
    }

    if (e_dock_type_basic != ds->dock_type) {
        val = wait_for_stable_signal(ds->ign_pin, DEBOUNCE_INTERIM + PATERN_INTERIM);
        val = pulses2freq(val, PATERN_INTERIM);
        val = freq2pattern(val);
        pr_notice("pattern[%d, %d] [%lld]%lld\n", val, gpio_get_value(ds->ign_pin), timer, ktime_to_ms(ktime_get()));
       	if (BASIC_PATTERN == val) {
            if (gpio_is_valid(ds->dock_pin)) {
				if (e_dock_type_smart == ds->dock_type && ds->ign_active_l == gpio_get_value(ds->ign_pin)) {
					pr_notice("smart cradle unplagged %lld [dock_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->dock_pin));
					if (ds->usb_psy) {
                        pr_notice("notify usb host about unplug cradel %lld\n", ktime_to_ms(ktime_get()));
						prop.intval = 0;
						//ds->usb_psy->set_property(ds->usb_psy, POWER_SUPPLY_PROP_BOOST_CURRENT, &prop);
                        power_supply_set_usb_otg(ds->usb_psy, prop.intval);
					}
					ds->dock_type = e_dock_type_unspecified;
					ds->sched_irq |= SWITCH_DOCK;
					act = 1;
				} else if (e_dock_type_smart != ds->dock_type && ds->dock_active_l != gpio_get_value(ds->dock_pin)) {
                    if (power_supply_is_system_supplied() > 0) {
                        pr_notice("basic cradle attempt to be plugged %lld  [dock_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->dock_pin)); 
                        ds->dock_type = e_dock_type_basic;
                        act = 1;
                    } else {
                        pr_notice("input power isn't supplied to system %lld  [dock_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->dock_pin)); 
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
						pr_notice("switch usb connector %lld\n", ktime_to_ms(ktime_get()));
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
//                ds->usb_psy->set_property(ds->usb_psy, POWER_SUPPLY_PROP_BOOST_CURRENT, &prop);
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
            	desc = irq_to_desc(ds->dock_irq);
            	if(0 == desc->depth) {
                	disable_irq_nosync(ds->dock_irq);
                    pr_notice("disable dock irq[%d] %lld\n", ds->dock_irq, ktime_to_ms(ktime_get()));
                }
                ds->sched_irq &= ~SWITCH_DOCK;
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

    if (e_dock_type_basic == ds->dock_type) {
        if (gpio_is_valid(ds->dock_pin)) {
            if (ds->dock_active_l == gpio_get_value(ds->dock_pin) ) {
                pr_notice("basic cradle plagged %lld\n", ktime_to_ms(ktime_get()));
                val |= SWITCH_DOCK;
            } else {
                pr_notice("basic cradle unplagged %lld\n", ktime_to_ms(ktime_get()));
                ds->dock_type = e_dock_type_unspecified;
            }
        }

        if (gpio_is_valid(ds->ign_pin)) {
            if (ds->ign_active_l == gpio_get_value(ds->ign_pin) ) {
                val |= SWITCH_IGN;
            }
        }
    }

    // interrupts handled
    if (ds->sched_irq & SWITCH_DOCK) {
//        pr_notice("enable dock monitor irq[%d]\n", ds->dock_irq);
    	desc = irq_to_desc(ds->dock_irq);
    	if(desc->depth > 0) {
    		ds->sched_irq &= ~SWITCH_DOCK;
    		enable_irq(ds->dock_irq);
    	}
    }

    if (ds->sched_irq & SWITCH_IGN) {
//        pr_notice("enable ignition monitor irq[%d]\n", ds->ign_irq);
    	desc = irq_to_desc(ds->ign_irq);
    	if(desc->depth > 0) {
        	ds->sched_irq &= ~SWITCH_IGN;
        	enable_irq(ds->ign_irq);
        }
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

static void dock_switch_work_virt_func(struct work_struct *work)
{
	struct dock_switch_device *ds  = container_of(work, struct dock_switch_device, work);
    int val = 0, err;
    

    if (VIRT_GPIO_OFF == ds->virt_init)
    	return;

	if (VIRT_GPIO_INIT == ds->virt_init) {
		if(gpio_is_valid(ds->ign_pin)) {
			err = devm_gpio_request(ds->pdev, ds->ign_pin, "ignition-state");//pdev->name);
			if (err < 0) {
				pr_err("failure to request the gpio[%d]\n", ds->ign_pin);
				return;
			} else {
				ds->virt_init = VIRT_GPIO_ON;
				gpio_direction_input(ds->ign_pin);
				gpio_export(ds->ign_pin, 0);
		        pr_notice("virt gpio is initialized\n");
			}
		} else {
			pr_err("ign gpio [%d] is not valid\n", ds->ign_pin);
			return;
		}
	}

	val |= SWITCH_ODOCK;
    if (ds->ign_active_l == gpio_get_value(ds->ign_pin) ) {
        val |= SWITCH_IGN;
    }

    if (ds->state != val) {
        pr_notice("ignition changed to %d\n", val);
		ds->state = val;
		switch_set_state(&ds->sdev, val);
	}
}

static irqreturn_t dock_switch_irq_handler(int irq, void *arg)
{
	struct dock_switch_device *ds = (struct dock_switch_device *)arg;

//    pr_notice("pins[%d]\n", irq);
    disable_irq_nosync(irq);

    if (irq == ds->dock_irq) {
        ds->sched_irq |= SWITCH_DOCK;
    }

    if (irq == ds->ign_irq) {
        ds->sched_irq |= SWITCH_IGN;
    }

    schedule_work(&ds->work);

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
    unsigned long val = 0;

    if (0 == reason) {
        pr_notice("%ld\n", reason);
		ds->virt_init = VIRT_GPIO_INIT;
   		schedule_work(&ds->work);
    } else if (1 == reason) {
        if (arg) {
        	val = *((unsigned long*)arg);
            pr_notice("%ld - %ld\n", reason, val);
        	if (val & (1 << ds->virt_gpio_offset)) {
        		schedule_work(&ds->work);
        	}
        }
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
    char gp_file[64];
    mm_segment_t prev_fs;
    int fd = 0;
    int err = 0;

    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    prev_fs = get_fs();
	set_fs(get_ds());

    pr_err("rs485en : /sys/class/gpio/gpio%d/value", ds->rs485en_vgpio_num);

    sprintf(gp_file, "/sys/class/gpio/gpio%d/value", ds->rs485en_vgpio_num);
    fd = sys_open(gp_file, O_RDWR, S_IRUSR|S_IRGRP);

    if(fd)
    {
        err = sys_read(fd, gp_file, 8); 
        sys_close(fd);
        
        if(err)
        {
            pr_err("error! couldn't connect to mcu on gpio %u ", ds->rs485en_vgpio_num);
        }
    }

    set_fs(prev_fs);

    return sprintf(buf,"%d\n", gp_file[0] - '0');
}

static ssize_t rs485_en_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int err = ENOENT; 
    char gp_file[64];
    int fd = 0;
    mm_segment_t prev_fs;

    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    prev_fs = get_fs();
	set_fs(get_ds());

    pr_err("rs485en : /sys/class/gpio/gpio%d/value", ds->rs485en_vgpio_num);
 
    sprintf(gp_file, "/sys/class/gpio/gpio%d/value", ds->rs485en_vgpio_num);
    fd = sys_open(gp_file, O_RDWR, S_IRUSR|S_IRGRP);

    if(fd)
    {

        err = sys_write(fd, buf, 1); 
        sys_close(fd);
        
        if(err)
        {
            pr_err("error! couldn't connect to mcu");
        }
    }

    set_fs(prev_fs);

    return (err);
}

static ssize_t j1708_en_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    char gp_file[64];
    mm_segment_t prev_fs;
    int fd = 0;
    int err = 0;

    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    prev_fs = get_fs();
	set_fs(get_ds());

    pr_err("j1708en : /sys/class/gpio/gpio%d/value", ds->j1708en_vgpio_num);

    sprintf(gp_file, "/sys/class/gpio/gpio%d/value", ds->j1708en_vgpio_num);
    fd = sys_open(gp_file, O_RDWR, S_IRUSR|S_IRGRP);

    if(fd)
    {
        err = sys_read(fd, gp_file, 8); 
        sys_close(fd);
        
        if(err)
        {
            pr_err("error! couldn't connect to mcu on gpio %u err = %u",ds->j1708en_vgpio_num,err);
        }
    }

    set_fs(prev_fs);

    return sprintf(buf,"%d\n", gp_file[0] - '0');
}

static ssize_t j1708_en_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int err = ENOENT; 
    char gp_file[64];
    int fd = 0;
    mm_segment_t prev_fs;

    struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
    struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

    prev_fs = get_fs();
	set_fs(get_ds());

    pr_err("j1708en : /sys/class/gpio/gpio%d/value", ds->j1708en_vgpio_num);
 
    sprintf(gp_file, "/sys/class/gpio/gpio%d/value", ds->j1708en_vgpio_num);
    fd = sys_open(gp_file, O_RDWR, S_IRUSR|S_IRGRP);

    if(fd)
    {
        err = sys_write(fd, buf, 1); 
        sys_close(fd);
        
        if(err)
        {
            pr_err("error! couldn't connect to mcu");
        }
    }

    set_fs(prev_fs);

    return (err);
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
                    ds->mcu_pins[J1708_GPIO_OFFSET] = ds->j1708en_vgpio_num;
                    gpio_direction_output(ds->mcu_pins[J1708_GPIO_OFFSET], 0);
                    gpio_export(ds->mcu_pins[J1708_GPIO_OFFSET], 0);
                }
            }

            if (gpio_is_valid(ds->rs485en_vgpio_num )) {           
                sprintf(gp_file, "mcu_out_%d",ds->rs485en_vgpio_num);
                pr_err( "%s %d %p", gp_file, ds->rs485en_vgpio_num,ds->pdev);
               err = devm_gpio_request(ds->pdev,ds->rs485en_vgpio_num, gp_file);
                if (err) {
                    pr_err("virtual out [%d] is busy!\n", (ds->rs485en_vgpio_num));
                } else {
                    ds->mcu_pins[RS48_GPIO_OFFSET] = ds->rs485en_vgpio_num;
                    gpio_direction_output(ds->mcu_pins[RS48_GPIO_OFFSET], 0);
                    gpio_export(ds->mcu_pins[RS48_GPIO_OFFSET], 0);
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

static int dock_switch_probe(struct platform_device *pdev)
{
	int err = -1;
	struct dock_switch_device *ds;
    struct device *dev = &pdev->dev;
    struct device_node *np;
    struct pinctrl_state *pctls;
    int	proj_num = 1;//portable
    const char *proj;
    uint32_t arr[2] = {0};
	
    np = dev->of_node;
    if (!np) {
        pr_err("failure to find device tree\n");
        return -EINVAL;
    }
	
	ds = devm_kzalloc(dev, sizeof(struct dock_switch_device), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

    proj = of_get_property(np, "compatible", NULL);
    if (proj && 0 == strncmp("mcn,fixed-dock-switch", proj, 18)) {
        proj_num = 2;
    }
    pr_notice("TAB8 %s \n", (2 == proj_num)?"fixed":"portable");

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

		if (1 == proj_num) {
			err = of_get_named_gpio_flags(np, "mcn,dock-pin", 0, (enum of_gpio_flags *)&ds->dock_active_l);
			if (!gpio_is_valid(err)) {
				pr_err("ivalid docking pin\n");
				err = -EINVAL;
				break;
			}

            ds->usb_psy = power_supply_get_by_name("usb");

            INIT_WORK(&ds->work, dock_switch_work_func);
            wake_lock_init(&ds->wlock, WAKE_LOCK_SUSPEND, "switch_dock_wait_lock");

			ds->dock_pin = err;
			ds->dock_active_l = !ds->dock_active_l;
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
            //proj TREQr_5
			err = of_property_read_u32_array(np, "mcn,virt-gpio", arr, 2);
			if (!err) {
				err                    = arr[0];//base
				ds->virt_gpio_offset   = arr[1];//offset
				err += ds->virt_gpio_offset;
				if (!gpio_is_valid(err)) {
					pr_err("ivalid ignition pin\n");
					err = -EINVAL;
					break;
				}
			} else {
				pr_err("cannot get ign gpio\n");
				err = -EINVAL;
				break;
			}

			pr_notice("ignition detect pin %d\n", err);
			ds->ign_pin = err;
			ds->virt_init = VIRT_GPIO_OFF;

			if(gpio_is_valid(ds->ign_pin)) {
				err = devm_gpio_request(dev, ds->ign_pin, "ignition-state");//pdev->name);
				if (err < 0) {
					pr_err("failure to request the gpio[%d]\n", ds->ign_pin);
				} else {
					err = gpio_direction_input(ds->ign_pin);
					if (err < 0) {
						pr_err("failure to set direction of the gpio[%d]\n", ds->ign_pin);
						break;
					}
					gpio_export(ds->ign_pin, 0);
					ds->virt_init = VIRT_GPIO_ON;
				}
			}

			ds->ign_active_l = 1;

			INIT_WORK(&ds->work, dock_switch_work_virt_func);

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
        ds->attr_J1708_en.attr.attr.mode = S_IRUGO|S_IWUGO;/*666*/
        ds->attr_J1708_en.attr.show = j1708_en_state_show;
        ds->attr_J1708_en.attr.store = j1708_en_state_store;
        sysfs_attr_init(&ds->attr_J1708_en.attr.attr);
        device_create_file((&ds->sdev)->dev, &ds->attr_J1708_en.attr);

        snprintf(ds->attr_rs485_en.name, sizeof(ds->attr_rs485_en.name) - 1, "rs485_en");
        ds->attr_rs485_en.attr.attr.name = ds->attr_rs485_en.name;
        ds->attr_rs485_en.attr.attr.mode = S_IRUGO|S_IWUGO;/*666*/
        ds->attr_rs485_en.attr.show = rs485_en_state_show;
        ds->attr_rs485_en.attr.store = rs485_en_state_store;
        sysfs_attr_init(&ds->attr_rs485_en.attr.attr);
        device_create_file((&ds->sdev)->dev, &ds->attr_rs485_en.attr);

        INIT_DELAYED_WORK(&ds->mcu_gpio_init_work, mcu_gpio_init_work);
        schedule_delayed_work(&ds->mcu_gpio_init_work, msecs_to_jiffies(100));
        ////////////////////////////////////////////////////

        INIT_DELAYED_WORK(&ds->vgpio_init_work, swithc_dock_outs_init_work);
        schedule_delayed_work(&ds->vgpio_init_work, msecs_to_jiffies(100));
             
        pr_notice("registered\n");

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
	struct irq_desc *desc;

    if (device_may_wakeup(dev)) {
        if (ds->ign_irq) {
        	desc = irq_to_desc(ds->ign_irq);
            if(desc->depth == 0) {
            	disable_irq_nosync(ds->ign_irq);
                pr_notice("disable wake source IGN[%d]\n", ds->ign_irq);
            }
            disable_irq_wake(ds->ign_irq);
        }
        if (ds->dock_irq && (e_dock_type_smart != ds->dock_type)) {
        	desc = irq_to_desc(ds->dock_irq);
            if(desc->depth == 0) {
            	disable_irq_nosync(ds->dock_irq);
                pr_notice("disable wake source DOCK[%d]\n", ds->dock_irq);
            }
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

