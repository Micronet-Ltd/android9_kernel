
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/of.h>//Michael Efimov: added
//#include <linux/workqueue.h>//Michael Efimov: added
//#include <linux/of_address.h>//Michael Efimov: added

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/dirent.h>
#include <linux/string.h>
#include <linux/hwmon-sysfs.h>
#include "../../misc/hi_3w/hi_3w.h"//Michael Efimov: added

#define VINPUTS_NAME	"vinputs"
#define	FIND_NAME 		"vgpio_in"

extern int32_t gpio_in_register_notifier(struct notifier_block *nb);
extern int cradle_register_notifier(struct notifier_block *nb);//Michael Efimov: added

struct gpio_set {
	int base;
	int ngpio;
};
struct work_params {
	int reason;
	int args;
};

struct vinput_key {
	unsigned int	code;
	int 			val;
};
//
//#define ABS_HAT0X		0x10
//#define ABS_HAT0Y		0x11
//#define ABS_HAT1X		0x12
//#define ABS_HAT1Y		0x13
//#define ABS_HAT2X		0x14
//#define ABS_HAT2Y		0x15
//#define ABS_HAT3X		0x16
//#define ABS_HAT3Y		0x17

static struct vinput_key vinputs[] = {
	{ABS_HAT0X, 0}, //KEY_F1
	{ABS_HAT0Y, 0},
	{ABS_HAT1X, 0},
	{ABS_HAT1Y, 0},
	{ABS_HAT2X, 0},
	{ABS_HAT2Y, 0},
	{ABS_HAT3X, 0},
	{ABS_HAT3Y, 0},
};
struct virt_inputs {
	struct 	miscdevice*	mdev;
	struct 	input_dev* 	input_dev;
	struct  work_struct work;
    struct  delayed_work    virtual_input_init_work;//Michael Efimov:  added
    struct  notifier_block virtual_inputs_cradle_notifier;//Michael Efimov:  added
	struct 	work_params wparams; 
	struct 	vinput_key* vmap;
	struct	gpio_set 	gpios_in;
	struct 	notifier_block   notifier;
	int		reinit;
    int     cradle_attached;//Michael Efimov:  added
    struct mutex lock;
};

static struct virt_inputs* vdev;

static int vinputs_init_files(void);

//#define DBG_INIT	1
//#ifdef	DBG_INIT
//char dbg_buf[1024] = {0};//temp!!!
//#endif
//////////////
inline static int is_gpios_exists(struct gpio_set* gpios)
{
	if (gpios->base > 0 && gpios->ngpio > 0) {
		return 1;
	}
	pr_info("%s: gpio failed base %d, ngpio %d\n", __func__, gpios->base, gpios->ngpio);
	return 0;
}

static int vinputs_get_gpios(int f_update)
{
	int i, err, val = 0, v = 0;
	int num = vdev->gpios_in.base;
	int qty = min_t(size_t, (vdev->gpios_in.ngpio), (ARRAY_SIZE(vinputs)));

	if(!is_gpios_exists(&vdev->gpios_in)) {
		return -1;
	}

	for (i = 0; i < qty; i++) {
		err = gpio_request(num, 0);
		if (!err && gpio_is_valid(num)) {

			v = gpio_get_value(num);
			gpio_free(num);

			val |= (v << i);
			if (f_update) {
				vdev->vmap[i].val = v;
			}
		}
		else {
			val |= (vdev->vmap[i].val << i);//don't change
			pr_err("%s: gpio_request failed err %d\n", __func__, err);
		}

		num++;
	}
	return val;
}

static int vinputs_get_val(void)
{
	int i, val = 0;
	int qty = min_t(size_t, (vdev->gpios_in.ngpio), (ARRAY_SIZE(vinputs)));

	for (i = 0; i < qty; i++) {
		val |= (vdev->vmap[i].val << i);
	}
	return val;
}
static ssize_t show_in_all(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = vinputs_get_val();

	return sprintf(buf, "0x%02X\n", val);
}

static ssize_t show_in(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct virt_inputs *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int ix = sensor_attr->index;

	if( (ix > ARRAY_SIZE(vinputs) - 1) || ix < 0 ){
		return sprintf(buf, "in%d - bad parameter\n", ix);
	}
	pr_info("%d\n", vdev->vmap[ix].code);
	return sprintf(buf, "%d\n", vdev->vmap[ix].val);
}
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, show_in, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, show_in, NULL, 4);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, show_in, NULL, 5);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, show_in, NULL, 6);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, show_in, NULL, 7);
static DEVICE_ATTR(in_all, S_IRUGO, show_in_all, NULL);

static struct attribute *in_attributes[] = {
	// INPUTS
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	&dev_attr_in_all.attr,
	NULL
};

static const struct attribute_group in_attr_group = {
	.attrs = in_attributes
};

static void vinputs_work_func(struct work_struct *work) 
{
	struct virt_inputs *vinp  = container_of(work, struct virt_inputs, work);
	unsigned long val = -1;
	int i, v, sync = 0;
	int qty = min_t(size_t, (vinp->gpios_in.ngpio), (ARRAY_SIZE(vinputs)));

	if (vinp->reinit) {
		pr_notice("%s: init\n", __func__);

        mutex_lock(&vinp->lock);
		val = vinputs_init_files();
        if (0 == val) {
            vinp->reinit = 0; 
            pr_notice("%s: succeed\n", __func__);
        }
        mutex_unlock(&vinp->lock);
	}
    if (vinp->reinit) {
        pr_notice("%s: failure\n", __func__);
        schedule_work(&vinp->work);
        return;
    }

	val = vinputs_get_gpios(0); 

	pr_notice("%s: gpio 0x%X\n", __func__, (unsigned int)val);

	for (i = 0; i < qty; i++) {
		v = (test_bit(i, &val)) ? (KEY_F1 + i) : 0;

		if(vinp->vmap[i].val != v) {
			input_report_abs(vinp->input_dev, vinp->vmap[i].code, v);
			vinp->vmap[i].val = v;
			sync = 1;
			pr_notice("%s sent input%i = %d\n", __func__, i, (unsigned int)v);
		}
	}

	if (sync) {
		input_sync(vinp->input_dev);
	}
}

//***       Michael Efimov: added       ***//
static void cradle_is_connected_work_fix(struct work_struct *work){
    uint32_t cmd = 0/*, inputs_status = 0*/;
    uint32_t inp_val[2] = {0};
    int temp_val = 0, sync = 0, /*number_chnl = 0,*/ cnt, strt_cnt, stp_cnt;

    struct virt_inputs *vinp = container_of(work, struct virt_inputs, virtual_input_init_work.work);
    if (vinp->cradle_attached) {
        strt_cnt = 0;
        stp_cnt = 2;
        /*hi_3w_tx_cmd(&cmd, 1);
        if (cmd & 0x18) {
            inputs_status = 1;
            if ((cmd & 0x18) == 0x08) {
                stp_cnt = 1;
                number_chnl = 2;
                pr_notice("first chnl %x\n", (cmd & 0x18)); //Michael Efimov: added for test
            }
            if ((cmd & 0x18) == 0x10) {
                strt_cnt = 1;
                number_chnl = 1;
                pr_notice("second chnl %x\n", (cmd & 0x18));//Michael Efimov: added for test
            }
            if ((cmd & 0x18) == 0x18) {
                pr_notice("both chnls %x\n", (cmd & 0x18));//Michael Efimov: added for test
            }
            if (number_chnl) {
                temp_val = 0;
                if (vinp->vmap[number_chnl - 1].val != temp_val) {
                    input_report_abs(vinp->input_dev, vinp->vmap[number_chnl - 1].code, temp_val);
                    vinp->vmap[number_chnl - 1].val = temp_val;
                    pr_notice("value key%d-%d\n",cnt+1 , inp_val[number_chnl - 1]);//Michael Efimov: added for test
                    sync = 1;
                }
            }
        }*/
        for (cnt = strt_cnt; cnt < stp_cnt; cnt++) {
            //if (inputs_status) {
                cmd = cnt << 2;
                cmd|=0x28;
                cmd<<=24;
                pr_notice("request command%d %x\n", (cnt+1), cmd);//Michael Efimov: added for test
                hi_3w_tx_cmd(&cmd, 1);
                pr_notice("responce command%d %x\n", (cnt+1), cmd);//Michael Efimov: added for test
                inp_val[cnt] = cmd;
                inp_val[cnt] &=~(0xFF000000);
                pr_notice("value%d-%d\n",cnt+1 , inp_val[cnt]);//Michael Efimov: added for test
                temp_val = (inp_val[cnt]>6000) ? (KEY_F1 + cnt) : 0;
            //}
            if (vinp->vmap[cnt].val!=temp_val) {
                input_report_abs(vinp->input_dev, vinp->vmap[cnt].code, temp_val);
                vinp->vmap[cnt].val = temp_val;
                pr_notice("value key%d-%d\n",cnt+1 , inp_val[cnt]);//Michael Efimov: added for test
                sync = 1;
            }
        }
        if (sync) {
            input_sync(vinp->input_dev);
            pr_notice("was sinchroniced\n");//Michael Efimov: added for test
        }
        pr_notice("responce command1 %d\n", inp_val[0]);//Michael Efimov: added for test
        pr_notice("responce command2 %d\n", inp_val[1]);//Michael Efimov: added for test
        schedule_delayed_work(&vdev->virtual_input_init_work, msecs_to_jiffies(1000));
    }
}
//****************************************//

static int32_t __ref vinputs_callback(struct notifier_block *nfb, unsigned long reason, void *arg)
{
    struct virt_inputs *vinp = container_of(nfb, struct virt_inputs, notifier);

    pr_notice("%s: [%lu]\n", __func__, reason);

	if (0 == reason) {
        mutex_lock(&vinp->lock);
		// vinp->reinit = 1;
	}
	schedule_work(&vinp->work);

    if (0 == reason) {
        mutex_unlock(&vinp->lock);
    }

    return NOTIFY_OK;
}

//***                                           Michael Efimov: added                                           ***//
static int __ref virtual_inputs_cradle_callback(struct notifier_block *nfb, unsigned long reason, void *p)
{
    struct virt_inputs *vinputs = container_of(nfb, struct virt_inputs, virtual_inputs_cradle_notifier);

    vinputs->cradle_attached = reason;
    if (vinputs->cradle_attached) {
        cancel_delayed_work(&vdev->virtual_input_init_work);
        schedule_delayed_work(&vdev->virtual_input_init_work, 0);
    }else{
        cancel_delayed_work(&vdev->virtual_input_init_work);
    }
	return NOTIFY_OK;
}
//****************************************************************************************************************//

static int vinputs_open(struct inode *inode, struct file *file)
{
	file->private_data = vdev;
	return 0;
}
static int vinputs_release(struct inode *inode, struct file *file)
{
//	struct virt_inputs* dev = file->private_data;

	return 0;
}
///test!!!!!!
/*
#define BUF_SIZE 1024
static int get_val_from_file(const char* fname)
{
	int fd, val = -1;
	char txt[4] = {0};

	fd = sys_open(fname, O_RDONLY, 0);
	if (fd > 0) {
		sys_read(fd, txt, 4);
		sys_close(fd);

		val = simple_strtoul(txt, 0, 10);
	}
	return val;
}
static int find_gname(void* buf, const char* in_dir, const char* cmp_str, struct gpio_set *set)
{
	int fd, fdf;
	struct linux_dirent64 *dirp;
	mm_segment_t old_fs;
	int num;
	char txt[128] = {0};
	const char* fname = "label";

	if (!in_dir || !fname || !cmp_str || (strlen(cmp_str) > sizeof(txt)) ) {
		pr_err("%s:bad params\n", __func__);
		//sprintf(dbg_buf, "%s:bad params\n", __func__);
		return -1;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pr_notice("%s:+\n", __func__);

	fd = sys_open(in_dir, O_RDONLY, 0); 

	if (fd < 0) {
		pr_err("%s:error open ret %d\n", __func__, fd);
		//sprintf(dbg_buf, "%s:error open ret %d\n", __func__, fd);
		set_fs(old_fs);
		return -1;
	}
	dirp = (struct linux_dirent64 *)buf;
	num = sys_getdents64(fd, dirp, BUF_SIZE);
	while (num > 0) {
		while (num > 0) {
			pr_info("%s: %s\n", __func__, dirp->d_name);

			if(0 == strcmp(dirp->d_name, fname)) {
				sprintf(txt,"%s/%s", in_dir, fname);
				fdf = sys_open(txt, O_RDONLY, 0);
				if (fdf < 0) {
					pr_err("%s:error open file %s [%d]\n", __func__, txt, fdf);
					//sprintf(dbg_buf, "%s:error open file %s [%d]\n", __func__, txt, fdf);
					//try next;
				}
				else {
					memset(txt, 0, sizeof(txt));
					sys_read(fdf, txt, strlen(cmp_str));
					sys_close(fdf);

					if (0 == strcmp(txt, cmp_str)) {

						sprintf(txt,"%s/%s", in_dir, "base");
						set->base = get_val_from_file(txt);

						sprintf(txt,"%s/%s", in_dir, "ngpio");
						set->ngpio = get_val_from_file(txt);
						num = 0;
						break;
					}
				}
			}

			num -= dirp->d_reclen;
			dirp = (void *)dirp + dirp->d_reclen;
		}

		dirp = buf;
		memset(buf, 0, BUF_SIZE);
		num = sys_getdents64(fd, dirp, BUF_SIZE);
	}
	pr_notice("%s: -\n", __func__);

	sys_close(fd);
	set_fs(old_fs);

	return set->ngpio;
}
int find_gpios(const char* find_name, struct gpio_set* gset)
{
	int fd;
	void *buf;
	struct linux_dirent64 *dirp;
	mm_segment_t old_fs;
	int num;
	const char* start_dir = "/sys/class/gpio/";
	const char* chip_str = "gpiochip";

	char dir_chip[64] = {0};

	memset(gset, 0, sizeof(struct gpio_set));

	buf = kzalloc(BUF_SIZE * 2, GFP_KERNEL);
	if (!buf) {
		pr_err("%s:kzalloc faile\n", __func__);
		//sprintf(dbg_buf, "%s:kzalloc faile\n", __func__);
		return -ENOMEM;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pr_notice("%s:+\n", __func__);

	fd = sys_open(start_dir, O_RDONLY, 0);

	if (fd < 0) {
		pr_err("%s:error open ret %d\n", __func__, fd);
		//sprintf(dbg_buf, "%s:error open ret %d ", __func__, fd);

		fd = sys_open("/sys/", O_RDONLY, 0);
		if (fd < 0) {
			pr_err("%s:error open sys ret %d\n", __func__, fd);
			sprintf(dir_chip, " - sys ret %d", fd);
			//strcat(dbg_buf, dir_chip);
		}
		else
			sys_close(fd);
		fd = sys_open("/proc/", O_RDONLY, 0);
		if (fd < 0) {
			pr_err("%s:error open /proc ret %d\n", __func__, fd);
			sprintf(dir_chip, " - /proc ret %d", fd);
			//strcat(dbg_buf, dir_chip);
		}
		else
			sys_close(fd);

		fd = sys_open("/sys/devices/virtual/gpio/", O_RDONLY, 0);
		if (fd < 0) {
			pr_err("%s:error open sys/devices/virtual/gpio/ ret %d\n", __func__, fd);
			sprintf(dir_chip, " - sys/devices/virtual/gpio/ ret %d\n", fd);
			//strcat(dbg_buf, dir_chip);
		}
		else
			sys_close(fd);
		return -1;
	}

	dirp = (struct linux_dirent64*)buf;
	num = sys_getdents64(fd, dirp, BUF_SIZE);
	while (num > 0) {
		while (num > 0) {
			pr_notice("%s: %s\n", __func__, dirp->d_name);

			if(0 == strncmp(dirp->d_name, chip_str, strlen(chip_str)) ) {

				sprintf(dir_chip, "%s%s", start_dir, dirp->d_name);
				if(find_gname(buf + BUF_SIZE, dir_chip, find_name, gset) > 0)
				{
					pr_notice("%s: found base %d, num %d\n", __func__, gset->base, gset->ngpio );

					num = 0;//for the next break
					break;
				}
			}

			num -= dirp->d_reclen;
			dirp = (void *)dirp + dirp->d_reclen;
		}

		dirp = buf;
		memset(buf, 0, BUF_SIZE);
		num = sys_getdents64(fd, dirp, BUF_SIZE);
	}
	pr_notice("%s: -\n", __func__);

	sys_close(fd);
	kfree(buf);

	return gset->ngpio;
}
*/
static int gpiochip_lable_match(struct gpio_chip* chip, void* data)
{
	return !strcmp(chip->label, data);
}
static int find_gpios_by_label(char* find_name, struct gpio_set* gset)
{
	struct gpio_chip *chip;
	chip = gpiochip_find(find_name, gpiochip_lable_match);

	if (!chip) {
		pr_err("%s: gpiochip not found, label %s\n", __func__, find_name);
		return 0;
	}

	gset->base = chip->base;
	gset->ngpio= chip->ngpio;

	pr_notice("%s: found base %d, num %d\n", __func__, gset->base, gset->ngpio );
	return gset->ngpio;
}

static int vinputs_init_files(void)
{
	if(find_gpios_by_label(FIND_NAME, &vdev->gpios_in) > 0) {
        if(is_gpios_exists(&vdev->gpios_in)) {
            return 0;
        }
	}
	return -1;
}

static const struct file_operations vinputs_dev_fops = {
	.owner      = THIS_MODULE,
	.open       = vinputs_open,
	.release    = vinputs_release,
	.llseek		= no_llseek,
};

static struct miscdevice vinputs_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= VINPUTS_NAME,
	.fops		= &vinputs_dev_fops,
};

static int vinputs_create_input_dev(struct virt_inputs* vdev)
{
	int error, i;

	vdev->input_dev = input_allocate_device();
	if (!vdev->input_dev) {
		pr_err("input_allocate_device failed\n");
		return -ENOMEM;
	}

	kfree(vdev->input_dev->name);
	vdev->input_dev->name = kstrndup(VINPUTS_NAME, sizeof(VINPUTS_NAME), GFP_KERNEL);

	input_set_capability(vdev->input_dev, EV_ABS, vdev->vmap[0].code);

	for (i = 0; i < ARRAY_SIZE(vinputs); i++) {
		input_set_abs_params(vdev->input_dev, vdev->vmap[i].code, 0, KEY_F1 + i, 0, 0); 
	}

	error = input_register_device(vdev->input_dev);
	if (error) {
		pr_err("input_register_device failed\n");
		input_free_device(vdev->input_dev);
		return error;
	 }

	return 0;
}

static int __init virtual_inputs_init(void)
{
	int error;
    int fixed_mode = 0;//Michael Efimov:  added
	struct device *dev;
    struct device_node *np;//Michael Efimov:  added

    //***          Michael Efimov:       added         ***//
    np = of_find_compatible_node(NULL, NULL, "mcn,fixed-vinputs");
    if (np) {
        fixed_mode = 1;
        pr_err("node is finded\n");
    }
    /*if (!np) {
        pr_err("driver is not finded\n");//Michael Efimov: added for test
    } else {
        fixed_mode = 1;
        pr_err("driver is finded\n");//Michael Efimov: added for test
    }*/
    //**************************************************//
	pr_info("%s:+\n", __func__);
	vdev = kzalloc(sizeof(struct virt_inputs), GFP_KERNEL);
	if (!vdev)
		return -ENOMEM;

	vdev->vmap = vinputs;
	vdev->reinit = 0;

	error = misc_register(&vinputs_dev);
	if (error) {
		pr_err("%s: misc_register failed\n", vinputs_dev.name);
		return -EINVAL;
	}

    //***          Michael Efimov:  added         ***//
    if (fixed_mode) {
        vdev->virtual_inputs_cradle_notifier.notifier_call = virtual_inputs_cradle_callback;//put pointer on the callback func to the notifier block
        cradle_register_notifier(&vdev->virtual_inputs_cradle_notifier);//register callback func in cradle notifier
        INIT_DELAYED_WORK(&vdev->virtual_input_init_work, cradle_is_connected_work_fix);//initial working func in bill of delayed work
        vdev->cradle_attached = 0;//initial var in structure for first check in the wirking func
        schedule_delayed_work(&vdev->virtual_input_init_work, 0);//run working func without delayer in first time

        //pr_info("%s:-\n", __func__);

        //return 0;
    }
    //*************************************************//

	vdev->mdev = &vinputs_dev;
    if(!fixed_mode){//Michael Efimov:  added - if(!fixed_mode){
        vdev->notifier.notifier_call = vinputs_callback;
        error = gpio_in_register_notifier(&vdev->notifier);
        if (error) {
            pr_err("failure to register remount notifier [%d]\n", error);
            misc_deregister(&vinputs_dev);
            kfree(vdev);
            return error;
        }
    }//Michael Efimov:  added - }
	error = vinputs_create_input_dev(vdev);
	if (error) {
		misc_deregister(&vinputs_dev);
		kfree(vdev);
		return error;
	}
    //if(!fixed_mode){//Michael Efimov:  added - if(!fixed_mode){
        dev = vdev->mdev->this_device;
        error = sysfs_create_group(&dev->kobj, &in_attr_group);
        if (error) {
            pr_err("%s: could not create sysfs group\n", __func__);
            input_free_device(vdev->input_dev);
            misc_deregister(&vinputs_dev);
            kfree(vdev);
            return error;
        }
    //}//Michael Efimov:  added - }
    mutex_init(&vdev->lock);
    vdev->reinit = 1;
    if(!fixed_mode){//Michael Efimov:  added - if(!fixed_mode){
        INIT_WORK(&vdev->work, vinputs_work_func);
        //	vinputs_init_files();
        schedule_work(&vdev->work); 
    }//Michael Efimov:  added - }
    pr_info("%s:-\n", __func__);
	return 0;
}

static void __exit virtual_inputs_exit(void)
{
	struct device *dev= vinputs_dev.this_device;

    cancel_work_sync(&vdev->work);
	sysfs_remove_group(&dev->kobj, &in_attr_group);
	input_free_device(vdev->input_dev);
	misc_deregister(&vinputs_dev);
	kfree(vdev);
}

late_initcall(virtual_inputs_init);
module_exit(virtual_inputs_exit);

MODULE_LICENSE("GPL");

