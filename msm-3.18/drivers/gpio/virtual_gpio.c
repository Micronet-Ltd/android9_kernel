#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/hwmon-sysfs.h>


MODULE_LICENSE("Dual BSD/GPL");

/////

//#define VGPIO_USE_SPINLOCK

struct vgpio_bank {
	unsigned long gpio_mask;
	unsigned long gpio_value;
	wait_queue_head_t wq;
#ifdef VGPIO_USE_SPINLOCK
	spinlock_t lock;
#else
	struct mutex lock;
#endif
};
unsigned long g_gpio;


#ifdef VGPIO_USE_SPINLOCK
#define DEFINE_LOCK_FLAGS(x) int x
#define LOCK_BANK(lock, flags) do { spin_lock_irqsave(&lock, flags); } while (0)
#define UNLOCK_BANK(lock, flags) do { spin_unlock_irqrestore(&lock, flags); } while (0)
#else
#define DEFINE_LOCK_FLAGS(x) // NOOP
#define LOCK_BANK(lock, flags) do { mutex_lock(&lock); } while (0)
#define UNLOCK_BANK(lock, flags) do { mutex_unlock(&lock); } while (0)
#endif

struct virt_gpio {
	struct gpio_chip gpiochip_out;
	struct gpio_chip gpiochip_in;

	unsigned long inuse;

	// atomic bitset
	unsigned long gpi_values;
	struct vgpio_bank gpo_bank;

	unsigned long enabled_out;
	unsigned long enabled_in;
};

struct virt_gpio * g_pvpgio;

/////
static RAW_NOTIFIER_HEAD(gpio_in_chain);
static DEFINE_RAW_SPINLOCK(gpio_in_chain_lock);

static void gpio_in_notify(unsigned long reason, void *arg)
{
    unsigned long flags;

    raw_spin_lock_irqsave(&gpio_in_chain_lock, flags);
    raw_notifier_call_chain(&gpio_in_chain, reason, arg);
    raw_spin_unlock_irqrestore(&gpio_in_chain_lock, flags);
}

int32_t gpio_in_register_notifier(struct notifier_block *nb)
{
    unsigned long flags;
    int32_t err;

    raw_spin_lock_irqsave(&gpio_in_chain_lock, flags);
    err = raw_notifier_chain_register(&gpio_in_chain, nb);
    raw_spin_unlock_irqrestore(&gpio_in_chain_lock, flags);
    pr_notice("%d\n", err);

    return err;
}
EXPORT_SYMBOL(gpio_in_register_notifier);
///////debug access
static ssize_t show_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, val = 0;

	for(i = 0; i < g_pvpgio->gpiochip_in.ngpio; i++) {
		val |= (test_bit(i, &g_pvpgio->gpi_values) << i);
	}
	return sprintf(buf, "%02X\n", val);
}
static ssize_t set_in(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char str[8] = {0};
	int i;
	unsigned long mask = 0, val = 0, changed_bits = 0;

	pr_info("%s, count %d\n", __func__, (int)count);
	if (5 != count) {
		pr_err("error: format XXXX (XX - mask, XX - value)\n");
		return count;
	}
	strncpy(str, buf, 2);
	mask = simple_strtol(str, NULL, 16);
	strncpy(str, buf + 2, 2);
	val = simple_strtol(str, NULL, 16);

	pr_info("%s, mask 0x%X, val 0x%X\n", __func__, (unsigned int)mask, (unsigned int)val);

	for(i = 0; i < g_pvpgio->gpiochip_in.ngpio; i++) {
		if(test_bit(i, &mask)) {
			if(test_bit(i, &val) != test_bit(i, &g_pvpgio->gpi_values)) {

				if(test_bit(i, &val)) {
					pr_info("%s set   INPUT%d\n", __func__, i);
					set_bit(i, &g_pvpgio->gpi_values);
				} else {
					pr_info("%s: clear INPUT%d\n", __func__, i);
					clear_bit(i, &g_pvpgio->gpi_values);
				}
				changed_bits |= (i<<i);
			}
		}
	}
	if(changed_bits)
	{
		g_gpio = changed_bits;
		pr_notice("gpio_in_notify 0x%X\n", (unsigned int)g_gpio);
		gpio_in_notify(1, &g_gpio);
	}


	return count;
}
static DEVICE_ATTR(dbg_inputs, S_IRUGO | S_IWUSR, show_in, set_in);

static struct attribute *in_attributes[] = {
	&dev_attr_dbg_inputs.attr,
	NULL
};

static const struct attribute_group in_attr_group = {
	.attrs = in_attributes
};

/////
static int vgpio_dev_open(struct inode *inode, struct file *file)
{
	struct virt_gpio * dev;
	DEFINE_LOCK_FLAGS(flags);

	dev = g_pvpgio;

	if(test_and_set_bit(1, &dev->inuse))
		return -EPERM;

	//dev = container_of(inode->i_cdev, struct virt_gpio, cdev);
	file->private_data = dev;

	LOCK_BANK(dev->gpo_bank.lock, flags);

	// Set all the bits in the mask so all values are updated first time
	dev->gpo_bank.gpio_mask = 0xff;

	UNLOCK_BANK(dev->gpo_bank.lock, flags);

	// Does this need to be done, or is it redundant
	wake_up_interruptible(&dev->gpo_bank.wq);

	return 0;
}

static int vgpio_dev_release(struct inode *inode, struct file *file)
{
	struct virt_gpio * dev = file->private_data;

	clear_bit(1, &dev->inuse);

	return 0;
}

// TODO: Test
static unsigned int vgpio_dev_poll(struct file * file,  poll_table * wait)
{
	struct virt_gpio * dev = file->private_data;
	unsigned int mask = 0;
	DEFINE_LOCK_FLAGS(flags);

	LOCK_BANK(dev->gpo_bank.lock, flags);
	poll_wait(file, &dev->gpo_bank.wq, wait);
	if(dev->gpo_bank.gpio_mask)
		mask |= POLLIN | POLLRDNORM;
	UNLOCK_BANK(dev->gpo_bank.lock, flags);

	return mask;
}

static ssize_t virt_gpio_chr_read(struct file * file, char __user * buf,
		size_t count, loff_t *ppos)
{
	struct virt_gpio * dev = file->private_data;
	uint8_t output[4];
	unsigned long out_mask;
	unsigned long out_values;
	DEFINE_LOCK_FLAGS(flags);

	if(count < sizeof(output)) {
		pr_err("%s() count too small\n", __func__);
		return -EINVAL;
	}

	LOCK_BANK(dev->gpo_bank.lock, flags);
	while(!dev->gpo_bank.gpio_mask) {
		UNLOCK_BANK(dev->gpo_bank.lock, flags);
		if((file->f_flags & O_NONBLOCK))
			return -EAGAIN;
		wait_event_interruptible(dev->gpo_bank.wq, dev->gpo_bank.gpio_mask);
		LOCK_BANK(dev->gpo_bank.lock, flags);
	}

	out_mask = dev->gpo_bank.gpio_mask;
	out_values = dev->gpo_bank.gpio_value;
	dev->gpo_bank.gpio_mask = 0;

	UNLOCK_BANK(dev->gpo_bank.lock, flags);

	if(out_mask) {
		int w = 4;
		output[0] = 0;
		output[1] = 0;
		output[2] = out_mask & 0xff;
		output[3] = out_values & 0xff;

		if(copy_to_user(buf, output, w))
			return -EINVAL;
		*ppos = 0;

		pr_debug("%s() return %d\n", __func__, w);
		return w;
	} else {
		pr_debug("%s() return 0 no io change\n", __func__);
	}

	return 0;
}

static ssize_t virt_gpio_chr_write(struct file * file, const char __user * buf,
		size_t count, loff_t * ppos)
{
	struct virt_gpio * dev = file->private_data;
	uint8_t msg[4];
	int i;
	uint8_t val = 0;

	if(count != 4)
		return -EINVAL;

	if(copy_from_user(msg, buf, count))
		return -EACCES;

	if(msg[0] || msg[1]) {
		pr_debug("%s() unsupported command %x,%x\n", __func__, msg[0], msg[1]);
		return -EINVAL;
	}

//	pr_err(" %02d %02d\n", msg[2], msg[3]);

	// TODO: use macro
	for(i = 0; i < dev->gpiochip_in.ngpio; i++) {
		if(msg[2] & (1<<i)) {
			if( msg[3] & (1<<i)) {
//				printk("%s set   INPUT%d\n", __func__, i);
				set_bit(i, &dev->gpi_values);
			} else {
//				printk("%s: clear INPUT%d\n", __func__, i);
				clear_bit(i, &dev->gpi_values);
			}
          	val |= (1<<i);
		}
	}
   	if(val)
	{
	    pr_err("gpio_in_notify %d\n", val);
	    g_gpio = val;
	    gpio_in_notify(1, &g_gpio);
	}
	return count;
}

/////

static int virt_gpio_out_request(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	set_bit(offset, &dev->enabled_out);

	return 0;
}

static int virt_gpio_in_request(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;
//	printk("%s: %d\n", __func__, offset);
	set_bit(offset, &dev->enabled_in);

	return 0;
}

static void virt_gpio_out_free(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	clear_bit(offset, &dev->enabled_out);
}

static void virt_gpio_in_free(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	clear_bit(offset, &dev->enabled_in);
}

static int virt_gpio_out_get(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static int virt_gpio_in_get(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;

//    printk("%s: %lx\n", __func__, dev->gpi_values);
	return test_bit(offset, &dev->gpi_values) ? 1 : 0;
}

static void virt_gpio_out_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct virt_gpio * dev = g_pvpgio;
	DEFINE_LOCK_FLAGS(flags); // make last

	pr_debug("%s() offset %d value %d\n", __func__, offset, value);

	LOCK_BANK(dev->gpo_bank.lock, flags);

	__set_bit(offset, &dev->gpo_bank.gpio_mask);
	if(value)
		__set_bit(offset, &dev->gpo_bank.gpio_value);
	else
		__clear_bit(offset, &dev->gpo_bank.gpio_value);

	UNLOCK_BANK(dev->gpo_bank.lock, flags);

	wake_up_interruptible(&dev->gpo_bank.wq);
}

static int virt_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	// set direction input
	pr_debug("%s() offset %d output\n", __func__, offset);
	return 0;
}

static int virt_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	pr_debug("%s() set output and offset %d to %d\n", __func__, offset, value);
	// set direction output and set value
	virt_gpio_out_set(chip, offset, value);
	// Set as output
	return 0;
}


static const struct file_operations vgpio_dev_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.read = virt_gpio_chr_read,
	.write = virt_gpio_chr_write,
	.open = vgpio_dev_open,
	.release = vgpio_dev_release,
	.poll   = vgpio_dev_poll,
};



static struct miscdevice vgpio_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "vgpio",
	.fops		= &vgpio_dev_fops,
};

static int __init virtual_gpio_init(void)
{
	struct virt_gpio * dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if(!dev)
		return -ENOMEM;

	g_pvpgio = dev;

	init_waitqueue_head(&dev->gpo_bank.wq);

#ifdef VGPIO_USE_SPINLOCK
	spin_lock_init(&dev->gpo_bank.lock);
#else
	mutex_init(&dev->gpo_bank.lock);
#endif

	ret = misc_register(&vgpio_dev);
	if(ret) {
		pr_err("%s() Unable to register misc device \n", __func__);
		return ret;
	}

	dev->gpiochip_out.label = "vgpio_out"; //virtgpio
	dev->gpiochip_out.request = virt_gpio_out_request;
	dev->gpiochip_out.free = virt_gpio_out_free;
	dev->gpiochip_out.direction_output = virt_gpio_direction_output;
	dev->gpiochip_out.set = virt_gpio_out_set;
	dev->gpiochip_out.get = virt_gpio_out_get;
	dev->gpiochip_out.base = -1;
	dev->gpiochip_out.ngpio = 8;
#ifdef VGPIO_USE_SPINLOCK
	dev->gpiochip_out.can_sleep = 0;
#else
	dev->gpiochip_out.can_sleep = 1;
#endif

	dev->gpiochip_in.label = "vgpio_in";
	dev->gpiochip_in.request = virt_gpio_in_request;
	dev->gpiochip_in.free = virt_gpio_in_free;
	dev->gpiochip_in.direction_input = virt_gpio_direction_input;
	dev->gpiochip_in.get = virt_gpio_in_get;
	dev->gpiochip_in.base = -1;
	dev->gpiochip_in.ngpio = 8;
	dev->gpiochip_in.can_sleep = 0;

	gpiochip_add(&dev->gpiochip_out);
	pr_debug("out base %d\n", dev->gpiochip_out.base);

	gpiochip_add(&dev->gpiochip_in);
	pr_debug("in base %d\n", dev->gpiochip_in.base);

    gpio_in_notify(0, 0);
//
	ret = sysfs_create_group(&vgpio_dev.this_device->kobj, &in_attr_group);
	if (ret) {
		pr_err("%s: could not create sysfs group [%d]\n", __func__, ret);
	}
//
	return 0;
}

static void __exit virtual_gpio_exit(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
	int ret;
#endif
	struct virt_gpio * dev;
	dev = g_pvpgio;

	if(dev->enabled_out) {
		int i;
		pr_info("some vgpio_in gpios are still enabled.\n");
		for(i = 0; i < dev->gpiochip_out.ngpio; i++) {
			if( (dev->enabled_out & (1<<i)) ) {
				pr_info("free vgpio_in %d.\n", i);
				gpio_free(dev->gpiochip_out.base + i);
			}
		}
	}

	if(dev->enabled_in) {
		int i;
		pr_info("some vgpio_out gpios are still enabled.\n");
		for(i = 0; i < dev->gpiochip_in.ngpio; i++) {
			if( (dev->enabled_in & (1<<i)) ) {
				pr_info("free vgpio_out %d.\n", i);
				gpio_free(dev->gpiochip_in.base + i);
			}
		}
	}

	// gpiochip_remove returns void in 3.18
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
	ret =
#endif
	gpiochip_remove(&dev->gpiochip_in);

	// gpiochip_remove returns void in 3.18
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
	ret =
#endif
	gpiochip_remove(&dev->gpiochip_out);

	misc_deregister(&vgpio_dev);

	kfree(dev);
}


module_init(virtual_gpio_init);
module_exit(virtual_gpio_exit);
