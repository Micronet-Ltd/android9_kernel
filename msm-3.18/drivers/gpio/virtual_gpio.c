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
//#define MCU_GPIO_USE_SPINLOCK 

////////////////////////////////////////////////////////////////////////
#define NUMBER_OF_MCU_GPIOS 160
#define NUMBER_OF_MCU_GPROUPS 160/sizeof(__u32) /*Barak - each group posses 32 independent GPIO
												 as for now (3/2019) In the current MCU the groups
												 are called A B C D and E*/
#define TURN_A9_GPIO_NUM_TO_MCU(x) (((x>>5)<<8)|(x&0x1f))		 
#define IS_CONNECTED(reason) (reason&1u)//look in switch_dock.c for enum NOTIFICATION_REASONS 
									    //to understand the logic here 
#define TO_FOR_MCU_RESP_MS 12000 //the timeout that each user space thread is waiting


#define	MCU_FREE_BIT (1 << 0)   //this bit in the MCU mask signals that there are no pending request to the MCU
#define	WAIT_FOR_PROCESS_BIT (1 << 2)//this bit in the MCU mask signals that there is a request 
						       //that should be processed and sent to the MCU
#define	RETURNED_FROM_MCU_BIT (1 << 3)//this bit in the MCU mask signals that a reply from the MCU has arrived
									  //and should be returned to the user
//should be identical to the same enum in control.h
typedef enum
{
	SYNC_INFO		= 0,
	COMM_WRITE_REQ 	= 1,
	COMM_READ_REQ	= 2,
	COMM_READ_RESP 	= 3,
	PING_REQ		= 4,
	PING_RESP		= 5,
	GPIO_INT_STATUS	= 6,
	POWER_MGM_STATUS = 7,
	ONE_WIRE_DATA =   8,
} packet_type_enum;

typedef enum {
	SET = COMM_WRITE_REQ,
	GET = COMM_READ_REQ,
}mcu_gpio_op_t;

typedef struct {
	int gpio_num;
	int gpio_value;
	mcu_gpio_op_t op;
}op_info_t;

struct mcu_bank {
	wait_queue_head_t mcu_wq;//will be used to sleep and wait for response from the MCU 
								  //in case a value needs to be retrieved
								//Note that this queue possesses a lock so mcu_bank
								//doesn't need one
	wait_queue_head_t respond_wq;//will be used by a single!! user space thread at a time 
							 //to sleep while waiting for the MCU to retrieve a response value to the user
 	unsigned long mcu_gpio_mask;
	 
	op_info_t gpio_info;
};
/*
//defined in switch dock and is used to inform
extern int cradle_register_notifier(struct notifier_block *nb);

struct notifier_block gpio_notifier ;

 unsigned long is_device_connected_to_mcu = 0;

int32_t __ref connected_to_mcu_gpio_callback(struct notifier_block *nfb, unsigned long reason, void *arg)
{
	//this action should be atomic for cases in which the device is for some reason
	// connected and disconnected very rapidly (for example if the physical connection is not stable)
	// We don't want the concurrent callbacks that access this variable to mix
	IS_CONNECTED(reason)? set_bit(1, &is_device_connected_to_mcu):
						  clear_bit(1, &is_device_connected_to_mcu);
    return NOTIFY_OK;
}*/



/////////////////////////////////////////////////////////////////
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
	struct gpio_chip gpiochip_mcu;

	unsigned long inuse;

	// atomic bitset
	unsigned long gpi_values;
	struct vgpio_bank gpo_bank;
	struct mcu_bank mcu_gpio_bank;

	unsigned long enabled_out;
	unsigned long enabled_in;
};

struct virt_gpio *g_pvpgio;

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

	for (i = 0; i < g_pvpgio->gpiochip_in.ngpio; i++)
	{
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
	if (5 != count)
	{
		pr_err("error: format XXXX (XX - mask, XX - value)\n");
		return count;
	}
	strncpy(str, buf, 2);
	mask = simple_strtol(str, NULL, 16);
	strncpy(str, buf + 2, 2);
	val = simple_strtol(str, NULL, 16);

	pr_info("%s, mask 0x%X, val 0x%X\n", __func__, (unsigned int)mask, (unsigned int)val);

	for (i = 0; i < g_pvpgio->gpiochip_in.ngpio; i++)
	{
		if (test_bit(i, &mask))
		{
			if (test_bit(i, &val) != test_bit(i, &g_pvpgio->gpi_values))
			{

				if (test_bit(i, &val))
				{
					pr_info("%s set   INPUT%d\n", __func__, i);
					set_bit(i, &g_pvpgio->gpi_values);
				}
				else
				{
					pr_info("%s: clear INPUT%d\n", __func__, i);
					clear_bit(i, &g_pvpgio->gpi_values);
				}
				changed_bits |= (i << i);
			}
		}
	}
	if (changed_bits)
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
	NULL};

static const struct attribute_group in_attr_group = {
	.attrs = in_attributes};

/////
static int vgpio_dev_open(struct inode *inode, struct file *file)
{
	struct virt_gpio *dev;
	DEFINE_LOCK_FLAGS(flags);

	dev = g_pvpgio;

	if (test_and_set_bit(1, &dev->inuse))
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
	struct virt_gpio *dev = file->private_data;

	clear_bit(1, &dev->inuse);

	return 0;
}

// TODO: Test
static unsigned int vgpio_dev_poll(struct file *file, poll_table *wait)
{
	struct virt_gpio *dev = file->private_data;
	unsigned int mask = 0;
	DEFINE_LOCK_FLAGS(flags);

	LOCK_BANK(dev->gpo_bank.lock, flags);

	poll_wait(file, &dev->gpo_bank.wq, wait);
	if (dev->gpo_bank.gpio_mask || (WAIT_FOR_PROCESS_BIT & dev->mcu_gpio_bank.mcu_gpio_mask))
	{
		mask |= POLLIN | POLLRDNORM;
	}
	UNLOCK_BANK(dev->gpo_bank.lock, flags);

	return mask;
}

static ssize_t virt_gpio_chr_read(struct file *file, char __user *buf,
								  size_t count, loff_t *ppos)
{
	struct virt_gpio *dev = file->private_data;
	uint8_t output[6]; //maximal possible data size needed to transfer info
	int w = 4;		   //default output size
	unsigned long out_mask;
	unsigned long out_values;
	unsigned int gpio_num;

	unsigned long flags; // for spin lock

	output[1] = (uint8_t)GPIO_INT_STATUS; //default, this place in the data should signal the ioriver/control
										  //which kind of operation should it do
	pr_err("%s() READ\n", __func__);
	if (count < sizeof(output))
	{
		pr_err("%s() count too small\n", __func__);
		return -EINVAL;
	}

	//check which action should be done by the iodriver/control after reading this value
	spin_lock_irqsave(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);

	/*if a set/get request is waiting to be processed and to be sent to the MCU:*/
	(WAIT_FOR_PROCESS_BIT&(dev->mcu_gpio_bank.mcu_gpio_mask)) ?
		dev->mcu_gpio_bank.mcu_gpio_mask = 0,
	    //set the variables from the protected buffer, mcu_gpio_bank to local ones
		output[1] = dev->mcu_gpio_bank.gpio_info.op,
		/*use the macro defined in the MCU to the one defined in the MCU*/
		gpio_num = dev->mcu_gpio_bank.gpio_info.gpio_num,
		out_values = dev->mcu_gpio_bank.gpio_info.gpio_value
		: 0 /*Don't do anything while the lock is closed*/;

	spin_unlock_irqrestore(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);

	switch(output[1])
	{	
		//This is the last implementation of the virtual GPIO before MCU-GPIO was added
		case GPIO_INT_STATUS:
			pr_err("GPIO_INT_STATUS!");
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

		if (out_mask)
		{
			output[0] = 0; //iodriver will set that
			//output[1] = 0;//already set before
			output[2] = out_mask & 0xff;
			output[3] = out_values & 0xff;

			if (copy_to_user(buf, output, w))
				return -EINVAL;
			*ppos = 0;

			pr_debug("%s() return %d\n", __func__, w);
			return w;
		}
		else
		{
			pr_debug("%s() return 0 no io change\n", __func__);
		}
		break; //switch case GPIO_INT_STATUS

	case COMM_READ_REQ:
		w = 5;
		pr_err("COMM_READ_REQ!, %ul %u %u",gpio_num,(uint8_t)(gpio_num >> 8), (uint8_t)(gpio_num & 0xFF));
		output[0] = 0;
		//output[1] was set already
		output[2] = 0; //iodriver will set that
		output[3] = (uint8_t)(gpio_num >> 8);
		output[4] = (uint8_t)(gpio_num & 0xFF);

		if (copy_to_user(buf, output, w))
			return -EINVAL;
		*ppos = 0;
		return w;
		break; //switch case COMM_READ_REQ
	case COMM_WRITE_REQ:
		//free the buffer after it was used, note that this action is not protected
		//or atomic as right now (31/3/2019) the logic dictates that in this case
		// it is not as important it might not be the case in the future
		dev->mcu_gpio_bank.mcu_gpio_mask = MCU_FREE_BIT;
		w = 6;
		pr_err("COMM_READ_REQ!");
		output[0] = 0;
		//output[1] was set already
		output[2] = 0; //iodriver will set that
		output[3] = (uint8_t)(gpio_num >> 8);
		output[4] = (uint8_t)(gpio_num & 0xFF);
		output[5] = (uint8_t)out_values;

		if (copy_to_user(buf, output, w))
			return -EINVAL;
		*ppos = 0;
		return w;
		break; //switch case COMM_WRITE_REQ

	default:
		pr_err("invalid option!");
		return 1;

		break; //useless but lets keep up with the convention
	}

	return 0;
}

static ssize_t virt_gpio_chr_write(struct file *file, const char __user *buf,
								   size_t count, loff_t *ppos)
{
	struct virt_gpio *dev = file->private_data;
	uint8_t msg[4];
	int i;
	uint8_t val = 0;
	unsigned long flags;

	if (count != 4)
		return -EINVAL;

	pr_err("virt_gpio_chr_write!");

	if (copy_from_user(msg, buf, count))
		return -EACCES;

	if (msg[0] || msg[1])
	{
		pr_err("%s() unsupported command %x,%x\n", __func__, msg[0], msg[1]);
		return -EINVAL;
	}

	pr_err("writing %02d %02d\n", msg[2], msg[3]);

	//Now check whether a user thread is waiting for response
	spin_lock_irqsave(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);
	if ((dev->mcu_gpio_bank.mcu_gpio_mask) & WAIT_FOR_PROCESS_BIT)
	{
		dev->mcu_gpio_bank.gpio_info.gpio_value = msg[2];
		spin_unlock_irqrestore(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);
	}
	else
	{
		spin_unlock_irqrestore(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);

		// TODO: use macro
		for (i = 0; i < dev->gpiochip_in.ngpio; i++)
		{
			if (msg[2] & (1 << i))
			{
				if (msg[3] & (1 << i))
				{
					//				printk("%s set   INPUT%d\n", __func__, i);
					set_bit(i, &dev->gpi_values);
				}
				else
				{
					//				printk("%s: clear INPUT%d\n", __func__, i);
					clear_bit(i, &dev->gpi_values);
				}
				val |= (1 << i);
			}
		}
		if (val)
		{
			pr_err("gpio_in_notify %d\n", val);
			g_gpio = val;
			gpio_in_notify(1, &g_gpio);
		}
	}

	return count;
}

/////

static int virt_gpio_out_request(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio *dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	set_bit(offset, &dev->enabled_out);

	return 0;
}

static int virt_gpio_mcu_request(struct gpio_chip *chip, unsigned offset)
{
	//struct virt_gpio * dev = g_pvpgio;
	//pr_err("%s() %d\n", __func__, offset);
	//set_bit(offset&0x1F/*modulo 32*/, (unsigned long *)&dev->enabled_mcu[offset>>5/*dividing by 32*/]);

	return 0;
}

static int virt_gpio_in_request(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio *dev = g_pvpgio;
	//	printk("%s: %d\n", __func__, offset);
	set_bit(offset, &dev->enabled_in);

	return 0;
}

static void virt_gpio_out_free(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio *dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	clear_bit(offset, &dev->enabled_out);
}

static void virt_gpio_in_free(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio *dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	clear_bit(offset, &dev->enabled_in);
}

static void virt_gpio_mcu_free(struct gpio_chip *chip, unsigned offset)
{
	//struct virt_gpio * dev = g_pvpgio;
	pr_err("%s() %d\n", __func__, offset);
	//clear_bit(offset&0x1F/*modulo 32*/, (unsigned long *)&dev->enabled_mcu[offset>>5/*dividing by 32*/]);
}

static int virt_gpio_out_get(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static void virt_gpio_mcu_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct virt_gpio *dev = g_pvpgio;
	unsigned long flags;
	spin_lock_irqsave(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);
	/*in case mcu_gpio_bank is free to pass information between the user threads and the read function
	  only one of the threads should be free to do so as only one request can be sent to the MCU at once,
	  this is why I've(Barak) choose to use the "exclusive" flag in here, it also provides thread safety 
	  to this design*/
	if (0 == wait_event_interruptible_exclusive_locked(dev->mcu_gpio_bank.mcu_wq,
													   (MCU_FREE_BIT & (dev->mcu_gpio_bank.mcu_gpio_mask))))
	{
		//Set the request
		dev->mcu_gpio_bank.gpio_info.gpio_num = TURN_A9_GPIO_NUM_TO_MCU(offset);
		dev->mcu_gpio_bank.gpio_info.gpio_value = value;
		dev->mcu_gpio_bank.gpio_info.op = SET;

		//signals vgpio_dev_poll that there
		//is a new pending request to the MCU
		//also removes MCU_FREE_BIT to signal other
		//set/get threads that mcu_gpio_bank is no longer free
		dev->mcu_gpio_bank.mcu_gpio_mask = WAIT_FOR_PROCESS_BIT;
	}
	spin_unlock_irqrestore(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);
}

static int virt_gpio_mcu_get(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio *dev = g_pvpgio;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);
	/*in case mcu_gpio_bank is free to pass information between the user threads and the read function
	  only one of the threads should be free to do so as only one request can be sent to the MCU at once,
	  this is why I've(Barak) chosen to use the "exclusive" flag in here' it also provides thread safety 
	  with this design*/
	if (0 == wait_event_interruptible_exclusive_locked(dev->mcu_gpio_bank.mcu_wq,
													   (MCU_FREE_BIT & (dev->mcu_gpio_bank.mcu_gpio_mask))))
	{
		//Set the request
		dev->mcu_gpio_bank.gpio_info.gpio_num = TURN_A9_GPIO_NUM_TO_MCU(offset);
		dev->mcu_gpio_bank.gpio_info.gpio_value = 0;
		dev->mcu_gpio_bank.gpio_info.op = GET;

		//signals vgpio_dev_poll that there
		//is a new pending request to the MCU
		//also removes MCU_FREE_BIT to signal other
		//set/get threads that mcu_gpio_bank is no longer free
		dev->mcu_gpio_bank.mcu_gpio_mask = WAIT_FOR_PROCESS_BIT;
		pr_err("%s() Wait for process bit %lu\n", __func__, (WAIT_FOR_PROCESS_BIT & (dev->mcu_gpio_bank.mcu_gpio_mask)));
	}

	//now wait for the elapsed time for a response from the MCU,note that a mutex might be better
	//used in here, but the user space process that connects to the MCU: the iodriver might crash
	//and this thread will be locked forever with all the others trying to access this virtual
	//file so a lock iplementation with a timeout is mandatory here
	if (0 < wait_event_interruptible_lock_irq_timeout(dev->mcu_gpio_bank.respond_wq,
													  (RETURNED_FROM_MCU_BIT & (dev->mcu_gpio_bank.mcu_gpio_mask)),
													  dev->mcu_gpio_bank.mcu_wq.lock,	  //this lock protects the data retreived in gpio_info
													  msecs_to_jiffies(TO_FOR_MCU_RESP_MS) //timeout
													  ))
	{

		ret = dev->mcu_gpio_bank.gpio_info.gpio_value; // get the retrieved value
	}
	/*	else
	{

		//to add signal the user that the timer has passed
	
		errno = ETIME;
	}*/

	dev->mcu_gpio_bank.mcu_gpio_mask = MCU_FREE_BIT; //After finishing ,signal other user threads that they are free
													 //to prepeare a new request for the MCU
	spin_unlock_irqrestore(&(dev->mcu_gpio_bank.mcu_wq.lock), flags);

	return ret;
}

static int virt_gpio_in_get(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio *dev = g_pvpgio;

	//    printk("%s: %lx\n", __func__, dev->gpi_values);
	return test_bit(offset, &dev->gpi_values) ? 1 : 0;
}

static void virt_gpio_out_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct virt_gpio *dev = g_pvpgio;
	DEFINE_LOCK_FLAGS(flags); // make last

	pr_debug("%s() offset %d value %d\n", __func__, offset, value);

	LOCK_BANK(dev->gpo_bank.lock, flags);

	__set_bit(offset, &dev->gpo_bank.gpio_mask);
	if (value)
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

static int virt_gpio_mcu_direction_input(struct gpio_chip *chip, unsigned offset)
{
	printk("%s() offset %d input\n", __func__, offset);
	//lets set the two leftmost bits to signal the mcu that the device should be set to input(11) or output(10)
	//those bits are not used by the regular mcu command
	//offset |= (((unsigned)1<<(sizeof(offset) - 1) * 8/*bits in a byte*/));
	virt_gpio_mcu_set(chip, offset, 0);
	return 0;
}

static int virt_gpio_mcu_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	//printk("%s() set output and offset %d to %d\n", __func__, offset, value);

	//virt_gpio_mcu_set(chip, offset,value);

	return 0;
}

static const struct file_operations vgpio_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = virt_gpio_chr_read,
	.write = virt_gpio_chr_write,
	.open = vgpio_dev_open,
	.release = vgpio_dev_release,
	.poll = vgpio_dev_poll,
};

static struct miscdevice vgpio_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vgpio",
	.fops = &vgpio_dev_fops,
};

static int __init virtual_gpio_init(void)
{
	struct virt_gpio *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	g_pvpgio = dev;

	init_waitqueue_head(&dev->gpo_bank.wq);
	init_waitqueue_head(&dev->mcu_gpio_bank.mcu_wq);
	init_waitqueue_head(&dev->mcu_gpio_bank.respond_wq);

#ifdef VGPIO_USE_SPINLOCK
	spin_lock_init(&dev->gpo_bank.lock);
#else
	mutex_init(&dev->gpo_bank.lock);
#endif
	//Set the mcu mask to it's initial state
	dev->mcu_gpio_bank.mcu_gpio_mask = MCU_FREE_BIT;

	ret = misc_register(&vgpio_dev);
	if (ret)
	{
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

	dev->gpiochip_mcu.label = "vgpio_mcu";
	dev->gpiochip_mcu.request = virt_gpio_mcu_request;
	dev->gpiochip_mcu.free = virt_gpio_mcu_free;
	dev->gpiochip_mcu.direction_input = virt_gpio_mcu_direction_input;
	dev->gpiochip_mcu.direction_output = virt_gpio_mcu_direction_output;
	dev->gpiochip_mcu.get = virt_gpio_mcu_get; //returns data in the
	dev->gpiochip_mcu.set = virt_gpio_mcu_set;
	dev->gpiochip_mcu.base = -1;
	dev->gpiochip_mcu.ngpio = 160;
	dev->gpiochip_mcu.can_sleep = 1;

	gpiochip_add(&dev->gpiochip_out);
	pr_debug("out base %d\n", dev->gpiochip_out.base);

	gpiochip_add(&dev->gpiochip_in);
	pr_debug("in base %d\n", dev->gpiochip_in.base);

	gpiochip_add(&dev->gpiochip_mcu);
	pr_err("in base %d\n", dev->gpiochip_mcu.base);

	gpio_in_notify(0, 0);
	//
	ret = sysfs_create_group(&vgpio_dev.this_device->kobj, &in_attr_group);
	if (ret)
	{
		pr_err("%s: could not create sysfs group [%d]\n", __func__, ret);
	}
	//
	return 0;
}

static void __exit virtual_gpio_exit(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
	int ret;
#endif
	struct virt_gpio *dev;
	dev = g_pvpgio;

	if (dev->enabled_out)
	{
		int i;
		pr_info("some vgpio_in gpios are still enabled.\n");
		for (i = 0; i < dev->gpiochip_out.ngpio; i++)
		{
			if ((dev->enabled_out & (1 << i)))
			{
				pr_info("free vgpio_in %d.\n", i);
				gpio_free(dev->gpiochip_out.base + i);
			}
		}
	}

	if (dev->enabled_in)
	{
		int i;
		pr_info("some vgpio_out gpios are still enabled.\n");
		for (i = 0; i < dev->gpiochip_in.ngpio; i++)
		{
			if ((dev->enabled_in & (1 << i)))
			{
				pr_info("free vgpio_out %d.\n", i);
				gpio_free(dev->gpiochip_in.base + i);
			}
		}
	}

	// gpiochip_remove returns void in 3.18
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
	ret =
#endif
		gpiochip_remove(&dev->gpiochip_in);

	// gpiochip_remove returns void in 3.18
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
	ret =
#endif
		gpiochip_remove(&dev->gpiochip_out);

	misc_deregister(&vgpio_dev);

	kfree(dev);
}

module_init(virtual_gpio_init);
module_exit(virtual_gpio_exit);
