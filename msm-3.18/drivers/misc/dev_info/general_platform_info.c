
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#define VER_BUF_SIZE    32
static char mcu_ver[VER_BUF_SIZE] = {0};
static char fpga_ver[VER_BUF_SIZE] = {0};
static const char *MCU_VER_FILE = "mcu_version";
static const char *FPGA_VER_FILE = "fpga_version";
static struct dentry *debugfs_base = 0;


/////
static int mcu_ver_print(struct seq_file *m, void *v) {
    seq_printf(m, mcu_ver);
    return 0;
}

static int proc_mcu_info_open(struct inode *inode, struct  file *file) 
{
    return single_open(file, mcu_ver_print, NULL);
}
static ssize_t dev_mcu_info_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
    char ver[VER_BUF_SIZE] = {0};

    if (count > sizeof(mcu_ver)) {
        return -EFAULT;
    }

    if (copy_from_user(ver, buffer, count)) { 
        return -EFAULT;
    }
    memset(mcu_ver, 0, sizeof(mcu_ver));
    strncpy(mcu_ver, ver, count);

    return count; 
}
////
static int fpga_ver_print(struct seq_file *m, void *v) {
    seq_printf(m, fpga_ver);
    return 0;
}

static int proc_fpga_info_open(struct inode *inode, struct  file *file) 
{
    return single_open(file, fpga_ver_print, NULL);
}
static ssize_t dev_fpga_info_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
    char ver[VER_BUF_SIZE] = {0};

    if (count > sizeof(fpga_ver)) {
        return -EFAULT;
    }

    if (copy_from_user(ver, buffer, count)) { 
        return -EFAULT;
    }
    memset(fpga_ver, 0, sizeof(fpga_ver));
    strncpy(fpga_ver, ver, count);

    return count; 
}
static const struct file_operations proc_info_mcu_fops = {
        .owner      = THIS_MODULE,
	.open	    = proc_mcu_info_open,
	.read	    = seq_read,
        .write	    = dev_mcu_info_proc_write,
	.llseek	    = seq_lseek,
	.release    = single_release,
};

static const struct file_operations proc_info_fpga_fops = {
        .owner      = THIS_MODULE,
	.open	    = proc_fpga_info_open,
	.read	    = seq_read,
        .write	    = dev_fpga_info_proc_write,
	.llseek	    = seq_lseek,
	.release    = single_release,
};
////////////
static int dev_info_debug_init(void)
{
    debugfs_base = debugfs_create_dir("dev_info", NULL);
    if (!debugfs_base)
            return -ENOMEM;

    if (!debugfs_create_file(MCU_VER_FILE, S_IRUGO | S_IWUGO, debugfs_base, NULL, &proc_info_mcu_fops))
            return -ENOMEM;

    if (!debugfs_create_file(FPGA_VER_FILE, S_IRUGO | S_IWUGO, debugfs_base, NULL, &proc_info_fpga_fops))
            return -ENOMEM;

    return 0;
}

int __init dev_info_init(void)
{
        proc_create(MCU_VER_FILE, S_IRUGO | S_IWUGO, NULL, &proc_info_mcu_fops);
        proc_create(FPGA_VER_FILE, S_IRUGO | S_IWUGO, NULL, &proc_info_fpga_fops);

        dev_info_debug_init();

	return 0;
}

void __exit dev_info_exit(void)
{
    remove_proc_entry(MCU_VER_FILE, NULL);
    remove_proc_entry(FPGA_VER_FILE, NULL);

    if (debugfs_base)
        debugfs_remove_recursive(debugfs_base);
}

module_init(dev_info_init);
module_exit(dev_info_exit);


