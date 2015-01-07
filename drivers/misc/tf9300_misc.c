/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

#define DBGMSG(e,...) printk("**%s:%d** " e "\n", __func__, __LINE__, ##__VA_ARGS__)

#define ELIJA_GET_MEMORY
#ifdef ELIJA_GET_MEMORY
#include <linux/fic_extend.h>
static int fic_memory_get_open_fs(struct inode *inode, struct file *file);
ssize_t fic_memory_get_seq_read(struct file *, char __user *, size_t, loff_t *);
static int fic_memory_get_seq_show(struct seq_file *seq, void *offset);

#endif

//////////////////////////////////////////////////////////////////////////////
#ifdef ELIJA_GET_MEMORY
const struct file_operations fic_memory_get_fops = {
	.owner = THIS_MODULE,
	.open = fic_memory_get_open_fs,
	.read = seq_read,
	.write = seq_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int fic_memory_get_open_fs(struct inode *inode, struct file *file) {
	return single_open(file, fic_memory_get_seq_show, NULL);
}

static int fic_memory_get_seq_show(struct seq_file *seq, void *offset) {
	long nMemorySize=fic_get_memory_size();
	printk("***%s:%d*** memory: %luk\n", __func__, __LINE__, nMemorySize);
	seq_printf(seq, "%luk\n", nMemorySize);
	return 0;
}


#endif
//////////////////////////////////////////////////////////////
	
static struct proc_dir_entry *proc_totalmemory;

static int __init tf9300_misc_func_init(void)
{
#ifdef ELIJA_GET_MEMORY
	proc_totalmemory = proc_create("totalmemory", 0666, NULL, &fic_memory_get_fops);
#endif	

	return 0;
}

static void __exit tf9300_misc_func_exit(void)
{
	remove_proc_entry("totalmemory", NULL);
}

MODULE_AUTHOR("ChuYuan Chiang <chuyuan.chiang@symbio.com>");
MODULE_DESCRIPTION("For TF9300 misc function driver");
MODULE_LICENSE("GPL");

module_init(tf9300_misc_func_init);
module_exit(tf9300_misc_func_exit);
