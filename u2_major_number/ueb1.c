#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>

MODULE_AUTHOR("Manuel Mauky");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver with major version number");
MODULE_SUPPORTED_DEVICE("none");


static int DriverOpen(struct inode *GeraeteDatei, struct file *Instanz)
{
	printk("mini. DriverOpen\n");
	return 0;
}

static struct file_operations Fops = {
	.open=	DriverOpen,
};

static int major_number;

static int __init mod_init(void)
{
	printk("mini. init\n");
	major_number = register_chrdev(0, "Treiber-Test", &Fops);
	
	printk("mini. Driver was registered with Major-Nr: %d\n", major_number);
	
	return 0;
}

static void __exit mod_exit(void)
{
	printk("mini. unregistering driver with major-nr: %d\n", major_number);
	unregister_chrdev(major_number, "Treiber-Test");
	return;
}

module_init(mod_init);
module_exit(mod_exit);

