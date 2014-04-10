
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("Manuel Mauky");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver with major number that can read and write from/to memory");
MODULE_SUPPORTED_DEVICE("none");

#define MODULENAME "ueb4_major"


static int major_number;

static char array[1024];
unsigned long ret;

int min;


static int open(struct inode *deviceFile, struct file *filePointer)
{
	printk("Speicher-Treiber. Open\n");
	return 0;
}


static ssize_t read(struct file *filePointer, char *buff, size_t count, loff_t *offPointer )
{
	printk("Speicher-Treiber. Read\n");

	min = min(strlen(array), count); 

	ret = copy_to_user(buff,array, min);

	return (count-ret);
}

static ssize_t write(struct file *filePointer, const char *buff, size_t count, loff_t *offPointer )
{
	printk("Speicher-Treiber. Write\n");
	
	ret = copy_from_user(array, buff, count);
	
	array[count - ret + 1] = 0;
	

	return (count-ret);
}


static int release(struct inode *GeraeteDatei, struct file *Instanz)
{
	printk("Speicher-Treiber. Release\n");
	return 0;
}



static struct file_operations fops = {
	open:	open,
	read: read,
	write: write,
	release: release,
};


static int __init mod_init(void)
{
	major_number = register_chrdev(0, "Treiber-Test", &fops);
	printk("mini. Driver was registered with Major-Nr: %d\n", major_number);
	return 0;
}

static void __exit mod_exit(void)
{	
	unregister_chrdev(major_number, "Treiber-Test");
	printk("mini. unregistering driver with major-nr: %d\n", major_number);
	return;
}

module_init(mod_init);
module_exit(mod_exit);

