
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("Manuel Mauky");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver with device number that can read and write");
MODULE_SUPPORTED_DEVICE("none");

#define MODULENAME "ueb4_device"


static dev_t template_dev_number;
static struct cdev *driver_object;
struct class *template_class;


static char array[1024];


static int open(struct inode *deviceFile, struct file *filePointer)
{
	printk("Speicher-Treiber. Open\n");
	return 0;
}


static ssize_t read(struct file *filePointer, char *buff, size_t count, loff_t *offPointer )
{
	printk("Speicher-Treiber. Read\n");
	unsigned long ret;

	ret = copy_to_user(buff,array, count);

	return (count-ret);
}

static ssize_t write(struct file *filePointer, const char *buff, size_t count, loff_t *offPointer )
{
	printk("Speicher-Treiber. Write\n");
	
	unsigned long ret;
	
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

	if( alloc_chrdev_region(&template_dev_number,0,1,MODULENAME) < 0 )
	{
		return -EIO;
	}
	
	driver_object = cdev_alloc();
	if(driver_object==NULL)
	{
		unregister_chrdev_region(template_dev_number, 1);
		return -EIO;
	}

	driver_object->owner = THIS_MODULE;
	driver_object->ops = &fops;

	if(cdev_add(driver_object, template_dev_number,1))
	{
		kobject_put(&driver_object->kobj);
		unregister_chrdev_region(template_dev_number, 1);
		return -EIO;
	}

	template_class = class_create(THIS_MODULE, MODULENAME);
	device_create(template_class, NULL, template_dev_number, NULL, "%s", MODULENAME);

	return 0;
}

static void __exit mod_exit(void)
{	
	device_destroy(template_class, template_dev_number);
	class_destroy(template_class);
	cdev_del(driver_object);
	unregister_chrdev_region(template_dev_number, 1);
	return;
}

module_init(mod_init);
module_exit(mod_exit);

