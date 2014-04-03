
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>

MODULE_AUTHOR("Manuel Mauky");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver with major version number");
MODULE_SUPPORTED_DEVICE("none");

#define MODULENAME "ueb3"


static dev_t template_dev_number;
static struct cdev *driver_object;
struct class *template_class;




static int DriverOpen(struct inode *GeraeteDatei, struct file *Instanz)
{
	printk("Devicenumber-driver. DriverOpen\n");
	return 0;
}


static struct file_operations fops = {
	.open=	DriverOpen,
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

