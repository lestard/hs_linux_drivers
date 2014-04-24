
//#include <linux/init.h>
//#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
//#include <linux/device.h>
//#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
//#include <linux/interrupt.h>
//#include <linux/unistd.h>

MODULE_AUTHOR("Manuel Mauky");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver für AD/DA Wandlerkarte NuDAQ PCI-9111HR");
MODULE_SUPPORTED_DEVICE("none");

#define MODULENAME "ueb5_ad_da"

#define VENDOR_ID 0x144a
#define DEVICE_ID 0x9111


#define REGISTER_AD_FIFO_VALUE 			0x00
#define REGISTER_DA_OUTPUT 			0x00
#define REGISTER_DIGITAL_IO 			0x02
#define REGISTER_EXTENDED_IO_PORTS 		0x04
#define REGISTER_AD_CHANNEL_CONTROL 		0x06
#define REGISTER_AD_CHANNEL_READBACK 		0x06
#define REGISTER_INPUT_SIGNAL_RANGE 		0x08
#define REGISTER_RANGE_STATUS_READBACK 		0x08
#define REGISTER_TRIGGER_MODE_CONTROL 		0x0A
#define REGISTER_AD_MODE_INTERRUPT_READBACK 	0x0A
#define REGISTER_SOFTWARE_TRIGGER		0x0E
#define REGISTER_INTERRUPT_CONTROL 		0x0C
#define REGISTER_8254_COUNTER_0 		0x40
#define REGISTER_8254_COUNTER_1 		0x42
#define REGISTER_8254_COUNTER_2 		0X44
#define REGISTER_8254_CONTROL 			0x46
#define REGISTER_INTERRUPT_CLEAR 		0x48

#define PTRG_OFF 		(0 << 3)
#define PTRG_ON 		(1 << 3)
#define EITS_EXTERNAL 		(1 << 2)
#define EITS_INTERNAL 		(0 << 2)
#define TPST_SOFTWARE_TRIGGER 	(0 << 1)
#define TPST_TIMER_PACER 	(1 << 1)
#define ASCAN_ON 		(1 << 0)
#define ASCAN_OFF 		(0 << 0)


#define AD_B_10_V	0
#define AD_B_5_V	1
#define AD_B_2_5_V	2
#define AD_B_1_25_V	3
#define AD_B_0_625_V	4

static dev_t mypci_dev_number;
static struct cdev *driver_object;
static struct class *mypci_class;
static struct device *mypci_dev;

static struct pci_device_id ids_table[] = {
	{ VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID},
	{ 0}
};

static unsigned long ioport=0L, iolen=0L, memstart=0L, memlen=0L;

static char *pt;

static int __devinit device_init(struct pci_dev *pdev, const struct pci_device_id *id){
	
	printk("MYPCI device_init\n");
	
	if(pci_enable_device(pdev)){
		printk("MYPCI - enable device failed\n");
		return -EIO;
	}

	ioport = pci_resource_start(pdev, 2);
	iolen = pci_resource_len(pdev, 2);
	
	if(request_region (ioport, iolen, pdev->dev.kobj.name) == NULL) {
		printk("MYPIC - error IO address conflict\n");
		dev_err(&pdev->dev, "I/O address conflict for device \"%s\"\n",
			pdev->dev.kobj.name);
		return -EIO;
	}

/*
	memstart = pci_resource_start( pdev, 2 );
	memlen = pci_resource_len( pdev, 2 );
	if( request_mem_region(memstart,memlen,pdev->dev.kobj.name)==NULL ) {
		printk("MYPCI - error Memory address conflict\n");
		dev_err(&pdev->dev,"Memory address conflict for device\n");
		goto cleanup_ports;
	}
	

	

*/

	pt = (char*) memstart ;

/*
	pt[REGISTER_AD_CHANNEL_CONTROL] = 0;
	
	pt[REGISTER_INPUT_SIGNAL_RANGE] = AD_B_10_V;
*/	

	
	printk("MYPCI - Device init erfolgreich\n");

	return 0;

//cleanup_mem:
//	release_mem_region( memstart, memlen );
cleanup_ports:
	release_region( ioport, iolen );
	return -EIO;
}

static void device_deinit(struct pci_dev *dev){
	printk("MYPCI device_deinit\n");
	
	if(ioport) {
		release_region(ioport,iolen);
	}

	if(memstart) {
		release_mem_region(memstart, memlen);
	}
		
	return;
}



static struct pci_driver pci_driver = {
	.name = MODULENAME,
	.id_table = ids_table,
	.probe = device_init,
	.remove = device_deinit,
};



static char array[1024];
unsigned long ret;



static ssize_t read(struct file *filePointer, char *buff, size_t count, loff_t *offPointer )
{
	int min;

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
	
	
	printk("Schicke den Wert %d an den DA-Wandler\n");

	pt[0] = 
	

	return (count-ret);
}


static struct file_operations fops = {
	write: write,
	read: read,
};


static int __init mod_init(void)
{
	int tmp;
	printk("MYPCI - begin init\n");

	if( alloc_chrdev_region(&mypci_dev_number,0,1,MODULENAME) < 0 )
	{
		return -EIO;
	}
	
	driver_object = cdev_alloc();
	if(driver_object==NULL)
	{
		unregister_chrdev_region(mypci_dev_number, 1);
		return -EIO;
	}

	driver_object->owner = THIS_MODULE;
	driver_object->ops = &fops;

	if(cdev_add(driver_object, mypci_dev_number,1))
	{
		kobject_put(&driver_object->kobj);
		unregister_chrdev_region(mypci_dev_number, 1);
		return -EIO;
	}

	mypci_class = class_create(THIS_MODULE, MODULENAME);
	mypci_dev = device_create(mypci_class, NULL, mypci_dev_number, NULL, "%s", MODULENAME);

	
	tmp = pci_register_driver(&pci_driver);
	printk("MYPCI - register driver returned: >%d<\n", tmp);
	if(tmp < 0) {
		printk("MYPCI - Error register driver\n");
		device_destroy (mypci_class, mypci_dev_number);
		unregister_chrdev_region(mypci_dev_number, 1);
		return -EIO;
	}

	printk("MYPCI - init successful\n");

	return 0;
}

static void __exit mod_exit(void)
{	
	printk("MYPCI - exit\n");

	pci_unregister_driver(&pci_driver);

	device_destroy(mypci_class, mypci_dev_number);
	class_destroy(mypci_class);
	cdev_del(driver_object);
	unregister_chrdev_region(mypci_dev_number, 1);
	return;
}

module_init(mod_init);
module_exit(mod_exit);

