#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/pci.h>


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

	pt = (char*) memstart ;

	
	// Auswahl des Kanals fuer den Multiplexer 0x06
	outb(0, ioport + REGISTER_AD_CHANNEL_CONTROL);	

	// Bereich -10 v ... +10 v waehlen  0x08
	outb(AD_B_10_V, ioport + REGISTER_INPUT_SIGNAL_RANGE);

	// AD-Trigger-Mode: Interne Quelle, Software Trigger 0x0A
	outb(1, ioport + REGISTER_TRIGGER_MODE_CONTROL);

	printk("MYPCI - Device init erfolgreich\n");

	return 0;
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






static ssize_t read(struct file *filePointer, char *buff, size_t count, loff_t *offPointer )
{	
	int i;
	short input;
	int timeout=0;	

	int status = 0;

	printk("MYPCI - Read\n");

		
	status = inb(ioport + 0x08);

	if((status & 0x10) != 0) {
		printk("fifo not empty");
		
		while((inb(ioport + 0x08) & 0x10) != 0){
			if(timeout == 100)
				break;
			timeout++;
			
			inw(ioport);
		}
	}

	
	// 0x0E
	outb(1, ioport + REGISTER_SOFTWARE_TRIGGER);
	
	i = 0;
	
	while(i < 100000) {
		i++;
		
		status = 0;	
		status = inb(ioport + 0x08);

		msleep(10);

		if((status & 0x80) != 0){
			break;
		}
	}


	input= inw(ioport);
	
	printk("Input gelesen: %i \n", input);

	if(copy_to_user(buff, &input, sizeof(input)) != 0){
		printk("write error");
	}

	return sizeof(input);
}

static ssize_t write(struct file *filePointer, __user const char *userbuffer, size_t count, loff_t *offPointer )
{

	char buffer[12];
	char vorkomma[12];
	char nachkomma[12];

	int wert = 0;
	int vorkommazahl = 0;
	int nachkommazahl = 0;
	
	int kommaposition = -1;
	int stellen = 0;
	int potenz = 1;

	int ergebnis = 0;
	int i;
	int ret;

	printk("Speicher-Treiber. Write\n");


	if(count > sizeof(buffer)){
		count = sizeof(buffer);
	}

	ret = copy_from_user(buffer, userbuffer, count);
	
	for(i=0 ; i<sizeof(buffer); i++){
		if(buffer[i] == '.' || buffer[i] == ','){
			kommaposition = i;
		}
	}

	if(kommaposition > -1){
		for(i=0 ; i<kommaposition ; i++){
			vorkomma[i] = buffer[i];
		}	

		vorkomma[sizeof(vorkomma) - 1] = '\0';


		for(i=kommaposition+1 ; i<sizeof(buffer) && buffer[i]>='0' && buffer[i]<='9' ; i++){
			nachkomma[i-kommaposition-1] = buffer[i];
		}

		for(i=0 ; i<sizeof(nachkomma) ; i++){
			if(nachkomma[i] != '\0'){
				stellen = i+1;
			}
		}

		for(i=0 ; i<stellen ; i++){
			potenz *= 10;			
		}

		vorkommazahl = simple_strtol(vorkomma, NULL, 10);
		nachkommazahl = simple_strtoul(nachkomma, NULL, 10);
	} else {
		vorkommazahl = simple_strtol(buffer, NULL, 10);
	}	

		
	if(buffer[0] == '-'){
		nachkommazahl *= -1;
	}

	if(vorkommazahl > 9 || vorkommazahl < -9){
		if(vorkommazahl > 9){
			ergebnis = 0xfff;
		}else {
			ergebnis = 0x000;
		}
	} else {
		wert = ((vorkommazahl + 10) * potenz) + nachkommazahl;
		ergebnis = (((wert * 4096) / 20) / potenz);
	}


	outw(ergebnis, ioport);

	printk("\nVorkommazahl: %d \n", vorkommazahl);
	printk("Nachkommazahl: %d \n", nachkommazahl);
	printk("Potenz: %d \n", potenz);
	printk("Rechenwert: %d \n", wert);
	printk("Ausgabewert: %X \n", ergebnis);

	return count;
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

