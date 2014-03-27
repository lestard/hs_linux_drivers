#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>

MODULE_AUTHOR("Manuel Mauky");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A simple Module test");
MODULE_SUPPORTED_DEVICE("none");

static int __init mod_init(void)
{
	printk("mod_init called\n");
	return 0;
}

static void __exit mod_exit(void)
{
	printk("mod_exit called\n");
}

module_init(mod_init);
module_exit(mod_exit);

