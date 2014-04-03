
# Name des Treibers
DRIVER_NAME = ueb4_device


ifneq ($(KERNELRELEASE),)

obj-m := $(DRIVER_NAME).o
else
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KDIR) M=$(PWD) -o modules
endif


#Muss als su ausgeführt werden
# Läd das Treibermodul und erzeugt eine Device-Datei
devices: removeDevices
	insmod $(DRIVER_NAME).ko
	mknod /dev/$(DRIVER_NAME) c 240 0
	chmod a+rw /dev/$(DRIVER_NAME)

#Muss als su ausgeführt werden
# Entfernt das Treibermodul und löscht die Device-Datei
removeDevices:
	rm -f /dev/$(DRIVER_NAME)
	rmmod $(DRIVER_NAME) || true	# durch || true wird verhindert, dass das Make-script abbricht, wenn das modul bereits entfernt wurde

# Löscht alle unnötigen Build-Artefakte
clean:
	rm -rf .tmp_versions/
	rm -f *.symvers
	rm -f *.o
	rm -f *.order
	rm -f .*.cmd
	rm -f *.ko
	rm -f *.mod.c
