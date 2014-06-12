
# Name des Treibers
DRIVER_NAME = ueb6_usb_osr

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

#Muss als su ausgeführt werden
# Entfernt das Treibermodul und löscht die Device-Datei
# durch || true wird verhindert, dass das Make-script abbricht, wenn das modul bereits entfernt wurde
removeDevices:
	rmmod $(DRIVER_NAME) || true	

# Löscht alle unnötigen Build-Artefakte
clean:
	rm -rf .tmp_versions/
	rm -f *.symvers
	rm -f *.o
	rm -f *.order
	rm -f .*.cmd
	rm -f *.ko
	rm -f *.mod.c
	rm -f *.out
