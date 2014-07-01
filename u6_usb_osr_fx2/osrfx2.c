/*****************************************************************************/
/* osrfx2.c    A Driver for the OSR USB FX2 Learning Kit device              */
/*                                                                           */
/* Copyright (C) 2006 by Robin Callender                                     */
/*                                                                           */
/* This program is free software. You can redistribute it and/or             */
/* modify it under the terms of the GNU General Public License as            */
/* published by the Free Software Foundation, version 2.                     */
/*                                                                           */
/*****************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/smp_lock.h>

/*****************************************************************************/
/* Die Vendor und Produkt-ID                                                 */
/*****************************************************************************/
#define VENDOR_ID   0x0547       
#define PRODUCT_ID  0x1002

#define DEVICE_MINOR_BASE   192
             
#undef TRUE
#define TRUE  (1)
#undef FALSE
#define FALSE (0)

/*****************************************************************************/
/* Die Kommandos, die das OSR-FX2 unterstützt.                               */
/*****************************************************************************/
#define OSRFX2_READ_SWITCHES              0xD6
#define OSRFX2_READ_BARGRAPH_DISPLAY      0xD7
#define OSRFX2_SET_BARGRAPH_DISPLAY       0xD8
#define OSRFX2_REENUMERATE                0xDA
              
/*****************************************************************************/
/* BARGRAPH_STATE ist eine Bit-Struktur die dem LED-Balken auf dem           */
/* OSR FX2 Board entspricht. Es repräsentiert also den Zustand der LEDs      */
/*****************************************************************************/
struct bargraph_state {
    union {
        struct {
            /*
			* Die einzelnen LEDs beginnend mit der obersten LED auf der Platine.
			* Der LED-Balken besitzt 10 LEDs, jedoch sind die beiden obersten 
			* nicht angeschlossen. Sie können daher nicht zum leuchten gebracht
			* werden und sind deshalb auch hier nicht aufgeführt. 
			*/
            unsigned char Bar4 : 1;
            unsigned char Bar5 : 1;
            unsigned char Bar6 : 1;
            unsigned char Bar7 : 1;
            unsigned char Bar8 : 1;
            unsigned char Bar1 : 1;
            unsigned char Bar2 : 1;
            unsigned char Bar3 : 1;
        };
        /*
         *  Der Zustand aller 8 LEDs in einem Char kodiert.
         */
        unsigned char BarsOctet;
    };
} __attribute__ ((packed));

static const unsigned char digit_to_segments [10] = { 
    0xD7,  /* 0 */
    0x06,  /* 1 */
    0xB3,  /* 2 */
    0xA7,  /* 3 */
    0x66,  /* 4 */
    0xE5,  /* 5 */
    0xF5,  /* 6 */
    0x07,  /* 7 */
    0xF7,  /* 8 */
    0x67   /* 9 */
};

/*****************************************************************************/
/* Diese Struktur kapselt den Zustand der Schalter-Leiste.                   */
/*****************************************************************************/
struct switches_state {
    union {
        struct {
            /*
             * Individual switches starting from the left
             */
            unsigned char Switch8 : 1;
            unsigned char Switch7 : 1;
            unsigned char Switch6 : 1;
            unsigned char Switch5 : 1;
            unsigned char Switch4 : 1;
            unsigned char Switch3 : 1;
            unsigned char Switch2 : 1;
            unsigned char Switch1 : 1;
        };
        /*
         *  The state of all the switches as a single octet.
         */
        unsigned char SwitchesOctet;
    };
} __attribute__ ((packed));

/*****************************************************************************/
/* ID-Tabelle mit Geräte-IDs die mit diesem Treiber funktionieren            */
/*****************************************************************************/
static struct usb_device_id id_table [] = {
    { USB_DEVICE( VENDOR_ID, PRODUCT_ID ) },
    { },
};

MODULE_DEVICE_TABLE(usb, id_table);

/*****************************************************************************/
/* Hiermit wird die interne Struktur und der Zustand des Geräts abgebildet.  */
/*****************************************************************************/
struct osrfx2 {

    struct usb_device    * udev;
    struct usb_interface * interface;
    
	/*
	 * Queue die zum Polling und für IRQ methoden benutzt wird.
	 */ 
    wait_queue_head_t FieldEventQueue;
    
    /*
     *  Buffers für den Transfer von Daten
     */
    unsigned char * int_in_buffer;
    unsigned char * bulk_in_buffer;
    unsigned char * bulk_out_buffer;
    
    /*
     *  Die Größen der einzelnen Buffer
     */
    size_t int_in_size;
    size_t bulk_in_size;
    size_t bulk_out_size;
    
    /*
     *  USB Endpoints
     */
    __u8  int_in_endpointAddr;
    __u8  bulk_in_endpointAddr;
    __u8  bulk_out_endpointAddr;
    
    /*
     *  Intervalle für die Endpunkte
     */
    __u8  int_in_endpointInterval;
    __u8  bulk_in_endpointInterval;
    __u8  bulk_out_endpointInterval;
    
    /*
     *  USB Request Blocks (URBs)
     */
    struct urb * bulk_in_urb;
    struct urb * int_in_urb;
    struct urb * bulk_out_urb;
    
    /*
     *  Referenz-Zähler
     */
    struct kref      kref;
    struct semaphore sem;

    /*
     *  Hier werden die Daten des Interrupts festgehalten.
     */
    struct switches_state  switches;

    unsigned char notify;

    /*
     * Zähler um Bulk-Lese- und Bulk-Schreibzugriffe mitzählen zu können. 
     */
    atomic_t bulk_write_available;
    atomic_t bulk_read_available;

    /*
     * Zähler für die geschriebenen und gelesenen Daten.
     * Bei Lese-Zugriffen wird der Zähler heruntergezählt, 
     * bei Schreibzugriffen wird heraufgezählt. 
     *
     * Dies wird benutzt, da laut Spec die Firmware des Boards bis zu 4 
     * Schreib-Pakete puffern kann. 
     * Beim fünften ankommenden Schreib-Paket wird der Schreib-Zugriff blockiert.
     * Es wird dann auf eine Lese-Anfrage gewartet um den Puffer wieder zu leeren. 
     * Erst danach können wieder neue Pakete geschrieben werden.
     *
     * Um die Gepufferten Pakete mitzählen zu können wird dieser Zähler benutzt.
     */
    size_t  pending_data;

    /*
     *  Power Management: Gibt an ob sich das Gerät gerade im Ruhemodus befindet
     */
    int   suspended;        /* boolean */

};
 
/*****************************************************************************/
/* Die Treiber-Struktur wird hier schon angelegt um sie im folgenden Code    */
/* benutzt werden kann. Die tatsächliche Wertzuweisung erfolgt weiter unten. */
/*****************************************************************************/
static struct usb_driver osrfx2_driver;

/*****************************************************************************/
/* Hier wird die Struktur für Interrupts der Schalter-Leiste definiert.      */
/*****************************************************************************/
struct interrupt_packet {

    struct switches_state  switches;

} __attribute__ ((packed));

/*****************************************************************************/
/* Mit dieser Methode wird der Zustand der Schalter-Leiste abgefragt und     */
/* als formatierter String (return param buf) zurück gegeben.                */
/*                                                                           */
/* Note the two different function defintions depending on kernel version.   */
/*****************************************************************************/
static ssize_t show_switches(struct device * dev, 
                             struct device_attribute * attr, 
                             char * buf)
{
    struct usb_interface   * intf   = to_usb_interface(dev);
    struct osrfx2          * fx2dev = usb_get_intfdata(intf);
    struct switches_state  * packet;
    int retval;

    packet = kmalloc(sizeof(*packet), GFP_KERNEL);
    if (!packet) {
        return -ENOMEM;
    }
    packet->SwitchesOctet = 0;

	// Hier wird der Zustand der Schalter vom Gerät abgefragt
    retval = usb_control_msg(fx2dev->udev, 
                             usb_rcvctrlpipe(fx2dev->udev, 0), 
                             OSRFX2_READ_SWITCHES, 
                             USB_DIR_IN | USB_TYPE_VENDOR,
                             0,
                             0,
                             packet, 
                             sizeof(*packet),
                             USB_CTRL_GET_TIMEOUT);

    if (retval < 0) {
        dev_err(&fx2dev->udev->dev, "%s - retval=%d\n", __FUNCTION__, retval);
        kfree(packet);
        return retval;
    }

	// Formatiertes Schreiben der Schalter-Zustände in den buf-Ausgabe-String
    retval = sprintf(buf, "%s%s%s%s%s%s%s%s",    /* left sw --> right sw */
                     (packet->Switch1) ? "*" : ".",
                     (packet->Switch2) ? "*" : ".",
                     (packet->Switch3) ? "*" : ".",
                     (packet->Switch4) ? "*" : ".",
                     (packet->Switch5) ? "*" : ".",
                     (packet->Switch6) ? "*" : ".",
                     (packet->Switch7) ? "*" : ".",
                     (packet->Switch8) ? "*" : "." );

    kfree(packet);

    return retval;
}

/*****************************************************************************/
/* Das Macro "DEVICE_ATTR" kommt aus dem sysfs.                              */
/* Es wird benutzt um Geräte-Attribute anzulegen.                            */
/*                                                                           */
/* Der erste Parameter ist der Name des Attributs. Anschließend werden die   */
/* Zugriffsmodi definiert. Hier können Werte aus linux/stat.h benutzt werden.*/ 
/* In unserem Fall 'S_IRUGO' für Read+User+Group+Others.                     */
/* Die letzten beiden Parameter sind Methoden-Referenzen für                 */
/* "show" und "store", also zum Lesen und Schreiben des Attributs.	         */
/*                                                                           */
/* Da wir hier an dieser Stelle nur den Zustand der Schalter lesen wollen,   */
/* ist der letzte Parameter für die Store-methode NULL.                      */
/* 																	         */
/* Das Macro legt das Attribut im sysfs Verzeichnis an unter:	             */
/* --- /sys/bus/usb/devices/<root_hub>-<hub>:1.0/switches	                 */
/* 															                 */
/* Das Attribut bekommt den namen "dev_attr_switches" und wird von diesem	 */
/* Treiber z.B. in der Probe und Disconnect Methode benutzt.				 */
/*****************************************************************************/
static DEVICE_ATTR( switches, S_IRUGO, show_switches, NULL );


/*****************************************************************************/
/* Diese Methode liest den Zustand der Bargraph LED-Leiste ein. Dieser wird  */
/* als formatierter String zurück gegeben.									 */
/*****************************************************************************/
static ssize_t show_bargraph(struct device * dev, 
                             struct device_attribute * attr, 
                             char * buf)
{
    struct usb_interface  * intf   = to_usb_interface(dev);
    struct osrfx2         * fx2dev = usb_get_intfdata(intf);
    struct bargraph_state * packet;
    int retval;

    packet = kmalloc(sizeof(*packet), GFP_KERNEL);
    if (!packet) {
        return -ENOMEM;
    }
    packet->BarsOctet = 0;
	
	// Einlesen des Zustands der LED-Leiste vom Gerät
    retval = usb_control_msg(fx2dev->udev, 
                             usb_rcvctrlpipe(fx2dev->udev, 0), 
                             OSRFX2_READ_BARGRAPH_DISPLAY, 
                             USB_DIR_IN | USB_TYPE_VENDOR,
                             0,
                             0,
                             packet, 
                             sizeof(*packet),
                             USB_CTRL_GET_TIMEOUT);

    if (retval < 0) {
        dev_err(&fx2dev->udev->dev, "%s - retval=%d\n", 
                __FUNCTION__, retval);
        kfree(packet);
        return retval;
    }
	
	// Formatiertes Schreiben in den Ausgabe-Puffer
    retval = sprintf(buf, "%s%s%s%s%s%s%s%s",    /* bottom LED --> top LED */
                     (packet->Bar1) ? "*" : ".",
                     (packet->Bar2) ? "*" : ".",
                     (packet->Bar3) ? "*" : ".",
                     (packet->Bar4) ? "*" : ".",
                     (packet->Bar5) ? "*" : ".",
                     (packet->Bar6) ? "*" : ".",
                     (packet->Bar7) ? "*" : ".",
                     (packet->Bar8) ? "*" : "." );

    kfree(packet);

    return retval;
}

/*****************************************************************************/
/* Diese Methode wird zum setzen der Bargraph LED-Leiste benutzt. 			 */
/*****************************************************************************/
static ssize_t set_bargraph(struct device * dev, 
                            struct device_attribute * attr, 
                            const char * buf,
                            size_t count)
{
    struct usb_interface  * intf   = to_usb_interface(dev);
    struct osrfx2         * fx2dev = usb_get_intfdata(intf);
    struct bargraph_state * packet;

    unsigned int value;
    int retval;
    char * end;

    packet = kmalloc(sizeof(*packet), GFP_KERNEL);
    if (!packet) {
        return -ENOMEM;
    }
    packet->BarsOctet = 0;
	
	// entgegeben nehmen des zu setzenden Werts aus Eingabe-Puffer "bug"
    value = (simple_strtoul(buf, &end, 10) & 0xFF);
    if (buf == end) {
        value = 0;
    }

	// Übernehmen/Umsetzen des eingelesenen Werts in die Bargraph-Status-Struktur
    packet->Bar1 = (value & 0x01) ? 1 : 0;
    packet->Bar2 = (value & 0x02) ? 1 : 0;
    packet->Bar3 = (value & 0x04) ? 1 : 0;
    packet->Bar4 = (value & 0x08) ? 1 : 0;
    packet->Bar5 = (value & 0x10) ? 1 : 0;
    packet->Bar6 = (value & 0x20) ? 1 : 0;
    packet->Bar7 = (value & 0x40) ? 1 : 0;
    packet->Bar8 = (value & 0x80) ? 1 : 0;

	// Nun wird die Bargraph-Struktur mit den zu setzenden Werten an das USB-Gerät gesendet
    retval = usb_control_msg(fx2dev->udev, 
                             usb_sndctrlpipe(fx2dev->udev, 0), 
                             OSRFX2_SET_BARGRAPH_DISPLAY, 
                             USB_DIR_OUT | USB_TYPE_VENDOR,
                             0,
                             0,
                             packet, 
                             sizeof(*packet),
                             USB_CTRL_GET_TIMEOUT);

    if (retval < 0) {
        dev_err(&fx2dev->udev->dev, "%s - retval=%d\n", 
                __FUNCTION__, retval);
    }
    
    kfree(packet);

    return count;
}

/*****************************************************************************/
/* Hier wird ein Geräte Attribut zum lesen und setzen der LED-Leiste 		 */ 
/* im sysfs angelegt, äquivalent zum obigen Aufruf bei "show_switches"       */
/*   ---  /sys/bus/usb/devices/<root_hub>-<hub>:1.0/bargraph                 */
/*                                                     						 */
/* Eine besonderheit hier ist, dass diesmal das Attribut nicht nur lesend,	 */
/* sondern auch schreibend benutzt werden kann. Deshalb wird bei den 		 */
/* Zugriffs-Modi zusätzlich 'S_IWUGO' (Write+User+Group+Other) und 			 */
/* als letzter Parameter die Methodenreferenz zu 'set_bargraph' 			 */
/* als Store-Methode mitgegeben.											 */
/* 																			 */
/* Es wird ein Attribut mit namen "dev_attr_bargraph" angelegt.				 */
/*****************************************************************************/
static DEVICE_ATTR( bargraph, S_IRUGO | S_IWUGO, show_bargraph, set_bargraph );



/*****************************************************************************/
/* Immer wenn ein Schalter am Gerät gedrückt wird, wird ein Interrupt-Paket  */	
/* gesendet. Diese Methode reagiert auf dieses Interrupt-Paket 				 */
/* und behandelt es.														 */
/*****************************************************************************/
static void interrupt_handler(struct urb * urb)
{
    struct osrfx2           * fx2dev = urb->context;
    struct interrupt_packet * packet = urb->transfer_buffer;
    int retval;

	// Im Normal-Fall sollte der Status 0 sein
    if (urb->status == 0) {

        /* 
		 *  Hole zunächst die neuen Schalter-Zustände vom Gerät ab.
         */
        fx2dev->switches.SwitchesOctet = packet->switches.SwitchesOctet;
        fx2dev->notify = TRUE;
        
        /*
		 * Alle wartenden Requests in der Queue werden aufgeweckt.
         */
        wake_up(&(fx2dev->FieldEventQueue));

        /* 
         *  Der Interrupt URB muss erneut gesetzt werden damit in der Zukunft wieder auf neue Interrupts reagiert werden kann.
         */
        retval = usb_submit_urb(urb, GFP_ATOMIC);
        if (retval != 0) {
            dev_err(&urb->dev->dev, "%s - error %d submitting interrupt urb\n",
                    __FUNCTION__, retval);
        }

        /* 
         *  Successful completion 
         */
        return;   
    } 
	
	/* Es gab wohl einen Fehler. Wenn der Return-Code bekannt ist, wird still returnt. Andernfalls wird eine Fehlermeldung ausgegeben.*/
    switch (urb->status) {
        case -ECONNRESET:
        case -ENOENT:
        case -ESHUTDOWN:
            return;
        default: 
            dev_err(&urb->dev->dev, "%s - non-zero urb status received: %d\n", 
                    __FUNCTION__, urb->status);
            return;
    }
}

/*****************************************************************************/
/* Diese Methode registriert den Interrupt-Handler beim Gerät damit auf 	 */
/* Interrupts reagiert werden kann.                                          */
/*****************************************************************************/
static int init_interrupts(struct osrfx2 * fx2dev)
{
    int pipe;
    int retval;

	// Hole die 'pipe' für das Gerät und den Endpunkt.
	// Eine 'Pipe' ist ein Integer, in den Verschiedene Informationen bitweise kodiert sind, wie: 
	// "device number", "endpoint number", "current data", "speed" und den Typ (control, interrupt, bulk, isochronous).
	// Die konkrete Kodierung kann in linux/usb.h angeschaut werden.
    pipe = usb_rcvintpipe(fx2dev->udev, fx2dev->int_in_endpointAddr); // Pipe für Control-Transfer
    
    fx2dev->int_in_size = sizeof(struct interrupt_packet);  
    
    fx2dev->int_in_buffer = kmalloc(fx2dev->int_in_size, GFP_KERNEL);
    if (!fx2dev->int_in_buffer) {
        return -ENOMEM;
    }

    fx2dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!fx2dev->int_in_urb) {
        return -ENOMEM;
    }

	// Initialisieren des Interrupt-URBs
    usb_fill_int_urb( fx2dev->int_in_urb, 	// ptr auf den zu initialisierenden URB
                      fx2dev->udev,			// ptr auf das Gerät zu diesem URB
                      pipe,					// die Endpunkt-Pipe
                      fx2dev->int_in_buffer, 	// der Transfer-Buffer
                      fx2dev->int_in_size,		// die Länge des Transfer-Buffers
                      interrupt_handler, 		// Methoden-Referenz auf den Interrupt-Handler
                      fx2dev,					// Der Geräte-Kontext, in dem der URB gesetzt wird
                      fx2dev->int_in_endpointInterval );	// Der Intervall des URBs

    retval = usb_submit_urb( fx2dev->int_in_urb, GFP_KERNEL );
    if (retval != 0) {
        dev_err(&fx2dev->udev->dev, "usb_submit_urb error %d \n", retval);
        return retval;
    }

    return 0; 
}

/*****************************************************************************/
/*          TODO                                                             */
/*****************************************************************************/
static int init_bulks(struct osrfx2 * fx2dev)
{
    fx2dev->bulk_in_buffer = kmalloc(fx2dev->bulk_in_size, GFP_KERNEL);
    if (!fx2dev->bulk_in_buffer) {
        return -ENOMEM;
    }
    fx2dev->bulk_out_buffer = kmalloc(fx2dev->bulk_out_size, GFP_KERNEL);
    if (!fx2dev->bulk_out_buffer) {
        return -ENOMEM;
    }

    init_MUTEX( &fx2dev->sem );
    init_waitqueue_head( &fx2dev->FieldEventQueue );

    return 0; 
}

/*****************************************************************************/
/* TODO This routine will attempt to locate the required endpoints and       */
/* retain relevant information in the osrfx2 structure instance.             */
/*****************************************************************************/
static int find_endpoints(struct osrfx2 * fx2dev)
{
    struct usb_interface * interface = fx2dev->interface;
    struct usb_endpoint_descriptor * endpoint;
    unsigned char dir;
    unsigned char attr;
    int i;

    for (i=0; i < interface->cur_altsetting->desc.bNumEndpoints; i++) {

        endpoint = &interface->cur_altsetting->endpoint[i].desc;
        dir  = endpoint->bEndpointAddress & USB_DIR_IN;
        attr = endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

        switch ((dir << 8) + attr) {
            case ((USB_DIR_IN << 8) + USB_ENDPOINT_XFER_INT) :
                fx2dev->int_in_endpointAddr = endpoint->bEndpointAddress;
                fx2dev->int_in_endpointInterval = endpoint->bInterval;
                fx2dev->int_in_size = endpoint->wMaxPacketSize;
                break;
            case ((USB_DIR_IN << 8) + USB_ENDPOINT_XFER_BULK) :
                fx2dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
                fx2dev->bulk_in_endpointInterval = endpoint->bInterval;
                fx2dev->bulk_in_size = endpoint->wMaxPacketSize;
                break;
            case ((USB_DIR_OUT << 8) + USB_ENDPOINT_XFER_BULK) :
                fx2dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
                fx2dev->bulk_out_endpointInterval = endpoint->bInterval;
                fx2dev->bulk_out_size = endpoint->wMaxPacketSize;
                break;
            default:
                break;
        }
    }
    if (fx2dev->int_in_endpointAddr   == 0 || 
        fx2dev->bulk_in_endpointAddr  == 0 ||
        fx2dev->bulk_out_endpointAddr == 0) {
        dev_err(&interface->dev, "%s - failed to find required endpoints\n", 
                __FUNCTION__);
        return -ENODEV;
    }
    return 0;
}

/*****************************************************************************/
/* TODO                                                                      */
/*****************************************************************************/
static void osrfx2_delete(struct kref * kref)
{
    struct osrfx2 * fx2dev = container_of(kref, struct osrfx2, kref);

    usb_put_dev( fx2dev->udev );
    
    if (fx2dev->int_in_urb) {
        usb_free_urb(fx2dev->int_in_urb);
    }
    if (fx2dev->int_in_buffer) {
        kfree(fx2dev->int_in_buffer);
    }
    if (fx2dev->bulk_in_buffer) {
        kfree( fx2dev->bulk_in_buffer );
    }
    if (fx2dev->bulk_out_buffer) {
        kfree( fx2dev->bulk_out_buffer );
    }

    kfree( fx2dev );
}

/*****************************************************************************/
/* TODO                                                                      */
/* osrfx2_open                                                               */
/*                                                                           */
/* Note:                                                                     */
/*   The serialization method used below has a side-effect which I don't     */
/*   particularly care for. In effect switch_events and bulk I/O are         */
/*   mutually exclusive, e.g. an open for switch_events will exclude         */
/*   opens for bulk I/O (osrbulk) and vis-a-verse.                           */
/*                                                                           */
/* Note:                                                                     */
/*   The usb_clear_halt() is being used to effect a pipe reset.              */
/*   This will clear any residual data at the endpoint and ready it for      */
/*   the new endpoint user's data.                                           */
/*****************************************************************************/
static int osrfx2_open(struct inode * inode, struct file * file)
{
    struct usb_interface * interface;
    struct osrfx2 * fx2dev;
    int retval;
    int flags;
    
    interface = usb_find_interface(&osrfx2_driver, iminor(inode));
    if (interface == NULL) 
        return -ENODEV;

    fx2dev = usb_get_intfdata(interface);
    if (fx2dev == NULL) 
        return -ENODEV;

    /*
     *   Serialize access to each of the bulk pipes.
     */ 
    flags = (file->f_flags & O_ACCMODE);

    if ((flags == O_WRONLY) || (flags == O_RDWR)) {
        if (atomic_dec_and_test( &fx2dev->bulk_write_available ) == 0) {
            atomic_inc( &fx2dev->bulk_write_available );
            return -EBUSY;
        }

        /*
         *   The write interface is serialized, so reset bulk-out pipe (ep-6).
         */
        retval = usb_clear_halt(fx2dev->udev, fx2dev->bulk_out_endpointAddr);
        if ((retval != 0) && (retval != -EPIPE)) {
            dev_err(&interface->dev, "%s - error(%d) usb_clear_halt(%02X)\n", 
                    __FUNCTION__, retval, fx2dev->bulk_out_endpointAddr);
        }
    }

    if ((flags == O_RDONLY) || (flags == O_RDWR)) {
        if (atomic_dec_and_test( &fx2dev->bulk_read_available ) == 0) {
            atomic_inc( &fx2dev->bulk_read_available );
            if (flags == O_RDWR) 
                atomic_inc( &fx2dev->bulk_write_available );
            return -EBUSY;
        }

        /*
         *   The read interface is serialized, so reset bulk-in pipe (ep-8).
         */
        retval = usb_clear_halt(fx2dev->udev, fx2dev->bulk_in_endpointAddr);
        if ((retval != 0) && (retval != -EPIPE)) {
            dev_err(&interface->dev, "%s - error(%d) usb_clear_halt(%02X)\n", 
                    __FUNCTION__, retval, fx2dev->bulk_in_endpointAddr);
        }
    }

    /*
     *   Set this device as non-seekable.
     */ 
    retval = nonseekable_open(inode, file);
    if (retval != 0) {
        return retval;
    }

    /*
     *   Increment our usage count for the device.
     */
    kref_get(&fx2dev->kref);

    /*
     *   Save pointer to device instance in the file's private structure.
     */
    file->private_data = fx2dev;

    return 0;
}

/*****************************************************************************/
/*          TODO                                                             */
/*****************************************************************************/
static int osrfx2_release(struct inode * inode, struct file * file)
{
    struct osrfx2 * fx2dev;
    int flags;

    fx2dev = (struct osrfx2 *)file->private_data;
    if (fx2dev == NULL)
        return -ENODEV;

    /* 
     *  Release any bulk_[write|read]_available serialization.
     */
    flags = (file->f_flags & O_ACCMODE);

    if ((flags == O_WRONLY) || (flags == O_RDWR))
        atomic_inc( &fx2dev->bulk_write_available );

    if ((flags == O_RDONLY) || (flags == O_RDWR)) 
        atomic_inc( &fx2dev->bulk_read_available );

    /* 
     *  Decrement the ref-count on the device instance.
     */
    kref_put(&fx2dev->kref, osrfx2_delete);
    
    return 0;
}

/*****************************************************************************/
/*   Lesezugriff                                                             */
/*****************************************************************************/
static ssize_t osrfx2_read(struct file * file, char * buffer, 
                           size_t count, loff_t * ppos)
{
    struct osrfx2 * fx2dev;
    int retval = 0;
    int bytes_read;
    int pipe;

    fx2dev = (struct osrfx2 *)file->private_data;

    /* Hole die Pipe für den Bulk-Transfer. */
    pipe = usb_rcvbulkpipe(fx2dev->udev, fx2dev->bulk_in_endpointAddr),

    /* 
     *  Führe einen Bulk-Lesezugriff aus um Daten vom Gerät zu bekommen.
     */
    retval = usb_bulk_msg( fx2dev->udev, 
                           pipe,
                           fx2dev->bulk_in_buffer, // hier werden die gelesenen Daten abgelegt
                           min(fx2dev->bulk_in_size, count), // die zu sendende Länge
                           &bytes_read, // die tatsächlich übertragene Länge
                           10000 ); // timeout

    /* 
     *  Wenn das Lesen erfolgreich war, müssen die Daten zum Userspace kopiert werden.
     */
    if (!retval) {
        if (copy_to_user(buffer, fx2dev->bulk_in_buffer, bytes_read)) {
            retval = -EFAULT;
        }
        else {
            retval = bytes_read;
        }
        
        /*
         *  Hier wird der Counter der gelesenen Bytes runtergezählt.
         * Siehe Dokumentation zu "pending_data" weiter oben.
         */
        fx2dev->pending_data -= retval;
    }

    return retval;
}

/*****************************************************************************/
/*    Schreib-Zugriff BULK                                                   */
/* Diese Hilfsfunktion wird beim Schreiben als asynchroner Callback benutzt. */
/* Die Funktion wird aufgerufen wenn der Schreibzugriff erfolgreich war.     */
/*****************************************************************************/
static void write_bulk_backend(struct urb * urb)
{
    struct osrfx2 * fx2dev = (struct osrfx2 *)urb->context;

    /* 
     *  Bestimmte Events sollen nicht als Fehler auftauchen und werden deshalb gefiltert.
     */
    if (urb->status && 
        !(urb->status == -ENOENT ||         // Wenn einer dieser 
          urb->status == -ECONNRESET ||     // Events auftritt,
          urb->status == -ESHUTDOWN)) {     // wird das nicht als Fehler gewertet
        dev_err(&fx2dev->interface->dev, 
                "%s - non-zero status received: %d\n",
                __FUNCTION__, urb->status);
    }

    /* 
     *  Verschiedene Buffer werden geleert, die beim Schreiben benutzt wurden.
     */
    usb_buffer_free( urb->dev, 
                     urb->transfer_buffer_length, 
                     urb->transfer_buffer, 
                     urb->transfer_dma );
}

/*****************************************************************************/
/*     Schreib-Zugriff                                                       */
/*****************************************************************************/
static ssize_t osrfx2_write(struct file * file, const char * user_buffer, 
                            size_t count, loff_t * ppos)
{
    struct osrfx2 * fx2dev;
    struct urb * urb = NULL;
    char * buf = NULL;
    int pipe;
    int retval = 0;

    fx2dev = (struct osrfx2 *)file->private_data;

    if (count == 0)
        return count;

    /* 
     *  Erzeuge einen USB Request Block
     */
    urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!urb) {
        retval = -ENOMEM;
        goto error;
    }

    /*
     * Erzeuge einen Buffer für den URB
     */
    buf = usb_buffer_alloc( fx2dev->udev, 
                            count, 
                            GFP_KERNEL, 
                            &urb->transfer_dma );
    if (!buf) {
        retval = -ENOMEM;
        goto error;
    }

    /*
     * Kopiere die zu schreibenden Daten in den URB-Puffer
     */ 
    if (copy_from_user(buf, user_buffer, count)) {
        retval = -EFAULT;
        goto error;
    }

    /* 
     * Es wird eine Pipe erzeugt für den Bulk-Sende-Betrieb 
     */
    pipe = usb_sndbulkpipe( fx2dev->udev, fx2dev->bulk_out_endpointAddr );

    // Hier wird der URB initialisiert und konfiguriert
    usb_fill_bulk_urb( urb, 
                       fx2dev->udev,
                       pipe,
                       buf, // die Daten, die transferiert werden
                       count, // die länge der zu sendenden Daten
                       write_bulk_backend, // Funktionsreferenz, 
                            //die aufgerufen wird nachdem das Senden erfolgreich war.
                       fx2dev );

    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    /* 
     *  Sende den URB an das Gerät. 
     */
    retval = usb_submit_urb(urb, GFP_KERNEL);
    if (retval) {
        dev_err(&fx2dev->interface->dev, "%s - usb_submit_urb failed: %d\n",
                __FUNCTION__, retval);
        goto error;
    }

    /*
     *  Der pending_data Zähler wird hochgezählt.
     */
    fx2dev->pending_data += count;

    /* 
     *  Der URB kann wieder freigegeben werden.
     */
    usb_free_urb(urb);

    return count;

error:
    usb_buffer_free(fx2dev->udev, count, buf, urb->transfer_dma);
    usb_free_urb(urb);
    return retval;
}

/*****************************************************************************/
/* Polling-Zugriff um abzufragen, ob gelesen bzw. geschrieben werden kann    */
/*****************************************************************************/
static unsigned int osrfx2_poll(struct file * file, poll_table * wait)
{
    struct osrfx2 * fx2dev = (struct osrfx2 *)file->private_data;
    unsigned int mask = 0;
    int retval = 0;
    
    // Der Semaphore wird aktiviert
    retval = down_interruptible( &fx2dev->sem );
    
    poll_wait(file, &fx2dev->FieldEventQueue, wait);

    if ( fx2dev->notify == TRUE ) {
        fx2dev->notify = FALSE;
        mask |= POLLPRI;  //High-Priority. Daten können ohne Blockieren gelesen werden.
    }

    if ( fx2dev->pending_data > 0) {
        mask |= POLLIN | POLLRDNORM; // Es kann gelesen werden
    }
    
    // Der Semaphore wird wieder freigegeben
    up( &fx2dev->sem );
    
    return mask;
}

/*****************************************************************************/
/* Die File-Operationen, die von diesem Treiber unterstützt werden. 		 */
/*****************************************************************************/
static struct file_operations osrfx2_file_ops = {
    .owner   = THIS_MODULE,
    .open    = osrfx2_open,
    .release = osrfx2_release,
    .read    = osrfx2_read,
    .write   = osrfx2_write,
    .poll    = osrfx2_poll,
};
 
/*****************************************************************************/
/* Klasse zur identifikation des USB Treibers. Es werden die unterstützten	 */
/* File-Operationen, der Geräte-Name und die Minor-Nummer vom USB core 		 */
/* angegeben.																 */
/*****************************************************************************/
static struct usb_class_driver osrfx2_class = {
    .name       = "device/osrfx2_%d",
    .fops       = &osrfx2_file_ops,
    .minor_base = DEVICE_MINOR_BASE,
};

/*****************************************************************************/
/* Event: Prüfung ob das Gerät bestimmte Funktionen unterstüzt und von		 */
/* diesem Treiber bedient werden kann.										 */
/*****************************************************************************/
static int osrfx2_probe(struct usb_interface * interface, 
                        const struct usb_device_id * id)
{
    struct usb_device * udev = interface_to_usbdev(interface);
    struct osrfx2 * fx2dev = NULL;
    int retval;

    fx2dev = kmalloc(sizeof(struct osrfx2), GFP_KERNEL);
    if (fx2dev == NULL) {
        retval = -ENOMEM;
        goto error;
    }
    memset(fx2dev, 0, sizeof(*fx2dev));
    kref_init( &fx2dev->kref );

    fx2dev->udev = usb_get_dev(udev);
    fx2dev->interface = interface;
    fx2dev->suspended = FALSE;

    fx2dev->bulk_write_available = (atomic_t) ATOMIC_INIT(1);
    fx2dev->bulk_read_available  = (atomic_t) ATOMIC_INIT(1);

    usb_set_intfdata(interface, fx2dev);

    retval = device_create_file(&interface->dev, &dev_attr_switches);
    if (retval != 0) 
        goto error;
	
    retval = device_create_file(&interface->dev, &dev_attr_bargraph);
    if (retval != 0) 
        goto error;

    retval = find_endpoints( fx2dev );
    if (retval != 0) 
        goto error;
    
    retval = init_interrupts( fx2dev );
    if (retval != 0)
        goto error;

    retval = init_bulks( fx2dev );
    if (retval != 0)
        goto error;

    retval = usb_register_dev(interface, &osrfx2_class);
    if (retval != 0) {
        usb_set_intfdata(interface, NULL);
    }

    dev_info(&interface->dev, "OSR USB-FX2 device now attached.\n");

    return 0;

error:
    dev_err(&interface->dev, "OSR USB-FX2 device probe failed: %d.\n", retval);
    if (fx2dev) {
        kref_put( &fx2dev->kref, osrfx2_delete );
    }
    return retval;
}

/*****************************************************************************/
/* Event-Handler: Das Gerät wurde entfernt.									 */
/*****************************************************************************/
static void osrfx2_disconnect(struct usb_interface * interface)
{
    struct osrfx2 * fx2dev;

    lock_kernel();

    fx2dev = usb_get_intfdata(interface);

    // Der Interrupt-Lese URB wird gestoppt
    usb_kill_urb(fx2dev->int_in_urb);
    
    usb_set_intfdata(interface, NULL);

    device_remove_file(&interface->dev, &dev_attr_switches);
    device_remove_file(&interface->dev, &dev_attr_bargraph);

    usb_deregister_dev(interface, &osrfx2_class);

    unlock_kernel();

    kref_put( &fx2dev->kref, osrfx2_delete );

    dev_info(&interface->dev, "OSR USB-FX2 now disconnected.\n");
}

/*****************************************************************************/
/* Event-Handler: Das Gerät hat sich in den Ruhe-Modus geschalten.           */
/*****************************************************************************/
static int osrfx2_suspend(struct usb_interface * intf, pm_message_t message)
{
    struct osrfx2 * fx2dev = usb_get_intfdata(intf);

    dev_info(&intf->dev, "%s - entry\n", __FUNCTION__);

    if (down_interruptible(&fx2dev->sem)) {
        return -ERESTARTSYS;
    }

    fx2dev->suspended = TRUE;

    /* 
     *  Der Interrupt-Lese URB wird gestoppt.
     */
    usb_kill_urb(fx2dev->int_in_urb);

    up(&fx2dev->sem);

    return 0;
}

/*****************************************************************************/
/* Event-Handler: Das Gerät ist aus dem Ruhe-Modus zurück gekehrt.           */
/*****************************************************************************/
static int osrfx2_resume(struct usb_interface * intf)
{
    int retval;
    struct osrfx2 * fx2dev = usb_get_intfdata(intf);

    dev_info(&intf->dev, "%s - entry\n", __FUNCTION__);

    // Setzen des Semaphore
    if (down_interruptible(&fx2dev->sem)) {
        return -ERESTARTSYS;
    }
    
    fx2dev->suspended = FALSE;

    /* 
     *  Der Interrupt-Lese URB wird neu initialisiert
     */
    retval = usb_submit_urb( fx2dev->int_in_urb, GFP_KERNEL );
    
    if (retval) {
        dev_err(&intf->dev, "%s - usb_submit_urb failed %d\n",
                __FUNCTION__, retval);

        switch (retval) {
        case -EHOSTUNREACH:
            dev_err(&intf->dev, "%s - EHOSTUNREACH probably cause: "
                    "parent hub/port still suspended.\n", 
                    __FUNCTION__);
            break;

        default:
            break;

        }
    }
    
    // Freilassen des Semaphores
    up(&fx2dev->sem);

    return 0;
}

/*****************************************************************************/
/* Die Treiber-Struktur des USB Treibers mit den Referenzen auf				 */
/* die entsprechenden methoden. Dieses Struct wird zum initialisieren		 */
/* und deinitialisieren benötigt.											 */
/*****************************************************************************/
static struct usb_driver osrfx2_driver = {
    .name        = "osrfx2",
    .probe       = osrfx2_probe, //enter
    .disconnect  = osrfx2_disconnect,
    .suspend     = osrfx2_suspend,
    .resume      = osrfx2_resume,
    .id_table    = id_table,
};

/*****************************************************************************/
/* Die init methode registriert den Treiber beim USB subsystem  			 */
/*****************************************************************************/
static int __init osrfx2_init(void)
{
    int retval;

    retval = usb_register(&osrfx2_driver);

    return retval;
}

/*****************************************************************************/
/* Die exit methode deregistriert den Treiber vom USB subsystem 			 */
/*****************************************************************************/
static void __exit osrfx2_exit(void)
{
    usb_deregister( &osrfx2_driver );
}

/*****************************************************************************/
/* Die Init und exit methoden des Treibers müssen dem System bekannt gemacht */
/* werden.		                         									 */
/*****************************************************************************/
module_init( osrfx2_init );
module_exit( osrfx2_exit );

MODULE_AUTHOR("Robin Callender");
MODULE_DESCRIPTION("A driver for the OSR USB-FX2 Learning Kit device");
MODULE_LICENSE("GPL");

