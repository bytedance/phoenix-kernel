// SPDX-License-Identifier: GPL-2.0+
/*
 * hid.c -- HID Composite driver
 *
 * Based on multi.c
 *
 * Copyright (C) 2010 Fabien Chouteau <fabien.chouteau@barco.com>
 */


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/usb/composite.h>
#include <linux/usb/g_hid.h>
/*Added by PICO_Driver jim.qiu for bug#58022: enable hid on second usb --start--*/
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#pragma GCC diagnostic ignored "-Wunused-variable"   // jim.qiu add for debug
/*Added by PICO_Driver jim.qiu for bug#58022: enable hid on second usb --end--*/

/*Modify by PICO_Driver sophia.wang for bug59744: test for hidraw node_1 start*/

#define DRIVER_DESC		"Neo3"
#define DRIVER_VERSION		"2010/03/16"

#include "u_hid.h"

/*-------------------------------------------------------------------------*/

#define HIDG_VENDOR_NUM		0x0525	/* XXX NetChip */
#define HIDG_PRODUCT_NUM	0xa4ac	/* Linux-USB HID gadget */

static char manufacturer_pico[] = "Pico";
static const char product_pico[] = DRIVER_DESC;
/*Modify by PICO_Driver sophia.wang for bug59744: test for hidraw node_1 end*/

/*-------------------------------------------------------------------------*/

struct hidg_func_node {
	struct usb_function_instance *fi;
	struct usb_function *f;
	struct list_head node;
	struct hidg_func_descriptor *func;
};

static LIST_HEAD(hidg_func_list);

/*-------------------------------------------------------------------------*/
USB_GADGET_COMPOSITE_OPTIONS();

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	/* .bcdUSB = DYNAMIC */

	/* .bDeviceClass =		USB_CLASS_COMM, */
	/* .bDeviceSubClass =	0, */
	/* .bDeviceProtocol =	0, */
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */

	/* Vendor and product id can be overridden by module parameters.  */
	.idVendor =		cpu_to_le16(HIDG_VENDOR_NUM),
	.idProduct =		cpu_to_le16(HIDG_PRODUCT_NUM),
	/* .bcdDevice = f(hardware) */
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	/* NO SERIAL NUMBER */
	.bNumConfigurations =	1,
};

static const struct usb_descriptor_header *otg_desc[2];

/* string IDs are assigned dynamically */
static struct usb_string strings_dev[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = manufacturer_pico,
	[USB_GADGET_PRODUCT_IDX].s = product_pico,
	[USB_GADGET_SERIAL_IDX].s = "",
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};
/*Modify by PICO_Driver sophia.wang for bug59744: test for hidraw node_1 start*/
static struct hidg_func_descriptor ghid_data1 = {
	.subclass = 0,
	.protocol = 0,
	.report_length = 64,
	.report_desc_length = 29,
	.report_desc = {
		0x05, 0x01,//usage page (generic desktop)
	    0x09, 0x00,//usage 06 keypord
	    0xa1, 0x01,//collection 01--Application

	    0x85, 0xaa,//report id

	    0x15, 0x00,//logical min---mouse lift can be 0 1
	    0x25, 0xff,//logcal max
	    0x19, 0x01,//usage min
	    0x29, 0x08,//usage max ---mouse 3  lift middle right
	    0x95, 0x3F,//report count 149 149*1 that means the input should be 149byte,if it has a report Id ,the input = 1byte (report id)+149*1byte
	    0x75, 0x08,//report size 8  =1byte
	    0x81, 0x02,//input
	    0x19, 0x01,
	    0x29, 0x08,
	    0x91, 0x02,//output
	    0xc0,
	}
};
/*Modify by PICO_Driver sophia.wang for bug59744: test for hidraw node_1 end*/

/*Added by PICO_Driver jim.qiu for bug#58022: enable hid on second usb --start--*/
static struct hidg_func_descriptor hid_default_data = {
    .subclass        = 0, /* No subclass */
    .protocol        = 3, /* NONE */
    .report_length        = 8,
    .report_desc_length    = 63,
    .report_desc        = {
        0x05, 0x03,    /* USAGE_PAGE (VR)     */
        0x09, 0x02,    /* USAGE (mouse) */
        0xa1, 0x01,    /* COLLECTION (Application) */
        0x05, 0x07,    /* USAGE_PAGE (Keyboard) */
        0x19, 0xe0,    /* USAGE_MINIMUM (Keyboard LeftControl) */
        0x29, 0xe7,    /* USAGE_MAXIMUM (Keyboard Right GUI) */
        0x15, 0x00,    /* LOGICAL_MINIMUM (0) */
        0x25, 0x01,    /* LOGICAL_MAXIMUM (1) */
        0x75, 0x01,    /* REPORT_SIZE (1) */
        0x95, 0x08,    /* REPORT_COUNT (8) */
        0x81, 0x02,    /* INPUT (Data,Var,Abs) */
        0x95, 0x01,    /* REPORT_COUNT (1) */
        0x75, 0x08,    /* REPORT_SIZE (8) */
        0x81, 0x03,    /* INPUT (Cnst,Var,Abs) */
        0x95, 0x05,    /* REPORT_COUNT (5) */
        0x75, 0x01,    /* REPORT_SIZE (1) */
        0x05, 0x08,    /* USAGE_PAGE (LEDs) */
        0x19, 0x01,    /* USAGE_MINIMUM (Num Lock) */
        0x29, 0x05,    /* USAGE_MAXIMUM (Kana) */
        0x91, 0x02,    /* OUTPUT (Data,Var,Abs) */
        0x95, 0x01,    /* REPORT_COUNT (1) */
        0x75, 0x03,    /* REPORT_SIZE (3) */
        0x91, 0x03,    /* OUTPUT (Cnst,Var,Abs) */
        0x95, 0x06,    /* REPORT_COUNT (6) */
        0x75, 0x08,    /* REPORT_SIZE (8) */
        0x15, 0x00,    /* LOGICAL_MINIMUM (0) */
        0x25, 0x65,    /* LOGICAL_MAXIMUM (101) */
        0x05, 0x07,    /* USAGE_PAGE (Keyboard) */
        0x19, 0x00,    /* USAGE_MINIMUM (Reserved) */
        0x29, 0x65,    /* USAGE_MAXIMUM (Keyboard Application) */
        0x81, 0x00,    /* INPUT (Data,Ary,Abs) */
        0xc0        /* END_COLLECTION */
    }
};


static struct hidg_func_descriptor hid_keyboard_data = {
    .subclass        = 0, /* No subclass */
    .protocol        = 1, /* Keyboard */
    .report_length        = 8,
    .report_desc_length    = 63,
    .report_desc        = {
        0x05, 0x01,    /* USAGE_PAGE (Generic Desktop)     */
        0x09, 0x06,    /* USAGE (Keyboard) */
        0xa1, 0x01,    /* COLLECTION (Application) */
        0x05, 0x07,    /* USAGE_PAGE (Keyboard) */
        0x19, 0xe0,    /* USAGE_MINIMUM (Keyboard LeftControl) */
        0x29, 0xe7,    /* USAGE_MAXIMUM (Keyboard Right GUI) */
        0x15, 0x00,    /* LOGICAL_MINIMUM (0) */
        0x25, 0x01,    /* LOGICAL_MAXIMUM (1) */
        0x75, 0x01,    /* REPORT_SIZE (1) */
        0x95, 0x08,    /* REPORT_COUNT (8) */
        0x81, 0x02,    /* INPUT (Data,Var,Abs) */
        0x95, 0x01,    /* REPORT_COUNT (1) */
        0x75, 0x08,    /* REPORT_SIZE (8) */
        0x81, 0x03,    /* INPUT (Cnst,Var,Abs) */
        0x95, 0x05,    /* REPORT_COUNT (5) */
        0x75, 0x01,    /* REPORT_SIZE (1) */
        0x05, 0x08,    /* USAGE_PAGE (LEDs) */
        0x19, 0x01,    /* USAGE_MINIMUM (Num Lock) */
        0x29, 0x05,    /* USAGE_MAXIMUM (Kana) */
        0x91, 0x02,    /* OUTPUT (Data,Var,Abs) */
        0x95, 0x01,    /* REPORT_COUNT (1) */
        0x75, 0x03,    /* REPORT_SIZE (3) */
        0x91, 0x03,    /* OUTPUT (Cnst,Var,Abs) */
        0x95, 0x06,    /* REPORT_COUNT (6) */
        0x75, 0x08,    /* REPORT_SIZE (8) */
        0x15, 0x00,    /* LOGICAL_MINIMUM (0) */
        0x25, 0x65,    /* LOGICAL_MAXIMUM (101) */
        0x05, 0x07,    /* USAGE_PAGE (Keyboard) */
        0x19, 0x00,    /* USAGE_MINIMUM (Reserved) */
        0x29, 0x65,    /* USAGE_MAXIMUM (Keyboard Application) */
        0x81, 0x00,    /* INPUT (Data,Ary,Abs) */
        0xc0        /* END_COLLECTION */
    }
};

static struct hidg_func_descriptor hid_mouse_data = {
    .subclass = 0,    /*NO SubClass*/
    .protocol = 2,    /*Mouse*/
    .report_length = 4,
    .report_desc_length = 52,
    .report_desc={
        0x05,0x01,    /*Usage Page (Generic Desktop Controls)*/
        0x09,0x02,    /*Usage (Mouse)*/
        0xa1,0x01,    /*Collction (Application)*/
        0x09,0x01,    /*Usage (pointer)*/
        0xa1,0x00,    /*Collction (Physical)*/
        0x05,0x09,    /*Usage Page (Button)*/
        0x19,0x01,    /*Usage Minimum(1)*/
        0x29,0x03,    /*Usage Maximum(3) */ 
        0x15,0x00,    /*Logical Minimum(1)*/
        0x25,0x01,    /*Logical Maximum(1)*/
        0x95,0x03,    /*Report Count(5)  */
        0x75,0x01,    /*Report Size(1)*/
        0x81,0x02,    /*Input(Data,Variable,Absolute,BitFiled)*/
        0x95,0x01,    /*Report Count(1)*/
        0x75,0x05,    /*Report Size(5) */
        0x81,0x01,    /*Input(Constant,Array,Absolute,BitFiled) */
        0x05,0x01,    /*Usage Page (Generic Desktop Controls)*/
        0x09,0x30,    /*Usage(x)*/
        0x09,0x31,    /*Usage(y)*/
        0x09,0x38,    /*Usage(Wheel)*/
        0x15,0x81,    /*Logical Minimum(-127)*/
        0x25,0x7f,    /*Logical Maximum(127)*/
        0x75,0x08,    /*Report Size(8)*/
        0x95,0x02,    /*Report Count(2)  */
        0x81,0x06,    /*Input(Data,Variable,Relative,BitFiled)*/
        0xc0,    /*End Collection*/
        0xc0    /*End Collection*/
    }
};

/*Added by PICO_Driver jim.qiu for bug#58022: enable hid on second usb --end--*/

/****************************** Configurations ******************************/

static int do_config(struct usb_configuration *c)
{
	struct hidg_func_node *e, *n;
	int status = 0;

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	list_for_each_entry(e, &hidg_func_list, node) {
		e->f = usb_get_function(e->fi);
		if (IS_ERR(e->f))
			goto put;
		status = usb_add_function(c, e->f);
		if (status < 0) {
			usb_put_function(e->f);
			goto put;
		}
	}

	return 0;
put:
	list_for_each_entry(n, &hidg_func_list, node) {
		if (n == e)
			break;
		usb_remove_function(c, n->f);
		usb_put_function(n->f);
	}
	return status;
}

static struct usb_configuration config_driver = {
	.label			= "PICOVR HID Gadget",
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

/****************************** Gadget Bind ******************************/

static int hid_bind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	struct list_head *tmp;
	struct hidg_func_node *n, *m;
	struct f_hid_opts *hid_opts;
	int status, funcs = 0;

	list_for_each(tmp, &hidg_func_list)
		funcs++;

	if (!funcs)
		return -ENODEV;

	list_for_each_entry(n, &hidg_func_list, node) {
		n->fi = usb_get_function_instance("hid");
		if (IS_ERR(n->fi)) {
			status = PTR_ERR(n->fi);
			goto put;
		}
		hid_opts = container_of(n->fi, struct f_hid_opts, func_inst);
		hid_opts->subclass = n->func->subclass;
		hid_opts->protocol = n->func->protocol;
		hid_opts->report_length = n->func->report_length;
		hid_opts->report_desc_length = n->func->report_desc_length;
		hid_opts->report_desc = n->func->report_desc;
	}


	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */

	status = usb_string_ids_tab(cdev, strings_dev);
	if (status < 0)
		goto put;
	device_desc.iManufacturer = strings_dev[USB_GADGET_MANUFACTURER_IDX].id;
	device_desc.iProduct = strings_dev[USB_GADGET_PRODUCT_IDX].id;

	if (gadget_is_otg(gadget) && !otg_desc[0]) {
		struct usb_descriptor_header *usb_desc;

		usb_desc = usb_otg_descriptor_alloc(gadget);
		if (!usb_desc)
			goto put;
		usb_otg_descriptor_init(gadget, usb_desc);
		otg_desc[0] = usb_desc;
		otg_desc[1] = NULL;
	}

	/* register our configuration */
	status = usb_add_config(cdev, &config_driver, do_config);
	if (status < 0)
		goto free_otg_desc;

	usb_composite_overwrite_options(cdev, &coverwrite);
	dev_info(&gadget->dev, DRIVER_DESC ", version: " DRIVER_VERSION "\n");

	return 0;

free_otg_desc:
	kfree(otg_desc[0]);
	otg_desc[0] = NULL;
put:
	list_for_each_entry(m, &hidg_func_list, node) {
		if (m == n)
			break;
		usb_put_function_instance(m->fi);
	}
	return status;
}

static int hid_unbind(struct usb_composite_dev *cdev)
{
	struct hidg_func_node *n;

	list_for_each_entry(n, &hidg_func_list, node) {
		usb_put_function(n->f);
		usb_put_function_instance(n->fi);
	}

	kfree(otg_desc[0]);
	otg_desc[0] = NULL;

	return 0;
}

static int hidg_plat_driver_probe(struct platform_device *pdev)
{
	//struct hidg_func_descriptor *func = dev_get_platdata(&pdev->dev);
	struct device_node *node = pdev->dev.of_node;
	//struct hidg_func_descriptor *func = &hid_keyboard_data;
	struct hidg_func_node *entry;
	u8 platformdev_id = 0xf0;
	u32 loopcnt,array_size = 0;
	struct hidg_func_descriptor *func = NULL;

	printk(KERN_ERR"%s line%d\n",__func__,__LINE__);

	if(!pdev)
		return -ENODEV;

	if(!func)
		func = &ghid_data1;

	#if 0
	func = dev_get_platdata(&pdev->dev);
	if (!func) {
		dev_err(&pdev->dev, "Platform data missing\n");
		return -ENODEV;
	}
	#endif
	#if 0///*Modify by PICO_Driver sophia.wang for bug59744: test for hidraw node_1 start*/
	if(!of_property_read_u8(node,"plat_id",&platformdev_id))
	{
		dev_info(&pdev->dev, "Platform data platformdev_id=%d\n",platformdev_id);
	}
	else
	{
		dev_err(&pdev->dev, "Platform device id missing\n");
	}


	if(!of_property_read_u8(node,"subclass",&func->subclass))
	{
		dev_info(&pdev->dev, "Platform data subclass=%d\n",func->subclass);
	}
	else
	{
		dev_err(&pdev->dev, "Platform device subclass missing\n");
	}	

	if(!of_property_read_u8(node,"protocol",&func->protocol))
	{
		dev_info(&pdev->dev, "Platform data protocol=%d\n",func->protocol);
	}
	else
	{
		dev_err(&pdev->dev, "Platform device protocol missing\n");
	}	

	if(!of_property_read_u16(node,"report_length",&func->report_length))
	{
		dev_info(&pdev->dev, "Platform data report_length=%d\n",func->report_length);
	}
	else
	{
		dev_err(&pdev->dev, "Platform device report_length missing\n");
	}

	if(!of_property_read_u16(node,"report_desc_length",&func->report_desc_length))
	{
		dev_info(&pdev->dev, "Platform data report_desc_length=%d\n",func->report_desc_length);
	}
	else
	{
		dev_err(&pdev->dev, "Platform device report_desc_length missing\n");
	}	

	of_get_property(node, "report_desc", &array_size);
	dev_info(&pdev->dev, "Platform data report_desc array_size=%u\n",array_size);
	if (array_size) {
		of_property_read_u8_array(node,
			"report_desc", func->report_desc,
			array_size);
	} else {
		dev_err(&pdev->dev, "err provide report_desc\n");
		return -EINVAL;
	}

	for(loopcnt=0;loopcnt < array_size;loopcnt++)
	{	
		printk("report_desc[%u]=0x%02x",loopcnt,func->report_desc[loopcnt]);
		//if(loopcnt > 0 && loopcnt%2){
		//	printk("\n");
		//}
	}
	#endif /*Modify by PICO_Driver sophia.wang for bug59744: test for hidraw node_1 end*/
	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->func = func;
	list_add_tail(&entry->node, &hidg_func_list);

	printk(KERN_ERR"%s line%d\n",__func__,__LINE__);

	return 0;
}

static int hidg_plat_driver_remove(struct platform_device *pdev)
{
	struct hidg_func_node *e, *n;

	list_for_each_entry_safe(e, n, &hidg_func_list, node) {
		list_del(&e->node);
		kfree(e);
	}

	return 0;
}


/****************************** Some noise ******************************/

static struct usb_composite_driver hidg_driver = {
	.name		= "g_hid",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.max_speed	= USB_SPEED_HIGH,
	.bind		= hid_bind,
	.unbind		= hid_unbind,
};

/*Added by PICO_Driver jim.qiu for bug#58022: enable hid on second usb --start--*/
/*Modify by PICO_Driver sophia.wang for bug59744: test for hidraw node_1 start*/

static const struct of_device_id of_hid_matach[] = {	
	{
		.compatible = "hid_mouse_slave",
	},	
	{ },
};
MODULE_DEVICE_TABLE(of, of_hid_matach);
/*Modify by PICO_Driver sophia.wang for bug59744: test for hidraw node_1 end*/

/*Added by PICO_Driver jim.qiu for bug#58022: enable hid on second usb --end--*/

static struct platform_driver hidg_plat_driver = {
	//.probe		= hidg_plat_driver_probe,
	.remove		= hidg_plat_driver_remove,
	.driver		= {
		.name	= "hidg",
		/*Added by PICO_Driver jim.qiu for bug#58022: enable hid on second usb --start--*/
		.of_match_table	= of_hid_matach,
		/*Added by PICO_Driver jim.qiu for bug#58022: enable hid on second usb --end--*/
	},
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Fabien Chouteau, Peter Korsgaard");
MODULE_LICENSE("GPL");

static int __init hidg_init(void)
{
	int status;

	status = platform_driver_probe(&hidg_plat_driver,
				hidg_plat_driver_probe);
	//status = platform_driver_register(&hidg_plat_driver);
	printk(KERN_ERR"%s line%d,status=%d\n",__func__,__LINE__,status);
	if (status < 0)
		return status;

	//printk(KERN_ERR"%s line%d,status=%d\n",__func__,__LINE__,status);

	status = usb_composite_probe(&hidg_driver);
	if (status < 0)
		platform_driver_unregister(&hidg_plat_driver);

	printk(KERN_ERR"%s line%d,status=%d\n",__func__,__LINE__,status);

	return status;
}
module_init(hidg_init);

static void __exit hidg_cleanup(void)
{
	usb_composite_unregister(&hidg_driver);
	platform_driver_unregister(&hidg_plat_driver);
}
module_exit(hidg_cleanup);
