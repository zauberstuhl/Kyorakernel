#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <linux/gpio.h>
#include <mach/irqs.h>
#include <mach/wifi_tiwlan.h>

static int amlogic_wifi_cd;	/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb) (int card_present, void *dev_id);
static void *wifi_status_cb_devid;

int amlogic_wifi_status_register(void (*callback) (int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;

	wifi_status_cb = callback;

	wifi_status_cb_devid = dev_id;
	return 0;
}

int amlogic_wifi_status(int irq)
{
	return amlogic_wifi_cd;
}

int amlogic_wifi_set_carddetect(int val)
{
	amlogic_wifi_cd = val;

	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk("%s: Nobody to notify\n", __func__);

	return 0;
}

static int amlogic_wifi_power_state;

//extern extern_wifi_power(on);
//#include <linux/sn7325.h>
int amlogic_wifi_power(int on)
{
	#if 0
	if(on)
	{
		printk("##########Power on WiFi!###########\n");
    	//set_gpio_val(GPIOD_bank_bit2_24(15), GPIOD_bit_bit2_24(15), 0);
		//set_gpio_mode(GPIOD_bank_bit2_24(15), GPIOD_bit_bit2_24(15), GPIO_OUTPUT_MODE);
		msleep(80);
        configIO(0, 0);
        setIO_level(0, 1, 5);
        setIO_level(0, 1, 7);
	}
	else
	{
		printk("##########Cut off wifi power!###########\n");
        configIO(0, 0);
        setIO_level(0, 0, 5);
        setIO_level(0, 0, 7);
	}
	#endif
	//extern_wifi_power(on);
	//gpio_set_value(PMENA_GPIO, on);

	amlogic_wifi_power_state = on;
	return 0;
}

static int amlogic_wifi_reset_state;
int amlogic_wifi_reset(int on)
{
	amlogic_wifi_reset_state = on;
	return 0;
}

struct wifi_platform_data amlogic_wifi_control = {
	.set_power = amlogic_wifi_power,
	.set_reset = amlogic_wifi_reset,
	.set_carddetect = amlogic_wifi_set_carddetect,
};

static struct resource amlogic_wifi_resources[] = {
	[0] = {
	       .name = "device_wifi_irq",
	       .start = INT_GPIO_2,
	       .end = INT_GPIO_2,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	       },
};

static struct platform_device amlogic_wifi_device = {
	.name = "device_wifi",
	.id = 1,
	.num_resources = ARRAY_SIZE(amlogic_wifi_resources),
	.resource = amlogic_wifi_resources,
	.dev = {
		.platform_data = &amlogic_wifi_control,
	},
};

int amlogic_wifi_init(void)
{
	int ret;
	ret = platform_device_register(&amlogic_wifi_device);
	return ret;
}

device_initcall(amlogic_wifi_init);
