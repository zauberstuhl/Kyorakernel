/*
 * drivers/input/touchscreen/adc_ts.c
 *
 * Using code from:
 *  - tsc2007.c
 *	Copyright (c) 2008 MtekVision Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/saradc.h>
#include <linux/adc_ts.h>

#define TS_POLL_DELAY		1 /* ms delay between samples */
#define TS_POLL_PERIOD		5 /* ms delay between samples */

#define	MAX_10BIT			((1 << 10) - 1)

#define READ_X	1
#define READ_Y	2
#define READ_Z1	3
#define READ_Z2	4

struct ts_event {
	u16	x;
	u16	y;
	u16	z1;
	u16	z2;
};

struct adcts {
	struct input_dev *input;
	char phys[32];
	struct delayed_work work;
	struct ts_event event;
	struct ts_event event_cache;
	bool pendown;
	int seq;
	
	u16 x_plate_ohms;
	int irq;
	int (*service)(int cmd);
	int poll_delay;
	int poll_period;
	int (*convert)(int x, int y);
};

#define adcts_cache_out(ts, ev) do { \
    ev = ts->event_cache; \
} while(0)

#define adcts_cache_in(ts, ev) do { \
    ts->event_cache = ev; \
} while(0)

#define adcts_clear_cache(ts) do { \
    memset(&ts->event_cache, 0 , sizeof(struct ts_event)); \
} while(0)

static u32 adcts_calculate_pressure(struct adcts *ts, struct ts_event *tc)
{
	u32 rt = 0;

	/* range filtering */
	if (tc->x == MAX_10BIT)
		tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		/* compute touch pressure resistance using equation #1 */
		rt = tc->z2 - tc->z1;
		rt *= tc->x;
		rt *= ts->x_plate_ohms;
		rt /= tc->z1;
		rt = (rt + 2047) >> 12;
	}

	return rt;
}

static void adcts_send_up_event(struct adcts *ts)
{
	struct input_dev *input = ts->input;

	dev_dbg(&ts->input->dev, "UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}


static void adcts_work(struct work_struct *work)
{
	struct adcts *ts =
		container_of(to_delayed_work(work), struct adcts, work);
	struct input_dev *input = ts->input;
	u32 rt;

	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */

	
	if (ts->seq == 0) { 
		if (ts->service(CMD_GET_PENDOWN)) {
			if (!ts->pendown) {
				ts->pendown = 1;
				input_report_key(input, BTN_TOUCH, 1);
				printk(KERN_INFO "DOWN\n");
			}
			ts->seq ++;
			ts->service(CMD_CLEAR_PENIRQ);
		}
		else  if (ts->pendown) {
			ts->pendown = 0;
			adcts_send_up_event(ts);
			adcts_clear_cache(ts);
			printk(KERN_INFO "UP\n");
		}
	}
	
	else if (ts->seq == 1) { 
		ts->event.x = ts->service(CMD_GET_X);
		ts->seq ++;
	}

	else if (ts->seq == 2) { 
		ts->event.y = ts->service(CMD_GET_Y);
		struct ts_event event;
		adcts_cache_out(ts, event);		
		adcts_cache_in(ts, ts->event);
		ts->event = event;
		if (ts->event.x || ts->event.y) {
			if (ts->convert) {
				int xy = ts->convert(ts->event.x, ts->event.y);
				ts->event.x = xy >> 16;
				ts->event.y = xy & 0xffff;
			}
			input_report_abs(input, ABS_X, ts->event.x);
            		input_report_abs(input, ABS_Y, ts->event.y);
            		rt = 500;	//debug
            		input_report_abs(input, ABS_PRESSURE, rt);
            		input_sync(input);
            		printk(KERN_INFO "x=%d, y=%d\n", ts->event.x, ts->event.y);
                  }
		ts->seq = 0;
		ts->service(CMD_SET_PENIRQ);
	}

	if (ts->pendown || (ts->irq < 0)) {
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(ts->poll_period));
	}
	else {
		ts->service(CMD_SET_PENIRQ);
		enable_irq(ts->irq);
		adcts_clear_cache(ts);
		printk(KERN_INFO "exit adc irq\n");
	}
}


static irqreturn_t adcts_irq(int irq, void *handle)
{
	struct adcts *ts = handle;
	printk(KERN_INFO "enter adc irq\n");

	if (ts->service(CMD_GET_PENDOWN)) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(ts->poll_delay));
	}
	
	return IRQ_HANDLED;
}

static void adcts_free_irq(struct adcts *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int __devinit adcts_probe(struct platform_device *pdev)
{
	struct adcts *ts;
	struct adc_ts_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata) {
		dev_err(&pdev->dev, "platform data is required!\n");
		return -EINVAL;
	}

	ts = kzalloc(sizeof(struct adcts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}
	ts->input = input_dev;

	platform_set_drvdata(pdev, ts);
	INIT_DELAYED_WORK(&ts->work, adcts_work);

	ts->x_plate_ohms = pdata->x_plate_ohms;
	ts->service = saradc_ts_service;
	ts->poll_delay = pdata->poll_delay ? pdata->poll_delay : TS_POLL_DELAY;
	ts->poll_period = pdata->poll_period ? pdata->poll_period : TS_POLL_PERIOD;
	ts->convert = pdata->convert;
	adcts_clear_cache(ts);

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&pdev->dev));

	input_dev->name = "adcts Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	int max = pdata->abs_xmax ? pdata->abs_xmax : MAX_10BIT;
	input_set_abs_params(input_dev, ABS_X, pdata->abs_xmin, max, 0, 0);
	max = pdata->abs_ymax ? pdata->abs_ymax : MAX_10BIT;
	input_set_abs_params(input_dev, ABS_Y, pdata->abs_ymin, max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_10BIT, 0, 0);

	ts->seq = 0;
	ts->pendown = 0;
	ts->irq = pdata->irq;
	ts->service(CMD_INIT_PENIRQ);
	if (ts->irq < 0) {
		schedule_delayed_work(&ts->work,
			      msecs_to_jiffies(ts->poll_delay));
	}
	else {
		err = request_irq(ts->irq, adcts_irq, 0, pdev->dev.driver->name, ts);
		if (err < 0) {
			dev_err(&pdev->dev, "irq %d busy?\n", ts->irq);
			goto err_free_mem;
		}
	}

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	return 0;

 err_free_irq:
	adcts_free_irq(ts);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit adcts_remove(struct platform_device *pdev)
{
	struct adcts *ts = platform_get_drvdata(pdev);
	adcts_free_irq(ts);
	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}


static struct platform_driver adcts_driver = {
	.probe      = adcts_probe,
	.remove     = adcts_remove,
	.suspend    = NULL,
	.resume     = NULL,
	.driver     = {
		.name   = "adc_ts",
	},
};

static int __devinit adcts_init(void)
{
	printk(KERN_INFO "ADC Touchscreen Driver init.\n");
	return platform_driver_register(&adcts_driver);
}

static void __exit adcts_exit(void)
{
	printk(KERN_INFO "ADC Touchscreen  Driver exit.\n");
	platform_driver_unregister(&adcts_driver);
}

module_init(adcts_init);
module_exit(adcts_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("ADC TouchScreen Driver");
MODULE_LICENSE("GPL");