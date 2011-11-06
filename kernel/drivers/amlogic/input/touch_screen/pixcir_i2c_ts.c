/* drivers/input/touchscreen/pixcir_i2c_ts.c
 *
 * Copyright (C) 2010 Pixcir, Inc.
 *
 * pixcir_i2c_ts.c V1.0  support multi touch
 * pixcir_i2c_ts.c V1.5  add Calibration function:
 *
 * CALIBRATION_FLAG	1
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/smp_lock.h>
#include <linux/delay.h>
#include <linux/slab.h>  //for mini6410 2.6.36 kree(),kmalloc()
#include <mach/gpio.h>

#include "pixcir_i2c_ts.h"

#define DRIVER_VERSION "v1.5"
#define DRIVER_AUTHOR "Bee<http://www.pixcir.com.cn>"
#define DRIVER_DESC "Pixcir I2C Touchscreen Driver with tune fuction"
#define DRIVER_LICENSE "GPL"

#define PIXCIR_DEBUG 0

#define TWO_POINTS

#define gpio_shutdown ((GPIOD_bank_bit2_24(23)<<16) |GPIOD_bit_bit2_24(23))
#define gpio_irq ((GPIOD_bank_bit2_24(24)<<16) |GPIOD_bit_bit2_24(24))
#define DBG_SIZE 100
int dbg_buf[DBG_SIZE][2];
int dbg_cnt=0;
/*********************************V2.0-Bee-0928-TOP****************************************/

#define SLAVE_ADDR		0x5c

#ifndef I2C_MAJOR
#define I2C_MAJOR 		125
#endif

#define I2C_MINORS 		256

#define  CALIBRATION_FLAG	1

static unsigned char status_reg = 0;

struct i2c_dev
{
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

static struct i2c_driver pixcir_i2c_ts_driver;
static struct class *i2c_dev_class;
static LIST_HEAD( i2c_dev_list);
static DEFINE_SPINLOCK( i2c_dev_list_lock);

/*static int i2cdev_check(struct device *dev, void *addrp)
 {
 struct i2c_client *client = i2c_verify_client(dev);

 if (!client || client->addr != *(unsigned int *)addrp)
 return 0;

 return dev->driver ? -EBUSY : 0;
 }

 static int i2cdev_check_addr(struct i2c_adapter *adapter,unsigned int addr)
 {
 return device_for_each_child(&adapter->dev,&addr,i2cdev_check);
 }*/

static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	i2c_dev = NULL;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list)
	{
		if (i2c_dev->adap->nr == index)
		goto found;
	}
	i2c_dev = NULL;
	found: spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS)
	{
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
				adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}
/*********************************V2.0-Bee-0928-BOTTOM****************************************/

static struct workqueue_struct *pixcir_wq;

struct pixcir_i2c_ts_data
{
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	int irq;
	int discard;
	int oldtouching;
};

static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len;
	msgs[1].buf=&buf[0];
	
	ret=i2c_transfer(client->adapter,msgs,2);
	return ret;
}

static void pixcir_ts_main(struct pixcir_i2c_ts_data *tsdata)
{
	unsigned char touching = 0;
	unsigned char oldtouching = 0;
	int posx1, posy1, posx2, posy2;
	unsigned char Rdbuf[10],auotpnum[1];
	int ret;
	int z = 50;
	int w = 15;

	memset(Rdbuf, 0, sizeof(Rdbuf));
	Rdbuf[0] = 0;
	ret = i2c_read_bytes(tsdata->client, Rdbuf, 10);
	if (ret != 2){
		dev_err(&tsdata->client->dev, "Unable to read i2c page!(%d)\n", ret);
		goto out;	
	}
	posy1 = ((Rdbuf[3] << 8) | Rdbuf[2]);
	posx1 = ((Rdbuf[5] << 8) | Rdbuf[4]);
	posy2 = ((Rdbuf[7] << 8) | Rdbuf[6]);
	posx2 = ((Rdbuf[9] << 8) | Rdbuf[8]);
	touching = Rdbuf[0];
	oldtouching = Rdbuf[1];
	//printk("touching:%-3d,oldtouching:%-3d,x1:%-6d,y1:%-6d,x2:%-6d,y2:%-6d\n",touching, oldtouching, posx1, posy1, posx2, posy2);

	if ((touching > 3)
	|| (posx1 < TOUCHSCREEN_MINX) || (posx1 > TOUCHSCREEN_MAXX)
	|| (posy1 < TOUCHSCREEN_MINY) || (posy1 > TOUCHSCREEN_MAXY)
	|| (posx2 < TOUCHSCREEN_MINX) || (posx2 > TOUCHSCREEN_MAXX)
	|| (posy2 < TOUCHSCREEN_MINY) || (posy2 > TOUCHSCREEN_MAXY)) {
		//invalid data,must discard the next twice data;
		tsdata->discard = 2;
	}
	else if (tsdata->discard) {
		tsdata->discard--;
	}
	else {
		if ((!(touching & 1) && (oldtouching & 1))
		|| ((touching & 1) && !(oldtouching & 1) && tsdata->oldtouching)) {
			input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, 0);
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, 0);
			input_mt_sync(tsdata->input);
//			printk("point 1 up\n");
		}
		if (!(touching & 2)&& (oldtouching & 2)) {
			input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, 1);
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, 0);
			input_mt_sync(tsdata->input);
//			printk("point 2 up\n");
		}
		if (touching & 1) {
			input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, 0);
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx1);
			input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy1);
			input_mt_sync(tsdata->input);
			dbg_buf[dbg_cnt][0] = posx1;
			dbg_buf[dbg_cnt][1] = posy1;
			if(++dbg_cnt == DBG_SIZE) dbg_cnt = DBG_SIZE - 1;		
//			printk("point 1 (%d, %d)\n", posx1, posy1);
		}
		if (touching & 2) {
			input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, 1);
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx2);
			input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy2);
			input_mt_sync(tsdata->input);
//			printk("point 2 (%d, %d)\n", posx2, posy2);
		}
		input_sync(tsdata->input);
		tsdata->oldtouching = touching;
	}

/*
	else {
		for (i=0; i<2; i++) {
			event[i].x = (Rdbuf[i*4+5] << 8) | Rdbuf[i*4+4];
			event[i].y = (Rdbuf[i*4+3] << 8) | Rdbuf[i*4+2];
			event[i].cur_sta = (Rdbuf[0] >> i) & 1;
			event[i].old_sta = (Rdbuf[1] >> i) & 1;
			
			if (event[i].cur_sta == UP) {
				if((event[i].old_sta == DOWN)||(event[i].last_sta == DOWN)) {
					input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, i);
					input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, 0);
					input_mt_sync(tsdata->input);
					ts_dbg("point_%d up\n", i);
				}
			}
			
			else if (event[i].cur_sta == DOWN) {
				if ((event[i].old_sta == UP)&&(event[i].last_sta == DOWN)) {
					input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, i);
					input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, 0);
					input_mt_sync(tsdata->input);
					ts_dbg("point_%d up\n", i);
				}
				input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
				input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
				input_report_abs(tsdata->input, ABS_MT_POSITION_X, event[i].x);
				input_report_abs(tsdata->input, ABS_MT_POSITION_Y, event[i].y);
				input_mt_sync(tsdata->input);
				ts_dbg("point_%d (%d, %d)\n", i, event[i].x, event[i].y);
			}
			event[i].last_sta = event[i].cur_sta;
		}	
	}
*/
	out:
		return;
}

static void pixcir_ts_poscheck(struct work_struct *work)
{
	struct pixcir_i2c_ts_data *tsdata = container_of(work,
			struct pixcir_i2c_ts_data, work.work);

	tsdata->oldtouching = 0;
	dbg_cnt = 0;
	while(!gpio_get_value(gpio_irq)){
		pixcir_ts_main(tsdata);
		//msleep(15);
		//mdelay(6);
		schedule();
	}
//	input_report_key(tsdata->input, BTN_TOUCH, 0);
//	input_report_abs(tsdata->input, ABS_PRESSURE, 0);
#ifdef TWO_POINTS
	if (tsdata->oldtouching & 1) {
		input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, 0);
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(tsdata->input);
//		printk("point 1 up\n");
	}
	if (tsdata->oldtouching & 2) {
		input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, 1);
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(tsdata->input);
//		printk("point 2 up\n");
	}
#endif
	if (tsdata->oldtouching) {
		input_sync(tsdata->input);
	}
//	printk("total = %d\n", dbg_cnt);
//	int i;
//	for(i=0;i<dbg_cnt;i++) {
//		printk("%4d(%4d, %4d)\n", i, dbg_buf[i][0], dbg_buf[i][1]);
//	}
//	dbg_cnt = 0;
	enable_irq(tsdata->irq);
}

static irqreturn_t pixcir_ts_isr(int irq, void *dev_id)
{
	struct pixcir_i2c_ts_data *tsdata = dev_id;

	disable_irq_nosync(irq);
//	printk("enter irq\n");
	queue_work(pixcir_wq, &tsdata->work.work);
	return IRQ_HANDLED;
}

static int pixcir_ts_open(struct input_dev *dev)
{
	return 0;
}

static void pixcir_ts_close(struct input_dev *dev)
{
}

static int pixcir_i2c_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct pixcir_i2c_ts_data *tsdata;
	struct input_dev *input;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int error;
	int i;
	u8 buf[33];

	printk("pixcir_i2c_ts_probe\n");
	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata)
	{
		dev_err(&client->dev, "failed to allocate driver data!\n");
		error = -ENOMEM;
		dev_set_drvdata(&client->dev, NULL);
		return error;
	}

	dev_set_drvdata(&client->dev, tsdata);

	input = input_allocate_device();
	if (!input)
	{
		dev_err(&client->dev, "failed to allocate input device!\n");
		error = -ENOMEM;
		input_free_device(input);
		kfree(tsdata);
	}

	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input->absbit[0] = BIT(ABS_X) | BIT(ABS_Y); // for android
	input_set_abs_params(input, ABS_X, TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_Y, TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 255, 0, 0);
#ifdef TWO_POINTS
	input_set_abs_params(input, ABS_MT_POSITION_X, TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 25, 0, 0);
#endif

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = pixcir_ts_open;
	input->close = pixcir_ts_close;

	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input = input;

	INIT_WORK(&tsdata->work.work, pixcir_ts_poscheck);

	tsdata->irq = client->irq;

	if (input_register_device(input))
	{
		input_free_device(input);
		kfree(tsdata);
	}

	tsdata->discard = 0;
	tsdata->oldtouching = 0;
	gpio_direction_output(gpio_shutdown, 1);
	gpio_direction_input(gpio_irq);
	gpio_enable_edge_int(gpio_to_idx(gpio_irq), 1, 0);
	if (request_irq(tsdata->irq, pixcir_ts_isr, IRQF_TRIGGER_FALLING,client->name, tsdata))
	{
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input = NULL;
	}

	device_init_wakeup(&client->dev, 1);

	/*********************************V2.0-Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev))
	{
		error = PTR_ERR(i2c_dev);
		return error;
	}

	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,
			client->adapter->nr), NULL, "pixcir_i2c_ts%d", client->adapter->nr);
	if (IS_ERR(dev))
	{
		error = PTR_ERR(dev);
		return error;
	}
	/*********************************V2.0-Bee-0928-BOTTOM****************************************/
	dev_err(&tsdata->client->dev, "insmod successfully!\n");

	msleep(200);
	memset(buf, 0xff, sizeof(buf));
	buf[0] = 20;
	i2c_master_send(tsdata->client, buf, 1);
	i2c_master_recv(tsdata->client, buf, sizeof(buf));
	for(i=0; i<sizeof(buf); i++)
		printk("register[%d] = %d\n ", i, buf[i]);
	
//	msleep(1);
//	//calibration
//	buf[0] = 0x37;
//	buf[1] = 0x03;
//	i2c_master_send(tsdata->client, buf, 2);
//	msleep(5000);
	
	return 0;
}

static int pixcir_i2c_ts_remove(struct i2c_client *client)
{
	int error;
	struct i2c_dev *i2c_dev;
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	free_irq(tsdata->irq, tsdata);
	/*********************************V2.0-Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev))
	{
		error = PTR_ERR(i2c_dev);
		return error;
	}
	return_i2c_dev(i2c_dev);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, client->adapter->nr));
	/*********************************V2.0-Bee-0928-BOTTOM****************************************/
	input_unregister_device(tsdata->input);
	kfree(tsdata);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static int pixcir_i2c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(tsdata->irq);

	return 0;
}

static int pixcir_i2c_ts_resume(struct i2c_client *client)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tsdata->irq);

	return 0;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_open                                         */
/*********************************V2.0-Bee-0928****************************************/
static int pixcir_open(struct inode *inode, struct file *file)
{
	int subminor;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;
	int ret = 0;
#if PIXCIR_DEBUG
	printk("enter pixcir_open function\n");
#endif
	subminor = iminor(inode);
#if PIXCIR_DEBUG
	printk("subminor=%d\n",subminor);
#endif
	lock_kernel();
	i2c_dev = i2c_dev_get_by_minor(subminor);
	if (!i2c_dev)
	{
		printk("error i2c_dev\n");
		return -ENODEV;
	}

	adapter = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adapter)
	{

		return -ENODEV;
	}
	//printk("after i2c_dev_get_by_minor\n");

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
	{
		i2c_put_adapter(adapter);
		ret = -ENOMEM;
	}
	snprintf(client->name, I2C_NAME_SIZE, "pixcir_i2c_ts%d", adapter->nr);
	client->driver = &pixcir_i2c_ts_driver;
	client->adapter = adapter;
	//if(i2cdev_check_addr(client->adapter,0x5c))
	//	return -EBUSY;
	file->private_data = client;

	return 0;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_ioctl                                        */
/*********************************V2.0-Bee-0928****************************************/
static int pixcir_ioctl(struct file *file, unsigned int cmd, unsigned int arg)
{
	//printk("ioctl function\n");
	struct i2c_client *client = (struct i2c_client *) file->private_data;
#if PIXCIR_DEBUG
	printk("cmd = %d,arg = %d\n", cmd, arg);
#endif

	switch (cmd)
	{
	case CALIBRATION_FLAG: //CALIBRATION_FLAG = 1
#if PIXCIR_DEBUG
		printk("CALIBRATION\n");
#endif
		client->addr = SLAVE_ADDR;
		status_reg = 0;
		status_reg = CALIBRATION_FLAG;
		break;

	default:
		break;//return -ENOTTY;
	}
	return 0;
}


/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_write                                        */
/*********************************V2.0-Bee-0928****************************************/
static ssize_t pixcir_write(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client;
	char *tmp;
	static int ret=0;

	client = file->private_data;

	//printk("pixcir_write function\n");
	switch(status_reg)
	{
		case CALIBRATION_FLAG: //CALIBRATION_FLAG=1
		tmp = kmalloc(count,GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;
		if (copy_from_user(tmp,buf,count))
		{ 	
			printk("CALIBRATION_FLAG copy_from_user error\n");
			kfree(tmp);
			return -EFAULT;
		}
		ret = i2c_master_send(client,tmp,count);
#if PIXCIR_DEBUG
		printk("CALIBRATION_FLAG,i2c_master_send ret = %d\n",ret);
#endif
		mdelay(100);
		if(ret!=count)
		{
			printk("CALIBRATION_FLAG,Unable to write to i2c page for calibratoion!\n");
		}

		kfree(tmp);

		status_reg = 0;
		break;


		default:
		break;
	}
	return ret;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_release                                         */
/*********************************V2.0-Bee-0928****************************************/
static int pixcir_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;
   #if PIXCIR_DEBUG
	printk("enter pixcir_release funtion\n");
   #endif
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

/*********************************V2.0-Bee-0928-TOP****************************************/
static const struct file_operations pixcir_i2c_ts_fops =
{	.owner = THIS_MODULE,  
	.write = pixcir_write,
	.open = pixcir_open, 
	.unlocked_ioctl = pixcir_ioctl,
	.release = pixcir_release, 
};
/*********************************V2.0-Bee-0928-BOTTOM****************************************/

static const struct i2c_device_id pixcir_i2c_ts_id[] =
{
	{ "pixcir168", 0 },
	{ }
};
MODULE_DEVICE_TABLE( i2c, pixcir_i2c_ts_id);

static struct i2c_driver pixcir_i2c_ts_driver =
{ 	.driver =
		{
			.owner = THIS_MODULE,
			.name = "pixcir_i2c_ts_driver_v1.5",
		}, 
	.probe = pixcir_i2c_ts_probe, 
	.remove = pixcir_i2c_ts_remove,
	.suspend = pixcir_i2c_ts_suspend, 
	.resume = pixcir_i2c_ts_resume,
	.id_table = pixcir_i2c_ts_id, 
};

static int __init pixcir_i2c_ts_init(void)
{
	int ret;
	printk("pixcir_i2c_init\n");
	pixcir_wq = create_singlethread_workqueue("pixcir_wq");
	if(!pixcir_wq)
	return -ENOMEM;
	/*********************************V2.0-Bee-0928-TOP****************************************/
	ret = register_chrdev(I2C_MAJOR,"pixcir_i2c_ts",&pixcir_i2c_ts_fops);
	if(ret)
	{
		printk(KERN_ERR "%s:register chrdev failed\n",__FILE__);
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE, "pixcir_i2c_dev");
	if (IS_ERR(i2c_dev_class))
	{
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	/********************************V2.0-Bee-0928-BOTTOM******************************************/
	return i2c_add_driver(&pixcir_i2c_ts_driver);
}

static void __exit pixcir_i2c_ts_exit(void)
{
	i2c_del_driver(&pixcir_i2c_ts_driver);
	/********************************V2.0-Bee-0928-TOP******************************************/
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR,"pixcir_i2c_ts");
	/********************************V2.0-Bee-0928-BOTTOM******************************************/
	if(pixcir_wq)
	destroy_workqueue(pixcir_wq);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

module_init( pixcir_i2c_ts_init);
module_exit( pixcir_i2c_ts_exit);

