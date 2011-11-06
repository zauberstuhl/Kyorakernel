/* 
 * sn7325 i2c interface
 * Copyright (C) 2010 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:  wang han<han.wang@amlogic.com>
 */  

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/sn7325.h>

static struct i2c_client *sn7325_client;

static int sn7325_i2c_read(unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = sn7325_client->addr,
            .flags = 0,
            .len = 1,
            .buf = buff,
        },
        {
            .addr = sn7325_client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = buff,
        }
    };

    res = i2c_transfer(sn7325_client->adapter, msgs, 2);
    if (res < 0) {
        pr_err("%s: i2c transfer failed\n", __FUNCTION__);
    }

    return res;
}

static int sn7325_i2c_write(unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msg[] = {
        {
        .addr = sn7325_client->addr,
        .flags = 0,
        .len = len,
        .buf = buff,
        }
    };

    res = i2c_transfer(sn7325_client->adapter, msg, 1);
    if (res < 0) {
        pr_err("%s: i2c transfer failed\n", __FUNCTION__);
    }

    return res;
}

static int sn7325_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int res = 0;
    struct sn7325_platform_data *pdata = NULL;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s: functionality check failed\n", __FUNCTION__);
        res = -ENODEV;
        goto out;
    }
    sn7325_client = client;
    pdata = client->dev.platform_data;
    
    if (pdata->pwr_rst)
    {
        pdata->pwr_rst();
        printk("***sn7325 reset***\n");
    }
    else
    {
        printk("***sn7325 no reset***\n");
    }

out:
    return res;
}

static int sn7325_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id sn7325_id[] = {
    { "sn7325", 0 },
    { }
};

static struct i2c_driver sn7325_driver = {
    .probe = sn7325_probe,
    .remove = sn7325_remove,
    .id_table = sn7325_id,
    .driver = {
    .name = "sn7325",
    },
};

static int __init sn7325_init(void)
{
    int res;

    if (sn7325_client)
    {
        res = 0;
    }
    else
    {
        res = i2c_add_driver(&sn7325_driver);
        if (res < 0) {
            printk("add sn7325 i2c driver error\n");
        }
    }

    return res;
}

arch_initcall(sn7325_init);
//module_init(sn7325_init);

unsigned char get_configIO(unsigned char port)
{
    unsigned char ioconf = 0;

    switch(port)
    {
        case 0:
        case 1:
            ioconf = 0x4+port;
            sn7325_i2c_read(&ioconf, 1);
            break;
    }

    return ioconf;
}

int configIO(unsigned char port, unsigned char ioflag)
{
    int res = 0;
    unsigned char buff_data[2];

    switch(port)
    {
        case 0:
        case 1:
            buff_data[0] = 0x4+port;
            buff_data[1] = ioflag;
            res = sn7325_i2c_write(buff_data, 2);
            break;
    }

    return res;
}

static unsigned char getIOport_level(unsigned char port)
{
    unsigned char iobits = port;

    switch(port)
    {
        case 0:
        case 1:
        case 2:
        case 3:
            sn7325_i2c_read(&iobits, 1);
            break;
    }

    return iobits;
}

unsigned char getIObit_level(unsigned char port, unsigned char offset)
{
    unsigned char iobits = 0;
    unsigned char ioConfig = 0;

    switch( port )
    {
        case 0:
        case 1:
            ioConfig = get_configIO(port);
            if ((iobits>>offset) & 1)
                iobits = port;
            else
                iobits += 2;
            sn7325_i2c_read(&iobits, 1);
            break;
    }

    return (iobits>>offset) & 1;
}

int setIO_level(unsigned char port, unsigned char iobits, unsigned char offset)
{
    int res = 0;
    unsigned char buff_data[2];
    unsigned char ioflag = 0;

    switch( port )
    {
        case 0:
        case 1:
            ioflag = getIOport_level(port+0x2);
            buff_data[0] = 0x2+port;
            buff_data[1] = iobits? (ioflag | (1<<offset)):(ioflag & (~(1<<offset)));
            res = sn7325_i2c_write(buff_data, 2);
            break;
    }

    return res;
}


EXPORT_SYMBOL_GPL(configIO);
EXPORT_SYMBOL_GPL(setIO_level);
EXPORT_SYMBOL_GPL(get_configIO);
EXPORT_SYMBOL_GPL(getIObit_level);
