#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/saradc.h>
#include "saradc_reg.h"

struct saradc {
	spinlock_t lock;
	struct calibration *cal;
	int cal_num;
};

static struct saradc *gp_saradc;

#define CHAN_XP	CHAN_0
#define CHAN_YP	CHAN_1
#define CHAN_XN	CHAN_2
#define CHAN_YN	CHAN_3

#define INTERNAL_CAL_NUM	6

static u8 chan_mux[SARADC_CHAN_NUM] = {0,1,2,3,4,5,6,7};


static void saradc_reset(void)
{
	int i;

	//set adc clock as 1.28Mhz
	set_clock_divider(20);
	enable_clock();
	enable_adc();

	set_sample_mode(DIFF_MODE);
	set_tempsen(0);
	disable_fifo_irq();
	disable_continuous_sample();
	disable_chan0_delta();
	disable_chan1_delta();

	set_input_delay(10, INPUT_DELAY_TB_1US);
	set_sample_delay(10, SAMPLE_DELAY_TB_1US);
	set_block_delay(10, BLOCK_DELAY_TB_1US);
	
	// channels sampling mode setting
	for(i=0; i<SARADC_CHAN_NUM; i++) {
		set_sample_sw(i, IDLE_SW);
		set_sample_mux(i, chan_mux[i]);
	}
	
	// idle mode setting
	set_idle_sw(IDLE_SW);
	set_idle_mux(chan_mux[CHAN_0]);
	
	// detect mode setting
	set_detect_sw(DETECT_SW);
	set_detect_mux(chan_mux[CHAN_0]);
	disable_detect_sw();
	disable_detect_pullup();
	set_detect_irq_pol(0);
	disable_detect_irq();

	enable_sample_engine();

//	printk("ADCREG reg0 =%x\n", get_reg(SAR_ADC_REG0));
//	printk("ADCREG ch list =%x\n", get_reg(SAR_ADC_CHAN_LIST));
//	printk("ADCREG avg =%x\n", get_reg(SAR_ADC_AVG_CNTL));
//	printk("ADCREG reg3=%x\n", get_reg(SAR_ADC_REG3));
//	printk("ADCREG ch72 sw=%x\n", get_reg(SAR_ADC_AUX_SW));
//	printk("ADCREG ch10 sw=%x\n", get_reg(SAR_ADC_CHAN_10_SW));
//	printk("ADCREG detect&idle=%x\n", get_reg(SAR_ADC_DETECT_IDLE_SW));
}

static int  saradc_internal_cal(struct calibration *cal)
{
	return -1;
	
	int i;
	int voltage[4] = {CAL_0P55V, CAL_1P10V, CAL_1P65V, CAL_2P20V};

	cal->ref = 0;
	(cal+1)->ref = 170;
	(cal+2)->ref = 341;
	(cal+3)->ref = 511;
	(cal+4)->ref = 684;
	(cal+5)->ref = 1023;
	
	set_cal_mux(MUX_CAL);
	enable_cal_res_array();	
	for (i=1; i<5; i++) {
		set_cal_voltage(voltage[i]);
		(cal+i)->val = get_adc_sample(-1);
		if ((cal+i)->val < 0) {
			printk(KERN_INFO "saradc calibration fail\n");
			return -1;
		}
	}
	
	printk(KERN_INFO "saradc calibration ok\n");
	return 0;
}

static int saradc_get_cal_value(struct calibration *cal, int num, int val)
{
	int ret = -1;
	int i;
	
	if (num < 2)
		return val;
		
	if (val <cal[0].val)
		return cal[0].ref;

	if (val > cal[num-1].val)
		return cal[num-1].ref;
	
	for (i=0; i<num; i++) {
		if (val < cal[i+1].val) {
			ret = val - cal[i].val;
			ret *= cal[i+1].ref - cal[i].ref;
			ret /= cal[i+1].val - cal[i].val;
			ret += cal[i].ref;
			break;
		}
	}
	return ret;
}


int get_adc_sample(int chan)
{
	int count;
	int value;
	int sum;
	
	if (!gp_saradc)
		return -1;
		
	spin_lock(&gp_saradc->lock);

	set_chan_list(chan, 1);
	set_avg_mode(chan, NO_AVG_MODE, SAMPLE_NUM_8);
	set_sample_mux(chan, chan_mux[chan]);
	set_detect_mux(chan_mux[chan]);
	set_idle_mux(chan_mux[chan]); // for revb
	enable_sample_engine();
	start_sample();

	// Read any CBUS register to delay one clock cycle after starting the sampling engine
	// The bus is really fast and we may miss that it started
	{ count = get_reg(ISA_TIMERE); }

	count = 0;
	while (delta_busy() || sample_busy() || avg_busy()) {
		if (++count > 10000) {
        			printk(KERN_ERR "ADC busy error.\n");
			goto end;
		}
	}
    stop_sample();
    
    sum = 0;
    count = 0;
    value = get_fifo_sample();
	while (get_fifo_cnt()) {
        value = get_fifo_sample() & 0x3ff;
        if ((value != 0x1fe) && (value != 0x1ff)) {
			sum += value & 0x3ff;
            count++;
        }
	}
	value = (count) ? (sum / count) : (-1);

end:
	//printk("ch%d = %d, count=%d\n", chan, value, count);
	disable_sample_engine();
	spin_unlock(&gp_saradc->lock);
	return value;
}

int saradc_ts_service(int cmd)
{
	int value = -1;
	
	switch (cmd) {
	case CMD_GET_X:
		//set_sample_sw(CHAN_YP, X_SW);
		value = get_adc_sample(CHAN_YP);
		set_sample_sw(CHAN_XP, Y_SW); // preset for y
		break;

	case CMD_GET_Y:
		//set_sample_sw(CHAN_XP, Y_SW);
		value = get_adc_sample(CHAN_XP);
		break;

	case CMD_GET_Z1:
		set_sample_sw(CHAN_XP, Z1_SW);
		value = get_adc_sample(CHAN_XP);
		break;

	case CMD_GET_Z2:
		set_sample_sw(CHAN_YN, Z2_SW);
		value = get_adc_sample(CHAN_YN);
		break;

	case CMD_GET_PENDOWN:
		value = !detect_level();
		set_sample_sw(CHAN_YP, X_SW); // preset for x
		break;
	
	case CMD_INIT_PENIRQ:
		enable_detect_pullup();
		enable_detect_sw();
		value = 0;
		printk(KERN_INFO "init penirq ok\n");
		break;

	case CMD_SET_PENIRQ:
		enable_detect_pullup();
		enable_detect_sw();
		value = 0;
		break;
		
	case CMD_CLEAR_PENIRQ:
		disable_detect_pullup();
		disable_detect_sw();
		value = 0;
		break;

	default:
		break;		
	}
	
	return value;
}

static ssize_t saradc_ch0_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", get_adc_sample(0));
}
static ssize_t saradc_ch1_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", get_adc_sample(1));
}
static ssize_t saradc_ch2_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", get_adc_sample(2));
}
static ssize_t saradc_ch3_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", get_adc_sample(3));
}
static ssize_t saradc_ch4_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", get_adc_sample(4));
}
static ssize_t saradc_ch5_show(struct class *cla, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", get_adc_sample(5));
}
static struct class_attribute saradc_class_attrs[] = {
    __ATTR_RO(saradc_ch0),
    __ATTR_RO(saradc_ch1),
    __ATTR_RO(saradc_ch2),
    __ATTR_RO(saradc_ch3),
    __ATTR_RO(saradc_ch4),
    __ATTR_RO(saradc_ch5),                    
    __ATTR_NULL
};
static struct class saradc_class = {
    .name = "saradc",
    .class_attrs = saradc_class_attrs,
};

static int __init saradc_probe(struct platform_device *pdev)
{
	int err;
	struct saradc *saradc;
	struct saradc_platform_data *pdata = pdev->dev.platform_data;

	saradc = kzalloc(sizeof(struct saradc), GFP_KERNEL);
	if (!saradc) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	saradc_reset();
	gp_saradc = saradc;

	if (pdata && pdata->cal) {
		saradc->cal = pdata->cal;
		saradc->cal_num = pdata->cal;
		printk(KERN_INFO "saradc use signed calibration data\n");
	}
	else {
		printk(KERN_INFO "saradc use internal calibration\n");
		saradc->cal = kzalloc(sizeof(struct calibration) * 
				INTERNAL_CAL_NUM, GFP_KERNEL);
		if (saradc->cal) {
			if (saradc_internal_cal(saradc->cal) < 0) {
				kfree(saradc->cal);
				saradc->cal = 0;
				printk(KERN_INFO "saradc calibration fail\n");
			}
			else {
				saradc->cal_num = INTERNAL_CAL_NUM;
				printk(KERN_INFO "saradc calibration ok\n");
			}
		}
	}
		
	spin_lock_init(&saradc->lock);	
	return 0;

err_free_mem:
	kfree(saradc);
	printk(KERN_INFO "saradc probe error\n");	
	return err;
}

static int __init saradc_remove(struct platform_device *pdev)
{
	struct saradc *saradc = platform_get_drvdata(pdev);
	disable_adc();
	disable_sample_engine();
	gp_saradc = 0;
	kfree(saradc);
	return 0;
}

static struct platform_driver saradc_driver = {
	.probe      = saradc_probe,
	.remove     = saradc_remove,
	.suspend    = NULL,
	.resume     = NULL,
	.driver     = {
		.name   = "saradc",
	},
};

static int __devinit saradc_init(void)
{
	printk(KERN_INFO "SARADC Driver init.\n");
	class_register(&saradc_class);
	return platform_driver_register(&saradc_driver);
}

static void __exit saradc_exit(void)
{
	printk(KERN_INFO "SARADC Driver exit.\n");
	platform_driver_unregister(&saradc_driver);
	class_unregister(&saradc_class);
}

module_init(saradc_init);
module_exit(saradc_exit);

MODULE_AUTHOR("aml");
MODULE_DESCRIPTION("SARADC Driver");
MODULE_LICENSE("GPL");