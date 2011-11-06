#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <linux/major.h>
#include <linux/slab.h>


#include <mach/am_regs.h>
#include <linux/amports/amaudio.h>

#include "amaudio.h"

#define AMAUDIO_DEVICE_COUNT    3

    
MODULE_DESCRIPTION("AMLOGIC Audio Control Interface driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin W");
MODULE_VERSION("1.0.0");

typedef struct {
	unsigned int in_op_ptr;
	unsigned int out_op_ptr;
	unsigned int type;
	unsigned int in_start;
	unsigned int out_start;	
}amaudio_t;

typedef struct{
  const char *name;
  struct device* dev;
  const struct file_operations* fops;
}amaudio_port_t;

extern int if_audio_in_i2s_enable(void);
extern int if_audio_out_enable(void);
extern unsigned int read_i2s_rd_ptr(void);
extern unsigned int audio_in_i2s_wr_ptr(void);
extern unsigned int read_i2s_mute_swap_reg(void);
extern void audio_i2s_swap_left_right(unsigned int flag);

static dev_t amaudio_devno;
static struct class* amaudio_clsp;
static struct cdev*  amaudio_cdevp;

static ssize_t amaudio_write(struct file *file, const char *buf,
                                size_t count, loff_t * ppos);

static ssize_t amaudio_read(struct file *file, char __user *buf, 
															size_t count, loff_t * ppos);
static int amaudio_open(struct inode *inode, struct file *file);

static int amaudio_release(struct inode *inode, struct file *file);

static int amaudio_ioctl(struct inode *inode, struct file *file,
                        unsigned int cmd, ulong arg);

const static struct file_operations amaudio_out_fops = {
  .owner    =   THIS_MODULE,
  .open     =   amaudio_open,
  .release  =   amaudio_release,
  .write    =   amaudio_write,
  .read     =   amaudio_read,
  .ioctl    =   amaudio_ioctl,
};

const static struct file_operations amaudio_in_fops = {
  .owner    =   THIS_MODULE,
  .open     =   amaudio_open,
  .release  =   amaudio_release,
  .write    =   amaudio_write,
  .read     =   amaudio_read,
  .ioctl    =   amaudio_ioctl,
};

const static struct file_operations amaudio_ctl_fops = {
  .owner    =   THIS_MODULE,
  .open     =   amaudio_open,
  .release  =   amaudio_release,
  .ioctl    =   amaudio_ioctl,
};

static amaudio_port_t amaudio_ports[]={
  {
    .name = "amaudio_out",
    .fops = &amaudio_out_fops,
  },
  {
    .name = "amaudio_in",
    .fops = &amaudio_in_fops,
  },
  {
    .name = "amaudio_ctl",
    .fops = &amaudio_ctl_fops,
  },
};


static ssize_t amaudio_write(struct file *file, const char *buf,
                                size_t count, loff_t * ppos)
{
	amaudio_t * amaudio = (amaudio_t *)file->private_data;

    if(count <=0 )
      return -EINVAL;
	if(amaudio->type == 1){
		if(!if_audio_in_i2s_enable()){
			printk("amaudio input can not write now\n");
			return -EINVAL;
		}
		copy_from_user((void*)(amaudio->in_op_ptr+amaudio->in_start), (void*)buf, count);
	}else if(amaudio->type == 0){
		if(!if_audio_out_enable()){
			printk("amaudio output can not write now\n");
			return -EINVAL;
		}
		copy_from_user((void*)(amaudio->out_op_ptr+amaudio->out_start), (void*)buf, count);
	}
	return count;
}
static ssize_t amaudio_read(struct file *file, char __user *buf, 
															size_t count, loff_t * ppos)
{
	amaudio_t * amaudio = (amaudio_t *)file->private_data;
	int len = 0;
    if(count <= 0)
      return -EINVAL;
	if(amaudio->type == 1){
		if(!if_audio_in_i2s_enable()){
			printk("amaudio input can not read now\n");
			return -EINVAL;
		}
        len = copy_to_user((void*)buf, (void*)(amaudio->in_op_ptr+amaudio->in_start), count);
		if(len){
			printk("amaudio read i2s in data failed\n");
		}
        memset((void*)(amaudio->in_op_ptr+amaudio->in_start), 0x78, count);
	}
	else if(amaudio->type == 0){
		if(!if_audio_out_enable()){
			printk("amaudio output can not read now\n");
			return -EINVAL;
		}

		len = copy_to_user((void*)buf, (void*)(amaudio->out_op_ptr+amaudio->out_start), count);
		if(len){
			printk("amaudio read i2s out data failed\n");
		}
	}
	return count - len;
}

static int amaudio_open(struct inode *inode, struct file *file)
{
  amaudio_port_t* this = &amaudio_ports[0];      
  amaudio_t * amaudio = kzalloc(sizeof(amaudio_t), GFP_KERNEL);
  if (if_audio_in_i2s_enable()){
    amaudio->in_start = ioremap_nocache(READ_MPEG_REG(AUDIN_FIFO0_START), 65536);
    //printk("amaudio->in_start = %x \n", amaudio->in_start);
    //printk("(AUDIN_FIFO0_START) = %x \n", READ_MPEG_REG(AUDIN_FIFO0_START));
  }
  if (if_audio_out_enable()){
    amaudio->out_start = ioremap_nocache(READ_MPEG_REG(AIU_MEM_I2S_START_PTR), 32768);
    //printk("amaudio->out_start = %x \n", amaudio->out_start);
	//printk("(AIU_MEM_I2S_START_PTR) = %x \n", READ_MPEG_REG(AIU_MEM_I2S_START_PTR));
  }
  if(iminor(inode) == 0){ // audio out
    printk("open audio out\n");
	amaudio->type = 0;		
  }else if(iminor(inode) == 1){									// audio in
	printk("open audio in\n");
	amaudio->type = 1;
  }else{						// audio control
  	printk("open audio control\n");
	amaudio->type = 2;
  }
  file->private_data = amaudio;
  file->f_op = this->fops;
  return 0;
}
static int amaudio_release(struct inode *inode, struct file *file)
{
	amaudio_t * amaudio = (amaudio_t *)file->private_data;

	if (if_audio_in_i2s_enable()){
      iounmap(amaudio->in_start);
    }
	if (if_audio_out_enable()){
      iounmap(amaudio->out_start);
	}
    kfree(amaudio);    
	return 0;
}
static int amaudio_ioctl(struct inode *inode, struct file *file,
                        unsigned int cmd, ulong arg)
{
	s32 r = 0;
	u32 reg;
	amaudio_t * amaudio = (amaudio_t *)file->private_data;
    switch(cmd){
		case AMAUDIO_IOC_GET_I2S_OUT_SIZE:
			if(if_audio_out_enable()){
				r = READ_MPEG_REG(AIU_MEM_I2S_END_PTR) - READ_MPEG_REG(AIU_MEM_I2S_START_PTR) + 64;
			}else{
				r = -EINVAL;
			}
			break;
		case AMAUDIO_IOC_GET_I2S_OUT_PTR:
			if(if_audio_out_enable()){
				r = read_i2s_rd_ptr() - READ_MPEG_REG(AIU_MEM_I2S_START_PTR);
			}else{
				r = -EINVAL;
			}
			break;
		case AMAUDIO_IOC_SET_I2S_OUT_OP_PTR:
			if(if_audio_out_enable()){
				if(arg < 0 || arg > (READ_MPEG_REG(AIU_MEM_I2S_END_PTR) - READ_MPEG_REG(AIU_MEM_I2S_START_PTR)+64)){
					r = -EINVAL;
				}else{
					amaudio->out_op_ptr = arg;
				}
			}else{
				r = -EINVAL;
			}
			break;
		case AMAUDIO_IOC_GET_I2S_IN_SIZE:
			if(if_audio_in_i2s_enable()){
				r = READ_MPEG_REG(AUDIN_FIFO0_END) - READ_MPEG_REG(AUDIN_FIFO0_START) + 8;
			}else{
				r = -EINVAL;
			}
			break;
		case AMAUDIO_IOC_GET_I2S_IN_PTR:
			if(if_audio_in_i2s_enable()){
				r = audio_in_i2s_wr_ptr() - READ_MPEG_REG(AUDIN_FIFO0_START);
			}else{
				r = -EINVAL;
			}
			break;
		case AMAUDIO_IOC_SET_I2S_IN_OP_PTR:
			if(if_audio_in_i2s_enable()){
				if(arg < 0 || arg > (READ_MPEG_REG(AUDIN_FIFO0_END)- READ_MPEG_REG(AUDIN_FIFO0_START)+8)){
					r = -EINVAL;
				}else{
					amaudio->in_op_ptr = arg;
				}
			}else{
				r = -EINVAL;
			}
			break;
		case AMAUDIO_IOC_SET_LEFT_MONO:
			audio_i2s_swap_left_right(1);
			break;
		case AMAUDIO_IOC_SET_RIGHT_MONO:
			audio_i2s_swap_left_right(2);
			break;
		case AMAUDIO_IOC_SET_STEREO:
			audio_i2s_swap_left_right(0);
			break;
		case AMAUDIO_IOC_SET_CHANNEL_SWAP:
			reg = read_i2s_mute_swap_reg();
			if(reg & 0x3)
				audio_i2s_swap_left_right(0);
			else
				audio_i2s_swap_left_right(3);
			break;
		default:
			break;
		
	};
	return r;
}

static const struct file_operations amaudio_fops = {
  .owner    =   THIS_MODULE,
  .open     =   amaudio_open,
  .ioctl    =   amaudio_ioctl,
  .release  =   amaudio_release,
};

static int __init amaudio_init(void)
{
  int ret = 0;
  int i=0;
  amaudio_port_t* ap;

  ret = alloc_chrdev_region(&amaudio_devno, 0, AMAUDIO_DEVICE_COUNT, AMAUDIO_DEVICE_NAME);
  if(ret < 0){
    printk(KERN_ERR"amaudio: faild to alloc major number\n");
    ret = - ENODEV;
    goto err;
  }
  amaudio_clsp = class_create(THIS_MODULE, AMAUDIO_CLASS_NAME);
  if(IS_ERR(amaudio_clsp)){
    ret = PTR_ERR(amaudio_clsp);
    goto err1;
  }
  amaudio_cdevp = kmalloc(sizeof(struct cdev), GFP_KERNEL);
  if(!amaudio_cdevp){
    printk(KERN_ERR"amaudio: failed to allocate memory\n");
    ret = -ENOMEM;
    goto err2;
  }
  // connect the file operation with cdev
  cdev_init(amaudio_cdevp, &amaudio_fops);
  amaudio_cdevp->owner = THIS_MODULE;
  // connect the major/minor number to cdev
  ret = cdev_add(amaudio_cdevp, amaudio_devno, AMAUDIO_DEVICE_COUNT);
  if(ret){
    printk(KERN_ERR "amaudio:failed to add cdev\n");
    goto err3;
  } 
  for(ap = &amaudio_ports[0], i=0; i< AMAUDIO_DEVICE_COUNT; ap++,  i++){    
    ap->dev = device_create(amaudio_clsp, NULL, MKDEV(MAJOR(amaudio_devno),i), NULL,amaudio_ports[i].name);
    if(IS_ERR(ap->dev)){
      printk(KERN_ERR"amaudio: failed to create amaudio device node\n");
      goto err4;
    }
  }
  
  printk(KERN_INFO"amaudio: device %s created\n", AMAUDIO_DEVICE_NAME);
  return 0;

err4:
  cdev_del(amaudio_cdevp);
err3:
  kfree(amaudio_cdevp);
err2:
  class_destroy(amaudio_clsp);  
err1:
  unregister_chrdev_region(amaudio_devno, AMAUDIO_DEVICE_COUNT);
err:
  return ret;  
  
}

static void __exit amaudio_exit(void)
{
  int i=0;
  unregister_chrdev_region(amaudio_devno, 1);
  for(i=0; i< AMAUDIO_DEVICE_COUNT; i++){
    device_destroy(amaudio_clsp, MKDEV(MAJOR(amaudio_devno),i));
  }
  cdev_del(amaudio_cdevp);
  kfree(amaudio_cdevp);
  class_destroy(amaudio_clsp);
  return;
}

module_init(amaudio_init);
module_exit(amaudio_exit);
