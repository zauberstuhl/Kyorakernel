#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <mach/am_regs.h>
#include "aml_audio_hw.h"

#define APB_BASE	0x5000

#define ADAC_RESET                		0x00
#define ADAC_LATCH                		0x01
#define ADAC_CLOCK                		0x02
// 0x03-0x0b  reserved
#define ADAC_I2S_CONFIG_REG1      		0x0c
#define ADAC_I2S_CONFIG_REG2      		0x0d
// 0x0e - 0x0f reserved
#define ADAC_POWER_CTRL_REG1      		0x10
#define ADAC_POWER_CTRL_REG2      		0x11
#define ADAC_POWER_CTRL_REG3      		0x12
// 0x13-0x17 reserved
#define ADAC_MUTE_CTRL_REG1       		0x18
#define ADAC_MUTE_CTRL_REG2						0x19
#define ADAC_DAC_ADC_MIXER        		0x1a
// 0x1b-0x1f reserved
#define ADAC_PLAYBACK_VOL_CTRL_LSB              0x20
#define ADAC_PLAYBACK_VOL_CTRL_MSB              0x21
#define ADAC_STEREO_HS_VOL_CTRL_LSB             0x22
#define ADAC_STEREO_HS_VOL_CTRL_MSB             0x23

#define ADAC_MAXREG	0x24

struct snd_soc_codec_device soc_codec_dev_aml_m1;
static struct snd_soc_codec *aml_m1_codec;

/* codec private data */
struct aml_m1_codec_priv {
	struct snd_soc_codec codec;
	u16 reg_cache[ADAC_MAXREG];
	unsigned int sysclk;
};

void latch_ (struct snd_soc_codec* codec)
{
    int latch;
    latch = 1;
    snd_soc_write(codec, ADAC_LATCH, latch);
    latch = 0;
    snd_soc_write(codec, ADAC_LATCH, latch);
}

void aml_m1_reset(struct snd_soc_codec* codec)
{
	unsigned long   data32;
	//audio_set_clk(AUDIO_CLK_FREQ_48,0);
	WRITE_MPEG_REG( HHI_AUD_PLL_CNTL, READ_MPEG_REG(HHI_AUD_PLL_CNTL) & ~(1 << 15));
	WRITE_MPEG_REG_BITS(HHI_AUD_CLK_CNTL, 1, 23, 1);
  msleep(100);
  
	data32  = 0;
  data32 |= 0 << 15;  // [15]     audac_soft_reset_n
  data32 |= 0 << 14;  // [14]     audac_reset_ctrl: 0=use audac_reset_n pulse from reset module; 1=use audac_soft_reset_n.
  data32 |= 0 << 9;   // [9]      delay_rd_en
  data32 |= 0 << 8;   // [8]      audac_reg_clk_inv
  data32 |= 0 << 1;   // [7:1]    audac_i2caddr
  data32 |= 0 << 0;   // [0]      audac_intfsel: 0=use host bus; 1=use I2C.
  WRITE_MPEG_REG(AIU_AUDAC_CTRL0, data32);
  
   // Enable APB3 fail on error
  data32  = 0;
  data32 |= 1     << 15;  // [15]     err_en
  data32 |= 255   << 0;   // [11:0]   max_err
  WRITE_MPEG_REG(AIU_AUDAC_CTRL1, data32);
  
	snd_soc_write(codec, ADAC_RESET, (0<<1));
	snd_soc_write(codec, ADAC_RESET, (0<<1));
	snd_soc_write(codec, ADAC_RESET, (0<<1));
	snd_soc_write(codec, ADAC_RESET, (0<<1));
	snd_soc_write(codec, ADAC_RESET, (0<<1));
	msleep(100);
	snd_soc_write(codec,ADAC_CLOCK, 0);
	snd_soc_write(codec,ADAC_I2S_CONFIG_REG1, 6);	
	snd_soc_write(codec, ADAC_I2S_CONFIG_REG2, 1); 		// I2S
	snd_soc_write(codec, ADAC_I2S_CONFIG_REG2, 0<<3); // split
	
	snd_soc_write(codec, ADAC_POWER_CTRL_REG1, 0xc3);
	snd_soc_write(codec, ADAC_POWER_CTRL_REG2, 0);
	snd_soc_write(codec, ADAC_MUTE_CTRL_REG1,0);
	snd_soc_write(codec,ADAC_DAC_ADC_MIXER, 0);
  snd_soc_write(codec,ADAC_PLAYBACK_VOL_CTRL_LSB, 0x54);
  snd_soc_write(codec,ADAC_PLAYBACK_VOL_CTRL_MSB, 0x54);
  snd_soc_write(codec,ADAC_STEREO_HS_VOL_CTRL_LSB, 0x28);
  snd_soc_write(codec,ADAC_STEREO_HS_VOL_CTRL_MSB, 0x28); 
   
	latch_(codec);
	snd_soc_write(codec, ADAC_POWER_CTRL_REG2, (0<<7));
    latch_(codec);
	snd_soc_write(codec, ADAC_POWER_CTRL_REG2, (1<<7));
    latch_(codec);

	snd_soc_write(codec, ADAC_RESET, (0<<1));
    latch_(codec);
	snd_soc_write(codec, ADAC_RESET, (1<<1));
    latch_(codec);
  msleep(100);
}

static const DECLARE_TLV_DB_SCALE(dac_volume, -12600, 150, 0);
static const DECLARE_TLV_DB_SCALE(hs_volume, -4000, 100, 0);

static const struct snd_kcontrol_new amlm1_snd_controls[] = {
	SOC_DOUBLE_R_TLV("LINEOUT Volume", ADAC_PLAYBACK_VOL_CTRL_LSB, ADAC_PLAYBACK_VOL_CTRL_MSB,
	       0, 84, 0, dac_volume),
	      
	SOC_DOUBLE_R_TLV("HeadSet Volume", ADAC_STEREO_HS_VOL_CTRL_LSB, ADAC_STEREO_HS_VOL_CTRL_MSB,
	       0, 46, 0, hs_volume),
	
	SOC_DOUBLE("AUX Switch", ADAC_POWER_CTRL_REG1, 6, 7, 1, 0),
};

static const struct snd_soc_dapm_widget amlm1_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("LINEOUTL"),
	SND_SOC_DAPM_OUTPUT("LINEOUTR"),
	SND_SOC_DAPM_OUTPUT("HP_L"),
	SND_SOC_DAPM_OUTPUT("HP_R"),
	
	SND_SOC_DAPM_DAC("DACL", "Left DAC Playback", ADAC_POWER_CTRL_REG1, 0, 0),
	SND_SOC_DAPM_DAC("DACR", "Right DAC Playback", ADAC_POWER_CTRL_REG1, 1, 0),
	
	SND_SOC_DAPM_PGA("HSL", ADAC_POWER_CTRL_REG1, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HSR", ADAC_POWER_CTRL_REG1, 5, 0, NULL, 0),
	
	SND_SOC_DAPM_PGA("PDZ", ADAC_POWER_CTRL_REG2, 7, 0, NULL, 0),
};

/* Target, Path, Source */

static const struct snd_soc_dapm_route audio_map[] = {
	{"LINEOUTL", NULL, "DACL"},
	{"LINEOUTR", NULL, "DACR"},
	{"HP_L", NULL, "HSL"},
	{"HP_R", NULL, "HSR"},
};

static int amlm1_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, amlm1_dapm_widgets,
				  ARRAY_SIZE(amlm1_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	return 0;
}
static int aml_m1_volatile_register(unsigned int reg)
{
	return 0;
}
static int aml_m1_write(struct snd_soc_codec *codec, unsigned int reg,
							unsigned int value)
{
	u16 *reg_cache = codec->reg_cache;

	if (reg >= codec->reg_cache_size)
		return -EINVAL;
	WRITE_APB_REG((APB_BASE+(reg<<2)), value);
	//latch(codec);
	reg_cache[reg] = value;
	return 0;
}

static unsigned int aml_m1_read(struct snd_soc_codec *codec,
							unsigned int reg)
{
	u16 *reg_cache = codec->reg_cache;
	if (reg >= codec->reg_cache_size)
		return -EINVAL;
	
	if(codec->volatile_register(reg)){
		//return READ_APB_REG(APB_BASE+(reg<<2));
	}
	return READ_APB_REG(APB_BASE+(reg<<2));
	//return reg_cache[reg];
}

static int aml_m1_codec_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct aml_m1_codec_priv *aml = codec->private_data;

	// TODO
	
	
	return 0;
}


static int aml_m1_codec_pcm_prepare(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	/* set active */
	
	// TODO

	return 0;
}

static void aml_m1_codec_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	/* deactivate */
	if (!codec->active) {
		udelay(50);
		
		// TODO
	}
}

static int aml_m1_codec_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 reg;
	// TODO

	reg = snd_soc_read(codec, ADAC_MUTE_CTRL_REG1);
	if(mute){
		reg |= 3;
	}
	else{
		reg &= ~3;
	}
	printk("aml_m1_codec_mute mute=%d\n",mute);
//	snd_soc_write(codec, ADAC_MUTE_CTRL_REG1, reg);
	return 0;
}

static int aml_m1_codec_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aml_m1_codec_priv *aml = codec->private_data;
	unsigned long data = 0;
	
	aml->sysclk = freq;
	switch (freq) {
	case 32000:
		data = 6;
		break;
	case 44100:
		data = 7;
		break;
	case 48000:
		data = 8;
		break;
	case 96000:
		data = 10;
		break;
	default:
		data = 6;
		break;
	}
	//snd_soc_write(codec,ADAC_CLOCK, 0);
	//snd_soc_write(codec,ADAC_I2S_CONFIG_REG1, data);
	return 0;
}


static int aml_m1_codec_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	
	// TODO
	
	return 0;
}

#define AML_RATES SNDRV_PCM_RATE_8000_96000

#define AML_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
	SNDRV_PCM_FMTBIT_S24_LE)


static struct snd_soc_dai_ops aml_m1_codec_dai_ops = {
	.prepare	= aml_m1_codec_pcm_prepare,
	.hw_params	= aml_m1_codec_hw_params,
	.shutdown	= aml_m1_codec_shutdown,
	.digital_mute	= aml_m1_codec_mute,
	.set_sysclk	= aml_m1_codec_set_dai_sysclk,
	.set_fmt	= aml_m1_codec_set_dai_fmt,
};

struct snd_soc_dai aml_m1_codec_dai = {
	.name = "AML-M1",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AML_RATES,
		.formats = AML_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AML_RATES,
		.formats = AML_FORMATS,},
	.ops = &aml_m1_codec_dai_ops,
	.symmetric_rates = 1,
};
EXPORT_SYMBOL_GPL(aml_m1_codec_dai);

static int aml_m1_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
	    break;
	default:
	    break;
	}
	codec->bias_level = level;
	return 0;
}

static int aml_m1_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	if (!aml_m1_codec) {
		dev_err(&pdev->dev, "AML_M1_CODEC not yet discovered\n");
		return -ENODEV;
	}
	codec = aml_m1_codec;			
	socdev->card->codec = codec;	
	
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		kfree(codec);
		dev_err(codec->dev, "aml m1 codec: failed to create pcms: %d\n", ret);
		goto pcm_err;
	}
	
pcm_err:
/*
	snd_soc_add_controls(codec, amlm1_snd_controls,
				ARRAY_SIZE(amlm1_snd_controls));
	amlm1_add_widgets(codec);
*/
	return 0;
}


static int aml_m1_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	return 0;
}

#ifdef CONFIG_PM
static int aml_m1_codec_suspend(struct platform_device* pdev)
{
    printk("aml_m1_codec_suspend\n");
    return 0;
}

static int aml_m1_codec_resume(struct platform_device* pdev)
{
    printk("aml_m1_codec resume\n");
    return 0;
}
#endif

struct snd_soc_codec_device soc_codec_dev_aml_m1 = {
	.probe =	aml_m1_codec_probe,
	.remove =	aml_m1_codec_remove,
#ifdef CONFIG_PM	
	.suspend = aml_m1_codec_suspend,
	.resume = aml_m1_codec_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
};
EXPORT_SYMBOL_GPL(soc_codec_dev_aml_m1);

static int aml_m1_register(struct aml_m1_codec_priv* aml_m1)
{
	struct snd_soc_codec* codec = &aml_m1->codec;
	int ret;
		
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "AML_M1_CODEC";
	codec->owner = THIS_MODULE;
	codec->private_data = aml_m1;

	codec->dai = &aml_m1_codec_dai;
	codec->num_dai = 1;

	codec->reg_cache = &aml_m1->reg_cache;
	codec->reg_cache_size = ARRAY_SIZE(aml_m1->reg_cache);
	codec->read = aml_m1_read;
	codec->write = aml_m1_write;
	codec->volatile_register = aml_m1_volatile_register;
	
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = aml_m1_set_bias_level;
	aml_m1_codec_dai.dev = codec->dev;
	
//	aml_m1_reset(codec);
	aml_m1_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	aml_m1_codec = codec;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_dai(&aml_m1_codec_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		goto err_codec;
	}

	return 0;

err_codec:
	snd_soc_unregister_codec(codec);
err:
	aml_m1_codec = NULL;
	kfree(aml_m1);
	return ret;
		
}

static void aml_m1_unregister(struct aml_m1_codec_priv *aml_m1)
{
	snd_soc_unregister_dai(&aml_m1_codec_dai);
	snd_soc_unregister_codec(&aml_m1->codec);
	aml_m1_codec = NULL;
	kfree(aml_m1);
}

static int aml_m1_codec_platform_probe(struct platform_device *pdev)
{
	struct aml_m1_codec_priv *aml_m1;
	struct snd_soc_codec *codec;

	aml_m1 = kzalloc(sizeof(struct aml_m1_codec_priv), GFP_KERNEL);
	if (aml_m1 == NULL)
		return -ENOMEM;

	codec = &aml_m1->codec;

	codec->control_data = NULL;
	codec->hw_write = NULL;
	codec->pop_time = 0;

	codec->dev = &pdev->dev;
	platform_set_drvdata(pdev, aml_m1);

	return aml_m1_register(aml_m1);
}

static int __exit aml_m1_codec_platform_remove(struct platform_device *pdev)
{
	struct aml_m1_codec_priv *aml_m1 = platform_get_drvdata(pdev);

	aml_m1_unregister(aml_m1);
	return 0;
}

static struct platform_driver aml_m1_codec_platform_driver = {
	.driver = {
		.name = "aml_m1_codec",
		.owner = THIS_MODULE,
		},
	.probe = aml_m1_codec_platform_probe,
	.remove = __exit_p(aml_m1_codec_platform_remove),
};

static int __init aml_m1_codec_modinit(void)
{
		return platform_driver_register(&aml_m1_codec_platform_driver);
}

static void __exit aml_m1_codec_exit(void)
{
		platform_driver_unregister(&aml_m1_codec_platform_driver);
}

module_init(aml_m1_codec_modinit);
module_exit(aml_m1_codec_exit);


MODULE_DESCRIPTION("ASoC AML M1 codec driver");
MODULE_AUTHOR("AMLogic Inc.");
MODULE_LICENSE("GPL");
