#include <linux/kfifo.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <asm/byteorder.h>

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <linux/string.h>

#include "sy24145.h"

static uint32_t sample_rate_g;

struct sy24145 {
	struct i2c_client *client;
	struct regmap *regmap;
	unsigned int mstr_volume;
	unsigned int l_volume;
	unsigned int r_volume;

	bool l_mute;
	bool r_mute;
};

static const struct reg_default sy24145_reg_defaults_8[] = {
	{ CLOCK_CONTROL, 0x1A },
	{ DEVICE_ID, 0x25 },
	{ ERROR_STATUS, 0x00 },
	{ SYSTEM_CONTROL_1, 0x5F },
	{ SYSTEM_CONTROL_2, 0x9E },
	{ SYSTEM_CONTROL_3, 0x7C },
	{ SOFT_MUTE, 0x30 },
	{ MASTER_VOLUME, 0x00 },
	{ CHANNEL1_VOLUME, 0x9F },
	{ CHANNEL2_VOLUME, 0x9F },
	{ ERROR_STATUS_2, 0x00 },
	{ VOL_FTUNE, 0x00 },
	{ SOFT_RESET, 0x00 },
	{ MODULATION_LIMIT, 0x77 },
	{ PWM_A_CHANNEL_DELAY, 0x00 },
	{ PWM_B_CHANNEL_DELAY, 0x00 },
	{ PWM_C_CHANNEL_DELAY, 0x00 },
	{ PWM_D_CHANNEL_DELAY, 0x00 },
	{ I2S_CONTROL, 0x10 },
	{ DSP_CONTROL_1, 0x06 },
	{ MONITOR_PIN_CONFIGURED_1, 0x00 },
	{ MONITOR_PIN_CONFIGURED_2, 0x00 },
	{ PWM_DIRECT_CURRENT_THRESHOLD, 0x05 },
	{ SHORT_CONTROL, 0xBD },
	{ FAULT_OUTPUT_TIME, 0x02 },
	{ OPERATION_MODE, 0x05 },
	{ CHECKSUM_CONTROL, 0x00 },
	{ INPUT_MUX, 0x00 },
	{ DSP_CONTROL_2, 0x00 },
	{ PWM_CONTROL, 0x30 },
	{ FAULT_SELECT, 0x12 },
	{ CHANNEL1_EQ_FILTER_CONTROL_1, 0x00 },
	{ CHANNEL1_EQ_FILTER_CONTROL_2, 0x00 },
	{ CHANNEL2_EQ_FILTER_CONTROL_1, 0x00 },
	{ CHANNEL2_EQ_FILTER_CONTROL_2, 0x00 },
	{ SPEQ_FILTER_CONTROL_1, 0x00 },
	{ SPEQ_FILTER_CONTROL_2, 0x00 },
	{ SPEQ_FILTER_CONTROL_3, 0x00 },
	{ BIST_CONTROL, 0x00 },
	{ PLL_CONTROL, 0x0000 },
	{ SPK_SEQUENCE_BYPASS, 0x00 },
	{ FUNC_TEST, 0x80 },
	{ TM_BY_REG, 0x00 },
	{ PROTECTION_SYSTEM_CONTROL, 0x1F },
	{ I2C_CONTROL, 0x03 },
	{ OSCILLATOR_TRIM_CONTROL, 0x01 },
	{ DRC_FTUNE, 0x20 },
	{ ERROR_DC_STATUS, 0x00 },
	{ DSP_CONTROL_3, 0xB0 },
	{ FUNC_DEBUG, 0xC8 },

};
static const struct reg_default sy24145_reg_defaults_16[] = {
	{ PRESCALER, 0x7FFF },
	{ POSTSCALER, 0x7FFF },
	{ AUTO_MUTE_THRESHOLD, 0x0000 },
};

static const struct reg_default sy24145_reg_defaults_24[] = {
	{ DRC1_LMT_CFG1, 0x3CC30C },
	{ DRC1_LMT_CFG2, 0x060F83 },
	{ DRC1_LMT_CFG3, 0x000122 },
	{ DRC2_LMT_CFG1, 0x3CC30C },
	{ DRC2_LMT_CFG2, 0x060F83 },
	{ DRC2_LMT_CFG3, 0x000122 },
	{ DRC3_LMT_CFG1, 0x3CC30C },
	{ DRC3_LMT_CFG2, 0x060F83 },
	{ DRC3_LMT_CFG3, 0x000122 },
	{ DRC4_LMT_CFG1, 0x3CC30C },
	{ DRC4_LMT_CFG2, 0x060F83 },
	{ DRC4_LMT_CFG3, 0x000122 },
	{ DRC_ENVLP_TC_UP, 0x010000 },
	{ DRC_ENVLP_TC_DN, 0x7B0000 },
	{ HARD_CLIPPER_THR, 0x7FFFFF },
	{ DSP_3D_COEF, 0x400000 },
	{ DSP_3D_MIX, 0x400000 },
	{ DRC1_ENVLP_TC_UP, 0x010000 },
	{ DRC1_ENVLP_TC_DN, 0x7B0000 },
	{ DRC2_ENVLP_TC_UP, 0x010000 },
	{ DRC2_ENVLP_TC_DN, 0x7B0000 },
	{ DRC3_ENVLP_TC_UP, 0x010000 },
	{ DRC3_ENVLP_TC_DN, 0x7B0000 },
	{ POWER_METER_CONTROL_RB1, 0x000000 },
	{ POWER_METER_CONTROL_RB2, 0x000000 },
};

static const struct reg_default sy24145_reg_defaults_32[] = {
	{ DRC_CONTROL, 0x01000000 },
	{ PLL_STATUS, 0x0063002D },
	{ OSCILLATOR_TRIM_REGISTER1, 0x00001000 },
	{ OSCILLATOR_TRIM_REGISTER2, 0x00101017 },
	{ ANALOG_REF_TOP_CONTROL, 0x00000200 },
	{ INTER_PRIVATE, 0x000000F0 },
	{ OC_DETECT_WINDOW_WIDTH, 0x00000006 },
	{ FAULT_OVER_CURRENT_THRESHOLD, 0x00002006 },
	{ PWM_MUX, 0x00000000 },
	{ PWM_OUTFLIP_1, 0x40003210 },
	{ PWM_OUTFLIP_2, 0x1000002F },
	{ PBQ_CHECKSUM, 0x00000000 },
	{ MDRC_CHECKSUM, 0x40000000 },
	{ PBQ_CH2_CHECKSUM, 0x00000000 },
};

static bool sy24145_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CLOCK_CONTROL ... VOL_FTUNE:
	case SOFT_RESET ... PWM_DIRECT_CURRENT_THRESHOLD:
	case SHORT_CONTROL ... FAULT_OUTPUT_TIME:
	case OPERATION_MODE ... FAULT_SELECT:
	case CHANNEL1_EQ_FILTER_CONTROL_1 ... POSTSCALER:
	case BQ0 ... CHANNEL12_LOUDNESS:
	case SPEQ_ATK_REL_TC_1 ... HARD_CLIPPER_THR:
	case OSCILLATOR_TRIM_CONTROL ... ANALOG_REF_TOP_CONTROL:
	case DSP_3D_COEF ... DRC_FTUNE:
	case OC_DETECT_WINDOW_WIDTH ... FAULT_OVER_CURRENT_THRESHOLD:
	case ERROR_DC_STATUS:
	case DSP_CONTROL_3 ... FUNC_DEBUG:
	case DRC1_ENVLP_TC_UP ... PBQ_CH2_CHECKSUM:
		return true;
	default:
		return false;
	}
}

static bool sy24145_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CLOCK_CONTROL:
	case ERROR_STATUS ... CHANNEL2_VOLUME:
	case VOL_FTUNE:
	case SOFT_RESET ... PWM_DIRECT_CURRENT_THRESHOLD:
	case SHORT_CONTROL ... FAULT_OUTPUT_TIME:
	case OPERATION_MODE ... FAULT_SELECT:
	case CHANNEL1_EQ_FILTER_CONTROL_1 ... POSTSCALER:
	case BQ0 ... CHANNEL12_LOUDNESS:
	case SPEQ_ATK_REL_TC_1 ... HARD_CLIPPER_THR:
	case OSCILLATOR_TRIM_CONTROL ... ANALOG_REF_TOP_CONTROL:
	case DSP_3D_COEF ... DRC_FTUNE:
	case OC_DETECT_WINDOW_WIDTH ... FAULT_OVER_CURRENT_THRESHOLD:
	case DSP_CONTROL_3 ... FUNC_DEBUG:
	case DRC1_ENVLP_TC_UP ... POWER_METER_CONTROL_RB1:
	case PBQ_CHECKSUM ... PBQ_CH2_CHECKSUM:
		return true;
	default:
		return false;
	}
}

static const DECLARE_TLV_DB_SCALE(sy24145_vol_tlv_master, -12600, 50, 0);
static const DECLARE_TLV_DB_SCALE(sy24145_vol_tlv_channels, -7900, 50, 0);

//Mute and Soft Volume Change
//The chip enters mute state by setting soft mute flag of register Address 0x06. 0x06[3] is master mute flag for both left
//channel and right channel. 0x06[0] is individually mute flag for left channel while 0x06[1] is individually mute flag for
//right channel. With soft mute, the volume gradually increases or decreases when mute is turned off or on respectively.
// Channel 1 - left, Channel 2 - right

static const struct snd_kcontrol_new sy24145_controls[] = {

	// // Soft mute register (0x06)
	SOC_SINGLE("Channel 1 soft mute", SOFT_MUTE, 0, 1,
		   SY24145_NO_INVERT), // DDMLR
	SOC_SINGLE("Channel 2 soft mute", SOFT_MUTE, 1, 1,
		   SY24145_NO_INVERT), // DDMRR

	// Master volume(0x07)
	SOC_SINGLE_RANGE_TLV("Master volume", MASTER_VOLUME, 0, 0x3, 0xff,
			     SY24145_NO_INVERT, sy24145_vol_tlv_master),

	// Channel 1(0x08) and 2(0x09) volume
	SOC_SINGLE_RANGE_TLV("Left volume", CHANNEL1_VOLUME, 0, 0x1, 0xFF,
			     SY24145_NO_INVERT, sy24145_vol_tlv_channels),
	SOC_SINGLE_RANGE_TLV("Right volume", CHANNEL2_VOLUME, 0, 0x1, 0xFF,
			     SY24145_NO_INVERT, sy24145_vol_tlv_channels),
};

static const struct snd_soc_dapm_widget sy24145_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("OUTL"),
	SND_SOC_DAPM_OUTPUT("OUTR"),
};

static const struct snd_soc_dapm_route sy24145_routes[] = {
	{ "OUTL", NULL, "Playback" },
	{ "OUTR", NULL, "Playback" },

};

static void sy24145_set_configuration_settings(struct sy24145 *sy24145)
{
	int ret = 0;
	ret = regmap_update_bits(sy24145->regmap, SOFT_MUTE, HARD_SOFT_UNMUTE_MASK, SOFT_UNMUTE_FROM_CLK_ERR);
	ret = regmap_update_bits(sy24145->regmap, SYSTEM_CONTROL_2,
				 LOUDNESS_EN_MASK,
				 LOUDNESS_EN); // Enable loudness
	ret = regmap_update_bits(
		sy24145->regmap, DRC_CONTROL, 15,
		0xF); // DRC Control: enable drc1, drc2, drc3, drc4
	ret = regmap_update_bits(sy24145->regmap, MASTER_VOLUME,
				 MASTER_VOLUME_MASK,
				 sy24145->mstr_volume); // Master volume
	ret = regmap_update_bits(sy24145->regmap, CHANNEL1_VOLUME,
				 CHANNEL_VOLUME_MASK,
				 sy24145->l_volume); // Left channel volume
	ret = regmap_update_bits(sy24145->regmap, CHANNEL2_VOLUME,
				 CHANNEL_VOLUME_MASK,
				 sy24145->r_volume); // Right channel volume
	ret = regmap_update_bits(sy24145->regmap, SOFT_MUTE,
				 DSP_DVOL_MUTE_LEFT_MASK,
				 (sy24145->l_mute == true) ?
					 DSP_DVOL_MUTE_LEFT :
					 DSP_DVOL_UNMUTE_LEFT);
	ret = regmap_update_bits(sy24145->regmap, SOFT_MUTE,
				 DSP_DVOL_MUTE_RIGHT_MASK,
				 (sy24145->r_mute == true) ?
					 DSP_DVOL_MUTE_RIGHT :
					 DSP_DVOL_UNMUTE_RIGHT);
	ret = regmap_update_bits(
		sy24145->regmap, PWM_CONTROL,
		PWM_CONTROL_STANDBY_MASK | PWM_CONTROL_SHUTDOWN_MASK,
		PWM_CONTROL_STANDBY_EXIT |
			PWM_CONTROL_SHUTDOWN_EXIT); // PWM Control: exit all-channel standby, exit all-channel shutdown
}

static void __attribute__((unused)) sy24145_print_reg(struct sy24145 *sy24145)
{
	int ret = 0;
	unsigned int reg_val = 0;

	printk("\tsy24145 registers\n");

	ret = regmap_read(sy24145->regmap, CLOCK_CONTROL, &reg_val);
	if (ret == 0)
		printk("sy24145 CLOCK_CONTROL(0x00): %d\n", reg_val);

	ret = regmap_read(sy24145->regmap, DEVICE_ID, &reg_val);
	if (ret == 0)
		printk("sy24145 DEVICE_ID(0x01): 0x%x\n", reg_val);

	ret = regmap_read(sy24145->regmap, SYSTEM_CONTROL_2, &reg_val);
	if (ret == 0)
		printk("sy24145 SYSTEM_CONTROL_2(0x04): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, SOFT_MUTE, &reg_val);
	if (ret == 0)
		printk("sy24145 SOFT_MUTE(0x06): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, MASTER_VOLUME, &reg_val);
	if (ret == 0)
		printk("sy24145 MASTER_VOLUME(0x07): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, I2S_CONTROL, &reg_val);
	printk("sy24145 I2S_CONTROL(0x15): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, MONITOR_PIN_CONFIGURED_1, &reg_val);
	if (ret == 0)
		if (ret == 0)
			printk("sy24145 MONITOR_PIN_CONFIGURED_1(0x17): %u\n",
			       reg_val);

	ret = regmap_read(sy24145->regmap, PWM_CONTROL, &reg_val);
	if (ret == 0)
		printk("sy24145 PWM_CONTROL(0x22): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, PRESCALER, &reg_val);
	if (ret == 0)
		printk("sy24145 PRESCALER(0x2C): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, POSTSCALER, &reg_val);
	if (ret == 0)
		printk("sy24145 POSTSCALER(0x2D): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, DRC_CONTROL, &reg_val);
	if (ret == 0)
		printk("sy24145 DRC_CONTROL(0x60): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, PLL_STATUS, &reg_val);
	if (ret == 0)
		printk("sy24145 PLL_STATUS(0x71): %u\n", reg_val);

	ret = regmap_read(sy24145->regmap, PLL_CONTROL, &reg_val);
	if (ret == 0)
		printk("sy24145 PLL_CONTROL(0x72): %u\n", reg_val);
}

static void __attribute__((unused))
sy24145_print_errors(struct sy24145 *sy24145)
{
	uint32_t err = 0;
	regmap_read(sy24145->regmap, ERROR_STATUS, &err);
	printk("sy24145 ERROR_STATUS(0x02): %u\n", err);
	if ((err & ERROR_STATUS_OTF) != 0)
		printk("sy24145: Over temperature or under voltage is detected\n");
	if ((err & ERROR_STATUS_OCF) != 0)
		printk("sy24145: Over current is detected\n");
	if ((err & ERROR_STATUS_SF) != 0)
		printk("sy24145: Short is detected\n");
	if ((err & ERROR_STATUS_PWM_DE) != 0)
		printk("sy24145: PWM DC detected\n");
	if ((err & ERROR_STATUS_LRCLKE) != 0)
		printk("sy24145: LRCLK error\n");
	if ((err & ERROR_STATUS_SCLKE) != 0)
		printk("sy24145: SCLK error\n");
	if ((err & ERROR_STATUS_DRC_CE) != 0)
		printk("sy24145: DRC checksum error\n");
	if ((err & ERROR_STATUS_PCE) != 0)
		printk("sy24145: BQ checksum error\n");

	regmap_read(sy24145->regmap, ERROR_STATUS_2, &err);
	printk("sy24145 ERROR_STATUS_2(0x0A): %u\n", err);
	if ((err & ERROR_STATUS_SLEF) != 0)
		printk("sy24145: Short load error\n");
	if ((err & ERROR_STATUS_OLEF) != 0)
		printk("sy24145: Open load error\n");

	regmap_read(sy24145->regmap, ERROR_DC_STATUS, &err);
	printk("sy24145 ERROR_DC_STATUS(0x89): %u\n", err);
	if ((err & ERROR_STATUS_PPEC2) != 0)
		printk("sy24145: Channel 2 has an p-side DC error\n");
	if ((err & ERROR_STATUS_PNEC2) != 0)
		printk("sy24145: Channel 2 has an n-side DC error\n");
	if ((err & ERROR_STATUS_PPEC1) != 0)
		printk("sy24145: Channel 1 has an p-side DC error\n");
	if ((err & ERROR_STATUS_PNEC1) != 0)
		printk("sy24145: Channel 1 has an n-side DC error\n");
}

static int sy24145_parse_dt_property(struct i2c_client *i2c,
				     struct sy24145 *sy24145)
{
	const struct device_node *np = i2c->dev.parent->of_node;
	const struct device *dev_parent = i2c->dev.parent;
	u32 val = 0;

	if (!np) {
		dev_err(dev_parent, "%s() i2c->dev.parent->of_node is NULL\n",
			__func__);
		return -ENODEV;
	}

	np = of_get_child_by_name(i2c->dev.parent->of_node, "sy24145");
	if (!np) {
		dev_err(dev_parent, "%s() Can not get child: sy24145\n",
			__func__);
		return -ENODEV;
	}

	if (of_property_read_u32(np, "master-volume", &val) == 0) {
		if (val > 0xFF)
			val = 0xFF;
		sy24145->mstr_volume = val;
	} else {
		printk("Can not read property master-volume\n");
		sy24145->mstr_volume = 0xFF;
	}

	if (of_property_read_u32(np, "left-ch-volume", &val) == 0) {
		if (val > 0xFF)
			val = 0xFF;
		sy24145->l_volume = val;
	} else {
		printk("Can not read property left-ch-volume\n");
		sy24145->l_volume = 0x7F;
	}

	if (of_property_read_u32(np, "right-ch-volume", &val) == 0) {
		if (val > 0xFF)
			val = 0xFF;
		sy24145->r_volume = val;
	} else {
		printk("Can not read property right-ch-volume\n");
		sy24145->r_volume = 0x7F;
	}

	sy24145->l_mute = of_property_read_bool(np, "left-ch-mute");
	sy24145->r_mute = of_property_read_bool(np, "right-ch-mute");

	return 0;
}

static int sy24145_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sy24145 *sy24145 = snd_soc_component_get_drvdata(component);
	unsigned int sample_rate = 0;
	unsigned int brt_sel = 0;
	unsigned int brt_sel_flag = 0;
	unsigned int reg_val = 0;
	unsigned int val_len = 0;

	switch (params_rate(params)) {
	case 32000:
		sample_rate = FS_RATE_CNFG_32kHZ;
		sample_rate_g = 32000;
		break;
	case 96000:
		sample_rate = FS_RATE_CNFG_96kHZ;
		sample_rate_g = 96000;
		break;
	case 44100:
		sample_rate = FS_RATE_CNFG_441_48kHZ;
		brt_sel = BRT_SEL_441kHZ;
		brt_sel_flag = 1;
		sample_rate_g = 44100;
		break;
	case 48000:
		sample_rate = FS_RATE_CNFG_441_48kHZ;
		brt_sel = BRT_SEL_48kHZ;
		brt_sel_flag = 1;
		sample_rate_g = 48000;
		break;
	default:
		sample_rate_g = 0;
		return -EINVAL;
	}

	regmap_update_bits(sy24145->regmap, CLOCK_CONTROL,
			   FS_CNFG_MANUAL_EN_MASK, FS_CNFG_MANUAL_EN_CONFIG_SR);

	if (brt_sel_flag == 1)
		regmap_update_bits(sy24145->regmap, CLOCK_CONTROL, BRT_SEL_MASK,
				   brt_sel);
	regmap_update_bits(sy24145->regmap, CLOCK_CONTROL, FS_RATE_CNFG_MASK,
			   sample_rate);

	regmap_read(sy24145->regmap, CLOCK_CONTROL, &reg_val);

	switch (params_width(params)) {
	case 16:
		val_len = I2S_VBITS_16;
		break;
	case 18:
		val_len = I2S_VBITS_18;
		break;
	case 20:
		val_len = I2S_VBITS_20;
		break;
	case 24:
		val_len = I2S_VBITS_24;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(sy24145->regmap, I2S_CONTROL, I2S_VBITS_MASK,
			   val_len);
	return 0;
}

static int sy24145_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	struct sy24145 *sy24145 = snd_soc_component_get_drvdata(component);
	unsigned int sclk = 0;
	unsigned int lrclk = 0;
	unsigned int format = 0;

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		sclk = I2S_SCLK_NOT_INVERT;
		lrclk = I2S_LR_POLARITY_NOT_INVERT;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		sclk = I2S_SCLK_NOT_INVERT;
		lrclk = I2S_LR_POLARITY_INVERT;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		sclk = I2S_SCLK_INVERT;
		lrclk = I2S_LR_POLARITY_NOT_INVERT;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		sclk = I2S_SCLK_INVERT;
		lrclk = I2S_LR_POLARITY_INVERT;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(sy24145->regmap, I2S_CONTROL,
			   I2S_LR_POLARITY_MASK | I2S_SCLK_INV_MASK,
			   sclk | lrclk);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format = I2S_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format = I2S_FMT_LJ;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		format = I2S_FMT_RJ;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(sy24145->regmap, I2S_CONTROL,
			   I2S_FMT_MASK | I2S_LR_POLARITY_MASK |
				   I2S_SCLK_INV_MASK,
			   format | sclk | lrclk);
	return 0;
}

static int sy24145_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct sy24145 *sy24145 = snd_soc_component_get_drvdata(component);
	unsigned int val = 0;

	val = (mute > 0) ? DSP_MVOL_MUTE : DSP_MVOL_UNMUTE;

	return regmap_update_bits(sy24145->regmap, SOFT_MUTE, DSP_MVOL_MASK,
				  val);
}

static const struct snd_soc_dai_ops sy24145_dai_ops = {
	.hw_params = sy24145_hw_params,
	.set_fmt = sy24145_set_dai_fmt,
	.mute_stream = sy24145_mute_stream,
};

#define SY24145_RATES                                                        \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_96000
#define SY24145_FORMATS                                        \
	(SNDRV_PCM_FMTBIT_U16_BE | SNDRV_PCM_FMTBIT_S16_BE |   \
	 SNDRV_PCM_FMTBIT_U18_3BE | SNDRV_PCM_FMTBIT_S18_3BE | \
	 SNDRV_PCM_FMTBIT_U20_3BE | SNDRV_PCM_FMTBIT_S20_3BE | \
	 SNDRV_PCM_FMTBIT_U24_BE | SNDRV_PCM_FMTBIT_S24_BE)

static struct snd_soc_dai_driver sy24145_dai = {
	.name = "sy24145-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SY24145_RATES,
		.formats = SY24145_FORMATS,
	},
	.ops = &sy24145_dai_ops,
};

static const struct snd_soc_component_driver sy24145_component_driver = {
	.controls = sy24145_controls,
	.num_controls = ARRAY_SIZE(sy24145_controls),
	.dapm_widgets = sy24145_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sy24145_dapm_widgets),
	.dapm_routes = sy24145_routes,
	.num_dapm_routes = ARRAY_SIZE(sy24145_routes),
	.use_pmdown_time = 1,
	.endianness = 1,
	.legacy_dai_naming = 1
};

static const struct regmap_config sy24145_regmap_config[] = {

	{
		.name = "#8",
		.reg_bits = 8,
		.val_bits = 8,
		.cache_type = REGCACHE_RBTREE,
		.reg_defaults = sy24145_reg_defaults_8,
		.num_reg_defaults = ARRAY_SIZE(sy24145_reg_defaults_8),
		.readable_reg = sy24145_readable_reg,
		.writeable_reg = sy24145_writeable_reg,
	},
	{
		.name = "#16",
		.reg_bits = 8,
		.val_bits = 16,
		.cache_type = REGCACHE_RBTREE,
		.reg_defaults = sy24145_reg_defaults_16,
		.num_reg_defaults = ARRAY_SIZE(sy24145_reg_defaults_16),
		.readable_reg = sy24145_readable_reg,
		.writeable_reg = sy24145_writeable_reg,
	},
	{
		.name = "#24",
		.reg_bits = 8,
		.val_bits = 24,
		.cache_type = REGCACHE_RBTREE,
		.reg_defaults = sy24145_reg_defaults_24,
		.num_reg_defaults = ARRAY_SIZE(sy24145_reg_defaults_24),
		.readable_reg = sy24145_readable_reg,
		.writeable_reg = sy24145_writeable_reg,
	},
	{
		.name = "#32",
		.reg_bits = 8,
		.val_bits = 32,
		.cache_type = REGCACHE_RBTREE,
		.reg_defaults = sy24145_reg_defaults_32,
		.num_reg_defaults = ARRAY_SIZE(sy24145_reg_defaults_32),
		.readable_reg = sy24145_readable_reg,
		.writeable_reg = sy24145_writeable_reg,
	}

};

static const struct i2c_device_id sy24145_id[] = {
	{ "sy24145", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sy24145_id);

#ifdef CONFIG_OF
static const struct of_device_id sy24145_of_ids[] = {
	{
		.compatible = "silergy,sy24145",
	},
	{}
};
MODULE_DEVICE_TABLE(of, sy24145_of_ids);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id sy24145_acpi_match[] = {
	{ "SYL24145", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, sy24145_acpi_match);
#endif

static int sy24145_i2c_read(struct i2c_client *client, uint8_t reg, uint8_t len,
			    uint8_t *val)
{
	__u8 _reg = reg;
	__u16 _len = len;

	uint8_t read_buf[20] = { 0 };
	int err = 0;

	struct i2c_msg msgs[] = {
		{
			/* register number */
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(_reg),
			.buf = &_reg,
		},
		{
			/* read register contents */
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = _len,
			.buf = (__u8 *)read_buf,
		},

	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err < 0) {
		dev_err(&client->dev, "Error during reading reg 0x%X\n", _reg);
		return err;
	}

	memcpy(val, read_buf, _len);

	dev_info(&client->dev, "Read register 0x%X complited successfully\n",
		 _reg);

	return 0;
}

static int sy24145_i2c_write(struct i2c_client *client, uint8_t reg,
			     uint8_t len, uint8_t *val)
{
	__u8 _reg = reg;
	__u16 _len = len;

	uint8_t write_buf[21] = { 0 };
	int err = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = _len + 1,
			.buf = write_buf,
		},
	};

	write_buf[0] = _reg;
	// Старшими битами вперед
	memcpy(write_buf + 1, val, _len);

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err < 0) {
		dev_err(&client->dev, "Error during writing to reg 0x%X\n",
			_reg);
		return err;
	}

	dev_info(&client->dev,
		 "Write to register 0x%X complited successfully\n", _reg);

	return 0;
}

static int strToU8(char *str, int len, uint8_t *num)
{
	int multiplier = 1;
	uint8_t ret = 0;

	for (int i = len - 1; i >= 0; --i) {
		if (((str[i] >= 48) && (str[i] <= 57)) || (str[i] == '\n')) {
			if (str[i] != '\n') {
				ret += (str[i] - 48) * multiplier;
				multiplier *= 10;
			}
		} else
			return -1;
	}

	*num = ret;
	return 0;
}

static int getU8ArrFromString(const char *buf, size_t count, uint8_t *u8arr,
			      const uint8_t size)
{
	char *split_str[size];
	char *isStr;
	char *input = (char *)kmalloc(count, GFP_KERNEL);
	int counter = 0;
	int ret = 0;
	uint8_t num = 0;
	strcpy(input, buf);
	isStr = strsep(&input, " ");

	while (isStr != NULL) {
		if (counter >= size) {
			for (int i = 0; i < size; ++i)
				kfree(split_str[i]);
			kfree(input);
			return -2;
		}
		split_str[counter] = (char *)kmalloc(strlen(isStr), GFP_KERNEL);
		strcpy(split_str[counter], isStr);
		isStr = strsep(&input, " ");
		counter++;
	}

	if (counter != size) {
		for (int i = 0; i < counter; ++i)
			kfree(split_str[i]);
		kfree(input);
		return -2;
	}

	for (int i = 0; i < size; ++i) {
		if (strToU8(split_str[i], strlen(split_str[i]), &num) == 0) {
			u8arr[i] = num;
		} else {
			for (int i = 0; i < size; ++i)
				kfree(split_str[i]);
			kfree(input);
			return -1;
		}
	}

	for (int i = 0; i < size; ++i)
		kfree(split_str[i]);
	kfree(input);
	return 0;
}

static int reverseArr(uint8_t *arr, const int size)
{
	for (int i = 0; i < size / 2; ++i) {
		uint8_t temp = arr[i];
		arr[i] = arr[size - 1 - i];
		arr[size - 1 - i] = temp;
	}
	return 0;
}

ssize_t sy24145_sys_show_sample_rate(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sy24145 *sy24145 = dev_get_drvdata(dev);
	int ret = 0;
	ret = sprintf(buf, "%u\n", sample_rate_g);
	return ret;
}

static DEVICE_ATTR(sample_rate, S_IRUSR, sy24145_sys_show_sample_rate, NULL);

ssize_t sy24145_sys_show_master_volume(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct sy24145 *sy24145 = dev_get_drvdata(dev);
	uint32_t master_volume = 0;
	int32_t retVolume = 0;
	uint8_t reg = MASTER_VOLUME;
	int ret = 0;

	ret = regmap_read(sy24145->regmap, reg, &master_volume);
	if (ret == 0) {
		if ((master_volume >= 0) && (master_volume <= 2))
			ret = sprintf(buf, "0\n");
		else {
			retVolume = -252 + master_volume - 0x3;
			ret = sprintf(buf, "%d\n", retVolume);
		}
	} else
		ret = sprintf(buf, "1\n");

	return ret;
}

static DEVICE_ATTR(master_volume, S_IRUSR, sy24145_sys_show_master_volume,
		   NULL);

static struct attribute *sy24145_attributes_sample_rate[] = {
	&dev_attr_sample_rate.attr,
	NULL,
};

static struct attribute *sy24145_attributes_master_volume[] = {
	&dev_attr_master_volume.attr,
	NULL,
};

static const struct attribute_group sy24145_sample_rate_group = {
	.attrs = sy24145_attributes_sample_rate,
};

static const struct attribute_group sy24145_master_volume_group = {
	.attrs = sy24145_attributes_master_volume,
};

static const struct attribute_group *sy24145_groups[] = {
	&sy24145_sample_rate_group,
	&sy24145_master_volume_group,
	NULL,
};

static int sy24145_i2c_probe(struct i2c_client *i2c)
{
	struct sy24145 *sy24145;
	int ret = 0;
	int dev_id = 0;

	sy24145 = devm_kzalloc(&i2c->dev, sizeof(*sy24145), GFP_KERNEL);
	if (sy24145 == NULL)
		return -ENOMEM;

	sy24145->client = i2c;

	i2c_set_clientdata(i2c, sy24145);

	sy24145->regmap = devm_regmap_init_i2c(i2c, sy24145_regmap_config);
	if (IS_ERR(sy24145->regmap))
		return PTR_ERR(sy24145->regmap);

	ret = regmap_read(sy24145->regmap, DEVICE_ID, &dev_id);
	if (ret == 0)
		dev_info(&i2c->dev, "sy24145 device id = 0x%x", dev_id);
	ret = devm_snd_soc_register_component(
		&i2c->dev, &sy24145_component_driver, &sy24145_dai, 1);

	ret = sy24145_parse_dt_property(i2c, sy24145);

	sy24145_set_configuration_settings(sy24145);

	ret = sysfs_create_groups(&i2c->dev.kobj, sy24145_groups);
	if (ret < 0)
		dev_err(&i2c->dev, "Failed to create sysfs group, %d\n", ret);
	sample_rate_g = 44100;
	return ret;
}

static struct i2c_driver sy24145_driver = {
	.driver		= {
		.name	= "sy24145",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(sy24145_of_ids),
		.acpi_match_table = ACPI_PTR(sy24145_acpi_match),
	},
	.probe		= sy24145_i2c_probe,
	.id_table   = sy24145_id,
};

module_i2c_driver(sy24145_driver);

MODULE_DESCRIPTION("sy24145 device driver");
MODULE_LICENSE("GPL");