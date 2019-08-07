/*
 * A V4L2 driver for OV5647 sensor on Jetson Nano.
 *
 */
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#define CREATE_TRACE_POINTS

#include "camera_gpio.h"
#include "ov5647.h"
#include "ov5647_modes_tbls.h"

//for internel driver debug
#define DEV_DBG_EN      0 
#if(DEV_DBG_EN == 1)    
#define vfe_dev_dbg(x,arg...) printk("[OV5647]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[OV5647]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[OV5647]"x,##arg)

#define LOG_ERR_RET(x)  { \
                          int ret;  \
                          ret = x; \
                          if(ret < 0) {\
                            vfe_dev_err("error at %s\n",__func__);  \
                            return ret; \
                          } \
                        }

//define module timing
#define MCLK              (24*1000*1000)
#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_LOW
#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR 0x5647

//#define QSXGA_12FPS

//define the voltage level of control signal
#define CSI_STBY_ON     1
#define CSI_STBY_OFF    0
#define CSI_RST_ON      0
#define CSI_RST_OFF     1
#define CSI_PWR_ON      1
#define CSI_PWR_OFF     0
#define CSI_AF_PWR_ON   1
#define CSI_AF_PWR_OFF  0
#define regval_list reg_list_a16_d8


#define REG_TERM 0xfffe
#define VAL_TERM 0xfe
#define REG_DLY  0xffff

#define OV5647_MAX_FRAME_LENGTH        (0x7fff)
#define OV5647_DEFAULT_FRAME_LENGTH    (1944)
#define OV5647_DEFAULT_PIXEL_CLOCK     (160)

#define OV5647_DEFAULT_LINE_LENGTH     (2592)

#define OV5647_GAIN_ADDR_MSB			0x350A
#define OV5647_GAIN_ADDR_LSB			0x350B
#define OV5647_GROUP_HOLD_ADDR			0x3208
#define OV5647_TIMING_REG20			    0x3820
#define OV5647_TIMING_REG21			    0x3821
#define VERTICAL_FLIP				((0x1 << 1) | (0x1 << 6))
#define HORIZONTAL_MIRROR_MASK			(0x3 << 1)

#define OV5647_REG_CHIPID_H             0x300A
#define OV5647_REG_CHIPID_L             0x300B

#define OV5647_TABLE_WAIT_MS	0
#define OV5647_TABLE_END	1
#define OV5647_MAX_RETRIES	3
#define OV5647_WAIT_MS		10  // goto line 1600

//sensor_write(sd, 0x350b, gainlow);
//sensor_write(sd, 0x350a, gainhigh);

#define OV5647_TABLE_END	1
/*
 * Our nominal (default) frame rate.
 */
#ifdef FPGA
#define SENSOR_FRAME_RATE 15
#else
#define SENSOR_FRAME_RATE 30
#endif

/*
 * The ov5647 sits on i2c with ID 0x6c
 */
#define I2C_ADDR 0x6c
#define  OV5647_SENSOR_NAME "ov5647"

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct ov5647 {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;
	struct mutex			lock;
	struct v4l2_mbus_framefmt	format;
	unsigned int			width;
	unsigned int			height;
	int				        power_count;
	struct clk			    *xclk;

	s32				group_hold_prev;
	u32				frame_length;
	bool			group_hold_en;
	
	struct camera_common_data	*s_data;
	struct camera_common_i2c	i2c_dev;
	struct tegracam_device		*tc_dev;
	
	struct gpio_desc		*pwdn;
	unsigned int			flags;

	struct mutex			streaming_lock;
	bool	    			streaming;
	
};


/*
 * Information we maintain about a known sensor.
 */
struct cfg_array { /* coming later */
	struct regval_list * regs;
	int size;
};

/*
 * Stuff that knows about the sensor.
 */
 
static void ov5647_gpio_set(struct camera_common_data *s_data,
			    unsigned int gpio, int val)
{
	struct camera_common_pdata *pdata = s_data->pdata;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_ctrl(s_data->dev, gpio, val, 1);
	else {
		if (gpio_cansleep(gpio))
			gpio_set_value_cansleep(gpio, val);
		else
			gpio_set_value(gpio, val);
	}
}
  
static int ov5647_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err;
	err = regmap_write(s_data->regmap, addr, val);

	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x\n", __func__, addr, val);

	return err;
}
static inline int ov5647_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int ov5647_write_table(struct ov5647 *priv, const ov5647_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;
	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 OV5647_TABLE_WAIT_MS,
					 OV5647_TABLE_END);
}

static inline void ov5647_get_frame_length_regs(ov5647_reg *regs,
				u32 frame_length)
{
	regs->addr = OV5647_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = OV5647_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void ov5647_get_coarse_time_regs(ov5647_reg *regs,
				u32 coarse_time)
{
	regs->addr = OV5647_COARSE_TIME_ADDR_1;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = OV5647_COARSE_TIME_ADDR_2;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = OV5647_COARSE_TIME_ADDR_3;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

static int ov5647_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	int err;
	struct ov5647 *priv = tc_dev->priv;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];
	struct device *dev = tc_dev->dev;

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		camera_common_i2c_aggregate(&priv->i2c_dev, true);
		/* enter group hold */
		err = ov5647_write_reg(priv->s_data,
				       OV5647_GROUP_HOLD_ADDR, val);
		if (err)
			goto fail;

		priv->group_hold_prev = 1;

		dev_dbg(dev, "%s: enter group hold\n", __func__);
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		/* leave group hold */
		err = ov5647_write_reg(priv->s_data,
				       OV5647_GROUP_HOLD_ADDR, 0x11);
		if (err)
			goto fail;

		err = ov5647_write_reg(priv->s_data,
				       OV5647_GROUP_HOLD_ADDR, 0x61);
		if (err)
			goto fail;

		camera_common_i2c_aggregate(&priv->i2c_dev, false);

		priv->group_hold_prev = 0;

		dev_dbg(dev, "%s: leave group hold\n", __func__);
	}

	return 0;

fail:
	dev_dbg(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static inline void ov5647_get_gain_regs(ov5647_reg *regs, u16 gain)
{
  //gainlow=(unsigned char)(gain_val&0xff);
  //gainhigh=(unsigned char)((gain_val>>8)&0x3);
	regs->addr = OV5647_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0x3;

	(regs + 1)->addr = OV5647_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xFF;
}

static int ov5647_set_gain(struct tegracam_device *tc_dev, s64 val);
static int ov5647_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int ov5647_set_exposure(struct tegracam_device *tc_dev, s64 val);
static void ov5647_update_ctrl_range(struct camera_common_data *s_data, s32 frame_length);

static int ov5647_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ov5647 *priv = (struct ov5647 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5647_reg reg_list[2];
	int err;
	u16 gain;
	int i;

	if (!priv->group_hold_prev)
		ov5647_set_group_hold(tc_dev, 1);

	/* translate value */
	gain = (u16) (((val * 16) +
			(mode->control_properties.gain_factor / 2)) /
			mode->control_properties.gain_factor);
	ov5647_get_gain_regs(reg_list, gain);
	dev_dbg(dev, "%s: gain %d val: %lld\n", __func__, gain, val);

	for (i = 0; i < 2; i++) {
		err = ov5647_write_reg(s_data, reg_list[i].addr, reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static void ov5647_update_ctrl_range(struct camera_common_data *s_data, s32 frame_length)
{

  //sensor_write(sd, 0x380f, (frame_length & 0xff));
	 ov5647_write_reg(s_data, 0x380f, (frame_length & 0xff) );
  //sensor_write(sd, 0x380e, (frame_length >> 8));
	 ov5647_write_reg(s_data, 0x380e, (frame_length >> 8) );

}

static int ov5647_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5647 *priv = tc_dev->priv;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5647_reg reg_list[2];
	int err;
	u32 frame_length;
	int i;

	if (!priv->group_hold_prev)
		ov5647_set_group_hold(tc_dev, 1);

	frame_length =  mode->signal_properties.pixel_clock.val *
		mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val;

	ov5647_get_frame_length_regs(reg_list, frame_length);
	dev_dbg(dev, "%s: val: %d\n", __func__, frame_length);

	for (i = 0; i < 2; i++) {
		err = ov5647_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	priv->frame_length = frame_length;

	return 0;

fail:
	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int ov5647_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5647 *priv = tc_dev->priv;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5647_reg reg_list[3];
	int err;
	u32 coarse_time;
	int i;

	if (!priv->group_hold_prev)
		ov5647_set_group_hold(tc_dev, 1);

	coarse_time = (u32)(((mode->signal_properties.pixel_clock.val*val)
			/mode->image_properties.line_length)/
			mode->control_properties.exposure_factor);

	ov5647_get_coarse_time_regs(reg_list, coarse_time);
	dev_dbg(dev, "%s: val: %d\n", __func__, coarse_time);

	for (i = 0; i < 3; i++) {
		err = ov5647_write_reg(s_data, reg_list[i].addr, reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: COARSE_TIME control error\n", __func__);
	return err;
}

static struct tegracam_ctrl_ops ov5647_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ov5647_set_gain,
	.set_exposure = ov5647_set_exposure,
	.set_frame_rate = ov5647_set_frame_rate,
	.set_group_hold = ov5647_set_group_hold,
};
#if 0
static struct tegracam_ctrl_ops ov5693_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = {OV5693_EEPROM_STR_SIZE,
				OV5693_FUSE_ID_STR_SIZE,
				OV5693_OTP_STR_SIZE},
	.set_gain = ov5693_set_gain,
	.set_exposure = ov5693_set_exposure,
	.set_exposure_short = ov5693_set_exposure_short, <<<< 
	.set_frame_rate = ov5693_set_frame_rate,
	.set_group_hold = ov5693_set_group_hold,
	.fill_string_ctrl = ov5693_fill_string_ctrl,
};

static struct tegracam_ctrl_ops imx219_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx219_set_gain,
	.set_exposure = imx219_set_exposure,
	.set_frame_rate = imx219_set_frame_rate,
	.set_group_hold = imx219_set_group_hold,
};
#endif

static int ov5647_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);

	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err) dev_err(dev, "%s failed.\n", __func__);
		else pw->state = SWITCH_ON;
		return err;
	}
	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor
	 */

	if (pw->avdd) err = regulator_enable(pw->avdd);
	if (err) goto ov5647_avdd_fail;

	if (pw->iovdd) err = regulator_enable(pw->iovdd);
	if (err) goto ov5647_iovdd_fail;

	usleep_range(1, 2);
	if (gpio_is_valid(pw->pwdn_gpio))
		ov5647_gpio_set(s_data, pw->pwdn_gpio, 1);

	/*
	 * datasheet 2.9: reset requires ~2ms settling time
	 * a power on reset is generated after core power becomes stable
	 */
	usleep_range(2000, 2010);

	if (gpio_is_valid(pw->reset_gpio))
		ov5647_gpio_set(s_data, pw->reset_gpio, 1);

	/* datasheet fig 2-9: t3 */
	usleep_range(2000, 2010);

	pw->state = SWITCH_ON;

	return 0;

ov5647_iovdd_fail:
	regulator_disable(pw->avdd);

ov5647_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);
	return -ENODEV;

}

static int ov5647_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct device *dev = s_data->dev;
	struct camera_common_pdata *pdata = s_data->pdata;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err) {
			goto power_off_done;
		} else {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	}

	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor
	 */
	usleep_range(21, 25);
	if (gpio_is_valid(pw->pwdn_gpio))
		ov5647_gpio_set(s_data, pw->pwdn_gpio, 0);
	usleep_range(1, 2);
	if (gpio_is_valid(pw->reset_gpio))
		ov5647_gpio_set(s_data, pw->reset_gpio, 0);

	/* datasheet 2.9: reset requires ~2ms settling time*/
	usleep_range(2000, 2010);

	if (pw->iovdd) regulator_disable(pw->iovdd);
	if (pw->avdd) regulator_disable(pw->avdd);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int ov5647_power_get(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = tc_dev->dev;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0, ret = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}
	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	/* analog 2.8v */
	err |= camera_common_regulator_get(dev,
			&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	err |= camera_common_regulator_get(dev,
			&pw->iovdd, pdata->regulators.iovdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}
	if (pdata->use_cam_gpio) {
		err = cam_gpio_register(dev, pw->pwdn_gpio);
		if (err)
			dev_err(dev, "%s ERR can't register cam gpio %u!\n",
				 __func__, pw->pwdn_gpio);
	} else {
		if (gpio_is_valid(pw->pwdn_gpio)) {
			ret = gpio_request(pw->pwdn_gpio, "cam_pwdn_gpio");
			if (ret < 0) {
				dev_dbg(dev, "%s can't request pwdn_gpio %d\n",
					__func__, ret);
			}
			gpio_direction_output(pw->pwdn_gpio, 1);
		}
		if (gpio_is_valid(pw->reset_gpio)) {
			ret = gpio_request(pw->reset_gpio, "cam_reset_gpio");
			if (ret < 0) {
				dev_dbg(dev, "%s can't request reset_gpio %d\n",
					__func__, ret);
			}
			gpio_direction_output(pw->reset_gpio, 1);
		}
	}

	pw->state = SWITCH_OFF;
	return err;
}

static int ov5647_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = tc_dev->dev;

	if (unlikely(!pw))
		return -EFAULT;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_deregister(dev, pw->pwdn_gpio);
	else {
		if (gpio_is_valid(pw->pwdn_gpio))
			gpio_free(pw->pwdn_gpio);
		if (gpio_is_valid(pw->reset_gpio))
			gpio_free(pw->reset_gpio);
	}

	return 0;
}

static int ov5647_set_mode(struct tegracam_device *tc_dev)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err;

	err = ov5647_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	return 0;
}

static int test_mode;
module_param(test_mode, int, 0644);

static int ov5647_start_streaming(struct tegracam_device *tc_dev)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int err;
	u8 val;

	mutex_lock(&priv->streaming_lock);
	err = ov5647_write_table(priv, mode_table[OV5647_MODE_START_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else {
		priv->streaming = true;
		mutex_unlock(&priv->streaming_lock);
	}
	if (pdata->v_flip) {
		ov5647_read_reg(s_data, OV5647_TIMING_REG20, &val);
		ov5647_write_reg(s_data, OV5647_TIMING_REG20,
				 val | VERTICAL_FLIP);
	}
	if (pdata->h_mirror) {
		ov5647_read_reg(s_data, OV5647_TIMING_REG21, &val);
		ov5647_write_reg(s_data, OV5647_TIMING_REG21,
				 val | HORIZONTAL_MIRROR_MASK);
	} else {
		ov5647_read_reg(s_data, OV5647_TIMING_REG21, &val);
		ov5647_write_reg(s_data, OV5647_TIMING_REG21,
				 val & (~HORIZONTAL_MIRROR_MASK));
	}
	if (test_mode)
		err = ov5647_write_table(priv,
			mode_table[OV5647_MODE_TEST_PATTERN]);

	return 0;

exit:
	dev_err(dev, "%s: error starting stream\n", __func__);
	return err;
}

static int ov5647_stop_streaming(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	struct device *dev = s_data->dev;
	u32 frame_time;
	int err;

	ov5647_update_ctrl_range(s_data, OV5647_MAX_FRAME_LENGTH);

	mutex_lock(&priv->streaming_lock);
	err = ov5647_write_table(priv, mode_table[OV5647_MODE_STOP_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else {
		priv->streaming = false;
		mutex_unlock(&priv->streaming_lock);
	}

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * frame_time = frame length rows * Tline
	 * Tline = line length / pixel clock (in MHz)
	 */
	frame_time = priv->frame_length *
		OV5647_DEFAULT_LINE_LENGTH / OV5647_DEFAULT_PIXEL_CLOCK;

	usleep_range(frame_time, frame_time + 1000);

	return 0;

exit:
	dev_err(dev, "%s: error stopping stream\n", __func__);
	return err;
}

static struct camera_common_pdata *ov5647_parse_dt(struct tegracam_device *tc_dev)
{
	// struct device *dev = tc_dev->dev;
	// struct device_node *node = dev->of_node;
	//struct camera_common_pdata *board_priv_pdata;
	//const struct of_device_id *match;
	//int gpio;
	//int err;
	struct camera_common_pdata *ret = NULL;

    return(ret);
}

static struct camera_common_sensor_ops ov5647_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ov5647_frmfmt),
	.frmfmt_table = ov5647_frmfmt,
	.power_on = ov5647_power_on,
	.power_off = ov5647_power_off,
	.write_reg = ov5647_write_reg,
	.read_reg = ov5647_read_reg,
	.parse_dt = ov5647_parse_dt,
	.power_get = ov5647_power_get,
	.power_put = ov5647_power_put,
	.set_mode = ov5647_set_mode,
	.start_streaming = ov5647_start_streaming,
	.stop_streaming = ov5647_stop_streaming,
};

#if 0
static struct camera_common_sensor_ops imx219_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx219_frmfmt),
	.frmfmt_table = imx219_frmfmt,
	.power_on = imx219_power_on,
	.power_off = imx219_power_off,
	.write_reg = imx219_write_reg,
	.read_reg = imx219_read_reg,
	.parse_dt = imx219_parse_dt,
	.power_get = imx219_power_get,
	.power_put = imx219_power_put,
	.set_mode = imx219_set_mode,
	.start_streaming = imx219_start_streaming,
	.stop_streaming = imx219_stop_streaming,
};
#endif
/* ----------------------------------------------------------------------- */
#if 0
static int ov5647_detect(struct v4l2_subdev *sd)
{
	u8 read;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
do {
	if ( ( ret = ov5647_write(sd, OV5647_SW_RESET, 0x01) ) < 0 ) break;
	if ( ( ret = ov5647_read(sd, OV5647_REG_CHIPID_H, &read) ) < 0 ) break;

	if (read != 0x56) {
		dev_err(&client->dev, "ID High expected 0x56 got %x", read);
		return -ENODEV;
	}

	if ( ( ret = ov5647_read(sd, OV5647_REG_CHIPID_L, &read) ) < 0 ) break;

	if (read != 0x47) {
		dev_err(&client->dev, "ID Low expected 0x47 got %x", read);
		return -ENODEV;
	}
 } while(0);
	return ov5647_write(sd, OV5647_SW_RESET, 0x00);
}
#endif
static int ov5647_board_setup(struct ov5647 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;

	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);

	err = camera_common_mclk_enable(s_data);
	if ( err ) {
		dev_err(dev, "Error %d turning on mclk\n", err);
		return err;
	}

	err = ov5647_power_on(s_data);
	if (err) {
		dev_err(dev, "Error %d during power on sensor\n", err);
		return err;
	}
/*
	err = ov5647_otp_setup(priv);
	if (err) {
		dev_err(dev, "Error %d reading otp data\n", err);
		goto error;
	}

	err = ov5647_fuse_id_setup(priv);
	if (err) {
		dev_err(dev, "Error %d reading fuse id data\n", err);
		goto error;
	}
*/
// error:
	ov5647_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
}

static int ov5647_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops ov5647_subdev_internal_ops = {
	.open = ov5647_open,
};

static struct regmap_config ov5647_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static int ov5647_sensor_probe(struct i2c_client *client,
      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct ov5647 *priv;
	int err;

	dev_dbg(dev, "probing v4l2 sensor at addr 0x%0x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct ov5647), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ov5647", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &ov5647_regmap_config;
	tc_dev->sensor_ops = &ov5647_common_ops;
	tc_dev->v4l2sd_internal_ops = &ov5647_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ov5647_ctrl_ops;
	
	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}

	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);
	mutex_init(&priv->streaming_lock);

	err = ov5647_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}
	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_dbg(dev, "detected ov5647 sensor\n");

  return 0;
}

static int ov5647_sensor_remove(struct i2c_client *client)
{
    struct camera_common_data *s_data = to_camera_common_data(&client->dev);
    struct ov5647 *priv = (struct ov5647 *)s_data->priv;
    
    //ov5693_debugfs_remove(priv);
    
    tegracam_v4l2subdev_unregister(priv->tc_dev);
    ov5647_power_put(priv->tc_dev);
    tegracam_device_unregister(priv->tc_dev);
    mutex_destroy(&priv->streaming_lock);

    return 0;
}

static const struct i2c_device_id ov5647_sensor_id[] = {
  { OV5647_SENSOR_NAME, 0 },
  { }
};
MODULE_DEVICE_TABLE(i2c, ov5647_sensor_id);

static struct i2c_driver ov5647_sensor_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name = OV5647_SENSOR_NAME,
	//.of_match_table = of_match_ptr(imx219_of_match),
  },
  .probe = ov5647_sensor_probe,
  .remove = ov5647_sensor_remove,
  .id_table = ov5647_sensor_id,
};

module_i2c_driver(ov5647_sensor_driver);

MODULE_AUTHOR("Michael Moore");
MODULE_DESCRIPTION("A OV5647 sensors drivers for nvidia Jetson Nano");
MODULE_LICENSE("GPL");

