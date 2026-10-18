// SPDX-License-Identifier: GPL-2.0
/*
 * GalaxyCore GC02M1 Camera Sensor driver
 *
 * Mainline-style port for Xiaomi marble (POCO F5) macro camera.
 * Register init derived from GalaxyCore GC02Mx family tables (Rockchip
 * gc02m2 / MTK gc02m1ofilm); chip-id is GC02M1-specific 0x02e0 @ 0xf0.
 *
 * Evidence: evidence/cam-slaveaddr-re-2026-10-19.md (CamX sensorDriverData
 * idReg=0xf0, id=0x02e0, slave 0x6e → Linux reg 0x37).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/media-entity.h>

#define GC02M1_CHIP_ID			0x02e0
#define GC02M1_REG_CHIP_ID		CCI_REG16(0xf0)

#define GC02M1_REG_PAGE			CCI_REG8(0xfe)
#define GC02M1_REG_STREAM		CCI_REG8(0x3e)
#define GC02M1_STREAM_ON		0x90
#define GC02M1_STREAM_OFF		0x00

#define GC02M1_REG_EXPOSURE_H		CCI_REG8(0x03)
#define GC02M1_REG_EXPOSURE_L		CCI_REG8(0x04)
#define GC02M1_EXPOSURE_MIN		4
#define GC02M1_EXPOSURE_MARGIN		16

#define GC02M1_REG_AGAIN		CCI_REG8(0xb6)
#define GC02M1_AGAIN_MIN		0
#define GC02M1_AGAIN_MAX		0x0f
#define GC02M1_AGAIN_DEFAULT		0

#define GC02M1_NATIVE_WIDTH		1600
#define GC02M1_NATIVE_HEIGHT		1200
#define GC02M1_HTS			0x0448
#define GC02M1_VTS_DEF			0x04f4
#define GC02M1_VTS_MIN			GC02M1_VTS_DEF
#define GC02M1_VTS_MAX			0x1fff

#define GC02M1_LINK_FREQ_336MHZ		(336ULL * HZ_PER_MHZ)
#define GC02M1_DATA_LANES		1
#define GC02M1_RGB_DEPTH		10
#define GC02M1_PIXEL_RATE \
	(GC02M1_LINK_FREQ_336MHZ * 2 * GC02M1_DATA_LANES / GC02M1_RGB_DEPTH)

#define GC02M1_MCLK_FREQ		(19200000)

static const char *const gc02m1_supply_name[] = {
	"dovdd",
	"avdd",
};

struct gc02m1 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc02m1_supply_name)];

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;

	bool identified;
};

static inline struct gc02m1 *to_gc02m1(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc02m1, sd);
}

/* GC02Mx family init (Rockchip gc02m2 / MTK gc02m1); stream left off. */
static const struct cci_reg_sequence gc02m1_init_regs[] = {
	{ CCI_REG8(0xfc), 0x01 },
	{ CCI_REG8(0xf4), 0x41 },
	{ CCI_REG8(0xf5), 0xc0 },
	{ CCI_REG8(0xf6), 0x44 },
	{ CCI_REG8(0xf8), 0x38 },
	{ CCI_REG8(0xf9), 0x82 },
	{ CCI_REG8(0xfa), 0x00 },
	{ CCI_REG8(0xfd), 0x80 },
	{ CCI_REG8(0xfc), 0x81 },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x01), 0x0b },
	{ CCI_REG8(0xf7), 0x01 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x87), 0x09 },
	{ CCI_REG8(0xee), 0x72 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x8c), 0x90 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x90), 0x00 },
	{ CCI_REG8(0x03), 0x04 },
	{ CCI_REG8(0x04), 0x7d },
	{ CCI_REG8(0x41), 0x04 },
	{ CCI_REG8(0x42), 0xf4 },
	{ CCI_REG8(0x05), 0x04 },
	{ CCI_REG8(0x06), 0x48 },
	{ CCI_REG8(0x07), 0x00 },
	{ CCI_REG8(0x08), 0x18 },
	{ CCI_REG8(0x9d), 0x18 },
	{ CCI_REG8(0x09), 0x00 },
	{ CCI_REG8(0x0a), 0x02 },
	{ CCI_REG8(0x0d), 0x04 },
	{ CCI_REG8(0x0e), 0xbc },
	{ CCI_REG8(0x17), 0x80 },
	{ CCI_REG8(0x19), 0x04 },
	{ CCI_REG8(0x24), 0x00 },
	{ CCI_REG8(0x56), 0x20 },
	{ CCI_REG8(0x5b), 0x00 },
	{ CCI_REG8(0x5e), 0x01 },
	{ CCI_REG8(0x21), 0x3c },
	{ CCI_REG8(0x44), 0x20 },
	{ CCI_REG8(0xcc), 0x01 },
	{ CCI_REG8(0x1a), 0x04 },
	{ CCI_REG8(0x1f), 0x11 },
	{ CCI_REG8(0x27), 0x30 },
	{ CCI_REG8(0x2b), 0x00 },
	{ CCI_REG8(0x33), 0x00 },
	{ CCI_REG8(0x53), 0x90 },
	{ CCI_REG8(0xe6), 0x50 },
	{ CCI_REG8(0x39), 0x07 },
	{ CCI_REG8(0x43), 0x04 },
	{ CCI_REG8(0x46), 0x2a },
	{ CCI_REG8(0x7c), 0xa0 },
	{ CCI_REG8(0xd0), 0xbe },
	{ CCI_REG8(0xd1), 0x40 },
	{ CCI_REG8(0xd2), 0x40 },
	{ CCI_REG8(0xd3), 0xb3 },
	{ CCI_REG8(0xde), 0x1c },
	{ CCI_REG8(0xcd), 0x06 },
	{ CCI_REG8(0xce), 0x6f },
	{ CCI_REG8(0xfc), 0x88 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x88 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0xe0), 0x01 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x53), 0x54 },
	{ CCI_REG8(0x87), 0x53 },
	{ CCI_REG8(0x89), 0x03 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xb0), 0x74 },
	{ CCI_REG8(0xb1), 0x04 },
	{ CCI_REG8(0xb2), 0x00 },
	{ CCI_REG8(0xb6), 0x00 },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0xd8), 0x00 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x2a },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x90 },
	{ CCI_REG8(0xc0), 0x19 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0xD0 },
	{ CCI_REG8(0xc0), 0x2F },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x90 },
	{ CCI_REG8(0xc0), 0x39 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x04 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x0f },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x1a },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x25 },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x2c },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x32 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x38 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x3c },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x02 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x02 },
	{ CCI_REG8(0xc0), 0x18 },
	{ CCI_REG8(0xc0), 0x5c },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x9f), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x26), 0x20 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x40), 0x22 },
	{ CCI_REG8(0x46), 0x7f },
	{ CCI_REG8(0x49), 0x0f },
	{ CCI_REG8(0x4a), 0xf0 },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0x14), 0x80 },
	{ CCI_REG8(0x15), 0x80 },
	{ CCI_REG8(0x16), 0x80 },
	{ CCI_REG8(0x17), 0x80 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x41), 0x20 },
	{ CCI_REG8(0x4c), 0x00 },
	{ CCI_REG8(0x4d), 0x0c },
	{ CCI_REG8(0x44), 0x08 },
	{ CCI_REG8(0x48), 0x03 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x90), 0x01 },
	{ CCI_REG8(0x91), 0x00 },
	{ CCI_REG8(0x92), 0x06 },
	{ CCI_REG8(0x93), 0x00 },
	{ CCI_REG8(0x94), 0x06 },
	{ CCI_REG8(0x95), 0x04 },
	{ CCI_REG8(0x96), 0xb0 },
	{ CCI_REG8(0x97), 0x06 },
	{ CCI_REG8(0x98), 0x40 },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x01), 0x23 },
	{ CCI_REG8(0x03), 0xce },
	{ CCI_REG8(0x04), 0x48 },
	{ CCI_REG8(0x15), 0x01 },
	{ CCI_REG8(0x21), 0x10 },
	{ CCI_REG8(0x22), 0x05 },
	{ CCI_REG8(0x23), 0x20 },
	{ CCI_REG8(0x25), 0x20 },
	{ CCI_REG8(0x26), 0x08 },
	{ CCI_REG8(0x29), 0x06 },
	{ CCI_REG8(0x2a), 0x0a },
	{ CCI_REG8(0x2b), 0x08 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x8c), 0x10 },
};

static const s64 gc02m1_link_freq_menu[] = {
	GC02M1_LINK_FREQ_336MHZ,
};

static int gc02m1_identify(struct gc02m1 *gc02m1)
{
	u64 chip_id;
	int ret;

	if (gc02m1->identified)
		return 0;

	ret = cci_read(gc02m1->regmap, GC02M1_REG_CHIP_ID, &chip_id, NULL);
	if (ret) {
		dev_err(gc02m1->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (chip_id != GC02M1_CHIP_ID) {
		dev_err(gc02m1->dev, "unexpected chip id 0x%04llx (want 0x%04x)\n",
			chip_id, GC02M1_CHIP_ID);
		return -ENXIO;
	}

	gc02m1->identified = true;
	dev_info(gc02m1->dev, "GC02M1 detected (0x%04llx)\n", chip_id);
	return 0;
}

static int gc02m1_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(gc02m1_supply_name),
				    gc02m1->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(gc02m1->xclk);
	if (ret) {
		dev_err(dev, "failed to enable xclk: %d\n", ret);
		goto disable_regs;
	}

	fsleep(1000);

	gpiod_set_value_cansleep(gc02m1->reset_gpio, 0);
	fsleep(10000);

	ret = gc02m1_identify(gc02m1);
	if (ret)
		goto assert_reset;

	return 0;

assert_reset:
	gpiod_set_value_cansleep(gc02m1->reset_gpio, 1);
	clk_disable_unprepare(gc02m1->xclk);
disable_regs:
	regulator_bulk_disable(ARRAY_SIZE(gc02m1_supply_name), gc02m1->supplies);
	return ret;
}

static int gc02m1_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	gpiod_set_value_cansleep(gc02m1->reset_gpio, 1);
	clk_disable_unprepare(gc02m1->xclk);
	regulator_bulk_disable(ARRAY_SIZE(gc02m1_supply_name), gc02m1->supplies);
	return 0;
}

static int gc02m1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc02m1 *gc02m1 = container_of(ctrl->handler, struct gc02m1, ctrls);
	int ret = 0;

	if (!pm_runtime_get_if_in_use(gc02m1->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, NULL);
		ret = cci_write(gc02m1->regmap, GC02M1_REG_EXPOSURE_H,
				(ctrl->val >> 8) & 0x3f, &ret);
		ret = cci_write(gc02m1->regmap, GC02M1_REG_EXPOSURE_L,
				ctrl->val & 0xff, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, NULL);
		ret = cci_write(gc02m1->regmap, GC02M1_REG_AGAIN, ctrl->val, &ret);
		break;
	default:
		break;
	}

	pm_runtime_put(gc02m1->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc02m1_ctrl_ops = {
	.s_ctrl = gc02m1_set_ctrl,
};

static int gc02m1_init_controls(struct gc02m1 *gc02m1)
{
	struct v4l2_ctrl_handler *hdl = &gc02m1->ctrls;
	s64 exposure_max = GC02M1_VTS_DEF - GC02M1_EXPOSURE_MARGIN;
	s64 hblank = GC02M1_HTS - GC02M1_NATIVE_WIDTH;
	s64 vblank_def = GC02M1_VTS_DEF - GC02M1_NATIVE_HEIGHT;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 6);
	if (ret)
		return ret;

	gc02m1->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
						   0, 0, gc02m1_link_freq_menu);
	if (gc02m1->link_freq)
		gc02m1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc02m1->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
					       GC02M1_PIXEL_RATE, GC02M1_PIXEL_RATE,
					       1, GC02M1_PIXEL_RATE);

	gc02m1->vblank = v4l2_ctrl_new_std(hdl, &gc02m1_ctrl_ops, V4L2_CID_VBLANK,
					   vblank_def, GC02M1_VTS_MAX - GC02M1_NATIVE_HEIGHT,
					   1, vblank_def);
	gc02m1->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
					   hblank, hblank, 1, hblank);
	if (gc02m1->hblank)
		gc02m1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc02m1->exposure = v4l2_ctrl_new_std(hdl, &gc02m1_ctrl_ops, V4L2_CID_EXPOSURE,
					     GC02M1_EXPOSURE_MIN, exposure_max, 1,
					     exposure_max);
	gc02m1->again = v4l2_ctrl_new_std(hdl, &gc02m1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
					  GC02M1_AGAIN_MIN, GC02M1_AGAIN_MAX, 1,
					  GC02M1_AGAIN_DEFAULT);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	gc02m1->sd.ctrl_handler = hdl;
	return 0;
}

static int gc02m1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int gc02m1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;
	fse->min_width = GC02M1_NATIVE_WIDTH;
	fse->max_width = GC02M1_NATIVE_WIDTH;
	fse->min_height = GC02M1_NATIVE_HEIGHT;
	fse->max_height = GC02M1_NATIVE_HEIGHT;
	return 0;
}

static int gc02m1_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, fmt->pad);
	format->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	format->width = GC02M1_NATIVE_WIDTH;
	format->height = GC02M1_NATIVE_HEIGHT;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;
	fmt->format = *format;
	return 0;
}

static int gc02m1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, 0);
	format->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	format->width = GC02M1_NATIVE_WIDTH;
	format->height = GC02M1_NATIVE_HEIGHT;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;
	return 0;
}

static int gc02m1_start_streaming(struct gc02m1 *gc02m1)
{
	int ret;

	ret = pm_runtime_resume_and_get(gc02m1->dev);
	if (ret < 0)
		return ret;

	ret = gc02m1_identify(gc02m1);
	if (ret)
		goto err;

	ret = cci_multi_reg_write(gc02m1->regmap, gc02m1_init_regs,
				  ARRAY_SIZE(gc02m1_init_regs), NULL);
	if (ret)
		goto err;

	ret = __v4l2_ctrl_handler_setup(&gc02m1->ctrls);
	if (ret)
		goto err;

	ret = cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, NULL);
	ret = cci_write(gc02m1->regmap, GC02M1_REG_STREAM, GC02M1_STREAM_ON, &ret);
	if (ret)
		goto err;

	return 0;
err:
	pm_runtime_put(gc02m1->dev);
	return ret;
}

static int gc02m1_stop_streaming(struct gc02m1 *gc02m1)
{
	int ret = 0;

	cci_write(gc02m1->regmap, GC02M1_REG_PAGE, 0x00, &ret);
	cci_write(gc02m1->regmap, GC02M1_REG_STREAM, GC02M1_STREAM_OFF, &ret);
	pm_runtime_put(gc02m1->dev);
	return ret;
}

static int gc02m1_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (enable)
		ret = gc02m1_start_streaming(gc02m1);
	else
		ret = gc02m1_stop_streaming(gc02m1);
	v4l2_subdev_unlock_state(state);
	return ret;
}

static const struct v4l2_subdev_video_ops gc02m1_video_ops = {
	.s_stream = gc02m1_s_stream,
};

static const struct v4l2_subdev_pad_ops gc02m1_pad_ops = {
	.enum_mbus_code = gc02m1_enum_mbus_code,
	.enum_frame_size = gc02m1_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc02m1_set_format,
};

static const struct v4l2_subdev_ops gc02m1_subdev_ops = {
	.video = &gc02m1_video_ops,
	.pad = &gc02m1_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc02m1_internal_ops = {
	.init_state = gc02m1_init_state,
};

static const struct media_entity_operations gc02m1_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int gc02m1_parse_fwnode(struct gc02m1 *gc02m1)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct device *dev = gc02m1->dev;
	unsigned int i;
	int ret;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GC02M1_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL, "only 1 data-lane supported\n");
		goto done;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(dev, -EINVAL, "link-frequencies missing\n");
		goto done;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		if (bus_cfg.link_frequencies[i] == GC02M1_LINK_FREQ_336MHZ)
			break;
	}
	if (i == bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(dev, -EINVAL,
				    "link-frequencies must include 336 MHz\n");
		goto done;
	}

	ret = 0;
done:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int gc02m1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc02m1 *gc02m1;
	unsigned int i;
	int ret;

	gc02m1 = devm_kzalloc(dev, sizeof(*gc02m1), GFP_KERNEL);
	if (!gc02m1)
		return -ENOMEM;

	gc02m1->dev = dev;
	dev_set_drvdata(dev, &gc02m1->sd);

	ret = gc02m1_parse_fwnode(gc02m1);
	if (ret)
		return ret;

	gc02m1->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gc02m1->regmap))
		return dev_err_probe(dev, PTR_ERR(gc02m1->regmap),
				     "failed to init CCI regmap\n");

	gc02m1->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(gc02m1->xclk))
		return dev_err_probe(dev, PTR_ERR(gc02m1->xclk), "failed to get xclk\n");

	ret = clk_set_rate(gc02m1->xclk, GC02M1_MCLK_FREQ);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set xclk rate\n");

	gc02m1->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gc02m1->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc02m1->reset_gpio),
				     "failed to get reset-gpios\n");

	for (i = 0; i < ARRAY_SIZE(gc02m1_supply_name); i++)
		gc02m1->supplies[i].supply = gc02m1_supply_name[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(gc02m1_supply_name),
				      gc02m1->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = gc02m1_init_controls(gc02m1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init controls\n");

	gc02m1->sd.owner = THIS_MODULE;
	gc02m1->sd.dev = dev;
	v4l2_i2c_subdev_init(&gc02m1->sd, client, &gc02m1_subdev_ops);
	gc02m1->sd.internal_ops = &gc02m1_internal_ops;
	gc02m1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc02m1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc02m1->pad.flags = MEDIA_PAD_FL_SOURCE;
	gc02m1->sd.entity.ops = &gc02m1_entity_ops;

	ret = media_entity_pads_init(&gc02m1->sd.entity, 1, &gc02m1->pad);
	if (ret)
		goto err_ctrl;

	gc02m1->sd.state_lock = gc02m1->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&gc02m1->sd);
	if (ret)
		goto err_entity;

	ret = gc02m1_power_on(dev);
	if (ret)
		goto err_finalize;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev_sensor(&gc02m1->sd);
	if (ret) {
		gc02m1_power_off(dev);
		goto err_rpm;
	}

	return 0;

err_rpm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_finalize:
	v4l2_subdev_cleanup(&gc02m1->sd);
err_entity:
	media_entity_cleanup(&gc02m1->sd.entity);
err_ctrl:
	v4l2_ctrl_handler_free(&gc02m1->ctrls);
	return ret;
}

static void gc02m1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc02m1->ctrls);
	pm_runtime_disable(gc02m1->dev);
	if (!pm_runtime_status_suspended(gc02m1->dev))
		gc02m1_power_off(gc02m1->dev);
	pm_runtime_set_suspended(gc02m1->dev);
}

static const struct of_device_id gc02m1_of_match[] = {
	{ .compatible = "galaxycore,gc02m1" },
	{ }
};
MODULE_DEVICE_TABLE(of, gc02m1_of_match);

static DEFINE_RUNTIME_DEV_PM_OPS(gc02m1_pm_ops, gc02m1_power_off, gc02m1_power_on, NULL);

static struct i2c_driver gc02m1_i2c_driver = {
	.driver = {
		.name = "gc02m1",
		.pm = pm_ptr(&gc02m1_pm_ops),
		.of_match_table = gc02m1_of_match,
	},
	.probe = gc02m1_probe,
	.remove = gc02m1_remove,
};
module_i2c_driver(gc02m1_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore GC02M1 camera sensor driver");
MODULE_LICENSE("GPL");
