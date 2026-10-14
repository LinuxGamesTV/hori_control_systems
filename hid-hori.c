// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for some Hori "special" devices
 *
 *  Copyright (c) 2024/2025 Peter Stein <linuxgamestv@gmail.com>
 */

#include <linux/bits.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/usb.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/unaligned.h>

#include "usbhid/usbhid.h"
#include "hid-ids.h"

#define HORI_TCS_WHEEL_RDESC_ORIG_SIZE		155
#define HORI_TCS_SHIFTER_RDESC_ORIG_SIZE	83

#define ABS_CLUTCH      ABS_RZ
#define JOY_RANGE	(BTN_DEAD - BTN_JOYSTICK + 1)

/*
 * Force feedback protocol (reverse engineered from USB captures)
 *
 * FFB commands are 32 byte vendor output reports (report id 0x20) sent to
 * the interrupt OUT endpoint (EP 2):
 *
 *   20 01 <sub> <args>		configuration commands
 *   20 02 10 <mode> <force16> 00 00 <u32 10000> <u32 10000>
 *				effect state, mode:
 *				0x10 = reset (once at init)
 *				0x00 = all effects off (wheel moves freely)
 *				0x80 = centering spring (wheel returns to center)
 *				0x40 = constant force, force16 signed LE
 *					(full scale assumed +-10000)
 *
 * Config subcommands seen in captures: 0x03 01 (init), 0x01 <0|1> (0 while
 * effects are driven), 0x02 <0|1> (1 enables effect updates) and
 * 0x05 00 08 07 (sent with every effect update, 0x0708 = 1800).
 *
 * The device acks every command on the interrupt IN endpoint with the
 * command byte | 0x80, e.g. 20 81 02 01 acks 20 01 02 01.
 */
#define HORI_FFB_REPORT_ID	0x20
#define HORI_FFB_REPORT_LEN	32
#define HORI_FFB_MAX_EFFECTS	16
#define HORI_FFB_FORCE_MAX	10000
#define HORI_FFB_TICK_MS	8

#define HORI_FFB_MODE_OFF	0x00
#define HORI_FFB_MODE_RESET	0x10
#define HORI_FFB_MODE_FORCE	0x40
#define HORI_FFB_MODE_SPRING	0x80

struct hori_ff_effect {
	bool active;
	u8 type;
	s16 level;
	u16 direction;
	u16 length;
	unsigned long start;
};

struct hori_device {
	struct hid_device *hdev;
	struct input_dev *input;
	struct delayed_work ff_work;
	spinlock_t lock;
	u8 mode;
	u16 gain;
	u16 autocenter;
	bool stopping;
	u8 outbuf[HORI_FFB_REPORT_LEN];
	struct hori_ff_effect effects[HORI_FFB_MAX_EFFECTS];
};

static struct hori_device *hori_from_input(struct input_dev *dev)
{
	return hid_get_drvdata(input_get_drvdata(dev));
}

/* Fixed report descriptors for HORI FFB Truck Control Systems
 * wheel controllers
 *
 * The original descriptors hide the separate throttle and brake axes in
 * a custom vendor usage page, providing only a combined value as
 * GenericDesktop.THROTTLE.
 * These descriptors remove the combined THROTTLE axis and instead report
 * separate throttle (GAS) and brake (BRAKE).
 */
static const __u8 tcs_wheel_rdesc_fixed[] = {
	0x05, 0x01,                    // Usage Page (Generic Desktop)           0
	0x09, 0x04,                    // Usage (Joystick)                       2
	0xa1, 0x01,                    // Collection (Application)               4
	0x85, 0x01,                    //  Report ID (1)                         6
	0x15, 0x00,                    //  Logical Minimum (0)                   8
	0x25, 0x07,                    //  Logical Maximum (7)                   10
	0x35, 0x00,                    //  Physical Minimum (0)                  12
	0x46, 0x3b, 0x07, 0x08,        //  Physical Maximum (315)                14
	0x65, 0x14,                    //  Unit (EnglishRotation: deg)           17
	0x09, 0x39,                    //  Usage (Hat switch)                    19
	0x75, 0x04,                    //  Report Size (4)                       21
	0x95, 0x01,                    //  Report Count (1)                      23
	0x81, 0x42,                    //  Input (Data,Var,Abs,Null)             25
	0x65, 0x00,                    //  Unit (None)                           27
	0x25, 0x01,                    //  Logical Maximum (1)                   29
	0x45, 0x01,                    //  Physical Maximum (1)                  31
	0x05, 0x09,                    //  Usage Page (Button)                   33
	0x19, 0x01,                    //  Usage Minimum (1)                     35
	0x29, 0x36,                    //  Usage Maximum (54)                    37
	0x75, 0x01,                    //  Report Size (1)                       39
	0x95, 0x36,                    //  Report Count (54)                     41
	0x81, 0x02,                    //  Input (Data,Var,Abs)                  43
	0x95, 0x06,                    //  Report Count (6)                      45
	0x81, 0x01,                    //  Input (Cnst,Arr,Abs)                  47
	0x27, 0xff, 0xff, 0x00, 0x00,  //  Logical Maximum (65535)               49
	0x47, 0xff, 0xff, 0x00, 0x00,  //  Physical Maximum (65535)              54
	0x75, 0x10,                    //  Report Size (16)                      59
	0x95, 0x08,                    //  Report Count (8)                      61
	0x05, 0x01,                    //  Usage Page (Generic Desktop)          63
	0x09, 0x30,                    //  Usage (X)                             65
	0x09, 0x31,                    //  Usage (Y)                             67
	0x09, 0x32,                    //  Usage (Z)                             69
	0x09, 0x33,                    //  Usage (Rx)                            71
	0x09, 0x34,                    //  Usage (Ry)                            73
	0x09, 0x35,                    //  Usage (Rz)                            75
	0x09, 0x36,                    //  Usage (Slider)                        77
	0x09, 0x36,                    //  Usage (Slider)                        79
	0x81, 0x02,                    //  Input (Data,Var,Abs)                  81
	0x26, 0xff, 0x00,              //  Logical Maximum (255)                 83
	0x46, 0xff, 0x00,              //  Physical Maximum (255)                86
	0x06, 0x00, 0xff,              //  Usage Page (Vendor Defined Page 1)    89
	0x09, 0x01,                    //  Usage (Vendor Usage 1)                92
	0x75, 0x08,                    //  Report Size (8)                       94
	0x95, 0x04,                    //  Report Count (4)                      96
	0x81, 0x02,                    //  Input (Data,Var,Abs)                  98
	0xc0,                          // End Collection                         100
	0x06, 0x20, 0xff,              // Usage Page (Vendor Usage Page 0xff20)  101
	0x09, 0x01,                    // Usage (Vendor Usage 0x01)              104
	0xa1, 0x01,                    // Collection (Application)               106
	0x85, 0x20,                    //  Report ID (32)                        108
	0x15, 0x00,                    //  Logical Minimum (0)                   110
	0x26, 0xff, 0x00,              //  Logical Maximum (255)                 112
	0x75, 0x08,                    //  Report Size (8)                       115
	0x95, 0x1f,                    //  Report Count (31)                     117
	0x09, 0x02,                    //  Usage (Vendor Usage 0x02)             119
	0x81, 0x00,                    //  Input (Data,Arr,Abs)                  121
	0x09, 0x02,                    //  Usage (Vendor Usage 0x02)             123
	0x91, 0x00,                    //  Output (Data,Arr,Abs)                 125
	0xc0,                          // End Collection                         127
	0x06, 0x21, 0xff,              // Usage Page (Vendor Usage Page 0xff21)  128
	0x09, 0x03,                    // Usage (Vendor Usage 0x03)              131
	0xa1, 0x01,                    // Collection (Application)               133
	0x85, 0x21,                    //  Report ID (33)                        135
	0x15, 0x00,                    //  Logical Minimum (0)                   137
	0x26, 0xff, 0x00,              //  Logical Maximum (255)                 139
	0x75, 0x08,                    //  Report Size (8)                       142
	0x95, 0x3f,                    //  Report Count (63)                     144
	0x09, 0x04,                    //  Usage (Vendor Usage 0x04)             146
	0x81, 0x00,                    //  Input (Data,Arr,Abs)                  148
	0x09, 0x04,                    //  Usage (Vendor Usage 0x04)             150
	0x91, 0x00,                    //  Output (Data,Arr,Abs)                 152
	0xc0,                          // End Collection                         154
};

static const __u8 tcs_shifter_rdesc_fixed[] = {
	0x05, 0x01,        // Usage Page (Generic Desktop)           0
	0x09, 0x04,        // Usage (Joystick)                       2
	0xa1, 0x01,        // Collection (Application)               4
	0x85, 0x01,        //  Report ID (1)                         6
	0x15, 0x00,        //  Logical Minimum (0)                   8
	0x25, 0x01,        //  Logical Maximum (1)                   10
	0x35, 0x00,        //  Physical Minimum (0)                  12
	0x45, 0x01,        //  Physical Maximum (1)                  14
	0x05, 0x09,        //  Usage Page (Button)                   16
	0x19, 0x01,        //  Usage Minimum (1)                     18
	0x29, 0x2b,        //  Usage Maximum (43)                    20
	0x75, 0x01,        //  Report Size (1)                       22
	0x95, 0x2b,        //  Report Count (43)                     24
	0x81, 0x02,        //  Input (Data,Var,Abs)                  26
	0x95, 0x05,        //  Report Count (5)                      28
	0x81, 0x01,        //  Input (Cnst,Arr,Abs)                  30
	0x26, 0xff, 0x00,  //  Logical Maximum (255)                 32
	0x46, 0xff, 0x00,  //  Physical Maximum (255)                35
	0x06, 0x00, 0xff,  //  Usage Page (Vendor Defined Page 1)    38
	0x09, 0x01,        //  Usage (Vendor Usage 1)                41
	0x75, 0x08,        //  Report Size (8)                       43
	0x95, 0x04,        //  Report Count (4)                      45
	0x81, 0x02,        //  Input (Data,Var,Abs)                  47
	0x09, 0x02,        //  Usage (Vendor Usage 2)                49
	0x95, 0x1f,        //  Report Count (31)                     51
	0x91, 0x02,        //  Output (Data,Var,Abs)                 53
	0xc0,              // End Collection                         55
	0x06, 0x21, 0xff,  // Usage Page (Vendor Usage Page 0xff21)  56
	0x09, 0x03,        // Usage (Vendor Usage 0x03)              59
	0xa1, 0x01,        // Collection (Application)               61
	0x85, 0x22,        //  Report ID (34)                        63
	0x15, 0x00,        //  Logical Minimum (0)                   65
	0x26, 0xff, 0x00,  //  Logical Maximum (255)                 67
	0x75, 0x08,        //  Report Size (8)                       70
	0x95, 0x3f,        //  Report Count (63)                     72
	0x09, 0x04,        //  Usage (Vendor Usage 0x04)             74
	0x81, 0x00,        //  Input (Data,Arr,Abs)                  76
	0x09, 0x04,        //  Usage (Vendor Usage 0x04)             78
	0x91, 0x00,        //  Output (Data,Arr,Abs)                 80
	0xc0,              // End Collection                         82
};

static const s32 hat_to_axis[9][2] = {
	{0, 0},
	{0, -1},
	{1, -1},
	{1, 0},
	{1, 1},
	{0, 1},
	{-1, 1},
	{-1, 0},
	{-1, -1},
};

static const __u8 *hori_report_fixup(struct hid_device *hdev, __u8 *rdesc,
				     unsigned int *rsize)
{
	switch (hdev->product) {
	case USB_DEVICE_ID_HORI_TRUCK_WHEEL:
		if (*rsize != HORI_TCS_WHEEL_RDESC_ORIG_SIZE)
			break;

		hid_info(hdev, "fixing up HORI TCS Wheel report descriptor\n");
		*rsize = sizeof(tcs_wheel_rdesc_fixed);
		return tcs_wheel_rdesc_fixed;

	case USB_DEVICE_ID_HORI_TRUCK_SHIFTER:
		if (*rsize != HORI_TCS_SHIFTER_RDESC_ORIG_SIZE)
			break;

		hid_info(hdev, "fixing up HORI TCS Shifter report descriptor\n");
		*rsize = sizeof(tcs_shifter_rdesc_fixed);
		return tcs_shifter_rdesc_fixed;
	}

	return rdesc;
}

static const u8 hori_ff_init_a[HORI_FFB_REPORT_LEN] = { 0x20, 0x01, 0x03, 0x01 };
static const u8 hori_ff_init_b[HORI_FFB_REPORT_LEN] = { 0x20, 0x01, 0x01, 0x01 };
static const u8 hori_ff_init_c[HORI_FFB_REPORT_LEN] = { 0x20, 0x01, 0x02, 0x00 };
static const u8 hori_ff_enable[HORI_FFB_REPORT_LEN] = { 0x20, 0x01, 0x02, 0x01 };
static const u8 hori_ff_params[HORI_FFB_REPORT_LEN] = { 0x20, 0x01, 0x05, 0x00, 0x08, 0x07 };
static const u8 hori_ff_autocenter_off[HORI_FFB_REPORT_LEN] = { 0x20, 0x01, 0x01, 0x00 };

static const u8 hori_ff_effect_report[HORI_FFB_REPORT_LEN] = {
	0x20, 0x02, 0x10, 0x00,
	0x00, 0x00,			/* [4:6] force, signed LE */
	0x00, 0x00,
	0x10, 0x27, 0x00, 0x00,		/* 10000 */
	0x10, 0x27, 0x00, 0x00,		/* 10000 */
};

/*
 * The output reports are passed through a DMA capable buffer, USB cannot
 * map stack memory.
 */
static void hori_ff_send(struct hori_device *hd, const u8 *data)
{
	memcpy(hd->outbuf, data, HORI_FFB_REPORT_LEN);

	if (hid_hw_output_report(hd->hdev, hd->outbuf,
				 HORI_FFB_REPORT_LEN) == -ENOSYS)
		hid_hw_raw_request(hd->hdev, HORI_FFB_REPORT_ID, hd->outbuf,
				   HORI_FFB_REPORT_LEN, HID_OUTPUT_REPORT,
				   HID_REQ_SET_REPORT);
}

static void hori_ff_send_mode(struct hori_device *hd, u8 mode)
{
	u8 buf[HORI_FFB_REPORT_LEN];

	memcpy(buf, hori_ff_effect_report, HORI_FFB_REPORT_LEN);
	buf[3] = mode;
	hori_ff_send(hd, buf);
}

static void hori_ff_arm(struct hori_device *hd)
{
	int i;

	for (i = 0; i < 13; i++) {
		hori_ff_send(hd, hori_ff_init_a);
		hori_ff_send(hd, hori_ff_init_b);
		hori_ff_send(hd, hori_ff_init_c);
	}
	hori_ff_send(hd, hori_ff_enable);
	hori_ff_send_mode(hd, HORI_FFB_MODE_RESET);

	/* The device needs a short spring phase before effects are accepted */
	for (i = 0; i < 15; i++) {
		hori_ff_send_mode(hd, HORI_FFB_MODE_SPRING);
		msleep(8);
	}
}

static void hori_ff_init_device(struct hori_device *hd)
{
	hori_ff_arm(hd);
	hori_ff_send_mode(hd, HORI_FFB_MODE_OFF);
	hd->mode = HORI_FFB_MODE_OFF;
}

static void hori_ff_workfn(struct work_struct *work)
{
	struct hori_device *hd = container_of(to_delayed_work(work),
					      struct hori_device, ff_work);
	u8 buf[HORI_FFB_REPORT_LEN];
	unsigned long flags;
	s32 force = 0;
	bool constant = false, spring = false;
	int i;

	spin_lock_irqsave(&hd->lock, flags);
	for (i = 0; i < HORI_FFB_MAX_EFFECTS; i++) {
		struct hori_ff_effect *eff = &hd->effects[i];

		if (!eff->active)
			continue;
		if (eff->length && time_after(jiffies, eff->start +
					      msecs_to_jiffies(eff->length))) {
			eff->active = false;
			continue;
		}

		switch (eff->type) {
		case FF_CONSTANT:
			constant = true;
			if (eff->direction >= 0x4000 && eff->direction < 0xc000)
				force -= eff->level;
			else
				force += eff->level;
			break;
		case FF_SPRING:
			spring = true;
			break;
		}
	}
	if (hd->autocenter)
		spring = true;
	force = force * hd->gain / 65535;
	spin_unlock_irqrestore(&hd->lock, flags);

	if (constant) {
		s32 wire = force * HORI_FFB_FORCE_MAX / 32767;

		if (wire > HORI_FFB_FORCE_MAX)
			wire = HORI_FFB_FORCE_MAX;
		if (wire < -HORI_FFB_FORCE_MAX)
			wire = -HORI_FFB_FORCE_MAX;

		if (hd->mode == HORI_FFB_MODE_OFF)
			hori_ff_arm(hd);
		hori_ff_send(hd, hori_ff_params);
		hori_ff_send(hd, hori_ff_autocenter_off);
		hori_ff_send(hd, hori_ff_enable);
		memcpy(buf, hori_ff_effect_report, HORI_FFB_REPORT_LEN);
		buf[3] = HORI_FFB_MODE_FORCE;
		put_unaligned_le16((u16)wire, &buf[4]);
		hori_ff_send(hd, buf);
		hd->mode = HORI_FFB_MODE_FORCE;
	} else if (spring) {
		if (hd->mode == HORI_FFB_MODE_OFF)
			hori_ff_arm(hd);
		hori_ff_send_mode(hd, HORI_FFB_MODE_SPRING);
		hd->mode = HORI_FFB_MODE_SPRING;
	} else {
		if (hd->mode != HORI_FFB_MODE_OFF) {
			hori_ff_send_mode(hd, HORI_FFB_MODE_OFF);
			hd->mode = HORI_FFB_MODE_OFF;
		}
		return;
	}

	spin_lock_irqsave(&hd->lock, flags);
	if (!hd->stopping)
		schedule_delayed_work(&hd->ff_work,
				      msecs_to_jiffies(HORI_FFB_TICK_MS));
	spin_unlock_irqrestore(&hd->lock, flags);
}

static int hori_ff_upload(struct input_dev *dev, struct ff_effect *effect,
			  struct ff_effect *old)
{
	struct hori_device *hd = hori_from_input(dev);
	struct hori_ff_effect *eff = &hd->effects[effect->id];
	unsigned long flags;

	spin_lock_irqsave(&hd->lock, flags);
	eff->type = effect->type;
	eff->direction = effect->direction;
	eff->length = effect->replay.length;
	if (effect->type == FF_CONSTANT)
		eff->level = effect->u.constant.level;
	spin_unlock_irqrestore(&hd->lock, flags);

	return 0;
}

static int hori_ff_erase(struct input_dev *dev, int effect_id)
{
	struct hori_device *hd = hori_from_input(dev);
	unsigned long flags;

	spin_lock_irqsave(&hd->lock, flags);
	hd->effects[effect_id].active = false;
	spin_unlock_irqrestore(&hd->lock, flags);

	return 0;
}

static int hori_ff_playback(struct input_dev *dev, int effect_id, int value)
{
	struct hori_device *hd = hori_from_input(dev);
	unsigned long flags;

	spin_lock_irqsave(&hd->lock, flags);
	hd->effects[effect_id].active = value;
	if (value)
		hd->effects[effect_id].start = jiffies;
	spin_unlock_irqrestore(&hd->lock, flags);

	schedule_delayed_work(&hd->ff_work, 0);

	return 0;
}

static void hori_ff_set_gain(struct input_dev *dev, u16 gain)
{
	struct hori_device *hd = hori_from_input(dev);
	unsigned long flags;

	spin_lock_irqsave(&hd->lock, flags);
	hd->gain = gain;
	spin_unlock_irqrestore(&hd->lock, flags);
}

static void hori_ff_set_autocenter(struct input_dev *dev, u16 magnitude)
{
	struct hori_device *hd = hori_from_input(dev);
	unsigned long flags;

	spin_lock_irqsave(&hd->lock, flags);
	hd->autocenter = magnitude;
	spin_unlock_irqrestore(&hd->lock, flags);

	schedule_delayed_work(&hd->ff_work, 0);
}

static void parse_tcs_axis_report(struct input_dev *input_dev, u8 *data)
{
	u8 hat = data[1] & 0x0f;

	input_report_abs(input_dev, ABS_X, get_unaligned_le16(&data[13]));
	input_report_abs(input_dev, ABS_Y, get_unaligned_le16(&data[11]));
	input_report_abs(input_dev, ABS_WHEEL, get_unaligned_le16(&data[9]));
	input_report_abs(input_dev, ABS_RX, get_unaligned_le16(&data[15]));
	input_report_abs(input_dev, ABS_RY, get_unaligned_le16(&data[17]));
	input_report_abs(input_dev, ABS_CLUTCH, get_unaligned_le16(&data[19]));
	input_report_abs(input_dev, ABS_GAS, get_unaligned_le16(&data[21]));
	input_report_abs(input_dev, ABS_BRAKE, get_unaligned_le16(&data[23]));

	/* Hat is always the upper nibble of the penultimate byte of the report. This hatswitch is very tricky */
	input_report_abs(input_dev, ABS_HAT0X, hat_to_axis[hat][2]);
	input_report_abs(input_dev, ABS_HAT0Y, -hat_to_axis[hat][6]);

	input_report_abs(input_dev, ABS_MISC, data[26] << 8);
}

static int hori_raw_event(struct hid_device *hdev,
			  struct hid_report *report, u8 *data, int len)
{
	struct hori_device *hd = hid_get_drvdata(hdev);

	if (hdev->product != USB_DEVICE_ID_HORI_TRUCK_WHEEL)
		return 0;

	/* Several wheels report as this id when operating in emulation mode. */

	/* Report 1 carries the wheel state, the 0x20 reports are FFB acks */
	if (report->id != 1 || !hd->input)
		return 0;

	parse_tcs_axis_report(hd->input, data);
	input_sync(hd->input);

	return 0;
}

static int hori_input_configured(struct hid_device *hdev,
				 struct hid_input *input)
{
	if (hdev->product != USB_DEVICE_ID_HORI_TRUCK_WHEEL)
		return 0;

	/* Several wheels report as this id when operating in emulation mode. */
	struct hori_device *hd = hid_get_drvdata(hdev);
	struct input_dev *input_dev = input->input;

	hd->input = input_dev;

	set_bit(EV_ABS, input_dev->evbit);

	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_WHEEL, input_dev->absbit);
	set_bit(ABS_RX, input_dev->absbit);
	set_bit(ABS_RY, input_dev->absbit);
	set_bit(ABS_CLUTCH, input_dev->absbit);
	set_bit(ABS_GAS, input_dev->absbit);
	set_bit(ABS_BRAKE, input_dev->absbit);
	set_bit(ABS_HAT0X, input_dev->absbit);
	set_bit(ABS_HAT0Y, input_dev->absbit);
	set_bit(ABS_MISC, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_WHEEL, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_RX, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_RY, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_CLUTCH, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_GAS, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_BRAKE, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_HAT0X, -1, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_HAT0Y, -1, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MISC, 0, 255, 255, 0);

	set_bit(FF_CONSTANT, input_dev->ffbit);
	set_bit(FF_SPRING, input_dev->ffbit);
	set_bit(FF_GAIN, input_dev->ffbit);
	set_bit(FF_AUTOCENTER, input_dev->ffbit);

	if (input_ff_create(input_dev, HORI_FFB_MAX_EFFECTS)) {
		hid_err(hdev, "cannot create FF device\n");
		return 0;
	}

	input_dev->ff->upload = hori_ff_upload;
	input_dev->ff->erase = hori_ff_erase;
	input_dev->ff->playback = hori_ff_playback;
	input_dev->ff->set_gain = hori_ff_set_gain;
	input_dev->ff->set_autocenter = hori_ff_set_autocenter;

	return 0;
}

static int hori_input_mapping(struct hid_device *dev, struct hid_input *input,
			      struct hid_field *field, struct hid_usage *usage,
			      unsigned long **bit, int *max)
{
	if (dev->product != USB_DEVICE_ID_HORI_TRUCK_WHEEL)
		return 0;

	/* Skip mapping of device axes */
	switch (usage->hid) {
	case HID_GD_X: case HID_GD_Y: case HID_GD_Z:
	case HID_GD_RX: case HID_GD_RY: case HID_GD_RZ:
	case HID_GD_SLIDER: case HID_GD_HATSWITCH:
		return -1;
	}

	return 0;
}

static int hori_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct hori_device *hd;
	int ret;

	hd = devm_kzalloc(&hdev->dev, sizeof(*hd), GFP_KERNEL);
	if (!hd)
		return -ENOMEM;

	hd->hdev = hdev;
	hd->gain = 65535;
	spin_lock_init(&hd->lock);
	INIT_DELAYED_WORK(&hd->ff_work, hori_ff_workfn);
	hid_set_drvdata(hdev, hd);

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		return ret;

	if (hdev->product == USB_DEVICE_ID_HORI_TRUCK_WHEEL)
		hori_ff_init_device(hd);

	return 0;
}

static void hori_remove(struct hid_device *hdev)
{
	struct hori_device *hd = hid_get_drvdata(hdev);
	unsigned long flags;

	if (hdev->product == USB_DEVICE_ID_HORI_TRUCK_WHEEL) {
		spin_lock_irqsave(&hd->lock, flags);
		hd->stopping = true;
		spin_unlock_irqrestore(&hd->lock, flags);
		cancel_delayed_work_sync(&hd->ff_work);
		hori_ff_send_mode(hd, HORI_FFB_MODE_OFF);
	}

	hid_hw_stop(hdev);
}

static const struct hid_device_id hori_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_HORI, USB_DEVICE_ID_HORI_TRUCK_WHEEL) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_HORI, USB_DEVICE_ID_HORI_TRUCK_SHIFTER) },
	{ }
};
MODULE_DEVICE_TABLE(hid, hori_devices);

static struct hid_driver hori_driver = {
	.name = "hid-hori-control-systems",
	.id_table = hori_devices,
	.probe = hori_probe,
	.remove = hori_remove,
	.report_fixup = hori_report_fixup,
	.input_mapping = hori_input_mapping,
	.input_configured = hori_input_configured,
	.raw_event = hori_raw_event,
};
module_hid_driver(hori_driver);

MODULE_AUTHOR("Peter Stein <linuxgamestv@gmail.com>");
MODULE_DESCRIPTION("HID driver for HORI Control Systems Wheels");
MODULE_LICENSE("GPL");
