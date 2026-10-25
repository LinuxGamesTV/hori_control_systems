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
#include <linux/fixp-arith.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/usb.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/unaligned.h>

#ifndef timer_container_of		/* added in 6.16 */
#define timer_container_of from_timer
#endif

#include "usbhid/usbhid.h"
#include "hid-ids.h"

#define HORI_TCS_WHEEL_RDESC_ORIG_SIZE		155
#define HORI_TCS_SHIFTER_RDESC_ORIG_SIZE	83

#define ABS_CLUTCH      ABS_RZ
#define JOY_RANGE	(BTN_DEAD - BTN_JOYSTICK + 1)

/*
 * Force feedback protocol
 *
 * Reverse engineered from HORI's DirectInput effect driver (HoriWFfb.dll,
 * "HORI FFB RWD DRIVERS" 1.18) and verified on hardware and against USB
 * captures of that driver.
 *
 * FFB commands are 32 byte vendor output reports (report id 0x20) sent to
 * the interrupt OUT endpoint. The wheel keeps up to 16 effects in slots and
 * plays them itself; the host only uploads, starts and stops them.
 *
 *   20 01 <cmd> <arg>		device control
 *	01 <1|0>		stop all effects / release stop
 *	02 <1|0>		host control of the motor on / off; while off the
 *				firmware applies its own centring spring
 *	03 01			free all effect slots
 *	(05 <u16>, a steering range sent by HORI's driver, is ignored by the
 *	 Truck wheel; its range is a stored setting, see the config channel)
 *
 *   20 02 <type> <op|slot> <params...>	effect slot 0..15
 *	type:	0x10 constant, 0x20 ramp, 0x31 square, 0x32 sine,
 *		0x33 triangle, 0x34 saw up, 0x35 saw down,
 *		0x41 spring, 0x42 damper, 0x43 inertia, 0x44 friction
 *	op:	0x10 create, 0x00 update, 0x40 play, 0x80 stop, 0x20 free
 *	params:	little endian u16/s16, magnitudes in +-10000, times in ms,
 *		duration 0 = infinite. Envelope = attack level, attack time,
 *		fade level, fade time.
 *	constant:  level, duration, envelope
 *	ramp:	   start, end, duration, envelope
 *	periodic:  period, magnitude, phase (ms), offset, duration, envelope
 *	condition: neg. coeff, neg. saturation, deadband, pos. coeff,
 *		   pos. saturation, deadband, center, duration
 *
 * The wheel acks every command on the interrupt IN endpoint with the class
 * byte | 0x80, e.g. 20 81 02 01 acks 20 01 02 01.
 */
#define HORI_FFB_REPORT_ID	0x20
#define HORI_FFB_PAYLOAD	31	/* bytes after the report id */
#define HORI_FFB_SLOTS		16
#define HORI_FFB_AC_SLOT	15	/* reserved for the autocenter spring */
#define HORI_FFB_USER_EFFECTS	15
#define HORI_FFB_FULL		10000	/* device units for 100 % */

#define HORI_FFB_CLASS_DEVCTL	0x01
#define HORI_FFB_CLASS_EFFECT	0x02

#define HORI_FFB_STOPALL	0x01
#define HORI_FFB_ACTUATORS	0x02
#define HORI_FFB_RESET		0x03

#define HORI_FFB_OP_UPDATE	0x00
#define HORI_FFB_OP_CREATE	0x10
#define HORI_FFB_OP_FREE	0x20
#define HORI_FFB_OP_PLAY	0x40
#define HORI_FFB_OP_STOP	0x80

static bool invert_ff;
module_param(invert_ff, bool, 0644);
MODULE_PARM_DESC(invert_ff, "Mirror all forces left/right (default: N)");

struct hori_device;

struct hori_ff_slot {
	struct hori_device *hd;
	struct timer_list timer;
	struct ff_effect eff;
	unsigned int repeats;	/* plays left, including the current one */
	u8 idx;
	bool uploaded;
	bool active;
	bool waiting_delay;
};

struct hori_device {
	struct hid_device *hdev;
	struct input_dev *input;
	struct hid_report *ffb_report;
	spinlock_t lock;	/* ffb_report values, slots, flags below */
	struct hori_ff_slot slot[HORI_FFB_SLOTS];
	u16 gain;
	bool ac_uploaded;
	bool motor_on;
	bool removed;
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

/*
 * Effects are sent with hid_hw_request(), which usbhid queues on the
 * interrupt OUT endpoint without sleeping, so this is safe from the
 * playback, gain and autocenter callbacks (they run in atomic context).
 * Caller holds hd->lock; data = the 31 bytes following the report id.
 */
static void hori_ff_send_locked(struct hori_device *hd, const u8 *data)
{
	struct hid_field *field;
	int i;

	if (hd->removed)
		return;

	field = hd->ffb_report->field[0];
	for (i = 0; i < HORI_FFB_PAYLOAD; i++)
		field->value[i] = data[i];

	hid_hw_request(hd->hdev, hd->ffb_report, HID_REQ_SET_REPORT);
}

static void hori_ff_devctl_locked(struct hori_device *hd, u8 cmd, u8 arg)
{
	u8 d[HORI_FFB_PAYLOAD] = { HORI_FFB_CLASS_DEVCTL, cmd, arg };

	hori_ff_send_locked(hd, d);
}

static void hori_ff_motor_on_locked(struct hori_device *hd)
{
	if (hd->motor_on)
		return;
	hori_ff_devctl_locked(hd, HORI_FFB_STOPALL, 0);
	hori_ff_devctl_locked(hd, HORI_FFB_ACTUATORS, 1);
	hd->motor_on = true;
}

/* hand the wheel back to firmware centring once nothing is loaded */
static void hori_ff_maybe_release_locked(struct hori_device *hd)
{
	int i;

	if (!hd->motor_on || hd->ac_uploaded)
		return;
	for (i = 0; i < HORI_FFB_USER_EFFECTS; i++)
		if (hd->slot[i].uploaded)
			return;

	hori_ff_devctl_locked(hd, HORI_FFB_STOPALL, 1);
	hori_ff_devctl_locked(hd, HORI_FFB_ACTUATORS, 0);
	hd->motor_on = false;
}

static s64 hori_div_round(s64 n, s32 d)
{
	return div_s64(n + (n < 0 ? -(d / 2) : d / 2), d);
}

/* v in [-in_max, in_max] -> device units [-10000, 10000], times gain */
static s32 hori_ff_scale(s32 v, s32 in_max, u16 gain)
{
	s64 r = hori_div_round((s64)v * HORI_FFB_FULL, in_max);

	r = hori_div_round(r * gain, 0xffff);
	return clamp_t(s64, r, -HORI_FFB_FULL, HORI_FFB_FULL);
}

/*
 * Projection of an effect direction onto the wheel axis, -0x7fff..0x7fff.
 * Linux 0x4000 (90 deg) maps to a negative level, which turns the wheel
 * left; this matches HORI's DirectInput mapping (DI east -> left).
 */
static s32 hori_ff_dir_factor(u16 direction)
{
	s32 f = -fixp_sin16(((u32)direction * 360) >> 16);

	return invert_ff ? -f : f;
}

static void hori_ff_put_env(u8 *p, const struct ff_envelope *env, u16 gain)
{
	/* without an envelope HORI's driver sends full levels, zero times */
	u16 al = env->attack_length ? env->attack_level : 0x7fff;
	u16 fl = env->fade_length ? env->fade_level : 0x7fff;

	put_unaligned_le16(hori_ff_scale(al, 0x7fff, gain), p);
	put_unaligned_le16(min_t(u32, env->attack_length, 0xffff), p + 2);
	put_unaligned_le16(hori_ff_scale(fl, 0x7fff, gain), p + 4);
	put_unaligned_le16(min_t(u32, env->fade_length, 0xffff), p + 6);
}

static int hori_ff_build(const struct ff_effect *e, u8 slot, u8 op, u16 gain,
			 u8 *d)
{
	u8 *p = d + 3;			/* report byte 4 */
	u16 dur = min_t(u32, e->replay.length, 0xffff);	/* 0 = infinite */
	s32 dirf;

	memset(d, 0, HORI_FFB_PAYLOAD);
	d[0] = HORI_FFB_CLASS_EFFECT;
	d[2] = op | slot;

	switch (e->type) {
	case FF_CONSTANT:
		dirf = hori_ff_dir_factor(e->direction);
		d[1] = 0x10;
		put_unaligned_le16((s16)hori_ff_scale(e->u.constant.level * dirf
						      / 0x7fff, 0x7fff, gain), p);
		put_unaligned_le16(dur, p + 2);
		hori_ff_put_env(p + 4, &e->u.constant.envelope, gain);
		break;

	case FF_RAMP:
		dirf = hori_ff_dir_factor(e->direction);
		d[1] = 0x20;
		put_unaligned_le16((s16)hori_ff_scale(e->u.ramp.start_level * dirf
						      / 0x7fff, 0x7fff, gain), p);
		put_unaligned_le16((s16)hori_ff_scale(e->u.ramp.end_level * dirf
						      / 0x7fff, 0x7fff, gain), p + 2);
		put_unaligned_le16(dur, p + 4);
		hori_ff_put_env(p + 6, &e->u.ramp.envelope, gain);
		break;

	case FF_PERIODIC: {
		const struct ff_periodic_effect *pe = &e->u.periodic;
		u16 period = pe->period ? clamp_t(u16, pe->period, 4, 10000) : 1;
		u16 phase = pe->phase;
		s32 mag, offset;

		switch (pe->waveform) {
		case FF_SQUARE:		d[1] = 0x31; break;
		case FF_SINE:		d[1] = 0x32; break;
		case FF_TRIANGLE:	d[1] = 0x33; break;
		case FF_SAW_UP:		d[1] = 0x34; break;
		case FF_SAW_DOWN:	d[1] = 0x35; break;
		default:
			return -EINVAL;
		}

		/*
		 * Vibrations use only the sign of the direction, not its
		 * projection: ff-core converts FF_RUMBLE to a sine with
		 * direction 0, which would otherwise scale to nothing on a
		 * one-axis wheel.
		 */
		dirf = hori_ff_dir_factor(e->direction);
		mag = pe->magnitude;
		offset = pe->offset;
		if (dirf < 0) {
			mag = -mag;
			offset = -offset;
		}
		if (mag < 0) {		/* device magnitude is unsigned */
			mag = -mag;
			phase += 0x8000;
		}

		put_unaligned_le16(period, p);
		put_unaligned_le16(hori_ff_scale(mag, 0x7fff, gain), p + 2);
		put_unaligned_le16((u32)period * phase >> 16, p + 4);
		put_unaligned_le16((s16)hori_ff_scale(offset, 0x7fff, gain), p + 6);
		put_unaligned_le16(dur, p + 8);
		hori_ff_put_env(p + 10, &pe->envelope, gain);
		break;
	}

	case FF_SPRING:
	case FF_DAMPER:
	case FF_INERTIA:
	case FF_FRICTION: {
		const struct ff_condition_effect *c = &e->u.condition[0];
		s32 lc = c->left_coeff, rc = c->right_coeff;
		u32 ls = c->left_saturation, rs = c->right_saturation;
		s32 center = c->center;
		s16 db = c->deadband * HORI_FFB_FULL / 0xffff;

		d[1] = e->type == FF_SPRING ? 0x41 :
		       e->type == FF_DAMPER ? 0x42 :
		       e->type == FF_INERTIA ? 0x43 : 0x44;

		if (invert_ff) {
			swap(lc, rc);
			swap(ls, rs);
			center = -center;
		}

		put_unaligned_le16((s16)hori_ff_scale(lc, 0x7fff, gain), p);
		put_unaligned_le16(hori_ff_scale(ls, 0xffff, gain), p + 2);
		put_unaligned_le16(db, p + 4);
		put_unaligned_le16((s16)hori_ff_scale(rc, 0x7fff, gain), p + 6);
		put_unaligned_le16(hori_ff_scale(rs, 0xffff, gain), p + 8);
		put_unaligned_le16(db, p + 10);
		put_unaligned_le16((s16)hori_ff_scale(center, 0x7fff, 0xffff),
				   p + 12);
		put_unaligned_le16(dur, p + 14);
		break;
	}

	default:
		return -EINVAL;
	}

	return 0;
}

static void hori_ff_slot_op_locked(struct hori_device *hd,
				   struct hori_ff_slot *s, u8 op)
{
	u8 d[HORI_FFB_PAYLOAD];

	if (!hori_ff_build(&s->eff, s->idx, op, hd->gain, d))
		hori_ff_send_locked(hd, d);
}

static void hori_ff_start_locked(struct hori_device *hd,
				 struct hori_ff_slot *s)
{
	s->waiting_delay = false;
	hori_ff_motor_on_locked(hd);
	hori_ff_slot_op_locked(hd, s, HORI_FFB_OP_PLAY);

	/* replays beyond the first are host-timed, as in HORI's driver */
	if (s->repeats > 1 && s->eff.replay.length)
		mod_timer(&s->timer,
			  jiffies + msecs_to_jiffies(s->eff.replay.length));
}

static void hori_ff_slot_timer(struct timer_list *t)
{
	struct hori_ff_slot *s = timer_container_of(s, t, timer);
	struct hori_device *hd = s->hd;
	unsigned long flags;

	spin_lock_irqsave(&hd->lock, flags);
	if (!s->active || !s->uploaded || hd->removed)
		goto out;

	if (s->waiting_delay) {
		hori_ff_start_locked(hd, s);
	} else if (--s->repeats) {
		hori_ff_slot_op_locked(hd, s, HORI_FFB_OP_PLAY);
		if (s->repeats > 1)
			mod_timer(&s->timer, jiffies +
				  msecs_to_jiffies(s->eff.replay.length));
	} else {
		s->active = false;
	}
out:
	spin_unlock_irqrestore(&hd->lock, flags);
}

static int hori_ff_upload(struct input_dev *dev, struct ff_effect *effect,
			  struct ff_effect *old)
{
	struct hori_device *hd = hori_from_input(dev);
	struct hori_ff_slot *s;
	unsigned long flags;
	u8 d[HORI_FFB_PAYLOAD];
	int ret;

	if (effect->id < 0 || effect->id >= HORI_FFB_USER_EFFECTS)
		return -EINVAL;
	s = &hd->slot[effect->id];

	spin_lock_irqsave(&hd->lock, flags);
	ret = hori_ff_build(effect, s->idx,
			    s->uploaded ? HORI_FFB_OP_UPDATE : HORI_FFB_OP_CREATE,
			    hd->gain, d);
	if (!ret) {
		hori_ff_motor_on_locked(hd);
		hori_ff_send_locked(hd, d);
		s->eff = *effect;
		s->uploaded = true;
	}
	spin_unlock_irqrestore(&hd->lock, flags);
	return ret;
}

static int hori_ff_erase(struct input_dev *dev, int id)
{
	struct hori_device *hd = hori_from_input(dev);
	struct hori_ff_slot *s;
	unsigned long flags;

	if (id < 0 || id >= HORI_FFB_USER_EFFECTS)
		return -EINVAL;
	s = &hd->slot[id];

	spin_lock_irqsave(&hd->lock, flags);
	if (s->uploaded) {
		timer_delete(&s->timer);
		s->active = false;
		hori_ff_slot_op_locked(hd, s, HORI_FFB_OP_FREE);
		s->uploaded = false;
		hori_ff_maybe_release_locked(hd);
	}
	spin_unlock_irqrestore(&hd->lock, flags);
	return 0;
}

static int hori_ff_playback(struct input_dev *dev, int id, int value)
{
	struct hori_device *hd = hori_from_input(dev);
	struct hori_ff_slot *s;
	unsigned long flags;

	if (id < 0 || id >= HORI_FFB_USER_EFFECTS)
		return -EINVAL;
	s = &hd->slot[id];

	spin_lock_irqsave(&hd->lock, flags);
	if (!s->uploaded) {
		spin_unlock_irqrestore(&hd->lock, flags);
		return -EINVAL;
	}

	timer_delete(&s->timer);
	if (value > 0) {
		s->active = true;
		s->repeats = value;
		if (s->eff.replay.delay) {
			s->waiting_delay = true;
			mod_timer(&s->timer, jiffies +
				  msecs_to_jiffies(s->eff.replay.delay));
		} else {
			hori_ff_start_locked(hd, s);
		}
	} else {
		s->active = false;
		s->waiting_delay = false;
		hori_ff_slot_op_locked(hd, s, HORI_FFB_OP_STOP);
	}
	spin_unlock_irqrestore(&hd->lock, flags);
	return 0;
}

static void hori_ff_set_gain(struct input_dev *dev, u16 gain)
{
	struct hori_device *hd = hori_from_input(dev);
	unsigned long flags;
	int i;

	spin_lock_irqsave(&hd->lock, flags);
	hd->gain = gain;
	for (i = 0; i < HORI_FFB_USER_EFFECTS; i++)
		if (hd->slot[i].uploaded)
			hori_ff_slot_op_locked(hd, &hd->slot[i],
					       HORI_FFB_OP_UPDATE);
	spin_unlock_irqrestore(&hd->lock, flags);
}

static void hori_ff_set_autocenter(struct input_dev *dev, u16 value)
{
	struct hori_device *hd = hori_from_input(dev);
	struct hori_ff_slot *s = &hd->slot[HORI_FFB_AC_SLOT];
	unsigned long flags;

	spin_lock_irqsave(&hd->lock, flags);
	if (value) {
		struct ff_condition_effect *c = &s->eff.u.condition[0];

		memset(&s->eff, 0, sizeof(s->eff));
		s->eff.type = FF_SPRING;
		c->left_coeff = c->right_coeff = value >> 1;
		c->left_saturation = c->right_saturation = 0xffff;

		hori_ff_motor_on_locked(hd);
		hori_ff_slot_op_locked(hd, s, hd->ac_uploaded ?
				       HORI_FFB_OP_UPDATE :
				       HORI_FFB_OP_CREATE | HORI_FFB_OP_PLAY);
		hd->ac_uploaded = true;
	} else if (hd->ac_uploaded) {
		hori_ff_slot_op_locked(hd, s, HORI_FFB_OP_STOP);
		hori_ff_slot_op_locked(hd, s, HORI_FFB_OP_FREE);
		hd->ac_uploaded = false;
		hori_ff_maybe_release_locked(hd);
	}
	spin_unlock_irqrestore(&hd->lock, flags);
}

static int hori_ff_init(struct hori_device *hd, struct input_dev *input_dev)
{
	static const int ff_bits[] = {
		FF_CONSTANT, FF_RAMP, FF_PERIODIC, FF_SQUARE, FF_SINE,
		FF_TRIANGLE, FF_SAW_UP, FF_SAW_DOWN, FF_SPRING, FF_DAMPER,
		FF_INERTIA, FF_FRICTION, FF_GAIN, FF_AUTOCENTER,
	};
	struct hid_report *rep;
	struct ff_device *ff;
	int i, ret;

	rep = hd->hdev->report_enum[HID_OUTPUT_REPORT].report_id_hash[HORI_FFB_REPORT_ID];
	if (!rep || rep->maxfield < 1 || rep->field[0]->report_size != 8 ||
	    rep->field[0]->report_count < HORI_FFB_PAYLOAD) {
		hid_warn(hd->hdev, "no FFB output report, force feedback disabled\n");
		return -ENODEV;
	}
	hd->ffb_report = rep;

	for (i = 0; i < ARRAY_SIZE(ff_bits); i++)
		set_bit(ff_bits[i], input_dev->ffbit);

	ret = input_ff_create(input_dev, HORI_FFB_USER_EFFECTS);
	if (ret)
		return ret;

	/* ff->private is kfree()d by ff-core, so it is left unused */
	ff = input_dev->ff;
	ff->upload = hori_ff_upload;
	ff->erase = hori_ff_erase;
	ff->playback = hori_ff_playback;
	ff->set_gain = hori_ff_set_gain;
	ff->set_autocenter = hori_ff_set_autocenter;
	return 0;
}

/* stop everything and give the wheel back to firmware centring */
static void hori_ff_shutdown(struct hori_device *hd)
{
	unsigned long flags;
	int i;

	if (hd->ffb_report) {
		spin_lock_irqsave(&hd->lock, flags);
		for (i = 0; i < HORI_FFB_SLOTS; i++)
			hd->slot[i].active = false;
		hori_ff_devctl_locked(hd, HORI_FFB_RESET, 1);
		hori_ff_devctl_locked(hd, HORI_FFB_STOPALL, 1);
		hori_ff_devctl_locked(hd, HORI_FFB_ACTUATORS, 0);
		hd->motor_on = false;
		hd->removed = true;	/* no more reports after this */
		spin_unlock_irqrestore(&hd->lock, flags);
	}

	for (i = 0; i < HORI_FFB_SLOTS; i++)
		timer_delete_sync(&hd->slot[i].timer);
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

	if (hori_ff_init(hd, input_dev))
		hid_warn(hdev, "force feedback not available\n");

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
	int i, ret;

	hd = devm_kzalloc(&hdev->dev, sizeof(*hd), GFP_KERNEL);
	if (!hd)
		return -ENOMEM;

	hd->hdev = hdev;
	hd->gain = 0xffff;
	spin_lock_init(&hd->lock);
	for (i = 0; i < HORI_FFB_SLOTS; i++) {
		hd->slot[i].hd = hd;
		hd->slot[i].idx = i;
		timer_setup(&hd->slot[i].timer, hori_ff_slot_timer, 0);
	}
	hid_set_drvdata(hdev, hd);

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		return ret;

	return 0;
}

static void hori_remove(struct hid_device *hdev)
{
	struct hori_device *hd = hid_get_drvdata(hdev);

	if (hdev->product == USB_DEVICE_ID_HORI_TRUCK_WHEEL)
		hori_ff_shutdown(hd);

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
