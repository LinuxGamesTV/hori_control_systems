# This is a Experimental Linux Kernel driver (for kernels 6.12 or higher) for the Hori Truck Control System.

All three pedals work now: Gas, brake and clutch.
Force Feedback works now: the wheel's own effect engine is used, so all Linux force feedback effects are supported (constant, ramp, sine/square/triangle/sawtooth, spring, damper, inertia, friction), plus gain and autocenter. The protocol notes are in `hid-hori.c`. Games using SDL might need the environment variable `SDL_JOYSTICK_WHEEL_DEVICES=0x0f0d/0x017a` to recognize the wheel. If forces feel mirrored in a game, load the module with `invert_ff=1`.

The wheel settings from the HORI Device Manager can be changed from Linux. They appear as files in the wheel's sysfs directory and are saved on the wheel, like Device Manager does:

| File | Values | Device Manager setting |
|---|---|---|
| `range` | 180-1800 in 90 degree steps, `0` = Game Setting | Steering Output Range |
| `wheel_deadzone` | -3..3 | Dead Zone |
| `wheel_sensitivity` | -3..3 | Sensitivity |
| `throttle_sensitivity`, `brake_sensitivity`, `clutch_sensitivity` | -3..3 | Pedal Sensitivity |
| `ffb_strength` | 1-10 | Force Feedback Adjustment |
| `ffb_vibration` | -2..2 | Vibration (scales high frequency force feedback, e.g. engine rumble) |
| `stick_level` | 0-3 | Analog Stick and D-pad |

```shell
W=$(ls -d /sys/bus/hid/drivers/hid-hori-control-systems/0003:0F0D:017A.*)
cat $W/range
echo 900 | sudo tee $W/range
```

Install the udev rules (`sudo make udev-rules`) to change them without root, e.g. from Oversteer.

Button 5 cannot be used as return or left mouse button, because the button is not mappable as left mouse button/return key.
The steering wheel behaves the same as under Windows. The left analog stick can now be used as a mouse.
The dead zones have also been resolved and the hatswitch is working as expected.

The shifter is fully working now, too.

This kernel driver is an early alpha.

## Testing
```shell
# Create a debug build
make debug

# Load built module. It will automatically reload
# it if it was loaded previously
sudo make load
```

## DKMS install
DKMS will install the module into system, and will update it every time you update your kernel. The module will persist after reboots.
It's the preferrable way to install it on most distros.

1. Install `dkms` package from your distro package manager
2. Clone repository to `/usr/src/hid-hori`
3. Install the module:
```
sudo dkms install /usr/src/hid-hori
```
4. Reboot

**I cannot be held responsible for any damage to the device. The use of this driver is at your own risk**

![Pedals_work](https://github.com/user-attachments/assets/7f347458-5c01-4d28-bd4c-e2b78a502ef2)

[Installation](https://github.com/LinuxGamesTV/hori_control_systems/wiki)
