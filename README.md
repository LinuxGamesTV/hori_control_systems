# This is a Experimental Linux Kernel driver (for kernels 6.12 or higher) for the Hori Truck Control System.

All three pedals work now: Gas, brake and clutch.
Force Feedback is not working yet.

Button 5 cannot be used as return or left mouse button, because the button is not mappable as left mouse button/return key.
The steering wheel behaves the same as under Windows. The left analog stick can now be used as a mouse.
The dead zones have also been resolved and the hatswitch is working as expected.

The dead zones and range from the Wheel are fixed at this moment and the driver does not yet have the userspace connection which will be used for the future Linux Hori Device Management GUI.

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
