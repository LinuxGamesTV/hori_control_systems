# This is a Experimental Linux Kernel (for kernels 6.12 or higher) driver for the Hori Truck Control System.

All three pedals works now, Gas, brake and clutch.
Force Feedback are not working yet.

Button 5 can't use as return or left mouse button, because the button is not mapped as left mouse button/return key.
The steering wheel behaves the same as under Windows. The left analog stick can now be used as a mouse.
The dead zones are also been resolved and the hatswich are working as expected.

The dead zones and range from the Wheel are fixed at this moment and the driver does not yet have a userspace connection for the future Linux Hori Device Management GUI

The shifter is full working now, too.

This kernel driver is early alpha.

## Testing
```shell
# Create a debug build
make debug

# Load built module. It will automatically reload
# it if it was loaded previously
sudo make load
```

## DKMS install
DKMS will install module into system, and will update it every time you update your kernel. Module will persist after reboots.
It's the preferrable way to install it on the most distros.

1. Install `dkms` package from your distro package manager
2. Clone repository to `/usr/src/hid-hori`
3. Install the module:
```
sudo dkms install /usr/src/hid-hori
```
4. Reboot

**I cannot be held responsible for any damage to the device. The use of this driver is at your own risk**

![Pedals_works](https://github.com/user-attachments/assets/7f347458-5c01-4d28-bd4c-e2b78a502ef2)

[Intallation](https://github.com/LinuxGamesTV/hori_control_systems/wiki)
