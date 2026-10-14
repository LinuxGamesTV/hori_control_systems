#Makefile
KVERSION := `uname -r`
KDIR := /lib/modules/${KVERSION}/build
MODULE_LOADED := $(shell lsmod | grep hid_hori)

default: clean
	$(MAKE) -C $(KDIR) M=$$PWD

debug: clean
	$(MAKE) -C $(KDIR) M=$$PWD EXTRA_CFLAGS="-g -DDEBUG"

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

install: default
	$(MAKE) -C $(KDIR) M=$$PWD modules_install
	depmod -A

unload:
	@if [ "$(MODULE_LOADED)" != "" ]; then\
		rmmod hid_hori;\
	fi

load: unload
	insmod hid-hori.ko

udev-rules:
	install -m 0644 udev/99-hori-wheel.rules /etc/udev/rules.d/
