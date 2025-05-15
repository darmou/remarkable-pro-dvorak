# Remarkable DVORAK keyboard c file for remarkable PRO Tablet

You will need to have a remarkable pro tablet in development mode to use these instructions.

This is a modified version of the ./drivers/platform/remarkable/rm-hwmon/rm_hwmon_keyboard.c
file that can be used with the Remarkable Pro folio keyboard as a dvorak keyboard layout.

Big thanks to the original author of these instructions for the original remarkable 2 tablet https://gist.github.com/alphashuro/3aa5b67309e974542b9ee59f19079755

## Installation

rm_hwmon_keyboard is the new pogo (keyboard layout) file for the Remarkable Pro tablet.

First go to the remarkable pro kernel source directory
which can be found https://github.com/reMarkable/linux-imx-rm 

### Building the rm_hwmon_keyboard.ko module



Extract the kernel source
tar -xvf linux-imx-rel-4.4-ng-3.18.1.1-4f90133eb4.tar.gz
cd linux-imx-rel-4.4-ng-3.18.1.1-4f90133eb4

Next we need to compile using the remarkable toolchain

```agsl
docker pull eeems/remarkable-toolchain:latest
docker run -v $(pwd):/linux-imx-rel-4.2-lod-3.16.3.0-f3e0cceb34 -ti eeems/remarkable-toolchain:latest
```

the following commands should be run in the container shell that opens after the previous command.
```
cd /linux-imx-rel-4.2-lod-3.16.3.0-f3e0cceb34
source `find /opt/codex -name environment-setup-cortexa53-crypto-remarkable-linux`
touch .scmversion
cp arch/arm64/configs/ferrari_defconfig .config
make olddefconfig
make drivers/platform/remarkable/rm-hwmon/rm_hwmon_keyboard.ko
make modules_prepare
exit # the driver should be compiled by this point, and you can safely exit the container's shell
```

On the remarkable pro tablet, this mount command allows you copy to /lib/modules as it’s usually mounted read only.
note replace `remarkable-pro-tablet` with the ip address or hostname of your remarkable pro tablet

### Installation
````
sftp root@remarkable-pro-tablet
put rm_hwmon_keyboard.ko
ssh root@remarkable-pro-tablet
mount -o rw,remount /

cp rm_hwmon_keyboard.ko /lib/modules/6.1.55+git+e1b4ca6065-imx8mm-ferrari/kernel/drivers/platform/remarkable/rm-hwmon/rm_hwmon_keyboard.ko
````

