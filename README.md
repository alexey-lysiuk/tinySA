tinySA - tiny Spectrum Analyzer
==========================================================

[![GitHub release](http://img.shields.io/github/release/erikkaashoek/tinySA.svg?style=flat)][release]
[![CircleCI](https://circleci.com/gh/erikkaashoek/tinySA.svg?style=shield)](https://circleci.com/gh/erikkaashoek/tinySA)

[release]: https://github.com/erikkaashoek/tinySA/releases

<div align="center">
<img src="/doc/tinySA.jpg" width="480px">
</div>

# About

tinySA is very tiny handheld Spectrum Analyzer (SA). It is
standalone with lcd display, portable device with battery. This
project aim to provide an useful instrument for the RF 
enthusiast.

This repository contains source of tinySA firmware.

# Support

General tinySA support questions should be posted here: https://groups.io/g/tinysa/messages

Use github issue list only for firmware bugs and preferrably cross post to: https://groups.io/g/tinysa/messages

## Prepare ARM Cross Tools

The released firmware is built with **Arm GNU Toolchain 11.3.Rel1** (arm-none-eabi-gcc 11.3.1 20220712), on Windows the "GNU Tools ARM Embedded\11.3 2022.08" toolchain of ChibiStudio. Use exactly this version. Parts of the firmware depend on how fast the compiled code runs, and firmware built with other gcc versions is known to show noise or fail the self test (issues #70 and #152).

All versions are on the [Arm GNU Toolchain downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) page. Check the version in use with

    $ arm-none-eabi-gcc --version

### MacOSX

Install the cross tools and the firmware updating tool.

    $ curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/11.3.rel1/binrel/arm-gnu-toolchain-11.3.rel1-darwin-x86_64-arm-none-eabi.tar.xz
    $ sudo tar xfJ arm-gnu-toolchain-11.3.rel1-darwin-x86_64-arm-none-eabi.tar.xz -C /usr/local
    $ PATH=/usr/local/arm-gnu-toolchain-11.3.rel1-darwin-x86_64-arm-none-eabi/bin:$PATH
    $ brew install dfu-util

### Linux (ubuntu)

    $ wget https://developer.arm.com/-/media/Files/downloads/gnu/11.3.rel1/binrel/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi.tar.xz
    $ sudo tar xfJ arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi.tar.xz -C /usr/local
    $ PATH=/usr/local/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi/bin:$PATH
    $ sudo apt install -y dfu-util

## Fetch source code

Fetch source and submodule.

    $ git clone https://github.com/erikkaashoek/tinySA.git
    $ cd tinySA
    $ git submodule update --init --recursive

## Build

Just make in the directory.

    $ make

For tinySA Ultra use this command

    $ make TARGET="F303"

### Build firmware using docker

Using [this docker image](https://hub.docker.com/r/edy555/arm-embedded) and without installing arm toolchain, you can build the firmware. Note that this image has gcc 8.2, not the release toolchain (see above).

    $ cd tinySA
    $ docker run -it --rm -v $(PWD):/work edy555/arm-embedded:8.2 make

## Flash firmware

First, make device enter DFU mode by one of following methods.

* Jumper BOOT0 pin at powering device
* Select menu Config->DFU (needs recent firmware)

Then, flash firmware using dfu-util via USB.

    $ dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin

Or simply use make.

    $ make flash

### Update from the SD card (tinySA Ultra)

The tinySA Ultra can install a new firmware from its SD card, without DFU:

1. Copy the firmware `.bin` file to the root folder of the SD card, for example with STORAGE→USB DISK. Eject the drive when the copy is done.
2. Select STORAGE→UPDATE FIRMWARE, pick the file and confirm with UPDATE.
3. The tinySA checks the file, programs it in about 10 seconds and restarts. Do not switch it off or remove the SD card during the update. If the card is removed anyway, the update waits and continues when it is inserted again.

Settings, calibration and presets are kept. The shell command `fwupdate {file}` checks a file without changing anything, `fwupdate {file} flash` installs it.

Only firmware that contains the update header (from this version on) can be installed from the SD card; older versions and the tinySA basic firmware are refused. If an update is interrupted, the tinySA does not start: hold the jog button while switching on to enter DFU mode and flash the firmware with a PC as above.

## Companion Tools

There are several numbers of great companion PC tools from third-party.

* [Soon to come](https://github.com/erikkaashoek/tinySA-Win) by Erik
* [tinySASaver](https://github.com/erikkaashoek/tinySA-saver) by Erik

## Documentation

* [tinySA User Guide](https://tinySA.org/wiki/)

## Reference

* [Specification](https://tinysa.org/wiki/pmwiki.php?n=Main.Specification)
* [Technical info](https://tinysa.org/wiki/pmwiki.php?n=Main.TechnicalDescription)

## Note

tinySA is a trademark owned by its respective owner. Unauthorized use the the name tinySA not permitted

## Authorized Distributor

* [See the Wiki](https://tinysa.org/wiki/pmwiki.php?n=Main.Buying)

## Credit

* [@erikkaashoek](https://github.com/erikkaashoek)

### Contributors

* [@edy555](https://github.com/edy555)
* [@hugen79](https://github.com/hugen79)
* [@cho45](https://github.com/cho45)
* [@DiSlord](https://github.com/DiSlord/)
