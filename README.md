# Atari System 1

Atari-System-1-compatible FPGA core for Analogue Pocket.

Based on the FPGA Atari System 1 core by d18c7db (Alex), ported from the [MiSTer version](https://github.com/MiSTer-devel/Arcade-Atari-system1_MiSTer).

## Compatibility

Supports all five Atari System 1 games that run on the original FPGA core:

* Indiana Jones and the Temple of Doom
* Marble Madness
* Peter Pack-Rat
* Road Runner
* RoadBlasters

## Features

### Service Mode

Toggling "Service mode" in the interact menu will reset the game to its service menu.  From here you can set the difficulty, health-per-coin, and other game parameters, as well as run through the various diagnostics tests.

Toggle "service mode" off to reset the game with the settings applied.  

Note: settings do not persist after exiting the core

### Control Options for Marble Madness

In handheld mode, the Marble is controlled with the dpad.

When docked and paired with a controller with analog controls, the left analog stick can be used to precisely control the Marble's speed and direction.

### Analog controls for RoadBlasters

When docked and paired with a controller with analog controls, the left analog stick can be used for precise steering, while the right analog stick can be used to control the throttle.

### Analog controls for Road Runner

When docked and paired with a controller with analog controls, the left analog stick can be used for movement. 

### Control Sensitivity

Control sensitivity can be selected for games featuring analog controls (Road Runner, RoadBlasters, and Marble Madness).  

Three sensitivity settings are available from the interact menu - low, medium, and high.  

- High-sensitivity controls allow for faster speeds and direction changes, at the expense of precision. 
- Low-sensitivity controls allow for greater precision of movement, at the expense of maximum speed.

## Usage

*No ROM files are included with this release.*  

Install the contents of the release to the root of the SD card.

Place the `.rom` files for the supported games onto the SD card under `Assets/system1/common`.

Note that generating the `.rom` format binaries used by this core is a bit more complex than usual.

First, you must run the associated per-game patching script to build intermediate zip files, containing assets from the base from the required ROMs corresponding to the files found in the most recent MAME release. These scripts pad the necessary roms to the required sizes, and place all dependencies in a single archive.

Then, these intermediate files can be processed using mra-tool with the provided `.mra` files to generate the correct `.rom` files accepted by the core.  

Note: these scripts have been tested on Linux; your mileage may vary on other targets.

Note: the `.mra` files provided are heaviliy modified from the original versions in MiSTeR due to the limitations of mra-tool (necessitating the rom pre-patching scripts). The original `.mra` files will not work for this core.

## History

v0.9.0
* Initial Release.

## License

All new materials for this port are licensed under the terms of the GPLv3.

Please see the headers and license files for the licensing terms of the individual dependencies.

## Attribution

```
Original Gauntlet FPGA-compatible core by d18c7db (Alex)
https://github.com/d18c7db/atari_system1_fpga
---
# JT51
YM2151 clone in verilog. FPGA proven.
(c) Jose Tejada 2016. Twitter: @topapate
(https://github.com/jotego/jt51)
```