# USB-Blaster Firmware for "REV. C USB BLASTER" board with CH552G chip.

This is a fork of [CH55x-USB-Blaster firmware by VladimirDuan](https://github.com/VladimirDuan/CH55x-USB-Blaster) that runs on a cheap clone Altera USB Blaster with CH552G chip.
It works correctly in Linux and Windows with my limited testing(I am testing with MAX V CPLD), no special drivers required. 

After fork vladimirduan repo and made some changes on locally to succesfully program our cpld product with "cheap clone Altera USB Blaster with CH552G" but after a few days I saw [this blog](https://www.downtowndougbrown.com/2024/06/fixing-a-knockoff-altera-usb-blaster-that-never-worked/) and our changes similar except some additional usb fixes(I don't interest linux anyway) but no one give build instruction or included sdk file etc. I decided to mirror @dougg3 repo and implement github action for automatic build and changed repo name for more searchable and then mxwiser decided to fork my repo and adding as mode and then hardware spi support. I don't remove any commit of @dougg3 anyway it's the story for this repo.

#
- This fork added all needed files for correct build.
- Implement CI to build and release binary files so you don't need to setup or download any build tools or sdk files.
- Support AS mode.
- Support hardware SPI now.

#
CH552G can be replaced by CH551G and CH554G, in fact these models have the same Die.

|   PIN   | CH552G |
|:-------:|:------:|
|   LED   |  P1.1  |
|   NCS   |  P1.4  |
|   TDI/ASDI   |  P1.5  |
|   TDO/nCfg-Done   |  P1.6  |
|   TCK/DCLK   |  P1.7  |
|   TMS/nCfg   |  P3.2  |
|   ASDO  |  P3.3  |
|   NCE   |  P3.4  |

Under normal circumstances, the JTAG interface is able to be compatible with the AS interface.
|   1     |   2    |
|:-------:|:------:|
|   TCK/DCLK   |  GND   |
|   TDO/nCfg-Done   |  NC    |
|   TMS/nCfg   | nCE |
|   ASDO    | nCS |
|   TDI/ASDI   |  GND   |

#
Classic JTAG Interface.
|   1     |   2    |
|:-------:|:------:|
|   TCK   |  GND   |
|   TDO   |  VCC   |
|   TMS   |  NC    |
|   NC    |  NC    |
|   TDI   |  GND   |

Classic AS Interface.
|   1       |   2    |
|:---------:|:------:|
|    DCLK   |  GND   |
| nCfg-Done |  VCC   |
|    nCfg   |  nCE   |
|    ASDO   |  nCS   |
|    ASDI   |  GND   |


