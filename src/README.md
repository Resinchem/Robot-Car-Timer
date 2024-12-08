### If you just want to install the firmware on your ESP32, you do not need to download the source code!
Just download the latest **RobotCarTimer_vx.xx_ESP32.bin** file from the [Releases page](https://github.com/Resinchem/Robot-Car-Timer/releases) and follow the installation and instructions that are found in the [wiki](https://github.com/Resinchem/Robot-Car-Timer/wiki).  Please try the steps in the wiki before posting questions about how to install the firmware.

## Source Code
Two versions of the source code are available.  _You only need one source code file!_

**racecar_timer.ino**

_This is the recommended version._ It contains all the features, including the ability to make changes to some options and settings using a web interface and to perform over-the-air (OTA) updates of the firmware wirelessly.  This is the only version supported after v0.21.

**racecar_nowifi.ino** (v0.21 and earlier only)

This version should only be used if the intended installation location does not have WiFi available.  It obviously does not include any sort of web interface and therefore any changes to the settings and options require editing the source code, recompiling and then uploading via the Arduino IDE or other compatible application.  And since over-the-air updates are not available without WiFi, each update or change requires connecting the controller to a PC/USB port for uploading.  _Releases of the software after v0.21 do not include a non-WiFi version!_
