## Robot Car Race Timer System
The project was developed as a simple automatic timing system for 6th grade computer science robot car races, but has many other potential applications.

![MainPage](https://github.com/user-attachments/assets/3a7efcf0-fd8a-4053-9d66-3ea6d5c9c996)

It uses and ESP32, MAX7219 dipslay and VL53L0X sensors to determine when an object crosses the start or finish line and starts or stops the timer (with 0.1 second resolution), shown on the matrix display.  Use of the VL53L0X distance sensors instead of motion allows movement around and near the 'race course' without triggering the sensors.  Optional LED lighting is also synced to the start, active and end or race phases.

The full version includes WiFi onboarding, a web interface for some settings and over-the-air (OTA) updates for future firmwware releases.  Note that support for the no-WiFi version was dropped beginning with version 0.22.  If you truly need a version that does not require or use WiFi, please see [release 0.21](https://github.com/Resinchem/Robot-Car-Timer/releases/tag/v0.21), which was the last version that supported a -noWiFi version.

You can watch a video on the build process here: [ESP32 Build: An Automated Timing System for Robot Racers](https://youtu.be/EBmh5WEJYhU)

Please refer to the [wiki](https://github.com/Resinchem/Robot-Car-Timer/wiki) (coming soon) for full information, including installation, setup, options and use of the system.
