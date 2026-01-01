# Open Pixel Poi Firmware

##### Links
- [User Manual](./MANUAL.md)
- [Firmware Flashy Thing](https://mitchlol.github.io/#openpixelpoi)

##### Firmware info
This fork preserves the original Arduino firmware, and adds a new experimental firmware based on esp-idf that should be compatible with the latest app (reports firmware version 0x02).

Platformio should get all the correct dependencies automatically based on the config, but in case it doesn't here are the details.
It used the adafruit neo pixel library 1.12.5.
It requires the esp32 boards to be installed, I reccomend v2.0.12 as newer versions cause led glitches.
When builiding, use the XIAO_ESP32C3 board for the gpio config to line up. All the other board settings can be default.

The esp-idf firmware added by me is based on **ESP-IDF release/5.3** and it is to be considered **experimental and untested. Voltage reading is implemented, but no battery safety checks are performed yet, no deep-sleep implemented yet, this firmware may permanently damage your device, or even set your house on fire if used incorrectly.**

I repeat: **Danger! Do not use the esp-idf firmware, especially not when running on battery!!!** Only use it for testing purposes with a stable power supply, if at all.
Or add the  emergency shutdown logic yourself.\
\
Setup on Linux and Mac: https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/linux-macos-setup.html

Setup on Windows: Just push some buttons in VScode, as you always do.

Clone this repository with:\
```git clone -b ci --single-branch https://github.com/5ch4um1/Open-Pixel-Poi.git```

Switch to your esp-idf dev environment (if not done already, I'd recommend to set up the alias) or in VScode there is a button called "open esp-idf shell" or something like that, good luck with finding it.

Build with ```idf.py build``` flash with ```idf.py flash``` and check debug messages via USB with ```idf.py monitor```

In case you missed all the other warnigns above: **Do NOT use the esp-idf firmware unless you really know what you are doing.**
