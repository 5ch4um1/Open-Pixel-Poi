# Open Pixel Poi Firmware

##### Links
- [User Manual](./MANUAL.md)
- [Firmware Flashy Thing](https://mitchlol.github.io/#openpixelpoi)

##### Firmware info
This fork adds a new experimental firmware based on esp-idf that should be compatible with the latest app (reports firmware version 0x02).

The esp-idf firmware is based on **ESP-IDF release/5.3** and it is to be considered **experimental and untested. Voltage reading is implemented, but no battery safety checks are performed yet, no deep-sleep implemented yet, this firmware may permanently damage your device, or even set your house on fire, cause earthquakes, regional flooding, landslides, coastal errosion, avalanches or worse if used incorrectly.**

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

Experimental web app to test the streaming mode: https://5ch4um1.es/oppstream.html 
