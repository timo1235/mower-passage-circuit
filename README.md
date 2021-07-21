# Mower passage code

This code is for controlling a passage for a mower.
The whole documentation can be found at [https://blog.altholtmann.com/passagenschaltung-rasenroboter/](https://blog.altholtmann.com/passagenschaltung-rasenroboter/)

## Sketches
The two sketches under `Mower/Mower_Sketch` and `Switch/Switch_Sketch` are projects for [PlatformIO and VSCode](https://platformio.org/install/ide?install=vscode).
But using the [Arduino IDE for ESP32](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/) is also possible. If using the
Arduino IDE copy the code from `~/src/main.cpp` to the Arduino IDE and compile. If the I2C display should be used, the library U8G2 has to be added to the IDE -> Use
google if you dont know how.

## Wiring
### Mower
[Mower wiring](Mower/mower_connectio_scheme.jpg "Mower connection scheme")
### Switch box
[Switch box wiring](Switch/switch_connection_scheme.jpg "Switch connection scheme")

## Debugging
If having any troubles with the functionalities not working, you can uncomment the debugging line and watch the serial monitor for debug output:
```
// Debugging. If enabled, the sketch will print DEBUGGING states to the serial monitor
// with 115200 Baudrate
// Default: commented
#define DEBUGGING // Uncomment to output DEBUGGING messages
```