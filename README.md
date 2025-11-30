# esp-pong
Wireless 2-player pong using 2 ESP32s and the ESP-NOW protocol

https://github.com/user-attachments/assets/b9c66c18-920a-4ca2-b1d6-fbd211c6f186

## Components
- ESP-WROOM32 (2)
- 128x64 Monochrome I2C display (SSD1306)
- Push button (4)
- 10k ohm resistor (4)

## Wiring
The default pin and circuit configuration for both Player 1 and Player 2 is as follows:
| ESP32 Pin | Component |
| --- | --- |
| GND | Display GND |
| GND | 10k ohm (2) <- button (2) |
| 5V | Display VDD |
| 26 | Display SCK |
| 27 | Display SDA | 
| 32 | Left button |
| 33 | Right button |

## How to run
1. Install Visual Studio Code with the PlatformIO extension
2. Clone the repo in desired directory
```
git clone https://github.com/j0of/esp-pong
```
3. Open esp-pong/player1 in VSCode
4. PlatformIO -> Upload and monitor to ONE ESP32 and note down the MAC address shown in terminal
5. Repeat with esp-pong/player2 using the second ESP32
6. Replace the respective MAC addresses in src/main.cpp:19 for both player1 and player2
7. Re-build and flash both ESP32s (NOTE : the ESP32 you used and the code you flashed the first time must be the same - if you uploaded player1 to one chip the first time, you must upload player 1 again)
8. Profit
