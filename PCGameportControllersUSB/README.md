# DaemonBite PC Gameport Controllers To USB Adapter

## Introduction
This is a simple to build adapter for connecting PC (gameport) joysticks and other controllers to USB.

The input lag for this adapter is minimal (should be less 1ms average connected to MiSTer).

## Parts you need
- Arduino Pro Micro (ATMega32U4)
- One or two female 15-pin DSUB connectors
- Micro USB cable

## Wiring

| Gameport P1 (and P2 with splitter) | Arduino Pro Micro |
| ------ | ------ |
| 1 VCC | VCC |
| 2 Button 1 | 3 PD0 |
| 3 X-axis 1 | A0 |
| 4 GND | GND |
| 5 GND | GND |
| 6 Y-axis 1 | A1 |
| 7 Button 2 | RXI PD2 |
| 8 VCC | VCC |
| 9 VCC | VCC |
| 10 Button 3 | 2 PD1 |
| 11 X-axis 2 | A2 (optional) |
| 12 MIDI OUT / GND | 4 PD4 (or GND) |
| 13 Y-axis 2 | A3 (optional) |
| 14 Button 4 | TXO PD3 |
| 15 MIDI IN / VCC | 5 PC6 (or VCC)  |

NOTE THAT P2 IS NOT CORRECT AND MAY NOT BE USED CURRENTLY
| Gameport (P2) | Arduino Pro Micro |
| ------ | ------ |
| 1 VCC | VCC |
| 2 Button 1 | 15   PB1 |
| 3 X-axis 1 | A2 |
| 4 GND | GND |
| 5 GND | GND |
| 6 Y-axis 1 | A3 |
| 7 Button 2 | 16   PB2 |
| 8 VCC | VCC |
| 9 VCC | VCC |
| 10 Button 3 | 14   PB3 |
| 12 MIDI OUT / GND | 6 PD7 (or GND) |
| 14 Button 4 | 8   PB4 |
| 15 MIDI IN / VCC | 7 PE6 (or VCC)  |

Note: A0,A1,A2 and A3 needs 39KΩ pull-down resistors, the adapter will not work without these.

## License
This project is licensed under the GNU General Public License v3.0.
