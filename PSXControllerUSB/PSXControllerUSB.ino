/*******************************************************************************
 * This file is part of PsxNewLib.                                             *
 *                                                                             *
 * Copyright (C) 2019-2020 by SukkoPera <software@sukkology.net>               *
 *                                                                             *
 * PsxNewLib is free software: you can redistribute it and/or                  *
 * modify it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * PsxNewLib is distributed in the hope that it will be useful,                *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with PsxNewLib. If not, see http://www.gnu.org/licenses.              *
 *******************************************************************************
 *
 * This sketch shows how the library can be used to turn a PSX controller into
 * an USB one, using an Arduino Leonardo or Micro and the excellent
 * ArduinoJoystickLibrary.
 *
 * For details on ArduinoJoystickLibrary, see
 * https://github.com/MHeironimus/ArduinoJoystickLibrary.
 */

#include <PsxControllerBitBang.h>
#include "Gamepad.h"

const char *gp_serial = "PSX to USB";

const byte PIN_PS2_ATT = 3;
const byte PIN_PS2_CMD = 2;
const byte PIN_PS2_DAT = 18;
const byte PIN_PS2_CLK = 4;

const unsigned long POLLING_INTERVAL = 1000U / 50U;

PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;

boolean haveController = false;

#define deadify(var, thres) (abs (var) > thres ? (var) : 0)

const byte ANALOG_DEAD_ZONE = 50U;

Gamepad_ Gamepad;
uint16_t buttons = 0;
uint16_t buttonsPrev = 0;
byte sticksPrev[4] = {0, 0, 0, 0};
void setup () {

}

void loop () {
	static unsigned long last = 0;
	
	if (millis () - last >= POLLING_INTERVAL) {
		last = millis ();
		
		if (!haveController) {
			if (psx.begin ()) {
				haveController = true;
			}
		} else {
			if (!psx.read ()) {
				haveController = false;
			} else {
        byte x,y;
        buttons = ((psx.buttonPressed (PSB_L2)) << 15) | ((psx.buttonPressed (PSB_R2)) << 14) | ((psx.buttonPressed (PSB_L1)) << 13) | ((psx.buttonPressed (PSB_R1)) << 12) | ((psx.buttonPressed (PSB_TRIANGLE)) << 11) | ((psx.buttonPressed (PSB_CIRCLE)) << 10) | ((psx.buttonPressed (PSB_CROSS)) << 9) | (((psx.buttonPressed (PSB_SQUARE)) ) << 8) | ((psx.buttonPressed (PSB_SELECT)) << 7) | ((psx.buttonPressed (PSB_R3)) << 6) | ((psx.buttonPressed (PSB_L3)) << 5) | ((psx.buttonPressed (PSB_START)) << 4) | ((psx.buttonPressed (PSB_PAD_UP)) << 3) | ((psx.buttonPressed (PSB_PAD_RIGHT)) << 2) | ((psx.buttonPressed (PSB_PAD_DOWN)) << 1) | (psx.buttonPressed (PSB_PAD_LEFT));
        if (buttons != buttonsPrev || buttons != buttonsPrev ) {
           Gamepad._GamepadReport.buttons = buttons;
				}

				if (psx.getLeftAnalog (x, y)) {
          if (x != sticksPrev[0] || x != sticksPrev[0]) {
            Gamepad._GamepadReport.LX = map(x, ANALOG_MIN_VALUE, ANALOG_MAX_VALUE, -128, 127);
            sticksPrev[0] = x;
          }
          if (y != sticksPrev[1] || y != sticksPrev[1]) {
            Gamepad._GamepadReport.LY = map(y, ANALOG_MIN_VALUE, ANALOG_MAX_VALUE, -128, 127);
            sticksPrev[1] = y;
          }
				}

        if (psx.getRightAnalog (x, y)) {
          if (x != sticksPrev[2] || x != sticksPrev[2]) {
            Gamepad._GamepadReport.RX = map(x, ANALOG_MIN_VALUE, ANALOG_MAX_VALUE, -128, 127);
            sticksPrev[2] = x;
          }
          if (y != sticksPrev[3] || y != sticksPrev[3]) {
            Gamepad._GamepadReport.RY = map(y, ANALOG_MIN_VALUE, ANALOG_MAX_VALUE, -128, 127);
            sticksPrev[3] = y;
          }
        }
        buttonsPrev = buttons;
        Gamepad.send();

			}
		}
	}
}
