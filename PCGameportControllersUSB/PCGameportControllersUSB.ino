/*  DaemonBite PC Gameport controllers to USB Adapter
 *  Author: Mikael Norrgård <mick@daemonbite.com>
 *
 *  Copyright (c) 2020 Mikael Norrgård <http://daemonbite.com>
 *  
 *  GNU GENERAL PUBLIC LICENSE
 *  Version 3, 29 June 2007
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *  
 */

#include "Gamepad.h"
#include <avdweb_AnalogReadFast.h>

#define DEBUG

#define THRESHOLD 100
#define X1         0
#define Y1         1
#define X2         2
#define Y2         3
#define UNDEFINED  0
#define R100K      1
#define R150K      2
#define R200K      3
#define ANALOGPIN0 18

#define GRIP_SYNC  0x1f
#define RED        0x01
#define BLUE       0x02
#define YELLOW     0x04
#define GREEN      0x08
#define L1         0x10
#define R1         0x20
#define L2         0x40
#define R2         0x80
#define SELECT     0x100
#define START      0x200
#define UP         0x1000
#define DOWN       0x2000
#define LEFT       0x4000
#define RIGHT      0x8000

#define NOP __asm__ __volatile__ ("nop\n\t")

// Struct for Sidewinder data
typedef union
{
	struct
	{
		uint16_t bTrigger:1;       // bit  1

		uint16_t bTopLeft:1;       // bit  2
		uint16_t bTopRightUp:1;   // bit  3
		uint16_t bTopRightDown:1;// bit  4

		uint16_t bA:1;             // bit  5
		uint16_t bB:1;             // bit  6
		uint16_t bC:1;             // bit  7
		uint16_t bD:1;             // bit  8
		
		uint16_t bShift:1;         // bit  9

		uint16_t x:10;            // bits 10-19
		uint16_t y:10;            // bits 20-29
		uint16_t zRot:7;           // bits 30-36
		uint16_t throttle:6;      // bits 37-42

		uint16_t hat:4;           // bits 43-46

		uint16_t reserved:1;      // bit 47
		uint16_t parity:1;        // bit 48
	};

	// the ints are used to access the struct-data at bit-level
	uint8_t bytes[6];
} sw_data_t;

enum ControllerMode {
  None,
  Analog1P,
  Analog2P,
  GrIp,
  SidewinderGP,
  SidewinderFFW,
  SidewinderFSP,
  SidewinderPP,
  Sidewinder3DP
};

// ATT: 20 chars max (including NULL at the end) according to Arduino source code.
// Additionally serial number is used to differentiate arduino projects to have different button maps!
const char *gp_serial = "PC Gameport to USB";

// Set up USB HID gamepads
Gamepad_ Gamepad[2];

volatile ControllerMode mode[2] = {None,None};

const uint8_t axisPin[2][2] = {{A0,A1},{A2,A3}};
const uint16_t grIpBits[20] = {0,0, SELECT,START,R2,BLUE ,0, L2,GREEN,YELLOW,RED ,0, L1,R1,UP,DOWN ,0, RIGHT,LEFT ,0};
volatile uint8_t grIpCounter[2] = {0,0};

// Buttons and axes states
volatile uint16_t buttons[2] = {0,0};
uint16_t buttonsPrev[2] = {0,0};
uint16_t axisValueInitial[2][2] = {{0,0},{0,0}};
uint8_t axisPot[2][2] = {{UNDEFINED, UNDEFINED},{UNDEFINED, UNDEFINED}}; 
int8_t axes[2][4]     = {{0,0,0,0},{0,0,0,0}};
int8_t axesPrev[2][4] = {{0,0,0,0},{0,0,0,0}};
uint8_t gp = 0;
uint8_t reg = 0;

volatile uint8_t grIpByte[2] = {0,0};
volatile uint8_t counter[2] = {0,0};

float potX1 = 0.0;

#ifdef DEBUG
  uint32_t microsPrint = 0;
#endif

// -----------------------------------
// Setup
// -----------------------------------
void setup()
{ 
  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  // Set D0-D3 as inputs and enable pull-up resistors (Buttons 1-4 for joystick 1)
  DDRD  &= ~B00001111;
  PORTD |=  B00001111;

  // Set B1 and B3 as output low (39K pull-downs for axis pins)
  DDRB  |=  B00001010;
  PORTB &= ~B00001010;

  // Set B4 and E6 as input high-z (330Ω pull-downs for axis pins)
  DDRB  &= ~B00010000;
  PORTB &= ~B00010000;
  DDRE  &= ~B01000000;
  PORTE &= ~B01000000;

  // Set D4 and D7 as output low (MIDI OUT pins, used as low for buttons 3-4)
  DDRD  |=  B10010000; 
  PORTD &= ~B10010000;

  // Set C6 input and enable pull-up resistors (MIDI IN pin, currently unused)
  DDRC  &= ~B01000000;
  PORTC |=  B01000000;

  // Interrupt 0 for PD3 (button 1 of game port)
  EICRA &= ~(bit(ISC00) | bit(ISC01)); // Clear existing flags of interrupt 0 
  EICRA |= bit(ISC01);                 // Interrupt on falling
  EIMSK |= bit(INT0);                  // Enable INT0

  // Interrupt 1 for PD2 (button 3 of game port)
  EICRA &= ~(bit(ISC10) | bit (ISC11));// Clear existing flags of interrupt 0 
  EICRA |= bit(ISC11);                 // Interrupt on falling
  EIMSK |= bit(INT1);                  // Enable INT1

  // Dummy analog read
  analogReadFast(A0);

  // Reset controller(s)
  for(gp=0; gp<2; gp++)
    Gamepad[gp].reset();
  gp = 0;

  // Wait for the controller(s) to initialize
  delay(500);

  // Identify controller(s)
  identifyControllers();
}

// -----------------------------------
// Main loop
// -----------------------------------
void loop() { while(1)
{
  switch(mode[0])
  {
    // No controller connected
    // --------------------
    case None:
      break;
    
    // Analog with 4 buttons (one player)
    // --------------------
    case Analog1P:
      // Read buttons
      reg = ~PIND;
      buttons[0] = (reg & B00001001) | ((reg & B0000010)<<1) | ((reg & B0000100)>>1);
      // Read axes
      axes[0][X1] = adcTo100(axisPin[0][X1],axisPot[0][X1]);
      axes[0][Y1] = adcTo100(axisPin[0][Y1],axisPot[0][Y1]);
      if(axisPot[1][X1] != UNDEFINED)
        axes[0][X2] = adcTo100(axisPin[1][X1],axisPot[1][X1]);
      if(axisPot[1][Y1] != UNDEFINED)
        axes[0][Y2] = adcTo100(axisPin[1][Y1],axisPot[1][Y1]);
      break;

    // Gravis GrIp protocol
    // --------------------
    case GrIp:
      // X-Axis
      if(buttons[0] & LEFT) axes[0][X1] = -100;
      else if(buttons[0] & RIGHT) axes[0][X1] = 100;
      else axes[0][X1] = 0;
      // Y-Axis
      if(buttons[0] & UP) axes[0][Y1] = -100;
      else if(buttons[0] & DOWN) axes[0][Y1] = 100;
      else axes[0][Y1] = 0;
      break;

  }

  switch(mode[1])
  {
    // No controller connected
    // --------------------
    case None:
      break;
    
    // Analog with 4 buttons (one player)
    // --------------------
    case Analog1P:
      // Read buttons
      reg = ~PIND;
      buttons[1] = ((reg & B0000010)>>1) | ((reg & B00001000)>>2);
      // Read axes
      axes[1][X1] = adcTo100(axisPin[1][X1],axisPot[1][X1]);
      axes[1][Y1] = adcTo100(axisPin[1][Y1],axisPot[1][Y1]);
      break;

    // Gravis GrIp protocol
    // --------------------
    case GrIp:
      /*// X-Axis
      if(buttons[0] & LEFT) axes[0][X1] = -100;
      else if(buttons[0] & RIGHT) axes[0][X1] = 100;
      else axes[0][X1] = 0;
      // Y-Axis
      if(buttons[0] & UP) axes[0][Y1] = -100;
      else if(buttons[0] & DOWN) axes[0][Y1] = 100;
      else axes[0][Y1] = 0;*/
      break;
  }

  #ifdef DEBUG
    if((micros()-microsPrint)>1000000)
    {
      clearScreen();
      Serial.println("----------------------");
      //Serial.print("Pot X1: ");
      //Serial.println(potX1);
      Serial.print("Counter 1: ");
      Serial.println(counter[0]);
      Serial.print("Pot Init 1: ");
      Serial.println(axisValueInitial[0][0]);
      Serial.print("Pot Init 2: ");
      Serial.println(axisValueInitial[0][1]);
      Serial.print("Pot Init 3: ");
      Serial.println(axisValueInitial[1][0]);
      Serial.print("Pot Init 4: ");
      Serial.println(axisValueInitial[1][1]);
      
      switch(mode[0])
      {
        case None:
          Serial.println("----------------------");
          Serial.println("Joy1: No controller detected");
          break;
        
        case Analog1P:
          Serial.println("----------------------");
          Serial.println("Joy1: Analog");
          Serial.println("AXIS1 AXIS2 AXIS3 AXIS4");
          Serial.print(potValueToString(axisPot[0][0]));
          Serial.print(potValueToString(axisPot[0][1]));
          Serial.print(potValueToString(axisPot[1][0]));
          Serial.println(potValueToString(axisPot[1][1]));
          Serial.print("X1: ");
          Serial.println(axes[0][X1]);
          Serial.print("Y1: ");
          Serial.println(axes[0][Y1]);
          Serial.print("X2: ");
          Serial.println(axes[0][X2]);
          Serial.print("Y2: ");
          Serial.println(axes[0][Y2]);
          Serial.println("Button 1 2 3 4");
          Serial.print(  "       ");
          digitalRead(3) ? Serial.print("  ")  : Serial.print("X ");
          digitalRead(0) ? Serial.print("  ")  : Serial.print("X ");
          digitalRead(2) ? Serial.print("  ")  : Serial.print("X ");
          digitalRead(1) ? Serial.println(" ") : Serial.println("X");
          break;

        case GrIp:
          Serial.println("----------------------");
          Serial.println("Joy1: Gravis GrIp");
          break;

        case SidewinderGP:
          Serial.println("----------------------");
          Serial.println("Joy1: Sidewinder Game Pad");
          break;

        case SidewinderFFW:
          Serial.println("----------------------");
          Serial.println("Joy1: Sidewinder Force Feedback Wheel");
          break;

        case SidewinderFSP:
          Serial.println("----------------------");
          Serial.println("Joy1: Sidewinder Freestyle Pro");
          break;

        case SidewinderPP:
          Serial.println("----------------------");
          Serial.println("Joy1: Sidewinder Precision Pro");
          break;

        case Sidewinder3DP:
          Serial.println("----------------------");
          Serial.println("Joy1: Sidewinder 3D Pro");
          break;
      }
      switch(mode[1])
      {
        case None:
          Serial.println("----------------------");
          Serial.println("Joy2: No controller detected");
          break;
        
        case Analog1P:
          Serial.println("----------------------");
          Serial.println("Joy2: Analog");
          Serial.println("AXIS1 AXIS2");
          Serial.print(potValueToString(axisPot[1][0]));
          Serial.println(potValueToString(axisPot[1][1]));
          Serial.print("X: ");
          Serial.println(axes[1][X1]);
          Serial.print("Y: ");
          Serial.println(axes[1][Y1]);
          Serial.println("Button 1 2");
          Serial.print(  "       ");
          digitalRead(2) ? Serial.print("  ")  : Serial.print("X ");
          digitalRead(1) ? Serial.println(" ") : Serial.println("X");
          break;

        case GrIp:
          Serial.println("----------------------");
          Serial.println("Joy2: Gravis GrIp");
          break;
      }
      microsPrint = micros();
    }
  #endif
  
  //for(gp=0; gp<JOYSTICK_COUNT; gp++)
  //{
    // Has any buttons changed state?
    if (buttons[0] != buttonsPrev[0] || axes[0][X1] != axesPrev[0][X1] || axes[0][Y1] != axesPrev[0][Y1] || axes[0][X2] != axesPrev[0][X2] || axes[0][Y2] != axesPrev[0][Y2])
    {
      Gamepad[0]._GamepadReport.buttons = buttons[0];
      Gamepad[0]._GamepadReport.x = axes[0][X1];
      Gamepad[0]._GamepadReport.y = axes[0][Y1];
      Gamepad[0]._GamepadReport.z = axes[0][X2];
      Gamepad[0]._GamepadReport.slider = axes[0][Y2];
      buttonsPrev[0] = buttons[0];
      axesPrev[0][X1] = axes[0][X1];
      axesPrev[0][Y1] = axes[0][Y1];
      Gamepad[0].send();
    }
  //}

}}

// -----------------------------------
// Interrupt 0 routine
// -----------------------------------
ISR (INT0_vect)
{
  switch(mode[0])
  {
    // Gravis GrIp protocol
    // --------------------
    case GrIp:
      // Shift sync byte left one bit
      grIpByte[0] <<= 1; 
      // Set first bit of grIpByte
      if(PIND & 0x04) 
        grIpByte[0] |= 1;
      // Reset counter to zero when sync frame is detected
      if((grIpByte[0] & GRIP_SYNC) == GRIP_SYNC) grIpCounter[0] = 0;
      // Set button
      if(grIpCounter[0] < 20 && grIpBits[grIpCounter[0]] > 0) {
        if(grIpByte[0] & 0x01) buttons[0] |= grIpBits[grIpCounter[0]]; else buttons[0] &= ~grIpBits[grIpCounter[0]];
      }
      // Increase button counter
      grIpCounter[0]++;
      break;

    // Sidewinder Gamepad
    // --------------------
    case SidewinderGP:
      // TODO: implement
      break;

    // None, for detecting controller
    // --------------------
    case None:
      if(counter[0]<255)
        counter[0]++;
      break;
  }
}

// -----------------------------------
// Interrupt 1 routine
// -----------------------------------
ISR (INT1_vect)
{
  switch(mode[1])
  {
    // Gravis GrIp protocol
    // --------------------
    case GrIp:
      // Shift sync byte left one bit
      grIpByte[1] <<= 1; 
      // Set first bit of grIpByte
      if(PIND & 0x04) 
        grIpByte[1] |= 1;
      // Reset counter to zero when sync frame is detected
      if((grIpByte[1] & GRIP_SYNC) == GRIP_SYNC) grIpCounter[1] = 0;
      // Set button
      if(grIpCounter[1] < 20 && grIpBits[grIpCounter[1]] > 0) {
        if(grIpByte[1] & 0x01) buttons[0] |= grIpBits[grIpCounter[1]]; else buttons[0] &= ~grIpBits[grIpCounter[1]];
      }
      // Increase button counter
      grIpCounter[1]++;
      break;

    // Sidewinder Gamepad
    // --------------------
    case SidewinderGP:
      // TODO: implement
      break;

    // None, for detecting controller
    // --------------------
    case None:
      if(counter[1]<255)
        counter[1]++;
      break;
  }
}

// -----------------------------------
// Linearize analog read to -100..100
// -----------------------------------
int8_t adcTo100(uint8_t pin, uint8_t potValue)
{
  uint16_t adc = 0;
  int32_t retval;

  // Calculate average of four analog reads
  for(uint8_t i=0; i<4; i++)
    adc += analogReadFast(pin);
  adc = adc >> 2; // Divide by 4

  if(adc < THRESHOLD)
    return 0; // Joystick probably not connected

  // Linearize value
  switch(potValue)
  {
    case R100K:
	    retval = ((int32_t)79794 / adc) - 178;
      break;
    case R150K:
	    retval = ((int32_t)53300 / adc) - 152;
      break;
    case R200K:
	    retval = ((int32_t)39900 / adc) - 139;
      break;
  }

  // Clamp and return
	if (retval < -100)
		return -100;
	else if (retval > 100)
		return 100;
	else
		return retval;
}

// -----------------------------------
// Identify the connected controller
// -----------------------------------
void identifyControllers()
{
  uint16_t analogValue = 0;
  float voltage = 0.0;
  float potValue = 0.0; 
  unsigned long millisNow = 0;

  // Check for Gravis Gamepad Pro on both "channels"
  millisNow = millis();
  // Reset counters
  counter[0] = 0;
  counter[1] = 0;
  // Wait for 100ms
  millisNow = millis();
  while((millis()-millisNow) <= 100)
    NOP;
  // Check counters to see if value has increased
  if(counter[0] >= 30) mode[0] = GrIp;
  if(counter[1] >= 30) mode[1] = GrIp;
  
  // Check for available analog axes
  for(int8_t i=0; i<4; i++)
  {
    analogValue = analogRead(ANALOGPIN0+i);
    axisValueInitial[i/2][i%2] = analogValue;
    voltage = analogValue / 204.6; // 0-1023 -> 0-5V value
    potValue = 195000.0 / voltage - 39000.0;
    if(i==0)
      potX1 = potValue*2.0;
    if(analogValue < THRESHOLD)
      axisPot[i/2][i%2] = UNDEFINED;
    else if(potValue > 90000.0)
      axisPot[i/2][i%2] = R200K;
    else if(potValue > 63000.0)
      axisPot[i/2][i%2] = R150K;
    else
      axisPot[i/2][i%2] = R100K;
  }
  
  // Note: If needing to switch back to analog input for A0, set pin as input hi-z (disabled pull-up)
  //       before doing an analog read.

  // Check for Sidewinder range of controllers (port 1 only)
  if(mode[0] == None) //&& (PIND & B00000101) == 0) // Check if both button 1 and 2 pins are low
  {
    if(axisValueInitial[0][0] < 800)
    {
      EIMSK &= ~bit(INT0);                 // Disable INT0 interrupt
      EIFR  |= bit(INTF0);                 // Clear INT0 interrupt flag
      EICRA &= ~(bit(ISC00) | bit(ISC01)); // Clear existing flags of interrupt 0 
      EICRA |= bit (ISC00) | bit (ISC01);  // Set rising level interrupt

      PORTF |=  B10000000;     // Set A0 (PF7) HIGH (Will be used as trigger for the Sidewinder to send data)
      DDRF  |=  B10000000;     // Set A0 as digital output (will be high)
      delay(100);              // Wait a while
      PORTF &= ~B10000000;     // Set A0 LOW (Start trigger)
      delayMicroseconds(40);   // Wait for the clock line to go low (not needed on all SW models)
      while((PIND & B00000001) == 0) // Wait until clock pins becomes high 
        NOP;
      delayMicroseconds(50);   // Wait a while still
      PORTF |=  B10000000;     // Set A0 HIGH (End trigger)
      counter[0] = 0;          // Reset counter (will count clock pulses)
      EIMSK |= bit(INT0);      // Enable INT0

      // Wait for 100ms
      millisNow = millis();
      while((millis()-millisNow) <= 100)
        NOP;

      // Detect Sidewinder controller depending on number of clock pulses received
      switch(counter[0])
      {
        // Sidewinder Gamepad
        case 16:
          mode[0] = SidewinderGP;
          break;

        // Sidewinder Force Feedback Wheel
        case 32:
          mode[0] = SidewinderFFW;
          break;

        // Sidewinder Freestyle Pro
        case 44:
          mode[0] = SidewinderFSP;
          break;

        // Sidewinder Precision Pro and Force Feedback Pro joysticks
        case 48:
          mode[0] = SidewinderPP;
          break;

        // Sidewinder 3D Pro Joystick
        case 64:
          mode[0] = Sidewinder3DP;
          break;  

        // No Sidewinder detected
        default:
          DDRF  &= ~B10000000; // Set A0 as input
          PORTF &= ~B10000000; // Disable pull-up for A0 so analog readings will be correct
          break;
      }
    }
  }

  // Set Joy1 as analog?
  if(mode[0] == None && axisPot[0] != UNDEFINED)
    mode[0] = Analog1P;

  // Set Joy2 as analog?
  if(mode[0] == GrIp && mode[1] == None && axisPot[2] != UNDEFINED)
    mode[1] = Analog1P;
}

// -----------------------------------
// Debug functions
// -----------------------------------
#ifdef DEBUG
void clearScreen()
{
  Serial.write(27);      
  Serial.print("[2J");
  Serial.write(27);
  Serial.print("[H");
}

char* potValueToString(uint8_t potValue)
{
  switch(potValue)
  {
    case 0:
      return "NONE  ";
      break;
    case 1:
      return "100K  ";
      break;
    case 2:
      return "150K  ";
      break;
    case 3:
      return "200K  ";
      break;
  }
}
#endif
