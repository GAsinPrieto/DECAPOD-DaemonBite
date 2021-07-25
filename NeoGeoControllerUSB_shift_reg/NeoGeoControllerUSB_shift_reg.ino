/*  NeoGeo Controller to USB
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
#include "shift_74597.h"

#define GAMEPAD_COUNT 2      // NOTE: No more than TWO gamepads are possible at the moment due to a USB HID issue.

#define QH 2 //PD1 IN
#define SCK 4 //PD4 OUT
#define RCK 9 //PB5 OUT
#define SLOAD 9 //SRLOAD

#define DEBOUNCE 0          // 1=Diddly-squat-Delay-Debouncing™ activated, 0=Debounce deactivated
#define DEBOUNCE_TIME 10    // Debounce time in milliseconds
//#define DEBUG             // Enables debugging (sends debug data to usb serial)

const char *gp_serial = "NeoGeo to USB";

uint8_t gp = 0;


/*uint8_t myPin_mask;
volatile uint8_t *myPin_port;*/
shift_74597 myShifter = shift_74597(QH, SCK, RCK, SLOAD);//, SCLR);


Gamepad_ Gamepad[GAMEPAD_COUNT];           // Set up USB HID gamepad
/*bool usbUpdate1 = false;     // Should gamepad data be sent to USB?
bool usbUpdate2 = false;     // Should gamepad data be sent to USB?
*/bool usbUpdate = false;     // Should gamepad data be sent to USB?
bool debounce = DEBOUNCE;   // Debounce?
uint8_t  pin;               // Used in for loops
uint32_t millisNow = 0;     // Used for Diddly-squat-Delay-Debouncing™

/*uint8_t  axesDirect1 = 0x0f;
uint8_t  axesDirect2 = 0x0f;
uint8_t  axes1 = 0x0f;
uint8_t  axes2 = 0x0f;
uint8_t  axesPrev1 = 0x0f;
uint8_t  axesPrev2 = 0x0f;
*/uint8_t  axesDirect[] = {0x0f,0x0f};
uint8_t  axes[] = {0x0f,0x0f};
uint8_t  axesPrev[] = {0x0f,0x0f};
uint8_t  axesBits[4] = {0x10,0x20,0x40,0x80};
uint32_t axesMillis[4];

/*uint16_t buttonsDirect1 = 0;
uint16_t buttonsDirect2 = 0;
uint16_t buttons1 = 0;
uint16_t buttons2 = 0;
uint16_t buttonsPrev1 = 0;
uint16_t buttonsPrev2 = 0;
*/uint16_t buttonsDirect[] = {0,0};
uint16_t buttons[] = {0,0};
uint16_t buttonsPrev[] = {0,0};
uint16_t buttonsBits[12] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x100,0x200,0x400,0x800};
uint32_t buttonsMillis[12];

char myInput0;
char myInput1;
char myInput2;

#ifdef DEBUG
  char buf[16];
  uint32_t millisSent = 0;
#endif

uint8_t reverse(uint8_t in)
{
  uint8_t out;
  out = 0;
  if (in & 0x01) out |= 0x80;
  if (in & 0x02) out |= 0x40;
  if (in & 0x04) out |= 0x20;
  if (in & 0x08) out |= 0x10;
  if (in & 0x10) out |= 0x08;
  if (in & 0x20) out |= 0x04;
  if (in & 0x40) out |= 0x02;
  if (in & 0x80) out |= 0x01;

  return(out);
}

void setup() 
{
  // Axes
  /*DDRF  &= ~B11110000; // Set A0-A3 as inputs
  PORTF |=  B11110000; // Enable internal pull-up resistors

  // Buttons
  DDRD  &= ~B10011111; // Set PD0-PD4 and PD7 as inputs// PD0-PD4 noPD7
  PORTD |=  B10011111; // Enable internal pull-up resistors
  DDRB  &= ~B01111110; // Set PB1-PB6 as inputs //PB1, noPB2-6
  PORTB |=  B01111110; // Enable internal pull-up resistors

  // Debounce selector switch (currently disabled)
  DDRE  &= ~B01000000; // Pin 7 as input
  PORTE |=  B01000000; // Enable internal pull-up resistor
*/
  // Initialize debouncing timestamps
  for(pin=0; pin<4; pin++)
    axesMillis[pin]=0;
  for(pin=0; pin<12; pin++)   
    buttonsMillis[pin]=0;

  #ifdef DEBUG
    Serial.begin(115200);
  #endif

    myShifter.init();//myPin_mask, &myPin_port);

}

void loop() 
{
  // Get current time, the millis() function should take about 2µs to complete
  millisNow = millis();

  myShifter.load();
  myInput0 = myShifter.getByte();
  myInput1 = myShifter.getByte();
  myInput2 = myShifter.getByte();

  //for(uint8_t i=0; i<10; i++) // One iteration (when debounce is enabled) takes approximately 35µs to complete, so we don't need to check the time between every iteration
  //{
    // Read axis and button inputs (bitwise NOT results in a 1 when button/axis pressed)
    axesDirect[0] = ~(reverse(myInput0 & B00001111));//~(PINF & B11110000);
    buttonsDirect[0] = ~((myInput0 & B11110000)>>4 | (myInput1 & B00001111)<<4 | (B11110000 << 4));//~((PIND & B00011111) | ((PIND & B10000000) << 4) | ((PINB & B01111110) << 4));

    axesDirect[1] = ~((reverse(myInput1 & B11110000))<<4);//~(PINF & B11110000);
    buttonsDirect[1] = ~((myInput2) | (B11110000 << 4));//~((PIND & B00011111) | ((PIND & B10000000) << 4) | ((PINB & B01111110) << 4));

    

    /*if(debounce)
    {
      // Debounce axes
      for(pin=0; pin<4; pin++)
      {
        // Check if the current pin state is different to the stored state and that enough time has passed since last change
        if((axesDirect & axesBits[pin]) != (axes & axesBits[pin]) && (millisNow - axesMillis[pin]) > DEBOUNCE_TIME)
        {
          // Toggle the pin, we can safely do this because we know the current state is different to the stored state
          axes ^= axesBits[pin];
          // Update the timestamp for the pin
          axesMillis[pin] = millisNow;
        }
      }
      
      // Debounce buttons
      for(pin=0; pin<12; pin++)
      {
        // Check if the current pin state is different to the stored state and that enough time has passed since last change
        if((buttonsDirect & buttonsBits[pin]) != (buttons & buttonsBits[pin]) && (millisNow - buttonsMillis[pin]) > DEBOUNCE_TIME)
        {
          // Toggle the pin, we can safely do this because we know the current state is different to the stored state
          buttons ^= buttonsBits[pin];
          // Update the timestamp for the pin
          buttonsMillis[pin] = millisNow;
        }
      }
    }
    else
    {*/

    for(gp = 0; gp < GAMEPAD_COUNT; gp++){
      axes[gp] = axesDirect[gp];
      buttons[gp] = buttonsDirect[gp];
      
    //}
  
    // Has axis inputs changed?
    if(axes[gp] != axesPrev[gp])
    {
      // UP + DOWN = UP, SOCD (Simultaneous Opposite Cardinal Directions) Cleaner
      if(axes[gp] & B10000000)
        Gamepad[gp]._GamepadReport.Y = -1;
      else if(axes[gp] & B01000000)
        Gamepad[gp]._GamepadReport.Y = 1;
      else
        Gamepad[gp]._GamepadReport.Y = 0;
      // UP + DOWN = NEUTRAL
      //Gamepad._GamepadReport.Y = ((axes & B01000000)>>6) - ((axes & B10000000)>>7);
      // LEFT + RIGHT = NEUTRAL
      Gamepad[gp]._GamepadReport.X = ((axes[gp] & B00010000)>>4) - ((axes[gp] & B00100000)>>5);
      axesPrev[gp] = axes[gp];
      usbUpdate = true;
    }

    /*if(axes2 != axesPrev2)
    {
      // UP + DOWN = UP, SOCD (Simultaneous Opposite Cardinal Directions) Cleaner
      if(axes2 & B10000000)
        Gamepad[1]._GamepadReport.Y = -1;
      else if(axes2 & B01000000)
        Gamepad[1]._GamepadReport.Y = 1;
      else
        Gamepad[1]._GamepadReport.Y = 0;
      // UP + DOWN = NEUTRAL
      //Gamepad._GamepadReport.Y = ((axes & B01000000)>>6) - ((axes & B10000000)>>7);
      // LEFT + RIGHT = NEUTRAL
      Gamepad[1]._GamepadReport.X = ((axes2 & B00010000)>>4) - ((axes2 & B00100000)>>5);
      axesPrev2 = axes2;
      usbUpdate2 = true;
    }*/
  
    // Has button inputs changed?
    if(buttons[gp] != buttonsPrev[gp])
    {
      Gamepad[gp]._GamepadReport.buttons = buttons[gp];
      buttonsPrev[gp] = buttons[gp];
      usbUpdate = true;
    }

    /*if(buttons2 != buttonsPrev2)
    {
      Gamepad[1]._GamepadReport.buttons = buttons2;
      buttonsPrev2 = buttons2;
      usbUpdate2 = true;
    }*/
  
    // Should gamepad data be sent to USB?
    if(usbUpdate)
    {
      //if(usbUpdate1){
        Gamepad[gp].send();
        usbUpdate = false;
      /*}
      if(usbUpdate2){
        Gamepad[1].send();
        usbUpdate2 = false;
      }*/

      #ifdef DEBUG
        sprintf(buf, "%06lu: %d%d%d%d", millisNow-millisSent, ((axes & 0x10)>>4), ((axes & 0x20)>>5), ((axes & 0x40)>>6), ((axes & 0x80)>>7) );
        Serial.print(buf);
        sprintf(buf, " %d%d%d%d", (buttons & 0x01), ((buttons & 0x02)>>1), ((buttons & 0x04)>>2), ((buttons & 0x08)>>3) );
        Serial.println(buf);
        millisSent = millisNow;
      #endif
    }
  }
 
}
