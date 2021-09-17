#include "Gamepad.h"
#include "SegaControllers32U4.h"

int GAMEPAD_COUNT = 1;		// NOTE: No more than TWO gamepads are possible at the moment due to a USB HID issue.
#define GAMEPAD_COUNT_MAX 4  // NOTE: For some reason, can't have more than two gamepads without serial breaking. Can someone figure out why?
//       (It has something to do with how Arduino handles HID devices)
#define BUTTON_COUNT       8 // Standard NES controller has four buttons and four axes, totalling 8
#define BUTTON_READ_DELAY 20 // Delay between button reads in µs

//GENESIS
SegaControllers32U4 controllers;

// Controller previous states
word lastState[2] = {1, 1};

#define UP    0x01
#define DOWN  0x02
#define LEFT  0x04
#define RIGHT 0x08

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

// Controllers
uint8_t buttons[GAMEPAD_COUNT_MAX] = {0, 0, 0, 0};
uint8_t buttonsPrev[GAMEPAD_COUNT_MAX] = {0, 0, 0, 0};
//uint8_t gpBit[GAMEPAD_COUNT_MAX] = {B10000000,B01000000,B00100000,B00010000};
uint8_t gpBit[GAMEPAD_COUNT_MAX] = {B00000010, B00000100, B00001000, B00010000};

uint8_t gp = 0;

// Timing
uint32_t microsButtons = 0;

const char *gp_serial = "DECAPOD";


Gamepad_ Gamepad[1];


void setup() {

}

void loop() {

  for (byte gp = 0; gp <= 1; gp++)
    Gamepad[gp].reset();
  DDRD  |=  B00000001; // output

  while (1)
  {
    //Serial.println("GENESIS");
    /*controllers.readState1();
      gp = 0;
      if (controllers.currentState[gp] != lastState[gp])
      {
      Gamepad[gp]._GamepadReport_GENESIS.buttons = controllers.currentState[gp] >> 4;
      Gamepad[gp]._GamepadReport_GENESIS.Y = ((controllers.currentState[gp] & SC_BTN_DOWN) >> SC_BIT_SH_DOWN) - ((controllers.currentState[gp] & SC_BTN_UP) >> SC_BIT_SH_UP);
      Gamepad[gp]._GamepadReport_GENESIS.X = ((controllers.currentState[gp] & SC_BTN_RIGHT) >> SC_BIT_SH_RIGHT) - ((controllers.currentState[gp] & SC_BTN_LEFT) >> SC_BIT_SH_LEFT);
      Gamepad[gp].send();
      lastState[gp] = controllers.currentState[gp];
      }


      controllers.readState2();
      gp = 1;
      if (controllers.currentState[gp] != lastState[gp])
      {
      Gamepad[gp]._GamepadReport_GENESIS.buttons = controllers.currentState[gp] >> 4;
      Gamepad[gp]._GamepadReport_GENESIS.Y = ((controllers.currentState[gp] & SC_BTN_DOWN) >> SC_BIT_SH_DOWN) - ((controllers.currentState[gp] & SC_BTN_UP) >> SC_BIT_SH_UP);
      Gamepad[gp]._GamepadReport_GENESIS.X = ((controllers.currentState[gp] & SC_BTN_RIGHT) >> SC_BIT_SH_RIGHT) - ((controllers.currentState[gp] & SC_BTN_LEFT) >> SC_BIT_SH_LEFT);
      Gamepad[gp].send();
      lastState[gp] = controllers.currentState[gp];
      }*/


    for (gp = 0; gp < GAMEPAD_COUNT; gp++) {

      /*if (gp==0) controllers.readState1();
        else controllers.readState2();*/

      controllers.readState(gp);

      if (controllers.currentState[gp] != lastState[gp])
      {
        Gamepad[gp]._GamepadReport.buttons = controllers.currentState[gp] >> 4;
        Gamepad[gp]._GamepadReport.Y = ((controllers.currentState[gp] & SC_BTN_DOWN) >> SC_BIT_SH_DOWN) - ((controllers.currentState[gp] & SC_BTN_UP) >> SC_BIT_SH_UP);
        Gamepad[gp]._GamepadReport.X = ((controllers.currentState[gp] & SC_BTN_RIGHT) >> SC_BIT_SH_RIGHT) - ((controllers.currentState[gp] & SC_BTN_LEFT) >> SC_BIT_SH_LEFT);
        Gamepad[gp].send();
        lastState[gp] = controllers.currentState[gp];
      }
    }
  }

  delay(10);
}
