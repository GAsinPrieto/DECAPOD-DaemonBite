#include "Gamepad.h"
#include "SegaControllers32U4.h"


#define NOT_SELECTED 0
#define NES_ 1
#define SNES_ 2
#define NEOGEO_ 3
#define PCE_ 4
#define GENESIS_ 5


//PCE
#define SELECT_PAUSE 20  // How many microseconds to wait after setting select/enable lines?
#define FRAME_TIME 16667 // The time of one "frame" in µs (used for turbo functionality)


#define DOWN_PCE     0x04
#define LEFT_PCE     0x08
#define RIGHT_PCE    0x02
#define UP_SH    0
#define DOWN_SH  2
#define LEFT_SH  3
#define RIGHT_SH 1





//NES
#define GAMEPAD_COUNT 2      // NOTE: No more than TWO gamepads are possible at the moment due to a USB HID issue.
#define GAMEPAD_COUNT_MAX 4  // NOTE: For some reason, can't have more than two gamepads without serial breaking. Can someone figure out why?
//       (It has something to do with how Arduino handles HID devices)
#define BUTTON_COUNT       8 // Standard NES controller has four buttons and four axes, totalling 8
#define BUTTON_READ_DELAY 20 // Delay between button reads in µs
#define MICROS_LATCH_NES       8 // 12µs according to specs (8 seems to work fine)
#define MICROS_CLOCK_NES       4 //  6µs according to specs (4 seems to work fine)
#define MICROS_PAUSE_NES       4 //  6µs according to specs (4 seems to work fine)

#define UP    0x01
#define DOWN  0x02
#define LEFT  0x04
#define RIGHT 0x08

// Set up USB HID gamepads
//Gamepad_ Gamepad[GAMEPAD_COUNT];

//SNES;
#define MICROS_LATCH_SNES      10 // 12µs according to specs (8 seems to work fine)
#define MICROS_CLOCK_SNES       5 //  6µs according to specs (4 seems to work fine)
#define MICROS_PAUSE_SNES       5 //  6µs according to specs (4 seems to work fine)

enum ControllerType {
  NONE,
  NES,
  SNES,
  NTT
};

//GENESIS
SegaControllers32U4 controllers;

// Controller previous states
word lastState[2] = {1, 1};




#define UP    0x01
#define DOWN  0x02
#define LEFT  0x04
#define RIGHT 0x08

#define NTT_CONTROL_BIT 0x20000000

// Controllers
uint8_t buttons[GAMEPAD_COUNT_MAX] = {0, 0, 0, 0};
uint8_t buttonsPrev[GAMEPAD_COUNT_MAX] = {0, 0, 0, 0};
//uint8_t gpBit[GAMEPAD_COUNT_MAX] = {B10000000,B01000000,B00100000,B00010000};
uint8_t gpBit[GAMEPAD_COUNT_MAX] = {B00000010, B00000100, B10000000, B0100000};


//SNES
ControllerType controllerType[GAMEPAD_COUNT_MAX] = {NONE, NONE};
uint32_t btnBits_SNES[32] = {0x10, 0x40, 0x400, 0x800, UP, DOWN, LEFT, RIGHT, 0x20, 0x80, 0x100, 0x200, // Standard SNES controller
                             0x10000000, 0x20000000, 0x40000000, 0x80000000, 0x1000, 0x2000, 0x4000, 0x8000, // NTT Data Keypad (NDK10)
                             0x10000, 0x20000, 0x40000, 0x80000, 0x100000, 0x200000, 0x400000, 0x800000,
                             0x1000000, 0x2000000, 0x4000000, 0x8000000
                            };
uint8_t buttonCount = 32;

uint8_t btnBits_NES[BUTTON_COUNT] = {0x20, 0x10, 0x40, 0x80, UP, DOWN, LEFT, RIGHT};


//PCE
// Controllers
uint8_t buttons_PCE[2][2]     = {{0, 0}, {0, 0}};
uint8_t buttonsPrev_PCE[2][2] = {{0, 0}, {0, 0}};

// Turbo timing
uint32_t microsNow = 0;
uint32_t microsEnable = 0;






uint8_t gp = 0;

// Timing
uint32_t microsButtons = 0;










const int pinSNES = A0;
const int pinNES = A2;
const int pinNEOGEO = A1;
const int pinGENESIS = A3;
const int pinSelect = 9;
int outputValueSNES = 0;        // value output to the PWM (analog out)
int outputValueNES = 0;        // value output to the PWM (analog out)
int outputValueNEOGEO = 0;        // value output to the PWM (analog out)
int outputValueGENESIS = 0;        // value output to the PWM (analog out)
int SISTEMA = 0;

const char *gp_serial = "DECAPOD";

void setup() {

  pinMode(pinSNES, OUTPUT);
  pinMode(pinNES, OUTPUT);
  pinMode(pinNEOGEO, OUTPUT);
  pinMode(pinGENESIS, OUTPUT);
  pinMode(pinSelect, INPUT);


  analogWrite(pinSNES, 255);
  if (analogRead(pinSelect) >= 1020) SISTEMA = SNES_; //Serial.println("SNES");
  else {
    analogWrite(pinSNES, 0);
    analogWrite(pinNES, 255);
    if (analogRead(pinSelect) >= 1020) SISTEMA = NES_; //Serial.println("NES");
    else {
      analogWrite(pinNES, 0);
      analogWrite(pinNEOGEO, 255);
      if (analogRead(pinSelect) >= 1020) SISTEMA = NEOGEO_; //Serial.println("NEOGEO");
      else {
        analogWrite(pinNEOGEO, 0);
        analogWrite(pinGENESIS, 255);
        if (analogRead(pinSelect) >= 1020) SISTEMA = GENESIS_; //Serial.println("GENESIS");
        else {
          analogWrite(pinGENESIS, 0);
          SISTEMA = PCE_;//Serial.println("PCE");
        }
      }
    }
  }

  pinMode(pinSNES, INPUT);
  pinMode(pinNES, INPUT);
  pinMode(pinNEOGEO, INPUT);
  pinMode(pinGENESIS, INPUT);
  pinMode(pinSelect, OUTPUT);

  if (SISTEMA == NES || SISTEMA == SNES) {
    delay(500);

    //SNES
    if (SISTEMA == SNES) detectControllerTypes();
  }
  else if (SISTEMA == PCE_) {
    //PCE
    // Set D0-D3 as inputs and enable pull-up resistors (port1 data pins) --> D4 (UP), B5 (R), D3 (DOWN), D2(L)
    DDRD  &= ~B00011100;
    DDRB  &= ~B00100000;
    PORTD |=  B00011100;
    PORTB |=  B00100000;

    // Set B1 and B3 as outputs and set them LOW --> B4 (SEL), D0 (OE)
    PORTB &= ~B00010000;
    DDRB  |=  B00010000;
    PORTD &= ~B00000001;
    DDRD  |=  B00000001;

    // Wait for the controller(s) to settle
    delay(100);
  }
  /*else if (SISTEMA == GENESIS_)
    {
    for (byte gp = 0; gp <= 1; gp++)
      Gamepad[gp].reset();
    }*/




}

void loop() {

  Gamepad_ Gamepad[GAMEPAD_COUNT](SISTEMA);
  //Gamepad_ Gamepad[GAMEPAD_COUNT];

  if (SISTEMA == GENESIS_)
  {
    for (byte gp = 0; gp <= 1; gp++)
      Gamepad[gp].reset();
  }

  switch (SISTEMA) {
    case NOT_SELECTED:
      break;

    case SNES_:

      while (1)
      {
        Serial.println("SNES");
        // See if enough time has passed since last button read
        if ((micros() - microsButtons) > BUTTON_READ_DELAY)
        {
          // Pulse latch
          sendLatch();

          for (uint8_t btn = 0; btn < buttonCount; btn++)
          {
            for (gp = 0; gp < GAMEPAD_COUNT; gp++)
              //(PINF & gpBit[gp]) ? buttons[gp] &= ~btnBits[btn] : buttons[gp] |= btnBits[btn];
              (PIND & gpBit[gp]) ? buttons[gp] &= ~btnBits_SNES[btn] : buttons[gp] |= btnBits_SNES[btn];
            sendClock();
          }

          // Check gamepad type
          for (gp = 0; gp < GAMEPAD_COUNT; gp++)
          {
            if (controllerType[gp] == NES) {   // NES
              bitWrite(buttons[gp], 5, bitRead(buttons[gp], 4));
              bitWrite(buttons[gp], 4, bitRead(buttons[gp], 6));
              buttons[gp] &= 0xC3F;
            }
            else if (controllerType[gp] == NTT) // SNES NTT Data Keypad
              buttons[gp] &= 0x3FFFFFF;
            else                               // SNES Gamepad
              buttons[gp] &= 0xFFF;
          }

          for (gp = 0; gp < GAMEPAD_COUNT; gp++)
          {
            // Has any buttons changed state?
            if (buttons[gp] != buttonsPrev[gp])
            {
              Gamepad[gp]._GamepadReport.buttons = (buttons[gp] >> 4); // First 4 bits are the axes
              Gamepad[gp]._GamepadReport.Y = ((buttons[gp] & DOWN) >> 1) - (buttons[gp] & UP);
              Gamepad[gp]._GamepadReport.X = ((buttons[gp] & RIGHT) >> 3) - ((buttons[gp] & LEFT) >> 2);
              buttonsPrev[gp] = buttons[gp];
              Gamepad[gp].send();
            }
          }

          microsButtons = micros();
        }
      }

      break;

    case NES_:
      while (1)
      {
        Serial.println("NES");
        // See if enough time has passed since last button read
        if ((micros() - microsButtons) > BUTTON_READ_DELAY)
        {
          // Pulse latch
          sendLatch();

          for (uint8_t btn = 0; btn < BUTTON_COUNT; btn++)
          {
            for (gp = 0; gp < GAMEPAD_COUNT; gp++)
              //(PINF & gpBit[gp]) ? buttons[gp] &= ~btnBits[btn] : buttons[gp] |= btnBits[btn];
              (PIND & gpBit[gp]) ? buttons[gp] &= ~btnBits_NES[btn] : buttons[gp] |= btnBits_NES[btn];
            sendClock();
          }

          for (gp = 0; gp < GAMEPAD_COUNT; gp++)
          {
            // Has any buttons changed state?
            if (buttons[gp] != buttonsPrev[gp])
            {
              Gamepad[gp]._GamepadReport.buttons = (buttons[gp] >> 4); // First 4 bits are the axes
              Gamepad[gp]._GamepadReport.Y = ((buttons[gp] & DOWN) >> 1) - (buttons[gp] & UP);
              Gamepad[gp]._GamepadReport.X = ((buttons[gp] & RIGHT) >> 3) - ((buttons[gp] & LEFT) >> 2);
              buttonsPrev[gp] = buttons[gp];
              Gamepad[gp].send();
            }
          }

          microsButtons = micros();
        }
      }

      break;

    case GENESIS_:
      //Serial.println("GENESIS");
      while (1)
      {
        controllers.readState();
        /*sendState(0);
          sendState(1);*/
        gp = 0;
        if (controllers.currentState[gp] != lastState[gp])
        {
          Gamepad[gp]._GamepadReport.buttons = controllers.currentState[gp] >> 4;
          Gamepad[gp]._GamepadReport.Y = ((controllers.currentState[gp] & SC_BTN_DOWN) >> SC_BIT_SH_DOWN) - ((controllers.currentState[gp] & SC_BTN_UP) >> SC_BIT_SH_UP);
          Gamepad[gp]._GamepadReport.X = ((controllers.currentState[gp] & SC_BTN_RIGHT) >> SC_BIT_SH_RIGHT) - ((controllers.currentState[gp] & SC_BTN_LEFT) >> SC_BIT_SH_LEFT);
          Gamepad[gp].send();
          lastState[gp] = controllers.currentState[gp];
        }


        gp = 1;
        if (controllers.currentState[gp] != lastState[gp])
        {
          Gamepad[gp]._GamepadReport.buttons = controllers.currentState[gp] >> 4;
          Gamepad[gp]._GamepadReport.Y = ((controllers.currentState[gp] & SC_BTN_DOWN) >> SC_BIT_SH_DOWN) - ((controllers.currentState[gp] & SC_BTN_UP) >> SC_BIT_SH_UP);
          Gamepad[gp]._GamepadReport.X = ((controllers.currentState[gp] & SC_BTN_RIGHT) >> SC_BIT_SH_RIGHT) - ((controllers.currentState[gp] & SC_BTN_LEFT) >> SC_BIT_SH_LEFT);
          Gamepad[gp].send();
          lastState[gp] = controllers.currentState[gp];
        }
      }
      break;

    case NEOGEO_:
      //Serial.println("NEOGEO");
      break;

    case PCE_:
      while (1)
      {
        Serial.println("PCE");
        // Handle clock for turbo functionality
        microsNow = micros();
        if ((microsNow - microsEnable) >= FRAME_TIME)
        {
          PORTD |= B00000001;                      // Set enable pin HIGH to increase clock for turbo
          delayMicroseconds(SELECT_PAUSE);         // Wait a while...
          PORTD &= ~B00000001;                     // Set enable pin LOW again
          microsEnable = microsNow;
        }

        // Clear button data
        buttons_PCE[0][0] = 0; buttons_PCE[0][1] = 0;
        buttons_PCE[1][0] = 0; buttons_PCE[1][1] = 0;

        // Read all button and axes states
        PORTB |= B00010000;                        // Set SELECT pin HIGH
        delayMicroseconds(SELECT_PAUSE);           // Wait a while...
        buttons_PCE[0][0] = (PIND & B00011100) | (PINB & B00100000);          // Read DPAD for controller 1
        //if (GAMEPAD_COUNT == 2)
        //  buttons_PCE[1][0] = (PINF & B11110000) >> 4; // Read DPAD for controller 2
        PORTB &= ~B00010000;                       // Set SELECT pin LOW
        delayMicroseconds(SELECT_PAUSE);           // Wait a while...
        buttons_PCE[0][1] = (PIND & B00011100) | (PINB & B00100000);          // Read buttons for controller 1
        //if (GAMEPAD_COUNT == 2)
        //  buttons_PCE[1][1] = (PINF & B11110000) >> 4; // Read buttons for controller 2

        // Invert the readings so a 1 means a pressed button
        buttons_PCE[0][0] = ~buttons_PCE[0][0]; buttons_PCE[0][1] = ~buttons_PCE[0][1];
        //buttons_PCE[1][0] = ~buttons_PCE[1][0]; buttons_PCE[1][1] = ~buttons_PCE[1][1];

        // Send data to USB if values have changed
        for (gp = 0; gp < GAMEPAD_COUNT; gp++)
        {
          // Has any buttons changed state?
          if (buttons_PCE[gp][0] != buttonsPrev_PCE[gp][0] || buttons_PCE[gp][1] != buttonsPrev_PCE[gp][1] )
          {
            Gamepad[gp]._GamepadReport.buttons = buttons_PCE[gp][1];
            Gamepad[gp]._GamepadReport.Y = ((buttons_PCE[gp][0] & DOWN) >> DOWN_SH) - ((buttons_PCE[gp][0] & UP) >> UP_SH);
            Gamepad[gp]._GamepadReport.X = ((buttons_PCE[gp][0] & RIGHT) >> RIGHT_SH) - ((buttons_PCE[gp][0] & LEFT) >> LEFT_SH);
            buttonsPrev_PCE[gp][0] = buttons_PCE[gp][0];
            buttonsPrev_PCE[gp][1] = buttons_PCE[gp][1];
            Gamepad[gp].send();
          }
        }

      }


      break;


  }


  delay(1000);
}


void sendLatch()
{
  // Send a latch pulse to the NES controller(s)
  PORTB |=  B00100000; // Set HIGH
  if (SISTEMA == NES) delayMicroseconds(MICROS_LATCH_NES);
  else if (SISTEMA == SNES) delayMicroseconds(MICROS_LATCH_SNES);
  PORTB &= ~B00100000; // Set LOW
  if (SISTEMA == NES) delayMicroseconds(MICROS_PAUSE_NES);
  if (SISTEMA == SNES) delayMicroseconds(MICROS_PAUSE_SNES);
}

void sendClock()
{
  // Send a clock pulse to the NES controller(s)
  PORTD |=  B10010000; // Set HIGH
  if (SISTEMA == NES) delayMicroseconds(MICROS_CLOCK_NES);
  if (SISTEMA == SNES) delayMicroseconds(MICROS_CLOCK_SNES);
  PORTD &= ~B10010000; // Set LOW
  if (SISTEMA == NES) delayMicroseconds(MICROS_PAUSE_NES);
  if (SISTEMA == SNES) delayMicroseconds(MICROS_PAUSE_SNES);
}


//SNES
void detectControllerTypes()
{
  uint8_t buttonCountNew = 0;

  // Read the controllers a few times to detect controller type
  for (uint8_t i = 0; i < 4; i++)
  {
    // Pulse latch
    sendLatch();

    // Read all buttons
    for (uint8_t btn = 0; btn < buttonCount; btn++)
    {
      for (gp = 0; gp < GAMEPAD_COUNT; gp++)
        //(PINF & gpBit[gp]) ? buttons[gp] &= ~btnBits[btn] : buttons[gp] |= btnBits[btn];
        (PIND & gpBit[gp]) ? buttons[gp] &= ~btnBits_SNES[btn] : buttons[gp] |= btnBits_SNES[btn];
      sendClock();
    }

    // Check controller types and set buttonCount to max needed
    for (gp = 0; gp < GAMEPAD_COUNT; gp++)
    {
      if ((buttons[gp] & 0xF3A0) == 0xF3A0) {  // NES
        if (controllerType[gp] != SNES && controllerType[gp] != NTT)
          controllerType[gp] = NES;
        if (buttonCountNew < 8)
          buttonCountNew = 8;
      }
      else if (buttons[gp] & NTT_CONTROL_BIT) { // SNES NTT Data Keypad
        controllerType[gp] = NTT;
        buttonCountNew = 32;
      }
      else {                                   // SNES Gamepad
        if (controllerType[gp] != NTT)
          controllerType[gp] = SNES;
        if (buttonCountNew < 12)
          buttonCountNew = 12;
      }
    }
  }

  // Set updated button count to avoid unneccesary button reads (for simpler controller types)
  buttonCount = buttonCountNew;
}
/*
  //GENESIS
  void sendState(byte gp)
  {
  // Only report controller state if it has changed
  if (controllers.currentState[gp] != lastState[gp])
  {
    Gamepad[gp]._GamepadReport.buttons = controllers.currentState[gp] >> 4;
    Gamepad[gp]._GamepadReport.Y = ((controllers.currentState[gp] & SC_BTN_DOWN) >> SC_BIT_SH_DOWN) - ((controllers.currentState[gp] & SC_BTN_UP) >> SC_BIT_SH_UP);
    Gamepad[gp]._GamepadReport.X = ((controllers.currentState[gp] & SC_BTN_RIGHT) >> SC_BIT_SH_RIGHT) - ((controllers.currentState[gp] & SC_BTN_LEFT) >> SC_BIT_SH_LEFT);
    Gamepad[gp].send();
    lastState[gp] = controllers.currentState[gp];
  }
  }*/
