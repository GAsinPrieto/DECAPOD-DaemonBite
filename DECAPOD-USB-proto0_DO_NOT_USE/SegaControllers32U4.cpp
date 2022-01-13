//
// SegaControllers32U4.cpp
//
// Authors:
//       Jon Thysell <thysell@gmail.com>
//       Mikael Norrgård <mick@daemonbite.com>
//
// (Based on the code by Jon Thysell, but the interfacing is almost completely
//  rewritten by Mikael Norrgård)
//
// Copyright (c) 2017 Jon Thysell <http://jonthysell.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "Arduino.h"
#include "SegaControllers32U4.h"

SegaControllers32U4::SegaControllers32U4(void)
{
  // Setup input pins (A0,A1,A2,A3,14,15 or PF7,PF6,PF5,PF4,PB3,PB1)
  _pinSelect1 = true;
  _pinSelect2 = true;

  for(byte i=0; i<=1; i++)
  {
    currentState[i] = 0;
    _connected[i] = 0;
    _sixButtonMode[i] = false;
    _ignoreCycles[i] = 0;
  }
}

void SegaControllers32U4::setup_controllers(){
  // Setup input pins (A0,A1,A2,A3,14,15 or PF7,PF6,PF5,PF4,PB3,PB1)
  DDRB  &= ~B00010000; // input
  PORTB |=  B00010000; // high to enable internal pull-up
  // Setup input pins (TXO,RXI,2,3,4,6 or PD3,PD2,PD1,PD0,PD4,PD7)
  DDRD  &= ~B10011110; // input
  PORTD |=  B10011110; // high to enable internal pull-up

  DDRB  |= B00100000; // Select pins as output
  DDRE  |= B01000000; 
  DDRD  |= B00000001; // Select pins as output
  PORTB |= B00100000; // Select pins high
  PORTE |= B01000000;
}


void SegaControllers32U4::readState()
{
	PORTD &= B11111110; // Controller select pin low - controller 1
	delayMicroseconds(2);
  // Set the select pins low/high
  _pinSelect1 = !_pinSelect1;
  if(!_pinSelect1) {
    PORTB &= ~B00100000;
  } else {
    PORTB |=  B00100000;
  }

  delayMicroseconds(2);

  PORTD |= B00000001; // Controller select pin high - controller 2

  delayMicroseconds(2);
  
  
  
  
  
  
  
  // Set the select pins low/high
  _pinSelect2 = !_pinSelect2;
  if(!_pinSelect2) {
    PORTB &= ~B00100000;
  } else {
    PORTB |=  B00100000;
  }

  delayMicroseconds(2);

  PORTD &= B11111110; // Controller select pin low - controller 1
  
  // Short delay to stabilise outputs in controller
  delayMicroseconds(SC_CYCLE_DELAY-8);

  // Read all input registers
  _inputReg1_1 = PINB;
  _inputReg1_2 = PIND;


  PORTD |= B00000001; // Controller select pin high - controller 2

  delayMicroseconds(6);

  _inputReg2_1 = PINB;
  _inputReg2_2 = PIND;

  readPort1();
  readPort2();
}

// "Normal" Six button controller reading routine, done a bit differently in this project
// Cycle  TH out  TR in  TL in  D3 in  D2 in  D1 in  D0 in
// 0      LO      Start  A      0      0      Down   Up      
// 1      HI      C      B      Right  Left   Down   Up
// 2      LO      Start  A      0      0      Down   Up      (Check connected and read Start and A in this cycle)
// 3      HI      C      B      Right  Left   Down   Up      (Read B, C and directions in this cycle)
// 4      LO      Start  A      0      0      0      0       (Check for six button controller in this cycle)
// 5      HI      C      B      Mode   X      Y      Z       (Read X,Y,Z and Mode in this cycle)    
// 6      LO      ---    ---    ---    ---    ---    Home    (Home only for 8bitdo wireless gamepads)      
// 7      HI      ---    ---    ---    ---    ---    ---    

void SegaControllers32U4::readPort1()
{
  if(_ignoreCycles[0] <= 0)
  {
    if(_pinSelect1) // Select pin is HIGH
    {
      if(_connected[0])
      {
        // Check if six button mode is active
        if(_sixButtonMode[0])
        {
          // Read input pins for X, Y, Z, Mode  //PB4 y PD4, PD3, PD1 en lugar de PF4-7
          (bitRead(_inputReg1_2, DB9_PIN1_BIT1) == LOW) ? currentState[0] |= SC_BTN_Z : currentState[0] &= ~SC_BTN_Z; //UP
          (bitRead(_inputReg1_2, DB9_PIN2_BIT1) == LOW) ? currentState[0] |= SC_BTN_Y : currentState[0] &= ~SC_BTN_Y; //DW
          (bitRead(_inputReg1_2, DB9_PIN3_BIT1) == LOW) ? currentState[0] |= SC_BTN_X : currentState[0] &= ~SC_BTN_X; //L
          (bitRead(_inputReg1_1, DB9_PIN4_BIT1) == LOW) ? currentState[0] |= SC_BTN_MODE : currentState[0] &= ~SC_BTN_MODE; //R
          _sixButtonMode[0] = false;
          _ignoreCycles[0] = 2; // Ignore the two next cycles (cycles 6 and 7 in table above)
        }
        else
        {
          // Read input pins for Up, Down, Left, Right, B, C
          (bitRead(_inputReg1_2, DB9_PIN1_BIT1) == LOW) ? currentState[0] |= SC_BTN_UP : currentState[0] &= ~SC_BTN_UP;
          (bitRead(_inputReg1_2, DB9_PIN2_BIT1) == LOW) ? currentState[0] |= SC_BTN_DOWN : currentState[0] &= ~SC_BTN_DOWN;
          (bitRead(_inputReg1_2, DB9_PIN3_BIT1) == LOW) ? currentState[0] |= SC_BTN_LEFT : currentState[0] &= ~SC_BTN_LEFT;
          (bitRead(_inputReg1_1, DB9_PIN4_BIT1) == LOW) ? currentState[0] |= SC_BTN_RIGHT : currentState[0] &= ~SC_BTN_RIGHT;
          (bitRead(_inputReg1_2, DB9_PIN6_BIT1) == LOW) ? currentState[0] |= SC_BTN_B : currentState[0] &= ~SC_BTN_B; //A
          (bitRead(_inputReg1_2, DB9_PIN9_BIT1) == LOW) ? currentState[0] |= SC_BTN_C : currentState[0] &= ~SC_BTN_C; //START
        }
      }
      else // No Mega Drive controller is connected, use SMS/Atari mode
      {
        // Clear current state
        currentState[0] = 0;
        
        // Read input pins for Up, Down, Left, Right, Fire1, Fire2
        if (bitRead(_inputReg1_2, DB9_PIN1_BIT1) == LOW) { currentState[0] |= SC_BTN_UP; }
        if (bitRead(_inputReg1_2, DB9_PIN2_BIT1) == LOW) { currentState[0] |= SC_BTN_DOWN; }
        if (bitRead(_inputReg1_2, DB9_PIN3_BIT1) == LOW) { currentState[0] |= SC_BTN_LEFT; }
        if (bitRead(_inputReg1_2, DB9_PIN4_BIT1) == LOW) { currentState[0] |= SC_BTN_RIGHT; }
        if (bitRead(_inputReg1_2, DB9_PIN6_BIT1) == LOW) { currentState[0] |= SC_BTN_A; }
        if (bitRead(_inputReg1_2, DB9_PIN9_BIT1) == LOW) { currentState[0] |= SC_BTN_B; }
      }
    }
    else // Select pin is LOW
    {
      // Check if a controller is connected
      _connected[0] = (bitRead(_inputReg1_2, DB9_PIN3_BIT1) == LOW && bitRead(_inputReg1_1, DB9_PIN4_BIT1) == LOW);
      
      // Check for six button mode
      _sixButtonMode[0] = (bitRead(_inputReg1_2, DB9_PIN1_BIT1) == LOW && bitRead(_inputReg1_2, DB9_PIN2_BIT1) == LOW);
      
      // Read input pins for A and Start 
      if(_connected[0])
      {
        if(!_sixButtonMode[0])
        {
          (bitRead(_inputReg1_2, DB9_PIN6_BIT1) == LOW) ? currentState[0] |= SC_BTN_A : currentState[0] &= ~SC_BTN_A;
          (bitRead(_inputReg1_2, DB9_PIN9_BIT1) == LOW) ? currentState[0] |= SC_BTN_START : currentState[0] &= ~SC_BTN_START; 
        }
      }
    }
  }
  else
  {
    if(_ignoreCycles[0]-- == 2) // Decrease the ignore cycles counter and read 8bitdo home in first "ignored" cycle, this cycle is unused on normal 6-button controllers
    {
      (bitRead(_inputReg1_2, DB9_PIN1_BIT1) == LOW) ? currentState[0] |= SC_BTN_HOME : currentState[0] &= ~SC_BTN_HOME;
    }
  }
}


void SegaControllers32U4::readPort2()
{
  if(_ignoreCycles[1] <= 0)
  {
    if(_pinSelect2) // Select pin is HIGH
    {
      if(_connected[1])
      {
        // Check if six button mode is active
        if(_sixButtonMode[1])
        {
          // Read input pins for X, Y, Z, Mode
          (bitRead(_inputReg2_2, DB9_PIN1_BIT1) == LOW) ? currentState[1] |= SC_BTN_Z : currentState[1] &= ~SC_BTN_Z; //UP
          (bitRead(_inputReg2_2, DB9_PIN2_BIT1) == LOW) ? currentState[1] |= SC_BTN_Y : currentState[1] &= ~SC_BTN_Y; //DW
          (bitRead(_inputReg2_2, DB9_PIN3_BIT1) == LOW) ? currentState[1] |= SC_BTN_X : currentState[1] &= ~SC_BTN_X; //L
          (bitRead(_inputReg2_1, DB9_PIN4_BIT1) == LOW) ? currentState[1] |= SC_BTN_MODE : currentState[1] &= ~SC_BTN_MODE; //R
          _sixButtonMode[1] = false;
          _ignoreCycles[1] = 2; // Ignore the two next cycles (cycles 6 and 7 in table above)
        }
        else
        {
          // Read input pins for Up, Down, Left, Right, B, C
          (bitRead(_inputReg2_2, DB9_PIN1_BIT1) == LOW) ? currentState[1] |= SC_BTN_UP : currentState[1] &= ~SC_BTN_UP;
          (bitRead(_inputReg2_2, DB9_PIN2_BIT1) == LOW) ? currentState[1] |= SC_BTN_DOWN : currentState[1] &= ~SC_BTN_DOWN;
          (bitRead(_inputReg2_2, DB9_PIN3_BIT1) == LOW) ? currentState[1] |= SC_BTN_LEFT : currentState[1] &= ~SC_BTN_LEFT;
          (bitRead(_inputReg2_1, DB9_PIN4_BIT1) == LOW) ? currentState[1] |= SC_BTN_RIGHT : currentState[1] &= ~SC_BTN_RIGHT;
          (bitRead(_inputReg2_2, DB9_PIN6_BIT1) == LOW) ? currentState[1] |= SC_BTN_B : currentState[1] &= ~SC_BTN_B; //A
          (bitRead(_inputReg2_2, DB9_PIN9_BIT1) == LOW) ? currentState[1] |= SC_BTN_C : currentState[1] &= ~SC_BTN_C; //START
        }
      }
      else // No Mega Drive controller is connected, use SMS/Atari mode
      {
        // Clear current state
        currentState[1] = 0;
        
        // Read input pins for Up, Down, Left, Right, Fire1, Fire2
        if (bitRead(_inputReg2_2, DB9_PIN1_BIT1) == LOW) { currentState[1] |= SC_BTN_UP; }
        if (bitRead(_inputReg2_2, DB9_PIN2_BIT1) == LOW) { currentState[1] |= SC_BTN_DOWN; }
        if (bitRead(_inputReg2_2, DB9_PIN3_BIT1) == LOW) { currentState[1] |= SC_BTN_LEFT; }
        if (bitRead(_inputReg2_1, DB9_PIN4_BIT1) == LOW) { currentState[1] |= SC_BTN_RIGHT; }
        if (bitRead(_inputReg2_2, DB9_PIN6_BIT1) == LOW) { currentState[1] |= SC_BTN_A; }
        if (bitRead(_inputReg2_2, DB9_PIN9_BIT1) == LOW) { currentState[1] |= SC_BTN_B; }
      }
    }
    else // Select pin is LOW
    {
      // Check if a controller is connected
      _connected[1] = (bitRead(_inputReg2_2, DB9_PIN3_BIT1) == LOW && bitRead(_inputReg2_1, DB9_PIN4_BIT1) == LOW);
      
      // Check for six button mode
      _sixButtonMode[1] = (bitRead(_inputReg2_2, DB9_PIN1_BIT1) == LOW && bitRead(_inputReg3, DB9_PIN2_BIT1) == LOW);

      // Read input pins for A and Start 
      if(_connected[1])
      {
        if(!_sixButtonMode[1])
        {
          (bitRead(_inputReg2_2, DB9_PIN6_BIT1) == LOW) ? currentState[1] |= SC_BTN_A : currentState[1] &= ~SC_BTN_A;
          (bitRead(_inputReg2_2, DB9_PIN9_BIT1) == LOW) ? currentState[1] |= SC_BTN_START : currentState[1] &= ~SC_BTN_START;
        }
      }
    }
  }
  else
  {
    if(_ignoreCycles[1]-- == 2) // Decrease the ignore cycles counter and read 8bitdo home in first "ignored" cycle, this cycle is unused on normal 6-button controllers
    {
      (bitRead(_inputReg2_2, DB9_PIN1_BIT1) == LOW) ? currentState[1] |= SC_BTN_HOME : currentState[1] &= ~SC_BTN_HOME;
    }
  }
}
