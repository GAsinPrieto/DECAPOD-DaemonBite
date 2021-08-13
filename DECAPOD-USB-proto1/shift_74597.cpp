#include "shift_74597.h"		



shift_74597::shift_74597(int QH){//, int SCK, int RCK, int SLOAD, int SCLR){
	_QH = QH;
	/*_SCK = SCK;
	_RCK = RCK;
	_SLOAD = SLOAD;
	_SCLR = SCLR;
	*/
	
}

void shift_74597::init(){//uint8_t myPin_mask, volatile uint8_t *myPin_port){

  DDRD  |=  B00010000;// PD4-output - SCK
  DDRD  &= ~B00000010;// PD1-input - QH
  PORTD |=  B00000010; // enable internal pull-ups
  DDRD  |=  B00100000;// PD5-output - RCK/SLOAD
  
  
	//pinMode(_QH, INPUT);
	//pinMode(_SCK, OUTPUT);
	//pinMode(_RCK, OUTPUT);
	//pinMode(_SLOAD, OUTPUT);
	//pinMode(_SCLR, OUTPUT);
	
	//digitalWrite(_SLOAD, HIGH);
	PORTD |= B00100000;// high
  //digitalWrite(_SCLR, HIGH);
	//digitalWrite(_SCK, LOW);
	PORTD &= ~B00110000;// low
  //digitalWrite(_RCK, LOW);
  //PORTB &= ~B00100000;// low

  /*myPin_mask = digitalPinToBitMask(_QH);
  *myPin_port = portInputRegister(digitalPinToPort(_QH));*/
}

shift_74597::~shift_74597(){
}
		
/*void shift_74597::clear(){
	digitalWrite(_SCLR, LOW);
	delayMicroseconds(DELAY);
	digitalWrite(_SCLR, HIGH);
	delayMicroseconds(DELAY);
}*/

void shift_74597::load(){
	//digitalWrite(_RCK, HIGH);
	PORTD |= B00100000;// high
	delayMicroseconds(DELAY);
	//digitalWrite(_RCK, LOW);
  PORTD &= ~B00100000;// low
	delayMicroseconds(DELAY);
	//digitalWrite(_SLOAD, LOW);
	PORTD &= ~B00100000;// low
	delayMicroseconds(DELAY);
	//digitalWrite(_SLOAD, HIGH);
  PORTD |= B00100000;// high
	delayMicroseconds(DELAY);
}

char shift_74597::getByte(){
//char getByte(){
	char result = 0;
	for (int i = 0; i <= 7; i++){
		if (digitalRead(_QH) == HIGH) {result |= (1<<(7-i));}
   //if (((*myPin_port & myPin_mask) != 0) == HIGH) {result |= (1<<(7-i));}
		//digitalWrite(_SCK, HIGH);
		PORTD |= B00010000;// high
    delayMicroseconds(DELAY);
		//digitalWrite(_SCK, LOW);
    PORTD &= ~B00010000;// low
		delayMicroseconds(DELAY);
	}
	return result;
}

char shift_74597::getByteReverse(){
	char result = 0;
	for (int i = 0; i <= 7; i++){
		if (digitalRead(_QH) == HIGH) {result |= (1<<i);}
		//digitalWrite(_SCK, HIGH);
    PORTD |= B00010000;// high
		delayMicroseconds(DELAY);
		//digitalWrite(_SCK, LOW);
    PORTD &= ~B00010000;// low
		delayMicroseconds(DELAY);
	}
	return result;
}
		
