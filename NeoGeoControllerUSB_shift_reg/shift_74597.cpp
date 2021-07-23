#include "shift_74597.h"		

shift_74597::shift_74597(int QH, int SCK, int RCK, int SLOAD){//, int SCLR){
	_QH = QH;
	_SCK = SCK;
	_RCK = RCK;
	_SLOAD = SLOAD;
	//_SCLR = SCLR;
	
	
}

void shift_74597::init(){
	pinMode(_QH, INPUT);
	pinMode(_SCK, OUTPUT);
	pinMode(_RCK, OUTPUT);
	pinMode(_SLOAD, OUTPUT);
	//pinMode(_SCLR, OUTPUT);
	
	digitalWrite(_SLOAD, HIGH);
	//digitalWrite(_SCLR, HIGH);
	digitalWrite(_SCK, LOW);
	digitalWrite(_RCK, LOW);
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
	digitalWrite(_RCK, HIGH);
	delayMicroseconds(DELAY);
	digitalWrite(_RCK, LOW);
	delayMicroseconds(DELAY);
	digitalWrite(_SLOAD, LOW);
	delayMicroseconds(DELAY);
	digitalWrite(_SLOAD, HIGH);
	delayMicroseconds(DELAY);
}

char shift_74597::getByte(){
	char result = 0;
	for (int i = 0; i <= 7; i++){
		if (digitalRead(_QH) == HIGH) {result |= (1<<(7-i));}
		digitalWrite(_SCK, HIGH);
		delayMicroseconds(DELAY);
		digitalWrite(_SCK, LOW);
		delayMicroseconds(DELAY);
	}
	return result;
}

char shift_74597::getByteReverse(){
	char result = 0;
	for (int i = 0; i <= 7; i++){
		if (digitalRead(_QH) == HIGH) {result |= (1<<i);}
		digitalWrite(_SCK, HIGH);
		delayMicroseconds(DELAY);
		digitalWrite(_SCK, LOW);
		delayMicroseconds(DELAY);
	}
	return result;
}
		
