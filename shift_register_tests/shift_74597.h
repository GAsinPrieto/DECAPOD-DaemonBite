#ifndef SHIFT_74597_INCLUDE
#define SHIFT_74597_INCLUDE

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define DELAY 10

class shift_74597{
	private:
		int _QH, _SCK, _RCK, _SLOAD;//, _SCLR;

	public:
		shift_74597(int QH, int SCK, int RCK, int SLOAD);//, int SCLR);
		~shift_74597();

		void init();
		//void clear();
		void load();
		char getByte();
		char getByteReverse();

};

#endif /*74597_INCLUDE*/
