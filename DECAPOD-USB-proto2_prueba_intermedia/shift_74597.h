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
		int _QH;
    uint8_t myPin_mask;
    //volatile uint8_t *myPin_port;

	public:
		shift_74597(int QH);
		~shift_74597();

		void init();
		//void init(uint8_t myPin_mask, volatile uint8_t *myPin_port);
    //void clear();
		void load();
		char getByte();
		//char getByte(uint8_t myPin_mask, uint8_t *myPin_port);
    char getByteReverse();

};

#endif /*74597_INCLUDE*/
