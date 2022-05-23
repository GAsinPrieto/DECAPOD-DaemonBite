void setup() {

  //while(!(UDADDR & _BV(ADDEN))){ //check USB connection
  //SerialNotInit=true;
  DDRD  = B00000000;
  PORTD |=  B00110110;
  DDRD  |=  B01000001;
  PORTD &= ~B01000001;
  DDRF  = B00000000;
  PORTF = B00000000;
  //}
}

void loop() {

  while (1);

}
