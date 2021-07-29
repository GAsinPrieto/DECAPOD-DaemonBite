# DECAPOD USB with support for for NES controllers

This subproject includes support for up to 4 NES controllers. Third and fourth controllers are connected to ? and ????

Note that you need to disable CDC profile in the Arduino, in order to allow more than 3 controllers on an Atmega32u4, thus, to reprogram it, you have to force bootloader by resetting twice the board before uploading the sketch.