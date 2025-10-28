# ESP32_SSTV_decoder
Esp32 CYD SSTV decoder and image drawing on TFT display
ESP32 with TFT display (In particular a CYD display which integrates ESP32) can decode SSTV signal and plot it on display.
My target was to use a CYD hardware as it is and receive SSTV images without using a PC. It works symply with connection to an analog pin of ESP32 module.
I selected a pin 35 as this one is on one connector of the board. You need to provide a galvanic isolation, like capacitor, and an offset voltage (1,6Vdc) to analog pin.
In this first version decoder can decode Martin1 mode only. Folowing versions will integrate other standard modes used on HAM frequencies.
This is work in progress, so first version is just a proofe of concept. Likely some improovements will follow as i get more ideas.
