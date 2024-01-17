#Description

A simple controller for RC car simulating different lights controls. Based on ESP32 S2 mini dev board and ELRS receiver.

Channel control:

Cahnnel # |Function|Notes|
---|---|---|
1|Steering||
2|ESC||
3|Main lights|enables head and tail light when above 1000uS
4|Turn lights|blink left turn when below 1300uS, blink right turn when above 1700uS, off if in between
5|Arm|when below 1500uS turns off steering and ESC control, limits light controls
6|Haz lights|blink both turn signals when above 1300uS



![](https://raw.githubusercontent.com/Le0Michine/ESP32-RC-Car-controller/master/pcb/pcb-top.svg)
![](https://raw.githubusercontent.com/Le0Michine/ESP32-RC-Car-controller/master/pcb/pcb-bottom.svg)
