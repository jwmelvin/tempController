# tempController
Arduino-based temperature controller with menu system

This uses a number of libraries, including for [Dallas temperature sensors](https://github.com/milesburton/Arduino-Temperature-Control-Library), [thermocouple temperature sensor](https://github.com/jwmelvin/MAX6675), [the menu interface](http://forum.arduino.cc/index.php?topic=38053.msg281972#msg281972), [timing loops](https://github.com/PaulStoffregen/MsTimer2), [nonvolatile memory](https://github.com/collin80/EEPROMAnything) and [the PID algorithm](https://github.com/br3ttb/Arduino-PID-Library). The hardware had two different versions, and HWv1 was my personal device so that's the one where I developed the code. It uses [my forked version of the PID library](https://github.com/jwmelvin/Arduino-PID-Library).

If implemented now, I would use some different libraries, like [bounce](https://github.com/thomasfredericks/Bounce2) for buttons and [encoder](https://github.com/PaulStoffregen/Encoder) for the rotary encoder.
