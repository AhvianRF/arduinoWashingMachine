# arduinoWashingMachine
Custom board for my (or any) washing machine using an Universal motor.

This project was started due to a non-existent already solution for an arduino washing machine controller. It includes PCB, Arduino code and other things.

A bit of context...

The washing machine showed errors while running (E30) which decodes as a "Door lock error". After looking at the door interlock mechanism, everything was fine.

After a thourough investigation, it appeared that all the hardware was good:

- 2 Spray valves
- Drain pump motor
- Pressure sensor
- Heater
- Universal motor + Tachometer

So the faulty component is for sure the control board. It uses a chinese microcontroller marked SH32f9801p of which, the only information available on line is under this link:
https://nl.aliexpress.com/i/1005008315345643.html?gatewayAdapt=glo2nld

Without any documentation, I decided to go the easiest way: MAKE my own board.

The essential components of a washing machine are easily controllable by simple relays or triacs, so nothing too hard to build under a limited timeframe. The work consist of driving relays, measuring frequencies and monitoring the line voltage. The trickiest part is the motor driver, which I have to build from scratch because where I live, there is no such already made board sold, and to order it would not only take ages but would get me broke for shipping fees and custom clearance.

A similar work was already done here https://www.hackster.io/saulius-bandzevicius/arduino-based-universal-ac-motor-speed-controller-a4ceaf by Saulius and if you decided to build this project, I highly recommend you to read what he wrote, especially on the safety notice.

Saulius uses a PID library to precisely control the motor rpm in a closed loop system. The shotcky diode (D1) that he uses on the tachometer circuit can be replaced by two 1N4148 switching diode clamp circuit that will limit (clamp) the AC from the tacho to around -0.7V and 5.67V which the comparator can handle without a problem. The 220nF capacitor C1 can be removed. I also used an LM311 instead of the LM393.

Add to the hardware:

- Tacho sense circuit (built)
- NTC thermistor circuit
- Digital input with pulseIn() to read the water level (pressure)


I blew up a few bunch of triacs all along the project due to the fact that I was not using a snubber circuit for the diac driver, which is a must to consider because the triac turns off at zero current, and we are syncing the gate pulses from the zero-cross voltage, which, for an inductive load is not in phase. A detailed explanation can be found here https://www.soloelectronica.net/triac/AN_3003.PDF


It is actually a work in progress.
