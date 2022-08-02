# This is an Atmega 32U4 based Synchronous MPPT Buck Solar Charge Controller
## Features:
- Programmable with Arduino IDE
- Input voltage 12 - 22V (36 cell solar panel)
- Output voltage 2.5 - 14.4V (single cell lithium up to 6 cell lead acid batteries can be directly charged)
- Simple MPPT (Maximum Power Point Tracking) solar charge controller for 18V solar panels
- Proper buck converter topology, which increases the current on the output side, not just PWM
- Sparkfun Pro Micro 5V, 16MHz or 3.3V, 8MHz (3.3v recommended, more efficient)
- ACS712 current sensor (5A version) on the output side
- Voltage dividers for voltage measurement on panel and output side
- Two N-channel mosfets, driven by IR2104 half bridge driver, inductor (synchronous buck converter, D5 increases efficiency)
- Third N-channel mosfet as efficient anti backfeed protection without the use of a lossy diode
- Supplied by the panel voltage, so it can't drain your battery during the night
- Working frequency 31.5kHz
- WARNING! This device is not intended to drive 5V USB devices directly. Do it at your own risk!
- Always use a regulated 5V USB adapter on the output! Otherwise, voltage glichtes may damage your USB device!
- This controller is COMMON NEGATIVE
- 3 operation modes: MPPT, CV, CC
- SD card data logger for time, voltage and current. You can import the txt files in Excel
- WARNING! Always adjust output voltage and output current limits according to your battery type, before you connect the battery
- Efficiency between 72% and 92%

New in V 1.0:
- Initial commit, tested with my 10W and 20W solar panels, charging my DIY USB power bank with 8 18650 cells in parallel. Two TP4056 lithium charger modules in paralel on the output
- An anti backfeed diode on the output is required, if you charge batteries directly! Otherwise, your low side mosfet may blow up!!

New in V 1.1:
- Anti backfeed diode is not required anymore, MOSFET Q1 added as anti backfeed protection. It is driven with 16V by a separate charge pump circuit (D3, D4, D6, C8)
- Direct charging tested with 6.6Ah, 12V Lead Acid battery and 4 18650 cells in parallel, charge current up to 3.8A
- Software changed: MOSFET Q3 protection (this is very critical, because it can short the battery to GND!)
- Freewheel diode D5 added to improve efficiency
- LM317T regulator replaced with a more efficient MP1584EN step down module

New in V 1.2:
- Optimised for 4.2V 1S LiPo 18650 direct charging

To do:
- Adding a configuration menu, using the buttons
- Changing the inductor to allow 5A charge current
- Designing a proper PCB

## Usage

See pictures
![](https://github.com/TheDIYGuy999/MPPT_Buck_Converter_Synchronous/blob/master/1.jpg)
![](https://github.com/TheDIYGuy999/MPPT_Buck_Converter_Synchronous/blob/master/2.jpg)
![](https://github.com/TheDIYGuy999/MPPT_Buck_Converter_Synchronous/blob/master/3.jpg)
![](https://github.com/TheDIYGuy999/MPPT_Buck_Converter_Synchronous/blob/master/Board.png)

Also have a look at the pdf schematic.

(c) 2018 TheDIYGuy999
