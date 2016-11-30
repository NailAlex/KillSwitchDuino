# KillSwitchDuino
Smart KillSwitch on Arduino for safiest shutdown gas engine and other useful features
## Hardware
- Arduino Nano (v3.0 with ATmega328) ![Arduino image] (/images/arduino_nano.jpg)
- Three-color LED lamp     ![Three_lolorLED image](/images/Three_Color_LED_Lamp.jpg)
- Piezo buzzer 5V (1x or 2x supported)    ![Buzzer image](/images/buzzer.jpg)
- 10K potentiometer(optional)  for service mode working      ![Potentiometer image](/images/potentiometer-10k.jpg)
- LM1084-ADJ (2x) or LM1084-ADJ/LT1083-ADJ
- Optional ACS758x current sensor(50A) module to read the current count and power consumption on-board electronics
- some count of resistors/capacitors

## PCB Hardware
- Basic killswitching PCB (KillSWduino_Basic_v2.lay6)
- HV killswitching PCB with 7.5A 5V regulator for RX power (KillSWduinoHV2_v2.lay6)


## Logic
(released)
Sketch read PPM paket from RX (or from embedded servotester with potentiometer and service jumper), detect LOW/HIGH channel position, press(from HIGH to LOW) count and changing engine ignition ON/OFF state. Buzzer short beeps on switching with different tones.

If switch pressed 3 times within 2 sec - run Search Buzzer beeps(0.5sec cycle). Next press 3 times within 2 sec - disable Search Buzzer.

If switch pressed 3 times within 5 sec from start - run Menu Mode for customizing and calibrating

Sketch read input battery voltage(need customize constants for actual resistors and regulator voltage value) with these choices:
- silent Green LED flashing at Low Level (~15% capacity)
- indicate Ultra Low Level (~5% capacity) with LED flashing and loud beeps from Buzzer

No engine switching blocking(as previous version) and more stability code.

Sketch analyze PPM RX signal quality for engine switching. If RX connection is lost or bad(automatic enable Small Search Buzzer until PPM RX signal restore).


(in progress)
Sketch Read analog data from optional external ACS758x current sensor and calculate monentary current, consumed current from day start(after ResetCurrentCounter procedure). Send to Serial port these values. 
Can be enable sound notification for low level capacity(confugurable constant).
...but today partial options avaiable for future


LED indicator states:
Solid GREEN - normal work
Blinking GREEN - Battery low voltage (Level1) 
Blinking GREEN and Buzzer beeping - Battery ultra low voltage (Level2) 
Blinking BLUE and Buzzer fast beeping - No RX signal alert
Blinking RED - Engine is OFF

Sketch write all telemetry to Serial: states, switching count, actual voltage/average voltage, battery type, servotester PPM out in ms/RX PPM read in ms. 
If connect to baseboard any BT module(example HC-05), all telemetry may be read on any mobile phone/tablet with BT serial reciver application.
In menu mode can be select info blocks for render or disable it


Sketch use EEPROM for store config.

Embedded interactive menu for RX calibrating, battery chemistry select and other functions(see Docs\Menu_struc.txt).


### Baseboards
- two versions baseboard PCBs - basic and HV version;
- all baseboard support 2s LiPo/LiFe batteries;
- all baseboard include opto-isolation for ignition power and separate battery for engine ignition;
- HV baseboard include direct HV power for two channels and external 5V 7.5A reg for Arduino/RX power;
- Basic baseboard include external 5V 5A(max 2A) reg for Arduino power for maximum stability for voltage readings(1A AM1117 give low stability on 5V bus;
- all baseboards support for 2 parallel connected piezo-buzzers(integrated and external), only 5V-type;
- All baseboard use external 2s LiPo/LiFe power supply connectors (see Docs for example wiring connections);
