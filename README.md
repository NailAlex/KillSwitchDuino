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

If switch pressed 3 times within 5 sec from start - run Menu Mode for customizing

Sketch read input battery voltage(need calibration in constants for actual resistors and regulator voltage value) with these choices:
- block engine (function can be disable) if it lower than Level1(low voltage level) 
- run unlimited cycle Imperial March(function can be disable) if it lower than Level2(very low voltage level) for prevent flying with bad battery. 
Use average battery voltage for last 3 second reads.

Sketch analyze PPM RX signal quality for engine blocking if RX connection is lost or bad(automatic enable Search Buzzer until PPM RX signal restore). Can be enable "NoRX engine Off" feature for hardcore users :)


(in progress)
Sketch Read analog data from optional external ACS758x current sensor and calculate monentary current, consumed current from day start(after ResetCurrentCounter procedure). Send to Serial port these values. Can be enable sound notification for low level capacity(confugurable constant).


LED indicator states:
Solid GREEN - normal work
Blinking GREEN - Battery low voltage (Level1) 
Blinking BLUE - No RX signal alert
Blinking RED - Engine is OFF

Sketch write all telemetry to Serial: states, switching count, momental voltage/average voltage, battery type, servotester PPM out in ms/RX PPM read in ms. If connect to baseboard any BT module(example HC-05), all telemetry may be read on any mobile phone/tablet with BT serial reciver application.

Sketch use EEPROM for store config.

Embedded interactive menu for RX calibrating, battery chemistry select and other functions(see Docs\Menu_struc.txt).


### Baseboards
- two versions baseboard PCBs - basic and HV version;
- all baseboard support 2s LiPo/LiFe batteries;
- all baseboard include opto-isolation for ignition power and separate battery for engine ignition;
- HV baseboard include direct HV power for two channels and external 5V 7.5A reg for Arduino/RX power;
- Basic baseboard include external 5V 5A(max 2A) reg for Arduino power for maximum stability for voltage readings;
- all baseboards support for 2 parallel connected piezo-buzzers(integrated and external), only 5V-type;
- All baseboard use external 2s LiPo/LiFe power supply connectors (see Docs for example wiring connections);
