# KillSwitchDuino
Smart KillSwitch on Arduino for safiest shutdown gas engine and other useful features
## Hardware
- Arduino Nano (v3.0 with ATmega328) ![Arduino image] (/images/arduino_nano.jpg)
- Three-color LED lamp     ![Three_lolorLED image](/images/Three_Color_LED_Lamp.jpg)
- Piezo buzzer 5V     ![Buzzer image](/images/buzzer.jpg)
- 10K potentiometer(optional)	   ![Potentiometer image](/images/potentiometer-10k.jpg)

## PCB Hardware
- 


## Logic
Sketch read PPM paket from RX pin(or from embedded servotester with potentiometer and service jumper), detect LOW/HIGH channel position, press(from HIGH to LOW) count and changing engine ignition ON/OFF state. Buzzer short beeps on switching with different tones.

If switch pressed 3 times within 2 sec - run Search Buzzer beeps(0.5sec cycle). Next press 3 times within 2 sec - disable Search Buzzer.

Sketch read input battery voltage(need calibration in constants for actual resistors and regulator voltage value) and block engine switching if it lower than Level1(low voltage level) and run unlimited cycle Imperial March if it lower than Level2(very low voltage level) for prevent flying with bad battery. Blocking use average battery voltage for last 2 second reads.

Sketch analyze PPM signal quality for engine blocking if RX connection is lost or bad.

LED indicator states:
Solid GREEN - normal work
Blinking GREEN - Battery low voltage (Level1) 
Blinking BLUE - No RX signal alert
Blinking RED - Engine is OFF

Sketch write all telemetry to Serial: states, switching count, momental voltage/average voltage, battery type, servotester PPM out in ms/RX PPM read in ms. If connect to baseboard any BT module(example HC-05), all telemetry may be read on any mobile phone/tablet with BT serial reciver application.

Sketch use EEPROM for store config.

Embedded interactive menu for RX calibrating, NoRX signal engine shotdown and battery chemistry select.


### Baseboards
- two versions baseboard PCBs - basic and HV version;
- all baseboard support 2s LiPo/LiFe batteries
- all baseboard include opto-isolation for ignition power and separate battery for engine ignition;
- HV baseboard include direct HV power for three channels and external 5V reg for Arduino/RX power;
- all baseboards support for 2 parallel connected piezo-buzzers(integrated and external), only 5V-type;
- basic baseboard use RX port as power supply(RX also must be HV compilant), HV baseboard use external 2s LiPo/LiFe power supply;
- 
