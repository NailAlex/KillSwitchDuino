
/* Smart KillSwitchDuino for flying model aircraft with gasoline engine and electronic inginion
 * Author: NailMan
 * Controller type: Arduino Nano(Mega328)
 * Project site: https://github.com/NailAlex/KillSwitchDuino
 * 
 * Version: 2.1 (current)
 * + Full recode 
 * + many changes and addons
 * - remove Engine blocking at critical alerts(Low Voltage, No RX signal
 * + add serial render parameters choice (on/off only needed parameters) in Menu
 * 
 *  
 * Version: 1.2
 * + add to menu Low Batt voltage Level2 blocking feature for non blocking operating
 * + add sound signals for Low Batt voltage Level1 alert
 * = changed battery chemistry constants and level1/level2 selector
 * + add minimum battery voltage parameter to telemetry
 * + add serial render parameters choice (on/off only needed parameters)
 * 
 * Version: 1.1
 * Features:
 * 1 - Handle PPM signal from RX for Kill Switching operate
 * 2 - Service mode(PIN D12 HIGH/LOW) for activate embedded servotester with external 10K Ohm variable resistor(PIN A1). No TX/RX needed!
 * 3 - Embedded RX voltage reading, analyze and blocking killswitch working if battery voltage i Low/Very Low for 2 sec average value(Level1 - soft block/Level2 - hard block)
 * 4 - Beeper for seaching mode(0.5sec cycle), audialize kill switching (short ping) and Imperial March for Level2 Voltage Alert
 * 5 - Three-color LED indicator: Greed solid - NORMAL, Green flash - Batt Low Voltage, Red flashing - Engine Off, Blue flash - No RX signal.
 * 6 - Serial port telemetry output for external serial devices(Bluetooth)
 * 7 - Configuration mode (like ESC) with EPPROM config saving:
 * a) RX signal calibrating procedure
 * b) Engine stopping if no RX signal (FailSave)
 * c) RX battery chemistry type select(LiPO, LiFE) for correct battery protecting
 * d) reset to Defaults
 * e) exit to normal mode
 * 
 * Used Libraries
 * Timer - //http://github.com/JChristensen/Timer
 * 
 * 
 */


// ****************************************************************
// ****                        INCLUDES                        ****
// ****************************************************************
//Libraries include
#include <Wire.h>
#include <Timer.h>  
#include <Servo.h>
#include <EEPROM.h>



// ****************************************************************
// ****                CONSTANTS AND DEFINES                   ****
// ****************************************************************

//Sensor and Generators PINs
//Sensors
#define MAIN_VOLTAGE_BATT_PIN A0                          //Analog pin for Main Battery voltage reading
#define CURRENT_SENSOR_IN_PIN A2                          //Analog pin for external ACS758 style current sensor data reading (for Feature ver 2.2)
#define SERVO_TESTER_IN_PIN A1                            //Analog pin for external potentiometer data reading 
#define MAINSWITCH_IN_PIN 2                               //Main switch PWM input pin (with hardware interrupt support)
#define SYSTEM_SERVICE_PIN 12                             //Service mode jumper input digital pin (connect to HIGH for activate service mode, see sheme https://github.com/NailAlex/KillSwitchDuino/tree/master/Docs)
//Generators
#define SERVO_TESTER_OUT_PIN 5                            //Embedded servotester output PWM pin(parallel with KILLSWITCH_INPIN on device)
#define KILLSWITCH_OUT_PIN 7                              //Killswitch output digital pin for engine ignition MOSFET switching
#define BUZZER_PIN 11                                     //BUZZER output PWM pin

//LEDs
#define MAIN_LED_R 10                                     //PWM output pin for Red component of SystemLED
#define MAIN_LED_G 9                                      //PWM output pin for Green component of SystemLED
#define MAIN_LED_B 6                                      //PWM output pin for Blue component of SystemLED
#define LED_PIN 13                                        //PWM output pin for Other features

//Servo and PPM 
#define DEF_SWITCH_LEVEL 1500                             //Default switching level in ms (1000<SwitchLevel<2000)
#define DEF_LOWER_PPM_LEVEL 1000                          //Default minimum level in ms (1000)
#define DEF_UPPER_PPM_LEVEL 2000                          //Default maximum level in ms (2000)

//System States
#define SYSTEM_NORMAL 0                                   //Normal switching operate
#define SYSTEM_FIRST_START 1                              //One Time initial startup procedure(after basic setup() )
#define SYSTEM_MENU_M 9                                   //Main menu
#define SYSTEM_MENU_10_CAL 10                             //Calibrate RX input
#define SYSTEM_MENU_20_BTYPE 20                           //Select Battery Chemistry type
#define SYSTEM_MENU_30_RENDER 30                          //Select Battery Chemistry type
#define SYSTEM_MENU_31_EN_SYSSTATES_RENDER 31             //Enable/Disable System States info(Counters, Alerts) render to Serial Port
#define SYSTEM_MENU_32_EN_BATTINFO_RENDER 32              //Enable/Disable Batetry info(Chemistry type, Voltages) render to Serial Port
#define SYSTEM_MENU_33_EN_PPMINFO_RENDER 33               //Enable/Disable PPM signal info(Current Levels) render to Serial Port
#define SYSTEM_MENU_34_EN_CURRENT_RENDER 34               //Enable/Disable CurrentSensor info render to Serial Port  (for Feature ver 2.2)
#define SYSTEM_MENU_40_DEFAULTS 50                        //Reset Config to Default
#define SYSTEM_CALIBRATING 100                            //RX inpout Calibrating mode


//Menus 
#define MENU_INITIAL_PERIOD 5000                          //Time period from end SYSTEM_FIRST_START procedure for enter in Menu Mode (3 times switch press)
#define MENU_CHOICE_PERIOD 3000                           //Time period for user choice in currentmenu

//Battery Section
#define BATT_LIFE 17                                      //Battery type - Lithium Ferrum Phosphate (LiFePO4)
#define BATT_LIPO 21                                      //Battery type - Lithium Polymer
#define BATT_CELLS 2                                      //Battery cell count (default is 2, but may be any)
const float BATT_LIFE_L1 = 3.15;                          //LiFE L1 level for Warning (~15% of useful capacity). Default 3.15V
const float BATT_LIFE_L2 = 3.0;                           //LiFE L2 level for ALERT (~5% of useful capacity, CutOff level). Default 3.0V
const float BATT_LIPO_L1 = 3.4;                           //LiPo L1 level for Warning (~15% of useful capacity). Default 3.4V
const float BATT_LIPO_L2 = 3.0;                           //LiPo L2 level for ALERT (~5% of useful capacity, CutOff level). Default 3.0V


//BUZZER
#define BUZZER_SEARCHER_PERIOD 2000                       //Time limits within activate Buzzer if >=3 time switch pressed
//#define BEEPER_FREQ 2300                                //For feature use
//Menu item indexes
#define SOUND_KSW_INTRO 0                                 //SystemOn melody index
#define SOUND_MM_INTRO 1                                  //Main Menu INTRO melody index
#define SOUND_MM_OUTRO 2                                  //Main Menu OUTRO melody index (exit to Normal Mode)
#define SOUND_M1_INTRO 3                                  //Main Menu INTRO melody index
#define SOUND_M2_INTRO 4                                  //Main Menu INTRO melody index
#define SOUND_M3_INTRO 5                                  //Main Menu INTRO melody index
#define SOUND_M31_INTRO 6                                  //Main Menu INTRO melody index
#define SOUND_M32_INTRO 7                                  //Main Menu INTRO melody index
#define SOUND_M33_INTRO 8                                  //Main Menu INTRO melody index
#define SOUND_M34_INTRO 9                                  //Main Menu INTRO melody index
#define SOUND_M4_DEFS_INTRO 10                             //Main Menu INTRO melody index





// ****************************************************************
// ****                      VARIABLES                         ****
// ****************************************************************


//Main System Mode variables
uint8_t SystemMode = SYSTEM_NORMAL;                       //System Mode state
bool SystemServiceMode = false;                           //Enable embedded servotester for PPM generating(SYSTEM_SERVICE_PIN must be HIGH)

//System state vars
bool SwitchPOS = false;                                   //Switch position. TRUE if down, FALSE if UP
bool SwitchPOS_fromUp = false;                            //from upper position flag
bool EngineSTOP = false;                                  //Switch for cut-off engine ignition
bool EngineSTOP_flag = false;                             //Stopflag for prevent fast double switching
bool LastEngineSTOP = false;                              //Last SystemEngineSTOP value for NoRX handling
uint8_t SystemStatusLowBatt = 0;                          //Low battery voltage Warning(=1) or Alert(=2)
bool SystemStatusNoRX = false;                            //No RX Signal Alert switch. If packets counter ~0 then signal lost

//Menus variables
bool MenuIntroPlayed = false;         
uint8_t MenuItem = 0;                                     //Menu Item counter. if =0 then just entering to menu and need play intro sound
unsigned long MenuInitialTime = 0;                        //Start Time(in ms) after SYSTEM_FIRST_START procedure
unsigned long MenuItemTime = 0;                           //Current Item start Time(in ms)
bool MenuItemSelected = false;                            //Selecting Item flag (if user press switch within MENU_CHOICE_PERIOD after play menu item intro sound


//Battery variables
byte  MainBattType = BATT_LIFE;                           //Main battery chemistry type
float MainBattVoltage = 0.00;                             //Main battery current voltage
const float MainBattR1 = 14870.0;                         //Volt devider upper leg resistor (+Vin -> read point). Need manual read(with multimeter) and fill this var your device!
const float MainBattR2 = 9825.0;                          //Volt devider lower leg resistor (read point -> GND). Need manual read(with multimeter) and fill this var your device!
float V_REF = 5.02;                                       //Accurate voltage vcalue from system linear regulator on device. Need manual read(with multimeter) and fill this var your device!
float MAIN_BATT_VADD = 0.03;                              //Volt correction addon. Need manual fill with multimeter compare on your device!
float MainBattVoltageSum = 0;                             //Volt sum for calc average
byte MainBattVoltageCount = 0;                            //Volt reading count for calc average
float MainBattVoltageAverage = 0;                         //Average voltage within reading period
byte MainBattVoltageAverage_time = 7;                     //1 time per 2 seconds period. One tick = 250ms period.
float MainBattLowVoltageL1 = 0;                           //Current L1 level for Warning and Green MainLED slow blinking and short beeps
float MainBattLowVoltageL2 = 0;                           //Current L2 level for ALERT and Green MainLED fast blinking and short beeps

//Servo and PPM variables
unsigned int ServoOutMS = 0;                              //Intergated ServoTester output PPM signal value to KillSwitch pin
unsigned int ServoInValue = 0;                            //Intergated ServoTester input analog value from potentiometer
unsigned int ServoLastOutMS = 0;                          //stopflag for one time switching
int PPMLowerLevel = DEF_LOWER_PPM_LEVEL;                  //Current PPM signal Lower Level value in ms (may change after calibrating procedure for custom TX/RX device levels)
int PPMUpperLevel = DEF_UPPER_PPM_LEVEL;                  //Current PPM signal Upper Level value in ms (may change after calibrating procedure for custom TX/RX device levels)
int PPMSwitchLevel = DEF_SWITCH_LEVEL;                    //Current PPM signal Switching Level value in ms. Calc with formula: PPMLowerLevel + (PPMUpperLevel - PPMLowerLevel)/2. 
                                                          //If upper/lower levels is asymmetric(thrust channel use, etc) - value is not 1500ms, else always is 1500ms ;)
int CalibrationUpperLevel = 0;                            //UpperLevel for Calibration procedure
int CalibrationLowerLevel = 0;                            //LowerLevel for Calibration procedure
int CalibrationSwitchLevel = 0;                           //Switching Level for Calibration procedure
int CalibratingStageCounter = 1;                          //Calibration procedure stage counter
int CalibratingCounter = 0;                               //Calibration procedure iteration counter


//Main Switch variables
volatile unsigned long MainSWTime = 0;                    //Input signal HIGH peak length in microseconds (=MainSWTime1-MainSWTime2)
volatile unsigned long MainSWTime1 = 0;                   //Main Switch counter1 (start read position). Start after LOW->HIGH detect
volatile unsigned long MainSWTime2 = 0;                   //Main Switch counter2 (end read position). Stop after HIGH->LOW detect
volatile uint8_t MainSWPressCounter = 0;                  //Main Switch press counter (From Upper to lower position)
volatile bool MainSWPressCounterChange = false;           //Input Switch press counter flag for prevent fast double switching
volatile unsigned long MainSWPressLaststime = 0;          //Input Switch press counter LastTime
uint8_t MainSWCounterReset_time = 3;                      //StepTimer for reset packet counter
volatile uint8_t MainSWPacketsCounter = 0;                //PPM packets couter  0<=X<=50
volatile uint8_t MaxMainSWPacketsCounter = 0;             //Packets per second couter  0<=X<=50


//Render and tri-LED indicator variables
uint8_t Render_time = 3;                                  //Render StepTimer counter. 1FPS telemetry render to Serial port
//LED13
uint8_t LED13_time = 3;                                   //Service LED13 StepTimer counter. 1FPS flashing for service LED13
bool LED13On = false;                                     //Service LED13 ON/OFF flag
bool LED13On_flag = false;                                //LED13 flag for prevent double switching
bool LED13Active = false;                                 //LED13 Switch

//EngineSTOP (LED_R)
uint8_t EngineSTOP_time = 3;                              //StopEngine StepTimer counter. 1FPS flashing (red) in Lower Main Switch position(ignition is OFF)
bool EngineSTOPMainLEDOn = false;                         //EngineSTOP MainLED(RED) ON/OFF flag
bool EngineSTOPMainLEDOn_flag = false;                    //EngineSTOP flag for prevent fast double switching
//Low Battery Voltage (LED_G)
uint8_t StatusLowBatt_time = 3;                           //Low Battery Voltage Level StepTimer counter. 1FPS flashing (Green) if L1, 0.5FPS if L2
bool StatusLowBattMainLEDOn = false;                      //LowBatt ON/OFF flag
bool StatusLowBattMainLEDOn_flag = false;                 //LowBatt flag for prevent fast double switching
//No RX signal (LED_B)
bool StatusNoRXMainLEDOn = false;                         //NoRX ON/OFF flag (4 times per second blue flashing, no timer counter needed)
bool StatusNoRXMainLEDOn_flag = false;                    //NoRX flag for prevent fast double switching

//Render options variables
bool RenderSystemStates = true;                           //Enable/disable System States to Serial Port
bool RenderBatteryInfo = true;                            //Enable/disable Battery info to Serial Port
bool RenderPPMInfo = true;                                //Enable/disable PPM signal info to Serial Port
bool RenderCurrentInfo = true;                                //Enable/disable PPM signal info to Serial Port

//BUZZER vars
uint8_t SearchBuzzer_time = 1;                            //SearchBuzzer StepTimer counter. 1FPS beeps
bool SearchBuzzerActive = false;                          //SearchBuzzer activation switch
bool SearchBuzzerOn = false;                              //SearchBuzzer ON/OFF flag
bool SearchBuzzerOn_flag = false;                         //SearchBuzzer ON/OFF flag for prevent fast double switching
int  counter = 0;

//Current Sensor
float CurrentAbs = 0.0;                                   //Momentary current value
float CurrentConsumed = 0.0;                        //Consumed energy from day start
uint8_t CurrentCalc_time = 1;                             //CurrentSensor StepTimer counter. 1FPS calculations

   


// ****************************************************************
// ****                      DEVICES                           ****
// ****************************************************************

Timer m_timer;                                            //Main Timer. 250ms period for one tick. All longtime periodical changes use this timer
Servo myservo;                                            //Servo Tester Generator for NoRX operate on ground. Enabled if SYSTEM_SERVICE_PIN is HIGH and generate PPM Signal on SERVO_TESTER_OUT_PIN mapped between current PPMLowerLevel<>PPMUpperLevel.


// ================================================================
// ===           HARDWARE INTERRUPT HANDLERS                    ===
// ================================================================

void MainSWHandler(){
//reset packets counter
  if (MainSWPacketsCounter >= 50) { MainSWPacketsCounter=0;}  

//reading PPM signal from channel
     if (digitalRead(MAINSWITCH_IN_PIN) == HIGH) 
        {MainSWTime1 = micros(); } 
     else {MainSWTime2 = micros();
           //counter overflow correction (~70min)
           if (MainSWTime2<MainSWTime1) {MainSWTime2=MainSWTime1+100;}
           //calc PPM signal length and packets count
           MainSWTime = MainSWTime2 - MainSWTime1;
           MainSWPacketsCounter++;
           if (MainSWPacketsCounter>MaxMainSWPacketsCounter) {MaxMainSWPacketsCounter=MainSWPacketsCounter;}
          }

//switch handling
 if (MainSWTime >= PPMSwitchLevel)
    {//Switch in Upper position
     SwitchPOS=true;
     SwitchPOS_fromUp=true;}
 else 
   { 
     //Switch in Lower position and if last position is Upper then proccessing for only one time change
     if (SwitchPOS_fromUp)
     {SwitchPOS=false;   
      //increment switch press counter for options
      MainSWPressCounter++;
      //Set change flag for press counter
      MainSWPressCounterChange=true;
      //reset position change flag
      SwitchPOS_fromUp=false;
      }
   }

}

//-----------------------------------------------------------------
// ================================================================
// ===                 HARDWARE READINGS                        ===
// ================================================================

void readMainBattVoltage(){
int value =0;
float vout=0;
  value = analogRead(MAIN_VOLTAGE_BATT_PIN);
  vout = (value * V_REF) / 1024.0;
  MainBattVoltage = (vout / (MainBattR2/(MainBattR1+MainBattR2))) + MAIN_BATT_VADD;
}
  
//-----------------------------------------------------------------

void readServoTester(){
 ServoInValue=analogRead(SERVO_TESTER_IN_PIN);
 ServoOutMS = map(ServoInValue,0,1023,PPMLowerLevel,PPMUpperLevel);
}

//-----------------------------------------------------------------

void CheckServoTester(){
  if (SystemServiceMode) {readServoTester();
    if (ServoOutMS != ServoLastOutMS) 
    {myservo.writeMicroseconds(ServoOutMS);
     ServoLastOutMS = ServoOutMS;}
  }
}

//-----------------------------------------------------------------
// ================================================================
// ===                       SOUND                              ===
// ================================================================

void beep(int note, int duration)
{
//Play tone on buzzerPin
tone(BUZZER_PIN, note, duration);

 
//Play different LED depending on value of 'counter'
  if(counter % 2 == 0)
  {digitalWrite(MAIN_LED_G, HIGH);
   delay(duration);
   digitalWrite(MAIN_LED_G, LOW);}
   else {
   digitalWrite(MAIN_LED_B, HIGH);
   delay(duration);
   digitalWrite(MAIN_LED_B, LOW);
   }
 
//Stop tone on buzzerPin
noTone(BUZZER_PIN);
 
delay(50);
 
//Increment counter
counter++;
}

//-----------------------------------------------------------------

void PlayMenuSound(const byte stype){
  //Play enter to menu sound
  
 switch (stype) {
   case SOUND_MM_INTRO:   //Main Menu Intro
     tone(BUZZER_PIN,300);
     delay(150);
     tone(BUZZER_PIN,800);
     delay(150);
     tone(BUZZER_PIN,1300);
     delay(150);
     tone(BUZZER_PIN,1800);
     delay(150);
     tone(BUZZER_PIN,2300);
     delay(150);    
     noTone(BUZZER_PIN);
     break;    
   case SOUND_MM_OUTRO:  //Main Menu Outro
     tone(BUZZER_PIN,2300,150);
     delay(150);
     tone(BUZZER_PIN,1800,150);
     delay(150);
     tone(BUZZER_PIN,1300,150);
     delay(150);
     tone(BUZZER_PIN,800,150);
     delay(150);
     tone(BUZZER_PIN,300,150);
     delay(150);
     noTone(BUZZER_PIN);    
    break;
   case SOUND_M1_INTRO:  //Calibrating SubMenu Intro
     tone(BUZZER_PIN,2300,200);
     delay(200);
     tone(BUZZER_PIN,200,200);
     delay(200);
     tone(BUZZER_PIN,2300,200);
     delay(200);
     noTone(BUZZER_PIN);
    break;
   case SOUND_M2_INTRO:  //Battery Chemistry SubMenu Intro
     tone(BUZZER_PIN,2300,500);
     delay(500);
     tone(BUZZER_PIN,200,150);
     delay(150);
     noTone(BUZZER_PIN);
    break;    
   case SOUND_M3_INTRO:  //NoRX Engine Off SubMenu Intro
     tone(BUZZER_PIN,200,500);
     delay(500);
     tone(BUZZER_PIN,2300,150);
     delay(150);
     noTone(BUZZER_PIN);
    break;    
   case SOUND_M31_INTRO:  //Enable/Disable Low voltage engine blocking
     tone(BUZZER_PIN,700,150);
     delay(150);
     tone(BUZZER_PIN,1400,150);
     delay(150);
     tone(BUZZER_PIN,2100,150);
     delay(150);
     noTone(BUZZER_PIN);
    break;    
   case SOUND_M32_INTRO:  //Enable/Disable Low voltage engine blocking
     tone(BUZZER_PIN,1800,150);
     delay(150);
     tone(BUZZER_PIN,900,150);
     delay(150);
     tone(BUZZER_PIN,450,150);
     delay(150);
     noTone(BUZZER_PIN);
    break;    
   case SOUND_M33_INTRO:  //Enable/Disable Low voltage engine blocking
     tone(BUZZER_PIN,450,150);
     delay(150);
     tone(BUZZER_PIN,900,150);
     delay(150);
     tone(BUZZER_PIN,450,150);
     delay(150);
     noTone(BUZZER_PIN);
    break;    
   case SOUND_M34_INTRO:  //Enable/Disable Low voltage engine blocking
     tone(BUZZER_PIN,900,150);
     delay(150);
     tone(BUZZER_PIN,450,150);
     delay(150);
     tone(BUZZER_PIN,900,150);
     delay(150);
     noTone(BUZZER_PIN);
    break;    
   case SOUND_M4_DEFS_INTRO:  //Setup to DEFAULTS SubMenu Intro
     tone(BUZZER_PIN,1000,500);
     delay(500);
     tone(BUZZER_PIN,1000,150);
     delay(150);
     noTone(BUZZER_PIN);
    break;    
  case SOUND_KSW_INTRO:  //Normal Mode Intro
    beep(349,300);
    beep(261,300);
    beep(440,300);
    break;    
 }
}

//-----------------------------------------------------------------

void PlayItemSound(const byte count){
 byte i;
  for (int i=0; i <=count-1; i++){
   tone(BUZZER_PIN,2300,100);
   delay(300);
   noTone(BUZZER_PIN);
  }  
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------
// ================================================================
// ===                   EEPROM and CONFIG                      ===
// ================================================================

void ResetSystemConfig(){
//Set Defaults Values
   PPMLowerLevel = DEF_LOWER_PPM_LEVEL;
   PPMUpperLevel = DEF_UPPER_PPM_LEVEL; 
   PPMSwitchLevel = DEF_SWITCH_LEVEL;
   MainBattType = BATT_LIFE;
   RenderSystemStates = true;
   RenderBatteryInfo = true; 
   RenderPPMInfo = true;    
   RenderCurrentInfo = true;
}

//-----------------------------------------------------------------

void SetupBatteryLevels(){
      switch (MainBattType) {
      case BATT_LIFE:
        MainBattLowVoltageL1 = BATT_CELLS * BATT_LIFE_L1;
        MainBattLowVoltageL2 = BATT_CELLS * BATT_LIFE_L2;
      break;
      case BATT_LIPO:
        MainBattLowVoltageL1 = BATT_CELLS * BATT_LIPO_L1;
        MainBattLowVoltageL2 = BATT_CELLS * BATT_LIPO_L2;
      break;
    }
}

//-----------------------------------------------------------------

void RenderCurrentConfig(){
  Serial.println("Config:");
          switch (MainBattType) {
          case BATT_LIFE: 
            Serial.print("Battery type: LiFE");
            break;
           case BATT_LIPO: 
            Serial.print("Battery type: LiPO"); 
            break;
            default:
            Serial.print("Battery type: Unk");
          }  
  Serial.print("  PPM levels Up/Lo/Sw: ");Serial.print(PPMUpperLevel);Serial.print("/");Serial.print(PPMLowerLevel);Serial.print("/");Serial.print(PPMSwitchLevel);Serial.print(" | Render Options(States/Battery/RX/CurrentSensor) : ");
  Serial.print(RenderSystemStates); Serial.print(" / ");Serial.print(RenderBatteryInfo);Serial.print(" / ");Serial.print(RenderPPMInfo);Serial.print(" / ");Serial.println(RenderCurrentInfo);
}

//-----------------------------------------------------------------

unsigned int EEPROMReadInt(int p_address)
        {
        byte lowByte = EEPROM.read(p_address);
        byte highByte = EEPROM.read(p_address + 1);
        return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
        }

//-----------------------------------------------------------------

void EEPROMWriteInt(int p_address, int p_value)
        {
        byte lowByte = ((p_value >> 0) & 0xFF);
        byte highByte = ((p_value >> 8) & 0xFF);
        EEPROM.write(p_address, lowByte);
        EEPROM.write(p_address + 1, highByte);
        }

//-----------------------------------------------------------------

bool ReadEEPROMConfig(){
 uint8_t b,b2 =0;
 int w = 0;
 bool newc = false;
//headers read. if =35 then EEPROM used, else EEPROM clean and use constants
 b=EEPROM.read(0);
 b2=EEPROM.read(31);
  if (b!=35 && b2!=35 ) {
  //EEPROM is clear(new device) and setup variables to default values  
  ResetSystemConfig();
  newc=true;
 }
  else {
  //read config from EEPROM!
   PPMLowerLevel = EEPROMReadInt(1);
   PPMUpperLevel = EEPROMReadInt(3);
   PPMSwitchLevel = EEPROMReadInt(5);
   MainBattType=EEPROM.read(7);
   SetupBatteryLevels(); 
   b=EEPROM.read(8);
   if (b==3) {RenderSystemStates=true;} else {RenderSystemStates=false;}
   b=EEPROM.read(9);
   if (b==3) {RenderBatteryInfo=true;} else {RenderBatteryInfo=false;}
   b=EEPROM.read(10);
   if (b==3) {RenderPPMInfo=true;} else {RenderPPMInfo=false;}
   b=EEPROM.read(11);
   if (b==3) {RenderCurrentInfo=true;} else {RenderCurrentInfo=false;}
   newc=false; 
  }
  return newc;
}

//-----------------------------------------------------------------

void WriteEEPROMConfig(){
 //write header1
  EEPROM.write(0,35);
  EEPROMWriteInt(1,PPMLowerLevel);
  EEPROMWriteInt(3,PPMUpperLevel);
  EEPROMWriteInt(5,PPMSwitchLevel);
  EEPROM.write(7,MainBattType);
  if (RenderSystemStates) {EEPROM.write(8, 3);} else {EEPROM.write(8, 5);}
  if (RenderBatteryInfo) {EEPROM.write(9, 3);} else {EEPROM.write(9, 5);}
  if (RenderPPMInfo) {EEPROM.write(10, 3);} else {EEPROM.write(10, 5);}
  if (RenderCurrentInfo) {EEPROM.write(11, 3);} else {EEPROM.write(11, 5);}
  //write header2
  EEPROM.write(31,35);
}


// ================================================================
// ===                     OTHER FUNCTIONS                      ===
// ================================================================

void RenderMenuNavigation(){
  switch (SystemMode) {
  case SYSTEM_MENU_M: 
    Serial.print("MAIN MENU"); 
    break;
   case SYSTEM_MENU_10_CAL: 
    Serial.print("M1_CALIBRATING RX"); 
    break;   
   case SYSTEM_MENU_20_BTYPE: 
    Serial.print("M2_BATTERY TYPE SELECT"); 
    break;    
   case SYSTEM_MENU_30_RENDER: 
    Serial.print("M3_RENDER OPTIONS SELECT"); 
    break;
   case SYSTEM_MENU_31_EN_SYSSTATES_RENDER: 
    Serial.print("M3-1_SYSTEM STATES RENDER"); 
    break;
   case SYSTEM_MENU_32_EN_BATTINFO_RENDER: 
    Serial.print("M3-2_BATTERY INFO RENDER"); 
    break;
   case SYSTEM_MENU_33_EN_PPMINFO_RENDER: 
    Serial.print("M3-3_PPM INFO RENDER"); 
    break;
   case SYSTEM_MENU_34_EN_CURRENT_RENDER: 
    Serial.print("M3-4_CURRENT INFO RENDER"); 
    break;
   case SYSTEM_MENU_40_DEFAULTS: 
    Serial.print("M4_RESET TO DEFAULTS"); 
    break;
 }
  Serial.print(" | Item : "); Serial.println(MenuItem);      
}

//-----------------------------------------------------------------

void RenderTelemetry(){

  if (SystemMode==SYSTEM_NORMAL) {
    Serial.print("NORMAL ");}  
 
  //System states render
  if (RenderSystemStates) {
  Serial.print("KSW: ");Serial.print(!EngineSTOP);Serial.print("/"); Serial.print(MaxMainSWPacketsCounter);Serial.print("/");  Serial.print(MainSWPressCounter);
  Serial.print(" | Serv/NoRX/LowBatt/Buzzer: ");Serial.print(SystemServiceMode);Serial.print("/");Serial.print(SystemStatusNoRX);Serial.print("/");Serial.print(SystemStatusLowBatt);Serial.print("/");Serial.print(SearchBuzzerActive);
  }

  //Battery status render
  if (RenderBatteryInfo) {
  Serial.print(" | Volt/Avg/Type: ");Serial.print(MainBattVoltage);Serial.print("/");Serial.print(MainBattVoltageAverage);Serial.print("/");
  switch (MainBattType) {
   case BATT_LIFE: 
    Serial.print("LiFE");
    break;
   case BATT_LIPO: 
    Serial.print("LiPO"); 
    break;
    default:
    Serial.print("Unk");
  }
  }

  //RX info render
  if (RenderPPMInfo) {
  Serial.print(" | RX Gen/Read/SW: ");Serial.print(ServoOutMS);Serial.print("/");Serial.print(MainSWTime);Serial.print("/");Serial.print(PPMSwitchLevel);}


  if (RenderCurrentInfo) {
  Serial.print(" | CurrSens Abs/Cons: ");Serial.print(CurrentAbs);Serial.print("/");Serial.print(CurrentConsumed);}
  
  Serial.println(" ");
 
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------
// ================================================================
// ===                           MENUS                          ===
// ================================================================


bool MenuInputHandler(){
 bool s;
  s=false;
   if (millis() - MenuItemTime <= MENU_CHOICE_PERIOD && MainSWPressCounter >=1 && MainSWPressCounterChange) 
   { //if Selected(switched down)
    //MenuItemTime=millis();
    MainSWPressCounterChange=false;
    MainSWPressCounter=0;
    tone(BUZZER_PIN,2300,30);
    s=true;
   } 
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
    MainSWPressCounter=0;
    MainSWPressCounterChange=false;
  //  MenuItemTime=millis();
  }
 return s;
}

//-----------------------------------------------------------------


void MenuMHandler (){
//Main Menu procedure
if (MenuItem==0) {PlayMenuSound(SOUND_MM_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);} 
else{
    MenuItemSelected=MenuInputHandler();

 //Play Menu Item selection sound
 if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}

 //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==5) {MenuItem=1;} else {MenuItem++;}
  MenuIntroPlayed=false;}

 switch (MenuItem) {
   case 1:   //Go To Calibrating RX signal input
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_10_CAL;MenuItem=0;}
    break;    
   case 2:   //Go To Battery chemistry type select 
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_20_BTYPE;MenuItem=0;}
    break;
   case 3:   //Go To Render options select 
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_30_RENDER;MenuItem=0;}
    break;
   case 4:   //Go To DEFAULTS procedure
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_40_DEFAULTS;MenuItem=0;}
    break;
   case 5:  //Exit to Normal Mode
    if (MenuItemSelected) {
         PlayMenuSound(SOUND_MM_OUTRO);
         MenuItem=0;
         Serial.println("Exit to normal mode. Save configuration to EEPROM");
         RenderCurrentConfig();
         WriteEEPROMConfig();
         SystemMode=SYSTEM_NORMAL;}
    break;    
 }
}
}

//-----------------------------------------------------------------

void Menu10Handler (){
//Calibrating Procedure SubMenu
//1 beep - Start Calibration procedure
//2 beeps - Exit to Main Menu

 if (MenuItem==0) {PlayMenuSound(SOUND_M1_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis(); } 
   MenuItemSelected=MenuInputHandler();

 //Play Menu Item selection sound
 if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==2) {MenuItem=1;} else {MenuItem++;}   
  MenuIntroPlayed=false; }
    
 switch (MenuItem) {
   case 1:   //Start RX Calibration procedure
    if (MenuItemSelected) {CalibratingStageCounter=1;SystemMode=SYSTEM_CALIBRATING;}
    break;    
   case 2:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_M;}
    break;    
 }
}

//-----------------------------------------------------------------

void Menu20Handler (){
//Select battery chemistry SubMenu
//1 beep - LiFe battery
//2 beep - LiPolymer battery
//3 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M2_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==3) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
 switch (MenuItem) {
   case 1:   //Setup to LiFePO4
    if (MenuItemSelected) {
         MainBattType=BATT_LIFE;
         SetupBatteryLevels(); 
         MenuItem=0;
         MenuIntroPlayed=false;
      }
    break;    
   case 2:  //Setup to LiPolymer
    if (MenuItemSelected) {
         MainBattType=BATT_LIPO;
         SetupBatteryLevels(); 
         MenuItem=0;
         MenuIntroPlayed=false;
         }
    break;    
   case 3:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_M;}
   break;    
 }
}

//-----------------------------------------------------------------

void Menu30Handler (){
//Select Serial Port Render Options SubMenu
//1 beep - Enable/Disable System States info render
//2 beep - Enable/Disable Battery info render
//3 beeps - Enable/Disable RX info render
//4 beeps - Enable/Disable Current Sensor info render (for future)
//5 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M3_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==5) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
 switch (MenuItem) {
   case 1:   //Go to Enable/Disable System States info render SubSubMenu (31)
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_31_EN_SYSSTATES_RENDER;MenuItem=0;}
    break;    
   case 2:  //Go to Enable/Disable Battery info render SubSubMenu (32)
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_32_EN_BATTINFO_RENDER;MenuItem=0;}
    break;    
   case 3:   //Go to Enable/Disable RX info render SubSubMenu (33)
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_33_EN_PPMINFO_RENDER;MenuItem=0;}
    break;    
   case 4:   //Go to Enable/Disable Enable/Disable Current Sensor info render SubSubMenu (34)
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_34_EN_CURRENT_RENDER;MenuItem=0;}
    break;    
   case 5:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_M;}
   break;    
 }
}


//-----------------------------------------------------------------

void Menu31Handler (){
//Enable/Disable System States info render
//1 beep - Enable render system states to Serial Port
//2 beep - Disable render system states to Serial Port
//3 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M31_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==3) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
 switch (MenuItem) {
   case 1:   //Enable
    if (MenuItemSelected) {
         RenderSystemStates=true;
         MenuItem=0;
         MenuIntroPlayed=false;
      }
    break;    
   case 2:  //Disable
    if (MenuItemSelected) {
         RenderSystemStates=false;
         MenuItem=0;
         MenuIntroPlayed=false;
         }
    break;    
   case 3:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_30_RENDER;}
   break;    
 }
}

//-----------------------------------------------------------------

void Menu32Handler (){
//Enable/Disable System States info render
//1 beep - Enable Battery info render to Serial Port
//2 beep - Disable Battery info render to Serial Port
//3 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M32_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==3) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
 switch (MenuItem) {
   case 1:   //Enable
    if (MenuItemSelected) {
         RenderBatteryInfo=true;
         MenuItem=0;
         MenuIntroPlayed=false;
      }
    break;    
   case 2:  //Disable
    if (MenuItemSelected) {
         RenderBatteryInfo=false;
         MenuItem=0;
         MenuIntroPlayed=false;
         }
    break;    
   case 3:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_30_RENDER;}
   break;    
 }
}

//-----------------------------------------------------------------

void Menu33Handler (){
//Enable/Disable System States info render
//1 beep - Enable RX info render to Serial Port
//2 beep - Disable RX info render to Serial Port
//3 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M33_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==3) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
 switch (MenuItem) {
   case 1:   //Enable
    if (MenuItemSelected) {
         RenderPPMInfo=true;
         MenuItem=0;
         MenuIntroPlayed=false;
      }
    break;    
   case 2:  //Disable
    if (MenuItemSelected) {
         RenderPPMInfo=false;
         MenuItem=0;
         MenuIntroPlayed=false;
         }
    break;    
   case 3:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_30_RENDER;}
   break;    
 }
}

//-----------------------------------------------------------------

void Menu34Handler (){
//Enable/Disable System States info render
//1 beep - Enable RX info render to Serial Port
//2 beep - Disable RX info render to Serial Port
//3 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M33_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==3) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
 switch (MenuItem) {
   case 1:   //Enable
    if (MenuItemSelected) {
         RenderCurrentInfo=true;
         MenuItem=0;
         MenuIntroPlayed=false;
      }
    break;    
   case 2:  //Disable
    if (MenuItemSelected) {
         RenderCurrentInfo=false;
         MenuItem=0;
         MenuIntroPlayed=false;
         }
    break;    
   case 3:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_30_RENDER;}
   break;    
 }
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Menu40ResetDefaultsHandler (){
//Reset Configuration to DEFAULTS SubMenu
//1 beep - Reset to defaults and exit to main menu
//2 beeps - Exit to Main Menu without changing

if (MenuItem==0) {PlayMenuSound(SOUND_M4_DEFS_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==2) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; }
    
 switch (MenuItem) {
   case 1:   //Reset Configuration to defaults and Go To Main Menu
    if (MenuItemSelected) {
       Serial.println("Reset configuration to DEFAULTS and exit to Main Menu");
       Serial.print("Current ");
       ResetSystemConfig();
       RenderCurrentConfig();
       MenuItem=0;
       SystemMode=SYSTEM_MENU_M;
      }
    break;    
   case 2:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_M;}
    break;    
 }
}

//-----------------------------------------------------------------

void CalibratingInput (const byte stage){
  int i;
 switch (stage) {
   case 1:   //Stage1 - play UpperPosition sound
    Serial.println("Move switch to Upper position"); 
     for (i=0; i <= 124; i++){
        analogWrite(BUZZER_PIN, i*24);
        delay(24);
     }
     analogWrite(BUZZER_PIN, 0);
     delay(3000);
     delayMicroseconds(1000); 
   break;    
   case 2:   //Stage2 - reading UpperPosition data
     CalibrationUpperLevel=MainSWTime;
     delayMicroseconds(1000); 
     CalibrationUpperLevel=MainSWTime;
     delayMicroseconds(1000); 
     CalibrationUpperLevel=MainSWTime;
     delayMicroseconds(1000); 
     CalibrationUpperLevel=MainSWTime;
     delayMicroseconds(1000); 
     CalibrationUpperLevel=MainSWTime;
     delayMicroseconds(1000); 
   break;    
   case 3:   //Stage3 - play LowerPosition sound
     Serial.println("Move switch to Lower position"); 
     for (i=0; i <= 124; i++){
        analogWrite(BUZZER_PIN, i*24);
        delay(24);
     }
     analogWrite(BUZZER_PIN, 0);
     delay(3000);
     delayMicroseconds(1000); 
   break;    
   case 4:   //Stage4 - reading LowerPosition data
    CalibrationLowerLevel=MainSWTime;
     delayMicroseconds(1000); 
    CalibrationLowerLevel=MainSWTime;
     delayMicroseconds(1000); 
    CalibrationLowerLevel=MainSWTime;
     delayMicroseconds(1000); 
    CalibrationLowerLevel=MainSWTime;
     delayMicroseconds(1000); 
    CalibrationLowerLevel=MainSWTime;
     delayMicroseconds(1000); 
   break;    
   case 5:   //Stage2 - Calculate levels
    CalibrationSwitchLevel = CalibrationLowerLevel + abs((CalibrationUpperLevel - CalibrationLowerLevel) / 2);
    Serial.print("Read value for UpperLevel: ");Serial.println(CalibrationUpperLevel);
    Serial.print("Read value for LowerLevel: ");Serial.println(CalibrationLowerLevel);
    Serial.print("SwitchLevel is ");Serial.println(CalibrationSwitchLevel);

       PPMLowerLevel = CalibrationLowerLevel;
       PPMUpperLevel = CalibrationUpperLevel; 
       PPMSwitchLevel = CalibrationSwitchLevel;
    
    SystemMode=SYSTEM_MENU_M;
    MenuItem=0;
   break;    
 }
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
// ================================================================
// ===                 TIMER and STATE  HANDLERS                ===
// ================================================================

void MainStateHandler(){

//Engine STOP handling
//in Normal operating mode Engine stop signal equal Switch position: lower pos = OFF, upper pos = ON
 if (SystemMode==SYSTEM_NORMAL){EngineSTOP=!SwitchPOS;} //Set EngineSTOP with Switch position Up=FALSE, Down=TRUE
 
 if (!EngineSTOP) { //if SwitchPOS in Upper position(ignition is ON) then setup output pin to HIGH one time!
  //prevent high speed duplicate switching with potential glitches with flags 
  if (!EngineSTOP_flag) {
     digitalWrite(KILLSWITCH_OUT_PIN, HIGH); 
     tone(BUZZER_PIN,2300,30);
     LastEngineSTOP=EngineSTOP;
     EngineSTOP_flag=true;
  }
 } 
 else 
 { //if SwitchPOS in Lower position(ignition is OFF) then setup output pin to LOW one time!
   //prevent high speed duplicate switching with potential glitches.
  if (EngineSTOP!=LastEngineSTOP) //if EngineSTOP real change his state then setup output pin and play sound
      {digitalWrite(KILLSWITCH_OUT_PIN, LOW);
       tone(BUZZER_PIN,200,30); 
       LastEngineSTOP=EngineSTOP;
       EngineSTOP_flag=false;}
 }

 //NoRX handling
  if (MaxMainSWPacketsCounter<=1) {
      SystemStatusNoRX=true;
     } else {SystemStatusNoRX=false;}
 
 //if Switch pressed(from Up to Down) then handling options  
   if (millis()-MainSWPressLaststime<=BUZZER_SEARCHER_PERIOD && MainSWPressCounter>=1) 
   { 
    switch (MainSWPressCounter) {
    case 3: 
     if (millis() - MenuInitialTime <= MENU_INITIAL_PERIOD){
     //Switched 3 times within Initial period - GO TO Menu Mode!
       SystemMode=SYSTEM_MENU_M;} 
     else {SearchBuzzerActive=!SearchBuzzerActive;
           LED13Active=SearchBuzzerActive;                     //diagnostic LED13
           MainSWPressCounter=0;
           MainSWPressCounterChange=false;
           MainSWPressLaststime=millis();}
          break; 
    case 1: 
     //start time for BUZZER_SEARCHER_PERIOD period
     if (MainSWPressCounterChange) {MainSWPressLaststime=millis(); MainSWPressCounterChange=false;}     
      break;    
    } 
  }

 //reset Switch press counter if passed BEEPER_ACT_PERIOD time period   
 if (millis()-MainSWPressLaststime>BUZZER_SEARCHER_PERIOD) {MainSWPressCounter=0; MainSWPressLaststime=millis(); } 
    
//Flashings LEDs
//Beeping search mode fashing 
 if (SearchBuzzerActive){
  if (SearchBuzzerOn) 
  {
   if (!SearchBuzzerOn_flag){  
    //tone(BUZZER_PIN,2300,450);  //decomment if use tone with frequency
    analogWrite(BUZZER_PIN,255);
    SearchBuzzerOn_flag=true;} }
   else 
   {
      if (SearchBuzzerOn_flag) {
       //noTone(BUZZER_PIN);        //decomment if use tone function
       analogWrite(BUZZER_PIN,0);
       SearchBuzzerOn_flag=false; }}
   }

//LED13 default fashing 
 if (LED13Active){
  if (LED13On && !LED13On_flag) {
    digitalWrite(LED_PIN,HIGH);
    LED13On_flag=true;
    } else {
    digitalWrite(LED_PIN,LOW);
    LED13On_flag=false;
    }} else {digitalWrite(LED_PIN,LOW);LED13On_flag=false;}

//NoRX flashing
  if (StatusNoRXMainLEDOn) { if (!StatusNoRXMainLEDOn_flag) {analogWrite(MAIN_LED_B,255); StatusNoRXMainLEDOn_flag=true;}} 
   else {analogWrite(MAIN_LED_B,LOW);StatusNoRXMainLEDOn_flag=false;}

  if (StatusLowBattMainLEDOn) { 
    if (!StatusLowBattMainLEDOn_flag) {analogWrite(MAIN_LED_G,255);StatusLowBattMainLEDOn_flag=true;}} 
  else {analogWrite(MAIN_LED_G,LOW);StatusLowBattMainLEDOn_flag=false;}

  if (EngineSTOPMainLEDOn) { 
    if (!EngineSTOPMainLEDOn_flag) {analogWrite(MAIN_LED_R,255);EngineSTOPMainLEDOn_flag=true;}}
  else {analogWrite(MAIN_LED_R,LOW);EngineSTOPMainLEDOn_flag=false;}

}

//-----------------------------------------------------------------

void MainTimerHandler(){

 readMainBattVoltage();
 //calc voltage sum for one more 250ms step
 MainBattVoltageSum+=MainBattVoltage;
 MainBattVoltageCount++;

 if ( MainBattVoltageAverage_time == 0 ){
   MainBattVoltageAverage=MainBattVoltageSum / MainBattVoltageCount;
   MainBattVoltageCount=0;
   MainBattVoltageSum=0.0;
   MainBattVoltageAverage_time=7;
 } else MainBattVoltageAverage_time--;
  
 
 //if main battery voltage between L1 and L2 then simple flashing GREEN system LED 
 if (MainBattVoltageAverage > MainBattLowVoltageL2 && MainBattVoltageAverage <= MainBattLowVoltageL1) 
    {SystemStatusLowBatt = 1;}
 //if main battery voltage less than L2 then fast flashing GREEN system LED and fast beeping
 if (MainBattVoltageAverage <= MainBattLowVoltageL2) 
    {SystemStatusLowBatt = 2;}
 if (MainBattVoltageAverage > MainBattLowVoltageL1) {SystemStatusLowBatt=0;} 


//render telemetry 

 if ( Render_time == 0 )
 {
   if (SystemMode==SYSTEM_NORMAL) {RenderTelemetry();}
   else {RenderMenuNavigation();}
  Render_time=3;
  } else Render_time--;


//LED flashing routines
//Buzzer flashing
  if ( SearchBuzzer_time == 0 ){
    SearchBuzzerOn = !SearchBuzzerOn;
    SearchBuzzer_time=1;
  } else SearchBuzzer_time--;


//LED13 (develop)
  if ( LED13_time == 0 ){
    LED13On = !LED13On;
  LED13_time=3;
  } else LED13_time--;

//NO RX Signal LED
  if (SystemStatusNoRX) {
      StatusNoRXMainLEDOn = !StatusNoRXMainLEDOn;
      tone(BUZZER_PIN, 2300,50);
      } else {StatusNoRXMainLEDOn=false;}
    
//EngineSTOP LED
  if ( EngineSTOP_time == 0 ){
   if (EngineSTOP) { EngineSTOPMainLEDOn = !EngineSTOPMainLEDOn; }
   else EngineSTOPMainLEDOn = false;
   EngineSTOP_time=3;
  } else EngineSTOP_time--;

//Low Batt voltage LED
  if ( StatusLowBatt_time == 0 ){
   if (SystemStatusLowBatt) 
    {StatusLowBattMainLEDOn = !StatusLowBattMainLEDOn; 
    if (SystemStatusLowBatt==2) {
        tone(BUZZER_PIN,2300,250);
       }
    } 
   else StatusLowBattMainLEDOn = true;
  StatusLowBatt_time=1;
  } else StatusLowBatt_time--;


//reset packets counter after one second
 if (MainSWCounterReset_time==0) {
  MaxMainSWPacketsCounter=0;
  MainSWCounterReset_time=3;
 } else MainSWCounterReset_time--;


}

//-----------------------------------------------------------------
//-----------------------------------------------------------------

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
   Serial.begin(38400);
   Wire.begin();
  
  //Pin setup
  pinMode(MAINSWITCH_IN_PIN, INPUT);
  pinMode(SYSTEM_SERVICE_PIN, INPUT);
  pinMode(SERVO_TESTER_OUT_PIN, OUTPUT);
  pinMode(KILLSWITCH_OUT_PIN, OUTPUT);
  digitalWrite(KILLSWITCH_OUT_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

 //Setup the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);
  pinMode(MAIN_LED_R, OUTPUT);
  pinMode(MAIN_LED_G, OUTPUT);
  pinMode(MAIN_LED_B, OUTPUT);

 //Setup hardware interrupt point
  digitalWrite(MAINSWITCH_IN_PIN, HIGH);
  attachInterrupt(0,MainSWHandler,CHANGE);

  SystemServiceMode=false;

  SearchBuzzerOn_flag=false;

 //Setup software timer interrupt 
  int tickEvent = m_timer.every(250, MainTimerHandler);  

 //Setup system mode to first start for normal use interrupts 
  SystemMode = SYSTEM_FIRST_START;
 
}

//-----------------------------------------------------------------

void FirstTimeRunInit(){
 if (ReadEEPROMConfig()) {
        Serial.println("EEPROM Config clean. Use DEFAULTS!");
        WriteEEPROMConfig();
        } 
        else
        {Serial.println("Use EEPROM Config. Set parameters");}

 if (digitalRead(SYSTEM_SERVICE_PIN)==HIGH) {
  SystemServiceMode=true;
  myservo.attach(SERVO_TESTER_OUT_PIN);}
  RenderCurrentConfig();
  PlayMenuSound(SOUND_KSW_INTRO);

   //reset counters
   MainSWPressCounter=0;
   readMainBattVoltage();
   MainBattVoltageCount=0;
   MainBattVoltageSum=0.0;
   MainBattVoltageAverage_time=7;
   MainBattVoltageAverage=MainBattVoltage;

  //initial menu time readings
   MainSWPressLaststime=millis();  
   MenuInitialTime = millis();
}
//-----------------------------------------------------------------
//-----------------------------------------------------------------

// ================================================================
// ===                      MAIN LOOP                           ===
// ================================================================

void loop() {
  m_timer.update();
  CheckServoTester();
 switch (SystemMode) {
    case SYSTEM_NORMAL: 
      MainStateHandler();       
     break;    
    case SYSTEM_FIRST_START:
      FirstTimeRunInit();
      SystemMode=SYSTEM_NORMAL;
     break;
    case SYSTEM_MENU_M: 
      MenuMHandler();
     break;    
    case SYSTEM_MENU_10_CAL: 
      Menu10Handler();
     break;    
    case SYSTEM_MENU_20_BTYPE: 
      Menu20Handler();
     break;    
    case SYSTEM_MENU_30_RENDER: 
      Menu30Handler();
     break;    
    case SYSTEM_MENU_31_EN_SYSSTATES_RENDER: 
      Menu31Handler();
     break;    
    case SYSTEM_MENU_32_EN_BATTINFO_RENDER:
      Menu32Handler();
     break;
    case SYSTEM_MENU_33_EN_PPMINFO_RENDER:
      Menu33Handler();
     break;
    case SYSTEM_MENU_34_EN_CURRENT_RENDER:
      Menu34Handler();
     break;
    case SYSTEM_MENU_40_DEFAULTS:
      Menu40ResetDefaultsHandler();
     break;
    case SYSTEM_CALIBRATING: 
      if (CalibratingStageCounter==2 || CalibratingStageCounter==4 ) 
        {
          for (CalibratingCounter=0; CalibratingCounter <= 255; CalibratingCounter++){
           CalibratingInput(CalibratingStageCounter); }
        } else {
           CalibratingInput(CalibratingStageCounter);}
           
      CalibratingStageCounter++;
     break;    
    }
}
