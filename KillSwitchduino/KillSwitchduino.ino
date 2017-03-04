
/* Smart KillSwitchDuino for flying model aircraft with gasoline engine and electronic inginion
 * Author: NailMan
 * Controller type: Arduino Nano(Mega328)
 * Project site: http://github.com/NailAlex/KillSwitchDuino
 * 
 * Version: 2.3 (current)
 * 
 * Used Libraries
 * Timer - //http://github.com/JChristensen/Timer
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
#define SYSTEM_MENU_20_SENSORS 20                           //Select Battery Chemistry type
#define SYSTEM_MENU_30_RESETS 30                        //Reset Config to Default
#define SYSTEM_MENU_40_RENDER 40                          //Select Battery Chemistry type
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
#define SOUND_M1_INTRO 3                                  //Menu 1 INTRO melody index (RX Calibration Submenu)
#define SOUND_M2_INTRO 4                                  //Menu 2 INTRO melody index (Sensors/Battery Submenu)
#define SOUND_M3_INTRO 5                                  //Menu 3 INTRO melody index (RESET Submenu)
#define SOUND_M4_INTRO 6                                  //Menu 3 INTRO melody index (Render Info Select Submenu)

//EEPROM
#define EEPROM_INDEX_ADDRESS 29                           //Index static address for last consumed mA value
#define EEPROM_DATA_START_ADDRESS 32                      //Index static address for last consumed mA value

//COUNTERS
#define CSENSOR_READ_PERIOD 100                           //Current Sensor perion for 10FPS readings

//STRING constants
char St_LiFe[ ] = "LiFe";
char St_LiPo[ ] = "LiPo";
char St_Unk[ ] = "Unknown";
char St_Yes[ ] = "Yes";
char St_No[ ] = "No";
char St_Now[ ] = " Now: ";
char St_SetThis[ ] = " Set this?";
char St_Exit[ ] = "Exit";
char St_ChangeTo[ ] = "Change to ";



// ****************************************************************
// ****                      VARIABLES                         ****
// ****************************************************************


//Main System Mode variables
uint8_t SystemMode = SYSTEM_NORMAL;                       //System Mode state
bool SystemServiceMode = false;                           //Enable embedded servotester for PPM generating(SYSTEM_SERVICE_PIN must be HIGH)

//System state vars
bool SwitchPOS = true;                                    //Switch position. TRUE if down, FALSE if UP
bool SwitchPOS_fromUp = false;                            //from upper position flag
bool EngineSTOP = false;                                  //Switch for cut-off engine ignition
bool EngineSTOP_flag = false;                             //Stopflag for prevent fast double switching
bool LastEngineSTOP = false;                              //Last SystemEngineSTOP value for NoRX handling
uint8_t SystemStatusLowBatt = 0;                          //Low battery voltage Warning(=1) or Alert(=2)
bool SystemStatusNoRX = false;                            //No RX Signal Alert switch. If packets counter ~0 then signal lost
uint8_t RXsignalRSSI = 100;                               //Low battery voltage Warning(=1) or Alert(=2)

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
float V_REF = 5.02;                                       //Accurate voltage value from system linear regulator on device. Need manual read(with multimeter) and fill this var your device!
float MAIN_BATT_VADD = 0.03;                              //Volt correction addon. Need manual fill with multimeter compare on your device!
float MainBattVoltageSum = 0;                             //Volt sum for calc average
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
volatile unsigned long PPMPacketsTime = 0;                //Main Switch counter1 (start read position). Start after LOW->HIGH detect
volatile uint8_t MainSWPacketsCounter = 0;                //PPM packets couter  0<=X<=50
volatile uint8_t MaxMainSWPacketsCounter = 0;             //Packets per second couter  0<=X<=50
volatile uint8_t LastMaxMainSWPacketsCounter = 0;         //Copy of packets per second couter  0<=X<=50
uint8_t MainSWPacketsCounter_time = 3;                    //PacketsQuality StepTimer counter. 1FPS


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
bool RenderCurrentInfo = true;                            //Enable/disable PPM signal info to Serial Port (if CurrentSensorEnabled=true)

//BUZZER vars
uint8_t SearchBuzzer_time = 1;                            //SearchBuzzer StepTimer counter. 1FPS beeps
bool SearchBuzzerActive = false;                          //SearchBuzzer activation switch
bool SearchBuzzerOn = false;                              //SearchBuzzer ON/OFF flag
bool SearchBuzzerOn_flag = false;                         //SearchBuzzer ON/OFF flag for prevent fast double switching
int  counter = 0;

//Current Sensor
float Current_mA = 0.0;                                   //Momentary current value in mA
float Current_Max_mA = 0.0;                               //Momentary current value in mA
float Current_mAperHourConsumed = 0;                      //Consumed energy from C_Censor reset
float Current_quartAverage_mA = 0.0;                      //Average current value for 1/4 sec
uint8_t CurrentCalc_time = 3;                             //CurrentSensor StepTimer counter. 1 sec calculations
bool CurrentSensorEnabled = true;                         //Use or not the Current Sensor
byte CurrentDump_time = 39;                               //CurrentSensor consumed amps dump counter. Every 10 sec dumping
unsigned long CurrentLastMillis = 0;
int CurrentSensorScaleFactor = 40;                        // See Scale Factors Below mV per Amp
                                                          //50A bi-directional = 40
                                                          //50A uni-directional = 60
                                                          //100A bi-directional = 20
                                                          //100A uni-directional = 40
                                                          //150A bi-directional = 13.3
                                                          //150A uni-directioal = 26.7
                                                          //200A bi-directional = 10
                                                          //200A uni-directional = 20
// Set you Offset
int ACSoffset = 2500;                                     // See offsets below
                                                          //If bi-directional = 2500
                                                          //If uni- directional = 600


//EEPROM manager variables
int eepromSaveAddress = EEPROM_DATA_START_ADDRESS;       //Consumed amps value(4 bytes) eeprom start address: 32..507
uint8_t eepromSaveCounter = 0;                           //Current session save counter. 
int eepromEndAddress = 507;                              //End address for save consumed amps value(4 bytes)


// ****************************************************************
// ****                      DEVICES                           ****
// ****************************************************************

Timer m_timer;                                            //Main Timer. 250ms period for one tick. All longtime periodical changes use this timer
Servo myservo;                                            //Servo Tester Generator for NoRX operate on ground. Enabled if SYSTEM_SERVICE_PIN is HIGH and generate PPM Signal on SERVO_TESTER_OUT_PIN mapped between current PPMLowerLevel<>PPMUpperLevel.


// ================================================================
// ===           HARDWARE INTERRUPT HANDLERS                    ===
// ================================================================

void MainSWHandler(){
  if (abs(micros() - PPMPacketsTime) >=1000000) {
    LastMaxMainSWPacketsCounter = MaxMainSWPacketsCounter;
    MaxMainSWPacketsCounter=0;
    MainSWPacketsCounter=0;
    PPMPacketsTime = micros();
    }
//  if (MainSWPacketsCounter >= 50) { MainSWPacketsCounter=1;}
//  if (MaxMainSWPacketsCounterReset_flag) {MaxMainSWPacketsCounterReset_flag=false;MaxMainSWPacketsCounter=0;}
//reading PPM signal from channel
     if (digitalRead(MAINSWITCH_IN_PIN) == HIGH) //if LOW->HIGH front detected, save it!
        {MainSWTime1 = micros(); } 
     else {MainSWTime2 = micros();//if HIGH->LOW front detected, save it too!
           //counter overflow correction (~70min)
           if (MainSWTime2<MainSWTime1) {MainSWTime2=MainSWTime1+2000;}
           //calc PPM signal length and packets count
           MainSWTime = MainSWTime2 - MainSWTime1;
           MainSWPacketsCounter++;
           if (MainSWPacketsCounter>MaxMainSWPacketsCounter) {MaxMainSWPacketsCounter=MainSWPacketsCounter;}
          }

//switch handling
 if (MainSWTime >= PPMSwitchLevel)
    {//Switch in Upper position! proccessing for only one time change
     if (!SwitchPOS_fromUp){
     SwitchPOS=true;         
     SwitchPOS_fromUp=true;
     }
     }
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
 int value = analogRead(MAIN_VOLTAGE_BATT_PIN);
 float vout = (value * V_REF) / 1024.0;  
  MainBattVoltage = (vout / (MainBattR2/(MainBattR1+MainBattR2))) + MAIN_BATT_VADD;
}
  
//-----------------------------------------------------------------

void readCurrentSensorData(){
 int RawValue = analogRead(CURRENT_SENSOR_IN_PIN);
 float Voltage = (RawValue * V_REF * 1000) / 1024;                         // Gets you mV
 Current_mA = abs((Voltage - ACSoffset) / CurrentSensorScaleFactor) * 1000;
 if (Current_mA>Current_Max_mA) {Current_Max_mA=Current_mA;}
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
     tone(BUZZER_PIN,500,200);
     delay(200);
     tone(BUZZER_PIN,2300,200);
     delay(200);
     noTone(BUZZER_PIN);
    break;
   case SOUND_M2_INTRO:  //Sensors SubMenu Intro
     tone(BUZZER_PIN,1500,500);
     delay(500);
     tone(BUZZER_PIN,2300,200);
     delay(200);
     noTone(BUZZER_PIN);
    break;    
   case SOUND_M3_INTRO:  //Resets SubMenu Intro
     tone(BUZZER_PIN,1000,500);
     delay(500);
     tone(BUZZER_PIN,2300,200);
     delay(200);
     noTone(BUZZER_PIN);
    break;    
   case SOUND_M4_INTRO:  //Render info options SubMenu Intro
     tone(BUZZER_PIN,2300,500);
     delay(500);
     tone(BUZZER_PIN,1000,200);
     delay(200);
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
   CurrentSensorEnabled=true;
   RenderSystemStates = true;
   RenderBatteryInfo = true; 
   RenderPPMInfo = true;    
   RenderCurrentInfo = true;
   Current_mAperHourConsumed = 0;
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
void RenderBType(){
          switch (MainBattType) {
          case BATT_LIFE: 
            Serial.print(St_LiFe);
            break;
           case BATT_LIPO: 
            Serial.print(St_LiPo); 
            break;
            default:
            Serial.print(St_Unk);
          }  
}

//-----------------------------------------------------------------

void RenderCurrentConfig(){
  Serial.println("Config:");
  Serial.print("B_Type: ");RenderBType();
  Serial.print(" | C_Sensor Enable: "); 
  if (CurrentSensorEnabled) {Serial.print(St_Yes);} else {Serial.print(St_No);}
  Serial.print(" | PPM Up/Lo/Sw: ");Serial.print(PPMUpperLevel);Serial.print("/"); Serial.print(PPMLowerLevel);Serial.print("/"); Serial.print(PPMSwitchLevel);
  Serial.print(" | R_Options (Stats/Bat/RX/CurSensor) : "); Serial.print(RenderSystemStates); Serial.print("/");
                                                            Serial.print(RenderBatteryInfo);Serial.print("/");
                                                            Serial.print(RenderPPMInfo);Serial.print("/");
                                                            Serial.println(RenderCurrentInfo);
}

//-----------------------------------------------------------------

unsigned int EEPROMReadInt(unsigned int p_address)
        {
        byte lowByte = EEPROM.read(p_address);
        byte highByte = EEPROM.read(p_address + 1);
        return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
        }

//-----------------------------------------------------------------

void EEPROMWriteInt(int p_address, unsigned int p_value)
        {
        byte lowByte = ((p_value >> 0) & 0xFF);
        byte highByte = ((p_value >> 8) & 0xFF);
        EEPROM.write(p_address, lowByte);
        EEPROM.write(p_address + 1, highByte);
        }

//-----------------------------------------------------------------

float EEPROMReadFloat(int addr) {   
    byte raw[4];
   for(byte i = 0; i < 4; i++) raw[i] = EEPROM.read(addr+i);
    float &num = (float&)raw;
    return num;
  }

//-----------------------------------------------------------------

void EEPROMWriteFloat(int addr, float num) {
    byte raw[4];
    (float&)raw = num;
    for(byte i = 0; i < 4; i++) EEPROM.write(addr+i, raw[i]);
  }  

//-----------------------------------------------------------------

void ReadEEPROMindex(){
  eepromEndAddress = EEPROM.length() - 4;
  eepromSaveAddress = EEPROMReadInt(EEPROM_INDEX_ADDRESS);
   if (eepromSaveAddress < EEPROM_DATA_START_ADDRESS || eepromSaveAddress > eepromEndAddress) 
    {eepromSaveAddress=EEPROM_DATA_START_ADDRESS; Current_mAperHourConsumed=0.0;} 
    else
    {Current_mAperHourConsumed = EEPROMReadFloat(eepromSaveAddress);
    eepromSaveAddress+=4;
    EEPROMWriteInt(EEPROM_INDEX_ADDRESS,eepromSaveAddress);
    eepromSaveCounter=49;   }
}


//-----------------------------------------------------------------

void DumpConsToEEPROM(){
 if (!StatusLowBattMainLEDOn) {analogWrite(MAIN_LED_B,255);} else {analogWrite(MAIN_LED_B,LOW);}
 if (eepromSaveCounter==0) {
     eepromSaveAddress+=4;
     if (eepromSaveAddress>eepromEndAddress) {eepromSaveAddress=EEPROM_DATA_START_ADDRESS;}
      EEPROMWriteInt(EEPROM_INDEX_ADDRESS,eepromSaveAddress);
    eepromSaveCounter=49;
  } else eepromSaveCounter--;
 EEPROMWriteFloat(eepromSaveAddress,Current_mAperHourConsumed);   
if (!StatusLowBattMainLEDOn) {analogWrite(MAIN_LED_B,LOW);} else {analogWrite(MAIN_LED_B,255);}
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
  eepromSaveAddress = 32;
  eepromSaveCounter=0;
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
   if (b==3) {CurrentSensorEnabled=true;} else {CurrentSensorEnabled=false;}
   b=EEPROM.read(9);
   if (b==3) {RenderSystemStates=true;} else {RenderSystemStates=false;}
   b=EEPROM.read(10);
   if (b==3) {RenderBatteryInfo=true;} else {RenderBatteryInfo=false;}
   b=EEPROM.read(11);
   if (b==3) {RenderPPMInfo=true;} else {RenderPPMInfo=false;}
   b=EEPROM.read(12);
   if (b==3) {RenderCurrentInfo=true;} else {RenderCurrentInfo=false;}
   unsigned int S;
    if (CurrentSensorEnabled) {ReadEEPROMindex();}
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
  if (CurrentSensorEnabled) {EEPROM.write(8, 3);} else {EEPROM.write(8, 5);}
  if (RenderSystemStates) {EEPROM.write(9, 3);} else {EEPROM.write(9, 5);}
  if (RenderBatteryInfo) {EEPROM.write(10, 3);} else {EEPROM.write(10, 5);}
  if (RenderPPMInfo) {EEPROM.write(11, 3);} else {EEPROM.write(11, 5);}
  if (RenderCurrentInfo) {EEPROM.write(12, 3);} else {EEPROM.write(12, 5);}
  //write header2
  EEPROM.write(31,35);
}

//-----------------------------------------------------------------

//-----------------------------------------------------------------

// ================================================================
// ===                     OTHER FUNCTIONS                      ===
// ================================================================

void ShowValue(const byte smode,const byte item){
 
 if (smode==SYSTEM_MENU_M) {
    switch (item) {
    case 1: 
      Serial.print("Calibrate RX");
    break;
    case 2: 
      Serial.print("Sensors/Battery");
    break;
    case 3: 
      Serial.print("Resets");
    break;
    case 4: 
      Serial.print("Render");
    break;
    case 5: 
     Serial.print(St_Exit);
    break;    
   }
 }

 if (smode==SYSTEM_MENU_10_CAL) {
    switch (item) {
    case 1: 
      Serial.print("Start calibrating");
    break;
    case 2: 
     Serial.print(St_Exit);
    break;    
   }
 }


 if (smode==SYSTEM_MENU_20_SENSORS) {
    switch (item) {
    case 1: 
      Serial.print(St_ChangeTo);Serial.print(St_LiFe);Serial.print(St_Now);RenderBType();Serial.print(St_SetThis); 
    break;
    case 2: 
      Serial.print(St_ChangeTo);Serial.print(St_LiPo);Serial.print(St_Now);RenderBType();Serial.print(St_SetThis); 
    break;
    case 3: 
     Serial.print("C_Sensor enabled: "); if (CurrentSensorEnabled) Serial.print(St_Yes); else Serial.print(St_No);
    break;    
    case 4: 
     Serial.print(St_Exit);
    break;    
   }
 }

 if (smode==SYSTEM_MENU_30_RESETS) {
    switch (item) {
    case 1: 
      Serial.print("C_Sensor Reset?");
    break;
    case 2: 
      Serial.print("Config Reset?");
    break;
    case 3: 
     Serial.print(St_Exit);
    break;    
   }
 }

 if (smode==SYSTEM_MENU_40_RENDER) {
    switch (item) {
    case 1: 
     Serial.print("Show Sys States: "); if (RenderSystemStates) Serial.print(St_Yes); else Serial.print(St_No);
    break;    
    case 2: 
     Serial.print("Show Batt Info: "); if (RenderBatteryInfo) Serial.print(St_Yes); else Serial.print(St_No);
    break;    
    case 3: 
     Serial.print("Show PPM Info: "); if (RenderPPMInfo) Serial.print(St_Yes); else Serial.print(St_No);
    break;    
    case 4: 
     Serial.print("Show C_Sensor info: "); if (RenderCurrentInfo) Serial.print(St_Yes); else Serial.print(St_No);
    break;    
    case 5: 
     Serial.print(St_Exit);
    break;    
   }
 }
}

//-----------------------------------------------------------------

void RenderMenuNavigation(){
  switch (SystemMode) {
  case SYSTEM_MENU_M: 
    Serial.print("MAIN MENU"); 
    break;
   case SYSTEM_MENU_10_CAL: 
    Serial.print("M1_CAL"); 
    break;   
   case SYSTEM_MENU_20_SENSORS: 
    Serial.print("M2_SENS"); 
    break;    
   case SYSTEM_MENU_30_RESETS: 
    Serial.print("M3_RESETS"); 
    break;
   case SYSTEM_MENU_40_RENDER: 
    Serial.print("M4_RENDER"); 
    break;
 }
  Serial.print(" | Item: "); Serial.print(MenuItem);Serial.print(" "); ShowValue(SystemMode,MenuItem);
  Serial.println(" ");
}

//-----------------------------------------------------------------

void RenderTelemetry(){

  if (SystemMode==SYSTEM_NORMAL) {
  Serial.print("NORMAL ");
  Serial.print(RXsignalRSSI);Serial.print(" "); 
  }  
 
  //System states render
  if (RenderSystemStates) {
  Serial.print("KSW: ");Serial.print(!EngineSTOP);Serial.print("/"); 
                        Serial.print(LastMaxMainSWPacketsCounter);Serial.print("/"); 
                        Serial.print(MainSWPressCounter);
  Serial.print(" | Serv/NoRX/LowBatt/Buzz: ");Serial.print(SystemServiceMode);Serial.print("/");
                                              Serial.print(SystemStatusNoRX);Serial.print("/");
                                              Serial.print(SystemStatusLowBatt);Serial.print("/");
                                              Serial.print(SearchBuzzerActive);
  }

  //Battery status render
  if (RenderBatteryInfo) {
  Serial.print(" | V/AvgV/Type: ");Serial.print(MainBattVoltage);Serial.print("/");Serial.print(MainBattVoltageAverage);Serial.print("/");RenderBType();
  }

  //RX info render
  if (RenderPPMInfo) {
  Serial.print(" | PPM Gen/Read/SW: ");Serial.print(ServoOutMS);Serial.print("/");Serial.print(MainSWTime);Serial.print("/");Serial.print(PPMSwitchLevel);}

  if (RenderCurrentInfo && CurrentSensorEnabled) {
  Serial.print(" | CSens mA/Max/Cons: ");Serial.print(Current_mA);Serial.print("/");Serial.print(Current_Max_mA);Serial.print("/");Serial.print(Current_mAperHourConsumed);
  }
  
  Serial.println("");
 
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
   case 2:   //Go To Battery chemistry type and Sensors select 
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_20_SENSORS;MenuItem=0;}
    break;
   case 3:   //Go To RESETS procedures
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_30_RESETS;MenuItem=0;}
    break;
   case 4:   //Go To Render options select 
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_40_RENDER;MenuItem=0;}
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

 if (MenuItem==0) {PlayMenuSound(SOUND_M1_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);} 
 else{
   MenuItemSelected=MenuInputHandler();

 //Play Menu Item selection sound
 if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==2) {MenuItem=1;} else {MenuItem++;}   
  MenuIntroPlayed=false; }
    
 if (MenuItemSelected) {
 switch (MenuItem) {
   case 1:   //Start RX Calibration procedure
    CalibratingStageCounter=1;SystemMode=SYSTEM_CALIBRATING;
   break;    
   case 2:  //Exit to Main Menu
    MenuItem=0;SystemMode=SYSTEM_MENU_M;
   break;    
 }
 }
 }
}

//-----------------------------------------------------------------

void Menu20Handler (){
//Select battery chemistry SubMenu
//1 beep - Select LiFe battery type
//2 beep - Select Li-Polymer battery type
//3 beep - Enable or Disable Current Sensor usage
//4 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M2_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);} 
  else{ MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==4) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
 if (MenuItemSelected) {
 switch (MenuItem) {
   case 1:   //Select LiFePO4
         MainBattType=BATT_LIFE;
         SetupBatteryLevels(); 
         MenuItem=0;
         MenuIntroPlayed=false;
    break;    
   case 2:  //Select Li-Polymer
         MainBattType=BATT_LIPO;
         SetupBatteryLevels(); 
         MenuItem=0;
         MenuIntroPlayed=false;
    break;    
   case 3:  //Enable or Disable Current Sensor usage
         CurrentSensorEnabled=!CurrentSensorEnabled;
         if (CurrentSensorEnabled) {tone(BUZZER_PIN,2300,400);} else {tone(BUZZER_PIN,300,400);}
         MenuItem=0; 
         MenuIntroPlayed=false;
    break;    
   case 4:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_M;}
   break;    
   }
 } 
 }
}

//-----------------------------------------------------------------

void Menu30Handler (){
//RESETs SubMenu
//1 beep - Reset CurrentSensor data and exit to Main Menu
//2 beep - Reset Configuration to DEFAULTS and exit to Main Menu
//3 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M3_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);
 } 
  else{ MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==3) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
 if (MenuItemSelected) {
 switch (MenuItem) {
   case 1:   //Reset CurrentSensor data
       Serial.println("Reset C_Sensor");
       Current_Max_mA = 0.0;
       Current_mAperHourConsumed = 0;
       MenuItem=0;
       SystemMode=SYSTEM_MENU_M;
    break;    
   case 2:  //Go to Enable/Disable Battery info render SubSubMenu (32)
       Serial.println("Reset config");
       ResetSystemConfig();
       RenderCurrentConfig();
       MenuItem=0;
       SystemMode=SYSTEM_MENU_M;
    break;    
   case 3:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_M;}
   break;    
  }
 }
 } 
}


//-----------------------------------------------------------------

void Menu40Handler (){
//Select Serial Port Render Options SubMenu
//1 beep - Show/Hide System States info
//2 beeps - Show/Hide Battery info
//3 beeps - Show/Hide RX info
//4 beeps - Show/Hide Current Sensor info
//5 beeps - Exit to Main Menu
 if (MenuItem==0) {PlayMenuSound(SOUND_M4_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);} 
 else{  MenuItemSelected=MenuInputHandler();

  //Play Menu Item selection sound
  if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==5) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 

   
if (MenuItemSelected) {
switch (MenuItem) {
   case 1:   
         RenderSystemStates=!RenderSystemStates;
         if (RenderSystemStates) {tone(BUZZER_PIN,2300,400);} else {tone(BUZZER_PIN,300,400);}
         MenuItem=0; 
         MenuIntroPlayed=false;
    break;    
   case 2:  
         RenderBatteryInfo=!RenderBatteryInfo;
         if (RenderBatteryInfo) {tone(BUZZER_PIN,2300,400);} else {tone(BUZZER_PIN,300,400);}
         MenuItem=0; 
         MenuIntroPlayed=false;
    break;    
   case 3:  
         RenderPPMInfo=!RenderPPMInfo;
         if (RenderPPMInfo) {tone(BUZZER_PIN,2300,400);} else {tone(BUZZER_PIN,300,400);}
         MenuItem=0;  //Repeat this item for change
         MenuIntroPlayed=false;
    break;    
   case 4:  
         RenderCurrentInfo=!RenderCurrentInfo;
         if (RenderCurrentInfo) {tone(BUZZER_PIN,2300,400);} else {tone(BUZZER_PIN,300,400);}
         MenuItem=0;  //Repeat this item for change
         MenuIntroPlayed=false;
    break;    
   case 5:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_M;}
   break;    
 }
 }
 }
}


//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
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
      for (i=0; i <= 7; i++){
       CalibrationUpperLevel=MainSWTime;
       delayMicroseconds(1000); 
      }
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
      for (i=0; i <= 7; i++){
       CalibrationLowerLevel=MainSWTime;
       delayMicroseconds(1000); 
      }
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
unsigned long mil;
//Engine STOP handling
//in Normal operating mode Engine stop signal equal Switch position: lower pos = OFF, upper pos = ON
 if (SystemMode==SYSTEM_NORMAL){
    //Set EngineSTOP with Switch position Up=FALSE, Down=TRUE
     if (RXsignalRSSI>4) EngineSTOP=!SwitchPOS;
     }
      
if (!SystemStatusNoRX) {
 if (!EngineSTOP) { //if SwitchPOS in Upper position(ignition is ON) then setup output pin to HIGH one time!
  //prevent high speed duplicate switching with potential glitches with flags 
  if (!EngineSTOP_flag) {
     digitalWrite(KILLSWITCH_OUT_PIN, HIGH); 
     tone(BUZZER_PIN,2300,30);
     EngineSTOP_flag=true;
  }
 } 
 else 
 { //if SwitchPOS in Lower position(ignition is OFF) then setup output pin to LOW one time!
   //prevent high speed duplicate switching with potential glitches.
  if (EngineSTOP_flag)
      {digitalWrite(KILLSWITCH_OUT_PIN, LOW);
       tone(BUZZER_PIN,200,30); 
       EngineSTOP_flag=false;}
 }
}
  mil = millis();
//if Switch pressed(from Up to Down) then handling options  
   if (mil-MainSWPressLaststime<=BUZZER_SEARCHER_PERIOD && MainSWPressCounter>=1) 
   { 
    switch (MainSWPressCounter) {
    case 3: 
     if (mil - MenuInitialTime <= MENU_INITIAL_PERIOD){
     //Switched 3 times within Initial period - GO TO Menu Mode!
       SystemMode=SYSTEM_MENU_M;} 
     else {SearchBuzzerActive=!SearchBuzzerActive;
           LED13Active=SearchBuzzerActive;                     //diagnostic LED13
           MainSWPressCounter=0;
           MainSWPressCounterChange=false;
           MainSWPressLaststime=mil;}
          break; 
    case 1: 
     //start time for BUZZER_SEARCHER_PERIOD period
     if (MainSWPressCounterChange) {MainSWPressLaststime=mil; MainSWPressCounterChange=false;}     
      break;    
    } 
  }

 //reset Switch press counter if passed BEEPER_ACT_PERIOD time period   
 if (mil-MainSWPressLaststime>BUZZER_SEARCHER_PERIOD) {MainSWPressCounter=0; MainSWPressLaststime=mil; } 
    
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
 //unsigned long t1 = micros();
//read input data from sensors
 readMainBattVoltage();
 
//calc voltage sum for one more 250ms step
 MainBattVoltageSum+=MainBattVoltage;
 
 if ( MainBattVoltageAverage_time == 0 ){
   MainBattVoltageAverage=MainBattVoltageSum / 8;
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



//processing RX signal quality
 if ( MainSWPacketsCounter_time == 0 ){
 if (LastMaxMainSWPacketsCounter<=20) {
  SystemStatusNoRX=true;
  RXsignalRSSI = 0;
  } else {SystemStatusNoRX=false;}
 if (LastMaxMainSWPacketsCounter>1 && LastMaxMainSWPacketsCounter<=10) 
 {RXsignalRSSI = 1;}
 if (LastMaxMainSWPacketsCounter>10 && LastMaxMainSWPacketsCounter<=20) 
 {RXsignalRSSI = 2;}
 if (LastMaxMainSWPacketsCounter>20 && LastMaxMainSWPacketsCounter<=30) 
 {RXsignalRSSI = 3;}
 if (LastMaxMainSWPacketsCounter>30 && LastMaxMainSWPacketsCounter<=40) 
 {RXsignalRSSI = 4;}
 if (LastMaxMainSWPacketsCounter>40) {RXsignalRSSI = 5;}
  MainSWPacketsCounter_time=3;
 } else MainSWPacketsCounter_time--;


//dump consumed mA to EEPROM 
 if (CurrentSensorEnabled) {

  readCurrentSensorData();
   Current_quartAverage_mA=(Current_quartAverage_mA + Current_mA) / 2;
   Current_mAperHourConsumed+=Current_quartAverage_mA / 14400; 

 if (CurrentDump_time==0) {
   DumpConsToEEPROM();
   CurrentDump_time=39;
  } else CurrentDump_time--;}

//Render telemetry 
 if ( Render_time == 0 )
 {
   if (SystemMode==SYSTEM_NORMAL) {RenderTelemetry();}
   else {RenderMenuNavigation();}
  Render_time=3;
  } else Render_time--;

//Buzzer flashing
  if ( SearchBuzzer_time == 0 ){
    SearchBuzzerOn = !SearchBuzzerOn;
    SearchBuzzer_time=1;
  } else SearchBuzzer_time--;


//LED flashing routines
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
//Serial.println(micros()-t1);
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
   Serial.begin(38400);
   Wire.begin();
  
//Setup the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);
  pinMode(MAIN_LED_R, OUTPUT);
  pinMode(MAIN_LED_G, OUTPUT);
  pinMode(MAIN_LED_B, OUTPUT);

//Setup PPM and Servo pins
  pinMode(MAINSWITCH_IN_PIN, INPUT);
  digitalWrite(MAINSWITCH_IN_PIN, HIGH);
  pinMode(KILLSWITCH_OUT_PIN, OUTPUT);
//attach hardware interrupt with ANY CHANGE option
  attachInterrupt(0,MainSWHandler,CHANGE);

//Setup Buzzer Pin   
  pinMode(BUZZER_PIN, OUTPUT);
//  digitalWrite(BUZZER_PIN, LOW);

//Setup Service Mode pin
 pinMode(SYSTEM_SERVICE_PIN, INPUT);
 if (digitalRead(SYSTEM_SERVICE_PIN)==HIGH) {
 pinMode(SERVO_TESTER_OUT_PIN, OUTPUT);
 SystemServiceMode=true;
  myservo.attach(SERVO_TESTER_OUT_PIN);} 
 else {SystemServiceMode=false;}

 //Setup software timer interrupt. 250ms cycle
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
        {Serial.println("Use EEPROM Config");}


//Setup voltages
   readMainBattVoltage();
   MainBattVoltageAverage=MainBattVoltage;
   
   //reset counters
   MainSWPacketsCounter=0;
   MainSWPressCounter=0;
   MainBattVoltageSum=0.0;
   MainBattVoltageAverage_time=7;
   MaxMainSWPacketsCounter=0;

//Setup flags
   EngineSTOP_flag=false; 
   SearchBuzzerOn_flag=false;

//First MainSwitch reading   
   if (MainSWTime >= PPMSwitchLevel)
    {
     //Switch in Upper position
     SwitchPOS=true;
     SwitchPOS_fromUp=true;} else 
     {
     //Switch in Lower position
     SwitchPOS=false;
     SwitchPOS_fromUp=false;}

  PlayMenuSound(SOUND_KSW_INTRO);
  RenderCurrentConfig();

  //initial menu time readings
   MainSWPressLaststime=millis();  
   MenuInitialTime = millis();
   CurrentLastMillis = millis();
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
    case SYSTEM_MENU_20_SENSORS: 
      Menu20Handler();
     break;    
    case SYSTEM_MENU_30_RESETS:
      Menu30Handler();
     break;
    case SYSTEM_MENU_40_RENDER: 
      Menu40Handler();
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
