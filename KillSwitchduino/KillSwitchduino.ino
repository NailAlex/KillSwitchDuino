/* Smart KillSwitchDuino for flying model aircraft with gasoline engine and electronic inginion
 * Author: NailMan
 * Controller type: Arduino Nano(Mega328)
 * Version: 1.1
 * Features:
 * 1 - Handle PPM signal from RX for Kill Switching operate
 * 2 - Service mode(PIN D12 HIGH/LOW) for activate embedded servotester with external 10K Ohm variable resistor(PIN A1). No TX/RX needed!
 * 3 - Embedded RX voltage reading, analyze and blocking killswitch working if battery voltage is Low/Very Low for 2 sec average value(Level1 - soft block/Level2 - hard block)
 * 4 - Beeper for seaching mode(0.5sec cycle), audialize kill switching (short ping) and Imperial March for Level2 Voltage Alert
 * 5 - Three-color LED indicator: Greed solid - NORMAL, Green flash - Batt Low Voltage, Red flashing - Engine Off, Blue flash - No RX signal.
 * 6 - Serial port telemetry output for external serial devices(Bluetooth)
 * 7 - Configuration mode (like ESC) with EPPROM config saving:
 * a) RX signal calibrating procedure
 * b) Engine stopping if no RX signal
 * c) RX battery chemistry type select(LiPO, LiFE) for correct battery protecting
 * d) reset to Defaults
 * e) exit to normal mode
 */


//Include Section
//Core includes
#include <Wire.h>
#include <Timer.h>  
#include <Servo.h>
#include <EEPROM.h>
 
//Sensors PINs
#define MainBattVoltagePin A0  
  
//LEDs
#define MAIN_LED_R 10
#define MAIN_LED_G 9
#define MAIN_LED_B 6
#define LED_PIN 13

//Servos and PPM
#define SERVO_TEST_OUTPIN 5              //Embedded servotester outbound pin(parallel with KILLSWITCH_INPIN on device)
#define SERVO_TEST_INPIN A1              //Embedded servotester inbound pin
#define KILLSWITCH_INPIN 2               //Main killswitch inbound pin 
#define KILLSWITCH_OUTPIN 7              //Killswitch outbound pin for MOSFET switching
#define DEF_KILLSWITCH_LEVEL 1500        //Default switching level in ms (1000<SwitchLevel<2000)
#define DEF_LOWER_PPM_LEVEL 1000
#define DEF_UPPER_PPM_LEVEL 2000
#define SYSTEM_SERVICE_PIN 12

//System States
#define SYSTEM_NORMAL 0
#define SYSTEM_FIRST_START 1
#define SYSTEM_IMPERIAL_MARCH 6
#define SYSTEM_MENU_M 21 
#define SYSTEM_MENU_1_CAL 22
#define SYSTEM_MENU_2_BTYPE 23
#define SYSTEM_MENU_3_RX 24
#define SYSTEM_MENU_4_DEFAULTS 25
#define SYSTEM_CALIBRATING 40

//Menus 
#define MENU_INITIAL_PERIOD 5000           //Time period from SYSTEM_FIRST_START for enter in Menu Mode
#define MENU_CHOICE_PERIOD 3000            //Time period for user choice

//BEEPER
#define BEEPER_PIN 11
#define BEEPER_ACT_PERIOD 2000  //Time limits within activate Beeper if >=3 time EngineStop pushed
#define BEEPER_FREQ 2300        //For feature use
#define SOUND_MM_INTRO 1
#define SOUND_MM_OUTRO 2
#define SOUND_M1_INTRO 3
#define SOUND_M2_INTRO 4
#define SOUND_M3_INTRO 5
#define SOUND_M4_INTRO 6


//Battery Section
//all battery types default as 2S
#define BATT_LIFE 17
#define BATT_LIPO 21
const float BATT_LIFE_LEVEL1 = 6.6;
const float BATT_LIFE_LEVEL2 = 6.1;
const float BATT_LIPO_LEVEL1 = 7.3;
const float BATT_LIPO_LEVEL2 = 7.1;
#define BATT_AVERAGE_METER_RANGE 2000   //battery voltage time range for calc average

//Variables
//System state vars
bool SwitchPOS = false;          //Switch position. TRUE if down, FALSE if UP
bool SwitchPOSUp = false;        //from upper position flag
bool EngineSTOP = false;         //Red flashig color MainLED if True (Engine Stopped) / OFF if On
bool EngineSTOPflag = false;     //stopflag for one time switching
bool LastEngineSTOP = false;
bool StatusLowBatt = false;      //Green flashing color MainLED if True / solid if false(input voltage OK)
bool StatusNoRX = false;         //Blue fast flashing color MainLED if true / OFF if input signal OK
bool LED13Active = false;        //service LED activity flag
bool UseNoRXEngineSTOP = false;

//main system mode var
uint8_t SystemMode = SYSTEM_NORMAL; //SYSTEM_NORMAL - Main operate mode
                                    //SYSTEM_FIRST_START - one time initiating after setup() passing
                                    //SYSTEM_IMPERIAL_MARCH - Imperial march mode
                                    //SYSTEM_MENU_M - Main Menu page
                                    //SYSTEM_MENU_1_CAL - Calibration Menu page
                                    //SYSTEM_MENU_2_BTYPE - Battery Type Select Menu page
                                    //SYSTEM_MENU_3_RX - NoRX EngineOff Select Menu page
bool SystemServiceMode = false;     //enable embedded servotester for PPM generating(SYSTEM_SERVICE_PIN must be HIGH)


//Menus variables
bool MenuMode = false;     
bool MenuIntroPlayed = false;         
uint8_t MenuItem = 0;              //Menu Item counter. if =0 then just entering to menu and need play intro sound
                                   //Proposed to the selection menu Item.
                                   //1 (1 short beep) - Go to Calibration Menu page
                                   //2 (2 short beeps) - Go to Battery Type Select Menu page
                                   //3 (3 short beeps) - Go to NoRX EngineOff Select Menu page
                                   //4 (4 short beeps) - Exit to Menu Mode to normal

unsigned long MenuInitialTime = 0; //Start Time(in ms) after SYSTEM_FIRST_START procedure
unsigned long MenuItemTime = 0;    //Current Item start Time(in ms)
bool MenuItemSelected = false;



//SoftTimers vars
Timer m_timer;
  
//Voltage vars
byte MainBattType = BATT_LIFE;
float MainBattVoltage = 0.0;
float MainBattR1 = 14966.0;                    //Volt devider upper leg resistor (+Vin -> read point) 
float MainBattR2 = 9856.0;                     //Volt devider lower leg resistor (read point -> GND) 
float V_REF = 5.05;                            //accurate value of the voltage from system linear regulator
float MAIN_BATT_VADD = 0.03;                   //Volt correction factor
float MainBattVoltageSum = 0;                  //Volt sum for calc average
byte MainBattVoltageCount = 0;                  //Volt reading count for calc average
float MainBattVoltageAverage = 0;              //Average voltage within reading period
byte MainBattVoltageAveragetime = 7;           //1 time per 2 seconds period
float MainBattLowVoltageLevel1 = 0;            //Current Level1 for blocking ignition and Blue MainLED blinking
float MainBattLowVoltageLevel2 = 0;            //Current Level2 for all blocking ignition and activate Imperial March
  
//Servo and PPM vars
unsigned int ServoOutMS = 0;                   //Intergated ServoTester output PPM signal value to KillSwitch pin
unsigned int ServoInValue = 0;                 //Intergated ServoTester input PPM signal value from potentiometer
Servo myservo;        
unsigned int ServoLastOutMS = 0;               //stopflag for one time switching
int PPMLowerLevel = DEF_LOWER_PPM_LEVEL;
int PPMUpperLevel = DEF_UPPER_PPM_LEVEL;
int PPMSwitchLevel = DEF_KILLSWITCH_LEVEL;
int CalibrationUpperLevel = 0;
int CalibrationLowerLevel = 0;
int CalibrationSwitchLevel = 0;
int CalibratingStageCounter = 1;
int CalibratingCounter = 0;

//Main Switch vars
volatile unsigned long KillSWTime = 0;  //Input Switch microsec length
volatile unsigned long KillSWTime1;     //Input Switch microsec counter1
volatile unsigned long KillSWTime2;     //Input Switch microsec counter2
volatile uint8_t KillSWPressCounter = 0;                //Input Switch press counter(From ON to OFF)
volatile bool KillSWPressCounterChange = false;      //Input Switch press counter flag
volatile unsigned long KillSWPressLaststime = 0;              //Input Switch press counter LastTime
volatile byte KillSWCounter = 0;        //packets couter1  0<=X<=50
volatile uint8_t KillSWCounterTime = 3;             //Timer divider for count
volatile uint8_t MaxKillSWCounter = 0;              //packets couter2  0<=X<=50



//Render vars
uint8_t Rendertime = 1;                     //2 times per second telemetry render to Serial port
uint8_t LED13time = 3;                      //1 time per second flashing
bool LED13On = false;                       //service LED13 ON/OFF flag
bool LED13Onflag = false;                   //LED13 flag for one time switching
uint8_t EngineSTOPtime = 3;                 //1 time per second flashing (red)
bool EngineSTOPMainLEDOn = false;           //EngineSTOP ON/OFF flag
bool EngineSTOPMainLEDOnflag = false;       //EngineSTOP flag for one time switching
uint8_t StatusLowBatttime = 1;              //2 times per second flashing (greed)
bool StatusLowBattMainLEDOn = false;        //LowBatt ON/OFF flag
bool StatusLowBattMainLEDOnflag = false;    //LowBatt flag for one time switching
bool StatusNoRXMainLEDOn = false;           //NoRX ON/OFF flag (4 times per second blue flashing, no timer counter needed)
bool StatusNoRXMainLEDOnflag = false;       //NoRX flag for one time switching


//Beeper vars
uint8_t BeeperTime = 1;                     //1 time per second beeps
bool BeeperActive = false;                  //beeper activate switch flag
bool BeeperOn = false;                      //Beeper ON/OFF flag
bool BeeperOnflag = false;                  //Beeper ON/OFF flag for one time switching
bool BeeperPingFlag = false;                //Beeper Ping flag

//EMPERIAL MARCH
const int c = 261;
const int d = 294;
const int e = 329;
const int f = 349;
const int g = 391;
const int gS = 415;
const int a = 440;
const int aS = 455;
const int b = 466;
const int cH = 523;
const int cSH = 554;
const int dH = 587;
const int dSH = 622;
const int eH = 659;
const int fH = 698;
const int fSH = 740;
const int gH = 784;
const int gSH = 830;
const int aH = 880;
const int buzzerPin = BEEPER_PIN;
const int ledPin1 = MAIN_LED_G;
const int ledPin2 = MAIN_LED_B;
int counter = 0;
 

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


 
void setup() {
  // put your setup code here, to run once:
 Serial.begin(38400);
 Wire.begin();
 
 //Setup the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);
  pinMode(MAIN_LED_R, OUTPUT);
  pinMode(MAIN_LED_G, OUTPUT);
  pinMode(MAIN_LED_B, OUTPUT);
  
//Setup Servo and PPM
  pinMode(SERVO_TEST_OUTPIN, OUTPUT);
  myservo.attach(SERVO_TEST_OUTPIN);
  pinMode(KILLSWITCH_INPIN, INPUT);
  pinMode(KILLSWITCH_OUTPIN, OUTPUT);
  digitalWrite(KILLSWITCH_INPIN, HIGH);
  attachInterrupt(0,KillSWHandler,CHANGE);

//Setup timers, run every 250ms
int tickEvent = m_timer.every(250, MainTimerHandler);  

//Setup system mode to first start for normal use interrupts 
 SystemMode = SYSTEM_FIRST_START;
 
}

//-----------------------------------------------------------------

void FirstTimeRunInit(){
   //reset counters
   KillSWPressLaststime=millis();  
   KillSWPressCounter=0;
   EngineSTOPflag=false; 
   readMainBattVoltage();
   MainBattVoltageCount=0;
   MainBattVoltageSum=0.0;
   MainBattVoltageAveragetime=7;
   MainBattVoltageAverage=MainBattVoltage;

   //ResetEEPROMConfig();
   if (!ReadEEPROMConfig()) {
        Serial.println("EEPROM Config clean. Use DEFAULTS!");
        WriteEEPROMConfig();} else
        {Serial.println("Use EEPROM Config. Set parameters");}

        RenderCurrentConfig();
 

    if (KillSWTime >= PPMSwitchLevel)
    {
     //Switch in Upper position
     SwitchPOS=false;
     SwitchPOSUp=true;} else 
     {
     //Switch in Upper position
     SwitchPOS=true;
     SwitchPOSUp=false;}

  
  
  Serial.println("Starting readings");

  //initial menu time reading
  MenuInitialTime = millis();
  

}


// ================================================================
// ===                 HARDWARE READINGS                        ===
// ================================================================

void readMainBattVoltage(){
int value =0;
float vout=0;
  value = analogRead(MainBattVoltagePin);
  vout = (value * V_REF) / 1024.0;
  MainBattVoltage = (vout / (MainBattR2/(MainBattR1+MainBattR2))) + MAIN_BATT_VADD;
}
  
//-----------------------------------------------------------------


void readServoTester(){
 ServoInValue=analogRead(SERVO_TEST_INPIN);
 ServoOutMS = map(ServoInValue,0,1023,PPMLowerLevel,PPMUpperLevel);
}

//-----------------------------------------------------------------


// ================================================================
// ===            OTHER FUNCTIONS/PROCEDURES                    ===
// ================================================================

//Imperial march section
void beep(int note, int duration)
{
//Play tone on buzzerPin
tone(buzzerPin, note, duration);
 
//Play different LED depending on value of 'counter'
if(counter % 2 == 0)
{
digitalWrite(ledPin1, HIGH);
delay(duration);
digitalWrite(ledPin1, LOW);
}else
{
digitalWrite(ledPin2, HIGH);
delay(duration);
digitalWrite(ledPin2, LOW);
}
 
//Stop tone on buzzerPin
noTone(buzzerPin);
 
delay(50);
 
//Increment counter
counter++;
}

void firstSection()
{
beep(a, 500);
beep(a, 500);
beep(a, 500);
beep(f, 350);
beep(cH, 150);
beep(a, 500);
beep(f, 350);
beep(cH, 150);
beep(a, 650);
 
delay(500);
 
beep(eH, 500);
beep(eH, 500);
beep(eH, 500);
beep(fH, 350);
beep(cH, 150);
beep(gS, 500);
beep(f, 350);
beep(cH, 150);
beep(a, 650);
 
delay(500);
}
 
void secondSection()
{
beep(aH, 500);
beep(a, 300);
beep(a, 150);
beep(aH, 500);
beep(gSH, 325);
beep(gH, 175);
beep(fSH, 125);
beep(fH, 125);
beep(fSH, 250);
 
delay(325);
 
beep(aS, 250);
beep(dSH, 500);
beep(dH, 325);
beep(cSH, 175);
beep(cH, 125);
beep(b, 125);
beep(cH, 250);
 
delay(350);
}

void ImperialMarch(){
//Play first section
firstSection();
 
//Play second section
secondSection();
 
//Variant 1
beep(f, 250);
beep(gS, 500);
beep(f, 350);
beep(a, 125);
beep(cH, 500);
beep(a, 375);
beep(cH, 125);
beep(eH, 650);
 
delay(500);

 
//Repeat second section
secondSection();
 
//Variant 2
beep(f, 250);
beep(gS, 500);
beep(f, 375);
beep(cH, 125);
beep(a, 500);
beep(f, 375);
beep(cH, 125);
beep(a, 650);
 
delay(650);



}

//-----------------------------------------------------------------

void ResetEEPROMConfig(){
 //write header1
  EEPROM.write(0,00);
  EEPROM.write(35,00);
}

//-----------------------------------------------------------------


bool ReadEEPROMConfig(){
 uint8_t b,b2 =0;
 int w = 0;
 bool newc = true;
//headers read. if =35 then EEPROM used, else EEPROM clean and use constants
 b=EEPROM.read(0);
 b2=EEPROM.read(35);
  if (b!=35 && b2!=35 ) {
  //EEPROM is clear(new device) and setup variables to default values  
   PPMLowerLevel = DEF_LOWER_PPM_LEVEL;
   PPMUpperLevel = DEF_UPPER_PPM_LEVEL; 
   PPMSwitchLevel = DEF_KILLSWITCH_LEVEL;
   UseNoRXEngineSTOP = false;
   MainBattType = BATT_LIFE;
   newc=true;
    }
  else {
  //read config from EEPROM!
   PPMLowerLevel = EEPROMReadInt(1);
   PPMUpperLevel = EEPROMReadInt(3);
   PPMSwitchLevel = EEPROMReadInt(5);
   b=EEPROM.read(7);
   if (b==3) {UseNoRXEngineSTOP=true;} else {UseNoRXEngineSTOP=false;}
   MainBattType=EEPROM.read(8);
   newc=false;
  }

   SetupBatteryLevels(); 

 return newc;
}


//-----------------------------------------------------------------

void RenderCurrentConfig(){
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
         Serial.print(" | PPM levels Up/Lo/Sw: ");Serial.print(PPMUpperLevel);Serial.print("/");Serial.print(PPMLowerLevel);Serial.print("/");Serial.print(PPMSwitchLevel);Serial.print(" | Use NoRX Eng STOP: ");Serial.println(UseNoRXEngineSTOP);
}

//-----------------------------------------------------------------


void SetupBatteryLevels(){
      switch (MainBattType) {
      case BATT_LIFE:
        MainBattLowVoltageLevel1 = BATT_LIFE_LEVEL1;
        MainBattLowVoltageLevel2 = BATT_LIFE_LEVEL2;
      break;
      case BATT_LIPO:
        MainBattLowVoltageLevel1 = BATT_LIPO_LEVEL1;
        MainBattLowVoltageLevel2 = BATT_LIPO_LEVEL2;
      break;
    }
}

//-----------------------------------------------------------------

void WriteEEPROMConfig(){
 //write header1
  EEPROM.write(0,35);
  EEPROMWriteInt(1,PPMLowerLevel);
  EEPROMWriteInt(3,PPMUpperLevel);
  EEPROMWriteInt(5,PPMSwitchLevel);
  if (UseNoRXEngineSTOP) {EEPROM.write(7, 3);} else {EEPROM.write(7, 5);}
  EEPROM.write(8,MainBattType);
  //write header2
  EEPROM.write(35,35);
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

void PlayMenuSound(const byte stype){
  //Play enter to menu sound
  
 switch (stype) {
   case 1:   //Main Menu Intro
     tone(BEEPER_PIN,300,150);
     delay(150);
     tone(BEEPER_PIN,800,150);
     delay(150);
     tone(BEEPER_PIN,1300,150);
     delay(150);
     tone(BEEPER_PIN,1800,150);
     delay(150);
     tone(BEEPER_PIN,2300,150);
     delay(150);    
     break;    
   case 2:  //Main Menu Outro
     tone(BEEPER_PIN,2300,150);
     delay(150);
     tone(BEEPER_PIN,1800,150);
     delay(150);
     tone(BEEPER_PIN,1300,150);
     delay(150);
     tone(BEEPER_PIN,800,150);
     delay(150);
     tone(BEEPER_PIN,300,150);
     delay(150);    
    break;
   case 3:  //Calibrating SubMenu Intro
     tone(BEEPER_PIN,2300,200);
     delay(200);
     tone(BEEPER_PIN,200,200);
     delay(200);
     tone(BEEPER_PIN,2300,200);
     delay(200);
    break;
   case 4:  //Battery Chemistry SubMenu Intro
     tone(BEEPER_PIN,2300,500);
     delay(500);
     tone(BEEPER_PIN,200,150);
     delay(150);
    break;    
   case 5:  //NoRX Engine Off SubMenu Intro
     tone(BEEPER_PIN,200,500);
     delay(500);
     tone(BEEPER_PIN,2300,150);
     delay(150);
    break;    
   case 6:  //Setup to DEFAULTS SubMenu Intro
     tone(BEEPER_PIN,1000,500);
     delay(500);
     tone(BEEPER_PIN,1000,150);
     delay(150);
    break;    
 }
  
  

}

//-----------------------------------------------------------------

void PlayItemSound(const byte count){
 byte i;
  for (int i=0; i <=count-1; i++){
   tone(BEEPER_PIN,2300,100);
   delay(300);
  }  
}


//-----------------------------------------------------------------
//-----------------------------------------------------------------
// ================================================================
// ===                   HARDWARE HANDLERS                      ===
// ================================================================

void KillSWHandler(){

//reading PPM signal from channel
     if (digitalRead(KILLSWITCH_INPIN) == HIGH) 
        {KillSWTime1 = micros(); } 
     else {KillSWTime2 = micros();
           //counter overflow correction (~70min)
           if (KillSWTime2<KillSWTime1) {KillSWTime2=KillSWTime1+100;}
           //calc PPM signal length and packets count
           KillSWTime = KillSWTime2 - KillSWTime1;
           KillSWCounter++;
            if (KillSWCounter>MaxKillSWCounter) { MaxKillSWCounter = KillSWCounter;} 
           }

//switch handling
 if (KillSWTime >= PPMSwitchLevel)
    {//Switch in Upper position
     SwitchPOS=false;
     SwitchPOSUp=true;}
 else 
   { 
     //Switch in Lower position and if last position is Upper then proccessing for only one time change
     if (SwitchPOSUp)
     {SwitchPOS=true; 
      //increment switch press counter for options
      KillSWPressCounter++;
      KillSWPressCounterChange=true;
      //reset position change flag
      SwitchPOSUp=false;
      }
   }
//reset packets counter
  if (KillSWCounter >= 50) { KillSWCounter=1;}  

}

//-----------------------------------------------------------------
// ================================================================
// ===                   SOFTWARE HANDLERS                      ===
// ================================================================

void MainStateHandler(){
if (digitalRead(SYSTEM_SERVICE_PIN)==HIGH) {SystemServiceMode=true;} else {SystemServiceMode=false;}

 //Engine STOP handling
 //in Normal operating mode Engine stop signal equal Switch position
 if (SystemMode==SYSTEM_NORMAL){EngineSTOP=SwitchPOS;
 } 
 if (!EngineSTOP) {
  //prevent high speed duplicate switching with potential glitches. 
  if (EngineSTOPflag) {digitalWrite(KILLSWITCH_OUTPIN, HIGH); 
  tone(BEEPER_PIN,2300,30);
  EngineSTOPflag=false;
  LastEngineSTOP=EngineSTOP;
  }
 } 
 else 
 {digitalWrite(KILLSWITCH_OUTPIN, LOW);
  if (EngineSTOP!=LastEngineSTOP) {tone(BEEPER_PIN,200,30);} 
  LastEngineSTOP=EngineSTOP;
  EngineSTOPflag=true;}

 //LowBatt handling
 if (StatusLowBatt) {
     EngineSTOP=true;}

 //NoRX handling
  if (MaxKillSWCounter <=1) {
       StatusNoRX=true;
       //EngineSTOP = LastEngineSTOP; 
      } else {StatusNoRX=false;}

 
 //if Switch pressed(from Up to Down) then handling options  
   if (millis()-KillSWPressLaststime<=BEEPER_ACT_PERIOD && KillSWPressCounter>=1) 
   { 
    switch (KillSWPressCounter) {
    case 3: 
     if (millis() - MenuInitialTime <= MENU_INITIAL_PERIOD){
     //Switched 3 times within Initial period - GO TO Menu Mode!
       MenuMode=true;
       SystemMode=SYSTEM_MENU_M; 
     } else {
     BeeperActive=!BeeperActive;
     LED13Active=BeeperActive;
     KillSWPressCounter=0;
     KillSWPressCounterChange=false;
     KillSWPressLaststime=millis();}
     
    break; 
    case 1: 
     //start time for BEEPER_ACT_PERIOD period
     if (KillSWPressCounterChange) {KillSWPressLaststime=millis(); KillSWPressCounterChange=false;}     
     break;    
    } 
  }

 //reset Switch press counter if passed BEEPER_ACT_PERIOD time period   
 if (millis()-KillSWPressLaststime>BEEPER_ACT_PERIOD) {KillSWPressCounter=0; KillSWPressLaststime=millis(); } 
    
//Flashings LEDs
//Beeping default fashing 
 if (BeeperActive){
  if (BeeperOn && !BeeperOnflag) {
    //tone(BEEPER_PIN,2300,400);  //decomment if use tone with frequency
    analogWrite(BEEPER_PIN,254);
    BeeperOnflag=true;
    } else {
    if (!BeeperOnflag) {
     //noTone(BEEPER_PIN);        //decomment if use tone function
     analogWrite(BEEPER_PIN,0);
    }
    BeeperOnflag=false;
    }}

//LED13 default fashing 
 if (LED13Active){
  if (LED13On && !LED13Onflag) {
    digitalWrite(LED_PIN,HIGH);
    LED13Onflag=true;
    } else {
    digitalWrite(LED_PIN,LOW);
    LED13Onflag=false;
    }} else {digitalWrite(LED_PIN,LOW);LED13Onflag=false;}

//NoRX flashing
  if (StatusNoRXMainLEDOn) { if (!StatusNoRXMainLEDOnflag) {analogWrite(MAIN_LED_B,255); StatusNoRXMainLEDOnflag=true;}} 
   else {analogWrite(MAIN_LED_B,LOW);StatusNoRXMainLEDOnflag=false;}

  if (StatusLowBattMainLEDOn) { 
    if (!StatusLowBattMainLEDOnflag) {analogWrite(MAIN_LED_G,255);StatusLowBattMainLEDOnflag=true;}} 
  else {analogWrite(MAIN_LED_G,LOW);StatusLowBattMainLEDOnflag=false;}

  if (EngineSTOPMainLEDOn) { 
    if (!EngineSTOPMainLEDOnflag) {analogWrite(MAIN_LED_R,255);EngineSTOPMainLEDOnflag=true;}}
  else {analogWrite(MAIN_LED_R,LOW);EngineSTOPMainLEDOnflag=false;}

}

//-----------------------------------------------------------------

void CheckServoTester(){
  if (SystemServiceMode) {
  readServoTester();
  if (ServoOutMS != ServoLastOutMS) {
    myservo.writeMicroseconds(ServoOutMS);
    ServoLastOutMS = ServoOutMS;
  }}
}

//-----------------------------------------------------------------

void MainTimerHandler(){
 readMainBattVoltage();
 //calc voltage sum for one more 250ms step
 MainBattVoltageSum+=MainBattVoltage;
 MainBattVoltageCount++;

 if ( MainBattVoltageAveragetime == 0 ){
   MainBattVoltageAverage=MainBattVoltageSum / MainBattVoltageCount;
   MainBattVoltageCount=0;
   MainBattVoltageSum=0.0;
   MainBattVoltageAveragetime=7;
 } else MainBattVoltageAveragetime--;
  
 
  //if average voltage between Level2 < X <= Level1, setup LowBatt state and "soft" ignition blocking
 if (MainBattVoltageAverage > MainBattLowVoltageLevel2 && MainBattVoltageAverage <= MainBattLowVoltageLevel1) 
    {StatusLowBatt = true;}
 else {StatusLowBatt=false;}
 
 if (MainBattVoltageAverage <= MainBattLowVoltageLevel2) 
    {EngineSTOP=true;
     StatusLowBatt = true;
     SystemMode = SYSTEM_IMPERIAL_MARCH;}


//render telemetry 
 if ( Rendertime == 0 ){
  
  switch (SystemMode) {
   case SYSTEM_NORMAL: 
    Serial.print("NORMAL ");
    break;
   case SYSTEM_FIRST_START: 
    Serial.print("FIRST "); 
    break;
   case SYSTEM_IMPERIAL_MARCH: 
    Serial.print("IMPERIAL "); 
    break;
   case SYSTEM_MENU_M: 
    Serial.print("MENU_M "); 
    break;
   case SYSTEM_MENU_1_CAL: 
    Serial.print("MENU_1 "); 
    break;   
   case SYSTEM_MENU_2_BTYPE: 
    Serial.print("MENU_2 "); 
    break;    
   case SYSTEM_MENU_3_RX: 
    Serial.print("MENU_3 "); 
    break;
   case SYSTEM_MENU_4_DEFAULTS: 
    Serial.print("MENU_4 "); 
    break;
  }  
  //KillSwitch states render
  Serial.print("KSW : ");Serial.print(!EngineSTOP);Serial.print(" / ");Serial.print(MaxKillSWCounter);Serial.print(" / ");Serial.print(KillSWPressCounter); 
  //Menu states render
  Serial.print(" M_Item : ");
  Serial.print(MenuItem); 
  //Service states redner
  Serial.print(" | Service/NoRX/LowBatt/Beeper : ");Serial.print(SystemServiceMode);Serial.print(" / ");Serial.print(StatusNoRX);Serial.print(" / ");Serial.print(StatusLowBatt);Serial.print(" / ");Serial.print(BeeperActive);
  //Battery status render
  Serial.print(" | VBatt/Avg/Type: ");Serial.print(MainBattVoltage);Serial.print(" / ");Serial.print(MainBattVoltageAverage);Serial.print(" / ");
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
  //RX input render
  Serial.print(" | KSW time Gen/Read/Sw: ");Serial.print(ServoOutMS);Serial.print("/");Serial.print(KillSWTime);Serial.print("/");Serial.println(PPMSwitchLevel);
  Rendertime=3;
  } else Rendertime--;


//LED flashing routines
//Beeper flashing
  if ( BeeperTime == 0 ){
    BeeperOn = !BeeperOn;
    BeeperTime=1;
  } else BeeperTime--;


//LED13 (develop)
  if ( LED13time == 0 ){
    LED13On = !LED13On;
  LED13time=3;
  } else LED13time--;

//NO RX Signal LED
  if (StatusNoRX) {
      StatusNoRXMainLEDOn = !StatusNoRXMainLEDOn;} else {StatusNoRXMainLEDOn=false;}
    
//EngineSTOP LED
  if ( EngineSTOPtime == 0 ){
   if (EngineSTOP) { EngineSTOPMainLEDOn = !EngineSTOPMainLEDOn; }
   else EngineSTOPMainLEDOn = false;
   EngineSTOPtime=3;
  } else EngineSTOPtime--;

//Low Batt voltage LED
  if ( StatusLowBatttime == 0 ){
   if (StatusLowBatt) { StatusLowBattMainLEDOn = !StatusLowBattMainLEDOn; } 
   else StatusLowBattMainLEDOn = true;
  StatusLowBatttime=1;
  } else StatusLowBatttime--;

//reset packets counter after one second
 if (KillSWCounterTime==0) {
  MaxKillSWCounter=0;
  KillSWCounterTime=3;
 } else KillSWCounterTime--;
}

//-----------------------------------------------------------------

bool MenuInputHandler(){
 bool s;
  s=false;
   if (millis() - MenuItemTime <= MENU_CHOICE_PERIOD && KillSWPressCounter >=1 && KillSWPressCounterChange) 
   { //if Selected(switched down)
    //MenuItemTime=millis();
    KillSWPressCounterChange=false;
    KillSWPressCounter=0;
    tone(BEEPER_PIN,2300,30);
    s=true;
   } 
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
    KillSWPressCounter=0;
    KillSWPressCounterChange=false;
  //  MenuItemTime=millis();
  }
 return s;
}


//-----------------------------------------------------------------


void MenuMHandler (){
if (MenuItem==0) {PlayMenuSound(SOUND_MM_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);} 
else{
    MenuItemSelected=MenuInputHandler();
    //Play Menu Item selection sound
    if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
 switch (MenuItem) {
   case 1:   //Go To Calibrating RX signal 
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_1_CAL;MenuItem=0;}
    break;    
   case 2:   //Go To Battery chemistry type select 
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_2_BTYPE;MenuItem=0;}
    break;
   case 3:   //Go To NoRX engine off select
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_3_RX;MenuItem=0;}
    break;
   case 4:   //Go To setup to DEFAULTS procedure
    if (MenuItemSelected) {SystemMode=SYSTEM_MENU_4_DEFAULTS;MenuItem=0;}
    break;
   case 5:  //Exit to Normal Mode
    if (MenuItemSelected) {
      PlayMenuSound(SOUND_MM_OUTRO);
      MenuItem=0;
       Serial.println("Save configuration to EEPROM");
      RenderCurrentConfig();
      WriteEEPROMConfig();
      SystemMode=SYSTEM_NORMAL;}
    break;    
 }
  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==5) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false;}
    }
}

//-----------------------------------------------------------------

void Menu1Handler (){
//Calibrating Procedure PreMenu
//1 beep - Start Calibration procedure
//2 beeps - Exit to Main Menu

if (MenuItem==0) {PlayMenuSound(SOUND_M1_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis(); } 
   MenuItemSelected=MenuInputHandler();

  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==2) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; }
    
    //Play Menu Item selection sound
    if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
 switch (MenuItem) {
   case 1:   //Start RX Calibration procedure
    if (MenuItemSelected) {
        CalibratingStageCounter=1; 
        SystemMode=SYSTEM_CALIBRATING;}
    break;    
   case 2:  //Exit to Main Menu
    if (MenuItemSelected) {MenuItem=0;SystemMode=SYSTEM_MENU_M;}
    break;    
 }
 
}

//-----------------------------------------------------------------

void Menu2Handler (){
//Select battery chemistry SubMenu
//1 beep - LiFe battery
//2 beep - LiPolymer battery
//3 beeps - Exit to Main Menu
if (MenuItem==0) {PlayMenuSound(SOUND_M2_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==3) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; } 
    
    //Play Menu Item selection sound
    if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();MenuItemTime=millis();}
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

void Menu3Handler (){
//Select battery chemistry SubMenu
//1 beep - ON
//2 beep - OFF
//3 beeps - Exit to Main Menu


if (MenuItem==0) {PlayMenuSound(SOUND_M2_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 

  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==3) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false;} 
 
   
   MenuItemSelected=MenuInputHandler();
    //Play Menu Item selection sound
    if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();MenuItemTime=millis();}
 switch (MenuItem) {
   case 1:   //Setup to ON
    if (MenuItemSelected) {
         UseNoRXEngineSTOP=true;
         MenuItem=0;
         MenuIntroPlayed=false;
      }
    break;    
   case 2:  //Setup to OFF
    if (MenuItemSelected) {
         UseNoRXEngineSTOP=false;
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

void Menu4Handler (){
//Reset Configuration to DEFAULTS
//1 beep - Reset to defaults and exit to main menu
//2 beeps - Exit to Main Menu without changing

if (MenuItem==0) {PlayMenuSound(SOUND_M4_INTRO); MenuIntroPlayed=false;MenuItem=1; delay(2000);MenuItemTime=millis();} 
   MenuItemSelected=MenuInputHandler();

  //Rotate Menu items choice
  if (millis() - MenuItemTime > MENU_CHOICE_PERIOD) {
  if (MenuItem==2) {MenuItem=1;} else {MenuItem++;} 
  MenuIntroPlayed=false; }
    
    //Play Menu Item selection sound
    if (!MenuIntroPlayed) {PlayItemSound(MenuItem); MenuIntroPlayed=true;MenuItemTime=millis();}
 switch (MenuItem) {
   case 1:   //Reset Configuration to defaults and Go To Main Menu
    if (MenuItemSelected) {
       PPMLowerLevel = DEF_LOWER_PPM_LEVEL;
       PPMUpperLevel = DEF_UPPER_PPM_LEVEL; 
       PPMSwitchLevel = DEF_KILLSWITCH_LEVEL;
       UseNoRXEngineSTOP = false;
       MainBattType = BATT_LIFE;
       Serial.println("Reset configuration to DEFAULTS");
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
        analogWrite(BEEPER_PIN, i*24);
        delay(24);
     }
     analogWrite(BEEPER_PIN, 0);
     delay(3000);
     delayMicroseconds(1000); 
   break;    
   case 2:   //Stage2 - reading UpperPosition data
     CalibrationUpperLevel=KillSWTime;
     delayMicroseconds(1000); 
     CalibrationUpperLevel=KillSWTime;
     delayMicroseconds(1000); 
     CalibrationUpperLevel=KillSWTime;
     delayMicroseconds(1000); 
     CalibrationUpperLevel=KillSWTime;
     delayMicroseconds(1000); 
     CalibrationUpperLevel=KillSWTime;
     delayMicroseconds(1000); 
   break;    
   case 3:   //Stage3 - play LowerPosition sound
     Serial.println("Move switch to Lower position"); 
     for (i=0; i <= 124; i++){
        analogWrite(BEEPER_PIN, i*24);
        delay(24);
     }
     analogWrite(BEEPER_PIN, 0);
     delay(3000);
     delayMicroseconds(1000); 
   break;    
   case 4:   //Stage4 - reading LowerPosition data
    CalibrationLowerLevel=KillSWTime;
     delayMicroseconds(1000); 
    CalibrationLowerLevel=KillSWTime;
     delayMicroseconds(1000); 
    CalibrationLowerLevel=KillSWTime;
     delayMicroseconds(1000); 
    CalibrationLowerLevel=KillSWTime;
     delayMicroseconds(1000); 
    CalibrationLowerLevel=KillSWTime;
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
     case SYSTEM_IMPERIAL_MARCH:
          MainStateHandler();
          ImperialMarch();
          break;
     case SYSTEM_MENU_M: 
      MenuMHandler();
     break;    
     case SYSTEM_MENU_1_CAL: 
      Menu1Handler();
     break;    
     case SYSTEM_MENU_2_BTYPE: 
      Menu2Handler();
     break;    
     case SYSTEM_MENU_3_RX: 
      Menu3Handler();
     break;    
     case SYSTEM_MENU_4_DEFAULTS:
      Menu4Handler();
     break;
     case SYSTEM_CALIBRATING: 
      if (CalibratingStageCounter==2 || CalibratingStageCounter==4 ) 
        {
          for (CalibratingCounter=0; CalibratingCounter <= 255; CalibratingCounter++){
           CalibratingInput(CalibratingStageCounter); 
          }
        } else {CalibratingInput(CalibratingStageCounter);}
      CalibratingStageCounter++;
     break;    
    }
}
