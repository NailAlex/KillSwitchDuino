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

//System States
#define SYSTEM_NORMAL 0
#define SYSTEM_FIRST_START 1
#define SYSTEM_IMPERIAL_MARCH 6
#define SYSTEM_MENU_M 21 
#define SYSTEM_MENU_1_CAL 22
#define SYSTEM_MENU_2_BTYPE 23
#define SYSTEM_MENU_3_RX 24
#define SYSTEM_SERVICE_PIN 12


//BEEPER
#define BEEPER_PIN 11
#define BEEPER_ACT_PERIOD 2000  //Time limits within activate Beeper if >=3 time EngineStop pushed
#define BEEPER_FREQ 2300        //For feature use

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

//SoftTimers vars
Timer m_timer;
  
//Voltage vars
uint8_t  MainBattType = BATT_LIFE;
float MainBattVoltage = 0.0;
float MainBattR1 = 14966.0;                    //Volt devider upper leg resistor (+Vin -> read point) 
float MainBattR2 = 9856.0;                     //Volt devider lower leg resistor (read point -> GND) 
float V_REF = 5.05;                            //accurate value of the voltage from system linear regulator
float MAIN_BATT_VADD = 0.03;                   //Volt correction factor
float MainBattVoltageSum = 0;                  //Volt sum for calc average
uint8_t MainBattVoltageCount = 0;                  //Volt reading count for calc average
float MainBattVoltageAverage = 0;              //Average voltage within reading period
uint8_t MainBattVoltageAveragetime = 7;        //1 time per 2 seconds period
float MainBattLowVoltageLevel1 = 0;            //Current Level1 for blocking ignition and Blue MainLED blinking
float MainBattLowVoltageLevel2 = 0;            //Current Level2 for all blocking ignition and activate Imperial March
  
//Servo and PPM vars
unsigned int ServoOutMS = 0;                   //Intergated ServoTester output PPM signal value to KillSwitch pin
unsigned int ServoInValue = 0;                 //Intergated ServoTester input PPM signal value from potentiometer
Servo myservo;        
unsigned int ServoLastOutMS = 0;               //stopflag for one time switching
unsigned int PPMLowerLevel = DEF_LOWER_PPM_LEVEL;
unsigned int PPMUpperLevel = DEF_UPPER_PPM_LEVEL;
unsigned int PPMSwitchLevel = DEF_KILLSWITCH_LEVEL;

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
 Serial.println("Starting readings");
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

  // ResetEEPROMConfig();
   if (!ReadEEPROMConfig()) {WriteEEPROMConfig();}

    if (KillSWTime >= PPMSwitchLevel)
    {
     //Switch in Upper position
     SwitchPOS=false;
     SwitchPOSUp=true;} else 
     {
     //Switch in Upper position
     SwitchPOS=true;
     SwitchPOSUp=false;}

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
 ServoOutMS = map(ServoInValue,0,1023,1000,2000);
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
   if (b==3) {UseNoRXEngineSTOP=true;} else if (b==5) {UseNoRXEngineSTOP=false;}
   MainBattType=EEPROM.read(8);
   newc=false;
  }

    
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
 
 
 return newc;
}


//-----------------------------------------------------------------

void WriteEEPROMConfig(){
 //write header1
  EEPROM.write(0,35);
  EEPROMWriteInt(1,PPMLowerLevel);
  EEPROMWriteInt(1,PPMUpperLevel);
  EEPROMWriteInt(1,PPMSwitchLevel);
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
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
// ================================================================
// ===                   HARDWARE HANDLERS                      ===
// ================================================================

void KillSWHandler(){
//reset packets counter
  if (KillSWCounter >= 50) { KillSWCounter=1;}  

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
     BeeperActive=!BeeperActive;
     LED13Active=BeeperActive;
     KillSWPressCounter=0;
     KillSWPressCounterChange=false;
     KillSWPressLaststime=millis();
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
}  
  
  Serial.print("KSW : ");Serial.print(!EngineSTOP);Serial.print(" / ");Serial.print(MaxKillSWCounter);Serial.print(" / ");Serial.print(KillSWPressCounter);  
  Serial.print(" | Service/NoRX/LowBatt/Beeper : ");Serial.print(SystemServiceMode);Serial.print(" / ");Serial.print(StatusNoRX);Serial.print(" / ");Serial.print(StatusLowBatt);Serial.print(" / ");Serial.print(BeeperActive);
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
  
  Serial.print(" | KSW time: ");Serial.print(ServoOutMS);Serial.print("/");Serial.println(KillSWTime);
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

void MenuMHandler (){



  
}

//-----------------------------------------------------------------

void Menu1Handler (){



  
}

//-----------------------------------------------------------------

void Menu2Handler (){



  
}

//-----------------------------------------------------------------

void Menu3Handler (){



  
}

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
      Menu2Handler();
     break;    
    }
}
