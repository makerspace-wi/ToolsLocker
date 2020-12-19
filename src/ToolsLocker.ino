/* DESCRIPTION
  ====================
  started on 09Sep2020 - uploaded on 06.09.2020 by Michael
  Code for Toolslocker control over RFID
  reading IDENT from xBee, retrait sending ...POR until time responds
  switch on power (12V) if someone chip is registered for Toolslocker
  open a door if switch is pushed

  Commands to Raspi --->
  'TOLO'  - from xBee (=Ident)

  'Ident;POR;Vx.x.x' - machine power on reset and sw version (Ident;por;Vx.x.x)
  'card;nn...'       - uid_2 from reader
  'Ident;ont'        - Ident is startet and time is on
  'Ident;WO;drxx'    - Ident; want open door with number xx? (colum|row) ---> "dr34"
  'Ident;OP;drxx'    - Ident; door opened with number xx (colum|row) ---> "dr34"
  'Ident;CL;drxx'    - Ident; door closed with number xx (colum|row) ---> "dr34"
  'Ident;ER;drxx'    - Ident; door has not opened with number xx (colum|row) ---> "dr34"
  'Ident;off'        - Ident reporting nobody logged in

  Commands from Raspi (Commands send from raspi, are allways inspected and if known, executed!!!)
  'time'      - format time33.33.3333 33:33:33
  'noreg'     - RFID-Chip not registed
  'ontxx'     - on to open doors for time xx min "ont15"
  'odiyy'     - open door immediately with number yy (colum|row) ---> "odi34"
  'tllo'      - from toollocker log all off
  'r3t...'    - display text in row 3 "r3tabcde12345", max 20
  'r4t...'    - display text in row 4 "r4tabcde12345", max 20

  last change: 11.12.2020 by Michael Muehl
  changed: beta version for 12 locks, tested one lock and button!!!!!!!!!!!!!
*/
#define Version "0.9.6" // (Test =0.9.x ==> 0.9.7)

#include <Arduino.h>
#include <TaskScheduler.h>
#include <Wire.h>
#include <LCDLED_BreakOUT.h>
#include <utility/Adafruit_MCP23017.h>
#include <SPI.h>
#include <MFRC522.h>

// PIN Assignments
// RFID Control -------
#define RST_PIN      4  // RFID Reset
#define SS_PIN      10  // RFID Select

// Machine Control (ext)
#define currMotor   A0  // Motor current (not used)
#define SSR_POWER   A2  // SSR POWER on / off  (Schlösser)
#define SSR_BOOZER  A3  // SSR Boozer on / off  (Zentral Boozer) [not used]

#define BUSError     8  // Bus error

// I2C IOPort definition
byte I2CFound = 0;
int I2CAdress[] = {0, 0, 0, 0, 0};
byte I2CTransmissionResult = 0;
#define I2CPort   0x20  // I2C Adress MCP23017 LCD Display
// check chanel is active
#define I2CDoors1 1     // I2C Adress MCP23017 Door row 1 (0x20 + 1)
#define I2CDoors2 2     // I2C Adress MCP23017 Door row 2 (0x20 + 2)
#define I2CDoors3 4     // I2C Adress MCP23017 Door row 3 (0x20 + 4)

// Pin Assignments Display (I2C LCD Port A/LED +Button Port B)
// Switched to LOW
#define FlashLED_A   0  // Flash LEDs oben
#define FlashLED_B   1  // Flash LEDs unten
#define buzzerPin    2  // Buzzer Pin
#define BUT_P1_LED   3  // not used
// Switched High - Low - High - Low
#define StopLEDrt    4  // StopLEDrt (LED + Stop-Taster)
#define StopLEDgn    5  // StopLEDgn (LED - Stop-Taster)
// switch to HIGH Value (def .h)
// BUTTON_P1  2         // not used
// BUTTON_P2  1         // StopSwitch
// BACKLIGHT for LCD-Display
#define BACKLIGHToff 0x0
#define BACKLIGHTon  0x1

// Pin Assignments Door Control (I2C (21, 22, 24) Port A: 2 door  Port B 2 doors)
// Array lock & position, button & Led
int schloss[] = {4, 0, 4, 8, 12}; // [Numbers, Port 1, Port 2, Port 3, Port 4]
int tastLed[] = {4, 1, 5, 9, 13};
int posLock[] = {4, 2, 6, 10, 14};
int butTast[] = {4, 3, 7, 11, 15};

byte sc = 0; // variable for lock colum
byte sr = 0; // variable for lock row

// DEFINES
#define porTime         5 // wait seconds for sending Ident + POR
#define CLOSE2END      15 // MINUTES before activation is off
#define CLEANON         4 // TASK_SECOND vac on for a time
#define repMES          1 // repeat commands
#define periRead      100 // read 100ms analog input for 50Hz (Strom)
#define currHyst       10 // [10] hystereses for current detection normal
#define currMean        3 // [ 3] current average over ...
#define intervalINC	 3600 // 3600 * 4
#define intervalPush    5 // seconds to push button before clean starts
#define intervalCLMn   30 // min clean time in seconds
#define intervalCLMx   10 * 60 // max clean time in seconds

#define stepsTB  2 // steps for door tools locker
#define debouTBo 2 // debounce button door tool locker open
#define debouTBc 5 // debounce door tool locker close

// CREATE OBJECTS
Scheduler runner;
LCDLED_BreakOUT lcd = LCDLED_BreakOUT();
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
Adafruit_MCP23017 tld1;
Adafruit_MCP23017 tld2;
Adafruit_MCP23017 tld3;

// Callback methods prototypes
void checkXbee();        // Task connect to xBee Server
void BlinkCallback();    // Task to let LED blink - added by D. Haude 08.03.2017
void UnLoCallback();     // Task to Unlock machine
void repeatMES();        // Task to repeat messages

void BuzzerOn();         // added by DieterH on 22.10.2017
void FlashCallback();    // Task to let LED blink - added by D. Haude 08.03.2017
void DispOFF();          // Task to switch display off after time

// ToolsLocker Check / Open
void ToolLockDoors();
void ToolButCheck();
void ToolOpenDoor();

// Functions define for C++
void OnTimed(long);
void flash_led(int);

// TASKS
Task tM(TASK_SECOND / 2, TASK_FOREVER, &checkXbee);	    // 500ms main task
Task tR(TASK_SECOND / 2, 0, &repeatMES);                // 500ms * repMES repeat messages
Task tU(TASK_SECOND / 4, TASK_FOREVER, &UnLoCallback);  // 250ms unlock ID (time = 4 * 60)
Task tB(TASK_SECOND * 5, TASK_FOREVER, &BlinkCallback); // 5000ms blinking for several events

Task tBU(TASK_SECOND / 10, 6, &BuzzerOn);               // 100ms 6x =600ms buzzer on for 6 times
Task tBD(1, TASK_ONCE, &FlashCallback);                 // Flash Delay leds
Task tDF(1, TASK_ONCE, &DispOFF);                       // display off after xx sec

Task tTBC(TASK_SECOND / 10, TASK_FOREVER, &ToolButCheck); // 100ms Check if a button is pressed for open or close one door
Task tTOD(1, TASK_ONCE, &ToolOpenDoor);                   // Open a door

// VARIABLES
unsigned long val;
int nr2Open[] = {0, 0, 0};
unsigned int timer = 0;
bool onTime = false;
int minutes = 0;
bool toggle = false;
unsigned long code;
byte atqa[2];
byte atqaLen = sizeof(atqa);
byte intervalRFID = 0;      // 0 = off; from 1 sec to 6 sec after Displayoff
bool displayIsON = false;   // if display is switched on = true
bool nextrun = false;       // programm runs more then 1 time

// tool locker doors:
unsigned int CLOSE = CLOSE2END; // RAM cell for before activation is off
bool checkDoors = false;     // not all doors are closed
byte countDoors = 0;         // counter for doors until the are closed
byte countTBo = 0;           // counter for tools locker (debounce open)
byte countTBc = 0;           // counter for tools locker (debounce closed)
bool dooropend = false;      // one door is opened

// Serial with xBee
String inStr = "";      // a string to hold incoming data
String IDENT = "";      // Machine identifier for remote access control
String SFMes = "";      // String send for repeatMES
byte plplpl = 0;        // send +++ control AT sequenz
byte getTime = porTime;

// ======>  SET UP AREA <=====
void setup()
{
  //init Serial port
  Serial.begin(57600);  // Serial
  inStr.reserve(40);    // reserve for instr serial input
  IDENT.reserve(5);     // reserve for IDENT serial output

  // initialize:
  Wire.begin();         // I2C

  // I2C _ Ports definition only for test if I2C is avilable
  for (sr = 0; sr < 5; sr++)
  {
    Wire.beginTransmission(I2CPort + sr);
    I2CTransmissionResult = Wire.endTransmission();
    if (I2CTransmissionResult == 0)
    {
      I2CFound++;
      I2CAdress[sr] = sr;
    }
  }

  // I2C Bus mit slave vorhanden
  if (I2CFound >= 1)
  {
    lcd.begin(20, 4);        // initialize the LCD
    if (I2CAdress[2] == I2CDoors1)
      tld1.begin(I2CDoors1); // used address (2)1
    if (I2CAdress[3] == I2CDoors2)
      tld2.begin(I2CDoors2); // used address (2)2
    if (I2CAdress[4] == I2CDoors3)
      tld3.begin(I2CDoors3); // used address (2)4

    SPI.begin();             // SPI

    mfrc522.PCD_Init();      // Init MFRC522
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

    // IO MODES
    pinMode(BUSError, OUTPUT);
    pinMode(SSR_POWER, OUTPUT);
    pinMode(SSR_BOOZER, OUTPUT);

    // Set default values
    digitalWrite(BUSError, HIGH); // turn the LED ON (init start)
    digitalWrite(SSR_POWER, LOW);
    digitalWrite(SSR_BOOZER, LOW);

    if (I2CAdress[4] == I2CDoors3)
    {
      for (sr = 1; sr < 5;sr++)
      {
        tld3.pinMode(schloss[sr], OUTPUT);
        tld3.pinMode(tastLed[sr], OUTPUT);
        tld3.pinMode(posLock[sr], INPUT);
        tld3.pinMode(butTast[sr], INPUT); // (int A later?)


        tld3.digitalWrite(schloss[sr], LOW);
        tld3.digitalWrite(tastLed[sr], LOW);
      }
    }

    runner.init();
    runner.addTask(tM);
    runner.addTask(tB);
    runner.addTask(tR);
    runner.addTask(tU);
    runner.addTask(tBU);
    runner.addTask(tBD);
    runner.addTask(tDF);

    runner.addTask(tTBC);
    runner.addTask(tTOD);
 
    lcd.clear();
    lcd.pinLEDs(buzzerPin, LOW);
    lcd.pinLEDs(BUT_P1_LED, LOW);
    but_led(1);
    flash_led(1);
    dispRFID();
    tM.enable();         // xBee check
    Serial.print("+++"); //Starting the request of IDENT
  }
  else 
  {
    tB.enable();  // enable Task Error blinking
    tB.setInterval(TASK_SECOND);
  }
}
// Setup End -----------------------------------

// TASK (Functions) ----------------------------
void checkXbee()
{
  if (IDENT.startsWith("TOLO") && plplpl == 2)
  {
    ++plplpl;
    tB.setCallback(retryPOR);
    tB.enable();
    digitalWrite(BUSError, LOW); // turn the LED off (Programm start)
  }
}

void retryPOR() {
  tDF.restartDelayed(TASK_SECOND * 30); // restart display light
  if (getTime < porTime * 5) {
    Serial.println(String(IDENT) + ";POR;V" + String(Version));
    ++getTime;
    tB.setInterval(TASK_SECOND * getTime);
    lcd.setCursor(0, 0); lcd.print(String(IDENT) + " ");
    lcd.setCursor(16, 1); lcd.print((getTime - porTime) * porTime);
  }
  else if (getTime == 255) {
	  tR.setIterations(repMES);
    tM.setCallback(checkRFID);
    tM.enable();
    tB.disable();
    displayON();
  }
}

void checkRFID()
{   // 500ms Tick
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
  {
    code = 0;
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      code = ((code + mfrc522.uid.uidByte[i]) * 10);
    }
    if (!digitalRead(SSR_POWER))
    { // Check if machine is switched on
      flash_led(4);
      tBD.setCallback(&FlashCallback);
      tBD.restartDelayed(100);
      tDF.restartDelayed(TASK_SECOND * 30);
    }
    Serial.println("card;" + String(code));
    // Display changes
    lcd.setCursor(5, 0); lcd.print("               ");
    lcd.setCursor(0, 0); lcd.print("Card# "); lcd.print(code);
    displayON();
  }

}

void UnLoCallback() {   // 500ms Tick
  uint8_t buttons = lcd.readButtons();
  if (timer > 0)
  {
    but_led(3);
    toggle = !toggle;
    if (toggle)
    { // toggle GREEN Button LED
      flash_led(2);
    }
    else
    {
      flash_led(3);
    }
    timer -= 1;
    minutes = timer / 240;
    if (timer % 240 == 0)
    {
      char tbs[8];
      sprintf(tbs, "% 4d", minutes);
      lcd.setCursor(16, 3); lcd.print(tbs);
    }
  }
  if (!dooropend && ((timer == 0 && onTime) || buttons & BUTTON_P2))
  {   //  time == 0 and timed or Button
      onTime = false;
      doorsclosed();
  }
}

// Task repeatMES: ------------------------
void repeatMES() {
  // --repeat messages from machines
  Serial.println(String(SFMes));
}

void BlinkCallback() {
  // --Blink if BUS Error
  digitalWrite(BUSError, !digitalRead(BUSError));
}

void FlashCallback() {
  flash_led(1);
}

void DispOFF() {
  displayIsON = false;
  lcd.setBacklight(BACKLIGHToff);
  lcd.clear();
  but_led(1);
  flash_led(1);
}

void ToolButCheck()
{
  digitalWrite(BUSError, HIGH);
  if (nr2Open[0] == 0)
  {
    for (sr = 1; sr < 5; sr++)
    {
      if (I2CAdress[2] == I2CDoors1)
      { // if channel 1 is a live
        sc = 1;
        if (!tld1.digitalRead(butTast[sr]) && !tld1.digitalRead(posLock[sr]) && countTBo == debouTBo)
        {
          nr2Open[0] = -5;
          nr2Open[1] = sc;
          nr2Open[2] = sr;
          Serial.println(String(IDENT) + ";WO" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
          lcd.setCursor(0, 2);
          lcd.print("Want Open Door " + String(nr2Open[1]) + String(nr2Open[2]) + " ");
          lcd.setCursor(0, 3);
          lcd.print("Wait for request   ");
        }
        if (((nr2Open[1] == 3 && nr2Open[2] == sr) || !tld1.digitalRead(butTast[sr])) && countTBc == 0 && countTBo < debouTBo)
        {
          ++countTBo;
        }
      }

      if (I2CAdress[3] == I2CDoors2)
      { // if channel 2 is a live
        sc = 2;
        if (!tld2.digitalRead(butTast[sr]) && !tld2.digitalRead(posLock[sr]) && countTBo == debouTBo)
        {
          nr2Open[0] = -5;
          nr2Open[1] = sc;
          nr2Open[2] = sr;
          Serial.println(String(IDENT) + ";WO" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
          lcd.setCursor(0, 2);
          lcd.print("Want Open Door " + String(nr2Open[1]) + String(nr2Open[2]) + " ");
          lcd.setCursor(0, 3);
          lcd.print("Wait for request   ");
        }
        if (((nr2Open[1] == 3 && nr2Open[2] == sr) || !tld2.digitalRead(butTast[sr])) && countTBc == 0 && countTBo < debouTBo)
        {
          ++countTBo;
        }
      }

      if (I2CAdress[4] == I2CDoors3)
      { // if channel 3 is a live
        sc = 3;
        if (!tld3.digitalRead(butTast[sr]) && !tld3.digitalRead(posLock[sr]) && countTBo == debouTBo)
        {
          nr2Open[0] = -5;
          nr2Open[1] = sc;
          nr2Open[2] = sr;
          Serial.println(String(IDENT) + ";WO" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
          lcd.setCursor(0, 2);
          lcd.print("Want Open Door " + String(nr2Open[1]) + String(nr2Open[2]) + " ");
          lcd.setCursor(0, 3);
          lcd.print("Wait for request   ");
        }
        if (((nr2Open[1] == 3 && nr2Open[2] == sr) || !tld3.digitalRead(butTast[sr])) && countTBc == 0 && countTBo < debouTBo)
        {
          ++countTBo;
        }
      }
    }
  }
  else if (nr2Open[0] >= -1)
  {
    if (nr2Open[1] == 1)
    {
      if (tld1.digitalRead(butTast[nr2Open[2]]) && !tld1.digitalRead(posLock[nr2Open[2]]) && countTBc == debouTBc)
      {
        countTBo = 0;
        countTBc = 0;
        tld1.digitalWrite(tastLed[sr], LOW);
        Serial.println(String(IDENT) + ";CL" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " closed      ");
        opendoors(CLOSE);
        dooropend = false;
        nr2Open[0] = 0;
        nr2Open[1] = 0;
        nr2Open[2] = 0;
      }
      if (tld1.digitalRead(butTast[nr2Open[2]]) && !tld1.digitalRead(posLock[nr2Open[2]]) && countTBo == debouTBo + stepsTB && countTBc < debouTBc)
      {
        ++countTBc;
      }
    }

    if (nr2Open[1] == 2)
    {
      if (tld2.digitalRead(butTast[nr2Open[2]]) && !tld2.digitalRead(posLock[nr2Open[2]]) && countTBc == debouTBc)
      {
        countTBo = 0;
        countTBc = 0;
        tld2.digitalWrite(tastLed[nr2Open[2]], LOW);
        Serial.println(String(IDENT) + ";CL" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " closed      ");
        opendoors(CLOSE);
        dooropend = false;
        nr2Open[0] = 0;
        nr2Open[1] = 0;
        nr2Open[2] = 0;
      }
      if (tld2.digitalRead(butTast[nr2Open[2]]) && !tld2.digitalRead(posLock[nr2Open[2]]) && countTBo == debouTBo + stepsTB && countTBc < debouTBc)
      {
        ++countTBc;
      }
    }

    if (nr2Open[1] == 3)
    {
      if (tld3.digitalRead(butTast[nr2Open[2]]) && !tld3.digitalRead(posLock[nr2Open[2]]) && countTBc == debouTBc)
      {
        countTBo = 0;
        countTBc = 0;
        tld3.digitalWrite(tastLed[nr2Open[2]], LOW);
        Serial.println(String(IDENT) + ";CL" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " closed      ");
        opendoors(CLOSE);
        dooropend = false;
        nr2Open[0] = 0;
        nr2Open[1] = 0;
        nr2Open[2] = 0;
      }
      if (tld3.digitalRead(butTast[nr2Open[2]]) && !tld3.digitalRead(posLock[nr2Open[2]]) && countTBo == debouTBo + stepsTB && countTBc < debouTBc)
      {
        ++countTBc;
      }
    }
  }
  digitalWrite(BUSError, LOW);
}

void ToolOpenDoor()
{
  if (nr2Open[0] == -1)
  {
    if (nr2Open[1] == 1)
    {
      if (tld1.digitalRead(posLock[nr2Open[2]]))
      {
        tTOD.disable();
        tld1.digitalWrite(schloss[nr2Open[2]], LOW);
        tld1.digitalWrite(tastLed[nr2Open[2]], HIGH);
        countTBo = debouTBo + stepsTB;
        Serial.println(String(IDENT) + ";OP" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " opened      ");
        lcd.setCursor(0, 3);
        lcd.print("Please close door   ");
        tTBC.enable();
      }
      else
      {
        tld1.digitalWrite(schloss[nr2Open[2]], LOW);
        Serial.println(String(IDENT) + ";ER" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " not opened  ");
        lcd.setCursor(0, 3);
        lcd.print("Please send message ");
        tTOD.restartDelayed(500);
        ++countTBo;
        if (countTBo % 2)
        {
          tld1.digitalWrite(tastLed[nr2Open[2]], HIGH);
        }
        else
        {
          tld1.digitalWrite(tastLed[nr2Open[2]], LOW);
        }
        if (countTBo >= 30)
        {
          tld1.digitalWrite(tastLed[nr2Open[2]], LOW);
          countTBo = debouTBo + stepsTB;
          tTOD.disable();
          tTBC.enable();
        }
      }
    }

    if (nr2Open[1] == 2)
    {
      if (tld2.digitalRead(posLock[nr2Open[2]]))
      {
        tTOD.disable();
        tld2.digitalWrite(schloss[nr2Open[2]], LOW);
        tld2.digitalWrite(tastLed[nr2Open[2]], HIGH);
        countTBo = debouTBo + stepsTB;
        Serial.println(String(IDENT) + ";OP" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " opened      ");
        lcd.setCursor(0, 3);
        lcd.print("Please close door   ");
        tTBC.enable();
      }
      else
      {
        tld2.digitalWrite(schloss[nr2Open[2]], LOW);
        Serial.println(String(IDENT) + ";ER" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " not opened  ");
        lcd.setCursor(0, 3);
        lcd.print("Please send message ");
        tTOD.restartDelayed(500);
        ++countTBo;
        if (countTBo % 2)
        {
          tld2.digitalWrite(tastLed[nr2Open[2]], HIGH);
        }
        else
        {
          tld2.digitalWrite(tastLed[nr2Open[2]], LOW);
        }
        if (countTBo >= 30)
        {
          tld2.digitalWrite(tastLed[nr2Open[2]], LOW);
          countTBo = debouTBo + stepsTB;
          tTOD.disable();
          tTBC.enable();
        }
      }
    }

    if (nr2Open[1] == 3)
    {
      if (tld3.digitalRead(posLock[nr2Open[2]]))
      {
        tTOD.disable();
        tld3.digitalWrite(schloss[nr2Open[2]], LOW);
        tld3.digitalWrite(tastLed[nr2Open[2]], HIGH);
        countTBo = debouTBo + stepsTB;
        Serial.println(String(IDENT) + ";OP" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " opened      ");
        lcd.setCursor(0, 3);
        lcd.print("Please close door   ");
        tTBC.enable();
      }
      else
      {
        tld3.digitalWrite(schloss[nr2Open[2]], LOW);
        Serial.println(String(IDENT) + ";ER" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        lcd.setCursor(0, 2);
        lcd.print("Door " + String(nr2Open[1]) + String(nr2Open[2]) + " not opened  ");
        lcd.setCursor(0, 3);
        lcd.print("Please send message ");
        tTOD.restartDelayed(500);
        ++countTBo;
        if (countTBo % 2)
        {
          tld3.digitalWrite(tastLed[nr2Open[2]], HIGH);
        }
        else
        {
          tld3.digitalWrite(tastLed[nr2Open[2]], LOW);
        }
        
        if (countTBo >= 30)
        {
          tld3.digitalWrite(tastLed[nr2Open[2]], LOW);
          countTBo = debouTBo + stepsTB;
          tTOD.disable();
          tTBC.enable();
        }
      }
    }
  }
}
// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
void noreg()
{
  digitalWrite(SSR_POWER, LOW);
  digitalWrite(SSR_BOOZER, LOW);
  lcd.setCursor(0, 2); lcd.print("Tag not registered");
  lcd.setCursor(0, 3); lcd.print("===> No access! <===");
  tM.enable();
  BadSound();
  but_led(1);
  flash_led(1);
}

void opendoors(long min)
{   // Turn on machine for minutes
  onTime = true;
  timer = min * 240;
  Serial.println(String(IDENT) + ";ont");
  char tbs[8];
  sprintf(tbs, "% 4d", timer / 240);
  lcd.setCursor(0, 3); lcd.print("Time left (min):"); lcd.print(tbs);
  granted();
}

// Tag registered
void granted()
{
  tM.disable();
  tDF.disable();
  but_led(3);
  flash_led(1);
  GoodSound();
  digitalWrite(SSR_POWER, HIGH);
  if (!nextrun)
  {
    lcd.setCursor(0, 2);
    lcd.print("Access granted      ");
  }
  else
  {
    lcd.setCursor(0, 2);
    lcd.print("Please open a door   ");
  }
  nextrun = true;
  tR.disable();
  tU.enable();
  tTBC.enable();
}

void dooropened(void)
{
  if (nr2Open[1] == 1)
  {
    if (!tld1.digitalRead(posLock[nr2Open[2]]))
    {
      dooropend = true;
      tTBC.disable();
      tld1.digitalWrite(schloss[nr2Open[2]], HIGH);
      timer = 0;
      onTime = false;
      but_led(3);
      flash_led(4);
      nr2Open[0] = -1;
      tTOD.enable();
      tTOD.restartDelayed(100);
    }
  }

  if (nr2Open[1] == 2)
  {
    if (!tld2.digitalRead(posLock[nr2Open[2]]))
    {
      dooropend = true;
      tTBC.disable();
      tld2.digitalWrite(schloss[nr2Open[2]], HIGH);
      timer = 0;
      onTime = false;
      but_led(3);
      flash_led(4);
      nr2Open[0] = -1;
      tTOD.enable();
      tTOD.restartDelayed(100);
    }
  }

  if (nr2Open[1] == 3)
  {
    if (!tld3.digitalRead(posLock[nr2Open[2]]))
    {
      dooropend = true;
      tTBC.disable();
      tld3.digitalWrite(schloss[nr2Open[2]], HIGH);
      timer = 0;
      onTime = false;
      but_led(3);
      flash_led(4);
      nr2Open[0] = -1;
      tTOD.enable();
      tTOD.restartDelayed(100);
    }
  }
}

// Switch off machine and stop
void doorsclosed(void)
{
  tTBC.disable();
  tU.disable();
  timer = 0;
  but_led(2);
  digitalWrite(SSR_POWER, LOW);
  digitalWrite(SSR_BOOZER, LOW);
  Serial.println(String(IDENT) + ";off");
  tDF.restartDelayed(TASK_SECOND * 30);
  BadSound();
  flash_led(1);
  tM.enable(); // added by DieterH on 18.10.2017
  countTBo = 0;
  countTBc = 0;
  // Display change
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System shut down at");

}

void but_led(int var)
{
  switch (var)
  {
    case 1:   // LEDs off
      lcd.pinLEDs(StopLEDrt, HIGH);
      lcd.pinLEDs(StopLEDgn, HIGH);
      break;
    case 2:   // RED LED on
      lcd.pinLEDs(StopLEDrt, LOW);
      lcd.pinLEDs(StopLEDgn, HIGH);
      break;
    case 3:   // GREEN LED on
      lcd.pinLEDs(StopLEDrt, HIGH);
      lcd.pinLEDs(StopLEDgn, LOW);
      break;
  }
}

void flash_led(int var)
{
  switch (var)
  {
    case 1:   // LEDs off
      lcd.pinLEDs(FlashLED_A, LOW);
      lcd.pinLEDs(FlashLED_B, LOW);
      break;
    case 2:
      lcd.pinLEDs(FlashLED_A, HIGH);
      lcd.pinLEDs(FlashLED_B, LOW);
      break;
    case 3:
      lcd.pinLEDs(FlashLED_A, LOW);
      lcd.pinLEDs(FlashLED_B, HIGH);
      break;
    case 4:
      lcd.pinLEDs(FlashLED_A, HIGH);
      lcd.pinLEDs(FlashLED_B, HIGH);
      break;
  }
}

void BuzzerOff()
{
  lcd.pinLEDs(buzzerPin, LOW);
  tBU.setCallback(&BuzzerOn);
}

void BuzzerOn()
{
  lcd.pinLEDs(buzzerPin, HIGH);
  tBU.setCallback(&BuzzerOff);
}

void BadSound(void)
{   // added by DieterH on 22.10.2017
  tBU.setInterval(100);
  tBU.setIterations(6); // I think it must be Beeps * 2?
  tBU.setCallback(&BuzzerOn);
  tBU.enable();
}

void GoodSound(void)
{
  lcd.pinLEDs(buzzerPin, HIGH);
  tBD.setCallback(&BuzzerOff);  // changed by DieterH on 18.10.2017
  tBD.restartDelayed(200);      // changed by DieterH on 18.10.2017
}

//  RFID ------------------------------
void dispRFID(void)
{
  lcd.print("Sys  V" + String(Version).substring(0,3) + " starts at:");
  lcd.setCursor(0, 1); lcd.print("Wait Sync xBee:");
}

void displayON()
{
  displayIsON = true;
  lcd.setBacklight(BACKLIGHTon);
  tB.disable();
  tM.enable();
  intervalRFID = 0;
}
// End Funktions --------------------------------

// Funktions Serial Input (Event) ---------------
void evalSerialData()
{
  inStr.toUpperCase();
  if (inStr.startsWith("OK"))
  {
    if (plplpl == 0)
    {
      ++plplpl;
      Serial.println("ATNI");
    }
    else
    {
      ++plplpl;
    }
  }

  if (inStr.startsWith("TOLO") && inStr.length() == 4)
  {
    Serial.println("ATCN");
    IDENT = inStr;
  }

  if (inStr.startsWith("TIME")) 
  {
    inStr.concat("                   ");     // add blanks to string
    lcd.setCursor(0, 1); lcd.print(inStr.substring(4,24));
    tB.setInterval(TASK_SECOND / 2);
    getTime = 255;
  }

  if (inStr.startsWith("NOREG") && inStr.length() == 5)
  {
    noreg();  // changed by D. Haude on 18.10.2017
  }

  if (inStr.startsWith("ONT") && inStr.length() >= 4 && inStr.length() < 6)
  {
    nextrun = false;
    nr2Open[0] = 0;
    CLOSE = inStr.substring(3).toInt();
    opendoors(CLOSE);
  }

  if (inStr.startsWith("ODI") && inStr.length() >= 4 && inStr.length() < 6)
  {
    nextrun = false;
    nr2Open[0] = 11;
    nr2Open[1] = inStr.substring(3, 4).toInt();
    nr2Open[2] = inStr.substring(4, 5).toInt();
    dooropened();
  }

  if (inStr.startsWith("TLLO") && inStr.length() == 4)
  {
    doorsclosed(); // Doors all closed
  }

  if (inStr.substring(0, 3) == "R3T" && inStr.length() >3)
  {  // print to LCD row 3
    inStr.concat("                   ");     // add blanks to string
    lcd.setCursor(0,2);
    lcd.print(inStr.substring(3,23)); // cut string lenght to 20 char
  }

  if (inStr.substring(0, 3) == "R4T" && inStr.length() >3)
  {  // print to LCD row 4
    inStr.concat("                   ");     // add blanks to string
    lcd.setCursor(0,3);
    lcd.print(inStr.substring(3,23));   // cut string lenght to 20 char  changed by MM 10.01.2018
  }
  inStr = "";
}

/* SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent()
 {
  char inChar = (char)Serial.read();
  if (inChar == '\x0d')
  {
    evalSerialData();
    inStr = "";
  }
  else if (inChar != '\x0a')
  {
    inStr += inChar;
  }
}
// End Funktions Serial Input -------------------

// PROGRAM LOOP AREA ----------------------------
void loop() {
  runner.execute();
}
