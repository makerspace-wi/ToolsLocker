/* DESCRIPTION
  ====================
  started on 09Sep2020 - uploaded on 06.09.2020 by Michael
  Code for Toolslocker control over RFID
  reading IDENT from xBee, retrait sending ...POR until time responds
  switch on power (12V) if someone chip is registered for Toolslocker
  open a door if switch is pushed

  Commands to Raspi --->
  xBeeName - from xBee (=Ident) [max 4 caracter including numbers] {xBeeName + n}

  'Ident;POR;Vx.x.x' - machine power on reset and sw version (Ident;por;Vx.x.x)
  'card;nn...'       - uid_2 from reader
  'Ident;ont'        - Ident is startet and time is on
  'Ident;WO;drxx'    - Ident; want open door with number xx? (colum|row) ---> "dr34"
  'Ident;OP;drxx'    - Ident; door opened with number xx (colum|row) ---> "dr34"
  'Ident;CL;drxx'    - Ident; door closed with number xx (colum|row) ---> "dr34"
  'Ident;ER;drxx'    - Ident; error - door has not opened with number xx (colum|row) ---> "dr34"
  'Ident;ER;I2Cx'    - Ident; error - I2C not available with num x
  'Ident;ER;nuxx'    - Ident; error - number xx not available
  'Ident;off'        - Ident reporting nobody logged in
  'Ident;cmd?;'      - Ident reporting unknown command
  'Ident;Num?;'      - Ident reporting unkown number

  Commands from Raspi (Commands send from raspi, are allways inspected and if known, executed!!!)
  'time'   - format time33.33.3333 33:33:33
  'noreg'  - RFID-Chip not registed
  'ontxx'  - on to open doors for time xx min "ont15"
  'odiyy'  - open door immediately with number yy (colum|row) ---> "odi34"
  'ldhhh'  - leds direct hhh = Hex col 1, 2, 3 row 4321 = val 8421 = byte
  'lbhhh'  - leds blink hhh = Hex col 1, 2, 3 row 4321 = val 8421 = byte
  'tllo'   - from toolslocker log all off
  'bueon'  - Buzzer Extern ON
  'bueto'  - Buzzer Extern TOggle
  'bueof'  - Buzzer Extern OFf
  'sbtzzz' - set blink time between zzz = 50ms ---> <1000ms (default = 100ms)
  'r3t...' - display text in row 3 "r3tabcde12345", max 20
  'r4t...' - display text in row 4 "r4tabcde12345", max 20

  last change: 20.10.2024 by Michael Muehl
  changed: add toogle for external buzzer
*/
#define Version "1.2.3" // (Test =1.2.x ==> 1.2.4)
#define xBeeName  "TOL" // Name and number for xBee
#define checkFA      2  // event check for every (1 second / FActor)

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
#define SSR_BUZZER  A3  // SSR Buzzer on / off  (Zentral Buzzer)

#define xBuError     8  // xBee and Bus error (13)

// I2C IOPort definition
byte I2CFound = 0;
int I2CAdress[] = {0, 0, 0, 0};
byte I2CTransmissionResult = 0;
#define I2CPort   0x20  // I2C Adress MCP23017 LCD Display
// check chanel is active
#define I2CDoors1 1     // I2C Adress MCP23017 Door row 1 (0x20 + 1)
#define I2CDoors2 2     // I2C Adress MCP23017 Door row 2 (0x20 + 2)
#define I2CDoors3 3     // I2C Adress MCP23017 Door row 3 (0x20 + 3)

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
// BUTTON_P1         6  // not used
// BUTTON_P2         7  // StopSwitch
// BACKLIGHT for LCD-Display
#define BACKLIGHToff 0x0
#define BACKLIGHTon  0x1

// Pin Assignments Door Control (I2C (21, 22, 23) Port A: 2 door  Port B 2 doors)
// Array lock & position, button & Led
int schloss[] = {4, 0, 4, 8, 12}; // [Numbers, Port 1, Port 2, Port 3, Port 4]
int tastLed[] = {4, 1, 5, 9, 13};
int posLock[] = {4, 2, 6, 10, 14};
int butTast[] = {4, 3, 7, 11, 15};

int flashNum[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte sc = 0; // variable for lock colum
byte sr = 0; // variable for lock row
byte lc = 0; // variable for led colum
byte lr = 0; // variable for led row
byte ln = 0; // variable for led number
byte alc[5]; // array for cols hhh

// DEFINES
#define porTime        5 // [  5] wait seconds for sending Ident + POR
#define disLightOn    30 // [.30] display light on for seconds
#define CLOSE2END     15 // [ 15] MINUTES blinking before activation is switched off
#define CLEANON        6 // [  6] TASK_SECOND dust collector on after current off
#define repMES         1 // [  1] repeat commands
#define periRead     100 // [100] ms read analog input for 50Hz (current)
#define currHyst      10 // [ 10] hystereses for current detection
#define currMean       3 // [  3] current average over ...
#define intervalINC 3600 // [3600] * 4
#define intervalPush   5 // [  5] seconds to push button before clean starts
#define intervalCLMn  30 // [ 30] min clean time in seconds
#define intervalCLMx  10 * 60 // [600] max clean time in seconds

#define stepsTB        2 // [  2] steps for door tools locker
#define debouTBo       2 // [  2] debounce button door tool locker open
#define debouTBc       5 // [  5] debounce door tool locker close

// CREATE OBJECTS
Scheduler r;
LCDLED_BreakOUT lcd = LCDLED_BreakOUT();
MFRC522 rfid(SS_PIN, RST_PIN);  // Create MFRC522 instance
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
void ToolDoorsLed();
void ToolOpenDoor();

// Functions define for C++
void OnTimed(long);
void flash_led(int);

// TASKS
Task tM(TASK_SECOND / 2, TASK_FOREVER, &checkXbee, &r);	         // 500ms main task
Task tR(TASK_SECOND / 2, 0, &repeatMES, &r);                     // 500ms * repMES repeat messages
Task tU(TASK_SECOND / checkFA, TASK_FOREVER, &UnLoCallback, &r); // 1000ms / checkFA ctor
Task tB(TASK_SECOND * 5, TASK_FOREVER, &BlinkCallback, &r);      // 5000ms blinking for several events

Task tBU(TASK_SECOND / 10, 6, &BuzzerOn, &r);                    // 100ms 6x =600ms buzzer on for 6 times
Task tBD(1, TASK_ONCE, &FlashCallback, &r);                      // Flash Delay leds
Task tDF(1, TASK_ONCE, &DispOFF, &r);                            // display off after xx sec

Task tTBC(TASK_SECOND / 10, TASK_FOREVER, &ToolButCheck, &r);    // 100ms Check if a button is pressed for open or close one door
Task tTDL(TASK_SECOND / 5, TASK_FOREVER, &ToolDoorsLed, &r);     // Doors led flashing
Task tTOD(1, TASK_ONCE, &ToolOpenDoor, &r);                      // Open a door

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
byte intervalRFID = 0;       // 0 = off; from 1 sec to 6 sec after Displayoff
bool displayIsON = false;    // if display is switched on = true
bool nextrun = false;        // programm runs more then 1 time

// tool locker doors:
unsigned int CLOSE = CLOSE2END; // RAM cell for before activation is off
bool checkDoors = false;     // not all doors are closed
int toloNR  = 0;             // toolslocker number
byte countDoors = 0;         // counter for doors until the are closed
byte countTBo = 0;           // counter for tools locker (debounce open)
byte countTBc = 0;           // counter for tools locker (debounce closed)
bool dooropend = false;      // one door is opened
bool togleds = false;        // toggle led doors

// Serial with xBee
String inStr = "";      // a string to hold incoming data
String IDENT = "";      // Machine identifier for remote access control
String SFMes = "";      // String send for repeatMES
String inChr = "";      // Input char
byte co_ok = 0;         // send +++ control AT sequenz OK
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

  SPI.begin();             // SPI

  rfid.PCD_Init();      // Init MFRC522
  // rfid.PCD_SetAntennaGain(rfid.RxGain_avg);
  rfid.PCD_SetAntennaGain(rfid.RxGain_max);

  // IO MODES
  pinMode(xBuError, OUTPUT);
  pinMode(SSR_POWER, OUTPUT);
  pinMode(SSR_BUZZER, OUTPUT);

  // Set default values
  digitalWrite(xBuError, HIGH); // turn the LED ON (init start)
  digitalWrite(SSR_POWER, LOW);
  digitalWrite(SSR_BUZZER, LOW);

  r.startNow();

  // Check if I2C _ Ports are avilable
  for (sr = 0; sr < 4; sr++)
  {
    Wire.beginTransmission(I2CPort + sr);
    I2CTransmissionResult = Wire.endTransmission();
    if (I2CTransmissionResult == 0)
    {
      I2CAdress[sr] = sr;
      I2CFound++;
    }
  }

  // I2C Bus mit slave vorhanden
  if (I2CFound >= 1)
  {
    lcd.begin(20, 4);        // initialize the LCD
    if (I2CAdress[1] == I2CDoors1)
      tld1.begin(I2CDoors1); // used address (2)1
    if (I2CAdress[2] == I2CDoors2)
      tld2.begin(I2CDoors2); // used address (2)2
    if (I2CAdress[3] == I2CDoors3)
      tld3.begin(I2CDoors3); // used address (2)4

    if (I2CAdress[1] == I2CDoors1)
    {
      for (sr = 1; sr < 5; sr++)
      {
        tld1.pinMode(schloss[sr], OUTPUT);
        tld1.pinMode(tastLed[sr], OUTPUT);
        tld1.pinMode(posLock[sr], INPUT);
        tld1.pinMode(butTast[sr], INPUT); // (int A later?)

        tld1.digitalWrite(schloss[sr], LOW);
        tld1.digitalWrite(tastLed[sr], LOW);
      }
    }

    if (I2CAdress[2] == I2CDoors2)
    {
      for (sr = 1; sr < 5; sr++)
      {
        tld2.pinMode(schloss[sr], OUTPUT);
        tld2.pinMode(tastLed[sr], OUTPUT);
        tld2.pinMode(posLock[sr], INPUT);
        tld2.pinMode(butTast[sr], INPUT); // (int A later?)

        tld2.digitalWrite(schloss[sr], LOW);
        tld2.digitalWrite(tastLed[sr], LOW);
      }
    }

    if (I2CAdress[3] == I2CDoors3)
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

    lcd.clear();
    lcd.pinLEDs(buzzerPin, LOW);
    lcd.pinLEDs(BUT_P1_LED, LOW);
    but_led(1);
    flash_led(1);
    dispRFID();
    tM.enable();         // xBee check
    Serial.print(F("+++")); //Starting the request of IDENT
  }
  else
  {
    tB.enable();  // enable Task Error blinking
    tB.setInterval(TASK_SECOND);
  }
}
// Setup End -----------------------------------

// TASK (Functions) ----------------------------
void checkXbee()  // check if xBee name is read out
{
  if (IDENT.startsWith(xBeeName) && co_ok == 2)
  {
    ++co_ok;
    tB.setCallback(retryPOR);
    tB.enable();
    digitalWrite(xBuError, LOW); // turn the LED off (Programm start)
  }
}

void retryPOR()   // wait until time string arrived
{
  tDF.restartDelayed(TASK_SECOND * disLightOn); // restart display light
  if (getTime < porTime * 5)
  {
    Serial.println(String(IDENT) + ";POR;V" + String(Version));
    ++getTime;
    tB.setInterval(TASK_SECOND * getTime);
    lcd.setCursor(0, 0); lcd.print(String(IDENT) + " ");
    lcd.setCursor(16, 1); lcd.print((getTime - porTime) * porTime);
  }
  else if (getTime == 255)
  {
    tR.setIterations(repMES);
    tM.setCallback(checkRFID);
    tM.enable();
    displayON();
  }
}

void checkRFID()  // wait until rfid token is recognized
{ // 500ms Tick
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial())
  {
    code = 0;
    for (byte i = 0; i < rfid.uid.size; i++)
    {
      code = ((code + rfid.uid.uidByte[i]) * 10);
    }
    if (!digitalRead(SSR_POWER))
    { // Check if machine is switched on
      flash_led(4);
      tBD.setCallback(&FlashCallback);
      tBD.restartDelayed(100);
      tDF.restartDelayed(TASK_SECOND * disLightOn);
    }
    Serial.println("card;" + String(code));
    // Display changes
    lcd.setCursor(5, 0); lcd.print(F("               "));
    lcd.setCursor(0, 0); lcd.print(F("Card# ")); lcd.print(code);
    displayON();
  }
}

void UnLoCallback() // switch off check for ontime
{   // 500ms Tick
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
    --timer;
    minutes = timer / (60 * checkFA);
    if (timer % (60 * checkFA) == 0)
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
void repeatMES()    // --repeat messages from machines 
{
  Serial.println(String(SFMes));
}

void BlinkCallback()  // --Blink if BUS Error
{
  digitalWrite(xBuError, !digitalRead(xBuError));
}

void FlashCallback()  // flash time for LEDS
{
  flash_led(1);
}

void DispOFF()    // switch display off and clear
{
  displayIsON = false;
  lcd.setBacklight(BACKLIGHToff);
  lcd.clear();
  but_led(1);
  flash_led(1);
}

void ToolDoorsLed()
{
  togleds = !togleds;
  for (ln = 0; ln < 12; ln++)
  {
    if (flashNum[ln] > 0)
    {
      lc = flashNum[ln] / 10;
      lr = flashNum[ln] - lc * 10;
      if (lc > 10)
        lc = lc - 10;
      if (togleds && flashNum[ln] < 100)
      {
        if (lc == I2CAdress[1])
        {
          tld1.digitalWrite(tastLed[lr], HIGH);
        }
        if (lc == I2CAdress[2])
        {
          tld2.digitalWrite(tastLed[lr], HIGH);
        }
        if (lc == I2CAdress[3])
        {
          tld3.digitalWrite(tastLed[lr], HIGH);
        }
      }
      else
      {
        if (lc == I2CAdress[1])
        {
          tld1.digitalWrite(tastLed[lr], LOW);
          if (flashNum[ln] > 100)
            flashNum[ln] = 0;
        }
        if (lc == I2CAdress[2])
        {
          tld2.digitalWrite(tastLed[lr], LOW);
          if (flashNum[ln] > 100)
            flashNum[ln] = 0;
        }
        if (lc == I2CAdress[3])
        {
          tld3.digitalWrite(tastLed[lr], LOW);
          if (flashNum[ln] > 100)
            flashNum[ln] = 0;
        }
      }
    }
  }
}

void ToolButCheck()
{
  digitalWrite(xBuError, HIGH);
  if (nr2Open[0] == 0)
  {
    for (sr = 1; sr < 5; sr++)
    {
      if (I2CAdress[1] == I2CDoors1)
      { // if channel 1 is a live
        if (!tld1.digitalRead(butTast[sr]) && !tld1.digitalRead(posLock[sr]) && countTBo == debouTBo)
        {
          nr2Open[0] = -5;
          nr2Open[1] = I2CDoors1;
          nr2Open[2] = sr;
          Serial.println(String(IDENT) + ";WO" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        }
        if (((nr2Open[1] == 3 && nr2Open[2] == sr) || !tld1.digitalRead(butTast[sr])) && countTBc == 0 && countTBo < debouTBo)
        {
          ++countTBo;
        }
      }

      if (I2CAdress[2] == I2CDoors2)
      { // if channel 2 is a live
        if (!tld2.digitalRead(butTast[sr]) && !tld2.digitalRead(posLock[sr]) && countTBo == debouTBo)
        {
          nr2Open[0] = -5;
          nr2Open[1] = I2CDoors2;
          nr2Open[2] = sr;
          Serial.println(String(IDENT) + ";WO" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        }
        if (((nr2Open[1] == 3 && nr2Open[2] == sr) || !tld2.digitalRead(butTast[sr])) && countTBc == 0 && countTBo < debouTBo)
        {
          ++countTBo;
        }
      }

      if (I2CAdress[3] == I2CDoors3)
      { // if channel 3 is a live
        if (!tld3.digitalRead(butTast[sr]) && !tld3.digitalRead(posLock[sr]) && countTBo == debouTBo)
        {
          nr2Open[0] = -5;
          nr2Open[1] = I2CDoors3;
          nr2Open[2] = sr;
          Serial.println(String(IDENT) + ";WO" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        }
        if (((nr2Open[1] == 3 && nr2Open[2] == sr) || !tld3.digitalRead(butTast[sr])) && countTBc == 0 && countTBo < debouTBo)
        {
          ++countTBo;
        }
      }
    }
  }
  else if (nr2Open[0] == -1)
  {
    if (nr2Open[1] == I2CAdress[1])
    {
      if (tld1.digitalRead(butTast[nr2Open[2]]) && !tld1.digitalRead(posLock[nr2Open[2]]) && countTBc == debouTBc)
      {
        countTBo = 0;
        countTBc = 0;
        tld1.digitalWrite(tastLed[nr2Open[2]], LOW);
        Serial.println(String(IDENT) + ";CL" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
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

    if (nr2Open[1] == I2CAdress[2])
    {
      if (tld2.digitalRead(butTast[nr2Open[2]]) && !tld2.digitalRead(posLock[nr2Open[2]]) && countTBc == debouTBc)
      {
        countTBo = 0;
        countTBc = 0;
        tld2.digitalWrite(tastLed[nr2Open[2]], LOW);
        Serial.println(String(IDENT) + ";CL" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
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

    if (nr2Open[1] == I2CAdress[3])
    {
      if (tld3.digitalRead(butTast[nr2Open[2]]) && !tld3.digitalRead(posLock[nr2Open[2]]) && countTBc == debouTBc)
      {
        countTBo = 0;
        countTBc = 0;
        tld3.digitalWrite(tastLed[nr2Open[2]], LOW);
        Serial.println(String(IDENT) + ";CL" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
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
  digitalWrite(xBuError, LOW);
}

void ToolOpenDoor()
{
  if (nr2Open[0] == -1)
  {
    if (nr2Open[1] == I2CAdress[1])
    {
      if (tld1.digitalRead(posLock[nr2Open[2]]))
      {
        tTOD.disable();
        tld1.digitalWrite(schloss[nr2Open[2]], LOW);
        countTBo = debouTBo + stepsTB;
        Serial.println(String(IDENT) + ";OP" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        tTBC.enable();
      }
      else
      {
        tld1.digitalWrite(schloss[nr2Open[2]], LOW);
        tld1.digitalWrite(tastLed[nr2Open[2]], LOW);
        tTOD.restartDelayed(TASK_SECOND / 10);
        ++countTBo;
        if (countTBo % 10 == 0)
        {
          tld1.digitalWrite(schloss[nr2Open[2]], HIGH);
          tld1.digitalWrite(tastLed[nr2Open[2]], HIGH);
          Serial.println(String(IDENT) + ";ER" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        }
        if (countTBo >= 10 * 10)
        {
          tld1.digitalWrite(schloss[nr2Open[2]], LOW);
          tld1.digitalWrite(tastLed[nr2Open[2]], LOW);
          lcd.setCursor(0, 2);
          countTBo = debouTBo + stepsTB;
          tTOD.disable();
          tTBC.enable();
        }
      }
    }

    if (nr2Open[1] == I2CAdress[2])
    {
      if (tld2.digitalRead(posLock[nr2Open[2]]))
      {
        tTOD.disable();
        tld2.digitalWrite(schloss[nr2Open[2]], LOW);
        countTBo = debouTBo + stepsTB;
        Serial.println(String(IDENT) + ";OP" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        tTBC.enable();
      }
      else
      {
        tld2.digitalWrite(schloss[nr2Open[2]], LOW);
        tld2.digitalWrite(tastLed[nr2Open[2]], LOW);
        tTOD.restartDelayed(TASK_SECOND / 10);
        ++countTBo;
        if (countTBo % 10 == 0)
        {
          tld2.digitalWrite(schloss[nr2Open[2]], HIGH);
          tld2.digitalWrite(tastLed[nr2Open[2]], HIGH);
          Serial.println(String(IDENT) + ";ER" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        }
        if (countTBo >= 10 * 10)
        {
          tld2.digitalWrite(schloss[nr2Open[2]], LOW);
          tld2.digitalWrite(tastLed[nr2Open[2]], LOW);
          countTBo = debouTBo + stepsTB;
          tTOD.disable();
          tTBC.enable();
        }
      }
    }

    if (nr2Open[1] == I2CAdress[3])
    {
      if (tld3.digitalRead(posLock[nr2Open[2]]))
      {
        tTOD.disable();
        tld3.digitalWrite(schloss[nr2Open[2]], LOW);
        countTBo = debouTBo + stepsTB;
        Serial.println(String(IDENT) + ";OP" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        tTBC.enable();
      }
      else
      {
        tld3.digitalWrite(schloss[nr2Open[2]], LOW);
        tld3.digitalWrite(tastLed[nr2Open[2]], LOW);
        tTOD.restartDelayed(TASK_SECOND / 10);
        ++countTBo;
        if (countTBo % 10 == 0)
        {
          tld3.digitalWrite(schloss[nr2Open[2]], HIGH);
          tld3.digitalWrite(tastLed[nr2Open[2]], HIGH);
          Serial.println(String(IDENT) + ";ER" + ";DR" + String(nr2Open[1]) + String(nr2Open[2]));
        }
        if (countTBo >= 10 * 10)
        {
          tld3.digitalWrite(schloss[nr2Open[2]], LOW);
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
int getNum(String strNum) // Check if realy numbers
{
  strNum.trim();
  for (byte i = 0; i < strNum.length(); i++)
  {
    if (!isDigit(strNum[i])) 
    {
      Serial.println(String(IDENT) + ";?;" + inStr + ";Num?;" + strNum);
      lcd.setCursor(0, 2); lcd.print(F("no mumber           "));
      lcd.setCursor(0, 3); lcd.print(F("logout and reset    "));
      return 0;
    }
  }
  return strNum.toInt();
}

void noact()
{ // no action
  digitalWrite(SSR_POWER, LOW);
  digitalWrite(SSR_BUZZER, LOW);
  tM.enable();
  BadSound();
  but_led(1);
  flash_led(1);
  tDF.restartDelayed(TASK_SECOND * disLightOn);
}

void opendoors(long min)
{   // Turn on machine for minutes
  onTime = true;
  timer = min * (60 * checkFA);
  Serial.println(String(IDENT) + ";ont");
  char tbs[8];
  sprintf(tbs, "% 4d", timer / (60 * checkFA));
  lcd.setCursor(0, 3); lcd.print(F("Time left (min):")); lcd.print(tbs);
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
  lcd.setCursor(0, 2);
  lcd.print(F("Access granted      "));
  tR.disable();
  tU.enable();
  tTDL.enable();
  tTBC.enable();
}

void dooropened(void)
{
  if (nr2Open[1] == I2CAdress[1])
  {
    if (!tld1.digitalRead(posLock[nr2Open[2]]))
    {
      dooropend = true;
      tTBC.disable();
      flashNum[(nr2Open[1] + 3 * (nr2Open[2] - 1)) - 1] = 0;
      tld1.digitalWrite(schloss[nr2Open[2]], HIGH);
      tld1.digitalWrite(tastLed[nr2Open[2]], HIGH);
      timer = 0;
      onTime = false;
      but_led(3);
      flash_led(4);
      nr2Open[0] = -1;
      tTOD.enable();
      tTOD.restartDelayed(100);
    }
  }

  if (nr2Open[1] == I2CAdress[2])
  {
    if (!tld2.digitalRead(posLock[nr2Open[2]]))
    {
      dooropend = true;
      tTBC.disable();
      flashNum[(nr2Open[1] + 3 * (nr2Open[2] - 1)) - 1] = 0;
      tld2.digitalWrite(schloss[nr2Open[2]], HIGH);
      tld2.digitalWrite(tastLed[nr2Open[2]], HIGH);
      timer = 0;
      onTime = false;
      but_led(3);
      flash_led(4);
      nr2Open[0] = -1;
      tTOD.enable();
      tTOD.restartDelayed(100);
    }
  }

  if (nr2Open[1] == I2CAdress[3])
  {
    if (!tld3.digitalRead(posLock[nr2Open[2]]))
    {
      dooropend = true;
      tTBC.disable();
      flashNum[(nr2Open[1] + 3 * (nr2Open[2] - 1)) - 1] = 0;
      tld3.digitalWrite(schloss[nr2Open[2]], HIGH);
      tld3.digitalWrite(tastLed[nr2Open[2]], HIGH);
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

void doorsclosed(void)
{ // Switch off machine and stop
  tTBC.disable();
  tTDL.disable();
  tU.disable();
  timer = 0;
  but_led(2);
  digitalWrite(SSR_POWER, LOW);
  digitalWrite(SSR_BUZZER, LOW);
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
  lcd.print(F("System shut down at"));
}

void BuzExtTog()
{ // toogle SSR_Buzzer for gong
  digitalWrite(SSR_BUZZER, !digitalRead(SSR_BUZZER));
}

void but_led(int var)
{ // button led switching
  switch (var)
  {
    case 1: // LED rt & gn off
      lcd.pinLEDs(StopLEDrt, HIGH);
      lcd.pinLEDs(StopLEDgn, HIGH);
      break;
    case 2: // RED LED on
      lcd.pinLEDs(StopLEDrt, LOW);
      lcd.pinLEDs(StopLEDgn, HIGH);
      break;
    case 3: // GREEN LED on
      lcd.pinLEDs(StopLEDrt, HIGH);
      lcd.pinLEDs(StopLEDgn, LOW);
      break;
  }
}

void flash_led(int var)
{ // flash leds switching
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

void BuzzerOff()  // switch buzzer off
{
  lcd.pinLEDs(buzzerPin, LOW);
  tBU.setCallback(&BuzzerOn);
}

void BuzzerOn()   // switch buzzer on
{
  lcd.pinLEDs(buzzerPin, HIGH);
  tBU.setCallback(&BuzzerOff);
}

void BadSound(void) // generate bad sound
{   // added by DieterH on 22.10.2017
  tBU.setInterval(100);
  tBU.setIterations(6); // I think it must be Beeps * 2?
  tBU.setCallback(&BuzzerOn);
  tBU.enable();
}

void GoodSound(void) // generate good sound
{
  lcd.pinLEDs(buzzerPin, HIGH);
  tBD.setCallback(&BuzzerOff);  // changed by DieterH on 18.10.2017
  tBD.restartDelayed(200);      // changed by DieterH on 18.10.2017
}

//  RFID ------------------------------
void dispRFID(void)
{
  lcd.print("Sys V" + String(Version).substring(0,3) + " starts at:");
  lcd.setCursor(0, 1); lcd.print(F("Wait Sync xBee:"));
}

void displayON()  // switch display on
{
  displayIsON = true;
  lcd.setBacklight(BACKLIGHTon);
  tB.disable();
  tM.enable();
  intervalRFID = 0;
}

int checkValues()
{
  if (sc == 0 || sc > 3 || sr == 0 || sr > 4)
  {
    Serial.println(String(IDENT) + ";ER" + ";NU" + String(sc) + String(sr));
    return 0;
  }
  
  if ((sc == I2CAdress[1] || sc == I2CAdress[2] || sc == I2CAdress[3]))
  {
    return 1;
  }
  else
  {
    Serial.println(String(IDENT) + ";ER" + ";I2C" + String(sc));
    return 0;
  }
}
// End Funktions --------------------------------

// Funktions Serial Input (Event) ---------------
void evalSerialData()
{
  inStr.toUpperCase();
  if (inStr.startsWith("OK"))
  {
    if (co_ok == 0)
    {
      Serial.println(F("ATNI"));
      ++co_ok;
    }
    else
    {
      ++co_ok;
    }
  }
  else if (co_ok ==1  && inStr.length() == 4)
  {
    if (inStr.startsWith(xBeeName))
    {
      IDENT = inStr;
      toloNR = inStr.substring(1).toInt();
      Serial.println(F("ATCN"));
    }
    else
    {
      lcd.setCursor(0, 2);
      lcd.print("?:" + inStr + ";not for:" + xBeeName); 
    }
  }
  else if (inStr.startsWith("TIME"))
  {
    inStr.concat("                   ");     // add blanks to string
    lcd.setCursor(0, 1); lcd.print(inStr.substring(4,24));
    tB.setInterval(TASK_SECOND / 2);
    getTime = 255;
  }
  else if (inStr.startsWith("NOREG") && inStr.length() ==5)
  { // chip not registed
    lcd.setCursor(0, 2); lcd.print(F("Tag not registered  "));
    lcd.setCursor(0, 3); lcd.print(F("===> No access! <==="));
    noact();
  }
  else if (inStr.startsWith("ONT") && inStr.length() >= 4 && inStr.length() < 7) 
  {
    nextrun = false;
    nr2Open[0] = 0;
    CLOSE = getNum(inStr.substring(3));
    if (CLOSE > 0) opendoors(CLOSE);
  }
  else if (inStr.startsWith("ODI") && inStr.length() >= 4 && inStr.length() < 6)
  {
    sc = getNum(inStr.substring(3, 4));
    sr = getNum(inStr.substring(4, 5));
    if (checkValues() == 1)
    {
      nr2Open[0] = 11;
      nr2Open[1] = sc;
      nr2Open[2] = sr;
      dooropened();
    }
  }
  else if (inStr.startsWith("LD") && inStr.length() == 5)
  {
    inChr = inStr.substring(2, 5);
    inChr.getBytes(alc, 4);
    for (byte i = 0; i < 4; i++)
    {
      if (alc[i] > 57)
      {
        alc[i] = alc[i] - 55;
      }
      else
      {
        alc[i] = alc[i] - 48;
      }
    }
    if (I2CAdress[1] == I2CDoors1)
    {
      tld1.digitalWrite(tastLed[1], bitRead(alc[0], 0));
      tld1.digitalWrite(tastLed[2], bitRead(alc[0], 1));
      tld1.digitalWrite(tastLed[3], bitRead(alc[0], 2));
      tld1.digitalWrite(tastLed[4], bitRead(alc[0], 3));
    }
    if (I2CAdress[2] == I2CDoors2)
    {
      tld2.digitalWrite(tastLed[1], bitRead(alc[1], 0));
      tld2.digitalWrite(tastLed[2], bitRead(alc[1], 1));
      tld2.digitalWrite(tastLed[3], bitRead(alc[1], 2));
      tld2.digitalWrite(tastLed[4], bitRead(alc[1], 3));
    }
    if (I2CAdress[3] == I2CDoors3)
    {
      tld3.digitalWrite(tastLed[1], bitRead(alc[2], 0));
      tld3.digitalWrite(tastLed[2], bitRead(alc[2], 1));
      tld3.digitalWrite(tastLed[3], bitRead(alc[2], 2));
      tld3.digitalWrite(tastLed[4], bitRead(alc[2], 3));
    }
  }
  else if (inStr.startsWith("LB") && inStr.length() == 5)
  {
    inChr = inStr.substring(2, 5);
    inChr.getBytes(alc, 4);
    for (byte i = 0; i < 4; i++)
    {
      if (alc[i] > 57)
      {
        alc[i] = alc[i] - 55;
      }
      else
      {
        alc[i] = alc[i] - 48;
      }
    }
    for (sc = 1; sc < 4; sc++)
    {
      for (sr = 1; sr < 5; sr++)
      {
        if (bitRead(alc[sc - 1], sr - 1))
        {
          flashNum[(sc + 3 * (sr - 1)) - 1] = (sc * 10 + sr);
        }
        else
        {
          flashNum[(sc + 3 * (sr - 1)) - 1] = flashNum[(sc + 3 * (sr - 1)) - 1] + 100;
        }
      }
    }
  }
  else if (inStr.startsWith("TLLO") && inStr.length() == 4)
  {
    doorsclosed(); // Doors all closed
  }
  else if (inStr.startsWith("BUEON") && inStr.length() == 5)
  {
    digitalWrite(SSR_BUZZER, HIGH);
    tB.disable();
  }
  else if (inStr.startsWith("BUETO") && inStr.length() == 5)
  {
    digitalWrite(SSR_BUZZER, HIGH);
    tB.setCallback(BuzExtTog);
    tB.enable();
  }
  else if (inStr.startsWith("BUEOF") && inStr.length() == 5)
  {
    digitalWrite(SSR_BUZZER, LOW);
    tB.disable();
  }
  else if (inStr.startsWith("SBT") && inStr.length() >= 4 && inStr.length() < 7)
  {
    val = getNum(inStr.substring(3, 6));
    if (val >= 50 && val < 1000)
    {
      tTDL.setInterval(val);
    }
  }
  else if (inStr.substring(0, 3) == "R3T" && inStr.length() > 3)
  {  // print to LCD row 3
    inStr.concat("                    ");     // add blanks to string
    lcd.setCursor(0,2);
    lcd.print(inStr.substring(3,23)); // cut string lenght to 20 char
  }
  else if (inStr.substring(0, 3) == "R4T" && inStr.length() >3)
  {  // print to LCD row 4
    inStr.concat("                    ");     // add blanks to string
    lcd.setCursor(0,3);
    lcd.print(inStr.substring(3,23));   // cut string lenght to 20 char  changed by MM 10.01.2018
  }
  else
  {
    Serial.println(String(IDENT) + ";cmd?;" + inStr);
    lcd.setCursor(0, 2); lcd.print(F("Unknown command     "));
    lcd.setCursor(0, 3); lcd.print(F("logout and reset    "));
    noact();
  }
  inStr = "";
}
// End Funktions Serial Input -------------------

// PROGRAM LOOP AREA ----------------------------
void loop()
{
  r.execute();
  if (Serial.available() > 0)
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
}
