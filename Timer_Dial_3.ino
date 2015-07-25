/*
 TIMER DIAL

 PUMP CONTROL

 Version 1.1 22.05.2015

 *** FOR ATTINY 84 ***

  The circuit:

 LCD:
 * LCD RS     pin 4  to digital pin 13 - (0)
 * LCD Enable pin 6  to digital pin 12 - (1)
 * LCD D4     pin 11 to digital pin 11 - (2)
 * LCD D5     pin 12 to digital pin 10 - (3)
 * LCD D6     pin 13 to digital pin  9 - (4)
 * LCD D7     pin 14 to digital din  8 - (5)
 * LCD R/W    pin 5 to ground
 * LCD VSS    pin 1 to ground
 * LCD VCC    pin 2 to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
// FLOW SWITCH ( tie to PUMP_ON if not used )
* Input FLOW_SW        Pin 7 PA6 (6)
// PUMP CONTROL OUTPUT
* Output PUMP_ON       Pin 6 PA7 (7)
// Rotating Digitalswitch
* Input START_STOP_SW  Pin 5 PB2 (8)
* Input TIME_DIAL_2    Pin 3 PB1 (9)
* Input TIME_DIAL_1    Pin 2 PB0 (10)

NOTE:
----

Pressing START_STOP_SW When reset toggles NO FLOW Alarm mode

*/

// Kick in some general definitions
// Encoder & Sw Library
#include <ClickEncoder.h>
// include the library code:
#include <LiquidCrystal.h>
// Timer Library
#include <MsTimer1.h>
// Debounce Library
#include <Bounce.h>
// Timer Library
#include <Timer.h>
//EEPROM library
#include <EEPROM.h>

// pins defnitions
#define FLOW_SW       6  /*6*/
#define PUMP_ON       7  /*7*/
#define START_STOP_SW 8  /*8*/
#define TIME_DIAL_1   10 /*10*/
#define TIME_DIAL_2   9  /*9*/
#define ONE_MS_TICK   1    /* 1ms Tick*/
#define TIMERTICK     100  /* 100 ms */
#define ALARMTICK     1    /* 1 S */

//#define DEBUG  /* uncomment for 60x speed debug mode*/
#ifdef DEBUG // speed UP
     #define TIMER_INC TIMERTICK*60
     #define ALARM_INC ALARMTICK*2
#else
     #define TIMER_INC TIMERTICK
     #define ALARM_INC ALARMTICK
#endif

#define TIME_F_ALARM    30 /* 30 seconds alarm time */

// Function Prototypes
void ito_hhmmss( char *s, unsigned long int n);

// initialize the library with the numbers of the interface pins
//  (RS,E,DB4,DB5,DB6,DB7)
LiquidCrystal lcd(0, 1, 2, 3, 4, 5);
// Instantiate a Bounce objecta with a 20 millisecond debounce time
Bounce bouncer = Bounce( FLOW_SW, 20 );
// Timer Object
Timer t (0, T_MONO , TIMER_INC);
// ALARM TIMER Object ( TIME_F_ALARM seconds )
Timer talarm ( TIME_F_ALARM, T_MONO, ALARM_INC );

/********************* GLOBAL VARIABLES *************************/
// last value
int16_t last = -1;
// 30 minutes default time
int16_t value = 30;
// habilitation to flow alarms
int flow_alarm = false;
//pointer to Encoder Object
ClickEncoder *encoder;

/*********************   Functions      **************************/

// process ISR From Timer 1 ( 10 ms )
void timer_tick() {
  static unsigned char ticks = 0;
  // process encoder
  encoder->service();
  //  PINA = (1 << PA6); // DEBUG
  // Update the debouncer
  bouncer.update ( );
  // clock chores each TIMERTICK
  if (++ticks >= TIMERTICK) {
    ticks = 0;
    clock_tick();
  }
}

// EXECUTE EACH TIMER TICK Ticks
void clock_tick()
{
  static unsigned int ms = 0;
  static unsigned int seg = 0;
  static unsigned int min = 0;
  // Put TIMERTICK tasks here
  t.Tick(); // increment timer by TIMERTICK ms
  // Write PUMP_ON  value  ON/OFF
  digitalWrite(PUMP_ON, (t.GetStatus() == T_RUN));
  ms += TIMERTICK;
  if (ms >= 1000) {
    ms = 0;
    seg++;
    // Put One Second tasks here
    talarm.Tick(); // kick alarm timer
    //    digitalWrite(PUMP_ON, !digitalRead(PUMP_ON));
  }
  if (seg >= 60) {
    seg = 0;
    min++;
    // Put One Minute tasks here
  }
}

// formats time for LCD

void ito_hhmmss( char* s, unsigned long int n, char blink) {
  // ( milliseconds)
  unsigned int hh, mm, ss;
  // FOR TESTING only
  //n=1000UL*(3600*4+60*12+13);
  // PARSE TIME VALUE ( MSECONDS )
  ss = (n / 1000UL) % 60;
  mm = (n / (1000 * 60UL)) % 60;
  hh =  n / (1000 * 3600UL);
  // GENERATE OUTPUT STRING
  s[0] = (hh / 10) + '0';
  s[1] = (hh % 10) + '0';
  s[2] = (blink ? ' ' : ':');
  s[3] = (mm / 10) + '0';
  s[4] = (mm % 10) + '0';
  s[5] = (blink ? ' ' : ':');
  s[6] = (ss / 10) + '0';
  s[7] = (ss % 10) + '0';
  s[8] = '\0';
}

void ito_ss( char* s, unsigned long int ss) {
  // ( milliseconds)
  // GENERATE OUTPUT STRING
  if ( ss > 9 ) s[0] = (ss / 10) + '0';
  else s[0] = ' ';
  s[1] = (ss % 10) + '0';
  s[2] = '\0';
}

/******************** SETUP ******************************/
void setup() {
  // some EEprom stuff
  int eeAddress = 0; //EEPROM address to start reading from
  bool changeFlag = false;
  // set up I/O
  pinMode(START_STOP_SW, INPUT_PULLUP);
  pinMode(FLOW_SW, INPUT);
  pinMode(PUMP_ON, OUTPUT);
  digitalWrite(PUMP_ON, false);
  // instance the encoder
  encoder = new ClickEncoder(TIME_DIAL_1, TIME_DIAL_2, START_STOP_SW, 2, false);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  // Print a message to the LCD.
  lcd.print(F("LCD SWITCH V 1.0"));
  delay(1000);
  // starts real display
  lcd.clear();
  // EEPROM Stuff for parameters
  // if eeprom is unused, set default value to 0 ( false )
  if (EEPROM.read(eeAddress ) == 255) EEPROM.update(eeAddress, false );

  flow_alarm = EEPROM.read(eeAddress );
  // Button pressed ?
  changeFlag = !digitalRead(START_STOP_SW);
  if (changeFlag) {
    // toggle flow alarm
    flow_alarm = !flow_alarm;
    EEPROM.update(eeAddress , flow_alarm);
  }
  // re red flow alarm flag
  flow_alarm = EEPROM.read(eeAddress);
  lcd.print(flow_alarm ? F("ALRM Habilit.") : F("ALRM Deshab."));
  lcd.setCursor(0, 1);
  if (changeFlag) lcd.print(F("*** CAMBIO ***"));
  delay(2000);
  lcd.clear();
  // start digit refresh timer 1
  MsTimer1::set(ONE_MS_TICK, timer_tick); // 1ms period
  MsTimer1::start();
}


/******************** MAIN LOOP ********************************/
void loop() {
  char s[17]; // LCD buffer
  // increment and bound value set by encoder
  value += 10 * encoder->getValue();
  value = constrain(value, 0, 180);
  if (value != last) {
    last = value;
  }
  // DEBUG Uncomment for permanent alarm processing, without setup
  // flow_alarm = true;
  
  // Displays Counter or set value
  lcd.setCursor(0, 0);
  ito_hhmmss( s, (t.GetStatus() == T_RUN) || (t.GetStatus() == T_HOLD) ? t.GetClock() : value * 60000L, false);
  lcd.print(s);

  // display Status
  lcd.setCursor(9, 0);
  switch (t.GetStatus()) {
    case T_RUN : lcd.print                     (F("--ON-- ")); break;
    case T_STOP: lcd.print                     (F("--OFF--")); break;
    case T_END : lcd.print((millis()%2000<1000)?F("--FIN--"):F("       ")); break;
    case T_HOLD: lcd.print((millis()%2000<1000)?F("--HLD--"):F("       ")); break;
    default:;
  }

  // Get the update value for flow switch
  lcd.setCursor(0, 1);
  lcd.print(bouncer.read() ? F("CARGA") : F("_____"));

  // Show alarm timer
  if ( flow_alarm) {
    // detects alarm timeout & holds timer
    lcd.setCursor(6, 1);
    if (talarm.GetStatus() == T_END) {
      lcd.print((millis()%2000<1000)?F("ALARMA"):F("      "));
      t.Hold();
    }
    else lcd.print(F("______"));

    lcd.setCursor(14, 1);
    ito_ss(s, talarm.GetClock());
    lcd.print(s);
    lcd.setCursor(13, 1);

  }


  // process flow alarm timer
  if (flow_alarm) {
    switch (t.GetStatus()) {
      // resets alarm timer if t not running
      case T_HOLD:
      case T_END : break;
      case T_STOP: talarm.Reset();  break;
      case T_RUN :
        // start alarm timer if not running
        if (talarm.GetStatus() != T_RUN)       talarm.Trigger();
        // Kicks alarm timer if running & flowing
        if  (bouncer.read()) talarm.Reset();
        // end of alarm timer, stop timer
        else if (talarm.GetStatus() == T_END)  t.Reset();
        break;
      default:;
    }
  }

  // READ & PROCESS USER CHOICES
  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::Pressed:
        break;
      case ClickEncoder::Held:
        // resets timers
        t.Reset();
        break;
      case ClickEncoder::Released:
        break;
      case ClickEncoder::Clicked:
        //if running hold
        if (flow_alarm) talarm.Reset();
        if  ((t.GetStatus() == T_RUN))       t.Hold();
        // if held continue
        else if ((t.GetStatus() == T_HOLD))  t.Continue();
        // else start timers
        else {
          // start timer
          t.SetClock((long) value * 60000L);
          t.Trigger();
        }
        break;
      default:;
    }
  }

// all's done. get some sleep
  delay (200);

}




