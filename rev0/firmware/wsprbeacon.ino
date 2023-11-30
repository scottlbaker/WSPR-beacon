
// ============================================================================
//
// An Arduino-based WSPR Beacon
//
// (c) Scott Baker KJ7NLA
//
//  Arduino IDE settings:
//  board: Arduino UNO
//  bootloader: no bootloader
//  programmer: AVRISP mkII
//
// Libraries
// ---------
// wspr.h         - a WSPR encoding lib
// si5351.h       - by Milldrum and Myers
// i2c.h          - a simple I2C lib
// ee.h           - a simple EEPROM lib
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// ============================================================================

#define DEBUG1    0                     // set to 1 for debug
#define DEBUG2    0                     // set to 1 for debug
#define VERSION   "WSPR 1.00A"          // firmware version
#define DATE      "Nov 29 2023"         // firmware release date

// Arduino Pins
#define SW1       5   // PD5   push button   (pin  9)
#define TXD       7   // PD7   serial data   (pin 11)
#define RXD       8   // PB0   serial data   (pin 12)
#define LED      13   // PB5   status LED    (pin 17)

#include "wspr.h"
#include "si5351.h"
#include "i2c.h"
#include "ee.h"
#include <avr/pgmspace.h>

// generic
#define OFF    0
#define ON     1
#define NO     0
#define YES    1
#define FALSE  0
#define TRUE   1
#define MANUAL 0
#define CAT    1

// delay times (ms)
#define DEBOUNCE          10
#define ONE_SECOND      1000
#define TWO_SECONDS     2000
#define TEN_SECONDS    10000

// user interface macros
#define NBP  0  // no-button-pushed
#define BSC  1  // button-single-click
#define BPL  2  // button-push-long

#define LONGPRESS  500
#define UIKEY      !digitalRead(SW1)

#define TIMER0_ON    0x03   // timer0 CTC clk/64
#define TIMER0_OFF   0x00   // timer0 CTC clk off
#define TIMSK0_ON    0x02   // timer0 interrupt on
#define TIMSK0_OFF   0x00   // timer0 interrupt off

#define TIMER1_ON    0x0d   // timer1 CTC clk/1024
#define TIMER1_OFF   0x08   // timer1 CTC clk off
#define TIMSK1_ON    0x02   // timer1 interrupt on
#define TIMSK1_OFF   0x00   // timer1 interrupt off

#define TIMER2_ON    0x02   // timer 2 CTC clk/8
#define TIMER2_OFF   0x00   // timer 2 CTC clk off
#define TIMSK2_ON    0x02   // timer 2 interrupt on
#define TIMSK2_OFF   0x00   // timer 2 interrupt off

// band assignments
#define BAND160  11
#define BAND80   10
#define BAND60    9
#define BAND40    8
#define BAND30    7
#define BAND20    6
#define BAND17    5
#define BAND15    4
#define BAND12    3
#define BAND10    2
#define BAND06    1
#define UNKNOWN   0

// WSPR Frequency Table
#define WSPR160   183810000
#define WSPR80    357010000
#define WSPR60    528870000
#define WSPR40    704010000
#define WSPR30   1014020000
#define WSPR20   1409710000
#define WSPR17   1810610000
#define WSPR15   2109610000
#define WSPR12   2492610000
#define WSPR10   2812610000
#define WSPR06   5029450000
#define CALFREQ  2800000000

// band labels
const char* band_label[] = {
"??", "6M", "10M","12M","15M","17M",
"20M","30M","40M","60M","80M","160M" };

// constant strings
char const change_msg[] PROGMEM = {
  "\r\n\
  press +- to change" };

char const fast_slow_msg[] PROGMEM = {
  "\r\n\
  press f/s/x for fast/slow/stop" };

char const save_exit_msg[] PROGMEM = {
  "\r\n\
  press . to save\r\n\
  press \\ to exit\r\n" };

char const ee_save_msg[] PROGMEM = {
  "Saved to EEPROM\r\n" };

char const ee_exit_msg[] PROGMEM = {
  "  Exiting without saving\r\n" };

char const help_msg[] PROGMEM = {
  "\r\n\
  A => TX schedule\r\n\
  B => band\r\n\
  C => cal mode\r\n\
  D => disable TX\r\n\
  E => enable TX\r\n\
  H => print help\r\n\
  I => print info\r\n\
  P => power (dBm)\r\n\
  Q => tx tune\r\n\
  R => reset\r\n\
  S => callsign\r\n\
  T => print time\r\n\n" };

// calibration data
#define CAL_DATA_INIT  64000
uint32_t cal_data = CAL_DATA_INIT;

// class instantiation
Si5351  si5351;
WSPR    wspr;
I2C     i2c;
EE      eeprom;

// global variables
uint8_t  enable_tx = TRUE;      // enable transmit
uint8_t  led_state = OFF;       // LED state
uint8_t  led_count = 0;         // LED count

#define BUFSIZE 98
volatile char buffer[BUFSIZE+2];

#define INIT_CHECK 314159265
uint32_t chk_data = 0;          // check if initialized

uint8_t  bandID    = BAND20;    // supported bands
uint64_t band_freq = WSPR20;    // WSPR band frequency

char mh4char[7];  // 4-character locator
char mh6char[7];  // 6-character locator

#define MINSATS 3
uint8_t numSats = 0;

#define CALLSIZE 17
char CallSign[CALLSIZE+2];

// global variables
volatile uint32_t msTimer   = 0;
volatile uint32_t loop_time = 0;
volatile uint8_t  timer1Tick = OFF;
volatile uint8_t  uart_state = 0;
volatile uint8_t  NMEAstart = NO;
volatile uint8_t  NMEAready = NO;

// timer constants
#define MS_CTC      249   // 1 ms
#define BAUD_CTC1    90   // first tick
#define BAUD_CTC2   210   // 9600 baud
#define WSPR_CTC  10672   // 667 us

uint8_t gotMH     = FALSE;
uint8_t gotTime   = FALSE;
uint8_t sigQual   = 0;    // 1 == OK

// transmit schedule
#define MIN_SCHD 1
#define MAX_SCHD 6

const uint8_t schd[7] =
  { 0,4,10,20,30,40,50
  };

uint8_t txIndex = 1;
uint8_t txTime  = schd[txIndex];

// power settings
#define MIN_DBM 1
#define MAX_DBM 18
#define DBM30   9

const uint8_t dbm[19] =
  { 0,3,7,10,13,17,20,23,27,30,
    33,37,40,43,47,50,53,57,60
  };

uint8_t dbmIndex = DBM30;
uint8_t dbmPower = dbm[dbmIndex];

uint8_t Hours   = 0;
uint8_t Minutes = 0;
uint8_t Seconds = 0;

char hrstr[3]  = {"00"};  // GPS hours
char mnstr[3]  = {"00"};  // GPS minutes
char scstr[3]  = {"00"};  // GPS seconds
char satstr[3] = {"00"};  // number of satellites
char sigstr[3] = {"00"};  // GPS signal quality

// lat/lon  strings
char latstr[12] = "0000.00000";
char lonstr[12] = "0000.00000";
char latdeg[3]  = "00";
char londeg[4]  = "000";
char latmin[9]  = "00.00000";
char lonmin[9]  = "00.00000";

char NSid = 'N';   // North-South
char EWid = 'W';   // East-West

float Latitude  = 0.0;
float Longitude = 0.0;

// WSPR prototype defs
void txWSPRmsg();
void processWSPR();
void checkGPStime();
void printGPStime();
void conv2min();
void getMaidenHead();

// GPG NMEA parsing
void restartBuf();
void parseNMEA();

// other prototype defs
void wait_ms(uint16_t dly);
void wait_us(uint16_t dly);
void blinkLED();
void blinkLED2();
void check_UI();
void putstr(char *str);
void putconst(char const *str);
void putch(char ch);
void print8(uint8_t val);
void print16(uint16_t val);
void print32(uint32_t val);
void print_freq(uint64_t val);
inline void putdot();
uint8_t sdRdy();
char getch();
void putCR();
void clrUart();
void copystr(char *dst, char *src);
uint8_t cmpstr(char *s1, char *s2);
uint8_t cmpcmd(char *dst, char *src);
uint8_t validcmd();
void CAT_control();
void prompt(char *str);
void show_info();
void show_version();
void show_call();
void enter_callsign();
void select_power();
void tx_tune(uint8_t cat);
void select_sched();
void select_band();
void band2freq();
void factory_reset();
void run_calibrate();
void save_eeprom();
void check_cal();
void start_rx();

// setup prototype defs
void init_pins();
void init_uart();
void init_i2c();
void init_eeprom();
void init_VFO();
void init_timer0();
void init_timer1();
void init_timer2();
void init_pci();
inline void start_timer0();
inline void stop_timer0();
inline void start_timer1();
inline void stop_timer1();
inline void start_timer2();
inline void stop_timer2();
inline void enable_pci();
inline void disable_pci();
inline void gie_ON();
inline void gie_OFF();

// catch undefined interrupts
ISR(BADISR_vect) {
  while(TRUE) {
    blinkLED();
    wait_ms(300);
  }
}

// watchdog ISR
ISR(WDT_vect) {
}

// timer 0 interrupt service routine
ISR(TIMER0_COMPA_vect) {
  msTimer++;
}

// timer 1 interrupt service routine
ISR(TIMER1_COMPA_vect) {
  timer1Tick = ON;
}

// timer 2 interrupt service routine
ISR(TIMER2_COMPA_vect) {
  uint8_t rxpin;
  uint8_t nmea_state;
  uint8_t NMEAcmd = 0;
  static char ch;
  static uint8_t windx = 0;
  switch (uart_state) {
    case 0:
      OCR2A = BAUD_CTC2;  // 9600 baud
      ch = 0;
      uart_state++;
      break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
      if (!NMEAready) {
        rxpin = digitalRead(RXD);
        ch |= (rxpin << (uart_state-1));
      }
      uart_state++;
      break;
    case 9:
      stop_timer2();
      nmea_state = (NMEAstart<<3) + NMEAready;
      if (windx >= BUFSIZE) {
        nmea_state = 8;
        ch = '$';  // force end of string
      }
      switch (nmea_state) {
        case 0:  // NMEAstart=NO NMEAready=NO
          if (ch == '$') NMEAstart = YES;
          break;
        case 1:  // NMEAstart=NO  NMEAready=YES
          break;
        case 8:  // NMEAstart=YES NMEAready=NO
          if (ch == '$') {
            buffer[windx] = 0;
            windx = 0;
            NMEAcmd = validcmd();
            if (NMEAcmd) {
              NMEAready = YES;
              parseNMEA();
            }
            else restartBuf();
          } else {
            buffer[windx++] = ch;
          }
          break;
        case 9:  // NMEAstart=YES NMEAready=YES
          break;
        default:
          break;
      }
      enable_pci();
      uart_state = 0;
      break;
    default:
      uart_state = 0;
      break;
  }
}

// rxd pin change service routine
ISR(PCINT0_vect) {
  disable_pci();
  OCR2A = BAUD_CTC1;  // first tick
  start_timer2();
}

// transmit WSPR message
void txWSPRmsg() {
  uint8_t i;
  uint8_t txbuf[WSPR_SYMBOL_COUNT+2];
  #define TONE_HZ  146     // 1.46 Hz
  disable_pci();
  stop_timer2();
  digitalWrite(LED,ON);
  putstr("** Tx Start  ");
  printGPStime();
  start_timer1();
  memset(txbuf, 0, WSPR_SYMBOL_COUNT);
  wspr.encode(CallSign, mh4char, dbmPower, txbuf, 1);
  si5351.output_enable(SI5351_CLK0, ON);
  for(i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    si5351.set_freq((band_freq + (txbuf[i] * TONE_HZ)), SI5351_CLK0);
    timer1Tick = OFF;
    putdot();
    if ((i==53)||(i==107)) putCR();
    while(!timer1Tick);
  }
  si5351.output_enable(SI5351_CLK0, OFF);
  stop_timer1();
  digitalWrite(LED,OFF);
  putstr("\r\n** Tx End\r\n");
  start_rx();
  restartBuf();
}

// check if it is time to transmit
void checkGPStime() {
  gotTime = FALSE;
  // for longer tx schedules do not transmit
  // at the top of the hour
  if ((txTime > 30) && (Minutes == 0)) return;
  if ((Seconds < 2) && (!(Minutes % txTime))) {
    if (enable_tx && gotMH) {
      if (cmpstr(CallSign, "MYCALL")) {
        putstr("ERROR: Please set Callsign\r\n");
      } else if (cal_data == CAL_DATA_INIT) {
        putstr("ERROR: Please run calibrate\r\n");
      } else {
        txWSPRmsg();  // send message
      }
    }
    Seconds = 50;  // to prevent re-trigger
  }
}

// print GPS time
void printGPStime() {
  print8(Hours);    putch(':');
  print8(Minutes);  putch(':');
  print8(Seconds);  putCR();
}

// convert coordinates to decimal
void conv2min() {
  uint8_t i=0;
  uint8_t j=0;
  // latitude
  while (i<2) latdeg[i++] = latstr[i];
  while (j<8) latmin[j++] = latstr[i++];
  // longitude
  i=0;
  j=0;
  while (i<3) londeg[i++] = lonstr[i];
  while (j<8) lonmin[j++] = lonstr[i++];
}

// calculate the Maidenhead locator
void getMaidenHead() {
  uint8_t o1, o2, o3;
  uint8_t a1, a2, a3;
  double remainder;
  conv2min();
  Latitude   = (float)atoi(latdeg);
  Latitude  += (atof(latmin))/60.0;
  Longitude  = (float)atoi(londeg);
  Longitude += (atof(lonmin))/60.0;
  if (NSid == 'S') Latitude  = -Latitude;
  if (EWid == 'W') Longitude = -Longitude;
  remainder = Longitude + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (double)o2;
  o3 = (int)(12.0 * remainder);
  remainder = Latitude + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (double)a2;
  a3 = (int)(24.0 * remainder);
  // 6-character locator
  mh6char[0] = (char)o1 + 'A';
  mh6char[1] = (char)a1 + 'A';
  mh6char[2] = (char)o2 + '0';
  mh6char[3] = (char)a2 + '0';
  mh6char[4] = (char)o3 + 'a';
  mh6char[5] = (char)a3 + 'a';
  mh6char[6] = (char)0;
  // 4-character locator
  copystr(mh4char, mh6char);
  mh4char[4] = (char)0;
  gotMH = TRUE;
}

// restart GPS data acquisition
void restartBuf() {
  NMEAstart = NO;
  NMEAready = NO;
  dbmPower = dbm[dbmIndex];
}

// parse NMEA sentence
void parseNMEA() {
  uint8_t done = NO;
  uint8_t i = 0;
  uint8_t rindx = 6;
  uint8_t gps_state = 0;
  char ch;
  while (!done) {
    ch = buffer[rindx++];
    switch (gps_state) {
      case 0:
        hrstr[0] = ch;
        gps_state++;
        break;
      case 1:
        hrstr[1] = ch;
        gps_state++;
        break;
      case 2:
        mnstr[0] = ch;
        gps_state++;
        break;
      case 3:
        mnstr[1] = ch;
        gps_state++;
        break;
      case 4:
        scstr[0] = ch;
        gps_state++;
        break;
      case 5:
        scstr[1] = ch;
        gps_state++;
        if (sigQual && (numSats >= MINSATS)) {
          gotTime = TRUE;
          Hours   = atoi(hrstr);
          Minutes = atoi(mnstr);
          Seconds = atoi(scstr);
        }
        if (gotMH) done = YES;  // get time only
        break;
      case 6:
        if (ch == ',') gps_state++;
        break;
      case 7:
        if (ch == ',') {
          latstr[i] = 0;
          i = 0;
          gps_state++;
        }
        else latstr[i++] = ch;
        break;
      case 8:
        NSid = ch;
        gps_state++;
        break;
      case 9:
        if (ch == ',') gps_state++;
        break;
      case 10:
        if (ch == ',') {
          lonstr[i] = 0;
          i = 0;
          gps_state++;
        }
        else lonstr[i++] = ch;
        break;
      case 11:
        EWid = ch;
        gps_state++;
        break;
      case 12:
        if (ch == ',') gps_state++;
        break;
      case 13:
        sigstr[1] = ch;
        sigQual = atoi(sigstr);
        gps_state++;
        break;
      case 14:
        if (ch == ',') gps_state++;
        break;
      case 15:
        satstr[0] = ch;
        gps_state++;
        break;
      case 16:
        satstr[1] = ch;
        numSats = atoi(satstr);
        done = YES;
        if (sigQual && (numSats >= MINSATS)) getMaidenHead();
        break;
      default:
        done = YES;
        break;
    }
  }
  restartBuf();
}

// millisecond delay
void wait_ms(uint16_t dly) {
  uint32_t startTime = msTimer;
  while((msTimer - startTime) < dly) {
    wait_us(10);
  }
}

// microsecond delay
void wait_us(uint16_t x) {
  uint16_t t = ((x * 3) + (x>>1));
  for (int16_t i=0; i<t; i++) {
    asm("");
  }
}

// blink the LED
void blinkLED() {
  digitalWrite(LED,led_state);
  led_state = !led_state;
}

// blink the LED (slower rate)
void blinkLED2() {
  led_count++;
  if (led_count == 1) {
    digitalWrite(LED,led_state);
    led_state = !led_state;
    led_count = 0;
  }
}

// check the UI pushbutton
void check_UI() {
  uint8_t  event = NBP;
  uint32_t t0;
  if (UIKEY) {
    event = BSC;
    t0 = msTimer;
    // check for long button press
    while (UIKEY && (event != BPL)) {
      if (msTimer > (t0 + LONGPRESS)) event = BPL;
      delay(DEBOUNCE);
    }
    switch (event) {
      case BSC:        // short click
        break;
      case BPL:        // long press
        tx_tune(MANUAL);
        break;
      default:
        break;
    }
  }
}

// print a string
void putstr(char *str) {
  uint8_t i=0;
  while(str[i] != '\0') {
    putch(str[i]);
    i++;
  }
}

// print a constant string
void putconst(char const *str) {
  char ch = ' ';
  while(ch != '\0') {
    ch = pgm_read_byte(str++);
    putch(ch);
  }
}

// print an 8-bit integer value
void print8(uint8_t val) {
  char tmp[4] = "  0";
  // convert to string
  for (uint8_t i=2; val; i--) {
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  // left justify
  while (tmp[0] == ' ') {
    tmp[0] = tmp[1];
    tmp[1] = tmp[2];
    tmp[2] = tmp[3];
  }
  putstr(tmp);
}

// print an 16-bit integer value
void print16(uint16_t val) {
  char tmp[6] = "    0";
  // convert to string
  for (uint8_t i=4; val; i--) {
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  // left justify
  while (tmp[0] == ' ') {
    tmp[0] = tmp[1];
    tmp[1] = tmp[2];
    tmp[2] = tmp[3];
    tmp[3] = tmp[4];
    tmp[4] = tmp[5];
  }
  putstr(tmp);
}

// print a 32-bit frequency value
void print32(uint32_t val) {
  char tmp[11] = "          ";
  // convert to string
  for (uint8_t i=9; val; i--) {
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  putstr(tmp);
}

// print a frequency value
void print_freq(uint64_t val) {
  char tmp[9] = "        ";
  val /= 100;
  int i = 7;
  // convert to string
  for (; val; i--) {
    tmp[i] = "0123456789"[val % 10];
    val /= 10;
  }
  putstr(tmp);
}

// print a char
void putch(char ch) {
  while (!(UCSR0A&(1<<UDRE0)));
  UDR0 = ch;
  while (!(UCSR0A&(1<<UDRE0)));
}

// print a dot
inline void putdot() {
  UDR0 = '.';
}

// serial data available
uint8_t sdRdy() {
  if (UCSR0A & (1<<RXC0)) return(TRUE);
  else return(FALSE);
}

// get a char
char getch() {
  while (!(UCSR0A & (1<<RXC0)));
  return UDR0;
}

// print a linefeed
void putCR() {
  putch('\r');
  putch('\n');
}

// flush uart
void clrUart() {
  char ch;
  while (sdRdy()) {
    ch = UDR0;
    wait_us(100);
  }
}

// copy a string
void copystr(char *dst, char *src) {
  uint8_t i = 0;
  while (src[i]) {
    dst[i] = src[i];
    i++;
  }
  dst[i] = 0;
}

// compare two strings
uint8_t cmpstr(char *s1, char *s2) {
  uint8_t i=0;
  while(s1[i] == s2[i]) {
    if(!s1[i] || !s2[i]) break;
    i++;
  }
  if(!s1[i] && !s2[i]) return TRUE;
  else return FALSE;
}

// compare two NMEA commands
uint8_t cmpcmd(char *cmd) {
  uint8_t i=2;
  for (uint8_t j=0; j<3; i++, j++ ) {
    if (buffer[i] != cmd[j]) return FALSE;
  }
  return TRUE;
}

// check for valid NMEA commands
uint8_t validcmd() {
  if (cmpcmd("GGA")) return 1;
  else return 0;
}

// handle console commands
void CAT_control() {
  char ch = UDR0;
  noInterrupts();
  if ((ch >= 'a') && (ch <= 'z')) ch -= 32;
  switch (ch) {
    case 'A':          // TX schedule
      select_sched();
      break;
    case 'B':          // select band
      select_band();
      break;
    case 'C':          // calibrate
      run_calibrate();
      break;
    case 'D':          // disable transmit
      enable_tx = FALSE;
      putstr(">> Tx Off\r\n");
      break;
    case 'E':          // enable transmit
      enable_tx = TRUE;
      putstr(">> Tx On\r\n");
      break;
    case 'I':          // print info
      show_info();
      break;
    case 'H':          // print help
      putconst(help_msg);
      break;
    case 'P':          // power (dBm)
      select_power();
      break;
    case 'Q':          // tx tune
      tx_tune(CAT);
      break;
    case 'R':          // factory reset
      factory_reset();
      break;
    case 'S':          // my callsign
      enter_callsign();
      break;
    case 'T':          // print time
      printGPStime();
      break;
    default:
      break;
  }
  clrUart();
  interrupts();
}

// print change prompt
void prompt(char *str) {
  putstr("\r\nCurrent ");
  putstr(str);
  putstr(" is ");
}

// print info and config data
void show_info() {
  show_version();                // firmware version
  show_call();                   // callsign
  putstr(latstr);                // lat/lon
  putch(NSid);
  putstr("  ");
  putstr(lonstr);
  putch(EWid);
  putCR();
  putstr(band_label[bandID]);    // band
  putstr(" ");
  print_freq(band_freq);         // band frequency
  putstr(" Hz\r\n");
  putstr("enable tx ");
  print8(enable_tx);             // enable transmit
  putCR();
  putstr("satellites ");         // satellites
  print8(numSats);
  putCR();
  putstr("signal qual ");
  print8(sigQual);               // signal quality
  putCR();
  putstr("schedule ");
  print8(txTime);                // transmit period
  putstr(" minutes\r\n");
  printGPStime();                // current time
}

// show version
void show_version() {
  putCR();
  putstr(VERSION);
  putCR();
  putstr(DATE);
  putCR();
}

// print callsign and location
void show_call() {
  putstr(CallSign);              // ham callsign
  putstr("  ");
  putstr(mh6char);               // grid location
  putstr("  ");
  print8(dbmPower);              // Tx power (dBm)
  putstr("dBm\r\n");
}

// eeprom addresses
#define DATA_ADDR    10
#define BAND_ADDR    20
#define INIT_ADDR    30
#define MPWR_ADDR    40
#define SCHD_ADDR    50
#define CALL_ADDR    60

uint8_t ix;

// enter my callsign (CAT command)
void enter_callsign() {
  char ch;
  uint8_t done = NO;
  prompt("Callsign");
  putstr(CallSign);
  putconst(save_exit_msg);
  ix = 0;
  while (!done) {
    if (sdRdy()) {
      ch = UDR0;
      if ((ch == '.') || (ch == 0x0a) || (ch == 0x0d)) {
        done = YES;
        eeprom.putstr(CALL_ADDR, CallSign);
        putCR();
        putconst(ee_save_msg);
      } else if ((ch == '/') || (ch == '\\')) {
        done = YES;
        eeprom.getstr(CALL_ADDR, CallSign);
      } else {
        if ((ch >= 'a') && (ch <= 'z')) ch -= 32;
        putch(ch);
        CallSign[ix++] = ch;
        CallSign[ix] = '\0';
        if (ix > CALLSIZE) ix=0;
      }
    }
  }
}

// select power setting
void select_power() {
  char ch;
  uint8_t done = NO;
  uint8_t saved_value = dbmIndex;
  prompt("Power");
  print8(dbmPower);
  putstr(" dBm");
  putconst(change_msg);
  putconst(save_exit_msg);
  while (!done) {
    if (sdRdy()) {
      ch = UDR0;
      switch (ch) {
        case '+':      // increment
          dbmIndex++;
          if (dbmIndex > MAX_DBM) dbmIndex = MAX_DBM;
          dbmPower = dbm[dbmIndex];
          print8(dbmPower);
          putCR();
          break;
        case '-':      // decrement
          dbmIndex--;
          if (dbmIndex < MIN_DBM) dbmIndex = MIN_DBM;
          dbmPower = dbm[dbmIndex];
          print8(dbmPower);
          putCR();
          break;
        case '.':      // exit with eeprom update
          eeprom.put(MPWR_ADDR, dbmIndex);
          putconst(ee_save_msg);
          done = YES;
          break;
        case '\\':     // exit without eeprom update
          dbmIndex = saved_value;
          dbmPower = dbm[dbmIndex];
          putconst(ee_exit_msg);
          done = YES;
          break;
        default:
          break;
      }
    }
  }
}

char const tune_on_msg[] PROGMEM = {
  "\r\n** Tune On\r\n" };

char const tune_off_msg[] PROGMEM = {
  "** Tune Off\r\n" };
  
char const any_key_msg[] PROGMEM = {
  "   Press any key to stop\r\n" };

// transmit on for antenna tuning
void tx_tune(uint8_t cat) {
  putconst(tune_on_msg);
  digitalWrite(LED,ON);
  si5351.set_freq(band_freq, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, ON);
  if (cat) {
    putconst(any_key_msg);
    while (!sdRdy());
    clrUart();
  } else {
    while (UIKEY) delay(DEBOUNCE);
  }
  si5351.output_enable(SI5351_CLK0, OFF);
  putconst(tune_off_msg);
  digitalWrite(LED,OFF);
}

// select tx schedule
void select_sched() {
  char ch;
  uint8_t done = NO;
  uint8_t saved_value = txIndex;
  prompt("Schedule");
  print8(txTime);
  putstr(" minutes");
  putconst(change_msg);
  putconst(save_exit_msg);
  while (!done) {
    if (sdRdy()) {
      ch = UDR0;
      switch (ch) {
        case '+':      // increment
          txIndex++;
          if (txIndex > MAX_SCHD) txIndex = MIN_SCHD;
          txTime = schd[txIndex];
          print8(txTime);
          putCR();
          break;
        case '-':      // decrement
          txIndex--;
          if (txIndex < MIN_SCHD) txIndex = MAX_SCHD;
          txTime = schd[txIndex];
          print8(txTime);
          putCR();
          break;
        case '.':      // exit with eeprom update
          eeprom.put(SCHD_ADDR, txIndex);
          putconst(ee_save_msg);
          done = YES;
          break;
        case '\\':     // exit without eeprom update
          txIndex = saved_value;
          txTime = schd[txIndex];
          putconst(ee_exit_msg);
          done = YES;
          break;
        default:
          break;
      }
    }
  }
}

// select band (CAT command)
void select_band() {
  char ch;
  uint8_t done = NO;
  uint8_t saved_value = bandID;
  #define MAX_BAND BAND160
  #define MIN_BAND BAND06
  prompt("Band");
  putstr(band_label[bandID]);
  putconst(change_msg);
  putconst(save_exit_msg);
  while (!done) {
    if (sdRdy()) {
      ch = UDR0;
      switch (ch) {
        case '+':      // increment
          bandID++;
          if (bandID > MAX_BAND) bandID = MIN_BAND;
          putstr(band_label[bandID]);
          putCR();
          break;
        case '-':      // decrement
          bandID--;
          if (bandID < MIN_BAND) bandID = MAX_BAND;
          putstr(band_label[bandID]);
          putCR();
          break;
        case '.':      // exit with eeprom update
          band2freq();
          eeprom.put(BAND_ADDR, bandID);
          putconst(ee_save_msg);
          done = YES;
          break;
        case '\\':     // exit without eeprom update
          bandID = saved_value;
          putconst(ee_exit_msg);
          done = YES;
          break;
        default:
          break;
      }
    }
  }
}

// band to frequency lookup
void band2freq() {
  switch (bandID) {
    case BAND160:
      band_freq = WSPR160;
      break;
    case BAND80:
      band_freq = WSPR80;
      break;
    case BAND60:
      band_freq = WSPR60;
      break;
    case BAND40:
      band_freq = WSPR40;
      break;
    case BAND30:
      band_freq = WSPR30;
      break;
    case BAND20:
      band_freq = WSPR20;
      break;
    case BAND17:
      band_freq = WSPR17;
      break;
    case BAND15:
      band_freq = WSPR15;
      break;
    case BAND12:
      band_freq = WSPR12;
      break;
    case BAND10:
      band_freq = WSPR10;
      break;
    case BAND06:
      band_freq = WSPR06;
      break;
  }
}

// factory reset (CAT command)
void factory_reset() {
  putstr("=> Factory Reset\r\n");
  cal_data  = CAL_DATA_INIT;
  chk_data  = INIT_CHECK;
  bandID    = BAND20;
  band_freq = WSPR20;
  dbmIndex  = DBM30;
  dbmPower  = dbm[dbmIndex];
  txIndex   = MIN_SCHD;
  txTime    = schd[txIndex];
  gotTime   = FALSE;
  Hours     = 0;
  Minutes   = 0;
  Seconds   = 0;
  copystr(CallSign, "MYCALL");
  copystr(mh4char,  "AA00");
  copystr(mh6char,  "AA00aa");
  save_eeprom();
}

// calibrate the VFO (CAT command)
void run_calibrate() {
  char ch;
  char echo = ' ';;
  uint8_t up = FALSE;
  uint8_t dn = FALSE;
  uint8_t xx = 0;
  #define FASTADJUST 100
  #define SLOWADJUST 10
  uint8_t adjval = FASTADJUST;
  uint8_t done = NO;
  disable_pci();
  stop_timer2();
  cal_data = eeprom.get32(DATA_ADDR);
  putstr("\r\nCal Data ");
  print32(cal_data);
  putconst(change_msg);
  putconst(fast_slow_msg);
  putconst(save_exit_msg);
  si5351.output_enable(SI5351_CLK2, ON);
  while (!done) {
    if (sdRdy()) {
      ch = UDR0;
      if ((ch >= 'a') && (ch <= 'z')) ch -= 32;
      switch (ch) {
        case '+':      // increment
          echo = ch;
          up = TRUE;
          dn = FALSE;
          break;
        case '-':      // decrement
          echo = ch;
          up = FALSE;
          dn = TRUE;
          break;
        case 'F':      // fast tune
          putch(ch);
          adjval = FASTADJUST;
          break;
        case 'S':      // slow tune
          putch(ch);
          adjval = SLOWADJUST;
          break;
        case 'X':      // stop tune
        case '=':
          putch(ch);
          up = FALSE;
          dn = FALSE;
          break;
        case '.':      // exit with eeprom update
        case '\\':     // exit without eeprom update
          done = YES;
          up = FALSE;
          dn = FALSE;
          break;
        default:
          break;
      }
    }
    if (up||dn) {
      if (up) cal_data = cal_data - adjval;
      if (dn) cal_data = cal_data + adjval;
      si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
      si5351.set_freq(CALFREQ, SI5351_CLK2);
      wait_us(1000);
      if (xx == 0) putch(echo);
      if (xx++ == 100) xx = 0;
    }
  }
  si5351.output_enable(SI5351_CLK2, OFF);
  if (ch == '.') {
    eeprom.put32(DATA_ADDR, cal_data);
    putCR();
    putconst(ee_save_msg);
  } else {
    putconst(ee_exit_msg);
  }
  start_rx();
  restartBuf();
}

// write to the eeprom
void save_eeprom() {
  eeprom.put32(DATA_ADDR, cal_data);
  eeprom.put32(INIT_ADDR, chk_data);
  eeprom.put(BAND_ADDR, bandID);
  eeprom.put(MPWR_ADDR, dbmIndex);
  eeprom.put(SCHD_ADDR, txTime);
  eeprom.putstr(CALL_ADDR, CallSign);
  putconst(ee_save_msg);
}

// watchdog constants
#define WD_START   0x40
#define WD_STOP    0x00

// initialize pins
void init_pins() {
  pinMode(LED, OUTPUT);
  pinMode(TXD, OUTPUT);
  pinMode(RXD, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);
  digitalWrite(LED,OFF);
  digitalWrite(TXD,ON);
  WDTCSR = WD_STOP;
}

// initialize the uart (for CAT)
void init_uart() {
  UBRR0H = 0x00;
  UBRR0L = 0x08;  // 115200 baud
  UCSR0C = 0x06;  // 8-data 1-stop
  UCSR0B = 0x18;  // Rx/TX enabled
}

// initialize the I2C bus
void init_i2c() {
  i2c.begin();
}

// load user data from eeprom
void init_eeprom() {
  bandID   = eeprom.get(BAND_ADDR);
  dbmIndex = eeprom.get(MPWR_ADDR);
  txIndex  = eeprom.get(SCHD_ADDR);
  cal_data = eeprom.get32(DATA_ADDR);
  chk_data = eeprom.get32(INIT_ADDR);
  eeprom.getstr(CALL_ADDR, CallSign);
}

// initialize the VFO
void init_VFO() {
  si5351.init();
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.output_enable(SI5351_CLK0, OFF);
  si5351.output_enable(SI5351_CLK1, OFF);
  si5351.output_enable(SI5351_CLK2, OFF);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
  si5351.set_freq(CALFREQ, SI5351_CLK2);
}

// initialize timer 0
void init_timer0() {
  TCCR0A = 0x02;          // CTC mode
  OCR0A = MS_CTC;         // 1 ms
  stop_timer0();
}

// initialize timer 1
void init_timer1() {
  TCCR1A = 0x00;          // CTC mode
  OCR1A = WSPR_CTC;       // 667 us
  stop_timer1();
}

// initialize timer 2
void init_timer2() {
  TCCR2A = 0x02;          // CTC mode
  OCR2A  = BAUD_CTC1;     // first tick
  stop_timer2();
}

// initialize rx pin interrupt
inline void init_pci() {
  PCICR  = 0x01;  // PCIE0
  PCMSK0 = 0x00;  // PCINT0
}

// start timer 0
inline void start_timer0() {
  TCCR0B = TIMER0_ON;
  TIMSK0 = TIMSK0_ON;
}

// stop timer 0
inline void stop_timer0() {
  TCCR0B = TIMER0_OFF;
  TIMSK0 = TIMSK0_OFF;
  TCNT0  = 0;
}

// start timer 1
inline void start_timer1() {
  TCCR1B = TIMER1_ON;
  TIMSK1 = TIMSK1_ON;
}

// stop timer 1
inline void stop_timer1() {
  TCCR1B = TIMER1_OFF;
  TIMSK1 = TIMSK1_OFF;
  TCNT1  = 0;
}

// start timer 2
inline void start_timer2() {
  TCCR2B = TIMER2_ON;
  TIMSK2 = TIMSK2_ON;
}

// stop timer 2
inline void stop_timer2() {
  TCCR2B = TIMER2_OFF;
  TIMSK2 = TIMSK2_OFF;
  TCNT2  = 0;
}

// enable rx pin interrupt
inline void enable_pci() {
  PCMSK0 = 0x01;  // PCINT0
}

// disable rx pin interrupt
inline void disable_pci() {
  PCMSK0 = 0x00;
}

// global interrupt enable
inline void gie_ON() {
  SREG |= 0x80;
}

// global interrupt disable
inline void gie_OFF() {
  SREG &= 0x7f;
}

// check cal data for factory reset
void check_cal() {
  if ((chk_data != INIT_CHECK) ||
    ((cal_data < 100) || (cal_data > 100000))) {
    factory_reset();
  }
  dbmPower = dbm[dbmIndex];
  txTime   = schd[txIndex];
  band2freq();
}

// wait for rx pin to remain idle
void start_rx() {
  uint16_t i;
  uint32_t j = 0;
  uint8_t done = NO;
  uint8_t rxdpin;
  while (!done) {
    for (i=0; i<255; i++) {
      rxdpin = digitalRead(RXD);
      if (!rxdpin) break;
    }
    if (i > 250) done = YES;
    j++;
    if (j > 500000) {
      putstr("ERROR: GPS Timeout\r\n");
      while(TRUE) {
        blinkLED();
        wait_ms(500);
      }
    }
  }
  enable_pci();
}

int main() {

  // setup
  init();
  init_pins();
  init_timer0();
  init_timer1();
  init_timer2();
  init_eeprom();
  init_pci();
  init_i2c();
  init_VFO();
  init_uart();
  show_version();
  start_timer0();
  check_cal();
  start_rx();

  // main loop
  loop_time = msTimer;
  while(TRUE) {
    #define ENDLOOP 500
    if ((msTimer - loop_time) >= ENDLOOP) {
      loop_time = msTimer;
      // check if the GPS time is updated
      if (gotTime) {
        checkGPStime();
        blinkLED2();
      }
    } else {
      check_UI();  // check pushbutton
      if (sdRdy()) CAT_control();
    }
  }
}

