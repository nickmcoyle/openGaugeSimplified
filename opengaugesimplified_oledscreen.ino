/* An arduino uno trip computer for 2005 Toyota Camry (K-line ISO9141 protocol 
*  using MC33290 ISO K line chip to level shift the 12v logic from the car computer to work with arduino
*/

/* OBDuino
 Copyright (C) 2008-2009
 Main coding/ISO/ELM: Frédéric (aka Magister on ecomodder.com)
 LCD part: Dave (aka dcb on ecomodder.com), optimized by Frédéric
 Fixes and Features: 
  Russ: Added serial_rx_off() and serial_rx_on() functions to disable and enable serial receiver
        Modified iso_write_byte() to disable serial receiver while sending to avoid echos in buffer
        Modified iso_write_byte() to include intrabyte delay
        Modified iso_write_data() to remove intrabyte delay (now included in iso_write_byte())
        Modified iso_read_data() to return only data, not PID + data
        Modified iso_read_data() to insure minimum 55 ms delay between requests
        Modified pid_reslen table to correct some lengths-others appear incorrect but aren't supported
          by my car, so I can't verify.  Specifically, some lengths are 8, when max data size in 
          iso 9141 packet is 7.
  Mike: Added Tracking of Fuel wasted while idling, total for tank displayed when engine shut off.
        Modified iso_read_byte() to return 0 if no respose is received [for when ECU shuts off quicker
           then the engine so the progam will now know when engine is off and can save parameters]
        Backlight will turn off when engine is not running.
Still need to:
          Modify iso_init to allow re-init without resetting arduino.
          Fix code to retrieve stored trouble codes.
 
 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later
 version.
 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 You should have received a copy of the GNU General Public License along with
 this program; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 */


#include <stdio.h>
#include<EEPROM.h>
#include <avr/pgmspace.h>


///////////for 0.96" OLED display ssd1306 clone from ebay
#include <SPI.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> 
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 lcd(OLED_RESET);

// Others prototypes
void long_to_dec_str(long value, char *decs, byte prec);
//first attempt to initialize with ecu
bool first_attempt = true;

/* PID codes */
unsigned long  pid01to20_support;  // this one always initialized at setup()
unsigned long  pid21to40_support=0;
unsigned long  pid41to60_support=0;
#define PID_SUPPORT00 0x00
#define ENGINE_RPM    0x0C
#define VEHICLE_SPEED 0x0D
#define MAF_AIR_FLOW  0x10
//#define MAN_PRESSURE  0x0B
//#define INT_AIR_TEMP  0x0F
#define PID_SUPPORT20 0x20
#define PID_SUPPORT40 0x40
#define LAST_PID      0x4E  // same as the last one defined above
// returned length of the PID response.
// constants so put in flash
const unsigned char pid_reslen[] PROGMEM=
{
  // pid 0x00 to 0x1F
  4,4,2,2,1,1,1,1,1,1,1,1,2,1,1,1,
  2,1,1,1,2,2,2,2,2,2,2,2,1,1,1,4,

  // pid 0x20 to 0x3F
  4,2,2,2,4,4,4,4,4,4,4,4,1,1,1,1,
  1,2,2,1,4,4,4,4,4,4,4,4,2,2,2,2,

  // pid 0x40 to 0x4E
  4,8,2,2,2,1,1,1,1,1,1,1,1,2,2
};

typedef struct
{
  byte use_metric;     // 0=US units, 1=SI
  byte per_hour_speed; // speed from which we toggle to fuel/hour (km/h or mph)
  byte fuel_adjust;    // because of variation from car to car, temperature, etc
  byte speed_adjust;   // because of variation from car to car, tire size, etc
  byte eng_dis;        // engine displacement in dL
  unsigned int  tank_size;   // tank size in dL or dgal depending of unit
}
params_t;
// parameters default values
params_t params=
{
  0,
  20,
  100,
  100,
  16,
  700,
};

#define STRLEN  40

/*
 * for ISO9141-2 Protocol
 */
#define K_IN    0
#define K_OUT   1

// some globals, for trip calculation and others
unsigned long old_time;
long vss=0;  // speed
long rpm=0;  //engine rpm
long maf=0;  // MAF
long mpg=0; //miles per gallon
long runningAvgMpg=0; //running average of last two non zero values of instantaneous miles per gallon
unsigned int avgMpg=0; //average miles per gallon saved to EEPROM
//number of samples we will save to use in calculating average mpg value saved to EEPROM
unsigned int const num_elements = 400;
uint8_t mpgArr[num_elements];
unsigned int i=0; //iterator for adding to mpg array
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
const long interval = 1000; 
const long interval2 = 500;
const long interval3 = 240000; //4 minutes
unsigned long getpid_time;
byte nbpid_per_second=0;

// flag used to save distance/average consumption in eeprom only if required
bool engine_running = false;

void serial_rx_on() {
//  UCSR0B |= _BV(RXEN0);  //enable UART RX
  Serial.begin(10400);    //setting enable bit didn't work, so do beginSerial
}

void serial_rx_off() {
  UCSR0B &= ~(_BV(RXEN0));  //disable UART RX
}

void serial_tx_off() {
   UCSR0B &= ~(_BV(TXEN0));  //disable UART TX
   delay(20);                 //allow time for buffers to flush
}


int iso_read_byte()
{
  int b;
  byte t=0;
  while(t!=125  && (b=Serial.read())==-1) {
    delay(1);
    t++;
  }
  if (t>=125) {
    b = 0;
  }
  return b;
}

void iso_write_byte(byte b)
{
  serial_rx_off();
  Serial.write(b);
  delay(10);    // ISO requires 5-20 ms delay between bytes.
  serial_rx_on();
}

// inspired by SternOBDII\code\checksum.c
byte iso_checksum(byte *data, byte len)
{
  byte i;
  byte crc;
  crc=0;
  for(i=0; i<len; i++)
    crc=crc+data[i];
  return crc;
}

// inspired by SternOBDII\code\iso.c
byte iso_write_data(byte *data, byte len)
{
  byte i, n;
  byte buf[20];
  // ISO header
  buf[0]=0x68;
  buf[1]=0x6A;    // 0x68 0x6A is an OBD-II request
  buf[2]=0xF1;    // our requesters address (off-board tool)
  // append message
  for(i=0; i<len; i++)
    buf[i+3]=data[i];
  // calculate checksum
  i+=3;
  buf[i]=iso_checksum(buf, i);
  // send char one by one
  n=i+1;
  for(i=0; i<n; i++)
  {
    iso_write_byte(buf[i]);
  }
  return 0;
}

// read n byte of data (+ header + cmd and crc)
// return the result only in data
byte iso_read_data(byte *data, byte len)
{
  byte i;
  byte buf[20];

  // header 3 bytes: [80+datalen] [destination=f1] [source=01]
  // data 1+1+len bytes: [40+cmd0] [cmd1] [result0]
  // checksum 1 bytes: [sum(header)+sum(data)]

    for(i=0; i<3+1+1+1+len; i++)
    buf[i]=iso_read_byte();

  // test, skip header comparison
  // ignore failure for the moment (0x7f)
  // ignore crc for the moment

  // we send only one command, so result start at buf[4] Actually, result starts at buf[5], buf[4] is pid requested...
  memcpy(data, buf+5, len);

  delay(55);    //guarantee 55 ms pause between requests

  return len;
}

/* ISO 9141 init */
byte iso_init()
{
  first_attempt = false;
  byte b;
  byte kw1, kw2;
  serial_tx_off(); //disable UART so we can "bit-Bang" the slow init.
  serial_rx_off();
  delay(3000); //k line should be free of traffic for at least two secconds.
  // drive K line high for 300ms
  digitalWrite(K_OUT, HIGH);
  delay(300);

  // send 0x33 at 5 bauds
  // start bit
  digitalWrite(K_OUT, LOW);
  delay(200);
  // data
  b=0x33;
  for (byte mask = 0x01; mask; mask <<= 1)
  {
    if (b & mask) // choose bit
      digitalWrite(K_OUT, HIGH); // send 1
    else
      digitalWrite(K_OUT, LOW); // send 0
    delay(200);
  }
  // stop bit + 60 ms delay
  digitalWrite(K_OUT, HIGH);
  delay(260);

  // switch now to 10400 bauds
  Serial.begin(10400);

  // wait for 0x55 from the ECU (up to 300ms)
  //since our time out for reading is 125ms, we will try it three times
  for(int i=0; i<3; i++) {
    b=iso_read_byte(); 
    if(b!=0)
      break;
  }
  
  if(b!=0x55)
    return -1;

  // wait for kw1 and kw2
  kw1=iso_read_byte();

  kw2=iso_read_byte();
//  delay(25);

  // sent ~kw2 (invert of last keyword)
  iso_write_byte(~kw2);

  // ECU answer by 0xCC (~0x33)
  b=iso_read_byte();
  if(b!=0xCC)
    return -1;

  // init OK!
  //
  return 0;
}
//======================= check if PID is supported by ecu:  ==================================================
// return 0 if pid is not supported, 1 if it is.
//for example, call the function like this to check if your ecu supports fuel level PID:
//      bool supported = is_pid_supported(0x2F,1); 
//       lcd.print(supported); //my car prints 0 because it doesnt support that PID :(
// mode is 0 for get_pid() and 1 for menu config to allow pid > 0xF0
byte is_pid_supported(byte pid, byte mode)
{
  // note that pid PID_SUPPORT00 (0x00) is always supported
  if(  (pid>0x00 && pid<=0x20 && ( 1L<<(0x20-pid) & pid01to20_support ) == 0 )
    || (pid>0x20 && pid<=0x40 && ( 1L<<(0x40-pid) & pid21to40_support ) == 0 )
    || (pid>0x40 && pid<=0x60 && ( 1L<<(0x60-pid) & pid41to60_support ) == 0 )
    || (pid>LAST_PID && (pid<0xF0 || mode==0) )
    )
    {
      return 0;
    }

  return 1;
}

// get value of a PID, return as a long value
// and also formatted for string output in the return buffer
long get_pid(byte pid, char *retbuf)
{
  byte cmd[2];    // to send the command
  byte i;
  byte buf[10];   // to receive the result
  long ret;       // will be the return value
  byte reslen;
  char decs[16];
  unsigned long time_now, delta_time;
  static byte nbpid=0;

  nbpid++;
  // time elapsed
  time_now = millis();
  delta_time = time_now - getpid_time;
  if(delta_time>1000)
  {
    nbpid_per_second=nbpid;
    nbpid=0;
    getpid_time=time_now;
  }

  // check if PID is supported (should not happen except for some 0xFn)
  if(!is_pid_supported(pid, 0))
  {
    // nope   
    return -1;
  }

  // receive length depends on pid
  reslen=pgm_read_byte_near(pid_reslen+pid);

  cmd[0]=0x01;    // ISO cmd 1, get PID
  cmd[1]=pid;
  // send command, length 2
  iso_write_data(cmd, 2);
  // read requested length, n bytes received in buf
  iso_read_data(buf, reslen);
//#endif

  // a lot of formulas are the same so calculate a default return value here
  // even if it's scrapped after, we still saved 40 bytes!
  ret=buf[0]*256U+buf[1];

  // formula and unit for each PID
  switch(pid)
  {
  case ENGINE_RPM:
      ret=ret/4U;
      break;
  case MAF_AIR_FLOW:
    // ret is not divided by 100 for return value!!
    long_to_dec_str(ret, decs, 2);    
    break;
 case VEHICLE_SPEED:
 //if(!params.use_metric)
    //ret=(buf[0] * params.speed_adjust) / 100U;
   // else
   // this PID is returned in kmh so multiply by .62 to get mph
    ret=((buf[0] * params.speed_adjust) / 100U )* (0.621371);
//#endif
    break;

    // for the moment, everything else, display the raw answer
  default:
    // transform buffer to an hex value
    ret=0;
    for(i=0; i<reslen; i++)
    {
      ret*=256L;
      ret+=buf[i];
    }    
    break;
  }

  return ret;
}

// ex: get a long as 687 with prec 2 and output the string "6.87"
// precision is 1 or 2
void long_to_dec_str(long value, char *decs, byte prec)
{
  byte pos;

  pos=strlen(decs)+1;  // move the \0 too
  // a simple loop takes less space than memmove()
  for(byte i=0; i<=prec; i++)
  {
    decs[pos]=decs[pos-1];  // move digit
    pos--;
  }

  // then insert decimal separator
  decs[pos]=params.use_metric?',':'.';
}

void check_supported_pids(void)
{
  char str[STRLEN];

#ifdef DEBUG
  pid01to20_support=0xBE1FA812;
#else
  pid01to20_support=get_pid(PID_SUPPORT00, str);
#endif

  if(is_pid_supported(PID_SUPPORT20, 0))
    pid21to40_support=get_pid(PID_SUPPORT20, str);

  if(is_pid_supported(PID_SUPPORT40, 0))
    pid41to60_support=get_pid(PID_SUPPORT40, str);
}


/*
 * Initialization
 */

void setup()                    // run once, when the sketch starts
{
    ///////////for OLED display
// by default, we'll generate the high voltage from the 3.3v 
  lcd.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C 
  // init done 
  lcd.display();
   // Clear the buffer.
  lcd.clearDisplay(); 
  // text display tests
  lcd.setTextSize(1);
  lcd.setTextColor(WHITE);
  lcd.setCursor(0,0);  
  lcd.display();  
  lcd.clearDisplay();
  

/////////////////end OLED display
     
         
#ifndef ELM
  byte r;

  // init pinouts
  pinMode(K_OUT, OUTPUT);
  pinMode(K_IN, INPUT);
#endif

  do // init loop
  {
  lcd.clearDisplay();
  lcd.setCursor(0,0);
  lcd.print("ISO9141 Init");
    
    lcd.setCursor(0,20); //3rd line;
    if(params.use_metric) {
  lcd.print("SI units");
  } else { lcd.print("US units"); }
  
  lcd.display();
  if(first_attempt) {
    delay(5000);
  }
    r=iso_init();
      if(r==0)  {
      lcd.setCursor(0,10); //Move to second line
    lcd.println("Successful!");
    lcd.display();
    lcd.setTextSize(2);
      } else    { 
      lcd.setCursor(0,10); //Move to second line
      lcd.println("Failed"); 
      lcd.display(); 
      } 
    delay(1000);
  }
  while(r!=0); // end init loop

    //connected successfully so load the previously saved avgMpg
   // save_avgMpg();
     load_avgMpg();
     
  // check supported PIDs
  check_supported_pids();

  old_time=millis();  // epoch
  getpid_time=old_time;
}

/*
 * Main loop
 */

void loop()                     
{
  //constantly get vss
  get_vss();
  //constantly get rpm
  get_rpm();

  //only get mpg once per second and average it
  unsigned long currentMillis = millis();  
  if (currentMillis - previousMillis >= interval) {
    // save the last time you did it
    previousMillis = currentMillis;
    get_mpg();
    add_to_mpgArr();    
  }

  
//only update display 2 times per second
  if (currentMillis - previousMillis2 >= interval2) {
    // save the last time you did it
    previousMillis2 = currentMillis;
    print_data();
  }

}

/*
 * Data from ecu related functions
 */

void get_vss() {
  char str[STRLEN];
  vss = get_pid(VEHICLE_SPEED, str);
}

void get_rpm() {
  char str[STRLEN];
  rpm = get_pid(ENGINE_RPM, str);
  if( rpm > 0 ) { 
    engine_running = true; 
  } else {
    engine_running = false;
  }
}

void get_mpg() {
  long icons; //instant consumption
  char str[STRLEN];
  // MPG
//14.7 grams of air to 1 gram of gasoline - ideal air/fuel ratio
//6.17 pounds per gallon - density of gasoline
//4.54 grams per pound - conversion
//VSS - vehicle speed in kilometers per hour
//0.621371 miles per hour/kilometers per hour - conversion
//3600 seconds per hour - conversion
//MAF - mass air flow rate in 100 grams per second
//100 - to correct MAF to give grams per second
  
   maf = get_pid(MAF_AIR_FLOW, str);
 //  icons = (maf*3355)/(vss   
  //long mpg =  14.7 * 6.17 * 454 * (vss * 0.621371) / (3600 * maf / 100);
  //vss is already in US units
  mpg =  (14.7 * 6.17 * 454 * vss) / (3600 * maf / 100); 
  //the first time this runs or if you are accelerating from a stop, if the variable is 0 then give it the value of mpg
  if(runningAvgMpg == 0) { 
    runningAvgMpg = mpg; 
  } else {
   runningAvgMpg = (runningAvgMpg + mpg) / 2;   
    } 

 }

 void add_to_mpgArr() {
  //if the mpgArr isn't full yet, then add the latest mpg value to it
  if(i<num_elements) {
    //and we don't want the array full of 0s while we sit at a stop light
      if(mpg>0) {       
         mpgArr[i] = mpg;
         i++;       
      
       //if i is 50 or 100 or 150 etc.. update avgMpg but dont restart the data collection array
          if( i%50 == 0 ) {
            //we've driven a little bit so update avgMpg value for our display and save it to EEPROM but dont start the data array back to 0
            avgMpg = (sum_array()+avgMpg) / (i+1);
            //save it to EEPROM
            save_avgMpg();
          }
      }
  } else {
    //mpgArray is full so update avgMpg value for our display and save it to EEPROM
    avgMpg = (sum_array()+avgMpg) / (num_elements+1);
    //save it to EEPROM
    save_avgMpg();
    //start over with the first element of mpgArr to add new values
    i=0;
  }
 
 }

 int sum_array() {
  int j; int sum=0;
  int elems = sizeof(mpgArr)/sizeof(mpgArr[0]);

  for(j=0; j<elems; j++)
  {
    sum = sum + mpgArr[j];
  }
  return sum;
 }
   

void print_data() 
{
  
  lcd.clearDisplay();
  lcd.setCursor(0,1);
  lcd.print(vss);
  //lcd.print(load_avgMpg());
  lcd.print(" MPH");  
  //debugging
  lcd.setTextSize(1);
  lcd.print("    i:");
  lcd.print(i);
 // lcd.print(" v:");
 // lcd.print(mpgArr[i-1]);
  //lcd.print(" ");
  //lcd.print(mpg);
  lcd.setTextSize(2);
  lcd.setCursor(0,18); //Move to 2nd line
  lcd.print("MPG:");
  lcd.print(runningAvgMpg);     
  lcd.setTextSize(1);
  lcd.setCursor(100,15);
  lcd.print("Avg:");
  lcd.setCursor(105,25);
  lcd.print(avgMpg);
  lcd.setTextSize(2);
  lcd.display();
}

byte load_avgMpg() {
  byte address = 0;
  long two = EEPROM.read(address);
  long one = EEPROM.read(address + 1);

 int value = ((two << 0) & 0xFFFFFF) + ((one << 8) & 0xFFFFFFFF);  
 avgMpg = value;
 
}

void save_avgMpg() { 
  unsigned int value=avgMpg; 
  byte address = 0;
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);
  
  EEPROM.update(address, two);
  EEPROM.update(address + 1, one);
  EEPROM.end();
}
