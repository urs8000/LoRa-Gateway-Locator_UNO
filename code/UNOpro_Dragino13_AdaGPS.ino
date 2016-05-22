/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello, world!", that
 * will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
 *  0.1% in g2). 
 *
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 *******************************************************************************/
/**************************************************************************************************************************
 
 
  follow the guidelines here  http://forum.thethingsnetwork.org/t/new-backend-how-to-connect/1983
  hoe to connect to the new backend
 
 
 **************************************************************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>



// LoRaWAN NwkSKey, network session key
// Get these keys from an activated device using: 
// ttnctl devices info <DEVICE> 
// 
// application "XXXXXXXXXXXX"
//
// ------------------------------------------ !! make a separate part for each device !! ----------------------------------------------------
static const u4_t DEVADDR = 0xXXXXXXXX; 
static const u1_t PROGMEM NWKSKEY[16] = { 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX }
static const u1_t PROGMEM APPSKEY[16] = { 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX };
// ------------------------------------------------------------------------------------------------------------------------------------------

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


int green  = 16;                         // CHANGE FOR UNO
int red    = 15;                         // just
int blue   = 17;                         // taken from Teensy3.2
int led13  = 13;
int number =  0;

#define SD_Select      8
#define Drag_Select   10

uint32_t timer = millis();    
String outbuf_s = "";

// restrict to channel_0 and SF7 if uncommented; otherwise all channels & SF12
  #define CHANNEL0

// enable debug statements to Serial Monitor if uncommented
  #define DEBUG1
  
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300;
long delayTime = (TX_INTERVAL * 1000);

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
  SoftwareSerial mySerial(5, 4);                            // CHANGED for Adafruit GPS Logger Shield
  Adafruit_GPS GPS(&mySerial);

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:
// Adafruit_GPS GPS(&Serial1);
// HardwareSerial mySerial = Serial1;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
  #define GPSECHO  false


//                                 1         2         3         4         5
//                        12345678901234567890123456789012345678901234567890
uint8_t  mydata[51]    = "                                                  ";
uint8_t  outbuffer[51] = "                                                 .";

char lat_buf[10];
char long_buf[10];
char Alti_buf[6];
char HDOP_buf[4];
long counter = 0;

static osjob_t sendjob;
File logfile;

// Pin mapping  for Dragino Board Version 1.3
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

// -------------------------------------------------------------------------------------
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

// -------------------------------------------------------------------------------------
void do_send(osjob_t* j){
      Serial.print("Time: ");
      Serial.println(millis() / 1000);
      // Show TX channel (channel numbers are local to LMIC)
      Serial.print("Send, txCnhl: ");
      Serial.println(LMIC.txChnl);
      Serial.print("Opmode check: ");
      // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      Serial.println("OP_TXRXPEND, not sending");
      blink(red,1);
    } else {
      Serial.println("ok");
      // get data
      // measure_and_copy();
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    }
    // Schedule a timed job to run at the given timestamp (absolute system time)
    // os_setTimedCallback(j, os_getTime()+sec2osticks(120), do_send);
    Serial.println();    
 }
 
// -------------------------------------------------------------------------------------
void blink(int pin, int number) {
  for (int x=0; x<=number; x++) {
    digitalWrite(pin, HIGH);
    delay(100);
    digitalWrite(pin, LOW);
  }
}

void allOff() {
  digitalWrite(red,   LOW); 
  digitalWrite(green, LOW);
  digitalWrite(blue,  LOW);
}

// -------------------------------------------------------------------------------------
void getPosition()  {
  boolean leave = false;
  while ( !leave ) {

   char c = GPS.read();
   // if you want to debug, this is a good time to do it!
    if ((c) && (GPSECHO))
       Serial.write(c);    
   
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
         break;      // we can fail to parse a sentence in which case we should just wait for another
  }

   if (timer > millis())  timer = millis();
   if (millis() - timer > 5000) {
    timer = millis(); // reset the timer
    if (GPS.fix) {                                   //   Serial.println("!");
      leave = true; String TempStr = "";
      counter++;  if (counter >= 10000) counter = 1;
         TempStr = String(GPS.year);
         String year_s   = zweistellig(TempStr);     // Serial.print(year_s);
         TempStr = String(GPS.month);
         String month_s  = zweistellig(TempStr);     // Serial.print(month_s);
         TempStr = String(GPS.day);
         String day_s    = zweistellig(TempStr);     // Serial.print(day_s);      
         TempStr = String(GPS.hour);
         String hour_s   = zweistellig(TempStr);     // Serial.print(hour_s);  // UTC! 
         TempStr = String(GPS.minute);
         String minute_s = zweistellig(TempStr);     // Serial.print(minute_s); 
                                                     // Serial.println();

      // dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);   ....4725.778
      dtostrf(GPS.latitude,9,3,lat_buf);
      // Serial.println(lat_buf);

      dtostrf(GPS.longitude,9,3,long_buf);
      // Serial.println(long_buf);
      
      String cnt;
      TempStr = String(counter);
      cnt = vierstellig(TempStr);
      dtostrf(GPS.altitude,5,0,Alti_buf);
      dtostrf(GPS.HDOP,3,0,HDOP_buf);
      
      //
      // later on you should compress your data and send only relevant information to keep airtime short
      //
      outbuf_s = cnt + " 20" + year_s + month_s + day_s + " " + hour_s + ":" + minute_s + lat_buf + long_buf + Alti_buf + HDOP_buf;
       Serial.println(outbuf_s);

      if (GPS.year != 16 && GPS.latitude < 4000.0) {
        leave = false;
        blink(red,1);
      }
      
    }
  }
}
}


// -------------------------------------------------------------------------------------
void measure_and_copy() {

  timer = millis();
  getPosition();
  
  // fill outbuffer mydata with space
  for (int r=0; r!=51; r++) {
    mydata[r]    = 0x20;
  }

  // copy DATE & POSITION into sendbuffer
  for (int r=0; r!=outbuf_s.length(); r++)  {
    mydata[r] = outbuf_s[r];
  }

   #ifdef DEBUG1
     // int q = sizeof(mydata);
     // for (int x=0; x!=q; x++) {
     //   Serial.print(mydata[x], HEX);
     // }
     // Serial.println();  
     Serial.print("Length: ");
     Serial.print(sizeof(mydata));
     Serial.print("  >");
     for (int q=0; q!=51; q++)  {
       Serial.print(char(mydata[q]));              //
     }
     Serial.print("<");
     Serial.println();
   #endif 
  
}

void log_on_SD()  {

  logfile.println(outbuf_s);
  logfile.flush();

} 

// ---------------------------------------------------------------------------------------------------
String zweistellig(String input) {  // to enhance date/time string from single to double character
  String result = "";
  if (input.length() == 1)  {
    result += "0";
    result += input;
  }
  else result = input;

  return result;
}

String vierstellig(String input) {  // to enhance date/time string from single to double character
  String result = "";               // umschreiben auf case, switch
  if (input.length() == 1)  {
    result += "000";
    result += input;
  } else {
    if (input.length() == 2) {
      result += "00";
      result += input;
    } else {
      if (input.length() == 3) {
        result += "0";
        result += input;
      } else {
        if (input.length() == 4) {
          result = input;
        }
      }
    }
   }    
  return result;
}


// -------------------------------------------------------------------------------------
void setup() {
  pinMode(red,   OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue,  OUTPUT);
   
  pinMode(SD_Select, OUTPUT);      digitalWrite(SD_Select, HIGH);
  pinMode(Drag_Select, OUTPUT);    digitalWrite(Drag_Select, HIGH);  
 
  digitalWrite(blue,HIGH);  //show that initialization begins
  delay(500);
  digitalWrite(blue,LOW);  //show that initialization begins

  Serial.begin(115200);
  Serial.println("Starting");       // write it to the console

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_Select, 11, 12, 13)) {
    // if (!SD.begin(SD_Select)) {            // if you're using an UNO, you can use this line instead
    Serial.println("Card init. failed!");
    blink(red,2);
  }
  char filename[15];
  strcpy(filename, "LOG00000.TXT");
  for (uint8_t i = 0; i < 1000; i++) {
    filename[5] = '0' + i/100;
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    blink(red,3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  delay(5000);

  // LMIC init
   #ifdef DEBUG1
     Serial.println("entering os_init");
   #endif
  
  os_init();

    #ifdef DEBUG1
       Serial.println("os_init passed");
    #endif

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

    #ifdef DEBUG1
     Serial.println("LMIC_reset passed");
    #endif
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
 
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // restrict to channel 0 * 
    #ifdef CHANNEL0
      LMIC_disableChannel(1);
      LMIC_disableChannel(2);
      LMIC_disableChannel(3);
      LMIC_disableChannel(4);
      LMIC_disableChannel(5);
      LMIC_disableChannel(6);
      LMIC_disableChannel(7);
      LMIC_disableChannel(8);
      Serial.println("---> ONLY channel 0 is active");
    #endif
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
  
  //
  Serial.flush();
   blink(red,1);
   blink(green,1);
   blink(blue,1);
   allOff();
  timer = millis(); // reset the timer
}

// -----------------------------------------------------------------------------------------
void loop() {

   char c = GPS.read();                  // the GPS must be read continiously
   if ((c) && (GPSECHO))                 // if you want to debug, this is a good time to do it!
        Serial.write(c);    
    if (GPS.newNMEAreceived()) {         // otherwise the data will not be updated properly
       if (!GPS.parse(GPS.lastNMEA()))   // recognized with date/time
        return;  
    }
 
  if (timer > millis())  timer = millis();
  if (millis() - timer > delayTime) {
    timer = millis(); // reset the timer

    measure_and_copy();
    
    digitalWrite(Drag_Select, HIGH);       // disable Dragino_Board_ChipSelect
    log_on_SD();                           // write a log to the SDcard
    digitalWrite(SD_Select, HIGH);            // disable SDcard_ChipSelect

    do_send(&sendjob);

  }
    os_runloop_once();

}

/*

//              0        1         2         3         4         5
//              123456789012345678901234567890123456789012345678901
// Length: 51  >20160520 14:57 4725.768  832.463  466  1           <

$GPGGA,142658.000,4725.7757,N,00832.4599,E,1,9,0.96,458.1,M,48.0,M,,*5C
$GPRMC,142658.000,A,4725.7757,N,00832.4599,E,0.35,127.56,200516,,,A*6D

*/