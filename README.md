# LoRaWan-Gateway Locator
This is an illustration of building a Coverage Measurement Equipment out of standard Arduino parts

The main problem was the "common" usage of pin-10 as ChipSelect (SS*) on the SPI bus.

This interferred between the Dragino board and the SD-Card.

Also to be changed had the Rx/Tx from the Adafruit GPS-Logger board from 7/8 to 4/5 not conflicting with the Dragino pins.

The LMIC 1.5 library used for this project uses itself 30kB of storage, nearly the full memory an UNO offers.

So I use a piggypack board, replacing the 328P-28DIL with a ATmega1284 which has 4 times the memory that an UNO has.


# BOM:
Adafruit Ultimate GPS Logger Shield  https://learn.adafruit.com/adafruit-ultimate-gps-logger-shield?view=all

Arduino UNO with 328P in DIL28 socket package!

HobbyTronics UNO*Pro http://www.hobbytronics.co.uk/arduino-uno-pro?keyword=uno%20pro

Dragino board Version 1.3 with RFM95W (marked as RFM96!) LoRaWan compatible radio

#Modifications:

all of them are made on the Adafruit GPS shield (see picture Mod_1)

1. cut at the TOP side the traces Tx, Rx & CCS

2. connect CCS -> 8, Tx -> 5, Rx -> 4

3. in the software use in the definitions

   #define SD_Select      8

   #define Drag_Select   10

   SoftwareSerial mySerial(5, 4);  // switch in position SW serial!

   Adafruit_GPS GPS(&mySerial);

   // Pin mapping  for Dragino Board Version 1.3

   const lmic_pinmap lmic_pins = {

   .nss = 10,

   .rxtx = LMIC_UNUSED_PIN,

   .rst = 9,

   .dio = {2, 6, 7},

   };


in setup() you have to configure the two different ChipSelect

  pinMode(SD_Select, OUTPUT);       digitalWrite(SD_Select, HIGH);  // DEselect 

  pinMode(Drag_Select, OUTPUT);  digitalWrite(Drag_Select, HIGH);   // DEselect
  

when writing to the SD card make shure that the Dragino is disabled

    digitalWrite(Drag_Select, HIGH);       // disable Dragino_Board_ChipSelect

    log_on_SD();                           // write a log to the SDcard

    digitalWrite(SD_Select, HIGH);         // disable SDcard_ChipSelect
  
  

