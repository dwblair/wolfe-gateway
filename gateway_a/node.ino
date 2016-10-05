// Sample RFM69 sender/node sketch, with ACK and optional encryption, and Automatic Transmission Control
// Sends periodic messages of increasing length to gateway (id=1)
// It also looks for an onboard FLASH chip, if present
// RFM69 library and sample code by Felix Rusu - http://LowPowerLab.com/contact
// Copyright Felix Rusu (2015)

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <SHT1x.h>

#define RESOLUTION 1024
#define VBATPIN A1

#define SLEEPCYCLES 112   // at 8S ea

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        2    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     100  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ACK_TIME    30  // # of ms to wait for an ack


//******************************
// THERMISTOR
//**********************


// which analog pin to connect
#define THERMISTORPIN A0         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000  
#define RESOLUTION 1024

int samples[NUMSAMPLES];

//*********************************************************************************************

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define SERIAL_BAUD   115200


#define sht_dataPin  6
#define sht_clockPin 7
SHT1x sht1x(sht_dataPin, sht_clockPin);

int TRANSMITPERIOD = 300; //transmit a packet to gateway so often (in ms)
char payload[] = "123 ABCDEFGHIJKLMNOPQRSTUVWXYZ";
char buff[20];
byte sendSize=0;
boolean requestACK = false;
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif


/*
 SLEEP_15MS,
 SLEEP_30MS, 
  SLEEP_60MS,
  SLEEP_120MS,
  SLEEP_250MS,
  SLEEP_500MS,
  SLEEP_1S,
  SLEEP_2S,
  SLEEP_4S,
  SLEEP_8S,
  SLEEP_FOREVER
*/

typedef struct {
  int           nodeId; //store this nodeId
  unsigned long packetCount; 
  float         therm;   //temperature maybe?
  float         temp_s;
  float         humid_s;
  float         lux;
  float         batt_v;
  float         sol_v;
  float         x1;
  float         x2;

   
} Payload;
Payload theData;


void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  //radio.setFrequency(919000000); //set frequency to some custom frequency

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(-70);
#endif

  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

  if (flash.initialize())
  {
    Serial.print("SPI Flash Init OK ... UniqueID (MAC): ");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
  else
    Serial.println("SPI Flash MEM not found (is chip soldered?)...");

#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

long lastPeriod = 0;

unsigned long packetCount=0;

void loop() {

//*********
// thermistor
//***********

 uint8_t i;
  float average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
 
  //Serial.print("Average analog reading "); 
  //Serial.println(average);
 
  // convert the value to resistance
  average = (RESOLUTION-1) / average - 1;
  average = SERIESRESISTOR / average;
  //Serial.print("Thermistor resistance "); 
  //Serial.println(average);
 
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
  //Serial.print("Temperature "); 
  //Serial.print(steinhart);
  //Serial.println(" *C");


  
 float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= RESOLUTION; // convert to voltage
  //measuredvbat = 6;

  /*
// create the payload
StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["id"] = NODEID;
  //root["millis"] = millis();
  JsonObject& data = root.createNestedObject("data");
  data["vbat"] = double_with_n_digits(measuredvbat, 3);
  data["therm"] = double_with_n_digits(therm,2);
  data["temp_c"] = postCount;
  data["humid"] = postCount;
  

  //char buf[251];
  char buf[root.measureLength()+1]; // note sure why we need +1, but seems to work with the indices
  
  root.printTo(buf, sizeof(buf));
  buf[sizeof(buf)-1] = 0;

  sendSize=sizeof(buf);
  */

 //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.readRSSI());Serial.print("]");

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
      delay(10);
    }
    Blink(LED,5);
    Serial.println();
  }



 //float therm = 3.;
   float therm = steinhart;
   
 float temp_s = 20.;  // sht1x.readTemperatureC();
 float humid_s = 1.; // sht1x.readHumidity();
 
 float lux = (float) analogRead(A3);
 float batt_v = measuredvbat;
 float sol_v = (float) analogRead(A2)*3*3.3/RESOLUTION;
 float x1 = (float) analogRead(A6);
 float x2 = (float) analogRead(A7);


 Serial.print("batt_v="); Serial.println(batt_v);
 Serial.print("sol_v="); Serial.println(sol_v);
 
  //fill in the struct with new values
    theData.nodeId = NODEID;
    theData.packetCount = packetCount;
    theData.therm = therm;
    theData.temp_s = temp_s;
    theData.humid_s = humid_s;
    theData.lux = lux;
    theData.batt_v = batt_v;
    theData.sol_v = sol_v;
    theData.x1 = x1;
    theData.x2 = x2;
   
    
 Serial.print("Sending struct (");
    Serial.print(sizeof(theData));
    Serial.print(" bytes) ... ");
   
    /*
    if (radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData)))
      Serial.print(" ok!");
    else Serial.print(" nothing...");
    Serial.println();
    Blink(LED,3);
*/
    radio.send(GATEWAYID, (const void*)(&theData), sizeof(theData));
     Blink(LED,3);
    //sendSize=80;
    //radio.send(GATEWAYID,buf,sendSize);
    
    //sendSize = (sendSize + 1) % 31;
    //Serial.println();
    //Blink(LED,3);

packetCount++;

for (int i=0; i< SLEEPCYCLES; i++) {
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

}
