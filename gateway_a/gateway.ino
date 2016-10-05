#include <Adafruit_FONA.h>

#include <RFM69.h>
#include <SPI.h>
//#include <SPIFlash.h>

#define NODEID      1
#define NETWORKID   100
#define FREQUENCY   RF69_868MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define KEY         "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define LED         9
#define SERIAL_BAUD 115200
#define ACK_TIME    30  // # of ms to wait for an ack

RFM69 radio;
//SPIFlash flash(8, 0xEF30); //EF40 for 16mbit windbond chip
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

#define DEBUG 1

#define FONA_RX 6
#define FONA_TX 7
#define FONA_RST 4
#define FONA_KEY 8
#define FONA_POWER_STATUS A7
#define failCountMax 10


char publicKey[]="XZwaDOAYKYUMo39Yz8moHXAMw9e";
char privateKey[]= "LYXAyEWwnwI0NnqGxD9NI4ROzw0";


char sendBuffer[230];

#ifdef __AVR__
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
#else
HardwareSerial *fonaSerial = &Serial1;
#endif

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;

float rando;

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


int initialize_fona() {
  
  int fonaStatus;
  
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    // things are bad -- couldn't find FONA
    fonaStatus=0;
  }
  else {
    
  //things are good -- FONA OK
 fonaStatus=1;
  }
  
  return fonaStatus;
  
}

int fona_find_network() {
  
  uint8_t n = fona.getNetworkStatus();
int avail = fona.available();
int registerCountDelaySeconds=10;
int registerWaitTotalSeconds=0;

int failCount=0;

while ((n!=1)&&(n!=5)&&(failCount<failCountMax)) {
  
    
      if(DEBUG)  Serial.print(F("Network status ")); 
      if(DEBUG) Serial.println(n);   
      
      n = fona.getNetworkStatus();
      //avail = fona.available();
      registerWaitTotalSeconds+=registerCountDelaySeconds;
      delay(registerCountDelaySeconds*1000);
      failCount=failCount+1;
     
}

  
  if(DEBUG) Serial.print("Network registration time=");
  if(DEBUG) Serial.println(registerWaitTotalSeconds);
  

return n;

}

void power_to_fona(int onoff) { // 0 = off, 1 = on
  
     int fonaPower=digitalRead(FONA_POWER_STATUS);

    //Serial.print("FONA status is ");
     Serial.println(fonaPower);
     
     if (!onoff) { // opposite of what we want, so trigger

    //Serial.println("turning on ... ");
    
    digitalWrite(FONA_KEY, HIGH); //go back to non-trigger
    digitalWrite(FONA_KEY, LOW); //turn on the SMS subcircuit
    delay(2000); //so now it's on
    digitalWrite(FONA_KEY, HIGH); //go back to non-trigger      
    }
    else {
     // error! was already on for some reason -- do nothing
     }
}

/*
void power_down_fona() {
  
     int fonaPower=digitalRead(FONA_POWER_STATUS);

    // Serial.print("FONA status is ");
     //Serial.println(fonaPower);
     
     if (fonaPower) { // on, so power down

    //Serial.println("turning on ... ");
    
    digitalWrite(FONA_KEY, HIGH); //go back to non-trigger
    digitalWrite(FONA_KEY, LOW); //turn on the SMS subcircuit
    delay(2000); //so now it's on
    digitalWrite(FONA_KEY, HIGH); //go back to non-trigger      
    }
    else {
     // error! was already on for some reason -- do nothing
     }
}

*/

void setup() {


  pinMode(FONA_POWER_STATUS,INPUT);
  pinMode(FONA_KEY, OUTPUT);
  // make sure the FONA power cycling key is initially pulled 'high' -- i.e. not triggered
  digitalWrite(FONA_KEY, HIGH); //go back to non-trigger


  if (DEBUG) {
  while (!Serial);

  Serial.begin(SERIAL_BAUD);
  }





  
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  //radio.setHighPower(); //uncomment only for RFM69HW!
  radio.encrypt(KEY);
  radio.promiscuous(promiscuousMode);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  
  
  Serial.println(buff);



}

byte ackCount=0;

void loop() {  


  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    Serial.print(" [RX_RSSI:");Serial.print(radio.readRSSI());Serial.print("]");
    if (promiscuousMode)
    {
      Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }

    if (radio.DATALEN != sizeof(Payload))
      Serial.print("Invalid payload received, not matching Payload struct!");
    else
    {
      theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
    

// FONA


if (DEBUG) Serial.println("Fona ...");

        int           nodeId; //store this nodeId
  unsigned long packetCount; 
  float         therm;   //temperature maybe?
  float         temp_s;
  float         humid_s;
  float         lux;
   float batt_v = 2.;
 float sol_v = 1.;
 float x2 = 2.;
 float x1 = 2.;



 nodeId = theData.nodeId;
   packetCount= theData.packetCount;
   therm =  theData.therm;
   temp_s = theData.temp_s;
  humid_s =  theData.humid_s;
   lux = theData.lux;
   batt_v = theData.batt_v;
   sol_v = theData.sol_v;
   x1 = theData.x1;
   x2 =  theData.x2;

    

   power_to_fona(1);  // in case not powered

  int fonaStatus=initialize_fona();

  
  if (!fonaStatus) { 
     if (DEBUG) Serial.println("FONA not found");


   power_to_fona(0);  
//   power_to_fona(1);

//power_down_fona();

//  power_up_fona();

//   if (!DEBUG) go_to_sleep_minutes(sleepMinutes); 
    
   //if(DEBUG) delay(60000);
   
  }

  else{

  int networkStatus=fona_find_network();
  
  if ((networkStatus!=1)&&(networkStatus!=5)) {
     if (DEBUG) Serial.println("Couldn't find network in failCountMax tries. Aborting.");

power_to_fona(0);

 // power_up_fona();
  
  }
  
  if (((networkStatus==1)||(networkStatus==5))&&fonaStatus) {  //then we're good to send a message!

    if (DEBUG) Serial.println("POSTING ...");

  /*
  fona.sendCheckReply(F("AT"), F("OK"));
  delay(100);

  //fona.sendCheckReply(F("AT+SAPBR=3,1,\"APN\",\"internet.comcel.com.co\""), F("OK"));
  //delay(100);

  fona.sendCheckReply(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""), F("OK"));
  delay(100);
  
  fona.sendCheckReply(F("AT+SAPBR=1,1"),F("OK"));
  delay(100);

  fona.sendCheckReply(F("AT+CMGF=1"), F("OK"));
  delay(5000);
  
  fona.sendCheckReply(F("AT+CGATT=1"), F("OK"));
  delay(5000);

 if (DEBUG) Serial.println("getting location ...");

  uint16_t returncode;
  char gpsbuffer[120];

   fona.getGSMLoc(&returncode,gpsbuffer,120);

if (DEBUG) {
    Serial.print("gpsstring=");
  Serial.println(gpsbuffer);
}

  char *latChar = strtok(gpsbuffer, ",");
  char *lonChar = strtok(NULL,",");
//  lat = atof(latp);
 // lon = atof(lonp);

  char *dateChar = strtok(NULL,",");
  char *timeChar = strtok(NULL,",");

if (DEBUG) {
  Serial.print("date=");
  Serial.println(dateChar);
  Serial.print("time=");
  Serial.println(timeChar);
  
    
  Serial.print("lat=");
  Serial.println(latChar);
  Serial.print("lon=");
  Serial.println(lonChar);

  
  Serial.println("got location.");
}

*/

// get battery level
uint16_t f_batt;
fona.getBattVoltage(&f_batt);

if (DEBUG) {Serial.print("batteryMV="); Serial.println(f_batt);}


// char conversions

/*
        int           nodeId; //store this nodeId
  unsigned long packetCount; 
  float         therm;   //temperature maybe?
  float         temp_s;
  float         humid_s;
  float         lux;
   float batt_v = 2.;
 float sol_v = 1.;
 float x2 = 2.;
 float bmp_press = 2.;
*/

  char thermChar[10];
  dtostrf(therm, 3, 2, thermChar);

  char tempChar[10];
  dtostrf(temp_s,3,2,tempChar);

 char humidChar[10];
  dtostrf(humid_s,3,2,humidChar);


int f_temp_bits = analogRead(A0); // thermistor connected to A0
   char f_tempChar[10];
  dtostrf(f_temp_bits,3,2,f_tempChar);
  
  char battChar[10]; /// remote battery
  dtostrf(batt_v,3,3,battChar);

  char f_battChar[10]; // fona battery
  dtostrf(f_batt/1000.,3,3,f_battChar);

 char solChar[10];
  dtostrf(sol_v,3,3,solChar);

   char x1Char[10];
  dtostrf(x1,3,3,x1Char);

   char x2Char[10];
  dtostrf(x2,3,3,x2Char);

 //  char a1Char[10];
  //dtostrf(a1,3,3,a1Char);

  // char a2Char[10];
  //dtostrf(a2,3,3,a2Char);

   //char a3Char[10];
  //dtostrf(a3,3,3,a3Char);
  
 char idChar[3];
  dtostrf(nodeId,3,2,idChar);
 //char idChar[]="1";

char packetCountChar[10];
  dtostrf(packetCount,3,2,packetCountChar);


//fona.sendCheckReply(F("AT+SAPBR=3,1,\"APN\",\"internet.comcel.com.co\""), F("OK"));
 fona.sendCheckReply(F("AT+SAPBR=3,1,\"APN\",\"wholesale\""), F("OK"));

  delay(100);

  fona.sendCheckReply(F("AT+SAPBR=1,1"),F("OK"));
  delay(2000);
  
  fona.sendCheckReply(F("AT+CMGF=0"), F("OK"));
  delay(2000);

  fona.sendCheckReply(F("AT+HTTPTERM"), F("OK"));
  delay(2000);
  
  fona.sendCheckReply(F("AT+HTTPINIT"), F("OK"));
  delay(2000);

  fona.sendCheckReply(F("AT+HTTPPARA=\"CID\",1"), F("OK"));
  delay(2000);



// BEGIN post -- PHANT

  //fona.sendCheckReply(F("AT+HTTPINIT"), F("OK"));
  //delay(2000);

 // fona.sendCheckReply(F("AT+HTTPPARA=\"CID\",1"), F("OK"));
  //delay(2000);

  
   //sprintf(sendBuffer,"AT+HTTPPARA=\"URL\",\"http://159.203.128.53/input/%s?private_key=%s&hour=%s&lat=%s&lon=%s&date=%s&temp=%s&conduct=%s&ph=%s&do=%s&batt=%s&sensortype=%s&sensorid=%s\"",publicKey,privateKey,timeChar,latChar,lonChar,dateChar,tempChar,conductChar,phChar,doChar,battChar,sensorTypeChar,sensorIDChar);
  //sprintf(sendBuffer,"AT+HTTPPARA=\"URL\",\"http://159.203.128.53/input/%s?private_key=%s&therm=%s&temp=%s&humid=%s&batt=%s&lux=%s&sensorid=%s&packetcount=%s\"",publicKey,privateKey,thermChar,tempChar,humidChar,battChar,luxChar,idChar,packetCountChar);
  
  
  //sprintf(sendBuffer,"AT+HTTPPARA=\"URL\",\"http://159.203.128.53/input/QlbwjXBaPqf1lwg2rO0lfJgoBek?private_key=3zRgpd2Wq6irKN4v5yWKsl5GxO8&therm=20.00&temp=1.00&humid=1.00&batt=4.190&lux=3.00&sensorid=1&packetcount=61\"");


    sprintf(sendBuffer,"AT+HTTPPARA=\"URL\",\"http://159.203.128.53/input/%s?private_key=%s&therm=%s&temp_s=%s&humid_s=%s&batt_v=%s&f_batt=%s&sol_v=%s&f_temp_bits=%s&x1=%s&x2=%s&sensorid=%s&packetcount=%s\"",publicKey,privateKey,thermChar,tempChar,humidChar,battChar,f_battChar,solChar,f_tempChar,x1Char, x2Char,idChar,packetCountChar);


  fona.sendCheckReply(sendBuffer,"OK");


  //fona.sendCheckReply(F("AT+HTTPDATA=1000,5000"), F("OK"));
  fona.sendCheckReply(F("AT+HTTPDATA=0,1000"), F("OK"));
 
  delay(15000);
  
  fona.sendCheckReply(F("AT+HTTPACTION=1"), F("OK"));
  delay(20000);

  fona.sendCheckReply(F("AT+HTTPTERM"), F("OK"));

  delay(5000);
  
  
  fona.sendCheckReply(F("AT+SAPBR=0,1"), F("OK"));
  delay(10000);



  }

  }

    }
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    Serial.println();
//    Blink(LED,3);
  }

}



