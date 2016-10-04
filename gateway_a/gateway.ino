nclude <Adafruit_FONA.h>

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
#define failCountMax 6


char publicKey[]="88aKkPpBpdipE21jdQwEua2xpjl";
char privateKey[]= "yLe3bPKlKASGDwRlY6WDhb5DY7Z";


char sendBuffer[220];

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
  float         bmp_press;
  float         bmp_temp;
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
 float bmp_temp = 2.;
 float bmp_press = 2.;



 nodeId = theData.nodeId;
   packetCount= theData.packetCount;
   therm =  theData.therm;
   temp_s = theData.temp_s;
  humid_s =  theData.humid_s;
   lux = theData.lux;
   batt_v = theData.batt_v;
   sol_v = theData.sol_v;
   bmp_press = theData.bmp_press;
   bmp_temp =  theData.bmp_temp;

    

//  power_up_fona();

  int fonaStatus=initialize_fona();

  
  if (!fonaStatus) { 
     if (DEBUG) Serial.println("FONA not found");

//    power_down_fona();

//   if (!DEBUG) go_to_sleep_minutes(sleepMinutes); 
    
   if(DEBUG) delay(60000);
   
  }

  else{

  int networkStatus=fona_find_network();
  
  if ((networkStatus!=1)&&(networkStatus!=5)) {
     if (DEBUG) Serial.println("Couldn't find network in failCountMax tries. Aborting.");
   
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
uint16_t battLevel;
fona.getBattVoltage(&battLevel);

if (DEBUG) {Serial.print("batteryMV="); Serial.println(battLevel);}


// char conversions

        int           nodeId; //store this nodeId
  unsigned long packetCount; 
  float         therm;   //temperature maybe?
  float         temp_s;
  float         humid_s;
  float         lux;
   float batt_v = 2.;
 float sol_v = 1.;
 float bmp_temp = 2.;
 float bmp_press = 2.;
float a1 = 1.;
float a2 = 2.;
float a3 = 3.;

  char thermChar[10];
  dtostrf(therm, 3, 2, thermChar);

  char tempChar[10];
  dtostrf(temp_s,3,2,tempChar);

 char humidChar[10];
  dtostrf(humid_s,3,2,humidChar);

   char luxChar[10];
  dtostrf(lux,3,2,luxChar);
  
  char battChar[10];
  dtostrf(batt_v,3,3,battChar);

 char solChar[10];
  dtostrf(sol_v,3,3,solChar);

   char bmp_pressChar[10];
  dtostrf(bmp_press,3,3,bmp_pressChar);

   char bmp_tempChar[10];
  dtostrf(bmp_temp,3,3,bmp_tempChar);

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


    sprintf(sendBuffer,"AT+HTTPPARA=\"URL\",\"http://159.203.128.53/input/%s?private_key=%s&therm=%s&temp_s=%s&humid_s=%s&batt_v=%s&lux=%s&sol_v=%s&bmp_press=%s&bmp_temp=%s&sensorid=%s&packetcount=%s\"",publicKey,privateKey,thermChar,tempChar,humidChar,battChar,luxChar,solChar,bmp_pressChar,bmp_tempChar,idChar,packetCountChar);


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



