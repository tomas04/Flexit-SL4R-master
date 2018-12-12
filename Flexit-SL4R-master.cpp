/*
* Untested fork of https://github.com/Vongraven/Flexit-SL4R-master
* Sending data over MQTT using ESP8266 and MAX3485. Waiting for parts..
*/

#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

#define RXen    2
#define TXen    3
#define COM_VCC 6

const char* mqtt_server = "YOUR_MQTT_SERVER_ADDRESS";
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

const char* ClientName = "CS50-Slave";

const char* InTopicPreheatOnOff = "/CS50-Command/heat";
const char* InTopicFanLevel = "/CS50-Command/fan_level";
const char* InTopicHeatExchTemp = "/CS50-Command/temperature";

const char* OutTopicPreheatOnOff = "/CS50-Response/heat";
const char* OutTopicFanLevel = "/CS50-Response/fan_level";
const char* OutTopicHeatExchTemp = "/CS50-Response/temperature";
const char* OutTopicPreheatActive = "/CS50-Response/heater_active";

uint8_t commandBuffer[18] = {195, 4, 0, 199, 81, 193, 4, 8, 32, 15, 0, 'F', 'P', 4, 0, 'T' };
uint8_t processedData [4]   = {};
uint8_t latestSentData [4]   = {};
uint8_t rawData [25]   = {};

uint8_t* rawFanLevel       = &rawData[5];
uint8_t* rawPreheatOnOff   = &rawData[6];
uint8_t* rawHeatExchTemp   = &rawData[9];
uint8_t* rawPreheatActive1 = &rawData[10];
uint8_t* rawPreheatActive2 = &rawData[11];

uint8_t* fanLevel          = &processedData[0];
uint8_t* heatExchTemp      = &processedData[1];
uint8_t* preheatOnOff      = &processedData[2];
uint8_t* preheatActive     = &processedData[3];

uint8_t* sentFanLevel      = &latestSentData[0];
uint8_t* sentHeatExchTemp  = &latestSentData[1];
uint8_t* sentPreheatOnOff  = &latestSentData[2];
uint8_t* sentPreheatActive = &latestSentData[3];

SoftwareSerial serialPort(RXen, TXen);
WiFiClient wifiClient;
PubSubClient client(wifiClient);

void reconnect();
void callback(char*, byte*, unsigned int);
void createCommand(uint8_t, uint8_t);
void sendCommand();
void processFlexitData();
void updateFlexitData();
void updateMQTTServer();

void setup() {
  serialPort.begin(19200); 
  //start wifi subsystem
  WiFi.begin(ssid, password);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //attempt to connect to the WIFI network and then connect to the MQTT server
  reconnect();
  //wait a bit before continuing
  delay(2000);

  pinMode(RXen, OUTPUT);
  pinMode(TXen, OUTPUT);
  pinMode(COM_VCC, OUTPUT);
  
  digitalWrite(RXen, LOW);
  digitalWrite(TXen, LOW);
  digitalWrite(COM_VCC, LOW);
}

void loop() {
  //reconnect if connection is lost
  if (!client.connected() && WiFi.status() == 3) {reconnect();}

  //maintain MQTT connection
  client.loop();
  
  updateFlexitData();
  updateMQTTServer();
  //if received CS50 controller data is different than what has been sent to MQTT server, publish the data to the MQTT server
  
  
  //MUST delay to allow ESP8266 WIFI functions to run
  delay(10);
}

void updateMQTTServer() {
  if (fanLevel != sentFanLevel)
    if(client.publish(OutTopicFanLevel, fanLevel, 1))
      sentFanLevel = fanLevel;
  if (heatExchTemp != sentHeatExchTemp)
    if(client.publish(OutTopicHeatExchTemp, heatExchTemp, 1))
      sentHeatExchTemp = heatExchTemp;
  if (preheatOnOff != sentPreheatOnOff)
    if(client.publish(OutTopicPreheatOnOff, preheatOnOff, 1))
      sentPreheatOnOff = preheatOnOff;
  if (preheatActive != sentPreheatActive)
    if(client.publish(OutTopicPreheatActive, preheatActive, 1))
      sentPreheatActive = preheatActive;
}

void reconnect() {

  //attempt to connect to the wifi if connection is lost
  if(WiFi.status() != WL_CONNECTED)
    //loop while we wait for connection
    while (WiFi.status() != WL_CONNECTED) 
      delay(500);

  //make sure we are connected to WIFI before attemping to reconnect to MQTT
  if(WiFi.status() == WL_CONNECTED)
  // Loop until we're reconnected to the MQTT server
    while (!client.connected())
      // Generate client name based on MAC address and last 8 bits of microsecond counter
      //if connected, subscribe to the topic(s) we want to be notified about
      if (client.connect(ClientName)) {
        client.subscribe(InTopicPreheatOnOff);
        client.subscribe(InTopicFanLevel);
        client.subscribe(InTopicHeatExchTemp);
      }
}
          
/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
* CS50 control board repeats 16 lines of data over and over. For this purpose only line number 15 is needed. 
* Look for a specific combination of bytes to identify the correct line.
* When combination is matched, read the line into buffer rawData[]
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateFlexitData() {
  digitalWrite(COM_VCC, HIGH); // activate MAX485 
  digitalWrite(RXen, LOW);     // RX enable
  uint8_t Buffer[1000];
  
  for (int i=0; i<1000; ++i) {
    while (!serialPort.available()); 
    Buffer[i] = serialPort.read();
    if (Buffer[i]==22 && Buffer[i-2]==193 && Buffer[i-8]==195) {
      for (int i=0; i<25; ++i) {
        while (!serialPort.available()); 
        rawData[i] = serialPort.read();
      }
    break;
    }
    //if (i == 999) serialPort.println("StBuffer not updated"); 
  }

  digitalWrite(RXen, HIGH);   // RX disable
  digitalWrite(COM_VCC, LOW); // deactivate MAX485      
  while (serialPort.available()) serialPort.read();     // empty serialPort RX buffer
}  

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
* - Fan level has three steps. They are received as 17, 34 or 51. Divide by 17 to get 1, 2 or 3.
* - Preheat state is received as 0 (off) or 128 (on), and is translated to 0/1
* - Heat exchanger temperature is unchanged (15 - 25 degrees) 
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void processFlexitData(){
  if (*rawFanLevel == 17 ||*rawFanLevel == 34 ||*rawFanLevel == 51) {
    *fanLevel = *rawFanLevel / 17;
    commandBuffer[11] = *rawFanLevel;      
  }
      
  if (*rawPreheatOnOff == 128) {
    *preheatOnOff = 1;
    commandBuffer[12] = 128;  
  }
  else if (*rawPreheatOnOff == 0) {
    *preheatOnOff = commandBuffer[12] = *preheatActive = 0; 
  }
      
  if (*rawHeatExchTemp >= 15 && *rawHeatExchTemp <= 25) {
    *heatExchTemp = commandBuffer[15] = *rawHeatExchTemp;  
  }

  if (*rawPreheatActive1 > 10 && *preheatActive == 0 && *preheatOnOff == 1) { 
    *preheatActive = 1;
  }
      
  if (*rawPreheatActive2 < 100 && *preheatActive == 1 && *preheatOnOff == 1) {
    *preheatActive = 0;
  }
}



/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
* createCommand() takes two arguments, command and level
* cmd: which value in commandbuffer to change
*     fan level           - cmd=11
*     preheat on/off      - cmd=12
*     heat exchanger temp - cmd=15
* 
* lvl: which value to send    
*     fanlevel    1 > 2   - lvl=18    (17 +1)
*                   > 3   - lvl=35    (34 +1)
'                 3 > 2   - lvl=50    (51 -1)
'                   > 1   - lvl=33    (34 -1)

*     preheat         on  - lvl=128   
*                     off - lvl=0
*                     
*     heat exchanger temp - lvl=15-25
'
* Calculate and append check sum
*
' Important! execute updateFlexitData() and processFlexitData() before each command.   
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void createCommand(uint8_t cmd, uint8_t lvl) {                                    
  commandBuffer[cmd] = lvl;
  int sum1=0, sum2=0;
  for ( int i=5; i<((uint8_t)sizeof(commandBuffer)-2) ; ++i) {
    sum1 = sum1 + commandBuffer[i];
    sum2 = (sum2 + sum1);
  }
  commandBuffer[sizeof(commandBuffer)-2] = sum1%256;  
  commandBuffer[sizeof(commandBuffer)-1] = sum2%256;

  sendCommand();
}

void callback(char* topic, byte* payload, unsigned int length) {
  if(topic==InTopicPreheatOnOff)
  {
    if (payload[0]==0)
      createCommand(15,0);
    else if (payload[0]==1)
      createCommand(15,128);
    else
      //should this be placed another place? Is callback paused while running callback?
      client.print("heat_cmd_unknown");
  }
  else if(topic==InTopicFanLevel)
  {
    if (payload[0]>=0 && (char)payload[0]<=3)
      createCommand(11,payload[0]);
    else
      client.print("fan_cmd_unknown");
  }
  else if(topic==InTopicHeatExchTemp)
  {
      if (payload[0]>=15 && (char)payload[0]<=25)
        createCommand(15,payload[0]);
      else
        client.print("tmp_cmd_unknown");
  } else {
    //client.print("topic " + topic + " unknown");
  }
  //then back to loop, wait for command confirmation from updateFlexitData()
}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
* Transmission of the commands must be timed so they dont collide with data from the CS50 control board. 
* Find the length of the incomming line (value number 8 in each line), count the bytes until end of line is reached, jump in and send command   
*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
void sendCommand () {  
  digitalWrite(COM_VCC, HIGH);      // activate MAX485
  digitalWrite(RXen, LOW);      // RX enable
  uint8_t data;  
  int Length, repeats=0;
  do {
    while (!serialPort.available());
    data=serialPort.read();
    if (data == 195){
      while (!serialPort.available());
      data=serialPort.read();
      if (data == 1){
        for (int i=0; i<6; ++i) {
          while (!serialPort.available());
          serialPort.read();
        }
    
        while (!serialPort.available());    
        Length = serialPort.read()+2;
        if (Length > 3 && Length < 33) {
          for (int i=0; i<Length; ++i) {
            while (!serialPort.available());
            serialPort.read();
          }
          digitalWrite(TXen, LOW);      // TX enable
          delay(10);
          serialPort.write(commandBuffer, 18);     // transmit command
          serialPort.flush();
          digitalWrite(TXen, HIGH);     // TX disable
          serialPort.println("command sent");
          ++repeats;
        }
      }
    }   
  } while (repeats<5);
  digitalWrite(RXen, HIGH);     // RX disable
  digitalWrite(COM_VCC, LOW);     // deactivate MAX485  
}