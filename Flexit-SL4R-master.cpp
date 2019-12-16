/*
* Untested fork of https://github.com/Vongraven/Flexit-SL4R-master
* Sending data over MQTT using ESP8266 and MAX3485. Waiting for parts..
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "Timer.h"

#define RX D3
#define TX D4
#define RXen D5
#define TXen D6

#define WIFI_SSID "wireless_network_name"
#define WIFI_PASS "wireless_network_psw"
#define MQTT_HOST "mqtt_server_ip_address"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt_user"
#define MQTT_PASS "mqtt_password"

#define TOPSZ 60  // Max number of characters in topic string
#define MESSZ 240 // Max number of characters in JSON message string

#define MQTT_CLIENT_ID "FlexitPanel"

//MQTT command:
#define MQTT_COMMAND_CHANNEL "command/FlexitPanel/#"
//MQTT status: 
//heater element enabled or disabled
#define MQTT_STATUS_HEATER_ENABLE "state/FlexitPanel/heater_enable"
//fan level, 1 (low), 2 (medium), 3 (high)
#define MQTT_STATUS_FAN "state/FlexitPanel/fan_level"
//temperature
#define MQTT_STATUS_TEMP "state/FlexitPanel/temperature"
//heater state, (is the heating element active)
#define MQTT_STATUS_HEATER_STATE "state/FlexitPanel/heater_state"

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
uint8_t* preheatEnable      = &processedData[2];
uint8_t* preheatState     = &processedData[3];

uint8_t* sentFanLevel      = &latestSentData[0];
uint8_t* sentHeatExchTemp  = &latestSentData[1];
uint8_t* sentPreheatEnable  = &latestSentData[2];
uint8_t* sentPreheatState = &latestSentData[3];

//Serial Serial1(2, 3);
WiFiClient espClient;               // Wifi Client
PubSubClient mqttClient(espClient);   // MQTT Client
SoftwareSerial softSer(RX, TX);
Timer t;

void initWIFI();
void initMQTT();
void initOTA();
void reconnectMQTT();
void mqttDataCb(char* topic, byte* data, unsigned int data_le);
void publishState();
void reconnect();
void callback(char*, byte*, unsigned int);
void createCommand(uint8_t, uint8_t);
void sendCommand();
void processFlexitData();
void updateFlexitData();
void updateMQTTServer(bool);
void oneLoop();
void fiveLoop();
void tenLoop();

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("Booting");
  initWIFI();
  initOTA();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Init loop timers
  t.every(1000, tenLoop);
  t.every(5000, fiveLoop);
  t.every(10000, tenLoop);

  //Set RX Enable and TX enable to output pins
  pinMode(RXen, OUTPUT);
  pinMode(TXen, OUTPUT);
  
  //Set both pins active..?
  digitalWrite(RXen, LOW);
  digitalWrite(TXen, LOW);

  initMQTT();
}

void initMQTT() {
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  Serial.println("Attempting MQTT connection...");
  // Attempt to connect
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
    Serial.println("connected");
    mqttClient.setCallback(mqttDataCb);
    mqttClient.subscribe(MQTT_COMMAND_CHANNEL);
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
  }
}

void initOTA() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void initWIFI() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
}

void loop() {
  ArduinoOTA.handle();
  mqttClient.loop();
  t.update();
}

void tenLoop() {
  publishState();
}

void fiveLoop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
}

void oneLoop(){
  updateFlexitData();
}

void mqttDataCb(char* topic, byte* data, unsigned int data_len) {
  char svalue[MESSZ];
  char topicBuf[TOPSZ];
  char dataBuf[data_len+1];

  strncpy(topicBuf, topic, sizeof(topicBuf));
  memcpy(dataBuf, data, sizeof(dataBuf));
  dataBuf[sizeof(dataBuf)-1] = 0;

  snprintf_P(svalue, sizeof(svalue), PSTR("RSLT: Receive topic %s, data size %d, data %s"), topicBuf, data_len, dataBuf);
  Serial.println(svalue);

  // Extract command
  memmove(topicBuf, topicBuf+sizeof(MQTT_COMMAND_CHANNEL)-2, sizeof(topicBuf)-sizeof(MQTT_COMMAND_CHANNEL));

  int16_t payload = atoi(dataBuf);     // -32766 - 32767
  Serial.print("Received payload: ");
  Serial.println(payload);
  if (!strcmp(topicBuf, "heater_enable")) {
    if (payload==0)
      createCommand(15,0);
    else if (payload==1)
      createCommand(15,128);
    else
      Serial.print("heater_enable payload unknown");
  } else if (!strcmp(topicBuf, "fan_level")) {
    if (payload>=0 && (char)payload<=3)
      createCommand(11,payload);
    else
      Serial.print("fan_level payload unknown");
  } else if (!strcmp(topicBuf, "temperature")) {
    if (payload>=15 && (char)payload<=25)
        createCommand(15,payload);
    else
      Serial.print("temperature payload unknown");
  } else {
      Serial.write("unknown topic");
  }
}

void publishState() {
  char message[10];

  if (fanLevel != sentFanLevel) 
  {
    itoa(*fanLevel, message, 10);
    if(mqttClient.publish(MQTT_STATUS_FAN, message))
      sentFanLevel = fanLevel;
  }
  if (heatExchTemp != sentHeatExchTemp)
    {
      itoa(*heatExchTemp, message, 10);
      if(mqttClient.publish(MQTT_STATUS_TEMP, message))
        sentHeatExchTemp = heatExchTemp;
    }
  if (preheatEnable != sentPreheatEnable) {
    itoa(*preheatEnable, message, 10);
    if(mqttClient.publish(MQTT_STATUS_HEATER_ENABLE, message))
      sentPreheatEnable = preheatEnable;
  }
  if (preheatState != sentPreheatState) {
    itoa(*preheatState, message, 10);
    if(mqttClient.publish(MQTT_STATUS_HEATER_STATE, message))
      sentPreheatState = preheatState;
  }
}
          
/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
* CS50 control board repeats 16 lines of data over and over. For this purpose only line number 15 is needed. 
* Look for a specific combination of bytes to identify the correct line.
* When combination is matched, read the line into buffer rawData[]
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateFlexitData() {
  digitalWrite(RXen, LOW);     // RX enable
  uint8_t Buffer[1000];
  
  for (int i=0; i<1000; ++i) {
    while (!softSer.available()); 
    Buffer[i] = softSer.read();
    if (Buffer[i]==22 && Buffer[i-2]==193 && Buffer[i-8]==195) {
      for (int i=0; i<25; ++i) {
        while (!softSer.available()); 
        rawData[i] = softSer.read();
      }
    break;
    }
    //if (i == 999) Serial1.println("StBuffer not updated"); 
  }

  digitalWrite(RXen, HIGH);   // RX disable   
  while (softSer.available()) softSer.read();     // empty Serial1 RX buffer
  publishState();
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
    *preheatEnable = 1;
    commandBuffer[12] = 128;  
  }
  else if (*rawPreheatOnOff == 0) {
    *preheatEnable = commandBuffer[12] = *preheatState = 0; 
  }
      
  if (*rawHeatExchTemp >= 15 && *rawHeatExchTemp <= 25) {
    *heatExchTemp = commandBuffer[15] = *rawHeatExchTemp;  
  }

  if (*rawPreheatActive1 > 10 && *preheatState == 0 && *preheatEnable == 1) { 
    *preheatState = 1;
  }
      
  if (*rawPreheatActive2 < 100 && *preheatState == 1 && *preheatEnable == 1) {
    *preheatState = 0;
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
  Serial.println("Creating command");
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

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
* Transmission of the commands must be timed so they dont collide with data from the CS50 control board. 
* Find the length of the incomming line (value number 8 in each line), count the bytes until end of line is reached, jump in and send command   
*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
void sendCommand () {  
  Serial.println("Sending command");
  digitalWrite(RX, LOW);      // RX enable
  uint8_t data;  
  int Length, repeats=0;
  do {
    //pause until a command has been received
    while (!softSer.available()){
      Serial.println("Waiting for command from CS50");
      delay(500);
    }
    data=softSer.read();
    if (data == 195){
      //pause until a command has been received
      while (!softSer.available()){
        Serial.println("Waiting for NEW command from CS50");
        delay(500);
      }
      data=softSer.read();
      if (data == 1) {
        for (int i=0; i<6; ++i) {
          while (!softSer.available());
          softSer.read();
        }
    
        while (!softSer.available()){
          Serial.println("Waiting for another new command from CS50");
          delay(500);
        }
        Length = softSer.read()+2;
        if (Length > 3 && Length < 33) {
          for (int i=0; i<Length; ++i) {
            while (!softSer.available());
            softSer.read();
          }
          digitalWrite(TX, LOW);      // TX enable
          delay(10);
          softSer.write(commandBuffer, 18);     // transmit command
          softSer.flush();
          digitalWrite(TX, HIGH);     // TX disable
          softSer.println("command sent");
          ++repeats;
        }
      }
    }   
  } while (repeats<5);
  digitalWrite(RX, HIGH);     // RX disable
}