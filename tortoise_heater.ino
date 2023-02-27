
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include "LittleFS.h"
#include <NTPClient.h>  //
#include <PubSubClient.h>
#include <ESP8266WebServer.h>
#include <Adafruit_Sensor.h>
#include <WiFiUdp.h>
//#include <ESP8266WebServer.h>
#include <OneWire.h>
//#include <iostream>
//#include <string.h> branch 
//#include <stdio.h>
//#include <cstdint>
#define Relay_Pin D5  //active board
#define LED_Pin 13    //on board LED_Pin
//#define LED_Pin D6//LED_Pin  //change when debuged
OneWire ds(D7);  //active board  // on pin 10 (a 4.7K resistor is necessary)

byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];
float celsius;
float s1;
float s2;
float s3;
float sVal1;
float sVal2;
float sVal3;
int adr;
int targetTemp;  //is set by DayHighTemp and NightHighTemp
uint_fast8_t DayHighTemp;    //is set by the sliders
uint_fast8_t NightHighTemp;  //is set by the sliders
//uint_fast8_t CompTemp;
uint_fast8_t Day;
uint_fast8_t Hours;
uint_fast8_t Minutes;
uint_fast8_t seconds;
uint_fast8_t Day_Hours;
uint_fast8_t Day_Minutes;
//uint_fast8_t Day_Settings;
uint_fast8_t Day_seconds;
uint_fast8_t Night_Hours;
uint_fast8_t Night_Minutes;
//uint_fast8_t Night_Settngs;
uint_fast8_t Night_seconds;
uint_fast8_t temp;
bool Am;
bool Reset =false;  //set when slider is moved
bool StartUp = 0;
bool Timer = 1;


/********************************************
 *    settup the time variables start       *
 * ******************************************/
const long utcOffsetInSeconds = 3600;

char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
char DayName[10];
char sensor[50];
char destination[50];
char AmType[2];
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

unsigned long currentMillis = millis();
unsigned long lastMillis = 0;
unsigned long previousMillis = 0;
const long interval = 5000;
/********************************************
 *    settup the time variables end         *
 * ******************************************/
// void readData(uint_fast8_t Day_Hours, uint_fast8_t Day_Minuts, uint_fast8_t DayHighTemp);
// void writeData(int dayHours, int dayMinuts, int dayTemp);
/********************************************
 *    wifi and pubSup credentials start     *
 * ******************************************/

const char* ssid = "Gimp";
const char* password = "FC7KUNPX";
const char* mqtt_server = "test.mosquitto.org";

WiFiClient Temp_Control;
PubSubClient client(Temp_Control);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
long int value = 0;

/********************************************
 *    wifi and pubSup credentials end       *
 * ******************************************/

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

/********************************************
 *       connect to the internet end        *
 * ******************************************/

/********************************************
 *              Callback start             *
 * ******************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("...");
  Serial.println("** 148 ** CALLBACK");
     Serial.print("Hours =  ");
     Serial.println(Hours);
      Serial.print(". Minutes = ");
      Serial.println(Minutes);



  for (unsigned int i = 0; i < length; i++) {
  }

  if ((char)payload[0] == 'N') {
    Reset = true;
    Serial.print("** 161 ** Reset =  ");
      Serial.println(Reset);    
  }

  if ((char)payload[0] == 'F') {
    Reset = false;
     Serial.print("** 167 ** Reset =  ");
      Serial.println(Reset);
  }

  if (Reset == true) {
    if (strstr(topic, "day_h")) {
      sscanf((char*)payload, "%02d", &Day_Hours);//if topic = day_h then Day_Hours = payload
      Serial.println("...");
      Serial.print("** 176 ** callBack ** Day_Hours =  ");
      Serial.println(Day_Hours);
    }
    if (strstr(topic, "day_m")) {
      sscanf((char*)payload, "%02d", &Day_Minutes);//if topic = day_m then Day_Minutes = payload
      Serial.println("...");
      Serial.print("** 183 ** Day_Minutes =  ");
      Serial.println(Day_Minutes);
    }
    if (strstr(topic, "day_temp")) {
      sscanf((char*)payload, "%02d", &DayHighTemp);//if topic = day_temp then DayHighTemp = payload
      Serial.println("...");
      Serial.print("** 190 ** DayHighTemp =  ");
      Serial.print(DayHighTemp);

    }

    if (strstr(topic, "night_h")) {
      sscanf((char*)payload, "%02d", &Night_Hours);//if topic = night_h then Night_Hours = payload
      Serial.println("...");
      Serial.print("** 199 ** Night_Hours =  ");
      Serial.println(Night_Hours);
    }

    if (strstr(topic, "night_m")) {
      sscanf((char*)payload, "%02d", &Night_Minutes);//if topic = night_m then Night_Minutes = payload
      Serial.println("...");
      Serial.print("** 207 ** Night_Minutes =  ");
      Serial.println(Night_Minutes);
    }

      if (strstr(topic, "night_temp")) {
        sscanf((char*)payload, "%02d", &NightHighTemp);//if topic = night_temp then night_temp = payload
        Serial.println("...");
        Serial.print("** 215 **NightHighTemp = ");
        Serial.println(NightHighTemp);
      }
/*****************************************
* don't really need this section.        *
******************************************/
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
      Serial.println("...");
      Serial.print("** 225 ** payload[0] =  ");
      Serial.println(payload[0]);
      digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on (Note that LOW is the voltage level
      // but actually the LED is on; this is because
      // it is active low on the ESP-01)
    } else if ((char)payload[0] == '0') {
      digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    }
  }
}
  /********************************************
 *               Callback end               *
 * ******************************************/

  void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      // Create a random client ID
      String clientId = "ESP8266Client";
      clientId += String(random(0xffff), HEX);
      // Attempt to connect
      if (client.connect(clientId.c_str())) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        client.publish("Temp_Control/pub", "hello world*****");
        // ... and resubscribe
        client.subscribe("Temp_Control/sub");
        client.subscribe("Temp_Control/reset");
        client.subscribe("day_h");
         client.subscribe("day_m");
        client.subscribe("day_temp");
        client.subscribe("night_h");
        client.subscribe("night_m");
        client.subscribe("night_temp");
        client.subscribe("TargetTemp");
      } else {
        Serial.print("failed, rc= ");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
  }

  /********************************************
 *     set the time and the temperature     *
 *                   start                  *
 * ******************************************/
  /*
void setTimeAndTemp()
{
Serial.print(" set time and temp (char)payload[i]  = ");
   Serial.println((char)payload[i]);
    
}*/

  /********************************************
 *     set the time and the temperature     *
 *                   start                  *
 * ******************************************/

  /*****************************************
  * this is going to be changed to a loop *
  * with float myFloat being assigned     *
  *  in void relay_Control()              *
  * ***************************************/


  /********************************************
 *        send temperature value            *
 *            to server for                 *                         
 *         temperature monitor              *
 *               to receive                 *
 *                 start                    *
 * ******************************************/
  void convertor(void) {
    char sensVal[50];
    float myFloat1 = s1;
    sprintf(sensVal, "%f", myFloat1);
    client.publish("heater", sensVal);
    float mysensor2 = s2;
    sprintf(sensVal, "%f", mysensor2);
    client.publish("coolSide", sensVal);
    float myFloat3 = s3;
    sprintf(sensVal, "%f", myFloat3);
    client.publish("outSide", sensVal);
    Serial.println("...");
  }
  

  /********************************************
 *        send temperature value            *
 *            to server for                 *                         
 *         temperature monitor              *
 *               to receive                 *
 *                  end                     *
 * ******************************************/
  /*************************************************************
 *                           Relay Control                    *
 *                                start                       *
 *************************************************************/

  void relay_Control() {
    currentMillis = millis();
    convertor();
    Serial.println("...");
        Serial.print("** 336 ** Am  = ");
        Serial.println(Am);
    if (Am == true) {
      Serial.println("...");
      Serial.print("** 341 ** targetTemp  = ");
        Serial.println(targetTemp);
      if (s1 < targetTemp) {
        digitalWrite(Relay_Pin, HIGH);
        digitalWrite(LED_Pin, HIGH);  //LED_Pin on
        if (s1 < targetTemp - 2) {

          if (Timer == 1) {
            previousMillis = currentMillis;
            Timer = 0;
          }
          if (currentMillis - previousMillis >= interval) {
            Timer = 1;
          }
        }
        Serial.println("...");
        Serial.print("** 357 ** s2  = ");
        Serial.println(s2);
      } else if (s1 > targetTemp) {
        digitalWrite(Relay_Pin, LOW);
        digitalWrite(LED_Pin, LOW);  //LED_Pin off
      }
      Serial.println("...");
        Serial.print("** 361 ** Day_Time = ");
        Serial.print(Day_Hours);
        Serial.print(" : ");
        Serial.println(Day_Minutes);
        Serial.print("** 565 ** DayHighTemp = ");
        Serial.println(DayHighTemp);
    }

    if (Am == false) {
      Serial.println("...");
       Serial.print("** 367 ** targetTemp  = ");
        Serial.println(targetTemp);
      if (s1 < targetTemp) {
        digitalWrite(Relay_Pin, HIGH);
        // digitalWrite (LED_Pin, HIGH);//LED_Pin on
        digitalWrite(LED_Pin, LOW);  //builtin LED_Pin on

        if (s1 < targetTemp - 2) {
          if (Timer == 1) {
            previousMillis = currentMillis;
            Timer = 0;
          }
          if (currentMillis - previousMillis >= interval) {
            Timer = 1;
          }
        }
      } else if (s1 > targetTemp) {
        digitalWrite(Relay_Pin, LOW);
        digitalWrite(LED_Pin, LOW);  //LED_Pin off
      }
       Serial.println("...");
        Serial.print("** 392 ** Night_Time = ");
        Serial.print(Night_Hours);
        Serial.print(" : ");
        Serial.println(Night_Minutes);
        Serial.print("** 596 ** Night_Temp = ");
        Serial.println(NightHighTemp);
    }
  }
  /*************************************************************
 *                           Relay Control                    *
 *                                End                         *
 *************************************************************/

  /*************************************************************
 *                           Sensor reading                   *
 *                                Start                       *
 *************************************************************/


  void sendSensor() {
    // if (StartUp == 0) {
    //    Serial.println("startup = ");
    //   Reset = false;
    //   StartUp = 1;
    // }
    /**************************
    *    DS18B20 Sensor      * 
    *      Starts Here       *
    **************************/


    if (!ds.search(addr)) {
      ds.reset_search();
      delay(250);
      Serial.print("**395 ** if (!ds.search(addr))");
      return;
    }
    for (i = 0; i < 8; i++) {  //we need to drop 8 bytes of data
    }
    adr = (addr[7]);

    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
    }
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);  // start conversion, with parasite power on at the end

    delay(1000);  // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);  // Read Scratchpad


    for (i = 0; i < 9; i++) {  // we need 9 bytes to drop off
      data[i] = ds.read();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3;  // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;       // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3;  // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1;  // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    /**************************
    *    DS18B20 Sensor       * 
    *      Ends Here          *
    **************************/

    /*************************************************************
 *                           Heater Control                  *
 *                                start                      *
 ************************************************************/

    celsius = (float)raw / 16.0;
    //if(adr == 228)  {        //inside board out side dial
    //if(adr == 89)  {        //outside board out side dial
    if (adr == 49) {  //test rig board out side dial
      s1 = (celsius);
Serial.print("** 476 ** s1 = ");
      Serial.println(s1);      
    }
    //if(adr == 197)  {        //inside boade Heater dial
    //if(adr == 96)  {        //out side board  Heater dial
    if (adr == 59) {   //out side board  Heater dial
      s2 = (celsius);  //change celsius to fahrenheit if you prefer output in Fahrenheit;
      Serial.println("...");
      Serial.print("** 483 ** s2 = ");
      Serial.println(s2);
      if (Am == true) {
        if (Day_Hours == Hours) {  //set targetTemp for the Night time setting
          Serial.println("...");
        Serial.print("** 488 ** Day_Hours = ");
      Serial.println(Day_Hours);
          if (Day_Minutes >= Minutes && Day_Minutes <= Minutes) {
                  targetTemp= DayHighTemp;
                  Serial.println("...");
            Serial.print("** 494 ** Day_Minutes = ");
      Serial.println(Day_Minutes);;
      Serial.println("...");
             Serial.print("** 497  ** targetTemp = ");
      Serial.println(targetTemp);

             }
        }
      }
      if (Am == false) {
        if (Night_Hours == Hours) {  //set targetTemp for the Night time setting
          Serial.println("...");
        Serial.print("** 505 ** Night_Hours = ");
      Serial.println(Night_Hours);
          if (Night_Minutes >= Minutes && Night_Minutes <= Minutes + 5) {
            Serial.println("...");
      Serial.print("** 510 ** Night_Minutes = ");
      Serial.println(Night_Minutes);
            targetTemp = NightHighTemp;
            Serial.println("...");
            Serial.print("** 514 ** targetTemp = ");
      Serial.println(targetTemp);
        }
      }
    }
    }
    //if(adr == 92)  {    //inside board inSide dial
    //if(adr == 116)  {    // outside board inSide dial
    if (adr == 181) {  // Test rig board inSide dial
      s3 = (celsius);
      Serial.println("...");
      Serial.print("** 524 ** s3 = ");
      Serial.println(s3);
    }
    // readData(Day_Hours,Day_Minutes,DayHighTemp);
   Serial.println("...");
        Serial.print("** 537 ** Day_Time = ");
        Serial.print(Day_Hours);
        Serial.print(" : ");
        Serial.println(Day_Minutes);
        Serial.print("** 542 ** DayHighTemp = ");
        Serial.println(DayHighTemp);

  // writeData(Day_Hours,Day_Minutes,DayHighTemp);
    char buf[2];

    client.publish("Temp_Control/setTemp", buf);
    relay_Control();  //call relay_Control function
    }
/*************************************************************
 *                           Heater Control                   *
 *                                  End                       *
 *************************************************************/


  /*************************************************************
 *                           Sensor reading                   *
 *                                End                         *
 *************************************************************/




 /**************************************************************
 *                  read data from LittleSF.                   *
 *.                             start.                         *
//  **************************************************************/

//  void readData(uint_fast8_t Day_Hours, uint_fast8_t Day_Minuts, uint_fast8_t DayHighTemp)
// {

//   // Open the file

//   File file = LittleFS.open("/SavedFile.txt", "r");
// delay(1000);
//   // Check if the file exists

//   if (!file)
//   {

//     // Read the file data and display it on LCD

//     Serial.println("No Saved Data!");

//     Serial.print("No Saved Data!");

//     return;
//   }

//   Serial.print("Saved Data :");

//   // set cursor to first column, second row

//   while (file.available())
//   {

//     // ;
//    temp = (file.read());
//     // // temp = temp - 48;
//     Serial.print("\\\\ 555 \\\\ temp = ");
//     Serial.println(temp);
//   }
//     Serial.print("dayHours = ");
//   Serial.println(Day_Hours);
//   Serial.print("dayMinuts = ");
//   Serial.println(Day_Minutes);
//   Serial.print("DayHighTemp = ");
//   Serial.println(DayHighTemp);
// delay(500);
//   // Close the file

//   file.close();
// }

 /**************************************************************
 *                  read data from LittleSF.                   *
 *.                           end                              *
 **************************************************************/

 /**************************************************************
 *                   write data to LittleSF.                   *
 *.                          start                             *
 **************************************************************/

//  void writeData(uint_fast8_t Day_Hours, uint_fast8_t Day_Minuts, uint_fast8_t DayHighTemp)
// {
// // readData(Day_Hours,Day_Minutes,DayHighTemp);
//         Serial.print("..... 574 .... Day_Hours = ");
//         Serial.println(Day_Hours);
//         Serial.print(".... 576 .... Day_Minutes = ");
//         Serial.println(Day_Minutes);
//         Serial.print(".... 578 .... DayHighTemp = ");
//         Serial.println(DayHighTemp);
//   // Open the file

//   File file = LittleFS.open("/SavedFile.txt", "w");
// delay(1000);
//   // Write to the file

//   file.print(Day_Hours);
//   file.print(Day_Minutes);
//   file.print(DayHighTemp);

//   // Close the file

//   file.close();

//   Serial.println("Write successful");


//   Serial.print("Data Saved :");
// }

 /**************************************************************
 *                   write data to LittleSF.                   *
 *.                           end                              *
 **************************************************************/

  void setup() {
    // Serial port for debugging purposes
    Serial.begin(115200);
    delay(1000);
    pinMode(Relay_Pin, OUTPUT);
    pinMode(LED_Pin, OUTPUT);      //digitalWrite (LED_Pin, LOW);//LED_Pin off
    pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output

    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
      // Start LittleFS

  if (!LittleFS.begin())
  {

    Serial.println("An Error has occurred while mounting LittleFS");

    // Print the error on display

    delay(1000);

    return;
  }

}    

  void loop() {
//delay(1000);
    if (!client.connected()) {
      reconnect();
    }
    sendSensor();
    client.loop();
    
    timeClient.update();
    Day = timeClient.getDay();
    Hours = timeClient.getHours();
    Minutes = timeClient.getMinutes();
    seconds = timeClient.getSeconds();
    Serial.println("...");
     Serial.print("** 697 ** Time =  ");
     Serial.print(Hours);
      Serial.print(" : ");
      Serial.print(Minutes);
      Serial.println("...");


    Am = true;
    AmType[1] = 'Y';
    if (Hours >= 12) {
      Am = false;
      AmType[1] = 'N';
    }

        // // readData(Day_Hours,Day_Minutes,DayHighTemp);
        // Serial.print("**** 652 *** Day_Hours = ");
        // Serial.println(Day_Hours);
        // Serial.print("**** 652 *** Day_Minutes = ");
        // Serial.println(Day_Minutes);
        // Serial.print("**** 652 *** DayHighTemp = ");
        // Serial.println(DayHighTemp);
        //  Day_Hours = client.subscribe("day_h"); 
        //  Serial.println("*** Day_Hours = ");   
        //  Serial.println(Day_Hours);   
  }
