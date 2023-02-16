#/*
  Solar32

  Sends basic ESP32 data and solar panel measurements to a Tago.io MQTT broker.

  created Jan2022
  by Flavio Puhl <flavio_puhl@hotmail.com>
  
  This example code is in the public domain.

*/


/*+--------------------------------------------------------------------------------------+
 *| Libraries                                                                            |
 *+--------------------------------------------------------------------------------------+ */

#include <Arduino.h>

#include <TaskScheduler.h>                // Scheduler Library

#include <WiFi.h>

#include <WiFiUDP.h>                     // Network Time Protocol NPT library
#include <NTPClient.h>                   // Network Time Protocol NPT library

#include <ESP32Time.h>                   // ESP32 internal RTC library

#include <esp_task_wdt.h>                // Watchdog library

#include <rom/rtc.h>                     // Wakeup reason

#include <WiFiMulti.h>                   // http update
#include <HTTPClient.h>                  // http update
#include <HTTPUpdate.h>                  // http update
#include <WiFiClientSecure.h>            // http update

#include <PubSubClient.h>                // MQTT
#include <ArduinoJson.h>                 // MQTT

#include <Preferences.h>                 // save data on NVM

/*+--------------------------------------------------------------------------------------+
 *| Constants declaration                                                                |
 *+--------------------------------------------------------------------------------------+ */
 
 // Insert here the wifi network credentials
const char *ssid                              = "CasaDoTheodoro1";                       // name of your WiFi network
const char *password                          = "09012011";                              // password of the WiFi network

const char *ID                                = "ThisIsMyESP32ID";                      // Name of our device, must be unique
const char* BROKER_MQTT                       = "mqtt.tago.io";                          // MQTT Cloud Broker URL
unsigned int PORT                             = 8883;
const char *TOKEN                             ="1a0dfb08-f3ac-4cef-8643-d8c5a5c39341";

const char *USER                              = "MQTTTuser";

char *DeviceName                              = "Solar32";
String FirmWareVersion                        = "Solar32_001";


// Insert here topics that the device will publish to broker
const char *TopicsToPublish[]                 = { 
                                                "data",
                                                "info"
                                              };

// Insert here the topics the device will listen from broker
const char *TopicsToSubscribe[]               = { 
                                                "Solar32_reset", 
                                                "Solar32_update",  
                                                "Solar32_builtinled"
                                              };

unsigned long previousMillis = 0; 

// Insert here the *.bin file repository path
#define OTAFIRMWAREREPO                       "https://firebasestorage.googleapis.com/v0/b/firmwareota-a580e.appspot.com/o/Solar32%2Ffirmware.bin?alt=media"

String WakeUpReasonCPU0                       = "";
String WakeUpReasonCPU1                       = "";

int UptimeHours                               = 0;

#define LED 2       // Built In Led pin

#define WATCHDOGTIMEOUT                       10    // Watchdog set to 10 sec

//unsigned int epochAtPwrOff;     // Last epoch time saved before device power off (in sec)
//unsigned int epochAtPwrOn;      // Current epoch time at device power on (in sec)
//unsigned int epochCurrent;      // Current epoch time saved (in sec)
//unsigned int TimeOff;           // Time the device was powered off (in sec)
//unsigned int TimeOn;            // Time the device was powered on (in sec)
//float TotalTimeOff;             // Total time the device was powered off in lifetime (in hours)
//float TotalTimeOn;              // Total time the device was powered on in lifetime (in hours)

const int TP1_Analog_channel_pin     =   35;    // ADC1_CH7 >>> GPIO35
int       TP1_ADC_VALUE              =   0;
float     TP1_voltage_value          =   0;
float     TP1_voltage_divider_ratio  =   3.367;

const int TP2_Analog_channel_pin     =   33;    // ADC1_CH5 >>> GPIO33
int       TP2_ADC_VALUE              =   0;
float     TP2_voltage_value          =   0;
float     TP2_voltage_divider_ratio  =   3.367;

const int TP3_Analog_channel_pin     =   34;    // ADC1_CH4 >>> GPIO32
int       TP3_ADC_VALUE              =   0;
float     TP3_voltage_value          =   0;
float     TP3_voltage_divider_ratio  =   3.367;

// ADC1_CH0 >>> GPIO36
// ADC1_CH1 >>> ??
// ADC1_CH2 >>> ??
// ADC1_CH3 >>> GPIO39
// ADC1_CH4 >>> GPIO32
// ADC1_CH5 >>> GPIO33
// ADC1_CH6 >>> GPIO34
// ADC1_CH7 >>> GPIO35

/*+--------------------------------------------------------------------------------------+
 *| Callback methods prototypes                                                          |
 *+--------------------------------------------------------------------------------------+ */
 
void VerifyWifi();
void DateAndTimeNPT();
void SerializeAndPublish();
void Uptime();
void IAmAlive();
void dataCollect();

/*+--------------------------------------------------------------------------------------+
 *| Tasks lists                                                                          |
 *+--------------------------------------------------------------------------------------+ */

Task t0(01*30*1000, TASK_FOREVER, &IAmAlive);               // 30sec rate
Task t1(01*30*1000, TASK_FOREVER, &VerifyWifi);             // 30sec rate
Task t2(60*60*1000, TASK_FOREVER, &DateAndTimeNPT);         // 1hour rate
Task t3(01*60*1000, TASK_FOREVER, &SerializeAndPublish);    // 1min rate
Task t4(60*60*1000, TASK_FOREVER, &Uptime);                 // 1hour rate

/*+--------------------------------------------------------------------------------------+
 *| Objects                                                                              |
 *+--------------------------------------------------------------------------------------+ */

Scheduler runner;                                           // Scheduler
ESP32Time rtc;

//BearSSL::WiFiClientSecure wClient;
//WiFiClient wClient;
WiFiClientSecure wClient;
PubSubClient MQTTclient(wClient);                           // Setup MQTT client

Preferences NVMdata;                // initiate an instance of the Preferences library

/*+--------------------------------------------------------------------------------------+
 *| Method to reset the device                                                           |
 *+--------------------------------------------------------------------------------------+ */
 
void deviceReset() {

  MQTTclient.publish(TopicsToPublish[1], "[Solar32] > Restarting device");
  delay(1000);
  ESP.restart();
  
}


/*+--------------------------------------------------------------------------------------+
 *| Method to drive the built-in led                                                     |
 *+--------------------------------------------------------------------------------------+ */
 
void builtInLedTest(int input) {

  if (input == 0 ) { 
    digitalWrite(LED,LOW);  
    Serial.println("Turning LED off"); 
    MQTTclient.publish(TopicsToPublish[1], "[Solar32] > Turning LED off");
    }     
  if (input == 1 ) { 
    digitalWrite(LED,HIGH); 
    Serial.println("Turning LED on");  
    MQTTclient.publish(TopicsToPublish[1], "[Solar32] > Turning LED on");
    }   
  
}

/*+--------------------------------------------------------------------------------------+
 *| Connect to WiFi network                                                              |
 *+--------------------------------------------------------------------------------------+ */

void setup_wifi() {
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
    WiFi.mode(WIFI_STA);                              // Setup ESP in client mode
    //WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.begin(ssid, password);                       // Connect to network

    int wait_passes = 0;
    while (WiFi.status() != WL_CONNECTED) {           // Wait for connection
      delay(500);
      Serial.print(".");
      if (++wait_passes >= 20) { ESP.restart(); }     // Restart in case of no wifi connection   
    }

  Serial.print("\nWiFi connected");
  Serial.print("\nIP address: ");
    Serial.println(WiFi.localIP());

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

}

/*+--------------------------------------------------------------------------------------+
 *| Verify and Manage WiFi network                                                       |
 *+--------------------------------------------------------------------------------------+ */

void VerifyWifi() {

  if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)){          // Check for network health
      
    Serial.printf("error: WiFi not connected, reconnecting \n");
            
      WiFi.disconnect();
      setup_wifi();             

  } 

}

/*+--------------------------------------------------------------------------------------+
 *| Get Date & Time                                                                      |
 *+--------------------------------------------------------------------------------------+ */

void DateAndTimeNPT(){

  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP); 

  timeClient.begin();                                                                     // Initialize a NTPClient to get time

  timeClient.setTimeOffset(-10800);                                                       // Set offset time in seconds to Brazil timezone GMT-3

    if(!timeClient.update()) {
      timeClient.forceUpdate();
    }

  timeClient.update();  

  time_t epochTime = timeClient.getEpochTime();                                             // The time_t type is just an integer.  It is the number of seconds since the Epoch.

  rtc.setTime(epochTime);                                                                   // Update internal RTC

  Serial.println("Internal RTC  : [ Updated ]");
 
}

unsigned long DateAndTimeEpochRTC(){

  unsigned long epochTime = rtc.getEpoch();

  return epochTime;
}


String DateAndTimeFormattedRTC(){

  time_t epochTime = rtc.getEpoch();                                                     // The time_t type is just an integer. 

  struct tm * tm = localtime(&epochTime);
  char DateAndTimeFormated[22];
    strftime(DateAndTimeFormated, sizeof(DateAndTimeFormated), "%d%b%Y %H-%M-%S", tm);   // https://www.cplusplus.com/reference/ctime/strftime/
  
  return DateAndTimeFormated;
}


/*+--------------------------------------------------------------------------------------+
 *| Method to print the reason by which ESP32 has been awaken from sleep                 |
 *+--------------------------------------------------------------------------------------+ */

String print_reset_reason(RESET_REASON reason)
{
  String rst_reason;

  switch ( reason)
  {
    case 1 : rst_reason = String(reason) + " POWERON_RESET";break;          /**<1, Vbat power on reset*/
    case 3 : rst_reason = String(reason) + " SW_RESET";break;               /**<3, Software reset digital core*/
    case 4 : rst_reason = String(reason) + " OWDT_RESET";break;             /**<4, Legacy watch dog reset digital core*/
    case 5 : rst_reason = String(reason) + " DEEPSLEEP_RESET";break;        /**<5, Deep Sleep reset digital core*/
    case 6 : rst_reason = String(reason) + " SDIO_RESET";break;             /**<6, Reset by SLC module, reset digital core*/
    case 7 : rst_reason = String(reason) + " TG0WDT_SYS_RESET";break;       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : rst_reason = String(reason) + " TG1WDT_SYS_RESET";break;       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : rst_reason = String(reason) + " RTCWDT_SYS_RESET";break;       /**<9, RTC Watch dog Reset digital core*/
    case 10 : rst_reason = String(reason) + " INTRUSION_RESET";break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : rst_reason = String(reason) + " TGWDT_CPU_RESET";break;       /**<11, Time Group reset CPU*/
    case 12 : rst_reason = String(reason) + " SW_CPU_RESET";break;          /**<12, Software reset CPU*/
    case 13 : rst_reason = String(reason) + " RTCWDT_CPU_RESET";break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : rst_reason = String(reason) + " EXT_CPU_RESET";break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : rst_reason = String(reason) + " RTCWDT_BROWN_OUT_RESET";break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : rst_reason = String(reason) + " RTCWDT_RTC_RESET";break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : rst_reason = String(reason) + " UNKNOW";
  }

  return rst_reason;
}


void verbose_print_reset_reason(int reason)
{
  switch ( reason)
  {
    case 1  : Serial.println (" Vbat power on reset");break;
    case 3  : Serial.println (" Software reset digital core");break;
    case 4  : Serial.println (" Legacy watch dog reset digital core");break;
    case 5  : Serial.println (" Deep Sleep reset digital core");break;
    case 6  : Serial.println (" Reset by SLC module, reset digital core");break;
    case 7  : Serial.println (" Timer Group0 Watch dog reset digital core");break;
    case 8  : Serial.println (" Timer Group1 Watch dog reset digital core");break;
    case 9  : Serial.println (" RTC Watch dog Reset digital core");break;
    case 10 : Serial.println (" Instrusion tested to reset CPU");break;
    case 11 : Serial.println (" Time Group reset CPU");break;
    case 12 : Serial.println (" Software reset CPU");break;
    case 13 : Serial.println (" RTC Watch dog Reset CPU");break;
    case 14 : Serial.println (" for APP CPU, reseted by PRO CPU");break;
    case 15 : Serial.println (" Reset when the vdd voltage is not stable");break;
    case 16 : Serial.println (" RTC Watch dog reset digital core and rtc module");break;
    default : Serial.println (" NO_MEAN");
  }
}


/*+--------------------------------------------------------------------------------------+
 *| Remote HTTP OTA                                                                      |
 *+--------------------------------------------------------------------------------------+ */

void RemoteHTTPOTA(){

  MQTTclient.publish(TopicsToPublish[1], "[Solar32] > Updating device");

  //if ((WiFi.status() == WL_CONNECTED && updateSoftareOnNextReboot == 1)) {
  if ((WiFi.status() == WL_CONNECTED)) {

    Serial.println("SW Update     :  [ Started ]");

    esp_task_wdt_init(WATCHDOGTIMEOUT, false); 


    WiFiClientSecure client; 
    client.setInsecure();       

    // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    httpUpdate.setLedPin(LED_BUILTIN, LOW);

    t_httpUpdate_return ret = httpUpdate.update(client, OTAFIRMWAREREPO);
    

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("SW Download   :  [ Done, will update on next reboot ]");
        break;
    }
  } else{
    Serial.println("SW Download   : [ Not requested ]");
  }
}


/*+--------------------------------------------------------------------------------------+
 *| Reconnect to MQTT client                                                             |
 *+--------------------------------------------------------------------------------------+ */
 
void MQTTconnect() {

  if(!MQTTclient.connected()) {                               // Check if MQTT client is connected
  
  Serial.println();
  Serial.println("MQTT Client   : [ not connected ]");

  MQTTclient.setServer(BROKER_MQTT, PORT);                    // MQTT broker info
  MQTTclient.setBufferSize(1024);
                      
    
    Serial.println("MQTT Client   : [ trying connection ]");
    
    if (MQTTclient.connect(ID,USER,TOKEN)) {
      Serial.println("MQTT Client   : [ broker connected ]");

      for(int i=0; i<=((sizeof(TopicsToPublish) / sizeof(TopicsToPublish[0]))-1); i++){

        Serial.print("MQTT Client   : [ publishing to ");
        Serial.print(TopicsToPublish[i]);
        Serial.println(" ]");
        
      }

      for(int i=0; i<=((sizeof(TopicsToSubscribe) / sizeof(TopicsToSubscribe[0]))-1); i++){

        Serial.print("MQTT Client   : [ subscribing to ");
        Serial.print(TopicsToSubscribe[i]);
        Serial.println(" ]");
        MQTTclient.subscribe(TopicsToSubscribe[i]);
      }

    } else {
      Serial.print("MQTT Client   : [ failed, rc= ");
      Serial.print(MQTTclient.state());
      Serial.println(" ]");

      delay(5000);
      setup_wifi();
    }
  }
}

/*+--------------------------------------------------------------------------------------+
 *| Serialize JSON and publish MQTT                                                      |
 *+--------------------------------------------------------------------------------------+ */

void SerializeAndPublish() {

  if (!MQTTclient.connected())                            // Reconnect if connection to MQTT is lost 
  {    MQTTconnect();      }

  dataCollect();                    // Read voltage values prior sending to broker

  char buffer[1024];                                      // JSON serialization 
   
  StaticJsonDocument<1024> doc;

    JsonObject doc_0 = doc.createNestedObject();
    doc_0["variable"] = "Solar32_DeviceName";
    doc_0["value"] = DeviceName;
    doc_0["unit"] = "";

    JsonObject doc_1 = doc.createNestedObject();
    doc_1["variable"] = "Solar32_FirmWareVersion";
    doc_1["value"] = FirmWareVersion;
    doc_1["unit"] = "";

    JsonObject doc_2 = doc.createNestedObject();
    doc_2["variable"] = "Solar32_WiFiRSSI";
    doc_2["value"] = WiFi.RSSI();
    doc_2["unit"] = "dB";

    JsonObject doc_3 = doc.createNestedObject();
    doc_3["variable"] = "Solar32_IP";
    doc_3["value"] = WiFi.localIP();
    doc_3["unit"] = "";

    JsonObject doc_4 = doc.createNestedObject();
    doc_4["variable"] = "Solar32_LastRoll";
    doc_4["value"] = DateAndTimeFormattedRTC();
    doc_4["unit"] = "";

    JsonObject doc_5 = doc.createNestedObject();
    doc_5["variable"] = "Solar32_UptimeHours";
    doc_5["value"] = UptimeHours;
    doc_5["unit"] = "h";

    JsonObject doc_6 = doc.createNestedObject();
    doc_6["variable"] = "Solar32_WakeUpReasonCPU0";
    doc_6["value"] = WakeUpReasonCPU0;
    doc_6["unit"] = "";

    JsonObject doc_7 = doc.createNestedObject();
    doc_7["variable"] = "Solar32_WakeUpReasonCPU1";
    doc_7["value"] = WakeUpReasonCPU1;
    doc_7["unit"] = "";

    JsonObject doc_8 = doc.createNestedObject();
    doc_8["variable"] = "Solar32_FreeHeap";
    doc_8["value"] = ESP.getFreeHeap();
    doc_8["unit"] = "bytes";

    JsonObject doc_9 = doc.createNestedObject();
    doc_9["variable"] = "Solar32_TP1";
    doc_9["value"] = TP1_voltage_value;
    doc_9["unit"] = "Volts";

    JsonObject doc_10 = doc.createNestedObject();
    doc_10["variable"] = "Solar32_TP2";
    doc_10["value"] = TP2_voltage_value;
    doc_10["unit"] = "Volts";  

    JsonObject doc_11 = doc.createNestedObject();
    doc_11["variable"] = "Solar32_TP3";
    doc_11["value"] = TP3_voltage_value;
    doc_11["unit"] = "Volts";       

    serializeJson(doc, buffer);

    Serial.printf("\nJSON Payload:");
      Serial.printf("\n");
    
    serializeJsonPretty(doc, Serial);                 // Print JSON payload on Serial port        
      Serial.printf("\n");
      Serial.println("MQTT Client   : [ Sending message to MQTT topic ]"); 
      Serial.println("");         

    MQTTclient.publish(TopicsToPublish[0], buffer);    // Publish data to MQTT Broker 
       
}

/*+--------------------------------------------------------------------------------------+
 *| MQTT Callback                                                                        |
 *+--------------------------------------------------------------------------------------+ */

String MQTTcallback(char* topicSubscribed, byte* payload, unsigned int length) {
  
  String payloadReceived = "";
  
  Serial.print("Message arrived [");
  Serial.print(topicSubscribed);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
    payloadReceived += (char)payload[i];
  }
  Serial.println();

  Serial.println(payloadReceived);

  if (strcmp (topicSubscribed, TopicsToSubscribe[0]) == 0 ) { deviceReset();}
  if (strcmp (topicSubscribed, TopicsToSubscribe[1]) == 0 ) { RemoteHTTPOTA();}
  if (strcmp (topicSubscribed, TopicsToSubscribe[2]) == 0 ) { builtInLedTest(payloadReceived.toInt());}

  return payloadReceived;

}


/*+--------------------------------------------------------------------------------------+
 *| Uptime counter in hours                                                              |
 *+--------------------------------------------------------------------------------------+ */
 
void Uptime() {

  UptimeHours++;            // Uptime counts the time the device is working since last power on

}


/*+--------------------------------------------------------------------------------------+
 *| Simple method to show that program is running                                        |
 *+--------------------------------------------------------------------------------------+ */

void IAmAlive(){

  Serial.print("RTC formatted  : ");
  Serial.println(DateAndTimeFormattedRTC());
  
  //Serial.print("RTC epoch      : ");
  //Serial.println(DateAndTimeEpochRTC());

  //NVMdata.putULong("epochCurrent", DateAndTimeEpochRTC());

  //TimeDiagnostics();

  //dataCollect();

  // Close the Preferences
  //NVMdata.end();
 
  
}

/*+--------------------------------------------------------------------------------------+
 *| Internet Availability Diagnostics                                                    |
 *+--------------------------------------------------------------------------------------+ */
/*
void TimeDiagnostics(){

// Device Total Time OFF (in hours)

  NVMdata.getFloat("TotalTimeOff", 0);              // Get previous saved value from NVM
  TimeOff = (epochAtPwrOn - epochAtPwrOff);         // Calculate the amount of time the device was powered off    
  TotalTimeOff = (TotalTimeOff + TimeOff)/3600;     // Add previous calculation to the total value
  NVMdata.putFloat("TotalTimeOff", TotalTimeOff);   // Save value back to NVM 
 
// Device Total Time ON

  TimeOn = (DateAndTimeEpochRTC() - epochAtPwrOn);  // Calculate the amount of time the device is powered on
  TotalTimeOn = float(TimeOn)/3600;                 // Calculate the amount of time the device is powered on in hours

// Print Diagnostics on serial
  Serial.printf("|-----------------------------------------------------|\n");
  Serial.printf("| Device Wake Up Epoch Time        (s) : %10u   |\n", epochAtPwrOn);
  Serial.printf("| Device Current Epoch Time        (s) : %10u   |\n", DateAndTimeEpochRTC());
  Serial.printf("| Device Total Time On since wake  (s) : %10u   |\n", TimeOn);
  Serial.printf("|                                  (h) : %10.3f   |\n", TotalTimeOn);
  Serial.printf("| Device Last Time Off period      (s) : %10u   |\n", TimeOff);
  Serial.printf("| Device Total Life Time Off       (h) : %10.3f   |\n", TotalTimeOff);
  Serial.printf("|-----------------------------------------------------|\n");
 
  
}
*/

/*+--------------------------------------------------------------------------------------+
 *| Collect data from analog inputs                                                      |
 *+--------------------------------------------------------------------------------------+ */

void dataCollect(){

  TP1_ADC_VALUE = analogRead(TP1_Analog_channel_pin);
  TP1_voltage_value = ((TP1_ADC_VALUE * 3.3 ) / (4095))*TP1_voltage_divider_ratio;

  TP2_ADC_VALUE = analogRead(TP2_Analog_channel_pin);
  TP2_voltage_value = ((TP2_ADC_VALUE * 3.3 ) / (4095))*TP2_voltage_divider_ratio;

  TP3_ADC_VALUE = analogRead(TP3_Analog_channel_pin);
  TP3_voltage_value = ((TP3_ADC_VALUE * 3.3 ) / (4095))*TP3_voltage_divider_ratio;

  //int analogValue = analogRead(2);
  //int analogVolts = analogReadMilliVolts(2);

  Serial.printf("|-----------------------------------------------------|\n");
  Serial.printf("| TP1_ADC_VALUE        (bits)  : %10u   |\n", TP1_ADC_VALUE);
  Serial.printf("| TP1_voltage_value    (v)     : %10.3f   |\n", TP1_voltage_value);
  Serial.printf("|-----------------------------------------------------|\n");
  Serial.printf("| TP2_ADC_VALUE        (bits)  : %10u   |\n", TP2_ADC_VALUE);
  Serial.printf("| TP2_voltage_value    (v)     : %10.3f   |\n", TP2_voltage_value);
  Serial.printf("|-----------------------------------------------------|\n");
  Serial.printf("| TP3_ADC_VALUE        (bits)  : %10u   |\n", TP3_ADC_VALUE);
  Serial.printf("| TP3_voltage_value    (v)     : %10.3f   |\n", TP3_voltage_value);
  Serial.printf("|-----------------------------------------------------|\n");
 
}


/*+--------------------------------------------------------------------------------------+
 *| Setup                                                                                |
 *+--------------------------------------------------------------------------------------+ */
 
void setup() {

  Serial.begin(115200);                                                                     // Start serial communication at 115200 baud
  delay(3000);

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  analogReadResolution(12);           // Resolution 12 bits (0 – 4095)
  analogSetAttenuation(ADC_11db);     // The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 2600 mV. (1V input = ADC reading of 1575).

  //NVMdata.begin("my-app", false);         // Open Preferences with my-app namespace

  //Remove all preferences under the opened namespace
  //preferences.clear();

  //Remove the specific key
  //preferences.remove("epochAtPwrOff");

  //epochAtPwrOff = NVMdata.getULong("epochCurrent", 0);  // retrives the last epoch time before device power off

  Serial.println();
  Serial.println(FirmWareVersion);
  Serial.println();

    runner.init();

    runner.addTask(t0);
    t0.enable();  
      Serial.println("Added task    : [ IAmAlive ]");

    runner.addTask(t1);
    t1.enable();  
      Serial.println("Added task    : [ VerifyWifi ]");

    runner.addTask(t2);
    t2.enable();
      Serial.println("Added task    : [ DateAndTimeNPT ]");

    runner.addTask(t3);
    t3.enable();
      Serial.println("Added task    : [ SerializeAndPublish ]");

    runner.addTask(t4);
    t4.enable();
      Serial.println("Added task    : [ Uptime ]");

  setup_wifi();         // Start wifi

  wClient.setInsecure();

  esp_task_wdt_init(WATCHDOGTIMEOUT, true);               // Enable watchdog                                               
  esp_task_wdt_add(NULL);                                 // No tasks attached to watchdog
     Serial.println("Watchdog      : [ initialized ]");

  DateAndTimeNPT();     // Get time from NPT and update ESP RTC

  //epochAtPwrOn  = DateAndTimeEpochRTC();

  MQTTconnect();        // Connect to MQTT Broker

  MQTTclient.setCallback(MQTTcallback);       // MQTT callback method to subscribing topics
 
  WakeUpReasonCPU0 = print_reset_reason(rtc_get_reset_reason(0));
    Serial.print("CPU0 rst reason:");
    Serial.print(WakeUpReasonCPU0);
      verbose_print_reset_reason(rtc_get_reset_reason(0));

  WakeUpReasonCPU1 = print_reset_reason(rtc_get_reset_reason(1));
    Serial.print("CPU1 rst reason:");
    Serial.print(WakeUpReasonCPU1);
      verbose_print_reset_reason(rtc_get_reset_reason(1));
   
  UptimeHours--;

  //SerializeAndPublish();
  Serial.println("");
  Serial.println("Setup         : [ finished ]");
  Serial.println("");

  esp_task_wdt_reset();   // feed watchdog
  
}


/*+--------------------------------------------------------------------------------------+
 *| main loop                                                                            |
 *+--------------------------------------------------------------------------------------+ */
 
void loop() {

  runner.execute();         // Scheduler stuff

  MQTTclient.loop();        // Needs to be in the loop to keep client connection alive

  esp_task_wdt_reset();     // feed watchdog

}
