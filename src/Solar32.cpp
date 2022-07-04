#include <Arduino.h>

//#define THINGER_SERIAL_DEBUG

#include <ThingerESP32.h>
#include <ThingerConsole.h>

#define uS_TO_S_FACTOR 1000000ULL   // Conversion factor for micro seconds to seconds 
#define TIME_TO_SLEEP  5            // Time ESP32 will go to sleep (in seconds) 

#define USERNAME "fpuhl"
#define DEVICE_ID "Solar32dev"
#define DEVICE_CREDENTIAL "XFGAgJyQyMz4XgrU"

#define SSID "CasaDoTheodoro1"
#define SSID_PASSWORD "09012011"

#define API_KEY "AIzaSyBVNZmsEJ6gAKi6F2dhpQRaDbJVcv-zMY4"     // Firebase: Define the API Key 
#define USER_EMAIL "flaviopuhljr@gmail.com"                   // Firebase: Define the user Email 
#define USER_PASSWORD "Theodoro@01"                           // Firebase: Define password 
#define STORAGE_BUCKET_ID "firmwareota-a580e.appspot.com"     // Firebase: Define the Firebase storage bucket ID e.g bucket-name.appspot.com 
#define FIRMWARE_PATH "test/firmware.bin"                     // Firebase: Define the firmware path on Firebase

const int SolarPanelAI = 34;        // Solar Panel is connected to GPIO 34 (Analog ADC1_CH6)  
const int BatteryAI = 35;           // Battery is connected to GPIO 35     (Analog ADC1_CH7)

float SolarPanelVoltage = 0;
float BatteryVoltage = 0;
float voltageDivisorRatio = 1;      // Here the voltage divisor rate
unsigned long previousMillis;

bool ECOmodeState = false;          // Eco mode initial state is false

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
ThingerConsole console(thing);

void setup() {
  // open serial for debugging
  Serial.begin(115200);

  console.set_prompt("Solar32");

  //pinMode(16, OUTPUT);
  pinMode(2, OUTPUT); // Buit-in led
  pinMode(16, OUTPUT); 

  thing.add_wifi(SSID, SSID_PASSWORD);

  // digital pin control example (i.e. turning on/off a light, a relay, configuring a parameter, etc)
  thing["GPIO_16"] << digitalPin(16);
  thing["GPIO_02"] << digitalPin(2); // built-in led
 

  // resource output example (i.e. reading a sensor value)
  thing["data"] >> [](pson& out){  
     // Add the values and the corresponding code
      out["SolarPanelVoltage"]    = SolarPanelVoltage;
      out["BatteryVoltage"]   = BatteryVoltage;
  };


  // ECO mode feedback to dashboard
  thing["ECOmode"] << [](pson& in){
    if(in.is_empty()){
        in = ECOmodeState;
    }
    else{
        ECOmodeState = in;
    }
  };

  // more details at http://docs.thinger.io/arduino/
}

void loop() {

  unsigned long currentMillis = millis();             /* capture the latest value of millis() */
  thing.handle();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  if (currentMillis - previousMillis >= 1000) {          

    SolarPanelVoltage = analogRead(SolarPanelAI)*((3*voltageDivisorRatio)/4095);
      delay(10);
    BatteryVoltage = analogRead(BatteryAI)*((3*voltageDivisorRatio)/4095);;
      delay(10);

    Serial.println("SolarPanel Voltage = " + String(SolarPanelVoltage) + " V || Battery Voltage    = " + String(BatteryVoltage) + " V"); 

    if(console){
      console.printf("SolarPanel Voltage = %f V || Battery Voltage    = %f V \r\n",SolarPanelVoltage,BatteryVoltage);
    }

    if(ECOmodeState){
      Serial.println("ECO Mode (light sleep) ... ON");
    } else {
      Serial.println("ECO Mode (light sleep) ... OFF");
    }
  previousMillis = currentMillis;
  }

  if(ECOmodeState){
    Serial.println("Going to light-sleep now");
    Serial.flush(); 
    esp_light_sleep_start();
  }
  

}
