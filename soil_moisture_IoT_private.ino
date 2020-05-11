#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#include "ESP8266WiFi.h"

uint8_t IN1 = D5;
uint8_t IN2 = D6;

//#define moisturePin A0 
const int moisturePin = A0 ; 
float sensorValue = 0; 
uint8_t ledPin = D6;
String command = "Auto";  

const char *ssid = "your-wifi-sid"; 
const char *password = "your-wifi-password";


/************************* MQTT Broker Setup *********************************/

#define SERVER      "your-server-ip-or-name"
#define SERVERPORT  1883                   // use 8883 for SSL
#define USERNAME    "your-mqtt-username"
#define KEY         "your-mqtt-key"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER, SERVERPORT, USERNAME, KEY);
Adafruit_MQTT_Publish soilMoisture = Adafruit_MQTT_Publish(&mqtt, "feeds/moisture");
Adafruit_MQTT_Subscribe waterBtn = Adafruit_MQTT_Subscribe(&mqtt, "feeds/water_the_plants");

void MQTT_connect();

void setup() { 
 Serial.begin(115200); 
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   Serial.print(".");
 }
 Serial.println("");
 Serial.println("WiFi connected");
 Serial.println("IP address: ");
 Serial.println(WiFi.localIP());

 mqtt.subscribe(&waterBtn);
// pinMode(ledPin, OUTPUT);
  pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
} 

void loop() { 
  MQTT_connect();
  String sensorValueStr;

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &waterBtn) {
      Serial.print(F("Got: "));
      Serial.println((char *)waterBtn.lastread);
      command = (char *)waterBtn.lastread;
      if(command.equals("Start")){
//        digitalWrite(ledPin, HIGH);
          turnOnMotor();
      } else if (command.equals("Stop")){
//        digitalWrite(ledPin, LOW);
        turnOffMotor();
        command = "Auto";
      }
    }
  }

  Serial.print(F("\nSending moisture val "));
  Serial.print("...");
  sensorValue = getMoistureVal(moisturePin);
 sensorValueStr = String(sensorValue);
// Serial.println(sensorValue); 
 if (! soilMoisture.publish(sensorValue)) {
    Serial.println(F("Failed"));
 } else {
    Serial.println(F("OK! "));
    Serial.println(sensorValue);
 }
  /* auto siram */
 if (command.equals("Auto")){ 
   if (sensorValue >= 41 && sensorValue <= 80){
//    digitalWrite(ledPin, LOW);
//    Serial.println(digitalRead(ledPin));
    turnOffMotor();
   } else if(sensorValue < 41) {
//    digitalWrite(ledPin, HIGH);
//    Serial.println(digitalRead(ledPin));
    turnOnMotor();
   }
 }
 delay(30); 
} 

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

float getMoistureVal(int moisturePin){
  float moisture_percentage;
  int sensor_analog;
  sensor_analog = analogRead(moisturePin);
  moisture_percentage = ( 100 - ( (sensor_analog/1023.00) * 100 ) );
  delay(1000);
 return moisture_percentage;
}

void turnOnMotor(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void turnOffMotor(){
  delay(2000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
