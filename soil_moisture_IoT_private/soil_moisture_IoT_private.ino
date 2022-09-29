#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHT.h>

#include "ESP8266WiFi.h"

// valve motor pins
int IN1 = D5;
int IN2 = D6;

uint8_t DHT11Pin = D7;

//#define moisturePin A0
const int moisturePin = A0 ;
float sensorValue = 0;
//uint8_t ledPin = D6;
String command = "Auto";

const char *ssid = "yourwifiname";
const char *password = "yourwifipassword";

const int AirValue = 620;
const int WaterValue = 310;

const char onlineStatusTopic[] = "kasmad/feeds/humidity-meter";


/************************* MQTT Broker Setup *********************************/

#define SERVER      "yourhost"
#define SERVERPORT  1883
#define USERNAME    "yourusername"
#define KEY         "yourpassword"

#define DHTTYPE DHT11

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER, SERVERPORT, USERNAME, KEY);
//Adafruit_MQTT_Publish soilMoisture = Adafruit_MQTT_Publish(&mqtt, "kasmad/feeds/humidity-meter");
//Adafruit_MQTT_Subscribe waterBtn = Adafruit_MQTT_Subscribe(&mqtt, "kasmad/feeds/water-the-plants");
//Adafruit_MQTT_Subscribe settings = Adafruit_MQTT_Subscribe(&mqtt, "kasmad/feeds/digifarm-settings");

/* thingspeak mqtt broker */
Adafruit_MQTT_Publish soilMoisture = Adafruit_MQTT_Publish(&mqtt, "channels/1382058/subscribe/fields/field1");
Adafruit_MQTT_Subscribe waterBtn = Adafruit_MQTT_Subscribe(&mqtt, "channels/1382058/subscribe/fields/field4");
Adafruit_MQTT_Subscribe settings = Adafruit_MQTT_Subscribe(&mqtt, "kasmad/feeds/digifarm-settings");

DHT dht(DHT11Pin, DHTTYPE);

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
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
}

void loop() {
  float suhu = dht.readTemperature();
  float kelembapan = dht.readHumidity();
  Serial.print(kelembapan); Serial.print("%");
  Serial.print("\t");
  Serial.println(suhu); Serial.print("* C");


  MQTT_connect();
  String sensorValueStr;

  Serial.print(F("\nSending moisture val "));
  Serial.print("...");
  sensorValue = getMoistureVal(moisturePin);
  sensorValueStr = String(sensorValue);
  Serial.println(sensorValue);
  if (! soilMoisture.publish(sensorValue)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK! "));
    Serial.println(sensorValue);
    blinkEsp();
  }

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &waterBtn) {
      Serial.print(F("Got: "));
      Serial.println((char *)waterBtn.lastread);
      command = (char *)waterBtn.lastread;

      if (command.equals("Auto")) { /* auto siram */
        if (sensorValue >= 41 && sensorValue <= 80) {
          turnOffMotor();
        } else if (sensorValue < 41) {
          turnOnMotor();
        }
      }

      /* manual override */
      if (command.equals("On")) {
        Serial.println("Seharusnya Motornya ON!");
        turnOnMotor();
        blinkEsp();
      } else if (command.equals("Off")) {
        Serial.println("Seharusnya Motornya OFF!");
        turnOffMotor();
        blinkEsp();
      }
    }
  }
  Serial.println("Motor status: " + String(digitalRead(IN1)));
  delay(30);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    blinkEsp();
    return;
  }

  // Send device's last status
//  mqtt.publish(onlineStatusTopic, mqtt.connected() ? "Online" : "Offline", true);

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

float getMoistureVal(int moisturePin) {
  float moisture_percentage;
  int sensor_analog;
  sensor_analog = analogRead(moisturePin);
  //  moisture_percentage = ( 100 - ( (sensor_analog/1023.00) * 100 ) ); // resistive sensor
  moisture_percentage = map(sensor_analog, AirValue, WaterValue, 0, 100); // capacitive sensor
  if (moisture_percentage >= 100) moisture_percentage = 100;
  else if (moisture_percentage <= 0) moisture_percentage = 0;
  else if (moisture_percentage > 0 && moisture_percentage < 100 <= 0) moisture_percentage;
  delay(1000);
  return moisture_percentage;
}

void turnOnMotor() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void turnOffMotor() {
  delay(2000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void blinkEsp(){
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
  delay(500);                      // Wait for a half second
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  delay(500);                      // Wait for a half second (to demonstrate the active low LED) 
}
