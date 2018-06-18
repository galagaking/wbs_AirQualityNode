#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include <Servo.h>
// Root JSON object
//
// Inside the brackets, 200 is the size of the memory pool in bytes.
// Don't forget to change this value to match your JSON document.
// Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<255> doc;

//////////////////////////
// MicroOLED Definition //
//////////////////////////
#define PIN_RESET 255  // Connect RST to pin 9 (SPI & I2C)
#define DC_JUMPER 0  // DC jumper setting(I2C only)
MicroOLED oled(PIN_RESET, DC_JUMPER);  // I2C Example

// WiFi Credentials
const char* ssid = "";
const char* password = "";
// MQTT Credentials
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_username = "";
const char* mqtt_password = "";
const int   mqtt_port=1883;

Servo myservo;  // create servo object to control a servo 

void callback(char* topic, byte* payload, unsigned int length);

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;

int value = 0;
String strTemperature="*";
String strPressure,strServo, strLed, strtVOC,strCO2,strHumidity;    //MQTT values will be stored in this variables
char sub_topic[30]; // topic to subscribe to
byte ledSignal=1;

int servoPos=90;      // start mid position (0-180 degrees)

void setup() {
  
  Serial.begin(115200);
  myservo.attach(D8);  // attaches the servo on D8 to the servo object 
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, ledSignal);   // Turn the LED off (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because 
                                    // it is acive low on the ESP-01)
  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  delay(1000);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  oled.clear(PAGE);
  oled.setCursor(0, 0);
  oled.setFontType(0);
  oled.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
    oled.print(".");
    oled.display();
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  oled.setCursor(0, 16);
  oled.setFontType(0);
  oled.println("WiFi");
  oled.println("connected");
  oled.setCursor(0,32);
  oled.println("Waiting   for MQTT");
  oled.display();
  delay(500);
}

void callback(char* topic, byte* payload, unsigned int length) {
// Called when data arrives
// as published on the topic
// Payload is in JSON

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] -> ");
  payload[length] = '\0';
  Serial.println(String((char*)payload));
//  String topicparse = String(topic).substring(22);

//  Serial.println(topicparse);

  // values will be stored and displayed in the main loop
  // servo position will be set in the main loop
    // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, payload);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  // Get the root object in the document
  JsonObject& root = doc.as<JsonObject>();

  // Fetch values.
  //
  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do doc["time"].as<long>();

  strTemperature=root["temp"].as<String>();
  strPressure=root["pressure"].as<String>();
  strtVOC=root["tvoc"].as<String>();
  strCO2=root["co2"].as<String>();
  strHumidity=root["humidity"].as<String>();
  servoPos=root["servo"].as<int>();
  ledSignal=1-root["led"].as<int>(); // onboard LED is active LOW 
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    uint32_t chipid=ESP.getChipId();
    char clientid[25];
    snprintf(clientid,25,"WIFI-Display-%08X",chipid); //this adds the mac address to the client for a unique id
    //clientID can also be used in a shared MQTT service ie HiveMQ as an unique topic to subscribe to
    Serial.print("Client ID: ");
    Serial.println(clientid);
    if (client.connect(clientid,mqtt_username,mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(clientid, "Wemos connected...");
      // ... and resubscribe
      client.subscribe(clientid);
      Serial.print("Subscribed to: ");
      Serial.println(clientid);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  delay(100);
  long now = millis();

  if (now - lastMsg > 5000) {
    lastMsg = now;
    if (strTemperature != "*") { //only show data after it is received
    oled.clear(PAGE);
    oled.setFontType(2);  // Set font to type 2
    oled.setCursor(0, 0); // Set cursor to top-left
    oled.print(strTemperature);
    oled.setCursor(0,20);
    oled.setFontType(0);
    oled.print("CO2: ");
    oled.print(strCO2);
    oled.setCursor(0,32);
    oled.setFontType(0);
    oled.print("tVOC: ");
    oled.print(strtVOC);
    oled.display();
    myservo.write(servoPos);
    digitalWrite(LED_BUILTIN, ledSignal);   // Turn the LED on/off
    }
  }
}  
