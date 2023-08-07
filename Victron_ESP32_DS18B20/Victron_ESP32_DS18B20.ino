/*
 This sketch connects to a Victron Energy GX device via MQTT
 and performs multiple duties:
   * Temperature and humidty sensor using AHT20 sensor
   * Remote mode switching of Multiplus inverter/charger
 
 It has been tested and deployed on Arduino Nano IOT 33.

 It has a dependency on the MQTT device service available here: https://github.com/freakent/dbus-mqtt-devices.

 Created January 2022 by Martin Jarvis (FreakEnt/Gone-sailing)

 To Do:
 1) Set the client ID using the WiFi MAC address
 2) Change payload of status message to allow for multiple services of the same type
 */
//#include <SPI.h>
//#include <WiFiNINA.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h" 

#include <OneWire.h>
#include <DallasTemperature.h>

const char VERSION[] = "v1.0"; 
const boolean headless = true; //useful for debugging 
float Level = 50;

#define WIFI_SSID "UBNT_IOT"
#define WIFI_PASSWORD "bdeaf13680"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
//const char ssid[] = SECRET_SSID;        // your network SSID (name)
//const char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
const char broker[] = MQTT_SERVER;      // "venus.local"; // "test.mosquitto.org";
int        port     = MQTT_SERVER_PORT;

//const int buttonPin = 2;     // the number of the pushbutton pin
//const int ledPin =  13;      // the number of the LED pin

const char clientId[] = "fe001"; // Do not include characters .-: only use underscores _
//const char serviceId[] = "t1"; // any string to uniquely identify the service on this device

const char serviceId[] = "t1"; // any string to uniquely identify the service on this device
const char tankServiceId[] = "tk1"; // another string to uniquely identify the tank service on this device


const char portalIDTopic[]  = "N/+/system/0/Serial"; // Topic to retrieve the VRM Portal ID for the Venus device
const char keepaliveTopicTemplate[] = "R/<portal ID>/keepalive"; // Topic to send keepalive request to
const char keepalivePayload[] = "[\"vebus/+/Mode\", \"temperature/+/#\"]"; // Choose which topics you want to keepalive (see dbus-mqtt)
const int  keepaliveInterval = 300000; // How often to publish the keep alive request (see dbus-mqtt)
const char deviceStatusTopicTemplate[] = "device/<client ID>/Status"; // Topic to send client status to and to regstister to the dbus-mqtt-device service
const char deviceInstanceTopicTemplate[] = "device/<client ID>/DeviceInstance"; // Topic to subscribe to so the dbus-mqtt-device service will provide an instance id to use when publishing to dbus-mqtt 
const char vebusModeTopicTemplate[] = "/<portal ID>/vebus/257/Mode"; // dbus-mqtt topic for handling the multiplus mode (inverter/charger onn/off), is prefixed with N, R or W at runtime
const char temperatureTopicTemplate[] = "/<portal ID>/temperature/<instance ID>/"; // dbus-mqtt topic for handling Temperator sensors, is prefixed with N or W at runtime
const int  tempReadingInterval = 50000; // How often to publish a temperature reading

const char tankTopicTemplate[] = "/<portal ID>/tank/<instance ID>/"; // dbus-mqtt topic for handling Temperator sensors, is prefixed with N or W at runtime
//const char tankLevelTopicTemplate[] = "/<portal ID>/tank/<instance ID>/Level"; // dbus-mqtt topic for handling the tank level


WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
int status = WL_IDLE_STATUS;     // the WiFi radio's status

String portalID = "";
int instanceID = -1;

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;          
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
// Temperature value
float temp;


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastKeepaliveTime = 0; // the last time a keepalive message was sent
unsigned long lastTempReadingTime = 0; // the last time a keepalive message was sent
//unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
//unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

//int buttonState;         // variable for reading the pushbutton status
//int buttonReading;
//int lastButtonReading = LOW;


void processTemperature() {
      // New temperature readings
    sensors.requestTemperatures(); 
    // Temperature in Celsius degrees
    temp = sensors.getTempCByIndex(0);
    // Temperature in Fahrenheit degrees
    //temp = sensors.getTempFByIndex(0);
    
    publishTemperature(temp);
    //serial.println(temp);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
     // connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
     // xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
     // xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

/* * * * * * *
 * S E T U P 
 * * * * * * */
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial && !headless) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Started.");

   // Start the DS18B20 sensor
  sensors.begin();

WiFi.onEvent(WiFiEvent);
 connectToWifi();
  //initialiseWifi();

  // initialize the LED pin as an output:
 // pinMode(ledPin, OUTPUT);
  
  // initialize the pushbutton pin as an input:
 // pinMode(buttonPin, INPUT);

  //initializeTemperature();
}

/* * * * * * *
 * L O O P 
 * * * * * * */
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi is not connected is lost, will attempt to reconnect in 30 seconds. WiFi status:"); Serial.println(WiFi.status());
    delay(30000);
    connectToWifi();
  }

  if (!mqttClient.connected()) {
    initialiseMQTT();
  } else {

    if (portalID != "") {

      // Handle Keepalive (see Victron dbus-mqtt) 
      if (millis() - lastKeepaliveTime >= keepaliveInterval) {
        lastKeepaliveTime = millis();       // save the last time a keepalive message was sent
        publishKeepalive();
      }

      if (instanceID > 0) {

        // Handle temperatures
        if ((millis() - lastTempReadingTime >= tempReadingInterval) || (lastTempReadingTime == 0)) {
          //byte mac[6];
         // WiFi.macAddress(mac);
          //Serial.print("MAC address: ");
         // printMacAddress(mac);
          lastTempReadingTime = millis();
          processTemperature();
        }
      }
    }

    mqttClient.poll();
  }

}

/* * * * * * * * * * * * * * * * *
 *   o n M q t t M e s s a g e 
 * * * * * * * * * * * * * * * * */
void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.print("Received a message on topic [");
  Serial.print(mqttClient.messageTopic());
  Serial.print("], length ");
  Serial.print(messageSize);
  Serial.println(" bytes");

  String topic = String(mqttClient.messageTopic());
  
  // Handle VRM Portal ID messages from dbus-mqtt
  if (topic.endsWith("/system/0/Serial") && messageSize > 0) {  // VRM Portal Id
  
    DynamicJsonDocument doc(messageSize+10); // {"value": "dca6328d3ea7"} see https://arduinojson.org/v6/assistant/
    DeserializationError err = deserializeJson(doc, mqttClient);
    if (err) {
      Serial.print("Error reading Portal ID from JSON; ");
      Serial.println(err.c_str());
      Serial.println();
      return;
    }
    doc.shrinkToFit();
    
    String newportalID = doc["value"].as<String>();
    if (portalID != newportalID) {
      portalID = newportalID;
      Serial.print("portalID: "); Serial.println(portalID); Serial.println();

      // subscribe to a Mode topic on the Vebus
      mqttClient.subscribe(vebusModeTopic("N"), 2);
      Serial.print("subscribe vebusModeTopic: "); Serial.println(vebusModeTopic("N")); Serial.println();
      // send a read request to dbus-mqtt
      mqttClient.beginMessage(vebusModeTopic("R"));
      mqttClient.print("");
      mqttClient.endMessage();    
      Serial.print("publish Read Request 'R' to vebusModeTopic: "); Serial.println(vebusModeTopic("R")); Serial.println();

    }
    
    return;
  }

  // Handle DeviceInstance messages from dbus-mqtt-devices
  if (topic == deviceInstanceTopic() && messageSize > 0) {
    DynamicJsonDocument doc(messageSize+20); // {"value": 100} see https://arduinojson.org/v6/assistant/
    DeserializationError err = deserializeJson(doc, mqttClient);
    if (err) {
      Serial.print("Error reading device instance JSON; ");
      Serial.println(err.c_str());
      Serial.println();
      return;
    }
    doc.shrinkToFit();
    
    instanceID = doc[serviceId];
    Serial.print("DeviceInstance: "); Serial.println(instanceID); Serial.println();

    return;
  }

   // Unexpected message so use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();

  Serial.println();
}

/* * * * * * *
 *  
 * * * * * * */
void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}



void initialiseMQTT() {
  
  // You can provide a unique client ID, if not set the library uses Arduino-millis()
  // Each client must have a unique client ID
  mqttClient.setId(clientId);

  // You can provide a username and password for authentication
  // mqttClient.setUsernamePassword("username", "password");

  // Set last will and testament
  mqttClient.beginWill(deviceStatusTopic(), false, 1);
  serializeJson(deviceStatusPayload(0), mqttClient);
  mqttClient.endWill();

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);


  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.print(mqttClient.connectError());
    Serial.print(", Wifi status = ");
    Serial.println(WiFi.status());

    delay(30000);
    return;
  }
  Serial.println("You're connected to the MQTT broker!"); Serial.println();

  Serial.print("Subscribing to Portal ID topic: "); Serial.println(portalIDTopic);
  mqttClient.subscribe(portalIDTopic, 2);   // subscribe to the main Portal ID topic

  Serial.print("Subscribing to Device Instance topic: "); Serial.println(deviceInstanceTopic());
  mqttClient.subscribe(deviceInstanceTopic(), 2);   

  mqttClient.beginMessage(deviceStatusTopic(), false, 1);
  serializeJson(deviceStatusPayload(1), mqttClient);
  mqttClient.endMessage();    

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);
}



/* * * * * * *
 *  MQTT Publish
 * * * * * * */
void publishKeepalive() {
  if (portalID != "") {
    Serial.print("publish to keepaliveTopic: ");
    Serial.println(keepaliveTopic());
    mqttClient.beginMessage(keepaliveTopic());
    mqttClient.print(keepalivePayload);
    mqttClient.endMessage();
  }
}

String keepaliveTopic() {
  String topic =  String(keepaliveTopicTemplate);
  topic.replace("<portal ID>", portalID);
  return topic;
}

String deviceStatusTopic() {
  String topic =  String(deviceStatusTopicTemplate);
  topic.replace("<client ID>", clientId);
  return topic;
}

DynamicJsonDocument deviceStatusPayload(int connected) {
  DynamicJsonDocument doc(1024);
  doc["clientId"] = clientId;
  doc["connected"] = connected;
  doc["version"] = VERSION;
  //JsonObject services = doc.createNestedObject("services");
 //services[serviceId] = "temperature, tank"; // Lowercase service type
  JsonObject services = doc.createNestedObject("services");
services[serviceId] = "temperature";
services[tankServiceId] = "tank";
  return doc;
}

String deviceInstanceTopic() {
  String topic =  String(deviceInstanceTopicTemplate);
  topic.replace("<client ID>", clientId);
  return topic;
}

String vebusModeTopic(String mode) {
  String topic =  String(vebusModeTopicTemplate);
  topic.replace("<portal ID>", portalID);
  return topic;
}

String temperatureTopic(String mode, String suffix) {
  String topic =  mode + String(temperatureTopicTemplate) + suffix;
  topic.replace("<portal ID>", portalID);
  topic.replace("<instance ID>", String(instanceID));
  return topic;
}

String tankTopic(String mode, String suffix) {
  String topic =  mode + String(tankTopicTemplate) + suffix;
  topic.replace("<portal ID>", portalID);
  topic.replace("<instance ID>", String(instanceID));
  return topic;
}

void publishTemperature(float temp) {
  Serial.println(); Serial.print("MQTT Connection status: "); Serial.print(mqttClient.connected());
  Serial.print(", WiFi status: "); Serial.println(WiFi.status());

  mqttClient.beginMessage(temperatureTopic("W", "Temperature"));
  mqttClient.print("{ \"value\":");
  mqttClient.print(temp);
  mqttClient.print("}");
  mqttClient.endMessage();    
  Serial.print("publish temperature ");
  Serial.print(temp);
  Serial.print(" to temperatureTopic: "); Serial.println(temperatureTopic("W", "Temperature")); 

   
  mqttClient.beginMessage(tankTopic("W", "Level"));
  mqttClient.print("{ \"value\":");
  mqttClient.print(Level);
  mqttClient.print("}");
  mqttClient.endMessage();    
  Serial.print("publish Tank ");
  Serial.print(Level);
  //Serial.print(" to tankTopic: "); Serial.println(tankTopic("W", "tank")); 
  Serial.print(" to tankTopic: "); Serial.println(tankTopic("W", "Level")); 

  
/*
  mqttClient.beginMessage(temperatureTopic("W", "Humidity"));
  mqttClient.print("{ \"value\":");
  mqttClient.print(humi);
  mqttClient.print("}");
  mqttClient.endMessage();    
  Serial.print("publish humidity ");
  Serial.print(humi);
  Serial.print(" to temperatureTopic: "); Serial.println(temperatureTopic("W", "Humidity")); 
  Serial.println();
*/
}

