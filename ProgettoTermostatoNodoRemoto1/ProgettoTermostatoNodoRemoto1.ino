/*********************  ESP8688 CLIENT 1 NODO REMOTO  ********************************/

#include <NullSpaceLib.h>
#ifdef ESP8266 
       #include <ESP8266WiFi.h>
#endif 
#ifdef ESP32   
       #include <WiFi.h>
#endif
#include "SinricPro.h"
#include "SinricProTemperaturesensor.h"
#include "SinricProLight.h"
#include <ArduinoJson.h> //configuratore: https://arduinojson.org/v6/assistant/#/step1
//DHT sensor library:1.4.4
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h> //Adafruit RTClib per DS3231 e DateTime
#include <ArduinoWebsockets.h> //Downloading ArduinoWebsockets@0.5.3
#include <Adafruit_NeoPixel.h>

#define REMOTE_NODE_ID 1

#define DHTPIN 2      // Digital pin connected to the DHT sensor GPIO2=D4
// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)
DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t g_dht_event;
TimerC timer_sampling_temp_hum;
uint32_t sampling_time_temp_hum;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(5, 5, NEO_GRB + NEO_KHZ800); //4 leds gpio3=RX

const char* ssid = "ssid"; //Enter SSID
const char* password = "pass"; //Enter Password
const char* websockets_server_host = "192.xxx.xxx.xxx"; //Enter server adress
const uint16_t websockets_server_port = 8080; // Enter server port
#define APP_KEY           "app_key"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "app_secret"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define TEMP_SENSOR_ID    "temp_sensor_id"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define LIGHT_ID          "Light_id"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define EVENT_WAIT_TIME   60000               // send event every 60 seconds
TimerC timer_syncpro_send;
class DataHolder
{
  public:
    DataHolder()
    {
      this->led_color = Adafruit_NeoPixel::Color(255,255,255);
      this->led_brightness = 100; //range 0-100
    }
    float node_temperature;
    float node_humidity;
    bool deviceTemperatureIsOn;
    bool deviceLedIsOn;
    uint8_t led_brightness;
    uint32_t led_color;
};
DataHolder g_DataHolder;

bool first_run = true;

//WEBSOCKET CLIENT
using namespace websockets;
TimerC timer_websocket_client_send;
#define sampling_time_websocket_client_send 1000 //ms
TimerC timer_websocket_client_rcv;
#define sampling_time_websocket_client_rcv 500 //ms
WebsocketsClient client;

void ConnectWiFi()
{
  //CONNESSIONE AL WIFI
  WiFi.begin(ssid, password);
  //TENTATIVI DI CONNESSIONE
  int counter = 0;
  while (!WiFi.isConnected()) 
  {
    delay(200);
    Serial.println("WiFi Non Connesso...Aspetta");    
    if (++counter > 100)
    {
      //SE NON SI CONNETTE DOPO 100 TENTATIVI, RESETTA ESP 
      ESP.restart();
    }
     
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());   //You can get IP address assigned to ESP
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro

  //DHT11 SENSOR
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  mySensor.onPowerState(onPowerStateTemperatureSensor);

  //LEDS  
  SinricProLight &myLight = SinricPro[LIGHT_ID];
  // set callback function to device
  myLight.onPowerState(onPowerStateLed);
  myLight.onBrightness(onBrightnessLed);
  myLight.onAdjustBrightness(onAdjustBrightnessLed);
  myLight.onColor(onColorLed);

  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  //SinricPro.restoreDeviceStates(true); // Uncomment to restore the last known state from the server.
  SinricPro.begin(APP_KEY, APP_SECRET);  
}
bool onPowerStateTemperatureSensor(const String &deviceId, bool &state) {
  Serial.printf("Temperaturesensor turned %s (via SinricPro) \r\n", state?"on":"off");
  g_DataHolder.deviceTemperatureIsOn = state; // turn on / off temperature sensor
  return true; // request handled properly
}
bool onPowerStateLed(const String &deviceId, bool &state) {
  Serial.printf("onPowerStateLed turned %s (via SinricPro) \r\n" , state?"on":"off");
  Serial.printf("g_DataHolder.led_color %d (via SinricPro) \r\n", g_DataHolder.led_color);
  Serial.printf("g_DataHolder.led_brightness %d (via SinricPro) \r\n", g_DataHolder.led_brightness);
  g_DataHolder.deviceLedIsOn = state;
  if (state) {
    for(uint16_t i=0; i<strip.numPixels(); i++) 
    {
      strip.setPixelColor(i, g_DataHolder.led_color);
    }
    strip.setBrightness(map(g_DataHolder.led_brightness, 0, 100, 0, 255));
  } else {
    strip.clear();
  }
  strip.show();
  return true; // request handled properly
}
bool onColorLed(const String &deviceId, byte &r, byte &g, byte &b) {
   g_DataHolder.led_color = strip.Color(r,g,b);
   if(!g_DataHolder.deviceLedIsOn) return false;
   for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, g_DataHolder.led_color);
  }
  strip.show();
  return true;
}
bool onBrightnessLed(const String &deviceId, int &brightness) {
  if(!g_DataHolder.deviceLedIsOn) return false;
  g_DataHolder.led_brightness=brightness;
  for(uint16_t i=0; i<strip.numPixels(); i++) 
  {
    strip.setPixelColor(i, g_DataHolder.led_color);
  }
  strip.setBrightness(map(g_DataHolder.led_brightness, 0, 100, 0, 255));
  strip.show();
  return true;
}
bool onAdjustBrightnessLed(const String &deviceId, int &brightnessDelta) {
  if(!g_DataHolder.deviceLedIsOn) return false;
  g_DataHolder.led_brightness += brightnessDelta;
  brightnessDelta = g_DataHolder.led_brightness; //update global brightness
  for(uint16_t i=0; i<strip.numPixels(); i++) 
  {
    strip.setPixelColor(i, g_DataHolder.led_color);
  }
  strip.setBrightness(map(g_DataHolder.led_brightness, 0, 100, 0, 255));
  strip.show();
  return true;
}

void setup() {
  Serial.begin(115200);
  ConnectWiFi();
  WiFi.setAutoReconnect(true);

  strip.begin();
  strip.setBrightness(255); //[0-255]
  strip.clear(); // metti ad off tutti i leds
  strip.show(); // Initialize all pixels to 'off'

  // Initialize device.
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
  sampling_time_temp_hum = sensor.min_delay / 1000; //ms

  connectWSClientToServer();

  setupSinricPro();

  timer_sampling_temp_hum.start();
  timer_websocket_client_send.start();
  timer_websocket_client_rcv.start();
  timer_syncpro_send.start();

}

void loop() 
{
  SinricPro.handle();
  bool ok = read_temp_hum_node_station(g_dht_event);
  if(ok)
  {
    clientWSSendDataToServer();
    sendDataToSyncPro();
  }

  clientCheckForIncomingMsgs();
}


bool read_temp_hum_node_station(sensors_event_t& dht_event)
{
  if(timer_sampling_temp_hum.getET()>=sampling_time_temp_hum)
  {
    //digitalWrite(PIN_RELE,!digitalRead(PIN_RELE));
    timer_sampling_temp_hum.reset();
    dht.temperature().getEvent(&dht_event);
    if (isnan(dht_event.temperature)) {
      //Serial.println(F("Error reading temperature!"));
      return false;
    }
    else {
      g_DataHolder.node_temperature = dht_event.temperature;

      //Serial.print(F("Temperature: "));
      //Serial.print(g_dht_event.temperature);
      //Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&dht_event);
    if (isnan(dht_event.relative_humidity)) {
      //Serial.println(F("Error reading humidity!"));
      return false;
    }
    else {
      g_DataHolder.node_humidity = dht_event.relative_humidity;
      //Serial.print(F("Humidity: "));
      //Serial.print(g_dht_event.relative_humidity);
      //Serial.println(F("%"));      
    }
  }
  return true;
}

void connectWSClientToServer()
{
    // try to connect to Websockets server
    bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
    if(connected) {
        Serial.println("Connecetd!");
        //client.send("Hello Server");
    } else {
        Serial.println("Not Connected!");
    }
    
    // run callback when messages are received
    client.onMessage([&](WebsocketsMessage message) {
        Serial.print("Got Message: ");
        Serial.println(message.data());
    });
}

void clientWSSendDataToServer()
{
  if(timer_websocket_client_send.getET() >= sampling_time_websocket_client_send)
  {
    timer_websocket_client_send.reset();
    //send data over websocket to server
    //TODO: BETTER TO USE JSON
    //I USE MY COSTUM STRING PROTOCOL
    char msg[20];
    sprintf(msg,"!%d,%s,%d", REMOTE_NODE_ID,String(g_DataHolder.node_temperature,1),static_cast<int>(g_DataHolder.node_humidity));
    client.send(msg);
    
  }
}

void sendDataToSyncPro()
{
  if(timer_syncpro_send.getET()>=EVENT_WAIT_TIME || first_run)
  {
    first_run = false;
    timer_syncpro_send.reset();
    if(!g_DataHolder.deviceTemperatureIsOn)
      return;    
    SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // get
    bool success = mySensor.sendTemperatureEvent(g_DataHolder.node_temperature, g_DataHolder.node_humidity); // send event
    if (success) {  // if event was sent successfuly, print temperature and humidity to serial
      Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.0f%%\r\n", g_DataHolder.node_temperature, g_DataHolder.node_humidity);
    } else {  // if sending event failed, print error message
      Serial.printf("Something went wrong...could not send Event to server!\r\n");
    }
  }
}

void clientCheckForIncomingMsgs()
{
  if(timer_websocket_client_rcv.getET() >= sampling_time_websocket_client_rcv)
  {
    timer_websocket_client_rcv.reset();
    // let the websockets client check for incoming messages
    if(client.available()) {
        client.poll();
    }
  }
}
