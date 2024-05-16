#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_AHTX0.h>
#include <FlowSensor.h>

#define RGB_R  1
#define RGB_G 41
#define RGB_B 42

// Set your Board and Server ID 
#define BOARD_ID 1
#define MAX_CHANNEL 13  // for North America // 13 in Europe

#define SDA_ADC 37
#define SCL_ADC 38

#define SDA_1   11
#define SCL_1   12
#define I2C_ENB 13

#define flow_type YFS201
#define pin 6 // pin -> interrupt pin

const int AirValue    = 20000;   //you need to replace this value with Value_1
const int WaterValue  = 7500;  //you need to replace this value with Value_2

TwoWire I2C_ADC = TwoWire(0);
TwoWire I2C_1   = TwoWire(1);

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
Adafruit_AHTX0 aht;

FlowSensor Sensor(flow_type, pin);
unsigned long timebefore = 0; // Same type as millis()
unsigned long reset = 0;

uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

int dev_id = 0;
float temp = 1.0;
float humi = 2.0;
float soil_m = 3.0;
float flow_r = 4.0;
float flow_v = 5.0;
boolean pump_stat = false;
boolean valve_stat = false;
int r_id = 0;
String jsonPayload;

//Structure to send data
//Must match the receiver structure
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    uint8_t      msgType;
    uint8_t      id;
    String       jsonData;
    float        temperature;
    float        humidity;
    float        soil_moisture;
    float        flow_rate;
    float        flow_volume;
    unsigned int readingId;
} struct_message;

typedef struct __attribute__((packed)) struct_command {
    uint8_t      msgType;
    uint8_t      id;
    boolean      valve;
    boolean      pump;
} struct_command;

typedef struct __attribute__((packed)) struct_maintenance {
    uint8_t      msgType;
    uint8_t      id;
    char*        node_state;
    uint16_t     time;
} struct_maintenance;

typedef struct struct_pairing {       // new structure for pairing
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
} struct_pairing;

void IRAM_ATTR count()
{
  Sensor.count();
}

//Create 2 struct_message 
struct_message incomingReadings;
struct_message outgoingReadings;

struct_pairing pairingData;

struct_command incoming_command;

struct_maintenance incoming_maintenance;


enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType {PAIRING, DATA, COMMAND, MAINTENANCE};
MessageType messageType; 

#ifdef SAVE_CHANNEL
  int lastChannel;
#endif  
int channel = 1;
 
// simulate temperature and humidity data
float t = 0;
float h = 0;

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings
unsigned long start;                // used to measure Pairing time
unsigned int readingId = 0;   

// simulate reading

void readFlow(){
	if (millis() - timebefore >= 1000)
	{
		Sensor.read();
    flow_r = Sensor.getFlowRate_m();
    flow_v = Sensor.getVolume();
		timebefore = millis();
	}

  // Reset Volume
	if (millis() - reset >= 120000)
	{
		Sensor.resetVolume();
		reset = millis();
	}
}

void readAHT(){
  sensors_event_t humidity, temperature;
  aht.getEvent(&humidity, &temperature);
  temp = temperature.temperature;
  humi = humidity.relative_humidity;

  delay(500);
}

void readSoilmoisture(){
  int16_t adc1, adc2;
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  Serial.printf("adc1: %d, adc2: %d\n", adc1, adc2);
  int16_t adc_avg = (adc1 + adc2)/2;
  soil_m = map(adc_avg, AirValue, WaterValue, 0, 100);
  if (soil_m > 100)
    soil_m = 100;
  else if (soil_m < 0)
    soil_m = 0; 

    delay(500);
}


void setColor(int R, int G, int B) {
  analogWrite(RGB_R, R);
  analogWrite(RGB_G, G);
  analogWrite(RGB_B, B);
}

void rgb_connect(){
  setColor(0,0,0);
  delay(10);
  setColor(0,255,255);
}

void rgb_disconnect(){
  setColor(0,0,0);
  delay(10);
  setColor(255,165,0);
}

void rgb_received(){
  setColor(0,0,0);
  setColor(128,0,128);
}


void addPeer(const uint8_t * mac_addr, uint8_t chan){
  esp_now_peer_info_t peer;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if(status == ESP_NOW_SEND_SUCCESS)
  rgb_connect();
  else
  rgb_disconnect();
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  Serial.println();
  rgb_received();
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0];
  Serial.println(type);
  switch (type) {
  case DATA :                           // the message is data type
      { 
        // StaticJsonDocument<1000> JSON_data_buffer;
        // JsonObject dat_a = JSON_data_buffer.createNestedObject();
        memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
        dev_id = int(incomingReadings.id);
        jsonPayload = String(incomingReadings.jsonData);
        Serial.println(jsonPayload);
        Serial.println();
        break;
      }


  case MAINTENANCE :                           // the message is command type
      {
        memcpy(&incoming_maintenance, incomingData, sizeof(incoming_maintenance));
        char* node_state  = incoming_maintenance.node_state;
        uint16_t time = incoming_maintenance.time;
        if(node_state == "sleep"){
              Serial.println("SLEEPING!");
            }
        else if(node_state == "reset"){
              Serial.println("RESETTING!");
            }
        else Serial.println("MAINTENANCE ACTION FAILED!");
        Serial.println();
        break;
      }

  case PAIRING:    // we received pairing data from server
  {
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if (pairingData.id == 0) {              // the message comes from server
      printMAC(mac_addr);
      rgb_received();
      Serial.print("Pairing done for ");
      printMAC(pairingData.macAddr);
      Serial.print(" on channel " );
      Serial.print(pairingData.channel);    // channel used by the server
      Serial.print(" in ");
      Serial.print(millis()-start);
      Serial.println("ms");
      addPeer(pairingData.macAddr, pairingData.channel); // add the server  to the peer list 
      #ifdef SAVE_CHANNEL
        lastChannel = pairingData.channel;
        EEPROM.write(0, pairingData.channel);
        EEPROM.commit();
      #endif  
      pairingStatus = PAIR_PAIRED;             // set the pairing status
    }
    break;
  }
  }  
}

PairingStatus autoPairing(){
  switch(pairingStatus) {
    case PAIR_REQUEST:
    rgb_disconnect();
    Serial.print("Pairing request on channel "  );
    Serial.println(channel);

    // set WiFi channel   
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
    }

    // set callback routines
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
  
    // set pairing data to send to the server
    pairingData.msgType = PAIRING;
    pairingData.id = BOARD_ID;     
    pairingData.channel = channel;

    // add peer and send request
    addPeer(serverAddress, channel);
    esp_now_send(serverAddress, (uint8_t *) &pairingData, sizeof(pairingData));
    previousMillis = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
    // time out to allow receiving response from server
    rgb_connect();
    currentMillis = millis();
    if(currentMillis - previousMillis > 250) {
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel ++;
      if (channel > MAX_CHANNEL){
         channel = 1;
      }   
      pairingStatus = PAIR_REQUEST;
    }
    break;

    case PAIR_PAIRED:
      // nothing to do here 
      rgb_received();
    break;
  }
  return pairingStatus;
}  

void setup() {
  Serial.begin(115200);
  Serial.println();
  // pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  pinMode(I2C_ENB, OUTPUT);
  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);

  digitalWrite(I2C_ENB, HIGH);
  delay(100);
  Serial.println("Hello!");
  Sensor.begin(count);
  rgb_disconnect();
  delay(1000);
  rgb_received();
  delay(1000);
  rgb_connect();
  delay(1000);

  I2C_ADC.begin(SDA_ADC, SCL_ADC, 100000); 
  I2C_1.begin(SDA_1, SCL_1, 100000);

  delay(400);
  if (!aht.begin(&I2C_1, BOARD_ID, 0x38)) {
    Serial.println("Could not find AHT? Check wiring");
    // while (1) 
    delay(10000);
  }
  Serial.println("AHT10 or AHT20 found");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin(0x48, &I2C_ADC)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  start = millis();

  #ifdef SAVE_CHANNEL 
    EEPROM.begin(10);
    lastChannel = EEPROM.read(0);
    Serial.println(lastChannel);
    if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL) {
      channel = lastChannel; 
    }
    Serial.println(channel);
  #endif  
  pairingStatus = PAIR_REQUEST;
}  

void loop() {

  readAHT();
  readFlow();
  readSoilmoisture();
  delay(500);
  if (autoPairing() == PAIR_PAIRED) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println(temp);
      Serial.println(humi);
      Serial.println(soil_m);
      Serial.println(flow_r);
      Serial.println(flow_v);
      // Save the last time a new reading was published
      rgb_disconnect();
      previousMillis = currentMillis;
      outgoingReadings.msgType = DATA;
      outgoingReadings.id = BOARD_ID;
      outgoingReadings.jsonData = "AGRINET-ACK";
      outgoingReadings.temperature = temp;
      outgoingReadings.humidity = humi;
      outgoingReadings.soil_moisture = soil_m;
      outgoingReadings.flow_rate = flow_r;
      outgoingReadings.flow_volume = flow_v;
      outgoingReadings.readingId = readingId++;
      esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
    }
  }
}