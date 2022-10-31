
//=============================================================================//

#define WIZFI360_EVB_PICO

#ifdef WIZFI360_EVB_PICO
  #include <WizFi360.h>
#else
  #include <WiFi.h>
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ptScheduler.h>
#include <kaa.h>
#include <SPI.h>
#include <TFT_eSPI.h>
// #include <Adafruit_GFX.h>    // Core graphics library
// #include <Adafruit_ST7735.h> // Hardware-specific library for ST7735

//=============================================================================//

//Baudrate
#define SERIAL_BAUDRATE   115200  //USB
#define SERIAL2_BAUDRATE  115200  //Wi-Fi module

#define KAA_HOST "mqtt.cloud.kaaiot.com"
#define KAA_PORT 1883
#define KAA_TOKEN "<token>"      // Endpoint token - you get (or specify) it during device provisioning
#define KAA_APP_VERSION "<version>"  // Application version - you specify it during device provisioning

#define PIN_VAC     28  //GP28, D28, ADC2, AC voltage output
#define PIN_IAC     27  //GP27, D27, ADC1, AC current output
#define PIN_RELAY   17  //GP17, D17, Output relay
#define PIN_PB      16  //GP16, D16, Push button
#define PIN_LED_DBG    25  //GP25, D25
#define PIN_LED_OUT    21  //GP25, D25

// //LCD pins for reference. These must be defined in the TFTeSPI User Setup file.
// //Pico SPI1
// #define TFT_CS     7   // Chip select control pin
// #define TFT_DC     11  // Data Command control pin (RS pin)
// #define TFT_RST    10  // Reset pin (could connect to NodeMCU RST, see next line)
// #define TFT_MOSI   15
// #define TFT_MISO   12
// #define TFT_SCLK   14

//=============================================================================//

const char* ssid = "SSID";        // WiFi name
const char* password = "password";    // WiFi password
const char* broker = KAA_HOST;
const String TOKEN = KAA_TOKEN;        // Endpoint token - you get (or specify) it during device provisioning
const String APP_VERSION = KAA_APP_VERSION;  // Application version - you specify it during device provisioning

WiFiClient wizfiClient; //web client object
PubSubClient mqtt(wizfiClient); //MQTT object
Kaa kaa (&mqtt, KAA_TOKEN, KAA_APP_VERSION);  //KAA object

ptScheduler publishTask(PT_TIME_1S * 5);  //publishes data to the server
ptScheduler mqttLoopTask(PT_TIME_1S); //processes MQTT messages and states
ptScheduler lcdTask (PT_TIME_500MS);  //refreshes the LCD contents
ptScheduler adcTask (PT_TIME_100MS);  //samples the ADCs and convert the values

bool load_state = false;  //load ON/OFF state
bool last_load_state = true;  //previous load state
int pageIndex = 0;  //current page of the LCD

float vac = 0.0;  //RMS AC voltage
float iac = 0.0;  //RMS AC current
float pac = 0.0;  //RMS AC power

//LCD
TFT_eSPI LCD = TFT_eSPI();  // Invoke custom library

//=============================================================================//
//Initializes WizFi360 module.
//This has to be performed before initializing the Wi-Fi object.

void initWizFi360() {
  WiFi.init(&Serial2);

  // check for the presence of the module
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi module is not present");
    // don't continue
    while (true);
  }
}

//=============================================================================//
//This function sets up the Wi-Fi object and connects to the provided network.

void setupWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.println();
    Serial.printf("Connecting to [%s]", ssid);
    WiFi.begin(ssid, password);
    connectWiFi();
  }
}

//=============================================================================//
//Connects to the Wi-Fi network and prints the local IP address.

void connectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("Local IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Internet is connected");
}

//=============================================================================//
//Receives MQTT messages sent by the broker. 

void mqttCallback (char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  kaa.messageArrivedCallback(topic, (char*)payload, length);
}

//=============================================================================//
//This connects to the MQTT broker and sends the device meta data.

void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.println("Attempting MQTT connection...");
    char *client_id = "WizSocket";

    if (mqtt.connect(client_id)) {
      Serial.print("Connected to MQTT broker ");
      Serial.println(broker);

      kaa.connect();
      composeAndSendMetadata();
      // subscribeToCommand();
    } else {
      Serial.print("Connection failed, rc = ");
      Serial.print(mqtt.state());
      Serial.println(". Trying again in 3 seconds");
      delay(3000);
    }
  }
}

//=============================================================================//
//Creates a metadata object and sends it to the cloud.

void composeAndSendMetadata() {
  StaticJsonDocument<255> doc_data;

  if (load_state) {
    doc_data["load_state"] = "ON";
  } else {
    doc_data["load_state"] = "OFF";
  }

  kaa.sendMetadata(doc_data.as<String>().c_str());
}

//=============================================================================//
//This is the KAA IoT command callback.
//Whenever a command is sent from the dashboard, the message arrives here for
//processing.

int commandCallback (char* topic, char* payload, unsigned int length) {
  Serial.printf("\nHandling command message on topic: %s\n", topic);

  DynamicJsonDocument doc(1023);
  deserializeJson(doc, payload, length);
  JsonVariant json_var = doc.as<JsonVariant>();
  DynamicJsonDocument commandResponse(1023);

  for (int i = 0; i < json_var.size(); i++) {
    unsigned int command_id = json_var[i]["id"].as<unsigned int>();
    commandResponse.createNestedObject();
    commandResponse[i]["id"] = command_id;
    commandResponse[i]["statusCode"] = 200;
    commandResponse[i]["payload"] = "done";
  }

  if (strcmp(topic, "LOAD") == 0) {
    Serial.println("Command received to to toggle the Load");
    StaticJsonDocument<255> doc_data;
  
    if (load_state) {
      Serial.println("Toggling Load to OFF..");
      doc_data["load_state"] = "OFF";
      load_state = false;
      // clearOutput(PIN_RELAY)
      // clearOutput(PIN_LED_OUT)
    } else {
      Serial.println("Toggling Load to ON..");
      doc_data["load_state"] = "ON";
      load_state = true;
      // setOutput(PIN_RELAY)
      // setOutput(PIN_LED_OUT)
    }
    kaa.sendMetadata(doc_data.as<String>().c_str());
  }

  String responseTopic = "kp1/" + APP_VERSION + "/cex/" + TOKEN + "/result/LOAD";
  mqtt.publish(responseTopic.c_str(), commandResponse.as<String>().c_str());
  Serial.println("Published response to LOAD command on topic: " + responseTopic);
  return 0;
}

//=============================================================================//

void subscribeToCommand() {
  String topic = "kp1/" + APP_VERSION + "/cex/" + TOKEN + "/command/LOAD/status";
  mqtt.subscribe(topic.c_str());
  Serial.println("Subscribed on topic: " + topic);
}

//=============================================================================//

void setup() {
  Serial.begin(SERIAL_BAUDRATE);  //USB serial
  Serial2.begin(SERIAL2_BAUDRATE); //WizFi serial
  delay(2000);

  // // LCD.setSPISpeed(27000000);

  LCD.begin();
  LCD.setRotation(1);
  LCD.fillScreen(TFT_BLACK);
  // LCD.setFreeFont(&Roboto_Thin_24);
  // LCD.setFreeFont(&FreeSans9pt7b);
  // LCD.setFreeFont(&FreeSansBold12pt7b);
  // LCD.setFreeFont(&FreeSans18pt7b);
  LCD.setFreeFont(&FreeSansBold9pt7b);
  LCD.fillScreen(TFT_BLACK);
  LCD.setTextSize(1);
  LCD.setTextColor(TFT_YELLOW);
  LCD.setCursor(20, 65);
  LCD.print("WizSocket");
  
  pinMode(PIN_LED_DBG, OUTPUT);
  pinMode(PIN_LED_OUT, OUTPUT);
  pinMode(PIN_IAC, INPUT);
  pinMode(PIN_VAC, INPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_PB, INPUT_PULLUP);

  clearOutput(PIN_LED_DBG);
  clearOutput(PIN_LED_OUT);

  analogReadResolution(12);

  #ifdef WIZFI360_EVB_PICO
    initWizFi360();
  #endif

  //MQTT Broker setup
  mqtt.setServer(broker, KAA_PORT);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(3600);
  mqtt.setCallback(mqttCallback);
  kaa.setCommandCallback(&commandCallback);

  delay(2000);
}

//=============================================================================//

void loop() {
  setupWiFi();

  if (!getPbState()) {
    load_state = true;
  } else {
    load_state = false;
  }

  if (last_load_state != load_state) {
    if (load_state) {
      setOutput(PIN_RELAY);
      setOutput(PIN_LED_OUT);
      composeAndSendMetadata();
    } else {
      clearOutput(PIN_RELAY);
      clearOutput(PIN_LED_OUT);
      composeAndSendMetadata();
      
    }
    last_load_state = load_state;
  }

  if (adcTask.call()) {
    // printAdcValues();

    vac = getAcVoltage();
    // Serial.print("VAC:");
    // Serial.println(vac);

    if (load_state) {
      iac = getAcCurrent();
    } else {
      iac = 0.0;
    }
    // Serial.print("IAC:");
    // Serial.println(iac);
    pac = getAcPower();
  }

  if (!mqtt.connected()) {
    Serial.println("MQTT is not active. Trying to reconnect..");
    mqttReconnect();
  }

  if (mqttLoopTask.call()) {
    mqtt.loop();
  }

  if (publishTask.call()) {
    DynamicJsonDocument telemetry(1023);
    telemetry.createNestedObject();

    telemetry[0]["voltage"] = vac;
    telemetry[0]["current"] = iac;
    telemetry[0]["power"] = pac;

    if (load_state)
      telemetry[0]["load"] = 1;
    else {
      telemetry[0]["load"] = 0;
    }

    String topic = "kp1/" + APP_VERSION + "/dcx/" + TOKEN + "/json";
    mqtt.publish(topic.c_str(), telemetry.as<String>().c_str());
    Serial.println("Published on topic: " + topic);
  }

  if (lcdTask.call()) {
    if (pageIndex == 0) {
      LCD.setFreeFont(&FreeSansBold9pt7b);
      LCD.fillScreen(TFT_BLACK);
      LCD.setTextSize(1);
      LCD.setTextColor(TFT_WHITE);
      LCD.setCursor(20, 20);
      LCD.print("WizSocket");

      LCD.setFreeFont();
      LCD.setTextSize(1);

      LCD.setCursor(5, 40);
      LCD.setTextColor(TFT_YELLOW);
      // LCD.println("Voltage: 228 V");
      LCD.println("Voltage:");
      LCD.setCursor(55, 40);
      LCD.print(String(vac) + " V");

      LCD.setCursor(5, 55);
      LCD.setTextColor(TFT_CYAN);
      // LCD.println("Current: 4.54 A");
      LCD.println("Current:");
      LCD.setCursor(55, 55);
      LCD.print(String(iac) + " A");

      LCD.setCursor(5, 70);
      LCD.setTextColor(TFT_GREEN);
      // LCD.println("Power: 23.45 W");
      LCD.println("Power:");
      LCD.setCursor(55, 70);
      LCD.print(String(pac) + " W");

      LCD.setCursor(5, 85);
      LCD.setTextColor(TFT_GOLD);

      if (WiFi.status() != WL_CONNECTED) {
        LCD.println("WiFi: Disconnected");
      } else {
        LCD.println("WiFi: Connected");
      }

      LCD.setCursor(5, 100);
      LCD.setTextColor(TFT_MAGENTA);

      if (!mqtt.connected()) {
        LCD.println("MQTT: Disconnected");
      } else {
        LCD.println("MQTT: Connected");
      }
    }
  }
}

//=============================================================================//

int getPbState() {
  int pbState = digitalRead(PIN_PB);
  // Serial.print("Push Button State = ");
  // Serial.println(pbState);
  // Serial.println();
  return pbState;
}

//=============================================================================//

void setOutput (int pin) {
  digitalWrite(pin, HIGH);
}

//=============================================================================//

void clearOutput (int pin) {
  digitalWrite(pin, LOW);
}

//=============================================================================//

void printAdcValues() {
  float iacCount = analogRead(PIN_IAC);
  float vacCount = analogRead(PIN_VAC);

  float vacValue = vacCount * (3.3/4096.0);
  float iacValue = iacCount * (3.3/4096.0);

  Serial.print("VAC ADC Count = ");
  Serial.println(vacCount);
  Serial.print("VAC Voltage = ");
  Serial.print(vacValue);
  Serial.println(" V");
  Serial.print(vacValue);
  Serial.println(" V");

  Serial.print("IAC ADC Count = ");
  Serial.println(iacCount);
  Serial.print("IAC Voltage = ");
  Serial.print(iacValue);
  Serial.println(" V");
}

//=============================================================================//

float getAcVoltage() {
  uint32_t period = 1000000 / 50;
  uint32_t t_start = micros();  
  uint32_t Vsum = 0, sampleCount = 0;
  int32_t Vnow;

  while ((micros() - t_start) < period) {
    Vnow = analogRead(PIN_VAC) - 1870;
    Vsum += (Vnow * Vnow);
    sampleCount++;
  }

  float Vrms = sqrt(Vsum / sampleCount) * (3.3 / 4096.0) / 0.001;
  float adjustment = -16.72;
  return (Vrms + adjustment);
}

//=============================================================================//

float getAdcVoltage (int pin) {
  float count = analogRead(pin);
  float voltage = count * (3.3 / 4096.0);
  return voltage;
}

//=============================================================================//

float getAcCurrent() {
  uint32_t period = 1000000 / 5;
  uint32_t t_start = micros();
  uint32_t Isum = 0, sampleCount = 0;
  int32_t Inow;

  while ((micros() - t_start) < period) {
    Inow = analogRead(PIN_IAC) - 1828;
    Isum += (Inow * Inow);
    sampleCount++;
  }

  float Irms = sqrt(Isum / sampleCount) * (3.3 / 4096.0) / 0.1;
  return Irms + 0.08;
}

//=============================================================================//

float getAcPower() {
  // return getAcCurrent() * getAcVoltage();
  pac = vac * iac;
  return pac;
}

//=============================================================================//


