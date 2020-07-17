#include <Arduino.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <Logger.h>
#include <ArduinoJson.h>
#include <FS.h>

//OTA related libraries
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>


const char *configPath = "/config.json";  
struct Config {
    bool using_congfig_file;
    char device_name[20];
    char version[8];
    char ap_ssid[64];
    char ap_ip[20];
    char ap_gateway[20];
    char ap_mask[20];
};
Config config;                         // <- global configuration object
const char compile_date[] = __DATE__ " " __TIME__;

//Servos
#define PIN_SERVO_PAN 14  //D5
#define PIN_SERVO_TILT 12 //D6
#define DUTY_MIN 20
#define DUTY_MAX 250

uint8 servoPinList[2] = {PIN_SERVO_PAN, PIN_SERVO_TILT};

//Laser
#define PIN_LASER 13  //D7

void printConfigFile(const char *filename) {
  // Open file for reading
  File file = SPIFFS.open(filename, "r");
  if (!file) {
    logger(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  while (file.available()) logger(file.readString());
  file.close();
}

void loadConfiguration(const char *filename, Config &config) {
    File file = SPIFFS.open(filename, "r");
    StaticJsonDocument<512> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error)
        logger(F("Failed to read config file, using default configuration"));

    // Copy values from the JsonDocument to the Config
    strlcpy(config.version,           // <- destination
            doc["version"] | "0.0",   // <- source
            sizeof(config.version));  // <- destination's capacity
    strlcpy(config.device_name, doc["device_name"] | "No Name (Default)", sizeof(config.device_name));          
    strlcpy(config.ap_ssid, doc["ap_ssid"] | "ESP Thing (Default)", sizeof(config.ap_ssid));  
    strlcpy(config.ap_ip, doc["ap_ip"] | "10.0.1.1", sizeof(config.ap_ip));  
    strlcpy(config.ap_gateway, doc["ap_gateway"] | "10.0.1.1", sizeof(config.ap_gateway));  
    strlcpy(config.ap_mask, doc["ap_path"] | "255.255.255.0", sizeof(config.ap_mask));  
}


void blinkLaser(int repetitions=3, int pause=100){
  for(int i=0; i<repetitions; i++) {
    digitalWrite(PIN_LASER, HIGH);
    delay(pause);
    digitalWrite(PIN_LASER, LOW);
    delay(pause);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_SERVO_PAN, OUTPUT);
  pinMode(PIN_SERVO_TILT, OUTPUT);

  //Mount File system
  if (SPIFFS.begin()) logger("FS mounted at setup");
  else logger("Error mounting FS at setup");

  logger("\n\n      -----------START----------");
  logger("| Compilation Date:      |");
  logger(compile_date);
  logger("|        OTA Active      |");
  logger("|          Setup         |");

  //Logger status
  logger("  Initialize Current Log file: " + String(clearCurrentLogFile()));

  //Load config
  logger(F("  Print config file..."));
  printConfigFile(configPath);
  logger(F("  Loading configuration..."));
  loadConfiguration(configPath, config);

  //WiFiManager
  WiFiManager wifiManager;
  
  //reset saved settings
  //wifiManager.resetSettings();
  
  //set custom ip for portal
  IPAddress ip, gw, mask;
  ip.fromString(config.ap_ip);
  gw.fromString(String(config.ap_gateway));
  mask.fromString(String(config.ap_mask));
  
  wifiManager.setAPStaticIPConfig(ip, gw, mask);

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect(config.ap_ssid);


  logger("  " + String(config.device_name) + ": " + String(config.version));
  logger("  Hardware MAC: " + String(WiFi.macAddress()));
  logger("  Software MAC: " + WiFi.softAPmacAddress());
  logger("  IP: " + WiFi.localIP().toString());  
  
  //OTA stuff
  ArduinoOTA.setHostname(config.device_name);
  logger("  Hostname: " + ArduinoOTA.getHostname());

  ArduinoOTA.onStart([]() {
    logger("\n*************************\n****** OTA STARTED ******");
  });

  ArduinoOTA.onEnd([]() {
    logger("\n*************************\n****** OTA FINISHED ******");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("*** OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: \n", error);
    if (error == OTA_AUTH_ERROR) logger("OTA Error: Auth Failed");
    else if (error == OTA_BEGIN_ERROR) logger("OTA Error: Begin Failed");
    else if (error == OTA_CONNECT_ERROR) logger("OTA Error: Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) logger("OTA Error: Receive Failed");
    else if (error == OTA_END_ERROR) logger("OTA Error: End Failed");
  });

  ArduinoOTA.begin();

  //Servo stuff
  analogWrite(PIN_SERVO_PAN, 512);
  analogWrite(PIN_SERVO_TILT, 512);
  analogWriteFreq(100);  /* Set PWM frequency to 50Hz */

  blinkLaser();

  logger("-------Setup Finished-------");
}

void moveServo (uint8 servoID, uint8 position){
  //convert 0-180 to duty
  uint16_t posDuty = map(position, 0, 180, DUTY_MIN, DUTY_MAX);
  //Move Servo to that position
  analogWrite(servoPinList[servoID], posDuty); 
}

void movePanTilt(uint8 pan, uint8 tilt) {
  moveServo(0, pan);
  moveServo(1, tilt);
}

void scanAll(bool laser = true, int pause = 10) {
  digitalWrite(PIN_LASER, laser);
  for (uint8 angle = 0; angle < 180; angle++) {
    movePanTilt(angle, angle);
    delay(pause);
      ArduinoOTA.handle();
  }

  for (uint8 angle = 180; angle > 0; angle--) {
    movePanTilt(angle, angle);
    delay(pause);
      ArduinoOTA.handle();
  }
}

void loop() {
  ArduinoOTA.handle();
  scanAll(true, 100);
  blinkLaser();
}