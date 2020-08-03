#include <Arduino.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <Logger.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <headers.h>

//OTA related libraries
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

//General variables
  #define ON 1
  #define OFF 0 

//Servos
#define PIN_SERVO_PAN D5
#define PIN_SERVO_TILT D6
#define DUTY_MIN 20
#define DUTY_MAX 250
#define SERVO_PAN_ID 0
#define SERVO_TILT_ID 1
uint8 servoPinList[2] = {PIN_SERVO_PAN, PIN_SERVO_TILT};

//Laser
#define PIN_LASER 13  //D7

//Button
#define PIN_BUT_TRIGGER_SEQUENCE D3
bool previousButtonState = true;

//Config stuff
const char *configPath = "/config.json";  
struct Config {
    bool using_congfig_file;
    char device_name[20];
    char version[8];
    char ap_ssid[64];
    char ap_ip[20];
    char ap_gateway[20];
    char ap_mask[20];

    int osc_device_id;
    char osc_server_ip[20];
    int osc_port_out;
    int osc_port_in;
    char osc_path[12];
    char osc_timer_path[8];
    char osc_enable_path[8];

    //Keep Alive
    char osc_keep_alive_path[sizeof(osc_path) + 15];
    char osc_keep_alive_enable_path[sizeof(osc_keep_alive_path) + 8];
    char osc_keep_alive_timer_path[sizeof(osc_keep_alive_path) + 8];
    int osc_keep_alive_enable;
    unsigned int osc_keep_alive_timer;

    //Laser
    char osc_laser_path[sizeof(osc_path) + 8];
    char osc_laser_enable_path[sizeof(osc_laser_path) + 8];
    int osc_laser_enable;

    //Pan and Tilt
    char osc_pt_path[sizeof(osc_path) + 8];
    char osc_pt_x_path[sizeof(osc_pt_path) + 8];
    char osc_pt_y_path[sizeof(osc_pt_path) + 8];
    char osc_pt_xy_path[sizeof(osc_pt_path) + 8];
    int osc_pt_x;
    int osc_pt_y;

    //Sequences
    char osc_sequence_path[sizeof(osc_path) + 8];
    int sequence_playing;
    int sequence_x_min;
    int sequence_x_max;
    int sequence_y_min;
    int sequence_y_max;

    //MQTT
    char mqtt_server[30];
    char mqtt_keep_alive_topic[8];
    char mqtt_sequence_topic[8];
};
Config config;                         
const char compile_date[] = __DATE__ " " __TIME__;

//WiFi
WiFiUDP udp;
IPAddress broadcastIP;
#define CLEAR_WIFI_SETTINGS false  //Change this to restore the WiFi Settings


//OSC stuff
OSCErrorCode error;
bool keepAliveToggle = false;

//Timers
unsigned long previousLoopTimer;

//MQTT
WiFiClient espClient;
PubSubClient client(espClient);

/********TURRET RELATED FUNCTIONS ***********/
/*FUNCTION: Turns Laser on and off
  - ARGS: 
    -- bool state
  - RETURN: 
    -- void 
*/
void laser(bool _state){
  digitalWrite(PIN_LASER, _state);
  config.osc_laser_enable = _state;
}
/*FUNCTION: Toggles the laser a number of times with specific intervals
  - ARGS: 
    -- uint8 repetitions: Number of times to blink
    -- uint8 pause: Pause between states
  - RETURN: 
    -- void 
*/
void blinkLaser(uint8 repetitions=3, uint8 pause=100){
  for(int i=0; i<repetitions; i++) {
    laser(ON);
    delay(pause);
    laser(OFF);
    delay(pause);
  }
}
/*FUNCTION: Moves the specified servo to a given degree
  - ARGS: 
    -- uint8 servoID: ID of the servo to move. in general just 0 (pan) or 1 (tilt)
    -- uint8 position: Position to move in degrees (0 - 180)

  - NOTES: 
    -- This function calls regularChecks()
*/
void moveServo (uint8 servoID, uint8 position){
  uint16_t posDuty = map(position, 0, 180, DUTY_MIN, DUTY_MAX); //Convert degrees to duty
  analogWrite(servoPinList[servoID], posDuty);  //Move the servo with the calculated duty 
  //Update position variables for later reporting
  if (servoID == SERVO_PAN_ID) {config.osc_pt_x = position;} 
  else if (servoID == SERVO_TILT_ID) {config.osc_pt_y = position;}
  //report position via OSC
  oscSendPT();
  regularChecks();  //Since moving a servo is something we do very frequently, we want to make sure this function allows for some regular checks
}
/*FUNCTION: Moves both servos at a given speed
  - ARGS: 
    -- uint8 pan: position of the PAN servo in degrees (0 - 180)
    -- uint8 tilt: position of the TILT servo in degrees (0 - 180)
    -- uint8 delay: pause between each moving step. This slows down and smothes the movements
*/
void movePT (uint8 _pan, uint8 _tilt, uint8 _delay) {
  logger("Moving PT @ " + String(_delay) + "ms per step");
  int pDelta = config.osc_pt_x - _pan;
  int tDelta = config.osc_pt_y - _tilt;

  logger("  P: " + String(config.osc_pt_x) + " --> " + String(_pan) + "     pDelta: " + String(pDelta) + " steps");
  logger("  T: " + String(config.osc_pt_y) + " --> " + String(_tilt) + "     tDelta: " + String(tDelta) + " steps");

  while((_pan != config.osc_pt_x) || (_tilt != config.osc_pt_y)) {
    if (_pan > config.osc_pt_x) {moveServo(SERVO_PAN_ID, ++config.osc_pt_x);}
    else if (_pan < config.osc_pt_x) {moveServo(SERVO_PAN_ID, --config.osc_pt_x);}

    if (_tilt > config.osc_pt_y) {moveServo(SERVO_TILT_ID, ++config.osc_pt_y);}
    else if (_tilt < config.osc_pt_y) {moveServo(SERVO_TILT_ID, --config.osc_pt_y);}
    delay(_delay);
  }
}
/*FUNCTION: Initiates a specific sequence
  - ARGS: 
    -- int sequenceID: ID of the sequence to be triggered
    -- int pause: Pause in between sequence steps
  - NOTES:
    -- You can define your own sequences of P&T and Laser here
*/ 
void triggerSequence(int sequenceID, int pause=500) {
  uint8 moveDelay = 100;
  logger("Trigger sequence initiated with ID: " + String(sequenceID));

  if (config.sequence_playing == sequenceID) {
    //logger("  Sequence " + String(sequenceID) + " is already playing");
    return;}

  else{
    config.sequence_playing = sequenceID;

    if(sequenceID == 0) {
      logger("  Stopping sequence.");
    }

    else if (sequenceID == 1){
      logger("  Sequence 1 - Starting");

      digitalWrite(PIN_LASER, HIGH);
      movePT(163, 152, moveDelay);
      delay(pause*2);
      movePT(106, 109, moveDelay);
      delay(pause);
      movePT(106, 138, moveDelay);
      delay(pause);
      movePT(106, 156, moveDelay);
      delay(pause);
      movePT(86, 152, moveDelay);
      delay(pause);
      movePT(74, 149, moveDelay);
      delay(pause);
      movePT(71, 161, moveDelay);
      delay(pause);
      movePT(70, 167, moveDelay);
      delay(pause);
      movePT(77, 169, moveDelay);
      delay(pause);
      movePT(82, 171, moveDelay);
      delay(pause);
      movePT(88, 165, moveDelay);
      delay(pause);
      movePT(97, 154, moveDelay);
      delay(pause);
      movePT(101, 113, moveDelay);
      delay(pause*5);
      digitalWrite(PIN_LASER, LOW);
    }

    else if (sequenceID == 2){
      logger("  Sequence 2 - Started");
      movePT(config.sequence_x_min, config.sequence_y_min, 5);
      blinkLaser();
      delay(500);

      movePT(config.sequence_x_max, config.sequence_y_min, 5);
      blinkLaser();
      delay(500);

      movePT(config.sequence_x_max, config.sequence_y_max, 5);
      blinkLaser();
      delay(500);

      movePT(config.sequence_x_min, config.sequence_y_max, 5);
      blinkLaser();
      delay(500);

      while(config.sequence_playing == 2){
        laser(ON);
        delay(random(500, 3000));
        movePT(random(config.sequence_x_min, config.sequence_x_max), random(config.sequence_y_min, config.sequence_y_max), random(30, 130));
        blinkLaser();
      }

      laser(OFF);
      logger("  Sequence 2 - Stopped by user");
      return;
    }

  laser(OFF);
  config.sequence_playing = 0;
  logger("  Sequence finished");
  }
}
void stopSequence(){
  if(config.sequence_playing) {triggerSequence(0);}
}

/******* CONFIGURATION FUNCTIONS **********/
void printConfigFile(const char *filename) {
  File file = SPIFFS.open(filename, "r");
  if (!file) {
    logger(F("Failed to read file"));
    return;
  }

  while (file.available()) logger(file.readString());
  file.close();
}
/*FUNCTION: Loads the information contained in configuration file into its corresponding variables.
            In case there is no configuration files, or a specific variable is not found in it, 
            a default value will be loaded.
  - ARGS: 
    -- const char *fileName: Pointer to the file containing the configuration json data.
                              This file must be uploaded separately using SPIFFS. See data/config.json
    -- Config &config: Address pointing to the configuration object defined at the beginning.
*/
void loadConfiguration(const char *fileName, Config &config) {
    File file = SPIFFS.open(fileName, "r");
    StaticJsonDocument<1024> doc;

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
    config.osc_port_out = doc["port_out"] | 9000; 
    config.osc_port_in = doc["port_in"] | 8000; 

    //General OSC Paths
    strlcpy(config.osc_path, doc["osc_path"] | "/default", sizeof(config.osc_path));
    strlcpy(config.osc_enable_path, doc["osc_enable_path"] | "/enable", sizeof(config.osc_enable_path));
    strlcpy(config.osc_timer_path, doc["osc_timer_path"] | "/timer", sizeof(config.osc_timer_path));

    //Keep Alive Paths
    strlcpy(config.osc_keep_alive_path, config.osc_path, sizeof(config.osc_keep_alive_path));
      strlcat(config.osc_keep_alive_path, doc["keep_alive_path"] | "/keepAlive", sizeof(config.osc_keep_alive_path));
    strlcpy(config.osc_keep_alive_enable_path, config.osc_keep_alive_path, sizeof(config.osc_keep_alive_enable_path));
      strlcat(config.osc_keep_alive_enable_path, config.osc_enable_path, sizeof(config.osc_keep_alive_enable_path));
    strlcpy(config.osc_keep_alive_timer_path, config.osc_keep_alive_path, sizeof(config.osc_keep_alive_timer_path));
      strlcat(config.osc_keep_alive_timer_path, config.osc_timer_path, sizeof(config.osc_keep_alive_timer_path));

    //Laser Paths
    strlcpy(config.osc_laser_path, config.osc_path, sizeof(config.osc_laser_path));
      strlcat(config.osc_laser_path, doc["laser_path"] | "/laser", sizeof(config.osc_laser_path));
    strlcpy(config.osc_laser_enable_path, config.osc_laser_path, sizeof(config.osc_laser_enable_path));
      strlcat(config.osc_laser_enable_path, config.osc_enable_path, sizeof(config.osc_laser_enable_path));

    //Pan and Tilt Paths
    strlcpy(config.osc_pt_path, config.osc_path, sizeof(config.osc_pt_path));
      strlcat(config.osc_pt_path, doc["pt_path"] | "/pt", sizeof(config.osc_pt_path));
    strlcpy(config.osc_pt_x_path, config.osc_pt_path, sizeof(config.osc_pt_x_path));
      strlcat(config.osc_pt_x_path, doc["pt_x_path"] | "/x", sizeof(config.osc_pt_x_path));
    strlcpy(config.osc_pt_y_path, config.osc_pt_path, sizeof(config.osc_pt_y_path));
      strlcat(config.osc_pt_y_path, doc["pt_y_path"] | "/y", sizeof(config.osc_pt_y_path));
    strlcpy(config.osc_pt_xy_path, config.osc_pt_path, sizeof(config.osc_pt_xy_path));
      strlcat(config.osc_pt_xy_path, doc["pt_xy_path"] | "/xy", sizeof(config.osc_pt_xy_path));

    //Sequence Paths
    strlcpy(config.osc_sequence_path, config.osc_path, sizeof(config.osc_sequence_path));
      strlcat(config.osc_sequence_path, doc["sequence_path"] | "/seq", sizeof(config.osc_sequence_path));
    config.sequence_x_min = doc["sequence_x_min"] | 0;
    config.sequence_x_max = doc["sequence_x_max"] | 180;
    config.sequence_y_min = doc["sequence_y_min"] | 0;
    config.sequence_y_max = doc["sequence_y_max"] | 180;

    config.osc_keep_alive_enable = doc["keep_alive_enable"] | 1;
    config.osc_keep_alive_timer = doc["keep_alive_timer"] | 1000;

    //MQTT
    strlcpy(config.mqtt_server, doc["mqtt_server"] | "broker.mqtt-dashboard.com", sizeof(config.mqtt_server));
    strlcpy(config.mqtt_keep_alive_topic, doc["mqtt_keep_alive_topic"] | "ka", sizeof(config.mqtt_keep_alive_topic));
    strlcpy(config.mqtt_sequence_topic, doc["mqtt_sequence_topic"] | "sq", sizeof(config.mqtt_sequence_topic));
}

/******** OSC FUNCTIONS ********/
void oscSend(char path[30], double value){
    OSCMessage oscSender(path);
    oscSender.add(value);
    udp.beginPacket(broadcastIP, config.osc_port_out);
    oscSender.send(udp);
    if (udp.endPacket() != 1)     logger(" Warning 21: OSC Sending Message Failed");
    oscSender.empty();
}
void printOscPaths() {
  char paths[sizeof(config)+(30*7)+(10*38)];
  strcpy(paths, ("\nGeneral OSC paths:\n "));
  strcat(paths, config.osc_path);           strcat(paths, ("\n   ..."));
  strcat(paths, config.osc_enable_path);    strcat(paths, ("\n   ..."));
  strcat(paths, config.osc_timer_path);     strcat(paths, ("\n   ..."));

  strcat(paths, ("\nKeep Alive Paths:\n "));
  strcat(paths, config.osc_keep_alive_path);        strcat(paths, ("\n   "));
  strcat(paths, config.osc_keep_alive_enable_path); strcat(paths, ("\n   "));
  strcat(paths, config.osc_keep_alive_timer_path);  strcat(paths, ("\n"));

  strcat(paths, ("\nLaser Paths:\n "));
  strcat(paths, config.osc_laser_path);        strcat(paths, ("\n   "));
  strcat(paths, config.osc_laser_enable_path); strcat(paths, ("\n   "));

  strcat(paths, ("\nPT Paths:\n "));
  strcat(paths, config.osc_pt_path);        strcat(paths, ("\n   "));
  strcat(paths, config.osc_pt_x_path); strcat(paths, ("\n   "));
  strcat(paths, config.osc_pt_y_path); strcat(paths, ("\n   "));
  strcat(paths, config.osc_pt_xy_path); strcat(paths, ("\n   "));

  strcat(paths, ("\nSequence Paths:\n "));
  strcat(paths, config.osc_sequence_path);        strcat(paths, ("\n   "));

  logger(paths);
}

void oscKeepAlive(OSCMessage &msg) {  //Response to incoming keep alive
  int ka = (int)msg.getFloat(0);
}
void oscEnableKeepalive(OSCMessage &msg) {
  config.osc_keep_alive_enable = msg.getInt(0);
}
void oscSendKeepAlive() {
    oscSend(config.osc_keep_alive_path, keepAliveToggle);
}
void oscSetKeepaliveTimer(OSCMessage &msg) {
  config.osc_keep_alive_timer = msg.getInt(0);
  logger("Keep Alive timer set to: " + String(config.osc_keep_alive_timer));
} 

void oscEnableLaser(OSCMessage &msg) {
  stopSequence();
  config.osc_laser_enable = msg.getInt(0);
  digitalWrite(PIN_LASER, config.osc_laser_enable);
  logger("Laser updated via OSC: " + String(config.osc_laser_enable));
}
void oscSendLaser() {
  oscSend(config.osc_laser_path, digitalRead(PIN_LASER));
}

void oscSetPTx(OSCMessage &msg) {
  stopSequence();
  config.osc_pt_x = msg.getInt(0);
  logger("PT set to X: " + String(config.osc_pt_x));
  moveServo(0, config.osc_pt_x);
}
void oscSetPTy(OSCMessage &msg) {
  stopSequence();
  config.osc_pt_y = msg.getInt(0);
  logger("PT set to Y: " + String(config.osc_pt_y));
  moveServo(1, config.osc_pt_y);
}
void oscSetPTxy(OSCMessage &msg) {
  stopSequence();
  config.osc_pt_x = msg.getInt(0);
  config.osc_pt_y = msg.getInt(1);
  logger("PT set to XY: " + String(config.osc_pt_x) + ", " + String(config.osc_pt_y));
  moveServo(0, config.osc_pt_x);
  moveServo(1, config.osc_pt_y);
}
void oscSendPT(){
    oscSend(config.osc_pt_x_path, config.osc_pt_x);
    oscSend(config.osc_pt_y_path, config.osc_pt_y);
}

void oscTriggerSequence(OSCMessage &msg) {
  int seq = msg.getInt(0);
  logger("Triggering Sequence via OSC: " + String(seq));
  triggerSequence(seq, 1000);
}

void processOSC(OSCMessage &msg){
  char address[30];
  msg.getAddress(address);
  Serial.print("OSC msg: ");
  Serial.print("  Address: ");
  Serial.println(address);
  Serial.print("  Size: ");
  Serial.println(msg.size());
  Serial.print("  Data Length: ");
  Serial.println(msg.getDataLength(0));

  for (int j=0; j<msg.size(); j++) {
    Serial.printf("   Message %i:\n", j);
    for (int i=0; i<msg.getDataLength(j); i++) {
      Serial.printf("     %i Data type: %c    Value: %f\n", i, msg.getType(i), msg.getFloat(i));
    }
  }
 }
void oscCheckForMsg(){
  OSCMessage msg;
  int size = udp.parsePacket();

  if (size > 0) {
    while (size--) {
      msg.fill(udp.read());
    }
    if (!msg.hasError()) {
      msg.dispatch(config.osc_keep_alive_path, oscKeepAlive);
      msg.dispatch(config.osc_keep_alive_enable_path, oscEnableKeepalive);
      msg.dispatch(config.osc_keep_alive_timer_path, oscSetKeepaliveTimer);

      msg.dispatch(config.osc_laser_enable_path, oscEnableLaser);
      msg.dispatch(config.osc_pt_x_path, oscSetPTx);
      msg.dispatch(config.osc_pt_y_path, oscSetPTy);
      msg.dispatch(config.osc_pt_xy_path, oscSetPTxy);
      msg.dispatch(config.osc_sequence_path, oscTriggerSequence);
    }
  }
}

/******** BUTTON FUNCTIONS ********/
void checkTriggerButton(){
  bool currentButtonState = digitalRead(PIN_BUT_TRIGGER_SEQUENCE);
  if (currentButtonState != previousButtonState) {  //Only act on state change
    previousButtonState = currentButtonState;
    if (!currentButtonState) {  //Button press detected
      logger("      Button has been pressed");
      if (config.sequence_playing == 0) {
        logger("Trigger button has been pressed - STARTING SEQUENCE");
        triggerSequence(2);
      }
      else {
        logger("Trigger button has been pressed - STOPPING Sequence");
        triggerSequence(0);
      }
    }
    delay(500);
  }
}

/******** MQTT FUNCTIONS ********/
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  logger("MQTT Message arrived on subscribed topic: " + String(topic));
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();

  if ((char)payload[0] == '0') {  //If payload is a '0', Stop the sequence
    logger("sq received a 0");
    stopSequence(); 
  }

  else {  //If payload is not '0', use payload as a Sequence ID and trigger the corresponding sequence
    triggerSequence(payload[0] - '0');
  }
}
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client- LASERCAT";

    // Attempt to connect and subscribe
    if (client.connect(clientId.c_str(), "mqtt", "mqtt")) {
      Serial.println("connected");
      client.publish(config.mqtt_keep_alive_topic, "on");
      client.subscribe(config.mqtt_sequence_topic);
    } 
    
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/******** OTHER FUNCTIONS ********/
void regularChecks(){
  ArduinoOTA.handle();
  oscCheckForMsg();
  checkTriggerButton();
  if (!client.connected()) connectMQTT();
  client.loop();

  //Keep Alive update
  if (config.osc_keep_alive_enable && 
      (millis() > (previousLoopTimer + config.osc_keep_alive_timer))) {
    oscSendKeepAlive();  //Send Keep alive periodically
    client.publish(config.mqtt_keep_alive_topic, "on");
    keepAliveToggle = !keepAliveToggle;
    previousLoopTimer = millis();
  }
}
void setup() {
  Serial.begin(115200);
  
  //Pin setup
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_SERVO_PAN, OUTPUT);
  pinMode(PIN_SERVO_TILT, OUTPUT);
  pinMode(PIN_BUT_TRIGGER_SEQUENCE,  INPUT_PULLUP);

  //Mount File system
  if (SPIFFS.begin()) logger("FS mounted at setup");
  else logger("Error mounting FS at setup");

  //Log initialization
  logger("\n\n      -----------START----------");
  logger("| Compilation Date:      |");
  logger(compile_date);
  logger("|        OTA Active      |");
  logger("|          Setup         |");
  logger("  Initialize Current Log file: " + String(clearCurrentLogFile()));

  //Load configuration
  logger(F("  Contents of the config file:"));
  printConfigFile(configPath);
  logger(F("  Loading configuration..."));
  loadConfiguration(configPath, config);

  //WiFiManager
  WiFiManager wifiManager;
  
 //Reset saved settings if needed
  if (CLEAR_WIFI_SETTINGS) {
    logger("Wifi settings have been cleared!");
    wifiManager.resetSettings();
  }
  
  //Custom ip for portal setup
  IPAddress ip, gw, mask;
  ip.fromString(config.ap_ip);
  gw.fromString(String(config.ap_gateway));
  mask.fromString(String(config.ap_mask));
  wifiManager.setAPStaticIPConfig(ip, gw, mask);

  //Fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect(config.ap_ssid);

  //Logs the WiFi connection results
  logger("  " + String(config.device_name) + ": " + String(config.version));
  logger("  Hardware MAC: " + String(WiFi.macAddress()));
  logger("  Software MAC: " + WiFi.softAPmacAddress());
  logger("  IP: " + WiFi.localIP().toString());  
  broadcastIP = WiFi.localIP();
  broadcastIP[3] = 255;
  logger("  Broadcast IP: " + broadcastIP.toString());
  
  //Initiate OSC port
  udp.begin(config.osc_port_in);
  logger("  OSC Listening on port: " + String(config.osc_port_in));
  printOscPaths();

  //OTA stuff
  ArduinoOTA.setHostname(config.device_name);
  logger("  Hostname: " + ArduinoOTA.getHostname());
  ArduinoOTA.onStart([]() {logger("\n*************************\n****** OTA STARTED ******");});
  ArduinoOTA.onEnd([]() {logger("\n*************************\n****** OTA FINISHED ******");});
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {Serial.printf("*** OTA Progress: %u%%\r", (progress / (total / 100)));});
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: \n", error);
    if (error == OTA_AUTH_ERROR) logger("OTA Error: Auth Failed");
    else if (error == OTA_BEGIN_ERROR) logger("OTA Error: Begin Failed");
    else if (error == OTA_CONNECT_ERROR) logger("OTA Error: Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) logger("OTA Error: Receive Failed");
    else if (error == OTA_END_ERROR) logger("OTA Error: End Failed");
  });
  ArduinoOTA.begin();

  //Initialize Servos
  analogWrite(PIN_SERVO_PAN, 512);
  analogWrite(PIN_SERVO_TILT, 512);
  analogWriteFreq(50);  /* Set PWM frequency to 50Hz */

  //Initialize MQTT client
  client.setServer(config.mqtt_server, 1883);
  client.setCallback(callbackMQTT);
  
  //Finish setup
  blinkLaser();
  logger("-------Setup Finished-------");
}
void loop() {
  regularChecks();
}