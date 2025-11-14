#include <FastLED.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiType.h>

#include <string>
#include <cstdio>
#include <iostream>
#include <PubSubClient.h>
#include <EEPROM.h>

#include "src/AnalogMultiplexer.h"
#include "src/PIDController.h"
#include "src/LineDriver.h"
#include "src/HS311Servo.h"
#include "src/MG90SServo.h"
#include "src/Motor.h"
#include "src/SharpIrSensor.h"
#include "src/Feet.h"
#include "src/LoopTimer.h"

// For secrets like:
//   STASSID
//   STAPSK
//   MQTT_SERVER
//   MQTT_USERNAME
//   MQTT_PASSWORD
//   MQTT_PORT
//
#include "Credentials.h"


// General Config
// =============================================
#define EEPROM_SIZE 100
#define EEPROM_MAGIC_CHAR '$'

#define MQTT_TOPIC_MAX_SIZE 30
#define MQTT_NAME_MAX_SIZE 20
#define MQTT_ID_MAX_SIZE (MQTT_NAME_MAX_SIZE + 5)

#define CONTROL_P_VAL (1.0f / 30.0f)
#define CONTROL_I_VAL (1.0f / 1000.0f)
#define CONTROL_D_VAL 0

#define PID_TARGET_VAL (POS_RANGE_MAX / 2)

#define MOTOR_LEFT_INV 0
#define MOTOR_RIGHT_INV 0

// Distance threshold at which a blockage is detected
#define BLOCKAGE_DETECT_THRESH_CM 20

//#define CLK_PIN   4
#define LED_TYPE WS2812B
#define LED_COLOR_ORDER GRB
#define LED_COUNT 20
#define LED_BRIGHTNESS 50

// Pins
// =============================================
#define LINE_IR_ADC_PIN 32    // ADC1_4 (GPIO_32)
#define MPLEX_S0_PIN 27  // GPIO_27
#define MPLEX_S1_PIN 26  // GPIO_26
#define MPLEX_S2_PIN 25  // GPIO_25
#define MPLEX_S3_PIN 33  // GPIO_33

#define MOTOR_LEFT_PIN 22   // GPIO_22
#define MOTOR_RIGHT_PIN 23  // GPIO_23

#define FOOT_SERVO_LEFT_PIN 5
#define FOOT_SERVO_RIGHT_PIN 18
#define TIPPING_SERVO_PIN 19

#define LEFT_IR_SENSOR_PIN 35  // ADC1_7
#define RIGHT_IR_SENSOR_PIN 34  // ADC1_6

#define LED_DATA_PIN 21
#define RANDOM_SEED_PIN 36  // ADC1_0, floating


// Types
// =============================================

// Flash storage type
struct SavedClientName {
  char magicChar;
  char name[MQTT_NAME_MAX_SIZE];
  uint8_t xorVal;
};

// State for robot driving
enum RobotDriveState {
  DriveStateIdle,    // Not driving
  DriveStateDriving, // Driving
  DriveStateWaiting  // Waiting for blockage to clear
};


// Variables
// =============================================
const char *wifiSSID = STASSID;
const char *wifiPassword = STAPSK;

// MQTT topics general to all robots
const char *mqttTopicGenMood = "robots/mood";      // Set the mood
const char *mqttTopicGenAction = "robots/action";  // Run an action
const char *mqttTopicGenReq = "robots/req";        // Request from all
const char *mqttTopicGenRsp = "robots/rsp";        // Response from all
const char *mqttTopicGenMotors = "robots/motors";  // Set motors to specific speed

// MQTT topics that affect self only - use client id
char mqttTopicSelfMood[MQTT_TOPIC_MAX_SIZE];    // robots/<client_id>/mood
char mqttTopicSelfAction[MQTT_TOPIC_MAX_SIZE];  // robots/<client_id>/action

// MQTT topic to request data from a specific robot
char mqttTopicSelfReq[MQTT_TOPIC_MAX_SIZE];     // robots/<client_id>/req
// MQTT topic to respond with requested data
char mqttTopicSelfRsp[MQTT_TOPIC_MAX_SIZE];     // robots/<client_id>/rsp

// Robot-specific client ID
char mqttClientName[MQTT_NAME_MAX_SIZE];
char mqttClientId[MQTT_ID_MAX_SIZE];

// Flag to indicate the need to save MQTT client name to flash. Writing
// to flash is not recommended during an interrupt service routine, so this
// is not directly done from the MQTT message handler, and instead done
// in the main loop.
uint8_t requiresClientNameSave = false;


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

AnalogMultiplexer analogMplex(MPLEX_S0_PIN, MPLEX_S1_PIN, MPLEX_S2_PIN, MPLEX_S3_PIN, LINE_IR_ADC_PIN);
PIDController pidController(PID_TARGET_VAL, CONTROL_P_VAL, CONTROL_I_VAL, CONTROL_D_VAL);

Motor leftMotor(MOTOR_LEFT_PIN, MOTOR_LEFT_INV);
Motor rightMotor(MOTOR_RIGHT_PIN, MOTOR_RIGHT_INV);

LineDriver lineDriver(&analogMplex, &pidController, &leftMotor, &rightMotor);

MG90SServo leftFootServo(FOOT_SERVO_LEFT_PIN);
MG90SServo rightFootServo(FOOT_SERVO_RIGHT_PIN);
HS311Servo tippingServo(TIPPING_SERVO_PIN);

Feet feet(&leftFootServo, &rightFootServo);

SharpIrSensor leftIrSensor(LEFT_IR_SENSOR_PIN);
SharpIrSensor rightIrSensor(RIGHT_IR_SENSOR_PIN);

float leftDistanceCm;
float rightDistanceCm;

// Time of last ultrasonic sensor trigger, to prevent sensor overlap
unsigned long lastUsSensorTrigMs;

RobotDriveState driveState = DriveStateIdle;

CRGB leds[LED_COUNT];

CRGB happyColor;

void pickHappyColor() {
  int choice = random(0, 6);
  switch (choice) {
    case 0:
      happyColor = CRGB::Blue;
      break;
    case 1:
      happyColor = CRGB::Purple;
      break;
    case 2:
      happyColor = CRGB::Orange;
      break;
    case 3:
      happyColor = CRGB::Cyan;
      break;
    case 4:
      happyColor = CRGB::Pink;
      break;
    case 5:
      happyColor = CRGB::Green;
      break;
    default:
      happyColor = CRGB::Purple;
  }
}

void setHappy () {
  fill_solid(leds, LED_COUNT, happyColor);
  FastLED.show();
}

void setAngry () {
  fill_solid(leds, LED_COUNT, CRGB::Red);
  FastLED.show();
}

void setDead () {
  FastLED.clear();
  FastLED.show();
}

void spinny () {
  uint8_t left = random(2);
  int startIrVal = analogMplex.sample(8);
  if (left) {
    leftMotor.setSpeed(-25);
    rightMotor.setSpeed(20);
  } else {
    leftMotor.setSpeed(20);
    rightMotor.setSpeed(-25);
  }
  delay(500);
  // 180 degress
  while(abs(analogMplex.sample(8) - startIrVal) > 300) {}
  delay(500);
  // 360 degrees
  while(abs(analogMplex.sample(8) - startIrVal) > 300) {}
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}

void reset () {
  driveState = DriveStateIdle;
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  feet.stopWalking();
  tippingServo.setPos(0);
  setHappy();
}

void tipOver () {
  // Pull blocking, ensuring full movement
  tippingServo.setPos(1000, BLOCKING);
  // Reset nonblocking, as timing not critical
  tippingServo.setPos(0, NONBLOCKING);
}

void halt () {
  driveState = DriveStateIdle;
  lineDriver.stop();
  feet.stopWalking();
}

void handleMoodCommand (std::string cmd) {
  if (cmd == "happy") {
    Serial.println("Happy!! (=^-^=)");
    setHappy();
  } else if (cmd == "angry") {
    Serial.println("Angry!!! (｡ •̀ ᴖ •́ ｡)");
    setAngry();
  }
}

void handleActionCommand (std::string cmd) {
  if (cmd == "drop") {
    Serial.println("Dropping!");
  } else if (cmd == "spin") {
    Serial.println("Do a spin!");
    spinny();
  } else if (cmd == "die") {
    Serial.println("Dying (x_x)");
    tipOver();
    halt();
    setDead();
  } else if (cmd == "reset") {
    Serial.println("Reset!");
    reset();
  } else if (cmd == "march") {
    Serial.println("Marching!	ᕕ( ᐛ )ᕗ");
    driveState = DriveStateDriving;
    feet.startWalking();
  } else if (cmd == "halt") {
    Serial.println("Halt!!!");
    halt();
  } else if (cmd == "calib") {
    Serial.println("Calibrating...");
    lineDriver.calibrate();
  } else if (cmd == "step") {
    leftMotor.setSpeed(15);
    rightMotor.setSpeed(15);
    delay(1000);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
  }
}

void handleReqCommand (std::string cmd, uint8_t isGeneral) {
  std::string response;
  if (cmd == "name") {
    response = "name=";
    response += mqttClientName;
  } else if (cmd == "hi") {
    response = "hi!!!";
  } else if ((cmd.substr(0, 8) == "setname=") && !isGeneral) {
    if ((cmd.size() > 8) && (cmd.size() < (8 + MQTT_NAME_MAX_SIZE-1))) {
      char buffer[20 + MQTT_NAME_MAX_SIZE];
      strlcpy(mqttClientName, cmd.substr(8).c_str(), MQTT_NAME_MAX_SIZE);
      // Set flag so main loop can write name to flash
      requiresClientNameSave = true;
      reconfigMqttTopics();
      snprintf(buffer, sizeof(buffer), "set name: %s", mqttClientName);
      response = buffer;
    } else {
      response = "name out of range";
    }
  } else {
    response = "invalid cmd";
  }
  const char *destTopic = (isGeneral ? mqttTopicGenRsp : mqttTopicSelfRsp);
  mqttClient.publish(destTopic, response.c_str(), true);
  Serial.print("Responded to ");
  Serial.print(destTopic);
  Serial.print(": ");
  Serial.println(response.c_str());
}

void handleMotorsCommand (std::string cmd) {
  if (cmd[2] != '=') {
    return;
  }
  std::string codeArg = cmd.substr(0, 2);
  std::string numArg = cmd.substr(3);
  try {
    int num = std::stoi(numArg);

    uint8_t setLeft = ((codeArg[0] == 'L') ? 1 : 0);
    uint8_t setRight = ((codeArg[0] == 'R') ? 1 : 0);
    if (codeArg[0] == 'M') {
      setLeft = 1;
      setRight = 1;
    }

    if (codeArg[1] == 'D') {
      if (setLeft) {
        leftMotor.setPulseWidth(num);
        Serial.print("Set left motor pulse width to ");
        Serial.println(num);
      }
      if (setRight) {
        rightMotor.setPulseWidth(num);
        Serial.print("Set right motor pulse width to ");
        Serial.println(num);
      }
    } else if (codeArg[1] == 'S') {
      if (setLeft) {
        leftMotor.setSpeed(num);
        Serial.print("Set left motor speed to ");
        Serial.println(num);
      }
      if (setRight) {
        rightMotor.setSpeed(num);
        Serial.print("Set right motor speed to ");
        Serial.println(num);
      }
    }
  } catch (const std::invalid_argument& e) {
    Serial.println("Invalid argument: Could not convert string to int.");
  } catch (const std::out_of_range& e) {
    Serial.println("Out of range: Converted value is too large or small for int.");
  }
}

void blinkLED () {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char debugStr[100];
  std::string topicStr(topic);
  std::string msgStr = "";
  for (int i = 0; i < length; i++) {
    msgStr += (char)payload[i];
  }

  snprintf(debugStr, sizeof(debugStr), "Message arrived on [%s]: %s", topic, msgStr.c_str());
  Serial.println(debugStr);

  if (topicStr == mqttTopicGenMood || topicStr == mqttTopicSelfMood) {
    handleMoodCommand(msgStr);
  } else if (topicStr == mqttTopicGenAction || topicStr == mqttTopicSelfAction) {
    handleActionCommand(msgStr);
  } else if (topicStr == mqttTopicGenReq) {
    handleReqCommand(msgStr, true);
  } else if (topicStr == mqttTopicSelfReq) {
    handleReqCommand(msgStr, false);
  } else if (topicStr == mqttTopicGenMotors) {
    handleMotorsCommand(msgStr);
  }
  blinkLED();
}

void initMqttTopicNames () {
  snprintf(mqttClientId, sizeof(mqttClientId), "kin-%s", mqttClientName);
  snprintf(mqttTopicSelfMood, sizeof(mqttTopicSelfMood), "robots/%s/mood", mqttClientId);
  snprintf(mqttTopicSelfAction, sizeof(mqttTopicSelfAction), "robots/%s/action", mqttClientId);
  snprintf(mqttTopicSelfReq, sizeof(mqttTopicSelfReq), "robots/%s/req", mqttClientId);
  snprintf(mqttTopicSelfRsp, sizeof(mqttTopicSelfRsp), "robots/%s/rsp", mqttClientId);
}

void reconfigMqttTopics () {
  mqttClient.unsubscribe(mqttTopicSelfMood);
  mqttClient.unsubscribe(mqttTopicSelfAction);
  mqttClient.unsubscribe(mqttTopicSelfReq);
  initMqttTopicNames();
  mqttClient.subscribe(mqttTopicSelfMood);
  mqttClient.subscribe(mqttTopicSelfAction);
  mqttClient.subscribe(mqttTopicSelfReq);
}

void mqttConnect () {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(mqttClientId, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT server");
      mqttClient.subscribe(mqttTopicGenMood);
      mqttClient.subscribe(mqttTopicGenAction);
      mqttClient.subscribe(mqttTopicGenReq);
      mqttClient.subscribe(mqttTopicGenMotors);
      
      mqttClient.subscribe(mqttTopicSelfMood);
      mqttClient.subscribe(mqttTopicSelfAction);
      mqttClient.subscribe(mqttTopicSelfReq);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(", trying again in 5 seconds");   // Wait 5 seconds before retrying
      delay(2500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(2500);
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

uint8_t computeXor(char *name) {
  uint8_t calcXor = 0;
  for (int i = 0; i < MQTT_NAME_MAX_SIZE; i++) {
    if (name[i] == '\0') {
      break;
    }
    calcXor = calcXor ^ (uint8_t)name[i];
  }
  return calcXor;
}

uint8_t loadClientName () {
  SavedClientName entry;
  EEPROM.get(0, entry);
  // Step 1 - check magic character matches, reducing chances of random data load
  if (entry.magicChar != EEPROM_MAGIC_CHAR) {
    return 0;
  }

  // Step 2 - compute and check xor, further reducing error
  if (computeXor(entry.name) != entry.xorVal) {
    return 0;
  }
  strncpy(mqttClientName, entry.name, MQTT_NAME_MAX_SIZE);
  return 1;
};

void saveClientName () {
  SavedClientName newEntry;
  newEntry.magicChar = EEPROM_MAGIC_CHAR;
  strncpy(newEntry.name, mqttClientName, MQTT_NAME_MAX_SIZE);
  newEntry.xorVal = computeXor(mqttClientName);
  EEPROM.put(0, newEntry);
  EEPROM.commit();
}

void checkObjectDetected () {
  float leftDistanceCm = leftIrSensor.getDistanceCm();
  float rightDistanceCm = rightIrSensor.getDistanceCm();
  uint8_t leftBlocked = (leftDistanceCm > 0 && leftDistanceCm < BLOCKAGE_DETECT_THRESH_CM);
  uint8_t rightBlocked = (rightDistanceCm > 0 && rightDistanceCm < BLOCKAGE_DETECT_THRESH_CM);
  if (leftBlocked || rightBlocked) {
    if (driveState == DriveStateDriving) {
      driveState = DriveStateWaiting;
      lineDriver.stop();
      feet.stopWalking();
    }
  } else {
    if (driveState == DriveStateWaiting) {
      driveState = DriveStateDriving;
      feet.startWalking();
    }
  }
}

void setup() {
  EEPROM.begin(EEPROM_SIZE);

  randomSeed(analogRead(RANDOM_SEED_PIN));

  Serial.begin(115200);
  Serial.println("Alive");

  // Wifi Setup
  // ------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPassword);
  // Wait for connection
  uint8_t ledState = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, ledState);
    ledState = !ledState;
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(wifiSSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // MQTT Setup
  // ------------------------------------------
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  
  if (!loadClientName()) {
    strncpy(mqttClientName, String(random(0xffff), HEX).c_str(), MQTT_NAME_MAX_SIZE);
  }
  initMqttTopicNames();

  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, LED_COLOR_ORDER>(leds, LED_COUNT)
    .setCorrection(TypicalLEDStrip)
    .setDither(1);
  
  FastLED.setBrightness(LED_BRIGHTNESS);

  pickHappyColor();
  
  reset();
}

void loop() {
  loopTimer.update();
  if (!mqttClient.connected()) {
    mqttConnect();
  }
  if (requiresClientNameSave) {
    saveClientName();
    Serial.println("Saved client name to flash.");
    requiresClientNameSave = false;
  }
  mqttClient.loop();
  if (driveState == DriveStateDriving) {
    lineDriver.drive();
  }
  checkObjectDetected();
  leftMotor.handleDisengage();
  rightMotor.handleDisengage();
  leftFootServo.update();
  rightFootServo.update();
  feet.update();
}
