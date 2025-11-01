#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiServer.h>
#include <WiFiType.h>

#include <string>
#include <cstdio>
#include <iostream>
#include <PubSubClient.h>
#include <EEPROM.h>

#include "src/AnalogMultiplexer.h"
#include "src/PIDController.h"
#include "src/LineDriver.h"
#include "src/MicroServoSG90.h"
#include "src/Motor.h"
#include "src/UltrasonicSensor.h"

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

#define CONTROL_P_VAL (1.0f / 65.0f)
#define CONTROL_I_VAL 0
#define CONTROL_D_VAL 0

#define PID_TARGET_VAL (POS_RANGE_MAX / 2)

#define MOTOR_LEFT_INV 0
#define MOTOR_RIGHT_INV 1

// Ultrasonid sensor polling period in millis
#define US_SENSOR_POLL_PER_MSEC 60

// Pins
// =============================================
#define IR_ADC_PIN 32    // ADC1_4 (GPIO_32)
#define MPLEX_S0_PIN 27  // GPIO_27
#define MPLEX_S1_PIN 26  // GPIO_26
#define MPLEX_S2_PIN 25  // GPIO_25
#define MPLEX_S3_PIN 33  // GPIO_33

#define MOTOR_LEFT_PIN 22   // GPIO_22
#define MOTOR_RIGHT_PIN 23  // GPIO_23

#define ARM_SERVO_LEFT_PIN 5
#define ARM_SERVO_RIGHT_PIN 18
#define TIPPING_SERVO_PIN 19

#define LEFT_US_SENSOR_PIN 16
#define RIGHT_US_SENSOR_PIN 17

#define TEMP_TRIG_PIN RIGHT_US_SENSOR_PIN
#define TEMP_ECHO_PIN LEFT_US_SENSOR_PIN


// Types
// =============================================

// Flash storage type
struct SavedClientName {
  char magicChar;
  char name[MQTT_NAME_MAX_SIZE];
  uint8_t xorVal;
};

// State to poll ultrasonic sensors
enum UsSensorPollState {
  UsPollStateTrig,
  UsPollStateEcho,
  UsPollStateWait
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

AnalogMultiplexer analogMplex(MPLEX_S0_PIN, MPLEX_S1_PIN, MPLEX_S2_PIN, MPLEX_S3_PIN, IR_ADC_PIN);
PIDController pidController(PID_TARGET_VAL, CONTROL_P_VAL, CONTROL_I_VAL, CONTROL_D_VAL);

Motor leftMotor(MOTOR_LEFT_PIN, MOTOR_LEFT_INV);
Motor rightMotor(MOTOR_RIGHT_PIN, MOTOR_RIGHT_INV);

LineDriver lineDriver(&analogMplex, &pidController, &leftMotor, &rightMotor);

MicroServoSG90 leftArmServo(ARM_SERVO_LEFT_PIN);
MicroServoSG90 rightArmServo(ARM_SERVO_RIGHT_PIN);
MicroServoSG90 tippingServo(TIPPING_SERVO_PIN);

UltrasonicSensor leftUsSensor(TEMP_TRIG_PIN, TEMP_ECHO_PIN);
//UltrasonicSensor rightUsSensor(RIGHT_US_SENSOR_PIN);

void ARDUINO_ISR_ATTR leftUsSensorISR () {
  leftUsSensor.sensorISR();
}

// The ultrasonic sensors cannot be polled simultaneously, and so
// polling must alternate between them. Addtionally, polling must
// wait a minimum period after successful measurement or timeout
// to ensure no signal overlap.
uint8_t pollLeftUsSensor = true;
UsSensorPollState ultrasonicPollState = UsPollStateTrig;

float leftDistanceCm;
float rightDistanceCm;

// Time of last ultrasonic sensor trigger, to prevent sensor overlap
unsigned long lastUsSensorTrigMs;

RobotDriveState driveState = DriveStateIdle;

void handleMoodCommand (std::string cmd) {
  if (cmd == "happy") {
    Serial.println("Happy!! (=^-^=)");
  } else if (cmd == "angry") {
    Serial.println("Angry!!! (｡ •̀ ᴖ •́ ｡)");
  }
}

void handleActionCommand (std::string cmd) {
  if (cmd == "drop") {
    Serial.println("Dropping!");
  } else if (cmd == "spin") {
    Serial.println("Do a spin!");
  } else if (cmd == "die") {
    Serial.println("Dying (x_x)");
  } else if (cmd == "reset") {
    Serial.println("Reset!");
  } else if (cmd == "march") {
    Serial.println("Marching!	ᕕ( ᐛ )ᕗ");
    driveState = DriveStateDriving;
  } else if (cmd == "halt") {
    Serial.println("Halt!!!");
    driveState = DriveStateIdle;
    lineDriver.stop();
  } else if (cmd == "calib") {
    Serial.println("Calibrating...");
    lineDriver.calibrate();
  } else if (cmd == "step") {
    leftMotor.setSpeed(15);
    rightMotor.setSpeed(15);
    delay(3000);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
  } else if (cmd == "ping") {
    pinMode(TEMP_TRIG_PIN, OUTPUT);
    digitalWrite(TEMP_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TEMP_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TEMP_TRIG_PIN, LOW);
    delayMicroseconds(5);

    pinMode(TEMP_ECHO_PIN, INPUT);
    unsigned long duration = pulseIn(TEMP_ECHO_PIN, HIGH);
    float distanceCm = duration * 0.0343 / 2;
    Serial.printf("Distance: %.2f cm\n", distanceCm);
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
      delay(5000);
    }
  }
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

void handleUltrasonicSensors () {
  //UltrasonicSensor *targetUsSensor = (pollLeftUsSensor ? &leftUsSensor : &rightUsSensor);
  UltrasonicSensor *targetUsSensor = &leftUsSensor;  // TODO: use both again
  switch (ultrasonicPollState) {
    // Trigger State: trigger and jump to echo
    case UsPollStateTrig:
      targetUsSensor->trigger();
      lastUsSensorTrigMs = millis();  // store trigger time for wait period
      ultrasonicPollState = UsPollStateEcho;
      break;
    // Echo State: wait for update or timeout
    case UsPollStateEcho:
      if (targetUsSensor->hasUpdate()) {
        // If update available, store distance and jump to wait
        float distanceCm = targetUsSensor->getDistanceCm();
        if (pollLeftUsSensor) {
          leftDistanceCm = distanceCm;
        } else {
          rightDistanceCm = distanceCm;
        }
        ultrasonicPollState = UsPollStateWait;
      } else if (targetUsSensor->checkTimedOut()) {
        // If sensor timed out, jump to wait
        ultrasonicPollState = UsPollStateWait;
        // Serial.print("Timeout for ");
        // Serial.println(pollLeftUsSensor ? "left" : "right");
      }
      break;
    // Wait State: wait for polling period to end
    case UsPollStateWait:
      if ((millis() - lastUsSensorTrigMs) > US_SENSOR_POLL_PER_MSEC) {
        // Switch to other sensor and jump to trigger
        //pollLeftUsSensor = !pollLeftUsSensor;  // TODO: re-enable
        ultrasonicPollState = UsPollStateTrig;
      }
      break;
    default:
      Serial.print("Invalid ultrasonic poll state: ");
      Serial.println(int(ultrasonicPollState));
      while (true) {} // hang forever (triggering watchdog error)
  }
}

void checkObjectDetected () {
  if (leftDistanceCm > 0 && leftDistanceCm < 30) {
    if (driveState == DriveStateDriving) {
      driveState = DriveStateWaiting;
      lineDriver.stop();
    }
  } else {
    if (driveState == DriveStateWaiting) {
      driveState = DriveStateDriving;
    }
  }
}

void setup() {
  EEPROM.begin(EEPROM_SIZE);

  Serial.begin(115200);
  Serial.println("Alive");

  // Wifi Setup
  // ------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPassword);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
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
  
  leftArmServo.setPos(500, 0);

  UltrasonicSensor::registerISR(&leftUsSensor, &leftUsSensorISR);
}

void loop() {
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
  handleUltrasonicSensors();
  checkObjectDetected();
  //leftArmServo.update();
  delay(10); // TODO: Remove or shorten
}
