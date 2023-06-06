//
// Created by Сергей Тяпкин1 on 05.06.2023.
//

#define _CPP_COMPILE_UNIQUE_NAME _serial_joints_control
#include "CppCompileUtils/CppCompileUtils.h"
CPP_COMPILE_CPP_HEADER

#include "DynamixelManipulator/DynamixelManipulator.h"


#define STEP 20
#define SPEED_STEP 10.0  // DPS
#define SPEED_INIT 10.0  // DPS


#define JOINT_POS_MIN_DEG 0.0
#define JOINT_POS_MAX_DEG 360.0
const float minPoses[] = {
    JOINT_POS_MIN_DEG, // min 1
    73, // min 2
    83, // min 3
    78, // min 4
    3,  // min 5
    120, // min 6
};
const float maxPoses[] = {
    JOINT_POS_MAX_DEG, // max 1
    283, // max 2
    275, // max 3
    253, // max 4
    245, // max 5
    193, // max 6
};
const unsigned jointsIds[] = {1, 2, 3, 4, 5, 6};
#define JOINTS_COUNT 6



CPP_COMPILE_BEFORE_SETUP

DynamixelManipulator DM(JOINTS_COUNT, minPoses, maxPoses);
void setup() {
}

void loop() {
  DM.LOOP_UPDATE();

  char key;
  static bool isLedLighting = false;
  static Joint::id selectedJoint = 1;

  if (Serial.available()) {
    key = Serial.read(); // get pressed key from serial port
    Serial.print("KEY: ");
    Serial.println(key);
    Serial.print("CODE: ");
    Serial.println(int(key));

    Joint::posDeg currentJointPos = DM.realPositions[selectedJoint - 1];
    Joint::speedDPS currentJointSpeed = DM.realSpeeds[selectedJoint - 1];
    switch (key) {
      case 'l': // toggle LED state
        if (isLedLighting)
          digitalWrite(LED_BUILTIN, LOW);
        else
          digitalWrite(LED_BUILTIN, HIGH);
        isLedLighting = !isLedLighting;
        break;
      case 'w': // move joint in '+' direction
        DM.setJointPosDeg(selectedJoint, currentJointPos + STEP);
        break;
      case 's': // move joint in '-' direction
        DM.setJointPosDeg(selectedJoint, currentJointPos - STEP);
        break;
      case 'e': // enable joint torque
        DM.enableJoint(selectedJoint);
        break;
      case 'd': // disable joint torque
        DM.disableJoint(selectedJoint);
        break;
      case '=': // plus speed limit
        DM.setJointSpeedDPS(selectedJoint, currentJointSpeed + SPEED_STEP);
        break;
      case '-': // minus speed limit
        DM.setJointSpeedDPS(selectedJoint, currentJointSpeed - SPEED_STEP);
        break;
      case 'Q':
        return 0;
      default:
        if (key > '0' && key <= '0' + JOINTS_COUNT) { // select joint by index
          selectedJoint = key - '0';
          Serial.print("You select joint ");
          Serial.println(selectedJoint);
        }
      }
    }

    delay(20);
}
CPP_COMPILE_AFTER_LOOP
