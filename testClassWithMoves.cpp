#define _CPP_COMPILE_UNIQUE_NAME _serial_move_by_serial_port_positions
#include "./CppCompileUtils/CppCompileUtils.h"
CPP_COMPILE_CPP_HEADER

#define MANIPULATOR_CLASS_VARIABLE DM
#include "./DynamixelManipulator/DynamixelManipulator.h"
#include "./DynamixelManipulatorMoves/DynamixelManipulatorMoves.h"
#include "./DynamixelManipulatorUtils/DynamixelManipulatorUtils.h"
#include "./Kinematics/Kinematics.h"


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

CPP_COMPILE_BEFORE_SETUP
DynamixelManipulatorMoves DM(6, minPoses, maxPoses);
void setup() {
  DM.SETUP();
  DM.addPoint(-20, -20, 30);
  DM.addPoint(-20, 0, 10);
  DM.addPoint(-20, 20, 30);
  DM.addPoint(0, 20, 10);
  DM.addPoint(20, 20, 30);
  DM.addPoint(20, 0, 10);
  DM.addPoint(20, -20, 30);
  DM.addPoint(0, -20, 10);
  DM.addPoint(-20, -20, 30);
  DM._printMovingPath();
  DM.go();
}

void loop() {
  KEYS_SECTION {

  }
  DM.DELAY();
}
CPP_COMPILE_AFTER_LOOP
