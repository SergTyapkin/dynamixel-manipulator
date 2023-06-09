#define _CPP_COMPILE_UNIQUE_NAME _serial_move_by_serial_port_positions
#include "CppCompileUtils/CppCompileUtils.h"
CPP_COMPILE_CPP_HEADER

#include "DynamixelManipulatorMoves/DynamixelManipulatorMoves.h"
#include "DynamixelManipulator/DynamixelManipulator.h"
#define MANIPULATOR_CLASS_VARIABLE DM
#define MANIPULATOR_POS_DEFAULT {135, 180, 155, 164, 180, 170}
#define MANIPULATOR_POS_OFF     {135, 155, 132, 164, 180, 170}
#include "DynamixelManipulatorUtils/DynamixelManipulatorUtils.h"
#include "Kinematics/Kinematics.h"

#define END_INPUT_KEY 'E'

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
#define JOINTS_COUNT 6


CPP_COMPILE_BEFORE_SETUP

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
DynamixelManipulatorMoves DM(JOINTS_COUNT, minPoses, maxPoses, dxl);

static bool isInputInProgress = true;
static Joint::posDeg curNumber = 0;
static int curNumberLen = 0;
static Joint::posDeg curPos[JOINTS_COUNT];
static int curPosSize = 0;
static char lastSymbol = ' ';
static bool isAfterPoint = false;
static int numberAfterPoint = 0;

void setup() {
  DM.SETUP();
}


void loop() {
  if (!isInputInProgress)
    DM.LOOP();

  KEYS_SECTION {
      ON_KEY('G') {
        DM._printMovingPath();
        DM.go();
      }
      ON_KEY(END_INPUT_KEY) {
        isInputInProgress = false;
        DM._printMovingPath();
      }

      if (isInputInProgress && __readedSymbol != '\0') {
        if ((__readedSymbol == ' ' || __readedSymbol == '\n') && lastSymbol != ' ') {
          curPos[curPosSize] = curNumber;
          curPosSize++;
          curNumber = 0;
          curNumberLen = 0;
          isAfterPoint = false;
          numberAfterPoint = 0;
          if (__readedSymbol == '\n') {
            DM.addPosition(curPos);
            curPosSize = 0;
          }
        } else if (__readedSymbol == '.' || __readedSymbol == ',') {
          isAfterPoint = true;
        } else if (__readedSymbol >= '0' && __readedSymbol <= '9') {
          if (isAfterPoint) { // after point
            numberAfterPoint++;
            curNumber = float(curNumber) + float(__readedSymbol - '0') / pow(10, numberAfterPoint);
          } else { // before point
            curNumber = 10 * curNumber + (__readedSymbol - '0');
            curNumberLen++;
          }
        }

        lastSymbol = __readedSymbol;
      }
  }

  if (!isInputInProgress)
    DM.DELAY();
}
CPP_COMPILE_AFTER_LOOP
