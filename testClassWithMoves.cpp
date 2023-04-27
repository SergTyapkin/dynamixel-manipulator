#include <iostream>
using namespace std;

#include "./DynamixelManipulator/DynamixelManipulator.h"
#define MANIPULATOR_CLASS_VARIABLE DM
#include "./DynamixelManipulatorUtils/DynamixelManipulatorUtils.h"
#include "./DynamixelManipulatorMoves/DynamixelManipulatorMoves.h"


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

int main() {
  DynamixelManipulatorMoves DM(6, minPoses, maxPoses);

  DM.addPoint(10, 10, 10);
  DM.addPoint(20, 30, 40);
  DM.addPoint(20, 50, 40);
  DM._printMovingPath();
  DM.removePoint(1);
  DM._printMovingPath();
  DM.addPoint(20, -20, 10);
  DM._printMovingPath();
  DM.go();

  return 0;
}
