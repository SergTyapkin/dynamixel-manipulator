#ifndef INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_
#define INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_


#include "../DynamixelManipulator/DynamixelManipulator.h"
#include "../Kinematics/Kinematics.h"
#include <stdlib.h>
#include <math.h>


class DynamixelManipulatorMoves: public DynamixelManipulator {
public:
  size_t addPoint(float x, float y, float z);
  void removeLastPoint();
  void removePoint(size_t pointIndex);
  void clearAllPoints();
  void pause();
  void resume();
protected:
  Joint::posDeg** movingPath;
  Joint::posDeg startPos[this.jointsCount];
  Joint::posDeg targetPos[this.jointsCount];
};


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_
