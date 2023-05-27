#ifndef INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_
#define INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_

#include <stdlib.h>
#include <math.h>

#include "../DynamixelManipulator/DynamixelManipulator.h"
#include "../Kinematics/Kinematics.h"
#include "../CppCompileUtils/CppCompileUtils.h"


#define MIN_JOINT_SPEED_DPS 6
#define MAX_JOINT_SPEED_DPS 20
#define MAX_JOINT_ACCELERATION_DPS 5

#define ACCURACY_TARGET_DIST 2


class DynamixelManipulatorMoves: public DynamixelManipulator {
public:
  void getTimedSmoothMovingSpeedDPS(const Joint::posDeg* start, const Joint::posDeg* end, float duration, const float currentTime, float* exportSpeedsDPS);

  DynamixelManipulatorMoves(
    size_t jointsCount,
    const Joint::posDeg* minJointsPoses,
    const Joint::posDeg* maxJointPoses,
    const Joint::id* jointsIds = NULL,
    UARTClass serialPort = DXL_SERIAL,
    unsigned baudrate = BAUDRATE,
    unsigned dirPin = DXL_DIR_PIN,
    float protocolVersion = DXL_PROTOCOL_VERSION
  );
  ~DynamixelManipulatorMoves();
  size_t addPoint(float x, float y, float z);
  size_t addPosition(const Joint::posDeg* positions);
  void removeLastPoint();
  void removePoint(size_t pointIndex);
  void clearAllPoints();
  void pause();
  void go();
  void _printMovingPath();

  void _setTarget(const Joint::posDeg* targetPositions);
  void LOOP_UPDATE();
protected:
  Joint::posDeg* startPos;
  Joint::posDeg* targetPos;

  Joint::posDeg** movingPath = NULL;
  bool isRunning = false;
  float currentTime = 0;
  size_t movingPathLen = 0;
  float _currentMovingDuration = 1;
  float _currentMovingTime;
};


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_
