#ifndef INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_
#define INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_

#include <stdlib.h>
#include <math.h>

#include "../DynamixelManipulator/DynamixelManipulator.h"
#include "../Kinematics/Kinematics.h"
#include "../CppCompileUtils/CppCompileUtils.h"


#define MIN_JOINT_SPEED_DPS 6  // 1 RPM
#define MAX_JOINT_SPEED_DPS 20
#define MAX_JOINT_ACCELERATION_DPS 10
#define MOVING_DURATION 1  // seconds

#define ACCURACY_TARGET_DIST 3


class DynamixelManipulatorMoves: public DynamixelManipulator {
public:
  void getTimedSmoothMovingSpeedDPS(const Joint::posDeg* start, const Joint::posDeg* end, float duration, const float currentTime, Joint::speedDPS* exportSpeedsDPS, Joint::posDeg* exportPosesDeg);

  DynamixelManipulatorMoves(
    size_t jointsCount,
    const Joint::posDeg* minJointsPoses,
    const Joint::posDeg* maxJointPoses,
    const Dynamixel2Arduino dxl,
    const Joint::id* jointsIds = NULL,
    unsigned baudrate = BAUDRATE,
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
  void SETUP();
  void LOOP(bool withPrint = false);
protected:
  Joint::posDeg* startPos;
  Joint::posDeg* targetPos;

  Joint::posDeg** movingPath = NULL;
  bool isRunning = false;
  float currentTime = 0;
  size_t movingPathLen = 0;
  float _currentMovingDuration = MOVING_DURATION;
  float _currentMovingTime;
};


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_MOVES_
