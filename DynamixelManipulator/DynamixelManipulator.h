#ifndef INCLUDE_DYNAMIXEL_MANIPULATOR_
#define INCLUDE_DYNAMIXEL_MANIPULATOR_


#include "../CppCompileUtils/CppCompileUtils.h"
#ifndef _CPP_COMPILE
  #include <Dynamixel2Arduino.h>
#endif
#include "../Kinematics/Kinematics.h"
#include <math.h>
#include <stdlib.h>


#define DXL_SERIAL Serial3
#define DXL_DIR_PIN 22
#define BAUDRATE  1000000
#define DXL_PROTOCOL_VERSION 1.0


#define DEG_MIN 0.0
#define DEG_MAX 360.0
#define RAW_MIN 0.0
#define RAW_MAX 4096.0

#define UPDATE_DELAY 0.050  // in seconds

#define MAX_JOINT_SPEED_PERCENT 2 // On first joint will be SPEED x2

// --- You can define MANIPULATOR_SERIAL_PORT_GRAPH_MODE in your script ---
#ifdef MANIPULATOR_SERIAL_PORT_GRAPH_MODE
  #define PRINT_SETUPS false
  #define PRINT_SETS   false
  #define PRINT_ERRS   false
#else
  #define PRINT_SETUPS true
  #define PRINT_SETS   true
  #define PRINT_ERRS   true
#endif

#define FOR_JOINTS_IDX(name_of_var) for (Joint::index name_of_var = 0; name_of_var < this->jointsCount; name_of_var++)
#define FOR_JOINTS_ID(name_of_var) for (Joint::index __i = 0, name_of_var = this->jointsIds[__i]; __i < this->jointsCount; __i++, name_of_var = this->jointsIds[__i])

#define PRINT_LIST(list)   \
  FOR_JOINTS_IDX(i) {      \
    Serial.print(list[i]); \
    Serial.print(" ");     \
  }
#define __PRINT_IFDEF(var, value) \
  if(var)                         \
    Serial.print(value);
#define __PRINTln_IFDEF(var, value) \
  if(var)                           \
    Serial.println(value);

namespace Joint {
  typedef float posDeg;
  typedef float posRaw;
  typedef float speedDPS;
  typedef float speedRPM;
  typedef float speedPercent;
  typedef float speedRaw;
  typedef float currentMA;
  typedef float currentPercent;
  typedef float currentRaw;
  typedef unsigned id;
  typedef unsigned index;
}


class DynamixelManipulator {
public:
  Joint::posDeg* realPositions;
  Joint::speedDPS* realSpeeds;
  Joint::currentMA* realCurrents;

//  Joint::speedDPS limitSpeeds[this.jointsCount];
//  Joint::currentMA limitCurrents[this.jointsCount];

  DynamixelManipulator(
    size_t jointsCount,
    const Joint::posDeg* minJointsPoses,
    const Joint::posDeg* maxJointPoses,
    const Dynamixel2Arduino dxl,
    const Joint::id* jointsIds = NULL,
    unsigned baudrate = BAUDRATE,
    float protocolVersion = DXL_PROTOCOL_VERSION
  );
  ~DynamixelManipulator();

  // --- Basic
  void SETUP();
  void LOOP(bool withPrint = false);
  void DELAY();

  void updateRealValues();

  // --- Position
  void setJointPosRaw(Joint::id id, Joint::posRaw pos);
  void setAllJointsPosesRaw(Joint::posRaw jointsPos);
  void setAllJointsPosesRaw(Joint::posRaw* jointsPositions);

  void setJointPosDeg(Joint::id id, Joint::posDeg pos);
  void setAllJointsPosesDeg(Joint::posDeg jointsPos);
  void setAllJointsPosesDeg(Joint::posDeg* jointsPositions);

  // --- Speed
  void setJointSpeedRaw(Joint::id id, Joint::speedRaw speed);
  void setAllJointsSpeedsRaw(Joint::speedRaw jointsSpeed);
  void setAllJointsSpeedsRaw(Joint::speedRaw* jointsSpeeds);

  void setJointSpeedPercent(Joint::id id, Joint::speedPercent speed);
  void setAllJointsSpeedsPercent(Joint::speedPercent jointsSpeed);
  void setAllJointsSpeedsPercent(Joint::speedPercent* jointsSpeeds);

  void setJointSpeedRPM (Joint::id id, Joint::speedRPM speed);
  void setAllJointsSpeedsRPM(Joint::speedRPM jointsSpeed);
  void setAllJointsSpeedsRPM(Joint::speedRPM* jointsSpeeds);

  void setJointSpeedDPS(Joint::id id, Joint::speedDPS speed);
  void setAllJointsSpeedsDPS(Joint::speedDPS jointsSpeed);
  void setAllJointsSpeedsDPS(Joint::speedDPS* jointsSpeeds);

  // --- Current
  void setJointCurrentRaw (Joint::id id, Joint::currentRaw current);
  void setAllJointsCurrentsRaw(Joint::currentRaw jointsCurrent);
  void setAllJointsCurrentsRaw(Joint::currentRaw* jointsCurrent);

  void setJointCurrentPercent (Joint::id id, Joint::currentPercent current);
  void setAllJointsCurrentsPercent(Joint::currentPercent jointsCurrent);
  void setAllJointsCurrentsPercent(Joint::currentPercent* jointsCurrent);

  void setJointCurrentMA(Joint::id id, Joint::currentMA current);
  void setAllJointsCurrentsMA(Joint::currentMA jointsCurrent);
  void setAllJointsCurrentsMA(Joint::currentMA* jointsCurrent);

  // --- Torque
  void enableJoint(Joint::id id);
  void enableAllJoints();
  void disableJoint(Joint::id id);
  void disableAllJoints();

  // ------- Print stats --------
  void printPositions();
  void printSpeeds();
  void printCurrents();

  void printPositionsIfChanged();

  // ------ Getters --------
  Joint::posDeg getPositionDeg(Joint::id jointId);
  Joint::speedDPS getSpeedDPS(Joint::id jointId);
  Joint::currentMA getCurrentMA(Joint::id jointId);
protected:
  size_t jointsCount;
  Dynamixel2Arduino dxl;
  Joint::posDeg* minJointsPoses;
  Joint::posDeg* maxJointsPoses;
  Joint::id* jointsIds;
  Joint::posDeg* prevPositions; // only for "print_if_changed"

  const Joint::posDeg* tmpMinJointsPoses;
  const Joint::posDeg* tmpMaxJointsPoses;
  const Joint::id* tmpJointsIds;
  unsigned baudrate;
  float protocolVersion;

  // -------- Set stats --------
  void setJointPosAny(Joint::id id, float position, const char* unitStr, ParamUnit unit);
  void setAllJointsPosAny(float position, const char* unitStr, ParamUnit unit);
  void setAllJointsPosAny(const float* positions, const char* unitStr, ParamUnit unit);

  void setJointSpeedAny(Joint::id id, float speed, const char* unitStr, ParamUnit unit);
  void setAllJointsSpeedsAny(float speed, const char* unitStr, ParamUnit unit);
  void setAllJointsSpeedsAny(const float* speeds, const char* unitStr, ParamUnit unit);

  void setJointCurrentAny(Joint::id id, float current, const char* unitStr, ParamUnit unit);
  void setAllJointsCurrentsAny(float current, const char* unitStr, ParamUnit unit);
  void setAllJointsCurrentsAny(const float* currents, const char* unitStr, ParamUnit unit);

  // -------- Read stats --------
  void readAllJointsPositionsRaw(Joint::posRaw* targetList);
  void readAllJointsPositionsDeg(Joint::posDeg* targetList);
  void readAllJointsSpeedsRaw(Joint::speedRaw* targetList);
  void readAllJointsSpeedsPercent(Joint::speedPercent* targetList);
  void readAllJointsSpeedsRPM(Joint::speedRPM* targetList);
  void readAllJointsSpeedsDPS(Joint::speedDPS* targetList);
  void readAllJointsCurrentsRaw(Joint::currentRaw* targetList);
  void readAllJointsCurrentsMA(Joint::currentMA* targetList);
  void readAllJointsCurrentsPercent(Joint::currentPercent* targetList);

  // --- Utils ---
  template <typename T>
  T* _newJointsArray();
  float* _newJointsArray();
};


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_
