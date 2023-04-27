#ifndef INCLUDE_DYNAMIXEL_MANIPULATOR_
#define INCLUDE_DYNAMIXEL_MANIPULATOR_


#define CPP_COMPILE_MODE


#ifndef CPP_COMPILE_MODE
  #include <Dynamixel2Arduino.h>
#endif
#include "../Kinematics/Kinematics.h"
#include <math.h>
#include <stdlib.h>


#ifndef CPP_COMPILE_MODE
  #define DXL_SERIAL Serial3
#else
  #include <iostream>
  #define DXL_SERIAL -1
  #define USBSerial int
  class Dynamixel2Arduino {
  public:
    Dynamixel2Arduino(int serialPort, int dirPin);
    Dynamixel2Arduino();

    void begin(int baudrate);
    void setPortProtocolVersion(float protocolVersion);
    bool ping(unsigned jointId);
    unsigned getModelNumber(unsigned jointId);
    unsigned setOperatingMode(unsigned jointId, unsigned opMode);

    unsigned getPresentCurrent(unsigned jointId, unsigned unit);
    unsigned getPresentVelocity(unsigned jointId, unsigned unit);
    unsigned getPresentPosition(unsigned jointId, unsigned unit);

    bool setGoalCurrent(unsigned jointId, unsigned val, unsigned unit);
    bool setGoalVelocity(unsigned jointId, unsigned val, unsigned unit);
    bool setGoalPosition(unsigned jointId, unsigned val, unsigned unit);

    bool torqueOn(unsigned jointId);
    bool torqueOff(unsigned jointId);
  };

  namespace ControlTableItem {
    typedef int ParamUnit;
    const unsigned OP_POSITION = 1;
    const unsigned UNIT_PERCENT = 2;
    const unsigned UNIT_RAW = 3;
    const unsigned UNIT_DEGREE = 4;
    const unsigned UNIT_RPM = 5;
    const unsigned UNIT_MA = 6;
  }

  class SerialPort {
  public:
    void begin(int baudrate);
    template <typename T>
    void print(T value);
    void print(float value);
    void print();
    template <typename T>
    void println(T value);
    void println(float value);
    void println();

    bool operator ! ();
  };
  #define Serial _dynamixel_manipulator_serial

  void delay(int ms);
#endif
#define DXL_DIR_PIN 22
#define BAUDRATE  1000000
#define DXL_PROTOCOL_VERSION 1.0


#define DEG_MIN 0.0
#define DEG_MAX 360.0
#define RAW_MIN 0.0
#define RAW_MAX 4096.0

#define UPDATE_DELAY 0.050  // in seconds

#define MAX_JOINT_SPEED_PERCENT 5 // On first joint will be SPEED x2

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
    const Joint::id* jointsIds = NULL,
    USBSerial serialPort = DXL_SERIAL,
    unsigned baudrate = BAUDRATE,
    unsigned dirPin = DXL_DIR_PIN,
    float protocolVersion = DXL_PROTOCOL_VERSION
  );
  ~DynamixelManipulator();

  // --- Basic
  void LOOP_UPDATE();
  void LOOP_PRINT();
  void DELAY();

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
protected:
  size_t jointsCount;
  Dynamixel2Arduino dxl;
  Joint::posDeg* minJointsPoses;
  Joint::posDeg* maxJointsPoses;
  Joint::id* jointsIds;
  Joint::posDeg* prevPositions; // only for "print_if_changed"

  // -------- Set stats --------
  void setJointPosAny(Joint::id id, float position, const char* unitStr, ControlTableItem::ParamUnit unit);
  void setAllJointsPosAny(float position, const char* unitStr, ControlTableItem::ParamUnit unit);
  void setAllJointsPosAny(const float* positions, const char* unitStr, ControlTableItem::ParamUnit unit);

  void setJointSpeedAny(Joint::id id, float speed, const char* unitStr, ControlTableItem::ParamUnit unit);
  void setAllJointsSpeedsAny(float speed, const char* unitStr, ControlTableItem::ParamUnit unit);
  void setAllJointsSpeedsAny(const float* speeds, const char* unitStr, ControlTableItem::ParamUnit unit);

  void setJointCurrentAny(Joint::id id, float current, const char* unitStr, ControlTableItem::ParamUnit unit);
  void setAllJointsCurrentsAny(float current, const char* unitStr, ControlTableItem::ParamUnit unit);
  void setAllJointsCurrentsAny(const float* currents, const char* unitStr, ControlTableItem::ParamUnit unit);

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
};


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_
