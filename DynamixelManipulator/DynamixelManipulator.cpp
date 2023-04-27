#include "DynamixelManipulator.h"


#define PRINT_LIST(list)   \
  FOR_JOINTS_IDX(i) {      \
    Serial.print(list[i]); \
    Serial.print(" ");     \
  }


// --- Prints ---
#define __PRINT_IFDEF(var, value) \
  if(var)                         \
    Serial.print(value);
#define __PRINTln_IFDEF(var, value) \
  if(var)                           \
    Serial.println(value);
#define PRINT_SET(value) __PRINT_IFDEF(PRINT_SETS, value)
#define PRINT_ERR(value) __PRINT_IFDEF(PRINT_ERRS, value)
#define PRINT_SETUP(value) __PRINT_IFDEF(PRINT_SETUPS, value)
#define PRINT_SETln(value) __PRINTln_IFDEF(PRINT_SETS, value)
#define PRINT_ERRln(value) __PRINTln_IFDEF(PRINT_ERRS, value)
#define PRINT_SETUPln(value) __PRINTln_IFDEF(PRINT_SETUPS, value)


// --- Errors ---
#define ERR_INIT_DEVICE            -11
#define ERR_INIT_JOINT             -12
#define ERR_SET_JOINT_MODE         -13
#define ERR_READ_JOINT_POSITION    -14
#define ERR_READ_JOINT_SPEED       -15
#define ERR_READ_JOINT_CURRENT     -16
#define ERR_SET_JOINT_POSITION     -17
#define ERR_SET_JOINT_SPEED        -18
#define ERR_SET_JOINT_CURRENT      -19
#define ERR_ENABLE_JOINT           -20
#define ERR_DISABLE_JOINT          -21


// --- Constnants ---
#define MIN_SPEED_RPM 1.0

#ifdef CPP_COMPILE_MODE
  // --- Dynamixel2Arduino
  Dynamixel2Arduino::Dynamixel2Arduino(int serialPort, int dirPin) {;}
  Dynamixel2Arduino::Dynamixel2Arduino() {;}

  void Dynamixel2Arduino::begin(int baudrate) {
    std::cout << "DXL INIT BAUDRATE:" << baudrate << std::endl;
  }
  void Dynamixel2Arduino::setPortProtocolVersion(float protocolVersion) {
    std::cout << "DXL INIT PROTOCOL VERSION:" << protocolVersion << std::endl;
  }
  bool Dynamixel2Arduino::ping(unsigned jointId) {
    return true;
  }
  unsigned Dynamixel2Arduino::getModelNumber(unsigned jointId) {
    return 1234;
  }
  unsigned Dynamixel2Arduino::setOperatingMode(unsigned jointId, unsigned opMode) {
    return 1234;
  }

  unsigned Dynamixel2Arduino::getPresentCurrent(unsigned jointId, unsigned unit) {return 0;}
  unsigned Dynamixel2Arduino::getPresentVelocity(unsigned jointId, unsigned unit) {return 0;}
  unsigned Dynamixel2Arduino::getPresentPosition(unsigned jointId, unsigned unit) {return 0;}

  bool Dynamixel2Arduino::setGoalCurrent(unsigned jointId, unsigned val, unsigned unit) {return true;}
  bool Dynamixel2Arduino::setGoalVelocity(unsigned jointId, unsigned val, unsigned unit) {return true;}
  bool Dynamixel2Arduino::setGoalPosition(unsigned jointId, unsigned val, unsigned unit) {return true;}

  bool Dynamixel2Arduino::torqueOn(unsigned jointId) {return true;}
  bool Dynamixel2Arduino::torqueOff(unsigned jointId) {return true;}

  // --- SerialPort
  void SerialPort::begin(int baudrate) {
    std::cout << "SERIAL INIT BAUDRATE:" << baudrate << std::endl;
  }
  template <typename T>
  void SerialPort::print(T value) {
    std::cout << value;
  }
  void SerialPort::print(float value) {
    std::cout << value;
  }
  void SerialPort::print() {
  }
  template <typename T>
  void SerialPort::println(T value) {
    std::cout << value << std::endl;
  }
  void SerialPort::println(float value) {
    std::cout << value << std::endl;
  }
  void SerialPort::println() {
    std::cout << std::endl;
  }
  bool SerialPort::operator ! () {
    return false;
  }
  SerialPort _dynamixel_manipulator_serial;

  void delay(int ms) {}
#endif

DynamixelManipulator::DynamixelManipulator(
  size_t jointsCount,
  const Joint::posDeg* minJointsPoses,
  const Joint::posDeg* maxJointsPoses,
  const Joint::id* jointsIds,
  USBSerial serialPort,
  unsigned baudrate,
  unsigned dirPin,
  float protocolVersion
) {
  this->jointsCount = jointsCount;

  this->dxl = Dynamixel2Arduino(serialPort, dirPin);

  this->realPositions = this->_newJointsArray<Joint::posDeg>();
  this->realSpeeds = this->_newJointsArray<Joint::speedDPS>();
  this->realCurrents = this->_newJointsArray<Joint::currentMA>();

  this->minJointsPoses = this->_newJointsArray<Joint::posDeg>();
  this->maxJointsPoses = this->_newJointsArray<Joint::posDeg>();
  this->jointsIds = this->_newJointsArray<Joint::id>();
  this->prevPositions = this->_newJointsArray<Joint::posDeg>();

  FOR_JOINTS_IDX(i) {
    this->minJointsPoses[i] = minJointsPoses[i];
    this->maxJointsPoses[i] = maxJointsPoses[i];
  }

  // --- Init serial port to log data in computer console
  Serial.begin(baudrate);
  while (!Serial) {;} // --- Wait for opening serial port from computer

  // --- Init Dynamixel class
  this->dxl.begin(baudrate);
  this->dxl.setPortProtocolVersion(protocolVersion);

  // --- Init every joint
  bool result = false;
  FOR_JOINTS_IDX(idx) {
    const Joint::id jointId = idx + 1;
    if (jointsIds == NULL) {
      this->jointsIds[idx] = jointId;
    } else {
      this->jointsIds[idx] = jointsIds[idx];
    }

    result = this->dxl.ping(jointId);
    if (result == false) {
      PRINT_SETUP("Failed to ping joint ID: ");
      PRINT_SETUPln(jointId);
      exit(ERR_INIT_JOINT);
    }
    PRINT_SETUP("Succeeded to ping. ");
    PRINT_SETUP("id: [");
    PRINT_SETUP(jointId);

    unsigned modelNumber = this->dxl.getModelNumber(jointId);
    PRINT_SETUP("] ModelNumber : ");
    PRINT_SETUPln(modelNumber);

    result = this->dxl.setOperatingMode(jointId, ControlTableItem::OP_POSITION);
    if (result == false) {
      PRINT_SETUPln("Failed to set position mode");
      exit(ERR_SET_JOINT_MODE);
    }

    // --- Limit the maximum Speed in Position Control Mode. Use 0 to Max speed
    float speed = MAX_JOINT_SPEED_PERCENT;
    if (jointId == 1)
      speed *= 2;
    result = dxl.setGoalVelocity(jointId, speed, ControlTableItem::UNIT_PERCENT);
    if (result == false) {
      PRINT_SETUP("Failed to set max Speed of joint ID: ");
      PRINT_SETUPln(jointId);
      exit(ERR_SET_JOINT_SPEED);
    }
  }
  PRINT_SETUPln();

  #ifdef GRAPH_MODE
      FOR_JOINTS_ID(i) {
        Serial.print("Pos[");
        Serial.print(i);
        Serial.print("] ");
      }
      FOR_JOINTS_ID(i) {
        Serial.print("Spd[");
        Serial.print(i);
        Serial.print("] ");
      }
  //    FOR_JOINTS_ID(i) {
  //      Serial.print("Cur[");
  //      Serial.print(i);
  //      Serial.print("] ");
  //    }
      Serial.println();
  #endif
}

DynamixelManipulator::~DynamixelManipulator() {
  free(this->realPositions);
  free(this->realSpeeds);
  free(this->realCurrents);
}

// --- Basic
void DynamixelManipulator::LOOP_UPDATE() {
  this->readAllJointsPositionsDeg(this->realPositions);
  this->readAllJointsSpeedsDPS(this->realSpeeds);
  this->readAllJointsCurrentsMA(this->realCurrents);
}
void DynamixelManipulator::LOOP_PRINT() {
  #ifdef MANIPULATOR_SERIAL_PORT_GRAPH_MODE
    PRINT_LIST(this->realPositions);
    PRINT_LIST(this->realSpeeds);
    // PRINT_LIST(this->realCurrents);
    Serial.println();
  #else
    this->printPositionsIfChanged();
  #endif
}
void DynamixelManipulator::DELAY() {
  delay(UPDATE_DELAY * 1000);
}

// --- Position
void DynamixelManipulator::setJointPosRaw(Joint::id id, Joint::posRaw pos) {
  this->setJointPosAny(id, pos, "", ControlTableItem::UNIT_RAW);
}
void DynamixelManipulator::setAllJointsPosesRaw(Joint::posRaw jointsPos) {
  this->setAllJointsPosAny(jointsPos, "", ControlTableItem::UNIT_RAW);
}
void DynamixelManipulator::setAllJointsPosesRaw(Joint::posRaw* jointsPoses) {
  this->setAllJointsPosAny(jointsPoses, "", ControlTableItem::UNIT_RAW);
}

void DynamixelManipulator::setJointPosDeg(Joint::id id, Joint::posDeg pos) {
  this->setJointPosAny(id, pos, "°", ControlTableItem::UNIT_DEGREE);
}
void DynamixelManipulator::setAllJointsPosesDeg(Joint::posDeg jointsPos) {
  this->setAllJointsPosAny(jointsPos, "°", ControlTableItem::UNIT_DEGREE);
}
void DynamixelManipulator::setAllJointsPosesDeg(Joint::posDeg* jointsPoses) {
  this->setAllJointsPosAny(jointsPoses, "°", ControlTableItem::UNIT_DEGREE);
}

// --- Speed
void DynamixelManipulator::setJointSpeedRaw(Joint::id id, Joint::speedRaw speed) {
  this->setJointSpeedAny(id, speed, "", ControlTableItem::UNIT_RAW);
}
void DynamixelManipulator::setAllJointsSpeedsRaw(Joint::speedRaw jointsSpeed) {
  this->setAllJointsSpeedsAny(jointsSpeed, "", ControlTableItem::UNIT_RAW);
}
void DynamixelManipulator::setAllJointsSpeedsRaw(Joint::speedRaw* jointsSpeeds) {
  this->setAllJointsSpeedsAny(jointsSpeeds, "", ControlTableItem::UNIT_RAW);
}

void DynamixelManipulator::setJointSpeedPercent(Joint::id id, Joint::speedPercent speed) {
  this->setJointSpeedAny(id, speed, "%", ControlTableItem::UNIT_PERCENT);
}
void DynamixelManipulator::setAllJointsSpeedsPercent(Joint::speedPercent jointsSpeed) {
  this->setAllJointsSpeedsAny(jointsSpeed, "%", ControlTableItem::UNIT_PERCENT);
}
void DynamixelManipulator::setAllJointsSpeedsPercent(Joint::speedPercent* jointsSpeeds) {
  this->setAllJointsSpeedsAny(jointsSpeeds, "%", ControlTableItem::UNIT_PERCENT);
}

void DynamixelManipulator::setJointSpeedRPM(Joint::id id, Joint::speedRPM speed) {
  this->setJointSpeedAny(id, speed, "RPM", ControlTableItem::UNIT_RPM);
}
void DynamixelManipulator::setAllJointsSpeedsRPM(Joint::speedRPM jointsSpeed) {
  this->setAllJointsSpeedsAny(jointsSpeed, "RPM", ControlTableItem::UNIT_RPM);
}
void DynamixelManipulator::setAllJointsSpeedsRPM(Joint::speedRPM* jointsSpeeds) {
  this->setAllJointsSpeedsAny(jointsSpeeds, "RPM", ControlTableItem::UNIT_RPM);
}

void DynamixelManipulator::setJointSpeedDPS(Joint::id id, Joint::speedDPS speed) {
  this->setJointSpeedAny(id, speed / 6.0, "DPS", ControlTableItem::UNIT_RPM);
}
void DynamixelManipulator::setAllJointsSpeedsDPS(Joint::speedDPS jointsSpeed) {
  this->setAllJointsSpeedsAny(jointsSpeed / 6.0, "DPS", ControlTableItem::UNIT_RPM);
}
void DynamixelManipulator::setAllJointsSpeedsDPS(Joint::speedDPS* jointsSpeeds) {
  FOR_JOINTS_ID(id) {
    this->setJointSpeedAny(id, jointsSpeeds[__i] / 6.0, "DPS", ControlTableItem::UNIT_RPM);
  }
  //this->setAllJointsSpeedsAny(jointsSpeeds / 6.0, "DPS", ControlTableItem::UNIT_RPM);
}

// --- Current
void DynamixelManipulator::setJointCurrentRaw (Joint::id id, Joint::currentRaw current) {
  this->setJointCurrentAny(id, current, "", ControlTableItem::UNIT_RAW);
}
void DynamixelManipulator::setAllJointsCurrentsRaw(Joint::currentRaw jointsCurrent) {
  this->setAllJointsCurrentsAny(jointsCurrent, "", ControlTableItem::UNIT_RAW);
}
void DynamixelManipulator::setAllJointsCurrentsRaw(Joint::currentRaw* jointsCurrents) {
  this->setAllJointsCurrentsAny(jointsCurrents, "", ControlTableItem::UNIT_RAW);
}

void DynamixelManipulator::setJointCurrentPercent (Joint::id id, Joint::currentPercent current) {
  this->setJointCurrentAny(id, current, "%", ControlTableItem::UNIT_PERCENT);
}
void DynamixelManipulator::setAllJointsCurrentsPercent(Joint::currentPercent jointsCurrent) {
  this->setAllJointsCurrentsAny(jointsCurrent, "%", ControlTableItem::UNIT_PERCENT);
}
void DynamixelManipulator::setAllJointsCurrentsPercent(Joint::currentPercent* jointsCurrents) {
  this->setAllJointsCurrentsAny(jointsCurrents, "%", ControlTableItem::UNIT_PERCENT);
}

void DynamixelManipulator::setJointCurrentMA (Joint::id id, Joint::currentMA current) {
  this->setJointCurrentAny(id, current, "mA", ControlTableItem::UNIT_MA);
}
void DynamixelManipulator::setAllJointsCurrentsMA(Joint::currentMA jointsCurrent) {
  this->setAllJointsCurrentsAny(jointsCurrent, "mA", ControlTableItem::UNIT_MA);
}
void DynamixelManipulator::setAllJointsCurrentsMA(Joint::currentMA* jointsCurrents) {
  this->setAllJointsCurrentsAny(jointsCurrents, "mA", ControlTableItem::UNIT_MA);
}

// --- Torque
void DynamixelManipulator::enableJoint(Joint::id id) {
  bool result = this->dxl.torqueOn(id);
  if (result == false) {
    PRINT_ERR("Failed to enable joint ID: ");
    PRINT_ERRln(id);
    exit(ERR_ENABLE_JOINT);
  }
}
void DynamixelManipulator::enableAllJoints() {
  FOR_JOINTS_ID(id) {
    this->enableJoint(id);
  }
}
void DynamixelManipulator::disableJoint(Joint::id id) {
  bool result = this->dxl.torqueOff(id);
  if (result == false) {
    PRINT_ERR("Failed to disable joint ID: ");
    PRINT_ERRln(id);
    exit(ERR_ENABLE_JOINT);
  }
}
void DynamixelManipulator::disableAllJoints() {
  FOR_JOINTS_ID(id) {
    this->disableJoint(id);
  }
}

// ------- Print stats --------
void DynamixelManipulator::printPositions() {
  Serial.print("[POS](°): ");
  PRINT_LIST(this->realPositions);
  Serial.println();
}
void DynamixelManipulator::printSpeeds() {
  Serial.print("[VEL](DPS): ");
  PRINT_LIST(this->realSpeeds);
  Serial.println();
}
void DynamixelManipulator::printCurrents() {
  Serial.print("[CUR](mA): ");
  PRINT_LIST(this->realCurrents);
  Serial.println();
}

void DynamixelManipulator::printPositionsIfChanged() {
  static bool isInitialized = false;

  // copy actual positions to "prevPositions". Only 1 time
  if (!isInitialized) {
    FOR_JOINTS_IDX(i) {
      this->prevPositions[i] = this->realPositions[i];
    }
    isInitialized = true;
    return;
  }

  // check if some position changed
  bool isChanged = false;
  FOR_JOINTS_IDX(i) {
    if (this->prevPositions[i] != this->realPositions[i]) {
      isChanged = true;
      break;
    }
  }

  // if not changed - not print
  if (!isChanged)
    return;

  // print positions and copy changed
  Serial.print("[POS CHANGED]: ");
  FOR_JOINTS_IDX(i) {
    if (this->prevPositions[i] != this->realPositions[i]) {
      Serial.print("<");
      Serial.print(this->realPositions[i]);
      Serial.print("> ");
      this->prevPositions[i] = this->realPositions[i];
      continue;
    }
    Serial.print(" ");
    Serial.print(this->realPositions[i]);
    Serial.print("  ");
  }
  Serial.println("");
}

// -------- Set stats --------
// - Positions
void DynamixelManipulator::setJointPosAny(Joint::id id, float position, const char* unitStr, ControlTableItem::ParamUnit unit) {
  PRINT_SET("[SET POS][");
  PRINT_SET(id);
  PRINT_SET("]: ");
  PRINT_SET(position);
  PRINT_SETln(unitStr);
  if (unit == ControlTableItem::UNIT_DEGREE && (position < this->minJointsPoses[id - 1] || position > this->maxJointsPoses[id - 1])) {
    PRINT_ERR("Failed to set position. Position for this joint must be between ");
    PRINT_ERR(this->minJointsPoses[id-1]);
    PRINT_ERR("° and ");
    PRINT_ERR(this->maxJointsPoses[id-1]);
    PRINT_ERRln("°");
    exit(ERR_SET_JOINT_POSITION);
  }
  bool result = this->dxl.setGoalPosition(id, position, unit);
  if (result == false) {
    PRINT_ERR("Failed to set position ");
    PRINT_ERR(position);
    PRINT_ERR(unitStr);
    PRINT_ERR(" on joint ID: ");
    PRINT_ERRln(id);
    exit(ERR_SET_JOINT_POSITION);
  }
}
void DynamixelManipulator::setAllJointsPosAny(float position, const char* unitStr, ControlTableItem::ParamUnit unit) {
  FOR_JOINTS_ID(id) {
    this->setJointPosAny(id, position, unitStr, unit);
  }
}
void DynamixelManipulator::setAllJointsPosAny(const float* positions, const char* unitStr, ControlTableItem::ParamUnit unit) {
  FOR_JOINTS_ID(id) {
    this->setJointPosAny(id, positions[__i], unitStr, unit);
  }
}
// - Speeds
void DynamixelManipulator::setJointSpeedAny(Joint::id id, float speed, const char* unitStr, ControlTableItem::ParamUnit unit) {
  PRINT_SET("[SET VEL][");
  PRINT_SET(id);
  PRINT_SET("]: ");
  PRINT_SET(speed);
  PRINT_SETln(unitStr);
  if (unit == ControlTableItem::UNIT_RPM && speed < MIN_SPEED_RPM) {
    speed = MIN_SPEED_RPM;
    PRINT_SET(" -> ");
    PRINT_SET(speed);
    PRINT_SET(" RPM");
    PRINT_SETln();
  }
  bool result = this->dxl.setGoalVelocity(id, speed, unit);
  if (result == false) {
    PRINT_ERR("Failed to set speed ");
    PRINT_ERR(speed);
    PRINT_ERR(unitStr);
    PRINT_ERR(" on joint ID: ");
    PRINT_ERRln(id);
    // exit(ERR_SET_JOINT_SPEED);
  }
}
void DynamixelManipulator::setAllJointsSpeedsAny(float speed, const char* unitStr, ControlTableItem::ParamUnit unit) {
  FOR_JOINTS_ID(id) {
    this->setJointSpeedAny(id, speed, unitStr, unit);
  }
}
void DynamixelManipulator::setAllJointsSpeedsAny(const float* speeds, const char* unitStr, ControlTableItem::ParamUnit unit) {
  FOR_JOINTS_ID(id) {
    this->setJointSpeedAny(id, speeds[__i], unitStr, unit);
  }
}
// - Currents
void DynamixelManipulator::setJointCurrentAny(Joint::id id, float current, const char* unitStr, ControlTableItem::ParamUnit unit) {
  PRINT_SET("[SET CUR][");
  PRINT_SET(id);
  PRINT_SET("]: ");
  PRINT_SET(current);
  PRINT_SETln(unitStr);
  bool result = this->dxl.setGoalCurrent(id, current, unit);
  if (result == false) {
    PRINT_ERR("Failed to set current ");
    PRINT_ERR(current);
    PRINT_ERR(unitStr);
    PRINT_ERR(" on joint ID: ");
    PRINT_ERRln(id);
    exit(ERR_SET_JOINT_CURRENT);
  }
}
void DynamixelManipulator::setAllJointsCurrentsAny(float current, const char* unitStr, ControlTableItem::ParamUnit unit) {
  FOR_JOINTS_ID(id) {
    this->setJointCurrentAny(id, current, unitStr, unit);
  }
}
void DynamixelManipulator::setAllJointsCurrentsAny(const float* currents, const char* unitStr, ControlTableItem::ParamUnit unit) {
  FOR_JOINTS_ID(id) {
    this->setJointCurrentAny(id, currents[__i], unitStr, unit);
  }
}

// -------- Read stats --------
void DynamixelManipulator::readAllJointsPositionsRaw(Joint::posRaw* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentPosition(id, ControlTableItem::UNIT_RAW);
  }
}
void DynamixelManipulator::readAllJointsPositionsDeg(Joint::posDeg* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentPosition(id, ControlTableItem::UNIT_DEGREE);
  }
}
void DynamixelManipulator::readAllJointsSpeedsRaw(Joint::speedRaw* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentVelocity(id, ControlTableItem::UNIT_RAW);
  }
}
void DynamixelManipulator::readAllJointsSpeedsPercent(Joint::speedPercent* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentVelocity(id, ControlTableItem::UNIT_PERCENT);
  }
}
void DynamixelManipulator::readAllJointsSpeedsRPM(Joint::speedRPM* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentVelocity(id, ControlTableItem::UNIT_RPM);
  }
}
void DynamixelManipulator::readAllJointsSpeedsDPS(Joint::speedDPS* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentVelocity(id, ControlTableItem::UNIT_RPM * 6.0);
  }
}
void DynamixelManipulator::readAllJointsCurrentsRaw(Joint::currentRaw* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentCurrent(id, ControlTableItem::UNIT_RAW);
  }
}
void DynamixelManipulator::readAllJointsCurrentsMA(Joint::currentMA* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentCurrent(id, ControlTableItem::UNIT_MA);
  }
}
void DynamixelManipulator::readAllJointsCurrentsPercent(Joint::currentPercent* targetList) {
  FOR_JOINTS_ID(id) {
    targetList[__i] = this->dxl.getPresentCurrent(id, ControlTableItem::UNIT_PERCENT);
  }
}

// -----
template <typename T>
T* DynamixelManipulator::_newJointsArray() {
    return (T*)malloc(sizeof(T) * this->jointsCount);
}
