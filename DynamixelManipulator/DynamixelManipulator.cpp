#include "DynamixelManipulator.h"


#define FOR_JOINTS_IDX(name_of_var) for (Joint::index name_of_var = 0; name_of_var < this.jointsCount; name_of_var++)
#define FOR_JOINTS_ID(name_of_var) for (Joint::index __i = 0, name_of_var = this.jointsIds[__i]; __i < this.jointsCount; __i++, name_of_var = this.jointsIds[__i])

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


class DynamixelManipulator {
public:
  Joint::posDeg realPositions[this.jointsCount];
  Joint::speedDPS realSpeeds[this.jointsCount];
  Joint::currentMA realCurrents[this.jointsCount];

//  Joint::speedDPS limitSpeeds[this.jointsCount];
//  Joint::currentMA limitCurrents[this.jointsCount];

  DynamixelManipulator(
      size_t jointsCount,
      const Joint::posDeg* minJointsPoses,
      const Joint::posDeg* maxJointPoses,
      const Joint::id* jointsIds,
      USBSerial serialPort = DXL_SERIAL,
      unsigned baudrate = BAUDRATE,
      unsigned dirPin = DXL_DIR_PIN,
      float protocolVersion = DXL_PROTOCOL_VERSION,
  ) {
    this.jointsCount = jointsCount;
    this.minJointsPoses = minJointsPoses;
    this.maxJointsPoses = maxJointsPoses;

    this.dxl = Dynamixel2Arduino(serialPort, dirPin);

    // --- Init serial port to log data in computer console
    Serial.begin(baudrate);
    while (!Serial) {;} // --- Wait for opening serial port from computer

    // --- Init Dynamixel class
    this.dxl.begin(baudrate);
    this.dxl.setPortProtocolVersion(protocolVersion);

    // --- Init every joint
    bool result = false;
    FOR_JOINTS_IDX(idx) {
      const Joint::id jointId = idx + 1;
      this.jointsIds[idx] = jointId;

      result = this.dxl.ping(jointId);
      if (result == false) {
        PRINT_SETUP("Failed to ping joint ID: ");
        PRINT_SETUPln(jointId);
        exit(ERR_INIT_JOINT);
      }
      PRINT_SETUP("Succeeded to ping. ");
      PRINT_SETUP("id: [");
      PRINT_SETUP(jointId);

      unsigned modelNumber = this.dxl.getModelNumber(jointId);
      PRINT_SETUP("] ModelNumber : ");
      PRINT_SETUPln(modelNumber);

      result = this.dxl.setOperatingMode(jointId, ControlTableItem::OP_POSITION);
      if (result == false) {
        PRINT_SETUPln("Failed to set position mode");
        exit(ERR_SET_JOINT_MODE);
      }

      // --- Limit the maximum Speed in Position Control Mode. Use 0 to Max speed
      float speed = MAX_JOINT_Speed_PERCENT;
      if (jointId == 1)
        speed *= 2;
      result = dxl.setGoalVelocity(jointId, speed, ControlTableItem::UNIT_PERCENT);
      if (result == false) {
        PRINT_SETUP("Failed to set max Speed of joint ID: ");
        PRINT_SETUPln(jointId);
        exit(ERR_SET_MAX_JOINT_Speed);
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
  ~DynamixelManipulator() {
  }

  // --- Basic
  void LOOP_UPDATE() {
    this.readAllJointsPositionsDeg(this.realPositions);
    this.readAllJointsSpeedsDPS(this.realSpeeds);
    this.readAllJointsCurrentsMA(this.realCurrents);
  }
  void LOOP_PRINT() {
    #ifdef MANIPULATOR_SERIAL_PORT_GRAPH_MODE
      PRINT_LIST(this.realPositions);
      PRINT_LIST(this.realSpeeds);
      // PRINT_LIST(this.realCurrents);
      Serial.println();
    #else
      this.printPositionsIfChanged();
    #endif
  }
  void DELAY() {
    delay(UPDATE_DELAY * 1000);
  }

  // --- Position
  void setJointPosRaw(Joint::id id, Joint::posRaw pos) {
    this.setJointPosAny(id, pos, "", ControlTableItem::UNIT_RAW);
  }
  void setAllJointsPosesRaw(Joint::posRaw jointsPos) {
    this.setAllJointsPosAny(jointsPos, "", ControlTableItem::UNIT_RAW);
  }
  void setAllJointsPosesRaw(Joint::posRaw* jointsPoses) {
    this.setAllJointsPosAny(jointsPoses, "", ControlTableItem::UNIT_RAW);
  }

  void setJointPosDeg(Joint::id id, Joint::posDeg pos) {
    this.setJointPosAny(id, pos, "°", ControlTableItem::UNIT_DEGREE);
  }
  void setAllJointsPosesDeg(Joint::posDeg jointsPos) {
    this.setAllJointsPosAny(jointsPos, "°", ControlTableItem::UNIT_DEGREE);
  }
  void setAllJointsPosesDeg(Joint::posDeg* jointsPoses) {
    this.setAllJointsPosAny(jointsPoses, "°", ControlTableItem::UNIT_DEGREE);
  }

  // --- Speed
  void setJointSpeedRaw(Joint::id id, Joint::speedRaw speed) {
    this.setJointSpeedAny(id, speed, "", ControlTableItem::UNIT_RAW);
  }
  void setAllJointsSpeedsRaw(Joint::speedRaw jointsSpeed) {
    this.setAllJointsSpeedsAny(jointsSpeed, "", ControlTableItem::UNIT_RAW);
  }
  void setAllJointsSpeedsRaw(Joint::speedRaw* jointsSpeeds) {
    this.setAllJointsSpeedsAny(jointsSpeeds, "", ControlTableItem::UNIT_RAW);
  }

  void setJointSpeedPercent(Joint::id id, Joint::speedPercent speed) {
    this.setJointSpeedAny(id, speed, "%", ControlTableItem::UNIT_PERCENT);
  }
  void setAllJointsSpeedsPercent(Joint::speedPercent jointsSpeed) {
    this.setAllJointsSpeedsAny(jointsSpeed, "%", ControlTableItem::UNIT_PERCENT);
  }
  void setAllJointsSpeedsPercent(Joint::speedPercent* jointsSpeeds) {
    this.setAllJointsSpeedsAny(jointsSpeeds, "%", ControlTableItem::UNIT_PERCENT);
  }

  void setJointSpeedRPM(Joint::id id, Joint::speedRPM speed) {
    this.setJointSpeedAny(id, speed, "RPM", ControlTableItem::UNIT_RPM);
  }
  void setAllJointsSpeedsRPM(Joint::speedRPM jointsSpeed) {
    this.setAllJointsSpeedsAny(jointsSpeed, "RPM", ControlTableItem::UNIT_RPM);
  }
  void setAllJointsSpeedsRPM(Joint::speedRPM* jointsSpeeds) {
    this.setAllJointsSpeedsAny(jointsSpeeds, "RPM", ControlTableItem::UNIT_RPM);
  }

  void setJointSpeedDPS(Joint::id id, Joint::speedDPS speed) {
    this.setJointSpeedAny(id, speed / 6.0, "DPS", ControlTableItem::UNIT_RPM);
  }
  void setAllJointsSpeedsDPS(Joint::speedDPS jointsSpeed) {
    this.setAllJointsSpeedsAny(jointsSpeed / 6.0, "DPS", ControlTableItem::UNIT_RPM);
  }
  void setAllJointsSpeedsDPS(Joint::speedDPS* jointsSpeeds) {
    this.setAllJointsSpeedsAny(jointsSpeeds / 6.0, "DPS", ControlTableItem::UNIT_RPM);
  }

  // --- Current
  void setJointCurrentRaw (Joint::id id, Joint::currentRaw current) {
    this.setJointCurrentAny(id, current, "", ControlTableItem::UNIT_RAW);
  }
  void setAllJointsCurrentsRaw(Joint::currentRaw jointsCurrent) {
    this.setAllJointsCurrentsAny(jointsCurrent, "", ControlTableItem::UNIT_RAW);
  }
  void setAllJointsCurrentsRaw(Joint::currentRaw* jointsCurrents) {
    this.setAllJointsCurrentsAny(jointsCurrents, "", ControlTableItem::UNIT_RAW);
  }

  void setJointCurrentPercent (Joint::id id, Joint::currentPercent current) {
    this.setJointCurrentAny(id, current, "%", ControlTableItem::UNIT_PERCENT);
  }
  void setAllJointsCurrentsPercent(Joint::currentPercent jointsCurrent) {
    this.setAllJointsCurrentsAny(jointsCurrent, "%", ControlTableItem::UNIT_PERCENT);
  }
  void setAllJointsCurrentsPercent(Joint::currentPercent* jointsCurrents) {
    this.setAllJointsCurrentsAny(jointsCurrents, "%", ControlTableItem::UNIT_PERCENT);
  }

  void setJointCurrentMA (Joint::id id, Joint::currentMA current) {
    this.setJointCurrentAny(id, current, "mA", ControlTableItem::UNIT_MA);
  }
  void setAllJointsCurrentsMA(Joint::currentMA jointsCurrent) {
    this.setAllJointsCurrentsAny(jointsCurrent, "mA", ControlTableItem::UNIT_MA);
  }
  void setAllJointsCurrentsMA(Joint::currentMA* jointsCurrents) {
    this.setAllJointsCurrentsAny(jointsCurrents, "mA", ControlTableItem::UNIT_MA);
  }

  // --- Torque
  void enableJoint(Joint::id id) {
    bool result = this.dxl.torqueOn(idx);
    if (result == false) {
      PRINT_ERR("Failed to enable joint ID: ");
      PRINT_ERRln(idx);
      exit(ERR_ENABLE_JOINT);
    }
  }
  void enableAllJoints() {
    FOR_JOINTS_ID(id) {
      this.enableJoint(id);
    }
  }
  void disableJoint(Joint::id id) {
    bool result = this.dxl.torqueOff(idx);
    if (result == false) {
      PRINT_ERR("Failed to disable joint ID: ");
      PRINT_ERRln(idx);
      exit(ERR_ENABLE_JOINT);
    }
  }
  void disableAllJoints() {
    FOR_JOINTS_ID(id) {
      this.disableJoint(id);
    }
  }

  // ------- Print stats --------
  void printPositions() {
    Serial.print("[POS](°): ");
    PRINT_LIST(this.realPositions);
    Serial.println();
  }
  void printSpeeds() {
    Serial.print("[VEL](DPS): ");
    PRINT_LIST(this.realSpeeds);
    Serial.println();
  }
  void printCurrents() {
    Serial.print("[CUR](mA): ");
    PRINT_LIST(this.realCurrents);
    Serial.println();
  }

  void printPositionsIfChanged() {
    static Joint::posDeg prevPositions[this.jointsCount];
    static bool isInitialized = false;

    // copy actual positions to "prevPositions". Only 1 time
    if (!isInitialized) {
      FOR_JOINTS_IDX(i) {
        prevPositions[i] = this.realPositions[i];
      }
      isInitialized = true;
      return;
    }

    // check if some position changed
    bool isChanged = false;
    FOR_JOINTS_IDX(i) {
      if (prevPositions[i] != this.realPositions[i]) {
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
      if (prevPositions[i] != this.realPositions[i]) {
        Serial.print("<");
        Serial.print(list[i]);
        Serial.print("> ");
        prevPositions[i] = this.realPositions[i];
        continue;
      }
      Serial.print(" ");
      Serial.print(this.realPositions[i]);
      Serial.print("  ");
    }
    Serial.println("");
  }
protected:
  size_t jointsCount;
  Dynamixel2Arduino dxl;

  // -------- Set stats --------
  // - Positions
  void setJointPosAny(Joint::id id, float position, const char* unitStr, ControlTableItem::ParamUnit unit) {
    PRINT_SET("[SET POS][");
    PRINT_SET(idx);
    PRINT_SET("]: ");
    PRINT_SET(position);
    PRINT_SETln(unitStr);
    if (unit == ControlTableItem::UNIT_DEGREE && (position < this.minJointsPoses(id - 1) || position > this.maxJointsPoses(id - 1))) {
      PRINT_ERR("Failed to set position. Position for this point must be between ");
      PRINT_ERR(MIN_POS(idx-1));
      PRINT_ERR("° and ");
      PRINT_ERR(MAX_POS(idx-1));
      PRINT_ERRln("°");
      exit(ERR_SET_JOINT_POSITION);
    }
    bool result = this.dxl.setGoalPosition(idx, position, unit);
    if (result == false) {
      PRINT_ERR("Failed to set position ");
      PRINT_ERR(position);
      PRINT_ERR(unitStr);
      PRINT_ERR(" on joint ID: ");
      PRINT_ERRln(idx);
      exit(ERR_SET_JOINT_POSITION);
    }
  }
  void setAllJointsPosAny(float position, const char* unitStr, ControlTableItem::ParamUnit unit) {
    FOR_JOINTS_ID(id) {
      this.setJointPosAny(id, position, unitStr, unit);
    }
  }
  void setAllJointsPosAny(const float* positions, const char* unitStr, ControlTableItem::ParamUnit unit) {
    FOR_JOINTS_ID(id) {
      this.setJointPosAny(id, positions[__i], unitStr, unit);
    }
  }
  // - Speeds
  void setJointSpeedAny(Joint::id id, float speed, const char* unitStr, ControlTableItem::ParamUnit unit) {
    PRINT_SET("[SET VEL][");
    PRINT_SET(idx);
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
    bool result = this.dxl.setGoalVelocity(idx, speed, unit);
    if (result == false) {
      PRINT_ERR("Failed to set speed ");
      PRINT_ERR(speed);
      PRINT_ERR(unitStr);
      PRINT_ERR(" on joint ID: ");
      PRINT_ERRln(idx);
      // exit(ERR_SET_JOINT_SPEED);
    }
  }
  void setAllJointsSpeedsAny(float speed, const char* unitStr, ControlTableItem::ParamUnit unit) {
    FOR_JOINTS_ID(id) {
      this.setJointSpeedAny(id, speed, unitStr, unit);
    }
  }
  void setAllJointsSpeedsAny(const float* speeds, const char* unitStr, ControlTableItem::ParamUnit unit) {
    FOR_JOINTS_ID(id) {
      this.setJointSpeedAny(id, speeds[__i], unitStr, unit);
    }
  }
  // - Currents
  void setJointCurrentAny(Joint::id id, float current, const char* unitStr, ControlTableItem::ParamUnit unit) {
    PRINT_SET("[SET CUR][");
    PRINT_SET(idx);
    PRINT_SET("]: ");
    PRINT_SET(current);
    PRINT_SETln(unitStr);
    bool result = this.dxl.setGoalCurrent(idx, current, unit);
    if (result == false) {
      PRINT_ERR("Failed to set current ");
      PRINT_ERR(current);
      PRINT_ERR(unitStr);
      PRINT_ERR(" on joint ID: ");
      PRINT_ERRln(idx);
      exit(ERR_SET_JOINT_CURRENT);
    }
  }
  void setAllJointsCurrentsAny(float current, const char* unitStr, ControlTableItem::ParamUnit unit) {
    FOR_JOINTS_ID(id) {
      this.setJointCurrentAny(id, current, unitStr, unit);
    }
  }
  void setAllJointsCurrentsAny(const float* currents, const char* unitStr, ControlTableItem::ParamUnit unit) {
    FOR_JOINTS_ID(id) {
      this.setJointCurrentAny(id, currents[__i], unitStr, unit);
    }
  }

  // -------- Read stats --------
  void readAllJointsPositionsRaw(Joint::posRaw* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentPosition(id, UNIT_RAW);
    }
  }
  void readAllJointsPositionsDeg(Joint::posDeg* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentPosition(id, UNIT_DEGREE);
    }
  }
  void readAllJointsSpeedsRaw(Joint::speedRaw* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentVelocity(id, UNIT_RAW);
    }
  }
  void readAllJointsSpeedsPercent(Joint::speedPercent* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentVelocity(id, UNIT_PERCENT);
    }
  }
  void readAllJointsSpeedsRPM(Joint::speedRPM* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentVelocity(id, UNIT_RPM);
    }
  }
  void readAllJointsSpeedsDPS(Joint::speedDPS* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentVelocity(id, UNIT_RPM * 6.0);
    }
  }
  void readAllJointsCurrentsRaw(Joint::currentRaw* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentCurrent(id, UNIT_RAW);
    }
  }
  void readAllJointsCurrentsMA(Joint::currentMA* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentCurrent(id, UNIT_MA);
    }
  }
  void readAllJointsCurrentsPercent(Joint::currentPercent* targetList) {
    FOR_JOINTS_ID(id) {
      targetList[__i] = this.dxl.getPresentCurrent(id, UNIT_PERCENT);
    }
  }

};


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_
