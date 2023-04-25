#include "Basics.h"


Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
pos actualJointsPositions[JOINTS_COUNT];
float actualJointsSpeedsDPS[JOINTS_COUNT];
float actualJointsCurrents[JOINTS_COUNT];


// ------- Default function --------
void SETUP() {
  pinMode(LED_BUILTIN, OUTPUT);

  // --- Init serial port to log data in computer console
  Serial.begin(BAUDRATE);
  while (!Serial) {;}

  // --- Init Dinamyxel device
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // --- Init every joint
  bool result = false;
  FOR_JOINTS_ID(jointId) {
    result = dxl.ping(jointId);
    if (result == false) {
      PRINT_SETUP("Failed to ping joint ID: ");
      PRINT_SETUPln(jointId);
      exit(ERR_INIT_JOINT);
    }
    PRINT_SETUP("Succeeded to ping. ");
    PRINT_SETUP("id: [");
    PRINT_SETUP(jointId);

    uint16_t modelNumber = dxl.getModelNumber(jointId);
    PRINT_SETUP("] ModelNumber : ");
    PRINT_SETUPln(modelNumber);

    result = dxl.setOperatingMode(jointId, OP_POSITION);
    if (result == false) {
      PRINT_SETUPln(F("Failed to set position mode"));
      exit(ERR_SET_JOINT_MODE);
    }

    // --- Limit the maximum Speed in Position Control Mode. Use 0 to Max speed
    float speed = MAX_JOINT_Speed_PERCENT;
    if (jointId == 1)
      speed *= 2;
    result = dxl.setGoalVelocity(jointId, speed, UNIT_PERCENT);
    if (result == false) {
      PRINT_SETUP(F("Failed to set max Speed of joint ID: "));
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

void LOOP_UPDATE_ACTUALS() {
  readAllJointsPositions(actualJointsPositions);
  readAllJointsSpeedsDPS(actualJointsSpeedsDPS);
  readAllJointsCurrents(actualJointsCurrents);
}
void LOOP_PRINT_CHANGED_STATS() {
  LOOP_UPDATE_ACTUALS();
  printPositionsIfChanged(actualJointsPositions);
}
void LOOP_PRINT_GRAPH_STATS() {
  LOOP_UPDATE_ACTUALS();
  _printList(actualJointsPositions);
  _printList(actualJointsSpeedsDPS);
//  _printList(actualJointsCurrents);
  Serial.println();
}


// ------- Set positions functions --------
void setJointPosition(id idx, pos position) {
  PRINT_SET("[SET POS][");
  PRINT_SET(idx);
  PRINT_SET("]: ");
  PRINT_SETln(position);
  if (position < MIN_POS(idx - 1) || position > MAX_POS(idx - 1)) {
    PRINT_ERR("Failed to set position. Position for this point must be between ");
    PRINT_ERR(MIN_POS(idx-1));
    PRINT_ERR(" and ");
    PRINT_ERRln(MAX_POS(idx-1));
    exit(ERR_SET_JOINT_POSITION);
  }
  bool result = dxl.setGoalPosition(idx, position, UNIT_DEGREE);
  if (result == false) {
    PRINT_ERR("Failed to set position ");
    PRINT_ERR(position);
    PRINT_ERR(" on joint ID: ");
    PRINT_ERRln(idx);
    exit(ERR_SET_JOINT_POSITION);
  }
}

void setAllJointsPositions(pos* jointsPositions) {
  FOR_JOINTS_ID(jointId) {
    setJointPosition(jointId, jointsPositions[jointId - 1]);
  }
}

// ------- Set Speed functions --------
void setJointSpeed(id idx, float percents) {
  PRINT_SET("[SET VEL][");
  PRINT_SET(idx);
  PRINT_SET("]: ");
  PRINT_SET(percents);
  PRINT_SETln("%");
  bool result = dxl.setGoalVelocity(idx, percents, UNIT_PERCENT);
  if (result == false) {
    Serial.print("Failed to set Speed ");
    Serial.print(percents);
    Serial.print(" on joint ID: ");
    Serial.println(idx);
    exit(ERR_SET_JOINT_Speed);
  }
}
void setJointSpeedDPS(id idx, float dps) {
  PRINT_SET("[SET VEL][");
  PRINT_SET(idx);
  PRINT_SET("]: ");
  PRINT_SET(dps);
  PRINT_SET(" DPS = ");
  float rpm = dps / 6.0;
  if (rpm < MIN_RPM)
    rpm = MIN_RPM;
  PRINT_SET(rpm);
  PRINT_SET(" RPM");
  PRINT_SETln();
  bool result = dxl.setGoalVelocity(idx, rpm, UNIT_RPM);
  if (result == false) {
    PRINT_ERR("Failed to set Speed DPS = ");
    PRINT_ERR(dps);
    PRINT_ERR(" RPM = ");
    PRINT_ERR(rpm);
    PRINT_ERR(" on joint ID: ");
    PRINT_ERRln(idx);
//    exit(ERR_SET_JOINT_Speed);
  }
}

void setAllJointsSpeeds(float* jointsSpeed) {
  FOR_JOINTS_ID(jointId) {
    setJointSpeed(jointId, jointsSpeed[jointId - 1]);
  }
}
void setAllJointsSpeedsDPS(float* jointsSpeed) {
  FOR_JOINTS_ID(jointId) {
    setJointSpeedDPS(jointId, jointsSpeed[jointId - 1]);
  }
}
// ------- Work with torque functions --------
void enable(id idx) {
  bool result = dxl.torqueOn(idx);
  if (result == false) {
    PRINT_ERR("Failed to enable joint ID: ");
    PRINT_ERRln(idx);
    exit(ERR_ENABLE_JOINT);
  }
}
void disable(id idx) {
  bool result = dxl.torqueOff(idx);
  if (result == false) {
    PRINT_ERR("Failed to disable joint ID: ");
    PRINT_ERRln(idx);
    exit(ERR_ENABLE_JOINT);
  }
}
void enableAll() {
  FOR_JOINTS_ID(jointId) {
    enable(jointId);
  }
}
void disableAll() {
  FOR_JOINTS_ID(jointId) {
    disable(jointId);
  }
}



// -------- Read stats --------
void readAllJointsPositions(pos* targetList) {
  FOR_JOINTS_ID(jointId) {
    pos jointPos = dxl.getPresentPosition(jointId, UNIT_DEGREE);
    targetList[jointId - 1] = jointPos;
  }
}
void readAllJointsSpeeds(float* targetList) {
  FOR_JOINTS_ID(jointId) {
    float jointSpeed = dxl.getPresentVelocity(jointId, UNIT_PERCENT);
    targetList[jointId - 1] = jointSpeed;
  }
}
void readAllJointsSpeedsDPS(float* targetList) {
  FOR_JOINTS_ID(jointId) {
    const float rpmSpeed = dxl.getPresentVelocity(jointId, UNIT_RPM);
    const float dpsSpeed = rpmSpeed * 6.0;
    targetList[jointId - 1] = dpsSpeed;
  }
}
void readAllJointsCurrents(float* targetList) {
  FOR_JOINTS_ID(jointId) {
    float jointCurrent = dxl.getPresentCurrent(jointId, UNIT_MILLI_AMPERE);
    targetList[jointId - 1] = jointCurrent;
  }
}


// ------- Print stats --------
void printPositions(const pos* list) {
  Serial.print("[POS](raw): ");
  _printList(list);
  Serial.println();
}
void printSpeeds(const pos* list) {
  Serial.print("[VEL](%): ");
  _printList(list);
  Serial.println();
}
void printSpeedsDPS(const pos* list) {
  Serial.print("[VEL](DPS): ");
  _printList(list);
  Serial.println();
}
void printCurrents(const pos* list) {
  Serial.print("[CUR](mA): ");
  _printList(list);
  Serial.println();
}

void printPositionsIfChanged(const pos* list) {
  static pos prevPositions[JOINTS_COUNT];
  static bool isInitialized = false;

  // copy actual positions to "prevPositions". Only 1 time
  if (!isInitialized) {
    FOR_JOINTS_IDX(i) {
      prevPositions[i] = list[i];
    }
    isInitialized = true;
    return;
  }

  // check if some position changed
  bool isChanged = false;
  FOR_JOINTS_IDX(i) {
    if (prevPositions[i] != list[i]) {
      isChanged = true;
      break;
    }
  }

  // if not changed - not print
  if (!isChanged)
    return;

  // print positions and copy changed
  Serial.print("[POS]: ");
  FOR_JOINTS_IDX(i) {
    if (prevPositions[i] != list[i]) {
      Serial.print("<");
      Serial.print(list[i]);
//      for (int i = 10000 - list[i]; i >= 1000; i /= 10)
//        Serial.print(" ");
      Serial.print("> ");
      prevPositions[i] = list[i];
      continue;
    }
    Serial.print(" ");
    Serial.print(list[i]);
//    for (int i = 10000 - list[i]; i >= 1000; i /= 10)
//      Serial.print(" ");
    Serial.print("  ");
  }
  Serial.println("");
}
