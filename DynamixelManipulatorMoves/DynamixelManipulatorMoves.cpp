#define _CPP_COMPILE_UNIQUE_NAME _serial_dynamixel_manipulator_moves
#include "DynamixelManipulatorMoves.h"
CPP_COMPILE_CPP_HEADER


#define PRINT_MOVES true

#define PRINT_MOVE(value) __PRINT_IFDEF(PRINT_MOVES, value)
#define PRINT_MOVEln(value) __PRINTln_IFDEF(PRINT_MOVES, value)


void get_T1_L_Acc_Dur(const Joint::posDeg startPos, const Joint::posDeg endPos, const float minDuration, float &T1, float &L, float &acceleration, float &duration) {
  // 1 variant: Duration = const
  duration = minDuration; // Total moving time
  L = fabs(endPos - startPos); // Total moving length
  T1 = duration / 2; // Time to stop acceleration
  acceleration = L / (T1 * T1);
  // ^ speed
  // | ---------------- max speed
  // |
  // |     _/T1\_
  // |   _/      \_
  // | _/          \_duration
  // |_______________________________> time
  if (acceleration > MAX_JOINT_ACCELERATION_DPS) { // Acceleration so much
    // 2 variant: Acceleration = const
    acceleration = float(MAX_JOINT_ACCELERATION_DPS);
    T1 = sqrt(L / acceleration);
    duration = T1 * 2;

    if (acceleration * T1 > MAX_JOINT_SPEED_DPS) { // MaxSpeed so much
      // 3 variant: Acceleration = const; MaxSpeed = const
      acceleration = float(MAX_JOINT_ACCELERATION_DPS);
      T1 = float(MAX_JOINT_SPEED_DPS) / MAX_JOINT_ACCELERATION_DPS;
      duration = L / float(MAX_JOINT_SPEED_DPS) + T1;
      // ^ speed
      // |
      // |
      // |----T1---------T2---- max speed
      // |   _/           \_
      // | _/               \_duration
      // |______________________________> time
    }
  }
}

void DynamixelManipulatorMoves::getTimedSmoothMovingSpeedDPS(const Joint::posDeg* start, const Joint::posDeg* end, const float movingDuration, const float currentTime, Joint::speedDPS* exportSpeedsDPS, Joint::posDeg* exportPosesDeg) {
  float T1, L, acceleration, duration;
  float maxDuration = movingDuration;
  FOR_JOINTS_IDX(i) {
    get_T1_L_Acc_Dur(start[i], end[i], movingDuration, T1, L, acceleration, duration);
    maxDuration = fmax(maxDuration, duration); // save max duration
  }

  FOR_JOINTS_IDX(i) {
    get_T1_L_Acc_Dur(start[i], end[i], maxDuration, T1, L, acceleration, duration);
    const float currentL = fabs(this->realPositions[i] - start[i]);
    const float speedAfterT1 = T1 * acceleration;
    const float L1 = acceleration * (T1 * T1) / 2;
    const float T2 = duration - T1;
    const float L2 = L1 + (T2 - T1) * speedAfterT1;


    float targetL = -1;
//    Serial.print("СurtentTime: ");
//    Serial.println(currentTime);
//    Serial.print("T1: ");
//    Serial.println(T1);
//    Serial.print("T2: ");
//    Serial.println(T2);
//    Serial.print("Duration: ");
//    Serial.println(duration);
//    Serial.print("L1: ");
//    Serial.println(L1);
//    Serial.print("L2: ");
//    Serial.println(L2);
//    Serial.print("L cur: ");
//    Serial.println(currentL);
//    Serial.print("L: ");
//    Serial.println(L);
    exportSpeedsDPS[i] = 0;
    if (currentTime <= T1) {
//    if (currentL <= L1) {
      // Starting
      float time = currentTime;
      exportSpeedsDPS[i] = acceleration * time;
      targetL = acceleration * (currentTime * time) / 2;
    } else if (currentTime <= T2) {
//    } else if (currentL <= L2) {
      // Center on max speed
      float time = currentTime - T1;
      exportSpeedsDPS[i] = speedAfterT1;
      targetL = L1 + time * speedAfterT1;
    } else if (currentTime <= duration) {
//    } else if (currentL <= L) {
      // Stopping
      float time = currentTime - T2;
      exportSpeedsDPS[i] = speedAfterT1 - (acceleration * time);
      targetL = L2 + (speedAfterT1 * time) - acceleration * (time * time) / 2;
    } else {
      // No movement. Must be already in end pos
      exportSpeedsDPS[i] = MIN_JOINT_SPEED_DPS;
      targetL = L;
    }
    Serial.print("L target: ");
    Serial.println(targetL);
    Serial.println("-------------");
    exportPosesDeg[i] = start[i] + (end[i] - start[i]) * (targetL / L);
    if (currentTime <= duration) {
//    if (currentL <= L) {
      exportSpeedsDPS[i] += (targetL - currentL) * 2;
    }
  }
}

DynamixelManipulatorMoves::DynamixelManipulatorMoves(
    size_t jointsCount,
    const Joint::posDeg* minJointsPoses,
    const Joint::posDeg* maxJointsPoses,
    const Joint::posDeg* jointsMoveAccuracies,
    const Dynamixel2Arduino dxl,
    const Joint::id* jointsIds,
    unsigned baudrate,
    float protocolVersion
): DynamixelManipulator(jointsCount, minJointsPoses, maxJointsPoses, dxl, jointsIds, baudrate, protocolVersion) {
  this->tmpJointsMoveAccuracies = jointsMoveAccuracies;
}

void DynamixelManipulatorMoves::SETUP() {
  DynamixelManipulator::SETUP();

  this->jointsMoveAccuracies = this->_newJointsArray();
  this->startPos = this->_newJointsArray();
  this->targetPos = this->_newJointsArray();

  this->updateRealValues(); // получаем текущую позицию
  FOR_JOINTS_IDX(i) {
    this->startPos[i] = this->realPositions[i]; // копируем в стартовую
  }

  FOR_JOINTS_IDX(i) {
    this->jointsMoveAccuracies[i] = this->tmpJointsMoveAccuracies[i];
  }
}

DynamixelManipulatorMoves::~DynamixelManipulatorMoves() {
  free(this->startPos);
  free(this->targetPos);

  for (size_t i = 0; i < this->movingPathLen; i++) {
    free(this->movingPath[i]);
  }
  if (this->movingPath != NULL)
    free(this->movingPath);
}

size_t DynamixelManipulatorMoves::addPoint(float x, float y, float z) {
  // получаем новую точку
  Joint::posDeg* newPoses = (Joint::posDeg*)malloc(sizeof(Joint::posDeg) * this->jointsCount);
  getAnglesByTargetPoint(x, y, z, this->realPositions, newPoses, this->jointsCount, this->minJointsPoses, this->maxJointsPoses);

  // добавляем в путь
  size_t res = this->addPosition(newPoses);

  // не забываем очистить память
  free(newPoses);
  return res;
}

size_t DynamixelManipulatorMoves::addPosition(const Joint::posDeg* positions) {
  // Создаём копию списка
  Joint::posDeg* positionsCopy = (Joint::posDeg*)malloc(sizeof(Joint::posDeg) * this->jointsCount);
  FOR_JOINTS_IDX(i) {
    positionsCopy[i] = positions[i];
  }

  // расширяем путь
  if (this->movingPath == NULL) { // создаем список
    this->movingPathLen++;
    this->movingPath = (Joint::posDeg**)malloc(sizeof(Joint::posDeg*) * 1);
  } else { // добавляем к списку
    this->movingPathLen++;
    this->movingPath = (Joint::posDeg**)realloc(this->movingPath, sizeof(Joint::posDeg*) * this->movingPathLen);
  }

  // добавляем в путь
  this->movingPath[this->movingPathLen - 1] = positionsCopy;

  // если точек до этого не было, то это новая цель
  if (this->isRunning && this->movingPathLen == 1) {
    this->_setTarget(positionsCopy);
  }

//  Serial.println("------- ADD POS -------");
//  Serial.print("Is running: ");
//  Serial.println(this->isRunning);
//  Serial.print("Path len: ");
//  Serial.println(this->movingPathLen);
//  Serial.print("Path: ");
//  this->_printMovingPath();
//  Serial.print("Cur time: ");
//  Serial.println(this->_currentMovingTime);
//  Serial.print("Start point: ");
//  PRINT_LIST(this->startPos);
//  Serial.println();
//  Serial.print("Target point: ");
//  PRINT_LIST(this->targetPos);
//  Serial.println();

  return this->movingPathLen - 1;
}

void DynamixelManipulatorMoves::_setTarget(const Joint::posDeg* targetPositions) {
  FOR_JOINTS_IDX(i) {
    this->startPos[i] = this->realPositions[i];
    this->targetPos[i] = targetPositions[i];
  }
  this->_currentMovingTime = 0;
  this->setAllJointsPosesDeg(this->targetPos);
}

void DynamixelManipulatorMoves::removeLastPoint() {
  this->removePoint(this->movingPathLen - 1);
}
void DynamixelManipulatorMoves::removePoint(size_t idx) {
  free(this->movingPath[idx]);
  for (size_t i = idx; i < this->movingPathLen - 1; i++) {
    this->movingPath[i] = this->movingPath[i + 1];
  }
  this->movingPathLen--;
}

void DynamixelManipulatorMoves::clearAllPoints() {
  for (size_t i = 0; i < this->movingPathLen; i++) {
    free(this->movingPath[i]);
  }
  free(this->movingPath);
  this->movingPath = NULL;
}

void DynamixelManipulatorMoves::pause() {
  this->isRunning = false;

  this->setAllJointsSpeedsDPS(Joint::speedDPS(0)); // Остановить
  this->currentTime = 0;
  this->_setTarget(this->realPositions); // Ставим стартом текущую позицию, чтоб плавно потом продолжил из этой точки
}
void DynamixelManipulatorMoves::go() {
  this->isRunning = true;

  if (this->movingPathLen > 0) {
    this->_setTarget(this->movingPath[0]);
  }
}
void DynamixelManipulatorMoves::_printMovingPath() {
  for (int i = 0; i < this->movingPathLen; i++) {
    FOR_JOINTS_IDX(j) {
      Serial.print(this->movingPath[i][j]);
      Serial.print(" ");
    }
    Serial.println(",");
  }
  Serial.println(".");
}

void DynamixelManipulatorMoves::LOOP(bool withPrint) {
  DynamixelManipulator::LOOP(withPrint);

  if (this->movingPathLen <= 0 || !this->isRunning)
    return;

  Joint::speedDPS currentSpeeds[this->jointsCount];
  Joint::speedDPS currentPoses[this->jointsCount];

  // Получаем скорости, которые надо выдать в текущий момент
  getTimedSmoothMovingSpeedDPS(this->startPos, this->targetPos, this->_currentMovingDuration, this->_currentMovingTime, currentSpeeds, currentPoses);
  float x, y, z;
  getPointByAngles(this->realPositions, x, y, z);
  Serial.print("Real point: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
  Serial.print("Real poses: ");
  PRINT_LIST(this->realPositions);
  Serial.println();
  getPointByAngles(currentPoses, x, y, z);
  Serial.print("Trgt point: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
  Serial.print("Trgt poses: ");
  PRINT_LIST(currentPoses);
  Serial.println();
  Serial.print("Set speeds: ");
  PRINT_LIST(currentSpeeds);
  Serial.println();

  this->setAllJointsSpeedsDPS(currentSpeeds);

  this->_currentMovingTime += UPDATE_DELAY;

  // Проверяем, закончен ли текущий отрезок
  bool targetReached = true;
//    FOR_JOINTS_IDX(i) { // Проверка, что мы вернулись к минимальной скорости
//      if (currentSpeeds[i] > MIN_JOINT_SPEED_DPS) {
//        targetReached = false;
//        break;
//      }
//    }
  FOR_JOINTS_IDX(i) {
//    Serial.print(fabs(this->realPositions[i] - this->targetPos[i]));
//    Serial.print("/");
//    Serial.print(this->jointsMoveAccuracies[i]);
//    Serial.print(" ");
    if (fabs(this->realPositions[i] - this->targetPos[i]) > this->jointsMoveAccuracies[i]) {  // Проверка по евклидовому расстоянию до точки
      targetReached = false;
    }
  }
//  Serial.println();

  if (targetReached) { // Достигли прошлую цель
    PRINT_MOVEln("POINT REACHED");
//    Serial.print("Reached point: ");
//    PRINT_LIST(this->movingPath[0]);
//    Serial.println();
    this->removePoint(0);
    if (this->movingPathLen > 0) { // Если есть новая цель
//      Serial.println("Path left. Get new target point [0]: ");
//      this->_printMovingPath();
//      PRINT_LIST(this->movingPath[0]);
//      Serial.println();
      this->_setTarget(this->movingPath[0]);
    }
//    Serial.println("0 points remained. Path done.");
  }
}
