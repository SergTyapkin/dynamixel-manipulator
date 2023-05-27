#define _CPP_COMPILE_UNIQUE_NAME _serial_dynamixel_manipulator_moves
#include "DynamixelManipulatorMoves.h"
CPP_COMPILE_CPP_HEADER



void DynamixelManipulatorMoves::getTimedSmoothMovingSpeedDPS(const Joint::posDeg* start, const Joint::posDeg* end, float duration, const float currentTime, float* exportSpeedsDPS) {
  FOR_JOINTS_IDX(i) {
    const float L = end[i] - start[i];
    float T1 = duration / 2;
    float acceleration = L / pow(T1, 2);
    if (acceleration * T1 > MAX_JOINT_SPEED_DPS) {
      T1 = duration - L / MAX_JOINT_SPEED_DPS;
      acceleration = MAX_JOINT_SPEED_DPS / T1;

      const float min_T1 = MAX_JOINT_SPEED_DPS / MAX_JOINT_ACCELERATION_DPS;
      if (T1 <= min_T1) {
        duration = -T1 + min_T1 * 2;
        T1 = min_T1;
        acceleration = MAX_JOINT_ACCELERATION_DPS;
      }
    }

    const float currentL = this->realPositions[i] - start[i];
    const float speedAfterT1 = T1 * acceleration;
    const float L1 = acceleration * pow(T1, 2) / 2;
    const float L2 = L - L1;
    const float T2 = duration - T1;


    float targetL;
    if (currentTime <= T1) {
      // Starting
      exportSpeedsDPS[i] = speedAfterT1 * ((currentL - 0) / L1);
      targetL = acceleration * pow(currentTime, 2) / 2;
    } else if (currentTime <= T2) {
      // Center on max speed
      exportSpeedsDPS[i] = speedAfterT1;
      targetL = L1 + (currentTime - T1) * MAX_JOINT_SPEED_DPS;
    } else if (currentTime <= duration) {
      // Stopping
      exportSpeedsDPS[i] = speedAfterT1 - speedAfterT1 * ((currentL - L2) / L1);
      targetL = L2 + ((currentTime - T2) * speedAfterT1 - acceleration * pow(currentTime - T2, 2) / 2);
    } else {
      // No movement. Must be already in end pos
      exportSpeedsDPS[i] = MIN_JOINT_SPEED_DPS;
      targetL = L;
    }
    exportSpeedsDPS[i] += (targetL - currentL) * 1;
  }
}

DynamixelManipulatorMoves::DynamixelManipulatorMoves(
    size_t jointsCount,
    const Joint::posDeg* minJointsPoses,
    const Joint::posDeg* maxJointsPoses,
    const Joint::id* jointsIds,
    UARTClass serialPort,
    unsigned baudrate,
    unsigned dirPin,
    float protocolVersion
): DynamixelManipulator(jointsCount, minJointsPoses, maxJointsPoses, jointsIds, serialPort, baudrate, dirPin, protocolVersion) {
  this->startPos = this->_newJointsArray();
  this->targetPos = this->_newJointsArray();

  DynamixelManipulator::LOOP_UPDATE(); // получаем текущую позицию
  FOR_JOINTS_IDX(i) {
    this->startPos[i] = this->realPositions[i]; // копируем в стартовую
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

void DynamixelManipulatorMoves::LOOP_UPDATE() {
  DynamixelManipulator::LOOP_UPDATE();

  Joint::speedDPS currentSpeeds[this->jointsCount];

  if (this->movingPathLen > 0 && this->isRunning) {
    // Получаем скорости, которые надо выдать в текущий момент
    getTimedSmoothMovingSpeedDPS(this->startPos, this->targetPos, this->_currentMovingDuration, this->_currentMovingTime, currentSpeeds);

    float x, y, z;
    getPointByAngles(this->realPositions, x, y, z);
    Serial.print("Point: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
    Serial.print("Speed: ");
    Serial.println(currentSpeeds[0]);

    this->setAllJointsSpeedsDPS(currentSpeeds);

    this->_currentMovingTime += 0.050;

    // Проверяем, закончен ли текущий отрезок
    bool targetReached = true;
//    FOR_JOINTS_IDX(i) { // Проверка, что мы вернулись к минимальной скорости
//      if (currentSpeeds[i] > MIN_JOINT_SPEED_DPS) {
//        targetReached = false;
//        break;
//      }
//    }
    FOR_JOINTS_IDX(i) {
      Serial.println(abs(this->realPositions[i] - this->targetPos[i]));
      if (abs(this->realPositions[i] - this->targetPos[i]) > ACCURACY_TARGET_DIST) {
        targetReached = false;
      }
    }

    if (targetReached) { // Достигли прошлую цель
      Serial.println("POINT REACHED");
      this->removePoint(0);
      if (this->movingPathLen > 0) { // Если есть новая цель
        this->_setTarget(this->movingPath[0]);
      }
    }
  }
}
