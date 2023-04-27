#include "DynamixelManipulatorMoves.h"

SerialPort _dynamixel_manipulator_moves_serial;


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
    USBSerial serialPort,
    unsigned baudrate,
    unsigned dirPin,
    float protocolVersion
): DynamixelManipulator(jointsCount, minJointsPoses, maxJointsPoses, jointsIds, serialPort, baudrate, dirPin, protocolVersion) {
  this->startPos = this->_newJointsArray<Joint::posDeg>();
  this->targetPos = this->_newJointsArray<Joint::posDeg>();
}
DynamixelManipulatorMoves::~DynamixelManipulatorMoves() {
  free(this->startPos);
  free(this->targetPos);
}

size_t DynamixelManipulatorMoves::addPoint(float x, float y, float z) {
  if (this->movingPath == NULL) { // создаем список
    this->movingPathLen++;
    this->movingPath = (Joint::posDeg**)malloc(sizeof(Joint::posDeg*) * 1);
  } else { // добавляем к списку
    this->movingPathLen++;
    this->movingPath = (Joint::posDeg**)realloc(this->movingPath, sizeof(Joint::posDeg*) * this->movingPathLen);
  }

  // получаем новую точку
  Joint::posDeg* newPoses = (Joint::posDeg*)malloc(sizeof(Joint::posDeg) * this->jointsCount);
  getAnglesByTargetPoint(x, y, z, this->realPositions, newPoses, this->jointsCount, this->minJointsPoses, this->maxJointsPoses);
  this->movingPath[this->movingPathLen - 1] = newPoses;

  // если точек до этого не было, то это новая цель
  if (this->movingPathLen == 1) {
    this->_setTarget(newPoses);
  }

  return this->movingPathLen - 1;
}

void DynamixelManipulatorMoves::_setTarget(Joint::posDeg* targetPositions) {
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
}
void DynamixelManipulatorMoves::go() {
  this->isRunning = true;
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
    FOR_JOINTS_IDX(i) {
      if (currentSpeeds[i] > MIN_JOINT_SPEED_DPS) {
        targetReached = false;
        break;
      }
    }
    if (targetReached) { // Достигли прошлую цель
      this->removePoint(0);
      if (this->movingPathLen > 0) { // Если есть новая цель
        this->_setTarget(this->movingPath[0]);
      }
    }
  }

  if (!this->isRunning) { // В случае паузы
    this->setAllJointsSpeedsDPS(Joint::speedDPS(0)); // Остановить
    this->currentTime = 0;
    this->_setTarget(this->targetPos); // Ставим стартом текущую позицию, чтоб плавно потом продолжил из этой точки
  }
}
