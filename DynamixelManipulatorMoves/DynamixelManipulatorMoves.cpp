#include "DynamixelManipulatorMoves.h"


#define POSES_DEG_ACCURANCY = 1;


void getTimedSmoothMovingSpeedDPS(const pos* start, const pos* end, float duration, const float currentTime, float* exportSpeedsDPS) {
  FOR_JOINTS_IDX(i) {
    const float L = end[i] - start[i];
    float T1 = duration / 2;
    float acceleration = L / pow(T1, 2);
    if (acceleration * T1 > MAX_SPEED_DPS) {
      T1 = duration - L / MAX_SPEED_DPS;
      acceleration = MAX_SPEED_DPS / T1;

      const float min_T1 = MAX_SPEED_DPS / MAX_ACCELERATION_DPS;
      if (T1 <= min_T1) {
        duration = -T1 + min_T1 * 2;
        T1 = min_T1;
        acceleration = MAX_ACCELERATION_DPS;
      }
    }

    const float currentL = actualJointsPositions[i] - start[i];
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
      targetL = L1 + (currentTime - T1) * MAX_SPEED_DPS;
    } else if (currentTime <= duration) {
      // Stopping
      exportSpeedsDPS[i] = speedAfterT1 - speedAfterT1 * ((currentL - L2) / L1);
      targetL = L2 + ((currentTime - T2) * speedAfterT1 - acceleration * pow(currentTime - T2, 2) / 2);
    } else {
      // No movement. Must be already in end pos
      exportSpeedsDPS[i] = MIN_SPEED_DPS;
      targetL = L;
    }
    exportSpeedsDPS[i] += (targetL - currentL) * 1;
  }
}


class DynamixelManipulatorMoves: public DynamixelManipulator {
public:
  size_t addPoint(float x, float y, float z) {
    if (this.movingPath == NULL) { // создаем список
      this.movingPathLen++;
      this.movingPath = (Joint::posDeg**)malloc(sizeof(Joint::posDeg*) * 1);
    } else { // добавляем к списку
      this.movingPathLen++;
      this.movingPath = (Joint::posDeg**)realloc(this.movingPath, sizeof(Joint::posDeg*) * this.movingPathLen);
    }

    // получаем новую точку
    Joint::posDeg* newPoses = (Joint::posDeg*)malloc(sizeof(Joint::posDeg) * this.jointsCount);
    getAnglesByTargetPoint(x, y, z, this.realPosition, newPoses);
    this.movingPath[this.movingPathLen - 1] = newPoses;

    // если точек до этого не было, то это новая цель
    if (this.movingPathLen == 1) {
      this._setTarget(newPoses);
    }
  }

  void _setTarget(Joint::posDeg* targetPositions) {
    FOR_JOINTS_IDX(i) {
      this.startPos[i] = this.realPositions[i];
      this.targetPos[i] = targetPositions[i];
    }
    this._currentMovingTime = 0;
    this.setAllJointsPosesDeg()
  }

  void removeLastPoint() {
    this.removePoint(this.movingPathLen - 1);
  }
  void removePoint(size_t idx) {
    free(this.movingPath[idx]);
    for (size_t i = idx; i < this.movingPathLen - 1; i++) {
      this.movingPath[i] = this.movingPath[i + 1];
    }
    this.movingPathLen--;
  }

  void clearAllPoints() {
    for (size_t i = 0; i < this.movingPathLen; i++) {
      free(this.movingPath[i]);
    }
    free(this.movingPath);
    this.movingPath = NULL;
  }

  void pause() {
    this.isRunning = false;
  }
  void resume() {
    this.isRunning = true;
  }

  void LOOP_UPDATE() {
    DynamixelManipulator.LOOP_UPDATE();

    Joint::speedDPS currentSpeeds[this.jointsCount];

    if (this.movingPathlen > 0 && this.isRunning) {
      // Получаем скорости, которые надо выдать в текущий момент
      getTimedSmoothMovingSpeedDPS(this.startPos, this.targetPos, this._currentMovingDiration, this._currentMovingTime, currentSpeeds);

      float x, y, z;
      getPointByAngles(actualJointsPositions, x, y, z);
      Serial.print("Point: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
      Serial.print("Speed: ");
      Serial.println(currentSpeeds[0]);

      this.setAllJointsSpeedsDPS(currentSpeeds);

      this._currentMovingTime += 0.050;

      // Проверяем, закончен ли текущий отрезок
      bool targetReached = true;
      FOR_JOINTS_IDX(i) {
        if (currentSpeeds[i] != 0) {
          targetReached = false;
          break;
        }
      }
      if (targetReached) { // Достигли прошлую цель
        this.removePoint(0);
        if (this.movingPathLen > 0) { // Если есть новая цель
          this._setTarget(this.movingPath[0]);
        }
      }
    }

    if (!this.isRunning) { // В случае паузы
      this.setAllJointsSpeedsDPS(0); // Остановить
      this.currentTime = 0;
      FOR_JOINTS_IDX(i) { // Ставим стартом текущую позицию, чтоб плавно потом продолжил из этой точки
        this.startPos[i] = this.realPositions[i];
      }
    }
  }
protected:
  bool isRunning = true;
  size_t movingPathLen = 0;
  float _currentMovingDiration = 1;
  float _currentMovingTime;
  Joint::posDeg** movingPath = NULL;
  Joint::posDeg startPos[this.jointsCount];
  Joint::posDeg targetPos[this.jointsCount];
};


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_
