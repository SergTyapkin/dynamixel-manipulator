#include "Kinematics.h"
//#include <iostream>
//using namespace std;


#define _WEIGHT_FOR_PENALTIES 10


float rad(float deg) {
  return deg / POS_MAX * 2 * M_PI;
}
void getPointByAngles(const pos* angles, float &exportX, float &exportY, float &exportZ) {
  exportX =
      (0 +
       _L1_1 * cos(rad(angles[1] - 90)) +
       _L1_2 * cos(rad(angles[1] - 180)) +
       _L2   * cos(rad(angles[1] - 90 + 90 + angles[2])) +
       _L3   * cos(rad(angles[1] - 90 + 90 + angles[2] + angles[3] - 180)))
      * cos(rad(angles[0]));
  exportY =
      (0 +
       _L1_1 * cos(rad(angles[1] - 90)) +
       _L1_2 * cos(rad(angles[1] - 180)) +
       _L2   * cos(rad(angles[1] - 90 + 90 + angles[2])) +
       _L3   * cos(rad(angles[1] - 90 + 90 + angles[2] + angles[3] - 180)))
      * sin(rad(angles[0]));
  exportZ =
      _L0 +
      _L1_1 * sin(rad(angles[1] - 90)) +
      _L1_2 * sin(rad(angles[1] - 180)) +
      _L2   * sin(rad(angles[1] - 90 + 90 + angles[2])) +
      _L3   * sin(rad(angles[1] - 90 + 90 + angles[2] + angles[3] - 180));
}
void get2DPointByAngles(const pos* angles, float &exportXY, float &exportZ) {
  exportXY =
      0 +
     _L1_1 * cos(rad(angles[1] - 90)) +
     _L1_2 * cos(rad(angles[1] - 180)) +
     _L2   * cos(rad(angles[1] - 90 + 90 + angles[2])) +
     _L3   * cos(rad(angles[1] - 90 + 90 + angles[2] + angles[3] - 180));
  exportZ =
      _L0 +
      _L1_1 * sin(rad(angles[1] - 90)) +
      _L1_2 * sin(rad(angles[1] - 180)) +
      _L2   * sin(rad(angles[1] - 90 + 90 + angles[2])) +
      _L3   * sin(rad(angles[1] - 90 + 90 + angles[2] + angles[3] - 180));
}
float _delta(const float* phi, float xy0, float z0) {
  pos phi_int[JOINTS_COUNT];
  FOR_JOINTS_IDX(i) {
    phi_int[i] = pos(phi[i]);
  }
  float xy, z;
  get2DPointByAngles(phi_int, xy, z);
//  cout << "Point: " << x << ", " << y << ", " << z << endl;
//  cout << "Target point: " << x0 << ", " << y0 << ", " << z0 << endl;
  const float res = sqrt(pow((xy - xy0), 2) + pow((z - z0), 2));
//  cout << "Delta: " << res << endl;
  return res;
}
float _penaltySum(const float* phi) {
  float res = 0;
  FOR_JOINTS_IDX(i) {
    if (phi[i] < MIN_POS(i))
      res += MIN_POS(i) - phi[i];
    else if (phi[i] > MAX_POS(i))
      res += phi[i] - MAX_POS(i);
  }
//  cout << "Penalty: " << res << endl;
  return res;
}
float _targetFunction(const float* phi, float xy0, float z0) {
  return _delta(phi, xy0, z0) + _WEIGHT_FOR_PENALTIES * _penaltySum(phi);
}

// ------

void getAnglesByTargetPoint(float x, float y, float z, const pos* currentPoses, pos* listToExportPoses) {
  float H = 1;

  float prevPoses[JOINTS_COUNT];
  float newPoses[JOINTS_COUNT];
  float tmpPoses[JOINTS_COUNT];
  FOR_JOINTS_IDX(i) {
//    cout << "[" << i << "]:" << currentPoses[i] << " ";
    prevPoses[i] = newPoses[i] = tmpPoses[i] = float(currentPoses[i]);
  }
//  cout << endl;

  // Предварительный геометрический поиск поворота базы
  float baseAngle;
  baseAngle = atan2f(y, x) / 2 / M_PI * POS_MAX;
  if (baseAngle < 0)
    baseAngle = POS_MAX + baseAngle;
  prevPoses[0] = newPoses[0] = tmpPoses[0] = baseAngle;
  float xy = sqrt(x * x + y * y);

  // Реализуем поиск минимума через градиентный метод с дроблением шага
  const float targetEps = 0.1;
  float currentDelta = targetEps + 1;
  float currentDerivations[JOINTS_COUNT];
  float derivationsNorma = 0;
  while(fabs(currentDelta) > targetEps) {  // Если смещение на прошлом шаге было достаточно велико, чтобы продолжить
    float deltaOfPosToFindDerivation = 1;  // для нахождения частной производной в точке нужно определить какое-то малое приращение
    for(unsigned i = 1; i < JOINTS_COUNT; i++) {  // Для каждого параметра-угла поворота
      tmpPoses[i] = prevPoses[i] + deltaOfPosToFindDerivation;  // ищем производную - то есть добавляем небольшое смещение по этому прараметру

      const float derivation = (_targetFunction(prevPoses, xy, z) - _targetFunction(tmpPoses, xy, z)) / deltaOfPosToFindDerivation;  // и ищем численно производную
//      cout << i << ": " << derivation << endl;
      currentDerivations[i] = derivation;  // сохраняем производную
      derivationsNorma += pow(derivation, 2);

      tmpPoses[i] = prevPoses[i]; // теперь tmpPoses = prevPoses
    }
    derivationsNorma = sqrt(derivationsNorma);

    const float coeffTolerance = 0.1;
    const float coeffDecreasing = 0.95;
    for(unsigned i = 1; i < JOINTS_COUNT; i++) {  // вычисляем новые значения для каждого параметра
      // дробление шага
      const float newPos = prevPoses[i] + H * currentDerivations[i];
      if (newPos > prevPoses[i] + coeffTolerance * H * derivationsNorma) {
//        H *= coeffDecreasing; // дробление шага отключено
      }
      newPoses[i] = prevPoses[i] + H * currentDerivations[i];  // шаг определен, спускаемся по градиенту. Если бы был -, то поднимались бы
    }

    currentDelta = 0;
    for(unsigned i = 1; i < JOINTS_COUNT; i++) {
//      cout << "[" << i << "]:" << newPoses[i] << " ";
      currentDelta += newPoses[i] - prevPoses[i];  // Ищем общее смещение как сумму смещений по каждому из параметров
      prevPoses[i] = tmpPoses[i] = newPoses[i];  // Копируем новые значения в старые, чтобы начать новую итерацию
    }
//    cout << endl;
//    cout << "Total delta: " << currentDelta << endl;
  }

  FOR_JOINTS_IDX(i) {
    listToExportPoses[i] = pos(newPoses[i]);
  }
}
