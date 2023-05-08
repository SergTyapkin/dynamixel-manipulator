#include "Kinematics.h"
//#include <iostream>
//using namespace std;


#define _WEIGHT_FOR_PENALTIES 10

#define DEG_MAX 360.0

float rad(float deg) {
  return deg / DEG_MAX * 2 * M_PI;
}
void getPointByAngles(const float* angles, float &exportX, float &exportY, float &exportZ) {
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
void get2DPointByAngles(const float* angles, float &exportXY, float &exportZ) {
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
float _delta(const float* phi, float xy0, float z0, unsigned jointsCount) {
  float phi_int[jointsCount];
  for (unsigned i = 0; i < jointsCount; i++) {
    phi_int[i] = float(phi[i]);
  }
  float xy, z;
  get2DPointByAngles(phi_int, xy, z);
//  cout << "Point: " << x << ", " << y << ", " << z << endl;
//  cout << "Target point: " << x0 << ", " << y0 << ", " << z0 << endl;
  const float res = sqrt(pow((xy - xy0), 2) + pow((z - z0), 2));
//  cout << "Delta: " << res << endl;
  return res;
}
float _penaltySum(const float* phi, unsigned jointsCount, const float* minPoses, const float* maxPoses) {
  float res = 0;
  for (unsigned i = 0; i < jointsCount; i++) {
    if (phi[i] < minPoses[i])
      res += minPoses[i] - phi[i];
    else if (phi[i] > maxPoses[i])
      res += phi[i] - maxPoses[i];
  }
//  cout << "Penalty: " << res << endl;
  return res;
}
float _targetFunction(const float* phi, float xy0, float z0, unsigned jointsCount, const float* minPoses, const float* maxPoses) {
  return _delta(phi, xy0, z0, jointsCount) + _WEIGHT_FOR_PENALTIES * _penaltySum(phi, jointsCount, minPoses, maxPoses);
}

// ------

void getAnglesByTargetPoint(float x, float y, float z, const float* currentPoses, float* listToExportPoses, unsigned jointsCount, const float* minPoses, const float* maxPoses) {
  float H = 1;

  float prevPoses[jointsCount];
  float newPoses[jointsCount];
  float tmpPoses[jointsCount];
  for (unsigned i = 0; i < jointsCount; i++) {
//    cout << "[" << i << "]:" << currentPoses[i] << " ";
    prevPoses[i] = newPoses[i] = tmpPoses[i] = float(currentPoses[i]);
  }
//  cout << endl;

  // Предварительный геометрический поиск поворота базы
  float baseAngle;
  baseAngle = atan2f(y, x) / 2 / M_PI * DEG_MAX;
  if (baseAngle < 0)
    baseAngle = DEG_MAX + baseAngle;
  prevPoses[0] = newPoses[0] = tmpPoses[0] = baseAngle;
  float xy = sqrt(x * x + y * y);

  // Реализуем поиск минимума через градиентный метод с дроблением шага
  const float targetEps = 0.1;
  float currentDelta = targetEps + 1;
  float currentDerivations[jointsCount];
  float derivationsNorma = 0;
  while(fabs(currentDelta) > targetEps) {  // Если смещение на прошлом шаге было достаточно велико, чтобы продолжить
    float deltaOfPosToFindDerivation = 1;  // для нахождения частной производной в точке нужно определить какое-то малое приращение
    for(unsigned i = 1; i < jointsCount; i++) {  // Для каждого параметра-угла поворота
      tmpPoses[i] = prevPoses[i] + deltaOfPosToFindDerivation;  // ищем производную - то есть добавляем небольшое смещение по этому прараметру

      const float derivation = (_targetFunction(prevPoses, xy, z, jointsCount, minPoses, maxPoses) - _targetFunction(tmpPoses, xy, z, jointsCount, minPoses, maxPoses)) / deltaOfPosToFindDerivation;  // и ищем численно производную
//      cout << i << ": " << derivation << endl;
      currentDerivations[i] = derivation;  // сохраняем производную
      derivationsNorma += pow(derivation, 2);

      tmpPoses[i] = prevPoses[i]; // теперь tmpPoses = prevPoses
    }
    derivationsNorma = sqrt(derivationsNorma);

    const float coeffTolerance = 0.1;
    const float coeffDecreasing = 0.95;
    for(unsigned i = 1; i < jointsCount; i++) {  // вычисляем новые значения для каждого параметра
      // дробление шага
      const float newPos = prevPoses[i] + H * currentDerivations[i];
      if (newPos > prevPoses[i] + coeffTolerance * H * derivationsNorma) {
//        H *= coeffDecreasing; // дробление шага отключено
      }
      newPoses[i] = prevPoses[i] + H * currentDerivations[i];  // шаг определен, спускаемся по градиенту. Если бы был -, то поднимались бы
    }

    currentDelta = 0;
    for(unsigned i = 1; i < jointsCount; i++) {
//      cout << "[" << i << "]:" << newPoses[i] << " ";
      currentDelta += newPoses[i] - prevPoses[i];  // Ищем общее смещение как сумму смещений по каждому из параметров
      prevPoses[i] = tmpPoses[i] = newPoses[i];  // Копируем новые значения в старые, чтобы начать новую итерацию
    }
//    cout << endl;
//    cout << "Total delta: " << currentDelta << endl;
  }

  for (unsigned i = 0; i < jointsCount; i++) {
    listToExportPoses[i] = float(newPoses[i]);
  }
}
