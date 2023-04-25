//
// Created by Сергей Тяпкин on 22.01.2023.
//
#include "../Kinematics.h"
#include <iostream>
using namespace std;

//pos newPoses[JOINTS_COUNT] = {45, 62+90, 90+30, 180, 0, 0};
int main() {
  const pos currentPoses[JOINTS_COUNT] = {90, 180, 180, 180, 180, 180};
  pos newPoses[JOINTS_COUNT];
  getAnglesByTargetPoint(-20, 25, 3, currentPoses, newPoses);
  float x, y, z;
  getPointByAngles(newPoses, x, y, z);
  cout << "Goes to: " << x << ", " << y << ", " << z << endl;

  FOR_JOINTS_IDX(i) {
    cout << "[" << i << "]:" << newPoses[i] << " ";
  }
  cout << endl;

  return 0;
}
