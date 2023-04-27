#ifndef INCLUDE_REVERSE_KINEMATICS_
#define INCLUDE_REVERSE_KINEMATICS_

#include <math.h>

#define _L0   16.0
#define _L1_1 20.0
#define _L1_2 6.0
#define _L2   13.5
#define _L3   17.8

void getAnglesByTargetPoint(float x, float y, float z, const float* currentPoses, float* listToExportPoses, unsigned jointsCount, float* minPoses, float* maxPoses);
void getPointByAngles(const float* angles, float &exportX, float &exportY, float &exportZ);

#endif //  INCLUDE_REVERSE_KINEMATICS_
