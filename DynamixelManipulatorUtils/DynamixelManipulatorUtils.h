#ifndef INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_
#define INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_


#include "../DynamixelManipulator/DynamixelManipulator.h"


// --- MACROses
#ifndef MANIPULATOR_CLASS_VARIABLE
  #error You need to specify your manipulator class variable name with "#define MANIPULATOR_CLASS_VARIABLE <variable_name>"
#endif


#define KEYS_SECTION \
  static unsigned __entersPressed = 0; \
  bool __isEnterPressedNow = false;    \
  char __readedSymbol = '\0';          \
  if (Serial.available()) {            \
    __readedSymbol = Serial.read();    \
    if (__readedSymbol == '\n') {      \
      __entersPressed += 1;            \
      __isEnterPressedNow = true;      \
    } else if (__readedSymbol == 'Q') {   \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsSpeedsDPS(7);           \
      static Joint::posDeg __null_pos_1[] = {135, 180, 155, 164, 180, 170}; \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsPosesDeg(__null_pos_1);                        \
      delay(3000);                     \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsSpeedsDPS(3);           \
      static Joint::posDeg __null_pos_2[] = {135, 155, 132, 164, 180, 170}; \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsPosesDeg(__null_pos_2);                        \
      delay(500);                      \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsSpeedsDPS(1); \
      delay(2500);    \
      MANIPULATOR_CLASS_VARIABLE.disableAllJoints();   \
      exit(0);        \
    }                 \
  }
#define ON_KEY(symbol) if (__readedSymbol != '\0' && __readedSymbol == symbol)
#define ON_ENTER(number) if (__isEnterPressedNow && __entersPressed == number)
#define ON_ANY_ENTER if (__isEnterPressedNow)


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_
