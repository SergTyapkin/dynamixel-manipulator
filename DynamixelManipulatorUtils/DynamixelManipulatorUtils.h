#ifndef INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_
#define INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_


#include "../DynamixelManipulator/DynamixelManipulator.h"


// --- MACROses
#ifndef MANIPULATOR_CLASS_VARIABLE
  #error You need to specify joints count in your manipulator class variable name with "#define MANIPULATOR_CLASS_VARIABLE <variable_name>"
#endif
#define FOR_JOINTS_IDX(name_of_var) for (Joint::index name_of_var = 0; name_of_var < MANIPULATOR_CLASS_VARIABLE.jointsCount; name_of_var++)
#define FOR_JOINTS_ID(name_of_var) for (Joint::index __i = 0, name_of_var = MANIPULATOR_CLASS_VARIABLE.jointsIds[__i]; __i < MANIPULATOR_CLASS_VARIABLE.jointsCount; __i++, name_of_var = this.jointsIds[__i])


#define KEYS_SECTION \
  static unsigned __entersPressed = 0; \
  bool __isEnterPressedNow = false;    \
  char __readedSymbol = '\0';          \
  if (Serial.available()) {            \
    __readedSymbol = Serial.read();    \
    if (__readedSymbol == '\n') {      \
      __entersPressed += 1;            \
      __isEnterPressedNow = true;      \
    }                                  \
    if (__readedSymbol == 'Q') {       \
      FOR_JOINTS_ID(i)                 \
        MANIPULATOR_CLASS_VARIABLE.setJointSpeed(i, 7);           \
      static pos __null_pos_1[] = {135, 180, 155, 164, 180, 170}; \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsPositions(__null_pos_1);                        \
      delay(3000);                     \
      FOR_JOINTS_ID(i)                 \
        MANIPULATOR_CLASS_VARIABLE.setJointSpeed(i, 3);           \
      static pos __null_pos_2[] = {135, 155, 132, 164, 180, 170}; \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsPositions(__null_pos_2);                        \
      delay(500);                      \
      FOR_JOINTS_ID(i)                 \
        MANIPULATOR_CLASS_VARIABLE.setJointSpeed(i, 1); \
      delay(2500);    \
      MANIPULATOR_CLASS_VARIABLE.disableAll();   \
      exit(0);        \
    }                 \
  }
#define ON_KEY(symbol) if (__readedSymbol != '\0' && __readedSymbol == symbol)
#define ON_ENTER(number) if (__isEnterPressedNow && __entersPressed == number)
#define ON_ANY_ENTER if (__isEnterPressedNow)


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_
