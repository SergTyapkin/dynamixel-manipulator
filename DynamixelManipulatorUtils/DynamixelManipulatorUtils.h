#ifndef INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_
#define INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_


#include "../DynamixelManipulator/DynamixelManipulator.h"


// --- MACROses
#ifndef MANIPULATOR_CLASS_VARIABLE
  #error You need to specify your manipulator class variable name with "#define MANIPULATOR_CLASS_VARIABLE <variable_name>"
#endif
#ifndef MANIPULATOR_POS_DEFAULT
  #error You need to specify default position in degrees of your manipulator "#define MANIPULATOR_POS_DEFAULT <{pos1, pos2, pos3, ...}>"
#endif
#ifndef MANIPULATOR_POS_OFF
  #error You need to specify off posiniton in degrees of your manipulator "#define MANIPULATOR_POS_OFF <{pos1, pos2, pos3, ...}>"
#endif

#define MANIPULATOR_TO_DEFAULT_POS \
  static Joint::posDeg __null_pos_0[] = MANIPULATOR_POS_DEFAULT; \
  MANIPULATOR_CLASS_VARIABLE.setAllJointsPosesDeg(__null_pos_0);

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
      MANIPULATOR_CLASS_VARIABLE.setAllJointsSpeedsDPS(40);          \
      static Joint::posDeg __null_pos_1[] = MANIPULATOR_POS_DEFAULT; \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsPosesDeg(__null_pos_1);                        \
      delay(5000);                     \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsSpeedsDPS(10);         \
      static Joint::posDeg __null_pos_2[] = MANIPULATOR_POS_OFF; \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsPosesDeg(__null_pos_2);                        \
      delay(1000);                      \
      MANIPULATOR_CLASS_VARIABLE.setAllJointsSpeedsDPS(1); \
      delay(2500);    \
      MANIPULATOR_CLASS_VARIABLE.disableAllJoints();   \
      exit(0);        \
    }                 \
  }
#define ON_KEY(symbol) if (__readedSymbol != '\0' && __readedSymbol == symbol)
#define ON_ENTER(number) if (__isEnterPressedNow && __entersPressed == number)
#define ON_ANY_ENTER if (__isEnterPressedNow)
#define ON_DIGIT(var_name) int var_name = __readedSymbol - '0'; if (__readedSymbol != '\0' && __readedSymbol >= '0' && __readedSymbol <= '9')


#endif //  INCLUDE_DYNAMIXEL_MANIPULATOR_UTILS_
