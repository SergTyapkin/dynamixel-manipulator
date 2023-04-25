#ifndef INCLUDE_BASICS_
#define INCLUDE_BASICS_


#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

//L: 180.00
//T1: 2.00
//T2: 3.00
//Tmax: 5.00
//L1: 60.00
//L2: 120.00

//#define GRAPH_MODE
#ifdef GRAPH_MODE
  #define PRINT_SETUPS false
  #define PRINT_SETS   false
  #define PRINT_ERRS   false
#else
  #define PRINT_SETUPS true
  #define PRINT_SETS   true
  #define PRINT_ERRS   true
#endif


#define JOINTS_COUNT 6
#define BAUDRATE  1000000

#define MAX_JOINT_SPEED_PERCENT 5 // On first joint will be SPEED x2

#define MIN_RPM 1

typedef float pos;
typedef unsigned id;


#define DXL_SERIAL Serial3
const uint8_t DXL_DIR_PIN = 22;

const float DXL_PROTOCOL_VERSION = 1.0;

// ---
#define POS_MIN 0.0
#define POS_MAX 360.0
const float minPoses[JOINTS_COUNT] = {
    POS_MIN, // min 1
    73, // min 2
    83, // min 3
    78, // min 4
    3,  // min 5
    120, // min 6
};
const float maxPoses[JOINTS_COUNT] = {
    POS_MAX, // max 1
    283, // max 2
    275, // max 3
    253, // max 4
    245, // max 5
    193, // max 6
};
#define MIN_POS(idx) minPoses[idx]
#define MAX_POS(idx) maxPoses[idx]

// --- MACROSes
#define FOR_JOINTS_IDX(name_of_var) for (id name_of_var = 0; name_of_var < JOINTS_COUNT; name_of_var++)
#define FOR_JOINTS_ID(name_of_var) for (id name_of_var = 1; name_of_var <= JOINTS_COUNT; name_of_var++)

#define ENTER_SECTION \
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
        setJointSpeed(i, 7);        \
      static pos __null_pos_1[] = {135, 180, 155, 164, 180, 170}; \
      setAllJointsPositions(__null_pos_1);                        \
      delay(3000);                     \
      FOR_JOINTS_ID(i)                 \
        setJointSpeed(i, 3);        \
      static pos __null_pos_2[] = {135, 155, 132, 164, 180, 170}; \
      setAllJointsPositions(__null_pos_2);                        \
      delay(500);                      \
      FOR_JOINTS_ID(i)                 \
        setJointSpeed(i, 1);        \
      delay(2500);    \
      disableAll();   \
      exit(0);        \
    }                 \
  }
#define ON_ENTER(number) if (__isEnterPressedNow && __entersPressed == number)
#define ON_ANY_ENTER if (__isEnterPressedNow)

// --- Prints ---
#define __PRINT_IFDEF(var, value) \
  if(var)                         \
    Serial.print(value);
#define __PRINTln_IFDEF(var, value) \
  if(var)                         \
    Serial.println(value);
#define PRINT_SET(value) __PRINT_IFDEF(PRINT_SETS, value)
#define PRINT_ERR(value) __PRINT_IFDEF(PRINT_ERRS, value)
#define PRINT_SETUP(value) __PRINT_IFDEF(PRINT_SETUPS, value)
#define PRINT_SETln(value) __PRINTln_IFDEF(PRINT_SETS, value)
#define PRINT_ERRln(value) __PRINTln_IFDEF(PRINT_ERRS, value)
#define PRINT_SETUPln(value) __PRINTln_IFDEF(PRINT_SETUPS, value)

// --- Global vars
extern Dynamixel2Arduino dxl;
extern pos actualJointsPositions[JOINTS_COUNT]; // READONLY!! Always contains current joints positions
extern float actualJointsSpeedDPS[JOINTS_COUNT]; // READONLY!! Always contains current joints Speeds
extern float actualJointsCurrents[JOINTS_COUNT]; // READONLY!! Always contains current joints currents



// ------- Default function --------
void SETUP();
void LOOP_PRINT_GRAPH_STATS();
void LOOP_PRINT_CHANGED_STATS();

// ------- Set positions functions --------
void setJointPosition(id idx, pos position);
void setAllJointsPositions(pos* jointsPositions);
// ------- Set Speed functions --------
void setJointSpeed(id idx, float percents);
void setJointSpeedDPS(id idx, float dps);
void setAllJointsSpeeds(float* jointsSpeed);
void setAllJointsSpeedsDPS(float* jointsSpeed);
// ------- Work with torque functions --------
void enable(id idx);
void disable(id idx);
void enableAll();
void disableAll();


// -------- Read stats --------
void readAllJointsPositions(pos* targetList);
void readAllJointsSpeeds(float* targetList);
void readAllJointsSpeedsDPS(float* targetList);
void readAllJointsCurrents(float* targetList);
// ------- Print stats --------
void _printList(const pos* list);
void printPositions(const pos* list);
void printSpeeds(const pos* list);
void printCurrents(const pos* list);

void printPositionsIfChanged(const pos* list);


#endif //  INCLUDE_BASICS_
