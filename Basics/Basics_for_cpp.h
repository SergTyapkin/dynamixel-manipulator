#ifndef INCLUDE_BASICS_
#define INCLUDE_BASICS_


//#include <Dynamixel2Arduino.h>
//using namespace ControlTableItem;


#define JOINTS_COUNT 6
#define BAUDRATE  1000000

#define ERR_INIT_DEVICE            -11
#define ERR_INIT_JOINT             -12
#define ERR_SET_JOINT_MODE         -13
#define ERR_READ_JOINT_POS         -14
#define ERR_SET_MAX_JOINT_VELOCITY -15
#define ERR_SET_JOINT_POSITION     -16
#define ERR_SET_JOINT_VELOCITY     -17
#define ERR_ENABLE_JOINT           -18
#define ERR_DISABLE_JOINT          -19

#define MAX_JOINT_VELOCITY_PERCENT 5 // On first joint will be SPEED x2

#define MIN_RPM 1

typedef float pos;
typedef unsigned id;


//#define DXL_SERIAL Serial3
//const uint8_t DXL_DIR_PIN = 22;

//const float DXL_PROTOCOL_VERSION = 1.0;

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
    235, // max 3
    259, // max 4
    245, // max 5
    193, // max 6
};
#define MIN_POS(idx) minPoses[idx]
#define MAX_POS(idx) maxPoses[idx]

// --- MACROSes
#define FOR_JOINTS_IDX(name_of_var) for (id name_of_var = 0; name_of_var < JOINTS_COUNT; name_of_var++)
#define FOR_JOINTS_ID(name_of_var) for (id name_of_var = 1; name_of_var <= JOINTS_COUNT; name_of_var++)


#endif //  INCLUDE_BASICS_
