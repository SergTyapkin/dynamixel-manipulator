//
// Created by Сергей Тяпкин1 on 04.05.2023.
//

//#define _CPP_COMPILE

/**
 (!!!) You must write after include this file in each .cpp endpoint file:
 #define _CPP_COMPILE_UNIQUE_NAME <_some_unique_name>
**/

#include <stdio.h>
#ifdef _WIN32
  #include <windows.h>
#else
  #include <unistd.h>
#endif

#ifndef LIBRARIES_CPPCOMPILEUTILS_H
#define LIBRARIES_CPPCOMPILEUTILS_H

#ifdef _CPP_COMPILE
  #warning "!!! COMPILING IN CPP MODE !!!"

  #ifndef _CPP_COMPILE_UNIQUE_NAME
  #error "_CPP_COMPILE_UNIQUE_NAME must be defined"
  #endif
  #warning "!!! COMPILING IN CPP MODE !!!"

  #define CPP_COMPILE_CPP_HEADER SerialPort _CPP_COMPILE_UNIQUE_NAME;
  #define Serial _CPP_COMPILE_UNIQUE_NAME

  #define Serial1 -1
  #define Serial2 -1
  #define Serial3 -1
  #define Serial4 -1
  #define Serial5 -1
  #define Serial6 -1
  #define Serial7 -1
  #define Serial8 -1
  #define Serial9 -1

  #include <iostream>
  #define USBSerial int
  #define UARTClass int

  class Dynamixel2Arduino {
  public:
    Dynamixel2Arduino(int serialPort, int dirPin);
    Dynamixel2Arduino();

    void begin(int baudrate);
    void setPortProtocolVersion(float protocolVersion);
    bool ping(unsigned jointId);
    unsigned getModelNumber(unsigned jointId);
    unsigned setOperatingMode(unsigned jointId, unsigned opMode);

    unsigned getPresentCurrent(unsigned jointId, unsigned unit);
    unsigned getPresentVelocity(unsigned jointId, unsigned unit);
    unsigned getPresentPosition(unsigned jointId, unsigned unit);

    bool setGoalCurrent(unsigned jointId, unsigned val, unsigned unit);
    bool setGoalVelocity(unsigned jointId, unsigned val, unsigned unit);
    bool setGoalPosition(unsigned jointId, unsigned val, unsigned unit);

    bool torqueOn(unsigned jointId);
    bool torqueOff(unsigned jointId);
  };

  enum ParamUnit {
    UNIT_PERCENT = 0,
    UNIT_RAW,
    UNIT_DEGREE,
    UNIT_RPM,
    UNIT_MILLI_AMPERE,
  };

  enum OperatingMode {
    OP_CURRENT = 0,
    OP_VELOCITY = 1,
    OP_POSITION = 3,
    OP_EXTENDED_POSITION = 4,
    OP_CURRENT_BASED_POSITION = 5,
    OP_PWM = 16,
  };

  class SerialPort {
  public:
    SerialPort();
    void begin(int baudrate);
    void print(unsigned value);
    void print(int value);
    void print(char value);
    void print(bool value);
    void print(const char* value);
    void print(float value);
    void print(double value);
    void print();
    void println(int value);
    void println(unsigned value);
    void println(char value);
    void println(bool value);
    void println(const char* value);
    void println(float value);
    void println(double value);
    void println();
    bool available();
    int parseInt();
    char read();

    bool operator ! ();
  };

  #define CPP_COMPILE_BEFORE_SETUP int main() {
  #define setup() * a;
  // CPP_COMPILE_BEFORE_SETUP
  // void setup() {
  //  ...
  // }
  // ------>
  // int main() {
  //  void *__setup_cpp_compile_empty;
  //  {
  //    ...
  //  }

  #define loop() * __loop_cpp_compile_empty; while(true)
  #define CPP_COMPILE_AFTER_LOOP }
  // void loop() {
  //  ...
  // }
  // CPP_COMPILE_AFTER_LOOP
  // ------>
  //  void * __loop_cpp_compile_empty;
  //  while(true) {
  //   ...
  //  }
  // }
  //

  void delay(unsigned ms);

#else
  #define CPP_COMPILE_AFTER_LOOP ;
  #define CPP_COMPILE_BEFORE_SETUP ;
  #define CPP_COMPILE_SERIAL(_name_) ;
  #define CPP_COMPILE_CPP_HEADER ;
#endif


#endif //LIBRARIES_CPPCOMPILEUTILS_H
