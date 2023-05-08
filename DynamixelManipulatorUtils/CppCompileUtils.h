//
// Created by Сергей Тяпкин1 on 04.05.2023.
//

/**
 (!!!) You must write after include this file in each .cpp endpoint file:
 SerialPort <_some_unique_name>;
 #define Serial <_some_unique_name>
**/

#include <stdio.h>
#ifdef _WIN32
  #include <windows.h>
#else
  #include <unistd.h>
#endif

#ifndef LIBRARIES_CPPCOMPILEUTILS_H
#define LIBRARIES_CPPCOMPILEUTILS_H


#define _CPP_COMPILE_MODE // in your code to enable it
#ifdef _CPP_COMPILE_MODE

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

namespace ControlTableItem {
  typedef int ParamUnit;
  const unsigned OP_POSITION = 1;
  const unsigned UNIT_PERCENT = 2;
  const unsigned UNIT_RAW = 3;
  const unsigned UNIT_DEGREE = 4;
  const unsigned UNIT_RPM = 5;
  const unsigned UNIT_MA = 6;
}

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

#define setup() __setup_cpp_compile_empty(){} int main()
#define CPP_COMPILE_END_SETUP {
// void setup() {
//  ...
//  CPP_COMPILE_END_SETUP
// }
// ------>
// void __setup_cpp_compile_empty(){}
// int main() {
//    ...
//  {}

#define loop() * __loop_cpp_compile_empty; while(true)
#define CPP_COMPILE_END_LOOP }
// void loop() {
//  ...
//  CPP_COMPILE_END_LOOP
// }
// ------>
//  void * __loop_cpp_compile_empty;
//  while(true) {
//   ...
//  }
// }
//


void delay(unsigned ms);

#else
#define CPP_COMPILE_END_SETUP
#define CPP_COMPILE_END_LOOP
#endif


#endif //LIBRARIES_CPPCOMPILEUTILS_H
