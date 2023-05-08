//
// Created by Сергей Тяпкин1 on 04.05.2023.
//

#include "CppCompileUtils.h"

#define _CPP_COMPILE_MODE
#ifdef _CPP_COMPILE_MODE

unsigned DEFAULT_JOINTS_POS[] = {135, 180, 155, 164, 180, 170}; // TODO

// --- Dynamixel2Arduino
Dynamixel2Arduino::Dynamixel2Arduino(int serialPort, int dirPin) {;}
Dynamixel2Arduino::Dynamixel2Arduino() {;}

void Dynamixel2Arduino::begin(int baudrate) {
  std::cout << "[DXL] INIT BAUDRATE:" << baudrate << std::endl;
}
void Dynamixel2Arduino::setPortProtocolVersion(float protocolVersion) {
  std::cout << "[DXL] INIT PROTOCOL VERSION:" << protocolVersion << std::endl;
}
bool Dynamixel2Arduino::ping(unsigned jointId) {
  return true;
}
unsigned Dynamixel2Arduino::getModelNumber(unsigned jointId) {
  return 1234;
}
unsigned Dynamixel2Arduino::setOperatingMode(unsigned jointId, unsigned opMode) {
  return 1234;
}

unsigned Dynamixel2Arduino::getPresentCurrent(unsigned jointId, unsigned unit) {
  return 0;
}
unsigned Dynamixel2Arduino::getPresentVelocity(unsigned jointId, unsigned unit) {
  return 0;
}
unsigned Dynamixel2Arduino::getPresentPosition(unsigned jointId, unsigned unit) {
  return DEFAULT_JOINTS_POS[jointId - 1];
}

bool Dynamixel2Arduino::setGoalCurrent(unsigned jointId, unsigned val, unsigned unit) {return true;}
bool Dynamixel2Arduino::setGoalVelocity(unsigned jointId, unsigned val, unsigned unit) {return true;}
bool Dynamixel2Arduino::setGoalPosition(unsigned jointId, unsigned val, unsigned unit) {return true;}

bool Dynamixel2Arduino::torqueOn(unsigned jointId) {return true;}
bool Dynamixel2Arduino::torqueOff(unsigned jointId) {return true;}

// --- SerialPort
SerialPort::SerialPort() { // Make singleton
  //TODO: singleton
}
void SerialPort::begin(int baudrate) {
  std::cout << "[SERIAL] INIT BAUDRATE:" << baudrate << std::endl;
}
void SerialPort::print(unsigned value) {
  std::cout << value;
}
void SerialPort::print(int value) {
  std::cout << value;
}
void SerialPort::print(char value) {
  std::cout << value;
}
void SerialPort::print(bool value) {
  std::cout << value;
}
void SerialPort::print(const char* value) {
  std::cout << value;
}
void SerialPort::print(float value) {
  std::cout << value;
}
void SerialPort::print(double value) {
  std::cout << value;
}
void SerialPort::print() {
}
void SerialPort::println(int value) {
  std::cout << value << std::endl;
}
void SerialPort::println(unsigned value) {
  std::cout << value << std::endl;
}
void SerialPort::println(char value) {
  std::cout << value << std::endl;
}
void SerialPort::println(bool value) {
  std::cout << value << std::endl;
}
void SerialPort::println(const char* value) {
  std::cout << value << std::endl;
}
void SerialPort::println(float value) {
  std::cout << value << std::endl;
}
void SerialPort::println(double value) {
  std::cout << value << std::endl;
}
void SerialPort::println() {
  std::cout << std::endl;
}
bool SerialPort::operator ! () {
  return false;
}
bool SerialPort::available() {
  return true;
}
int SerialPort::parseInt() {
  return 55;
}
char SerialPort::read() {
  return getchar();
}

void delay(unsigned ms) {
  #ifdef _WIN32
    Sleep(ms);
  #else
    usleep(ms * 1000);
  #endif
}


#endif
