#include "./DynamixelManipulatorUtils/CppCompileUtils.h"
SerialPort _serial_test_cpp_compile;
#define Serial _serial_test_cpp_compile

void setup() {
  bool a = Serial.available();
  unsigned time = 0;

  CPP_COMPILE_END_SETUP
}

void loop() {
  Serial.println(time);
  time += 100;
  delay(100);

  CPP_COMPILE_END_LOOP
}
