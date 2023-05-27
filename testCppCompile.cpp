#define _CPP_COMPILE_UNIQUE_NAME _serial_test_cpp_compile
#include "./CppCompileUtils/CppCompileUtils.h"
CPP_COMPILE_CPP_HEADER


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
