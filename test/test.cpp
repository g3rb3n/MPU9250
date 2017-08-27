#include <Arduino.h>
#include <unity.h>

#include "../src/MPU9250.h"

#ifdef UNIT_TEST

using namespace g3rb3n;

MPU9250 mpu;

void isConnected(void) {
  TEST_ASSERT_EQUAL(true, mpu.connected());
}

void accelEnabled(void) {
  TEST_ASSERT_EQUAL(true, mpu.accelXEnabled() && mpu.accelYEnabled() && mpu.accelZEnabled());
}

void gyroEnabled(void) {
  TEST_ASSERT_EQUAL(true, mpu.gyroXEnabled() && mpu.gyroYEnabled() && mpu.gyroZEnabled());
}

void accelometerScale(void) {
  float res, exp;

  mpu.setAccelometerScale(AS_2G);
  res = mpu.accelometerScaleMode();
  exp = AS_2G;
  TEST_ASSERT_EQUAL(exp, res);

  mpu.setAccelometerScale(AS_4G);
  res = mpu.accelometerScaleMode();
  exp = AS_4G;
  TEST_ASSERT_EQUAL(exp, res);

  mpu.setAccelometerScale(AS_8G);
  res = mpu.accelometerScaleMode();
  exp = AS_8G;
  TEST_ASSERT_EQUAL(exp, res);

  mpu.setAccelometerScale(AS_16G);
  res = mpu.accelometerScaleMode();
  exp = AS_16G;
  TEST_ASSERT_EQUAL(exp, res);
}

void gyroscopeScale(void) {
  GyroscopeScale res, exp;

  mpu.setGyroscopeScale(GS_250DPS);
  res = mpu.gyroscopeScaleMode();
  exp = GS_250DPS;
  TEST_ASSERT_EQUAL(exp, res);

  mpu.setGyroscopeScale(GS_500DPS);
  res = mpu.gyroscopeScaleMode();
  exp = GS_500DPS;
  TEST_ASSERT_EQUAL(exp, res);

  mpu.setGyroscopeScale(GS_1000DPS);
  res = mpu.gyroscopeScaleMode();
  exp = GS_1000DPS;
  TEST_ASSERT_EQUAL(exp, res);

  mpu.setGyroscopeScale(GS_2000DPS);
  res = mpu.gyroscopeScaleMode();
  exp = GS_2000DPS;
  TEST_ASSERT_EQUAL(exp, res);
}

void measuresGrafity(void) {
  Point3D<float> a;
  mpu.acceleration(a);
  float squareLength = a.x*a.x + a.y*a.y + a.z*a.z;
  TEST_ASSERT_FLOAT_WITHIN(0.1, 1.0, squareLength);
}

void setup() {
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(isConnected);
  
  mpu.initialize();
  
  RUN_TEST(accelEnabled);
  RUN_TEST(gyroEnabled);
  RUN_TEST(accelometerScale);
  RUN_TEST(gyroscopeScale);
}

void loop() {
  uint8_t count = 10;
  for (uint8_t i = 0 ; i < 10 ; ++i)
  {
    while (!mpu.available()){}
    RUN_TEST(measuresGrafity);
  }
  UNITY_END();
}

#endif
