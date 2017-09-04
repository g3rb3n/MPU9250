#include <Arduino.h>

#include <I2C.h>
#include <Point3D.h>
#include <Point3D_Print.h>

#include "MPU9250.h"


using namespace g3rb3n;

MPU9250 mpu(0x68, D2, D1);

bool sample()
{
  if (!mpu.available())
    return false;
    
  //Point3D<int16_t> a;
  Point3D<float> a;
  mpu.acceleration(a);
  float squareLength = a.x*a.x + a.y*a.y + a.z*a.z;
  //a *= 1000;
  Serial.print(squareLength, 10);
  Serial.print(' ');
  Serial.print('(');
  Serial.print(a.x, 10);
  Serial.print(' ');
  Serial.print(a.y, 10);
  Serial.print(' ');
  Serial.print(a.z, 10);
  Serial.print(')');
  Serial.println();
  // + (String)a.x + "," + (String)a.y + "," + (String)a.z + ") " + (String)squareLength);
  //Serial.printf("%5.4f,%5.4f,%5.4f %5.4f\n", a.x, a.y, a.z, squareLength);
  //a *= 1000;
  //Serial.printf("%10i %10i %10i %10i\n", millis(), a.x, a.y, a.z);
  //Serial.printf("%f,%f,%f %f\n", a.x, a.y, a.z, squareLength);
  return true;
}

void debugSettings()
{
  Serial.print("AccelerationResolution: ");
  Serial.print(mpu.accelometerScaleMode());
  Serial.print(' ');
  Serial.println(mpu.accelerationResolution());
  Serial.print("GyroscopeResolution:    ");
  Serial.print(mpu.gyroscopeScaleMode());
  Serial.print(' ');
  Serial.println(mpu.gyroscopeResolution());
  Serial.print("SampleRateDivider:      ");
  Serial.print(mpu.sampleRateDividerMode());
  Serial.print(' ');
  Serial.println(mpu.sampleRateDivider());
  Serial.print("GyroFrequency:          ");
  Serial.println(mpu.gyroscopeFrequency());
  Serial.print("AccelFrequency:         ");
  Serial.println(mpu.accelometerFrequency());
  Serial.print("FifoMode:               ");
  Serial.println(mpu.fifoMode());
  Serial.print("ClockSource:            ");
  Serial.println(mpu.clockSource());
  Serial.println();
}

void debugSample()
{
  for (int i = 0 ; i < 10 ; )
  {
    if (sample())
      ++i;
  }
}

void debugSampleFifo()
{
  Point3D<float> samples[10];
  uint8_t got = mpu.acceleration(samples, 10);
  for (int i = 0 ; i < 10 && i < got ; ++i)
  {
    printPoint(samples[i], 10);
    Serial.println();
  }
}

void debugAccel()
{
  Point3D<int16_t> offsetS16;
  Point3D<uint16_t> offsetU16;
  Point3D<float> offsetF;

  mpu.dumpMem();
  
  mpu.readAccelometerOffsetRaw(offsetU16);
  Serial.print("U16 ");
  printPoint(offsetU16, 10);
  Serial.println();

  mpu.accelometerOffset(offsetS16);
  Serial.print("S16 ");
  printPoint(offsetS16, 10);
  Serial.println();

  mpu.accelometerOffset(offsetF);
  Serial.print("F   ");
  printPoint(offsetF, 10);
  Serial.println();
}


void debug(String s)
{
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  Serial.println(s);
  debugAccel();
  debugSettings();
  debugSample();
  debugSampleFifo();
  Serial.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  Serial.println();
}

void setup() {
  Serial.begin(230400);
  Serial.println();
  Serial.println("MPU9250 online");

  while(!mpu.connected())
  {
    Serial.print("Waiting for MPU9250 ");
    Serial.print("MPU9250 responded with : ");
    Serial.print(mpu.identification());
    Serial.println();
  }
  
  mpu.reset();
  while (!mpu.isReset())
  {
    Serial.println("Wait for MPU reset");
  }

  debug("Initial");
  //mpu.initialize();
  mpu.wakeUp();

  //Point3D<float> a;
  //Point3D<float> g;
  //mpu.selfTest(a,g);
  //mpu.printSelfTest(a,g);
  
  //mpu.setGyroscopeOffset(gyroscopeOffset);
  Point3D<int32_t> accelometerOffset;

  mpu.determineAccelometerOffset(accelometerOffset);
  Serial.print("Determined offset");
  printPoint(accelometerOffset, 10);
  Serial.println();

  debug("Accelometer Offset initial");

  Point3D<int16_t> p0(0,0,0);
  mpu.setAccelometerOffset(p0);
  debug("Accelometer Offset after p0");

  mpu.determineAccelometerOffset(accelometerOffset);
  Serial.print("Setting offset");
  printPoint(accelometerOffset, 10);
  Serial.println();
  accelometerOffset.x = -accelometerOffset.x;
  accelometerOffset.y = -accelometerOffset.y;
  accelometerOffset.z = -accelometerOffset.z;
  accelometerOffset /= 2;
  accelometerOffset.z += 1024;
  mpu.setAccelometerOffset(accelometerOffset);
  debug("Accelometer Offset after calibration");

  mpu.setClockSource(MPU9250_CS_AUTOMATIC);
  mpu.setAccelometerRate(AR_41);
  mpu.setSampleRateDividerMode(RATE_DIVIDER_1);
  mpu.setGyroscopeScale(GS_250DPS);
  mpu.setAccelometerScale(AS_2G);
  mpu.enableI2CBypass();
  mpu.enableRawDataReadyInterrupt(true);

  debug("Final");
  Serial.println("MPU9250 initialized");

  delay(1000);
}

void loop()
{
  sample();
}

/*
  Point3D<int32_t> accelometerOffset(0,0,0);
  mpu.determineAccelometerOffset(accelometerOffset);
  Serial.print("Accelometer Offset ");
  printPoint(accelometerOffset, 10);
  Serial.println();

  Point3D<float> accelometerOffsetF(0,0,0);
  mpu.determineAccelometerOffset(accelometerOffsetF);
  Serial.print("Accelometer Offset ");
  printPoint(accelometerOffsetF, 10);
  Serial.println();

  Point3D<int32_t> gyroscopeOffset(0,0,0);
  mpu.determineGyroscopeOffset(gyroscopeOffset);
  Serial.print("Gyroscope Offset ");
  printPoint(gyroscopeOffset, 10);
  Serial.println();

  Point3D<float> gyroscopeOffsetF(0,0,0);
  mpu.determineGyroscopeOffset(gyroscopeOffsetF);
  Serial.print("Gyroscope Offset ");
  printPoint(gyroscopeOffsetF, 10);
  Serial.println();
  mpu.calibrate();
*/
