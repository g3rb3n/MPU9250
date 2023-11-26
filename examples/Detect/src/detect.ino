#include <Arduino.h>

#include "MPU9250.h"

#include <Point3D_Print.h>

using namespace g3rb3n;

MPU9250 mpu;

Point3D<float> a;
Point3D<float> avg;
Point3D<float> avgAdd;

long next;
long now;
uint16_t count = 0;
bool debug = false;
float threshold = .03;
float avgD = 100.;

bool sample()
{
  now = millis();
  if (now > next)
  {
    Serial.print("@ ");
    Serial.print(static_cast<float>(count), 2);
    Serial.print(" Hz");
    Serial.println();
    count = 0;
    next = now + 1000;
  }
  ++count;
  mpu.acceleration(a);

  avg *= .99;
  avgAdd = a;
  avgAdd /= 100.;
  avg += avgAdd;
  
  if(
    abs(a.x - avg.x) > threshold ||
    abs(a.y - avg.y) > threshold ||
    abs(a.z - avg.z) > threshold
  )
  {
    Serial.println("quake");
  }
  a = avg;
  if(debug)
  {
    Serial.print(millis(), 10);
    Serial.print('\t');
    Serial.print(a.x, 10);
    Serial.print('\t');
    Serial.print(a.y, 10);
    Serial.print('\t');
    Serial.print(a.z, 10);
    Serial.println();
  }
  return true;
}

void printSettings()
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

void setup() {
  Serial.begin(230400);
  Serial.println();
  Serial.println("MPU9250 quake detector");

  while(!mpu.connected())
  {
    Serial.print("Waiting for MPU9250, it responded with : ");
    Serial.print(mpu.identification());
    Serial.println();
  }
  
  mpu.reset();
  while (!mpu.isReset())
  {
    Serial.println("Wait for MPU reset");
  }

  mpu.initialize();
  mpu.wakeUp();

  mpu.setClockSource(MPU9250_CS_INTERNAL);
  mpu.setAccelometerScale(AS_2G);
  mpu.setAccelometerFrequencyMode(AF_1046);
  mpu.setSampleRateDividerMode(RATE_DIVIDER_1);

  printSettings();
  
  Serial.println("MPU9250 initialized");

  delay(1000);
}

void loop()
{
  sample();
}
