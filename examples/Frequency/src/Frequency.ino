#include <Arduino.h>

#include "MPU9250.h"
#include "MPU9250_registers.h"
#include "MPU9250_register_masks.h"

#include <Point3D_Print.h>

using namespace g3rb3n;

MPU9250 mpu;

long count;
long countUnavailable;

#define PRINT_POINT false

Point3D<int16_t> a;

#define SAMPLES 20
#if defined(ARDUINO_ARCH_AVR)
  #define SAMPLES 20
#elif defined(ARDUINO_ARCH_ESP8266)
  #define SAMPLES 200
#else
  #warning "No SAMPLES set for ARCH"
#endif

Point3D<int16_t> samples[SAMPLES];
uint16_t n_samples = SAMPLES;
uint16_t n_samples_fifo = SAMPLES;
uint16_t n_samples_reg = SAMPLES;
bool configure(bool ae, bool ge, bool te, AccelometerFrequency af, GyroscopeFrequency gf, SampleRateDivider srd)
{
  mpu.setAccelometerEnabled(ae);
  mpu.setGyroscopeEnabled(ge);
//  mpu.setThermometerEnabled(te);
  mpu.setAccelometerFrequencyMode(af);
  mpu.setGyroscopeFrequencyMode(gf);
  mpu.setSampleRateDividerMode(srd);
  return true;
}


bool sampleAcceleration()
{
  if (!mpu.available())
  {
    ++countUnavailable;
    return false;
  }
  ++count;
  mpu.acceleration(a);
  if (PRINT_POINT)
  {
    printPoint(a, 10);
    Serial.println();
  }
  return true;
}

bool sampleGyroscope()
{
  if (!mpu.available())
  {
    ++countUnavailable;
    return false;
  }
  ++count;
  mpu.gyroscope(a);
  if (PRINT_POINT)
  {
    printPoint(a, 10);
    Serial.println();
  }
  return true;
}

void debugSettings()
{
  Serial.print("AccelerationResolution:  ");
  Serial.print(mpu.accelometerScaleMode());
  Serial.print(' ');
  Serial.print(mpu.accelerationResolution());
  Serial.println();
  Serial.print("GyroscopeResolution:     ");
  Serial.print(mpu.gyroscopeScaleMode());
  Serial.print(' ');
  Serial.print(mpu.gyroscopeResolution());
  Serial.println();
  Serial.print("SampleRateDivider:       ");
  Serial.print(mpu.sampleRateDividerMode());
  Serial.print(' ');
  Serial.print(mpu.sampleRateDivider());
  Serial.println();
  Serial.print("Gyroscope IF:            ");
  Serial.print(mpu.gyroInternalFrequencyMode());
  Serial.print(' ');
  Serial.print(mpu.gyroInternalFrequency());
  Serial.println();
  Serial.print("Gyroscope FCHOICE:     ");
  Serial.print(mpu.gyroscopeFrequencyChoice());
  Serial.println();
  Serial.print("Gyroscope DLPF config  ");
  Serial.print(mpu.gyroscopeDlpfConfig());
  Serial.println();
  Serial.print("Gyroscope DOR:           ");
  Serial.print(mpu.gyroscopeDataOutputRate());
  Serial.println();
  Serial.print("GyroscopeFrequency:           ");
  Serial.print(mpu.gyroscopeFrequency());
  Serial.println();
  Serial.print("Accelometer IF:          ");
  Serial.print(mpu.accelometerInternalFrequencyMode());
  Serial.print(' ');
  Serial.print(mpu.accelometerInternalFrequency());
  Serial.println();
  Serial.print("AccelometerFrequency:          ");
  Serial.print(mpu.accelometerFrequency());
  Serial.println();
  Serial.print("Accelometer FCHOICE:     ");
  Serial.print(mpu.accelometerFrequencyChoice());
  Serial.println();
  Serial.print("Accelometer DLPF config  ");
  Serial.print(mpu.accelometerDlpfConfig());
  Serial.println();
  Serial.print("Accelometer DOR:         ");
  Serial.print(mpu.accelometerDataOutputRate());
  Serial.println();
  Serial.print("FifoMode:                ");
  Serial.print(mpu.fifoMode());
  Serial.println();
  Serial.print("ClockSource:             ");
  Serial.print(mpu.clockSource());
  Serial.println();
  Serial.print("Gyroscope SR effective:  ");
  Serial.print(mpu.isGyroscopeSampleRateDividerEffective());
  Serial.println();
  Serial.print("Accelometer SR effective:");
  Serial.print(mpu.isAccelometerSampleRateDividerEffective());
  Serial.println();
  Serial.print("GYRO_CONFIG GYRO_CT_EN        ");
  Serial.println(mpu.dumpReg(MPU9250_GYRO_CONFIG, MPU9250_MASK_GYRO_CT_EN, MPU9250_SHIFT_GYRO_CT_EN));
  Serial.print("PWR_MGMT_2 DIS_G              ");
  Serial.println(mpu.dumpReg(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_G, MPU9250_SHIFT_DISABLE_G));
  Serial.print("GYRO_CONFIG GYRO_FS_SEL       ");
  Serial.println(mpu.dumpReg(MPU9250_GYRO_CONFIG, MPU9250_MASK_GYRO_FS_SEL, MPU9250_SHIFT_GYRO_FS_SEL));
  Serial.print("GYRO_CONFIG FCHOICE_B         ");
  Serial.println(mpu.dumpReg(MPU9250_GYRO_CONFIG, MPU9250_MASK_FCHOICE_B, MPU9250_SHIFT_FCHOICE_B));
  Serial.print("CONFIG DLPF_CFG               ");
  Serial.println(mpu.dumpReg(MPU9250_CONFIG, MPU9250_MASK_DLPF_CFG, MPU9250_SHIFT_DLPF_CFG));
  Serial.print("ACCEL_CONFIG ACCEL_ST_EN      ");
  Serial.println(mpu.dumpReg(MPU9250_ACCEL_CONFIG, MPU9250_MASK_A_ST_EN, MPU9250_SHIFT_A_ST_EN));
  Serial.print("PWR_MGMT_2 DIS_A              ");
  Serial.println(mpu.dumpReg(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_A, MPU9250_SHIFT_DISABLE_A));
  Serial.print("ACCEL_CONFIG ACCEL_FS_SEL     ");
  Serial.println(mpu.dumpReg(MPU9250_ACCEL_CONFIG, MPU9250_MASK_ACCEL_FS_SEL, MPU9250_SHIFT_ACCEL_FS_SEL));
  Serial.print("ACCEL_CONFIG2 ACCEL_FCHOICE_B ");
  Serial.println(mpu.dumpReg(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_ACCEL_FCHOICE_B, MPU9250_SHIFT_ACCEL_FCHOICE_B));
//  Serial.println(mpu.dumpReg(0x1d, 0x08, 3));
  Serial.print("ACCEL_CONFIG2 A_DLPF_CFG      ");
  Serial.println(mpu.dumpReg(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_A_DLPF_CFG, MPU9250_SHIFT_A_DLPF_CFG));
//  Serial.println(mpu.dumpReg(0x1d, 0x07, 0));
  Serial.print("SMPLRT_DIV                    ");
  Serial.println(mpu.dumpReg(MPU9250_SMPLRT_DIV, 0xff, 0));
  Serial.println();
}

void debugSampleAcceleration()
{
  count = 0;
  countUnavailable = 0;

  long start = micros();
  for (int i = 0 ; i < n_samples_reg ; )
  {
    if (sampleAcceleration())
      ++i;
  }
  long duration = micros() - start;
  Serial.print("Sampled at ");
  Serial.print(static_cast<float>(count) / duration * 1000000);
  Serial.print(" Hz");
  Serial.println();
  Serial.print("Unavailable ");
  Serial.print(countUnavailable);
  Serial.print(" tries");
  Serial.println();
}

void debugSampleGyroscope()
{
  count = 0;
  countUnavailable = 0;
  long start = micros();
  for (int i = 0 ; i < n_samples_reg ; )
  {
    if (sampleGyroscope())
      ++i;
  }
  long duration = micros() - start;
  Serial.print("Sampled at ");
  Serial.print(static_cast<float>(count) / duration * 1000000);
  Serial.print(" Hz");
  Serial.println();
  Serial.print("Unavailable ");
  Serial.print(countUnavailable);
  Serial.print(" tries");
  Serial.println();

}


void debugSampleAccelerationFifo()
{
  count = 0;
  long start = micros();
  uint8_t got = mpu.acceleration(samples, n_samples);
  long duration = micros() - start;
  if (PRINT_POINT)
  {
    for (uint8_t i = 0 ; i < got ; ++i)
    {
      printPoint(samples[i], 10);
      Serial.println();
    }
  }
  Serial.print("Sampled at ");
  Serial.print(static_cast<float>(got) / duration * 1000000);
  Serial.print(" Hz");
  Serial.println();
}

void debugSampleGyroscopeFifo()
{
  count = 0;
  long start = micros();
  uint8_t got = mpu.gyroscope(samples, n_samples);
  long duration = micros() - start;
  if (PRINT_POINT)
  {
    for (uint8_t i = 0 ; i < got ; ++i)
    {
      printPoint(samples[i], 10);
      Serial.println();
    }
  }
  Serial.print("Sampled at ");
  Serial.print(static_cast<float>(got) / duration * 1000000);
  Serial.print(" Hz");
  Serial.println();
}

void debugSampleGyroscopeFifoTest()
{
  uint8_t got = mpu.testGyroscopeFifo(n_samples_fifo);
}

void debugSampleAccelerationFifoTest()
{
  uint8_t got = mpu.testAccelometerFifo(n_samples_fifo);
}


void debugAcceleration(String s)
{
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  Serial.println(s);
  debugSettings();
  debugSampleAcceleration();
  debugSampleAccelerationFifo();
  debugSampleAccelerationFifoTest();
  Serial.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  Serial.println();
}

void debugGyroscope(String s)
{
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  Serial.println(s);
  Serial.print("Gyroscope only @ ");
  Serial.print(mpu.gyroscopeFrequency());
  Serial.print(" / ");
  Serial.print(mpu.sampleRateDivider());
  Serial.print(" reg 1a DLPF");
  Serial.print(mpu.dumpReg(0x1a, 0x07, 0));
  Serial.print(" 1b FCHOICE");
  Serial.print(mpu.dumpReg(0x1b, 0x03, 0));
  Serial.println();
  debugSettings();
  debugSampleGyroscope();
  debugSampleGyroscopeFifo();
  debugSampleGyroscopeFifoTest();
  Serial.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  Serial.println();
}

void iterateAccelometerFchoiceAndDlpf()
{
  configure(true, false, false, AF_1046, GF_8800, RATE_DIVIDER_4);
  for (uint8_t i = 0 ; i <= 0x0f; ++i)
  {
    mpu.setReg(0x1d, 0x0f, 0, i);
    Serial.print("Accelometer only @ ");
    Serial.print(mpu.accelometerFrequency());
    Serial.print(" / ");
    Serial.print(mpu.sampleRateDivider());
    Serial.print(" reg ");
    Serial.print(mpu.dumpReg(0x1d, 0xff, 0));
    Serial.println();
    debugAcceleration("");
  }
}

void iterateGyroscopeFchoiceAndDlpf()
{
  configure(false, true, false, AF_1046, GF_8800, RATE_DIVIDER_4);
  for (uint8_t d = 0 ; d <= 0x07; ++d)
  {
      uint8_t f = 0x00;
//    for (uint8_t f = 0 ; f <= 0x03; ++f)
//    {
      mpu.setReg(0x1a, 0x07, 0, d);
      mpu.setReg(0x1b, 0x03, 0, f);
      debugGyroscope("");
//    }
  }
}

void iterateGyroscopeVarious()
{
  configure(false, true, false, AF_1046, GF_8800, RATE_DIVIDER_0);
  debugGyroscope("");
  configure(false, true, false, AF_1046, GF_3600_NO_DLPF, RATE_DIVIDER_0);
  debugGyroscope("");
  configure(false, true, false, AF_1046, GF_3600, RATE_DIVIDER_0);
  debugGyroscope("");  
  configure(false, true, false, AF_5_05, GF_8800, RATE_DIVIDER_0);
  debugGyroscope("Gyroscope only at 8800 Hz / 1");
  configure(false, true, false, AF_5_05, GF_8800, RATE_DIVIDER_7);
  debugGyroscope("Gyroscope only at 8800 Hz / 8");
  configure(false, true, false, AF_5_05, GF_3600, RATE_DIVIDER_7);
  debugGyroscope("Gyroscope only at 3600 Hz / 1");
  configure(false, true, false, AF_5_05, GF_250, RATE_DIVIDER_7);
  debugGyroscope("Gyroscope only at 250 Hz / 1");
  configure(false, true, false, AF_5_05, GF_184, RATE_DIVIDER_7);
  debugGyroscope("Gyroscope only at 184 Hz / 1");
  configure(false, true, false, AF_5_05, GF_92, RATE_DIVIDER_7);
  debugGyroscope("Gyroscope only at 92 Hz / 1");  
}

void iterateAccelometerVarious()
{
  configure(true, false, false, AF_1046, GF_8800, RATE_DIVIDER_0);
  debugAcceleration("Accelometer only at 1046 Hz / 1");
  configure(true, false, false, AF_1046, GF_8800, RATE_DIVIDER_1);
  debugAcceleration("Accelometer only at 1046 Hz / 2");
  configure(true, false, false, AF_1046, GF_8800, RATE_DIVIDER_7);
  debugAcceleration("Accelometer only at 1046 Hz / 8");
  configure(true, false, false, AF_281_1_NO_DLPF, GF_8800, RATE_DIVIDER_7);
  debugAcceleration("Accelometer only at 281.1 Hz / 1");
  configure(true, false, false, AF_281_1, GF_8800, RATE_DIVIDER_7);
  debugAcceleration("Accelometer only at 281.1 Hz / 1");
  configure(true, false, false, AF_99, GF_8800, RATE_DIVIDER_7);
  debugAcceleration("Accelometer only at 99 Hz / 1");
  configure(true, false, false, AF_44_8, GF_8800, RATE_DIVIDER_7);
  debugAcceleration("Accelometer only at 44.8 Hz / 1");
  configure(true, false, false, AF_21_2, GF_8800, RATE_DIVIDER_7);
  debugAcceleration("Accelometer only at 21.2 Hz / 1");
  configure(true, false, false, AF_10_2, GF_8800, RATE_DIVIDER_7);
  debugAcceleration("Accelometer only at 10.2 Hz / 1");
  configure(true, false, false, AF_5_05, GF_8800, RATE_DIVIDER_7);
  debugAcceleration("Accelometer only at 5.05 Hz / 1");  
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
    delay(500);
  }
  
  mpu.setClockSource(MPU9250_CS_INTERNAL);
  mpu.setGyroscopeScale(GS_250DPS);
  mpu.setAccelometerScale(AS_16G);
  mpu.enableI2CBypass();
  mpu.enableRawDataReadyInterrupt(true);
  Serial.println("MPU9250 initialized");

  delay(1000);

  //iterateAccelometerFchoiceAndDlpf();
  //iterateGyroscopeFchoiceAndDlpf();
  //iterateAccelometerVarious();
  //iterateGyroscopeVarious();

  configure(true, false, false, AF_281_1_NO_DLPF, GF_3600_NO_DLPF, RATE_DIVIDER_0);
  debugAcceleration("");
  configure(false, true, false, AF_281_1_NO_DLPF, GF_3600_NO_DLPF, RATE_DIVIDER_0);
  debugGyroscope("");

  configure(true, false, false, AF_1046, GF_8800, RATE_DIVIDER_0);
  debugAcceleration("");

  configure(true, false, false, AF_1046, GF_8800, RATE_DIVIDER_0);
  debugGyroscope("");
  debugGyroscope("");



  configure(true, false, false, AF_281_1, GF_184, RATE_DIVIDER_7);
  debugAcceleration("");

  configure(false, true, false, AF_281_1, GF_184, RATE_DIVIDER_7);
  debugGyroscope("");

  configure(true, false, false, AF_99, GF_92, RATE_DIVIDER_7);
  debugAcceleration("");

  configure(false, true, false, AF_99, GF_92, RATE_DIVIDER_7);
  debugGyroscope("");

  configure(true, false, false, AF_44_8, GF_41, RATE_DIVIDER_7);
  debugAcceleration("");

  configure(false, true, false, AF_44_8, GF_41, RATE_DIVIDER_7);
  debugGyroscope("");

}

void loop()
{
}
