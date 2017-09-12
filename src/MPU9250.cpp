
#include <Arduino.h>

#include "MPU9250.h"
#include "MPU9250_registers.h"
#include "MPU9250_register_masks.h"
#include "MPU9250_register_values.h"

#include <Point3D.h>
#include <Point3D_Math.h>
#include <Point3D_Print.h>

#include <I2C.h>

#define MPU9250_ADDRESS   0x68  // Device address when ADO = 0
//#define MPU9250_ADDRESS   0x69  // Device address when ADO = 1

namespace g3rb3n
{

MPU9250::MPU9250():
  i2c(MPU9250_ADDRESS)
{
}

MPU9250::MPU9250(uint8_t address):
  i2c(address)
{
}

MPU9250::MPU9250(uint8_t address, uint8_t sda, uint8_t scl):
  i2c(address, sda, scl)
{
}


MPU9250::~MPU9250()
{}

uint8_t MPU9250::address() const
{
  return i2c.address();
}

void MPU9250::reset()
{
  i2c.writeMaskClearSet(MPU9250_PWR_MGMT_1, MPU9250_MASK_H_RESET);
}

bool MPU9250::isReset() const
{
  return !i2c.readMaskBit(MPU9250_PWR_MGMT_1, MPU9250_MASK_H_RESET);  
}

void MPU9250::wakeOnMotion(uint8_t threshold)
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_1, MPU9250_MASK_CYCLE);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_1, MPU9250_MASK_SLEEP);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_1, MPU9250_MASK_GYRO_STANDBY);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_XG);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_YG);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_ZG);

  i2c.writeMaskShiftValue(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_ACCEL_FCHOICE_B, MPU9250_SHIFT_ACCEL_FCHOICE_B, 1);
  i2c.writeMaskShiftValue(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_A_DLPF_CFG, MPU9250_SHIFT_A_DLPF_CFG, 1);

  i2c.writeMaskClearSet(MPU9250_INT_ENABLE, MPU9250_MASK_WOM_EN);
  
  i2c.writeMaskSet(MPU9250_MOT_DETECT_CTRL, MPU9250_MASK_ACCEL_INTEL_EN);
  i2c.writeMaskSet(MPU9250_MOT_DETECT_CTRL, MPU9250_MASK_ACCEL_INTEL_MODE);

  i2c.writeMaskSet(MPU9250_WOM_THR, threshold);
  
  i2c.writeMaskShiftValue(MPU9250_LP_ACCEL_ODR, MPU9250_MASK_LPOSC_CLKSEL, 0, 1);
  
  i2c.writeMaskSet(MPU9250_PWR_MGMT_1, MPU9250_MASK_CYCLE);

}

void MPU9250::setAccelOnlyLowPowerMode()
{
  i2c.writeMaskClearSet(MPU9250_PWR_MGMT_1, MPU9250_MASK_CYCLE);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_1, MPU9250_MASK_SLEEP);
  //Step 3 from procedure defined in 4.35
  //TEMP_DIS register bit is not documented
  //writeMaskClearSet(MPU9250_PWR_MGMT_1, MPU9250_MASK_TEMP_DIS);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_XG);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_YG);
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_ZG);
}


void MPU9250::setLowPassRate(LowPassRate lpr)
{
  i2c.writeMaskShiftValue(MPU9250_MASK_DLPF_CFG, 0, 0, lpr);
}

GyroscopeInternalFrequency MPU9250::gyroInternalFrequencyMode() const
{
  uint8_t fchoice = i2c.readMaskShift(MPU9250_GYRO_CONFIG, MPU9250_MASK_FCHOICE_B, 0);
  uint8_t dlpf = i2c.readMaskShift(MPU9250_CONFIG, MPU9250_MASK_DLPF_CFG, 0);
  if (fchoice % 2 == 0x01)
    return GIF_32K;
  if (fchoice == 0x02)
    return GIF_32K;
  if (dlpf == 0x00 || dlpf == 0x07)
    return GIF_8K;
  return GIF_1K;
}

float MPU9250::gyroInternalFrequency() const
{
  GyroscopeInternalFrequency mode = gyroInternalFrequencyMode();
  switch(mode)
  {
    case GIF_32K: return 32000;
    case GIF_8K: return 8000;
    case GIF_1K: return 1000;
  }
}


void MPU9250::setGyroscopeFrequencyMode(GyroscopeFrequency mode)
{
  uint8_t fchoice_b;
  uint8_t dlpf;
  if (mode == GF_8800)
    fchoice_b = 0x01;
  else if (mode == GF_3600_NO_DLPF)
    fchoice_b = 0x02;
  else
    fchoice_b = 0x00;

  switch(mode)
  {
    case GF_250:  dlpf = 0; break;
    case GF_184:  dlpf = 1; break;
    case GF_92:   dlpf = 2; break;
    case GF_41:   dlpf = 3; break;
    case GF_20:   dlpf = 4; break;
    case GF_10:   dlpf = 5; break;
    case GF_5:    dlpf = 6; break;
    case GF_3600: dlpf = 7; break;
    case GF_3600_NO_DLPF:dlpf = 0; break;
    case GF_8800: dlpf = 0; break;
  }

  i2c.writeMaskShiftValue(MPU9250_GYRO_CONFIG, MPU9250_MASK_FCHOICE_B, MPU9250_SHIFT_FCHOICE_B, fchoice_b);
  i2c.writeMaskShiftValue(MPU9250_CONFIG, MPU9250_MASK_DLPF_CFG, MPU9250_SHIFT_DLPF_CFG, dlpf);
}


GyroscopeFrequency MPU9250::gyroscopeFrequencyMode() const
{
  uint8_t fchoice_b = i2c.readMaskShift(MPU9250_GYRO_CONFIG, MPU9250_MASK_FCHOICE_B, MPU9250_SHIFT_FCHOICE_B);
  uint8_t dlpf = i2c.readMaskShift(MPU9250_CONFIG, MPU9250_MASK_DLPF_CFG, MPU9250_SHIFT_DLPF_CFG);
  
//  if (fchoice_b % 2 == 0x01)
  if (fchoice_b == 0x01 || fchoice_b == 0x03)
    return GF_8800;
  if (fchoice_b == 0x02)
    return GF_3600_NO_DLPF;
    
  switch(dlpf)
  {
    case 0: return GF_250;
    case 1: return GF_184;
    case 2: return GF_92;
    case 3: return GF_41;
    case 4: return GF_20;
    case 5: return GF_10;
    case 6: return GF_5;
    case 7: return GF_3600;
  }
  return GF_3600;
}

uint16_t MPU9250::gyroscopeFrequency() const
{
  GyroscopeFrequency f = gyroscopeFrequencyMode();
  switch(f)
  {
    case GF_250:  return 250;
    case GF_184:  return 184;
    case GF_92:   return 92;
    case GF_41:   return 41;
    case GF_20:   return 20;
    case GF_10:   return 10;
    case GF_5:    return 5;
    case GF_3600: return 3600;
    case GF_8800: return 8800;
    default: return 0;
  }
}


void MPU9250::setAccelometerFrequencyMode(AccelometerFrequency mode)
{
  uint8_t fchoice_b;
  uint8_t dlpf;
  if (mode == AF_1046)
    fchoice_b = 0x01;
//    fchoice_b = 0x00;
  else
    fchoice_b = 0x00;
//    fchoice_b = 0x01;

  switch(mode)
  {
    case AF_281_1_NO_DLPF: dlpf = 0; break;
    case AF_281_1:   dlpf = 1; break;
    case AF_99:      dlpf = 2; break;
    case AF_44_8:    dlpf = 3; break;
    case AF_21_2:    dlpf = 4; break;
    case AF_10_2:    dlpf = 5; break;
    case AF_5_05:    dlpf = 6; break;
    case AF_420:     dlpf = 7; break;
    default:         dlpf = 0; break;
//    case AF_460: dlpf = 7; break;
  }
  i2c.writeMaskShiftValue(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_ACCEL_FCHOICE_B, MPU9250_SHIFT_ACCEL_FCHOICE_B, fchoice_b);
  i2c.writeMaskShiftValue(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_A_DLPF_CFG, MPU9250_SHIFT_A_DLPF_CFG, dlpf);
}

float MPU9250::accelometerInternalFrequency() const
{
  AccelometerInternalFrequency af = accelometerInternalFrequencyMode();
  if (af == AIF_4K)
    return 4000;
  return 1000;
}

AccelometerInternalFrequency MPU9250::accelometerInternalFrequencyMode() const
{
  AccelometerFrequency af = accelometerFrequencyMode();
  if (af == AF_1046)
    return AIF_4K;
  return AIF_1K;
}

AccelometerFrequency MPU9250::accelometerFrequencyMode() const
{
  uint8_t fchoice_b = i2c.readMaskShift(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_ACCEL_FCHOICE_B, 0);
  uint8_t dlpf = i2c.readMaskShift(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_A_DLPF_CFG, 0);
  if (fchoice_b == 0x01)
//  if (fchoice_b == 0x00)
    return AF_1046;
  switch(dlpf)
  {
    case 0: return AF_281_1_NO_DLPF;
    case 1: return AF_281_1;
    case 2: return AF_99;
    case 3: return AF_44_8;
    case 4: return AF_21_2;
    case 5: return AF_10_2;
    case 6: return AF_5_05;
    case 7: return AF_420;
    default:return AF_1046;
  }
}

uint16_t MPU9250::accelometerFrequency() const
{
  AccelometerFrequency f = accelometerFrequencyMode();
  switch(f)
  { 
    case AF_281_1_NO_DLPF: return 281;
    case AF_281_1:   return 281;
    case AF_99:      return 99;
    case AF_44_8:    return 45;
    case AF_21_2:    return 21;
    case AF_10_2:    return 10;
    case AF_5_05:    return 5;
    case AF_420:     return 420;
    case AF_1046:    return 1046;
    default:         return 0;
  }
}

SampleRateDivider MPU9250::sampleRateDividerMode() const
{
  uint8_t r = i2c.readByte(MPU9250_SMPLRT_DIV);
  return (SampleRateDivider)r;
}

float MPU9250::accelometerFrequencyDivided() const
{
  float freq = accelometerFrequency();
  float div = sampleRateDividerMode();
  return freq / (1. + div);
}

void MPU9250::setFifoMode(FifoMode mode)
{
  i2c.writeMaskShiftValue(MPU9250_CONFIG, MPU9250_MASK_FIFO_MODE, MPU9250_SHIFT_FIFO_MODE, mode);
}

FifoMode MPU9250::fifoMode() const
{
  return (FifoMode)i2c.readMaskShift(MPU9250_CONFIG, MPU9250_MASK_FIFO_MODE, MPU9250_SHIFT_FIFO_MODE);
}

void MPU9250::enableFifo()
{
  i2c.writeMaskSet(MPU9250_USER_CTRL, MPU9250_MASK_FIFO_EN);
}

void MPU9250::disableFifo()
{
  i2c.writeMaskClear(MPU9250_USER_CTRL, MPU9250_MASK_FIFO_EN);
}

bool MPU9250::fifoEnabled() const
{
  return i2c.readMaskBit(MPU9250_USER_CTRL, MPU9250_MASK_FIFO_EN);
}

void MPU9250::enableGyroscopeToFifo()
{
  i2c.writeMaskSet(MPU9250_FIFO_EN, MPU9250_MASK_GYRO_OUT);
}

void MPU9250::disableGyroscopeToFifo()
{
  i2c.writeMaskClear(MPU9250_FIFO_EN, MPU9250_MASK_GYRO_OUT);
}

void MPU9250::enableAccelometerToFifo()
{
  i2c.writeMaskSet(MPU9250_FIFO_EN, MPU9250_MASK_ACCEL_OUT);
}

void MPU9250::disableAccelometerToFifo()
{
  i2c.writeMaskClear(MPU9250_FIFO_EN, MPU9250_MASK_ACCEL_OUT);
}

void MPU9250::setFsync(ExtSyncSet mode)
{
  i2c.writeMaskShiftValue(MPU9250_CONFIG, MPU9250_MASK_EXT_SYNC_SET, MPU9250_SHIFT_EXT_SYNC_SET, mode);
}

void MPU9250::setAccelometerScale(AccelometerScale scale)
{
  _accelometerScale = scale;
  i2c.writeMaskShiftValue(MPU9250_ACCEL_CONFIG, MPU9250_MASK_ACCEL_FS_SEL, MPU9250_SHIFT_ACCEL_FS_SEL, scale);
}

AccelometerScale MPU9250::accelometerScaleMode() const
{
  return (AccelometerScale)i2c.readMaskShift(MPU9250_ACCEL_CONFIG, MPU9250_MASK_ACCEL_FS_SEL, MPU9250_SHIFT_ACCEL_FS_SEL);
//  return _accelometerScale;
}

void MPU9250::setGyroscopeScale(GyroscopeScale scale)
{
  _gyroscopeScale = scale;
  i2c.writeMaskShiftValue(MPU9250_GYRO_CONFIG, MPU9250_MASK_GYRO_FS_SEL, MPU9250_SHIFT_GYRO_FS_SEL, scale);
}

GyroscopeScale MPU9250::gyroscopeScaleMode() const
{
  return (GyroscopeScale)i2c.readMaskShift(MPU9250_GYRO_CONFIG, MPU9250_MASK_GYRO_FS_SEL, MPU9250_SHIFT_GYRO_FS_SEL);
//  return _gyroscopeScale;
}

uint8_t MPU9250::identification() const
{
  return i2c.readByte(MPU9250_WHO_AM_I);
}

bool MPU9250::connected() const
{
  uint8_t id = identification();
  return 
       id == MPU9250_IDENTIFIER_0
    || id == MPU9250_IDENTIFIER_1
    || id == MPU9250_IDENTIFIER_2
    || id == MPU9250_IDENTIFIER_3
  ;
}

void MPU9250::enableI2CBypass()
{
  i2c.writeMaskSet(MPU9250_INT_PIN_CFG, MPU9250_MASK_BYPASS_EN);
}

void MPU9250::enableRawDataReadyInterrupt(bool enable)
{
  i2c.writeMaskSet(MPU9250_INT_PIN_CFG, MPU9250_MASK_RAW_RDY_INT);
}

void MPU9250::setGyroscopeEnabled(bool en)
{
  if (en)
    i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MAKS_MPU9250_DISABLE_GYRO);
  else
    i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MAKS_MPU9250_DISABLE_GYRO);
}

void MPU9250::enableGyroscope()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MAKS_MPU9250_DISABLE_GYRO);
}

void MPU9250::disableGyroscope()
{
  i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MAKS_MPU9250_DISABLE_GYRO);
}

void MPU9250::setAccelometerEnabled(bool en)
{
  if (en)
    i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MAKS_MPU9250_DISABLE_ACCEL);
  else
    i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MAKS_MPU9250_DISABLE_ACCEL);
}


void MPU9250::enableAccelometer()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MAKS_MPU9250_DISABLE_ACCEL);
}

void MPU9250::disableAccelometer()
{
  i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MAKS_MPU9250_DISABLE_ACCEL);
}

void MPU9250::calibrate()
{
  Point3D<int32_t> gyroscopeOffset;
  determineGyroscopeOffset(gyroscopeOffset);
  setGyroscopeOffset(gyroscopeOffset);
//  accelerationBias, gyroscopeBias);
}

void MPU9250::gyroscope(Point3D<float>& dest) const
{
  Point3D<int16_t> raw;
  gyroscope(raw);
  dest = raw;
  dest *= gyroscopeResolution();
}

void MPU9250::acceleration(Point3D<float>& dest) const
{
  Point3D<int16_t> raw;
  acceleration(raw);
  dest = raw;
  dest *= accelerationResolution();
}

void MPU9250::acceleration(Point3D<int16_t>& destination) const
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  i2c.readBytes(MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination.x = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination.y = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination.z = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

uint16_t MPU9250::fifo(Point3D<int16_t>* destination, uint16_t length)
{
  uint16_t count = 0;
  uint16_t packetCount;
  uint16_t i;
  uint8_t data[6];
  while(count < length)
  {
    packetCount = fifoCount() / 6;
    for (i = 0; i < packetCount && count < length; ++i, ++count)
    {
      i2c.readBytes(MPU9250_FIFO_R_W, 6, &data[0]);
      bytesToPoint3D(&data[0], destination[count]);
    }
  }
  return count;
}

uint16_t MPU9250::acceleration(Point3D<float>* destination, uint16_t length)
{
  float resolution = accelerationResolution();
  Point3D<int16_t> samples[length];
  uint8_t got = acceleration(samples, length);
  for (uint8_t i = 0 ; i < got ; ++i){
    destination[i] = samples[i];
    destination[i] *= resolution;
  }
  return got;
}

uint16_t MPU9250::acceleration(Point3D<int16_t>* destination, uint16_t length)
{
  enableFifo();
  Serial.print("fifoCount:");
  Serial.println(fifoCount());
  resetFifo();
  while(!isFifoReset())
    Serial.print("Wait for FIFO reset done");
  Serial.print("fifoCount:");
  Serial.println(fifoCount());
//  int d = 1000. * length * accelometerDataDelay();
//  Serial.println(d);
  enableAccelometerToFifo();
  //delay(d); 
  uint16_t packetCount = fifo(destination, length);
  disableAccelometerToFifo();

  if (packetCount < length)
  {
    Serial.print("Got less packets than expected ");
    Serial.print(packetCount);
    Serial.print(" < ");
    Serial.print(length);
    Serial.println();
  }
  return packetCount;
}

uint16_t MPU9250::gyroscope(Point3D<float>* destination, uint16_t length)
{
  float resolution = gyroscopeResolution();
  Point3D<int16_t> samples[length];
  uint8_t got = gyroscope(samples, length);
  for (uint8_t i = 0 ; i < got ; ++i){
    destination[i] = samples[i];
    destination[i] *= resolution;
  }
  return got;
}

uint16_t MPU9250::gyroscope(Point3D<int16_t>* destination, uint16_t length)
{
  enableFifo();
  Serial.print("fifoCount:");
  Serial.println(fifoCount());
  resetFifo();
  while(!isFifoReset())
    Serial.print("Wait for FIFO reset done");
  Serial.print("fifoCount:");
  Serial.println(fifoCount());
  //int d = 1000. * length * gyroscopeDataDelay();
  //Serial.println(d);
  enableGyroscopeToFifo();
  //delay(d);
  uint16_t packetCount = fifo(destination, length);
  disableGyroscopeToFifo();

  if (packetCount != length)
  {
    Serial.print("Got more of less gyro packets than expected ");
    Serial.print(packetCount);
    Serial.print(" < ");
    Serial.print(length);
    Serial.println();
  }
  
  return packetCount;
}

uint16_t MPU9250::testGyroscopeFifo(uint16_t length)
{
  uint8_t data[6]; // data array to hold accelerometer x, y, z data
  uint16_t packetCount;
  uint16_t count = 0;
  uint8_t i;
  Point3D<int16_t> destination;
  enableFifo();
  resetFifo();
  while(!isFifoReset())
    Serial.print("Wait for FIFO reset done");
  long start = micros();
  enableGyroscopeToFifo();
  while(count < length)
  {
    packetCount = fifoCount() / 6;
    for (i = 0; i < packetCount && i < length; ++i) {
      i2c.readBytes(MPU9250_FIFO_R_W, 6, &data[0]);
      bytesToPoint3D(&data[0], destination);
      ++count;
    }
  }
  disableGyroscopeToFifo();
  long stop = micros();
  long duration = stop - start;
  Serial.print("Sampled fifo @ ");
  Serial.print(static_cast<float>(count) / duration * 1000000);
  Serial.print(" Hz");
  Serial.println();
  return min(count, length);
}

uint16_t MPU9250::testAccelometerFifo(uint16_t length)
{
  uint8_t data[6]; // data array to hold accelerometer x, y, z data
  uint16_t packetCount;
  uint16_t count = 0;
  uint8_t i;
  Point3D<int16_t> destination;
  enableFifo();
  resetFifo();
  while(!isFifoReset())
    Serial.print("Wait for FIFO reset done");
  long start = micros();
  enableAccelometerToFifo();
  while(count < length)
  {
    packetCount = fifoCount() / 6;
    for (i = 0; i < packetCount && i < length; ++i) {
      i2c.readBytes(MPU9250_FIFO_R_W, 6, &data[0]);
      bytesToPoint3D(&data[0], destination);
      ++count;
    }
  }
  disableAccelometerToFifo();
  long stop = micros();
  long duration = stop - start;
  Serial.print("Sampled fifo @ ");
  Serial.print(static_cast<float>(count) / duration * 1000000);
  Serial.print(" Hz");
  Serial.println();
  return min(count, length);
}

float MPU9250::gyroscopeDataOutputRate() const
{
  float i = gyroInternalFrequency();
  float d = 1.;
  if (isGyroscopeSampleRateDividerEffective())
    d = sampleRateDivider();
  return i / d;
}

float MPU9250::gyroscopeDataDelay() const
{
  return 1. / gyroscopeDataOutputRate();
}

float MPU9250::accelometerDataOutputRate() const
{
  float i = accelometerInternalFrequency();
  float d = 1;
  if (isAccelometerSampleRateDividerEffective())
    d = sampleRateDivider();
  return i / d;
}

float MPU9250::accelometerDataDelay() const
{
  return 1. / accelometerDataOutputRate();
}

bool MPU9250::isAccelometerSampleRateDividerEffective() const
{
  uint8_t fchoice = accelometerFrequencyChoice();
  uint8_t dlpf_cfg = accelometerDlpfConfig();
  return fchoice == 0x00 && dlpf_cfg > 0 && dlpf_cfg < 7;
}

uint8_t MPU9250::accelometerFrequencyChoice() const
{
  return i2c.readMaskShift(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_ACCEL_FCHOICE_B, MPU9250_SHIFT_ACCEL_FCHOICE_B);
}

uint8_t MPU9250::accelometerDlpfConfig() const
{
  return i2c.readMaskShift(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_A_DLPF_CFG, MPU9250_SHIFT_A_DLPF_CFG);
}

bool MPU9250::isGyroscopeSampleRateDividerEffective() const
{
  uint8_t fchoice = gyroscopeFrequencyChoice();
  uint8_t dlpf_cfg = gyroscopeDlpfConfig();
  return fchoice == 0x00 && dlpf_cfg > 0 && dlpf_cfg < 7;
}

uint8_t MPU9250::gyroscopeFrequencyChoice() const
{
  return i2c.readMaskShift(MPU9250_GYRO_CONFIG, MPU9250_MASK_FCHOICE_B, MPU9250_SHIFT_FCHOICE_B);
}

uint8_t MPU9250::gyroscopeDlpfConfig() const
{
  return i2c.readMaskShift(MPU9250_CONFIG, MPU9250_MASK_DLPF_CFG, MPU9250_SHIFT_DLPF_CFG);
}


void MPU9250::gyroscope(Point3D<int16_t>& destination) const
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  i2c.readBytes(MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination.x = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination.y = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination.z = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU9250::temperature(uint16_t& temp) const
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  i2c.readBytes(MPU9250_TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  temp = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

//TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity)+ 21degC
void MPU9250::temperature(float& temp) const
{
  uint16_t r;
  temperature(r);
  double t_o = r;
  double t_off = 0.0;
  double t_sens = 1;
  temp = (t_o - t_off) / t_sens + 21.0;
}


bool MPU9250::available() const
{
  return (i2c.readByte(MPU9250_INT_STATUS) & 0x01);
}

float MPU9250::gyroscopeResolution() const
{
  switch (_gyroscopeScale)
  {
    case GS_250DPS:  return 250.0/32768.0;
    case GS_500DPS:  return 500.0/32768.0;
    case GS_1000DPS: return 1000.0/32768.0;
    case GS_2000DPS: return 2000.0/32768.0;
  }
  return 0;
}

float MPU9250::accelerationResolution() const
{
  switch (_accelometerScale)
  {
    case AS_2G:  return 2.0/32768.0;
    case AS_4G:  return 4.0/32768.0;
    case AS_8G:  return 8.0/32768.0;
    case AS_16G: return 16.0/32768.0;
  }
  return 0;
}

void MPU9250::wakeUp()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_1, BIT_6_5);
  delay(100);
}

void MPU9250::setClockSource(ClockSource source)
{
  i2c.writeMaskShiftValue(MPU9250_PWR_MGMT_1, MPU9250_MASK_CLKSEL, MPU9250_CLKSEL_SHIFT, source);
  delay(200); 
}

ClockSource MPU9250::clockSource() const
{
  return (ClockSource)i2c.readMaskShift(MPU9250_PWR_MGMT_1, MPU9250_MASK_CLKSEL, MPU9250_CLKSEL_SHIFT);
}

/*
void MPU9250::configure()
{
  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
   i2c.writeByte(MPU9250_CONFIG, 0x03);
}
*/
void MPU9250::setSampleRateDividerMode(SampleRateDivider rate)
{
  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate 
  // determined inset in CONFIG above  
   i2c.writeByte(MPU9250_SMPLRT_DIV, rate);
}

uint8_t MPU9250::sampleRateDivider() const
{
  SampleRateDivider mode = sampleRateDividerMode();
  switch(mode){
    case RATE_DIVIDER_0: return 1;
    case RATE_DIVIDER_1: return 2;
    case RATE_DIVIDER_2: return 3;
    case RATE_DIVIDER_3: return 4;
    case RATE_DIVIDER_4: return 5;
    case RATE_DIVIDER_5: return 6;
    case RATE_DIVIDER_6: return 7;
    case RATE_DIVIDER_7: return 8;    
  }
}

void MPU9250::enableGyroX()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_XG);
}

void MPU9250::enableGyroY()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_YG);
}

void MPU9250::enableGyroZ()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_ZG);
}


void MPU9250::disableGyroX()
{
  i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_XG);
}

void MPU9250::disableGyroY()
{
  i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_YG);
}

void MPU9250::disableGyroZ()
{
  i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_ZG);
}


bool MPU9250::gyroscopeXEnabled() const
{
  return !i2c.readMaskBit(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_XG);
}

bool MPU9250::gyroscopeYEnabled() const
{
  return !i2c.readMaskBit(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_YG);
}

bool MPU9250::gyroscopeZEnabled() const
{
  return !i2c.readMaskBit(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_ZG);
}


void MPU9250::enableAccelX()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_XA);
}

void MPU9250::enableAccelY()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_YA);
}

void MPU9250::enableAccelZ()
{
  i2c.writeMaskClear(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_ZA);
}


void MPU9250::disableAccelX()
{
  i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_XA);
}

void MPU9250::disableAccelY()
{
  i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_YA);
}

void MPU9250::disableAccelZ()
{
  i2c.writeMaskSet(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_ZA);
}


bool MPU9250::accelometerXEnabled() const
{
  return !i2c.readMaskBit(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_XA);
}

bool MPU9250::accelometerYEnabled() const
{
  return !i2c.readMaskBit(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_YA);
}

bool MPU9250::accelometerZEnabled() const
{
  return !i2c.readMaskBit(MPU9250_PWR_MGMT_2, MPU9250_MASK_DISABLE_ZA);
}



void MPU9250::setAccelometerRate(AccelometerRate rate)
{
  uint8_t mask = MPU9250_MASK_ACCEL_FCHOICE_B & MPU9250_MASK_A_DLPF_CFG;
  i2c.writeMaskClear(MPU9250_ACCEL_CONFIG2, mask);
  if (rate == AR_1130)
  {
    mask = MPU9250_MASK_ACCEL_FCHOICE_B & MPU9250_MASK_A_DLPF_CFG;
    i2c.writeMaskClearSet(MPU9250_ACCEL_CONFIG2, mask);
    return;
  }
  i2c.writeMaskShiftValue(MPU9250_ACCEL_CONFIG2, MPU9250_MASK_ACCEL_FCHOICE_B, MPU9250_SHIFT_ACCEL_FCHOICE_B, rate);
}

void MPU9250::configureInterrupts()
{
  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  i2c.writeByte(MPU9250_INT_PIN_CFG, 0x22);
  i2c.writeByte(MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

uint16_t MPU9250::fifoCount() const
{
  uint8_t data[2];
  i2c.readBytes(MPU9250_FIFO_COUNTH, 2, &data[0]);
  return ((uint16_t)data[0] << 8) | data[1];
}

void MPU9250::resetFifo()
{
  i2c.writeMaskSet(MPU9250_USER_CTRL, MPU9250_MASK_FIFO_RST);
}

bool MPU9250::isFifoReset() const
{
  return !i2c.readMaskBit(MPU9250_USER_CTRL, MPU9250_MASK_FIFO_RST);
}

void MPU9250::initialize()
{  
//  wakeUp();
//  setClockSource(MPU9250_CS_AUTOMATIC);
//  enableAccelometer();
//  enableGyroscope();
//  setSampleRateDivider(RATE_DIVIDER_1);
//  setGyroscopeScale(GS_250DPS);
//  setAccelometerScale(AS_2G);
//  setAccelometerRate(AR_41);
//  setFifoMode(FM_OVERWRITE);
//  setFsync(DISABLED);
//  enableI2CBypass();
//  enableRawDataReadyInterrupt(true);
//  configureInterrupts();
}

void MPU9250::configureForCalibration()
{
  // reset device
//   i2c.writeByte(MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
//   delay(100);
    
  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
  // else use the internal oscillator, bits 2:0 = 001
   i2c.writeByte(MPU9250_PWR_MGMT_1, 0x01);  
   i2c.writeByte(MPU9250_PWR_MGMT_2, 0x00);
   delay(200);

 // Configure device for bias calculation
   i2c.writeByte(MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
   i2c.writeByte(MPU9250_FIFO_EN, 0x00);      // Disable FIFO
   i2c.writeByte(MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
   i2c.writeByte(MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
   i2c.writeByte(MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
   i2c.writeByte(MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
   delay(15);
   
   // Configure MPU6050 gyro and accelerometer for bias calculation
   i2c.writeByte(MPU9250_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
   i2c.writeByte(MPU9250_SMPLRT_DIV, 0x00);  // Set sample rate to 1130 Hz
   i2c.writeByte(MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
   //i2c.writeByte(MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity  
   //setAccelometerScale(AS_2G);
   setAccelometerScale(AS_16G);
}

void MPU9250::determineAccelometerOffset(Point3D<float>& offset)
{
  Point3D<int32_t> pi;
  float sensitivity = 16384.;  // = 16384 LSB/G
  determineAccelometerOffset(pi);
  offset = pi;
  offset /= sensitivity;
  //offset.x = (float) pi.x / sensitivity;
  //offset.y = (float) pi.y / sensitivity;
  //offset.z = (float) pi.z / sensitivity;
}

void MPU9250::determineAccelometerOffset(Point3D<int32_t>& offset)
{
  uint8_t data[6]; // data array to hold accelerometer x, y, z data
  uint16_t ii, packet_count;
  Point3D<int32_t> sum;
  uint32_t expected = 80;

  configureForCalibration();
  enableFifo();
  resetFifo();
  while(!isFifoReset())
    Serial.print("Wait for FIFO reset done");

  enableAccelometerToFifo();
  // accumulate 80 samples in 80 milliseconds = 480 bytes
  delay(1000 * expected / 1130); 
  disableAccelometerToFifo();

  packet_count = fifoCount() / 6;
  if (packet_count != expected)
  {
    Serial.print("Expected ");
    Serial.print(expected);
    Serial.print(" measurements on Fifo.");
    Serial.print(" Found ");
    Serial.print(packet_count);
    Serial.println();
  }
  //iterate over fifo and sum up
  Point3D<int16_t> t;
  Point3D<int32_t> t2;
  for (uint8_t i = 0; i < packet_count; i++) {
    i2c.readBytes(MPU9250_FIFO_R_W, 6, &data[0]);
    bytesToPoint3D(&data[0], t);
    //printPoint(t, 10);
    //Serial.println();
    t2 = t;
    sum += t2;
  }
  Serial.print("Sum xyz ");
  printPoint(sum, 10);
  Serial.println();

  // Normalize sums to get average count biases
  sum /= packet_count;
  Serial.print("Avg xyz ");
  printPoint(sum, 10);
  Serial.println();
  
  offset = sum;
  //sum.x -= 16384;
  //offset.x = sum.z;
  //offset.y = sum.y;
  //offset.z = sum.x;

  Serial.print("Avg xyz ");
  printPoint(sum, 10);
  Serial.println();
}

void MPU9250::determineGyroscopeOffset(Point3D<float>& offset)
{
  Point3D<int32_t> pi;
  float sensitivity = 131.;  // = 131 LSB/degrees/sec
  determineGyroscopeOffset(pi);
  offset = pi;
  offset /= sensitivity;
}

/*
 Function which accumulates gyro and accelerometer data after device initialization. 
 It calculates the average of the at-rest readings and then loads the resulting Offset into accelerometer and gyro bias registers.
*/
void MPU9250::determineGyroscopeOffset(Point3D<int32_t>& gyroscopeBias)
{
  uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t bias[3]  = {0, 0, 0};
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint8_t expected = 80;
  
  configureForCalibration();
  enableFifo();
  resetFifo();
  while(!isFifoReset())
    Serial.print("Wait for FIFO reset done");

  enableGyroscopeToFifo();
  // accumulate 80 samples in 80 milliseconds = 480 bytes
  delay(expected); 

  // At end of sample accumulation, turn off FIFO sensor read
  disableGyroscopeToFifo();

  // read FIFO sample count
  // How many sets of full gyro and accelerometer data for averaging
  i2c.readBytes(MPU9250_FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/6;
  if (packet_count != expected)
  {
    Serial.print("Expected ");
    Serial.print(expected);
    Serial.print(" measurements on Fifo.");
    Serial.print(" Found ");
    Serial.print(packet_count);
    Serial.println();
  }
  for (ii = 0; ii < packet_count; ii++) {
    int16_t temp[3] = {0, 0, 0};

    // read data for averaging
    i2c.readBytes(MPU9250_FIFO_R_W, 6, &data[0]);

    // Form signed 16-bit integer for each sample in FIFO
    temp[0]  = (int16_t) (((int16_t)data[0] << 8) | data[1]);
    temp[1]  = (int16_t) (((int16_t)data[2] << 8) | data[3]);
    temp[2]  = (int16_t) (((int16_t)data[4] << 8) | data[5]);
    
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    bias[0]  += (int32_t) temp[0];
    bias[1]  += (int32_t) temp[1];
    bias[2]  += (int32_t) temp[2];
  }

  // Normalize sums to get average count biases
  bias[0]  /= (int32_t) packet_count;
  bias[1]  /= (int32_t) packet_count;
  bias[2]  /= (int32_t) packet_count;

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  // Biases are additive, so change sign on calculated average gyro biases
  data[0] = (-bias[0]/4  >> 8) & 0xFF; 
  data[1] = (-bias[0]/4)       & 0xFF; 
  data[2] = (-bias[1]/4  >> 8) & 0xFF;
  data[3] = (-bias[1]/4)       & 0xFF;
  data[4] = (-bias[2]/4  >> 8) & 0xFF;
  data[5] = (-bias[2]/4)       & 0xFF;
  
//  writeGyroscopeOffset(data);
  
  // Output scaled gyro biases for display in the main program
  gyroscopeBias.x = bias[0];
  gyroscopeBias.y = bias[1];
  gyroscopeBias.z = bias[2];
}

/*
OffsetLSB= X_OFFS_USR * 4 / 2^FS_SEL
OffsetDPS= X_OFFS_USR * 4 / 2^FS_SEL / Gyro_Sensitivity
Nominal Conditions FS_SEL = 0
Gyro_Sensitivity = 2^16 LSB / 500dps
Max 999.969 dps
Min -1000 dps
Step 0.0305 dps
*/
void MPU9250::bytesToPoint3D(uint8_t* data, Point3D<int16_t>& p)
{
//  Serial.println(((int16_t)data[0] << 8));
  p.x = (int16_t) ( ((int16_t)data[0] << 8) | data[1] );
  p.y = (int16_t) ( ((int16_t)data[2] << 8) | data[3] );
  p.z = (int16_t) ( ((int16_t)data[4] << 8) | data[5] );
}

void MPU9250::point3DToBytes(const Point3D<int16_t> &p, uint8_t *data)
{
  data[0] = p.x >> 8;
  data[1] = p.x;
  data[2] = p.y >> 8;
  data[3] = p.y;
  data[4] = p.z >> 8;
  data[5] = p.z;
}

void MPU9250::gyroscopeOffset(Point3D<int16_t> &p)
{
  uint8_t data[6];
  i2c.readBytes(MPU9250_XG_OFFSET_H, 6, &data[0]);
  bytesToPoint3D(data, p);
}

void MPU9250::setGyroscopeOffset(const Point3D<int16_t>& p)
{
  uint8_t data[6];
  point3DToBytes(p, &data[0]);
  setGyroscopeOffset(data);
}

void MPU9250::setGyroscopeOffset(uint8_t* data)
{
  // Push gyro biases to hardware registers
  i2c.writeByte(MPU9250_XG_OFFSET_H, data[0]);
  i2c.writeByte(MPU9250_XG_OFFSET_L, data[1]);
  i2c.writeByte(MPU9250_YG_OFFSET_H, data[2]);
  i2c.writeByte(MPU9250_YG_OFFSET_L, data[3]);
  i2c.writeByte(MPU9250_ZG_OFFSET_H, data[4]);
  i2c.writeByte(MPU9250_ZG_OFFSET_L, data[5]);
}
  

void MPU9250::dumpMem() const
{
  uint8_t data[9];
  i2c.readBytes(MPU9250_XA_OFFSET_H, 9, &data[0]);
  for (int i = 0 ; i < 9 ; ++i)
  {
    Serial.print(data[i],16);
    Serial.print(' ');
  }
  Serial.println();
}

void MPU9250::accelometerOffset(Point3D<float>& p)
{
  Point3D<int16_t> pi;
  accelometerOffset(pi);
  p = pi;
  float sensitivity = 16384.;
  p /= sensitivity;
}

void MPU9250::accelometerOffset(Point3D<int16_t>& p)
{
  //Serial.println("Read accel offset");  
  uint8_t data[6];
  i2c.readBytes(MPU9250_XA_OFFSET_H, 2, &data[0]);
  //Serial.println(data[0]);
  //Serial.println(data[1]);
  i2c.readBytes(MPU9250_YA_OFFSET_H, 2, &data[2]);
  //Serial.println(data[2]);
  //Serial.println(data[3]);
  i2c.readBytes(MPU9250_ZA_OFFSET_H, 2, &data[4]);
  //Serial.println(data[4]);
  //Serial.println(data[5]);
//  printDataArr(&data[0]);
  bytesToPoint3D(data, p);
  p /= 2;
}

void MPU9250::setAccelometerOffset(const Point3D<int16_t>& p)
{
  uint8_t data[6];
  point3DToBytes(p, data);
  setAccelometerOffset(data);
}

void MPU9250::setAccelometerOffset(uint8_t* accel_bias)
{
  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  //
  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  //Serial.println("setAccelometerOffset");
  //Serial.println(accel_bias[0]);
  //Serial.println(accel_bias[1]);
  //Serial.println(accel_bias[2]);
  //Serial.println(accel_bias[3]);
  //Serial.println(accel_bias[4]);
  //Serial.println(accel_bias[5]);

  uint16_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  uint8_t data[6];
  //uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  Point3D<uint8_t> maskBit;

  // Read factory accelerometer trim values
  maskBit.x = i2c.readMaskBit(MPU9250_XA_OFFSET_L, 0x01);
  maskBit.y = i2c.readMaskBit(MPU9250_YA_OFFSET_L, 0x01);
  maskBit.z = i2c.readMaskBit(MPU9250_ZA_OFFSET_L, 0x01);

  Serial.print("mask bits xyz ");
  Serial.print(maskBit.x);
  Serial.print(',');
  Serial.print(maskBit.y);
  Serial.print(',');
  Serial.print(maskBit.z);
  Serial.println();

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  //  accel_bias_reg[0] -= (accel_bias[0]/8); 
  //  accel_bias_reg[1] -= (accel_bias[1]/8);
  //  accel_bias_reg[2] -= (accel_bias[2]/8);

  accel_bias_reg[0] = accel_bias[0] << 8 | accel_bias[1]; 
  accel_bias_reg[1] = accel_bias[2] << 8 | accel_bias[3];
  accel_bias_reg[2] = accel_bias[4] << 8 | accel_bias[5];

  accel_bias_reg[0] = accel_bias_reg[0] << 1;
  accel_bias_reg[1] = accel_bias_reg[1] << 1;
  accel_bias_reg[2] = accel_bias_reg[2] << 1;
  
  data[0] = (accel_bias_reg[0] >> 8);
  data[1] = (accel_bias_reg[0]);
  data[1] = data[1] | maskBit.x; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8);
  data[3] = (accel_bias_reg[1]);
  data[3] = data[3] | maskBit.y; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8);
  data[5] = (accel_bias_reg[2]);
  data[5] = data[5] | maskBit.z; // preserve temperature compensation bit when writing back to accelerometer bias registers

  //Serial.println("Write to A_OFFSET");
  //Serial.println(data[0]);
  //Serial.println(data[1]);
  //Serial.println(data[2]);
  //Serial.println(data[3]);
  //Serial.println(data[4]);
  //Serial.println(data[5]);
  i2c.writeByte(MPU9250_XA_OFFSET_H, data[0]);
  i2c.writeByte(MPU9250_XA_OFFSET_L, data[1]);
  i2c.writeByte(MPU9250_YA_OFFSET_H, data[2]);
  i2c.writeByte(MPU9250_YA_OFFSET_L, data[3]);
  i2c.writeByte(MPU9250_ZA_OFFSET_H, data[4]);
  i2c.writeByte(MPU9250_ZA_OFFSET_L, data[5]);

  //Serial.println("Read from A_OFFSET");
  //Serial.println(i2c.readByte(MPU9250_XA_OFFSET_H));
  //Serial.println(i2c.readByte(MPU9250_XA_OFFSET_L));
  //Serial.println(i2c.readByte(MPU9250_YA_OFFSET_H));
  //Serial.println(i2c.readByte(MPU9250_YA_OFFSET_L));
  //Serial.println(i2c.readByte(MPU9250_ZA_OFFSET_H));
  //Serial.println(i2c.readByte(MPU9250_ZA_OFFSET_L));

}

void MPU9250::readAccelometerOffsetRaw(Point3D<uint16_t>& bits)
{
  uint8_t data[2];
  i2c.readBytes(MPU9250_XA_OFFSET_H, 2, &data[0]);
  bits.x = (uint16_t)data[0] << 8 | data[1];
  i2c.readBytes(MPU9250_YA_OFFSET_H, 2, &data[0]);
  bits.y = (uint16_t)data[0] << 8 | data[1];
  i2c.readBytes(MPU9250_ZA_OFFSET_H, 2, &data[0]);
  bits.z = (uint16_t)data[0] << 8 | data[1];
}

void MPU9250::readAccelometerUnspecifiedBits(Point3D<uint8_t>& bits)
{
  uint8_t data[6];
  // Read factory accelerometer trim values
  i2c.readBytes(MPU9250_XA_OFFSET_H, 6, &data[0]); 
  bits.x = data[1] & 0x01;
  bits.y = data[3] & 0x01;
  bits.z = data[5] & 0x01;
}

void MPU9250::printSelfTest(Point3D<float>& accelerationFactor, Point3D<float>& gyroscopeFactor)
{
  Serial.print("x-axis self test: acceleration trim within : "); 
  Serial.print(accelerationFactor.x * 100.); 
  Serial.println("% of factory value");

  Serial.print("y-axis self test: acceleration trim within : ");
  Serial.print(accelerationFactor.y * 100.);
  Serial.println("% of factory value");

  Serial.print("z-axis self test: acceleration trim within : ");
  Serial.print(accelerationFactor.z * 100.);
  Serial.println("% of factory value");

  Serial.print("x-axis self test: gyration trim within : ");
  Serial.print(gyroscopeFactor.x * 100.);
  Serial.println("% of factory value");

  Serial.print("y-axis self test: gyration trim within : ");
  Serial.print(gyroscopeFactor.y * 100.);
  Serial.println("% of factory value");

  Serial.print("z-axis self test: gyration trim within : ");
  Serial.print(gyroscopeFactor.z * 100.);
  Serial.println("% of factory value");
}

/*
 Accelerometer and gyroscope self test; check calibration wrt factory settings
 Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
*/
void MPU9250::selfTest(Point3D<float>&accelerationFactor, Point3D<float>&gyroscopeFactor)
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  i2c.writeByte(MPU9250_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  i2c.writeByte(MPU9250_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  i2c.writeByte(MPU9250_GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  i2c.writeByte(MPU9250_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  i2c.writeByte(MPU9250_ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
    i2c.readBytes(MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

    i2c.readBytes(MPU9250_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  i2c.writeByte(MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  i2c.writeByte(MPU9250_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
    i2c.readBytes(MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

    i2c.readBytes(MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }   

  // Configure the gyro and accelerometer for normal operation
  i2c.writeByte(MPU9250_ACCEL_CONFIG, 0x00);  
  i2c.writeByte(MPU9250_GYRO_CONFIG,  0x00);  
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = i2c.readByte(MPU9250_SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = i2c.readByte(MPU9250_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = i2c.readByte(MPU9250_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = i2c.readByte(MPU9250_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = i2c.readByte(MPU9250_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = i2c.readByte(MPU9250_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  accelerationFactor.x = ((float)(aSTAvg[0] - aAvg[0]))/factoryTrim[0] - 1;
  accelerationFactor.y = ((float)(aSTAvg[1] - aAvg[1]))/factoryTrim[1] - 1;
  accelerationFactor.z = ((float)(aSTAvg[2] - aAvg[2]))/factoryTrim[2] - 1;
  gyroscopeFactor.x = ((float)(gSTAvg[0] - gAvg[0]))/factoryTrim[3] - 1;
  gyroscopeFactor.y = ((float)(gSTAvg[1] - gAvg[1]))/factoryTrim[4] - 1;
  gyroscopeFactor.z = ((float)(gSTAvg[2] - gAvg[2]))/factoryTrim[5] - 1;
}

void MPU9250::gyroscopeSelfTestValue(Point3D<uint8_t>& point) const
{
  point.x = i2c.readByte(MPU9250_SELF_TEST_X_ACCEL);
  point.y = i2c.readByte(MPU9250_SELF_TEST_Y_ACCEL);
  point.z = i2c.readByte(MPU9250_SELF_TEST_Z_ACCEL);
}

void MPU9250::accelometerSelfTestValue(Point3D<uint8_t>& point) const
{
  point.x = i2c.readByte(MPU9250_SELF_TEST_X_GYRO);
  point.y = i2c.readByte(MPU9250_SELF_TEST_Y_GYRO);
  point.z = i2c.readByte(MPU9250_SELF_TEST_Z_GYRO);
}

void MPU9250::accelerationAverage(Point3D<int16_t> average, uint16_t count) const
{
  Point3D<int32_t> sum;
  Point3D<int32_t> acceleration32;
  Point3D<int16_t> acceleration16;
  for(uint8_t i = 0; i < 200; ++i)
  {
    acceleration(acceleration16);
    acceleration32 = acceleration16;
    sum += acceleration32;
  }
  sum /= count;
  average = sum;
}

void MPU9250::accelerationAverage(Point3D<float> average, uint16_t count) const
{
  Point3D<int32_t> sum;
  Point3D<int32_t> acceleration32;
  Point3D<int16_t> acceleration16;
  for(uint8_t i = 0; i < 200; ++i)
  {
    acceleration(acceleration16);
    acceleration32 = acceleration16;
    sum += acceleration32;
  }
  sum /= count;
  average = sum;
}

void MPU9250::gyroscopeAverage(Point3D<int16_t> average, uint16_t count) const
{
  Point3D<int32_t> sum;
  Point3D<int32_t> gyroscope32;
  Point3D<int16_t> gyroscope16;
  for(uint8_t i = 0; i < 200; ++i)
  {
    gyroscope(gyroscope16);
    gyroscope32 = gyroscope16;
    sum += gyroscope32;
  }
  sum /= count;
  average = sum;
}

void MPU9250::gyroscopeAverage(Point3D<float> average, uint16_t count) const
{
  Point3D<int32_t> sum;
  Point3D<int32_t> gyroscope32;
  Point3D<int16_t> gyroscope16;
  for(uint8_t i = 0; i < 200; ++i)
  {
    gyroscope(gyroscope16);
    gyroscope32 = gyroscope16;
    sum += gyroscope16;
  }
  sum /= count;
  average = sum;
}

void MPU9250::enableAccelometerSelfTest() const
{
  i2c.writeMaskSet(MPU9250_ACCEL_CONFIG, MPU9250_MASK_A_ST_EN);
}

void MPU9250::enableGyroscopeSelfTest() const
{
  i2c.writeMaskSet(MPU9250_GYRO_CONFIG, MPU9250_MASK_GYRO_CT_EN);
}

void MPU9250::disableAccelometerSelfTest() const
{
  i2c.writeMaskClear(MPU9250_ACCEL_CONFIG, MPU9250_MASK_A_ST_EN);
}

void MPU9250::disableGyroscopeSelfTest() const
{
  i2c.writeMaskClear(MPU9250_GYRO_CONFIG, MPU9250_MASK_GYRO_CT_EN);
}

/*
 Accelerometer and gyroscope self test; check calibration wrt factory settings
 Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
*/
void MPU9250::selfTest2(Point3D<float>& accelerationFactor, Point3D<float>& gyroscopeFactor)
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  setSampleRateDividerMode(RATE_DIVIDER_0);
  //  i2c.writeByte(MPU9250_CONFIG, 0x02);        // Set FIFO replace, disable FSYNC, set DLPF to 92 Hz
  setFifoMode(FM_OVERWRITE);
  setFsync(MPU_DISABLED);
  setLowPassRate(LPR_92);
  //set gyro sample rate to 1 kHz and 
  //i2c.writeByte(MPU9250_GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  setGyroscopeScale(GS_250DPS);
  setAccelometerRate(AR_92);
  setAccelometerScale(AS_2G);
  //i2c.writeByte(MPU9250_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  //i2c.writeByte(MPU9250_ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g
  // get average current values of gyro and acclerometer

  Point3D<float> averageAcceleration;
  accelerationAverage(averageAcceleration, 200);
  
  Point3D<float> averageGyroscope;
  gyroscopeAverage(averageGyroscope, 200);

  // Configure the accelerometer for self-test
  setAccelometerScale(AS_2G);
  enableAccelometerSelfTest();
  enableGyroscopeSelfTest();
//  i2c.writeByte(MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
//  i2c.writeByte(MPU9250_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  Point3D<float> averageAccelerationSelfTest;
  accelerationAverage(averageAccelerationSelfTest, 200);
  
  Point3D<float> averageGyroscopeSelfTest;
  gyroscopeAverage(averageGyroscopeSelfTest, 200);

  // Configure the gyro and accelerometer for normal operation
  disableAccelometerSelfTest();
  disableGyroscopeSelfTest();
  //i2c.writeByte(MPU9250_ACCEL_CONFIG, 0x00);  
  //i2c.writeByte(MPU9250_GYRO_CONFIG,  0x00);  
  delay(25);  // Delay a while to let the device stabilize

  Point3D<uint8_t> gyroscopeSelfTest;
  gyroscopeSelfTestValue(gyroscopeSelfTest);
  Point3D<float> gyroscopeSelfTestF = gyroscopeSelfTest;

  Point3D<uint8_t> accelometerSelfTest;
  accelometerSelfTestValue(accelometerSelfTest);
  Point3D<float> accelometerSelfTestF = accelometerSelfTest;
  
  gyroscopeSelfTestF -= 1.0;
  accelometerSelfTestF -= 1.0;
  
  Point3D<float> accelometerTrim(1.01, 1.01, 1.01);
  powPoint(accelometerTrim, accelometerSelfTestF);

  Point3D<float> gyroscopeTrim(1.01, 1.01, 1.01);
  powPoint(gyroscopeTrim, gyroscopeSelfTestF);

//  uint8_t FS = 0;
  uint8_t FACTOR = 1 << FS;
  float factor = static_cast<float>(2620 / FACTOR);

  accelometerTrim *= factor;
  gyroscopeTrim *= factor;

  accelometerTrim -= 1.0;
  gyroscopeTrim -= 1.0;

  Point3D<float> accelerationFactor2 = (averageAccelerationSelfTest - averageAcceleration) / accelometerTrim;
  Point3D<float> gyroscopeFactor2 = (averageGyroscopeSelfTest - averageGyroscope) / gyroscopeTrim;
  accelerationFactor = accelerationFactor2;
  gyroscopeFactor = gyroscopeFactor2;
}

uint8_t MPU9250::dumpReg(uint8_t reg, uint8_t mask, uint8_t shift) const
{
  return i2c.readMaskShift(reg, mask, shift);
}
void MPU9250::setReg(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value)
{
  return i2c.writeMaskShiftValue(reg, mask, shift, value);
}
}
