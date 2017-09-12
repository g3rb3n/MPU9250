#ifndef _MPU9250_REGISTER_ENUMS_H
#define _MPU9250_REGISTER_ENUMS_H

enum AccelometerScale {
  AS_2G = 0,
  AS_4G,
  AS_8G,
  AS_16G
};

enum GyroscopeScale {
  GS_250DPS = 0,
  GS_500DPS,
  GS_1000DPS,
  GS_2000DPS
};


enum AccelometerInternalFrequency
{
  AIF_4K = 4000,
  AIF_1K = 1000
};
/*
enum AccelFrequency
{
  AF_460 = 0,
  AF_184,
  AF_92,
  AF_41,
  AF_20,
  AF_10,
  AF_5,
  AF_1130  
};
*/
enum AccelometerFrequency
{
  AF_281_1_NO_DLPF = 0,
  AF_281_1,
  AF_99,
  AF_44_8,
  AF_21_2,
  AF_10_2,
  AF_5_05,
  AF_420,
  AF_1046 = -1
};

enum GyroscopeInternalFrequency
{
  GIF_32K = 32000,
  GIF_8K = 8000,
  GIF_1K = 1000
};

enum GyroscopeFrequency
{
  GF_8800,
  GF_3600,
  GF_3600_NO_DLPF,
  GF_250,
  GF_184,
  GF_92,
  GF_41,
  GF_20,
  GF_10,
  GF_5
};

enum TemperatureFrequency
{
  TF_4000,
  TF_188,
  TF_98,
  TF_42,
  TF_20,
  TF_10,
  TF_5
};

enum AccelometerRate
{
  AR_460 = 0,
  AR_184,
  AR_92,
  AR_41,
  AR_20,
  AR_10,
  AR_5,
  AR_1130  
};

enum SampleRateDivider{
  RATE_DIVIDER_0 = 0,
  RATE_DIVIDER_1,
  RATE_DIVIDER_2,
  RATE_DIVIDER_3,
  RATE_DIVIDER_4,
  RATE_DIVIDER_5,
  RATE_DIVIDER_6,
  RATE_DIVIDER_7
};

enum LowPassRate
{
  LPR_250 = 0,
  LPR_184,
  LPR_92,
  LPR_41,
  LPR_20,
  LPR_10,
  LPR_5,
  LPR_3600
};



enum FifoMode
{
  FM_OVERWRITE = 0,
  FM_HOLD
};

enum ExtSyncSet{
  MPU_DISABLED = 0,
  TEMP_OUT_L,
  GYRO_XOUT_L,
  GYRO_YOUT_L,
  GYRO_ZOUT_L,
  ACCEL_XOUT_L,
  ACCEL_YOUT_L,
  ACCEL_ZOUT_L
};

enum ClockSource
{
  MPU9250_CS_INTERNAL = 0,
  MPU9250_CS_AUTOMATIC = 1,
  MPU9250_CS_STOP = 7
};

#endif
