#ifndef _MPU9250_H
#define _MPU9250_H

#include <Point3D.h>
#include <I2C.h>

#include "MPU9250_enums.h"

#if defined(ARDUINO_ARCH_ESP8266)
  #define TWO_WIRE_PINS
#else
  #warning "No support for two wire pin setting"
#endif

namespace g3rb3n
{

  class MPU9250
  {
    
  private:
    I2C i2c;

    Point3D<float> accelerationBias = {0, 0, 0};
    Point3D<float> gyroscopeBias = {0, 0, 0};
    
    AccelometerScale _accelometerScale;
    GyroscopeScale _gyroscopeScale;
    
  public:
    MPU9250();
    MPU9250(uint8_t address);
    MPU9250(uint8_t address, uint8_t sda, uint8_t scl);
    ~MPU9250();

    uint8_t address() const;
    uint8_t identification() const;
    bool connected() const;

    void initialize();

    /*
    Returns true if new data is available
    */
    bool available() const;

    /*
    Read acceleration from registers
    */
    void acceleration(Point3D<float>& destination) const;
    void acceleration(Point3D<int16_t>& destination) const;

    /*
    Read acceleration from FIFO
    */
    uint16_t acceleration(Point3D<int16_t>* destination, uint16_t length);
    uint16_t acceleration(Point3D<float>* destination, uint16_t length);

    /*
    Read gyroscope from registers
    */
    void gyroscope(Point3D<float>& destination) const;
    void gyroscope(Point3D<int16_t>& destination) const;

    uint16_t gyroscope(Point3D<int16_t>* destination, uint16_t length);
    uint16_t gyroscope(Point3D<float>* destination, uint16_t length);

    uint16_t testGyroscopeFifo(uint16_t length);
    uint16_t testAccelometerFifo(uint16_t length);

    uint16_t fifo(Point3D<int16_t>* destination, uint16_t length);
    /*
    Read temperature from registers
    */
    void temperature(uint16_t& temp) const;
    void temperature(float& temp) const;

    void reset();
    bool isReset() const;

    /*
    Enables I2C bypass for AK ICU
    */
    void enableI2CBypass();

    /*
    Set interrupt when data is available
    */
    void enableRawDataReadyInterrupt(bool enable);

    /*
    Accelometer scale mode constrols the maximum range 2G, 4G, 8G or 16G
    */
    void setAccelometerScale(AccelometerScale scale);
    AccelometerScale accelometerScaleMode() const;
    float accelerationResolution() const;

    /*
    Gyroscope scale mode constrols the maximum range 250, 500, 1000, 2000
    */
    void setGyroscopeScale(GyroscopeScale scale);
    GyroscopeScale gyroscopeScaleMode() const;
    float gyroscopeResolution() const;
    
    /*
    Sample rate devider
    */
    void setSampleRateDividerMode(SampleRateDivider rate);
    SampleRateDivider sampleRateDividerMode() const;
    uint8_t sampleRateDivider() const;

    void setAccelometerRate(AccelometerRate rate);
    
    /*
    Gyro frequency
    */
    void setGyroscopeFrequencyMode(GyroscopeFrequency mode);
    GyroscopeFrequency gyroscopeFrequencyMode() const;
    uint16_t gyroscopeFrequency() const;
    GyroscopeInternalFrequency gyroInternalFrequencyMode() const;
    float gyroInternalFrequency() const;
    float gyroscopeDataOutputRate() const;
    float gyroscopeDataDelay() const;

    /* 
    Accelometer frequency
    */
    void setAccelometerFrequencyMode(AccelometerFrequency mode);
    AccelometerFrequency accelometerFrequencyMode() const;
    uint16_t accelometerFrequency() const;
    AccelometerInternalFrequency accelometerInternalFrequencyMode() const;
    float accelometerInternalFrequency() const;
    float accelometerFrequencyDivided() const;
    float accelometerDataOutputRate() const;
    float accelometerDataDelay() const;

    bool isAccelometerSampleRateDividerEffective() const;
    uint8_t accelometerFrequencyChoice() const;
    uint8_t accelometerDlpfConfig() const;

    bool isGyroscopeSampleRateDividerEffective() const;
    uint8_t gyroscopeFrequencyChoice() const;
    uint8_t gyroscopeDlpfConfig() const;

    void wakeUp();

    ClockSource clockSource() const;
    void setClockSource(ClockSource source);

    FifoMode fifoMode() const;
    void setFifoMode(FifoMode);
    void enableFifo();
    void disableFifo();
    bool fifoEnabled() const;
    void enableGyroscopeToFifo();
    void disableGyroscopeToFifo();
    void enableAccelometerToFifo();
    void disableAccelometerToFifo();

    void setLowPassRate(LowPassRate lpr);
    void setFsync(ExtSyncSet s);
    void configureInterrupts();
    void setAccelOnlyLowPowerMode();

    void enableAccelometerSelfTest() const;
    void enableGyroscopeSelfTest() const;
    void disableAccelometerSelfTest() const;
    void disableGyroscopeSelfTest() const;

    void accelerationAverage(Point3D<int16_t> average, uint16_t count) const;
    void accelerationAverage(Point3D<float> average, uint16_t count) const;
    void gyroscopeAverage(Point3D<int16_t> average, uint16_t count) const;
    void gyroscopeAverage(Point3D<float> average, uint16_t count) const;

    void gyroscopeSelfTestValue(Point3D<uint8_t>& point) const;
    void accelometerSelfTestValue(Point3D<uint8_t>& point) const;

    void selfTest(Point3D<float> &accelerationBias, Point3D<float> &gyroscopeBias);
    void selfTest2(Point3D<float>& accelerationFactor, Point3D<float>& gyroscopeFactor);

    void calibrate();
    void determineGyroscopeOffset(Point3D<float>& offset);
    void determineGyroscopeOffset(Point3D<int32_t>& offset);
    void determineAccelometerOffset(Point3D<float>& offset);
    void determineAccelometerOffset(Point3D<int32_t>& offset);

    uint16_t fifoCount() const;
    void resetFifo();
    bool isFifoReset() const;
    
    void configureForCalibration();
    void bytesToPoint3D(uint8_t *data, Point3D<int16_t>& p);
    void point3DToBytes(const Point3D<int16_t>& p, uint8_t *data);    

    void gyroscopeOffset(Point3D<int16_t>& p);
    void setGyroscopeOffset(const Point3D<int16_t>& p);
    void setGyroscopeOffset(uint8_t* data);
    
    void accelometerOffset(Point3D<int16_t>& p);
    void accelometerOffset(Point3D<float>& p);
    void setAccelometerOffset(uint8_t* data);
    void setAccelometerOffset(const Point3D<int16_t>& p);
    
    void readAccelometerOffsetRaw(Point3D<uint16_t>& bits);
    void readAccelometerUnspecifiedBits(Point3D<uint8_t>& bits);

    void dumpMem() const;

    void printSelfTest(Point3D<float>& accelerationFactor, Point3D<float>& gyroscopeFactor);


    void setAccelometerEnabled(bool en);
    void setGyroscopeEnabled(bool en);

    void enableAccelometer();
    void enableGyroscope();

    void disableAccelometer();
    void disableGyroscope();
    
    bool accelometerXEnabled() const;
    bool accelometerYEnabled() const;
    bool accelometerZEnabled() const;

    bool gyroscopeXEnabled() const;
    bool gyroscopeYEnabled() const;
    bool gyroscopeZEnabled() const;

    void enableAccelX();
    void enableAccelY();
    void enableAccelZ();

    void enableGyroX();
    void enableGyroY();
    void enableGyroZ();

    void disableAccelX();
    void disableAccelY();
    void disableAccelZ();

    void disableGyroX();
    void disableGyroY();
    void disableGyroZ();
    
    uint8_t dumpReg(uint8_t reg, uint8_t mask, uint8_t shift) const;
    void setReg(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);
  };
}

#endif
