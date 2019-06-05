/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *                                                       *
 * Class to read Sensor BH 1750                          *
 *                                                       *
 * @measurement Illumination                             *
 * @protocol i2c                                         *
 * @git https://github.com/plomi-net/arduino-codes       *
 * @author MP | Plomi.net                                *
 *                                                       *
 * default i2c addr: 0x76 or 0x72                        *
 *                                                       *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/**
 * Modes:
 *  0x10    measure with accuracy of 1 lx and continious Measurement
 *  0x11    measure with accuracy of 0.5 lx and continious Measurement
 *  0x13    measure with accuracy of 4 lx and continious Measurement
 *  0x20    measure with accuracy of 1 lx and single Measurement
 *  0x21    measure with accuracy of 0.5 lx and single Measurement
 *  0x23    measure with accuracy of 4 lx and single Measurement
 *
 */
#include <Arduino.h>
#include <Wire.h>

#ifdef DEBUG
    #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)  
    #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #else
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
  #endif

#ifndef __SENSOR_BH1750__
#define __SENSOR_BH1750__

class SensorBH1750
{
  private:
    int _addr;
    uint8_t _mode;
    float result=65535;

  public:
    /*
     * Constructor
     */
    SensorBH1750()
    {
        Wire.begin();
    }

    boolean setAddress(uint8_t addr) {
        this->_addr = addr;
        return 1;
    }

    boolean setMode(uint8_t mode) {
        if(
            mode != 0x10 && mode != 0x11 && mode != 0x13 &&
            mode != 0x20 && mode != 0x21 && mode != 0x23
        ) {
            DEBUG_PRINTLN("SensorBH1750 mode not allowed!");
            return 0;
        }
        this->_mode = mode;
        return 1;
    }

    boolean setupSensor() {   
        // send mode to addr
        byte ack = 5;
        if(!this->_mode) {
            DEBUG_PRINTLN("SensorBH1750 mode is not set!");
            return 0;
        }
        Wire.beginTransmission(this->_addr);
        Wire.write(this->_mode);
        ack = Wire.endTransmission();
        delay(10);

        if(ack == 0) {
            DEBUG_PRINTLN("SensorBH1750 ACK OK");
            return 1;
        } else {
            DEBUG_PRINTLN("SensorBH1750 error on setup the device");
            return 0;
        }
    }

    boolean readSensor()
    {
        // needed to request the sensor
        if(!this->setupSensor()) {
            return 0;
        }

        Wire.beginTransmission(this->_addr);
        Wire.write((uint8_t)this->_mode);
        Wire.endTransmission();

        // sensor takes a measure, wait
        if(this->_mode == 0x23 || this->_mode == 0x13) {
            delay(16);
        } else {
            delay(120);
        }

        // receive the data from wire bus
        uint16_t tmp_res=65535;
        Wire.requestFrom(this->_addr, 2);
        if (Wire.available() == 2) {
            tmp_res = Wire.read();
            tmp_res <<= 8;
            tmp_res |= Wire.read();
        }
        this->result = (tmp_res / 1.2);
        return 1;
    }

    float getIlluminance() {
        return this->result;
    }
};
#endif
