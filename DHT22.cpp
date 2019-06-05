/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *                                                       *
 * Class to read Sensor DHT 22                           *
 *                                                       *
 * @measurement Humidity, Temperature                    *
 * @protocol proprietary                                 *
 * @git https://github.com/plomi-net/arduino-codes       *
 * @author MP | Plomi.net                                *
 *                                                       *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <Arduino.h>

#ifdef DEBUG
    #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)  
    #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #else
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
  #endif

#ifndef __SENSOR_DHT22__
#define __SENSOR_DHT22__

class SensorDHT22
{
  private:
    uint8_t _connectedPin;
    uint32_t _lastTimeReaded;
    float temperature, humidity;

    boolean _calcTemperature(uint8_t byte1, uint8_t byte2)
    {
        float temp = NAN;
        temp = byte1 & 0x7F;
        temp *= 256;
        temp += byte2;
        temp *= 0.1;
        if (byte1 & 0x80)
        {
            temp *= -1;
        }
        this->temperature = temp;
        return 1;
    }

    boolean _calcHumidity(uint8_t byte1, uint8_t byte2)
    {
        float relhum = NAN;
        relhum = byte1;
        relhum *= 256;
        relhum += byte2;
        relhum *= 0.1;
        this->humidity = relhum;
        return 1;
    }

  public:
    /**
     * Constructor
     */
    SensorDHT22()
    {
        
    }

    boolean setupSensor(byte connectedPin) {
        if (isnan(this->_connectedPin))
        {
            DEBUG_PRINTLN("SensorDHT22 has no configured Pin");
            return 0;
        }  
        this->_connectedPin = connectedPin;
        return 1;
    }

    /**
     * reads the sensor if ready
     */
    boolean readSensor()
    {
        if (isnan(this->_connectedPin))
        {
            DEBUG_PRINTLN("SensorDHT22 has no configured Pin");
            return 0;
        }

        uint8_t resp[40];
        uint8_t result[5];

        uint32_t currentTime = millis();
        if ((currentTime - this->_lastTimeReaded) < 2000)
        {
            DEBUG_PRINTLN("SensorDHT22 readed between 2s twice");
            return 0;
        }
        this->_lastTimeReaded = currentTime;

        // reset data array
        result[0] = result[1] = result[2] = result[3] = result[4] = 0;

        // set output und send signal to start
        pinMode(this->_connectedPin, OUTPUT);
        digitalWrite(this->_connectedPin, HIGH);
        delay(300);
        digitalWrite(this->_connectedPin, LOW);
        delay(25);

        digitalWrite(this->_connectedPin, HIGH);
        delayMicroseconds(40);

        pinMode(this->_connectedPin, INPUT_PULLUP);
        delayMicroseconds(40);

        // Time critical operation, using interrupts
        pulseInLong(this->_connectedPin, HIGH, 1000); // first up is not needed
        for (int i = 0; i < 40; i += 1)
        {
            resp[i] = pulseInLong(this->_connectedPin, HIGH, 1000);
        }

        // convert response times to binary data
        for (int i = 0; i < 40; ++i)
        {
            result[i / 8] <<= 1;
            if (resp[i] == 0)
            {
                DEBUG_PRINTLN("SensorDHT22 error reading packet");
                return 0; // error reading packets
            }
            if (resp[i] > 60)
            {
                result[i / 8] |= 1;
            }
        }

        // check parity bit
        if (result[4] == ((result[0] + result[1] + result[2] + result[3]) & 0xFF))
        {
            DEBUG_PRINTLN("SensorDHT22 Parity Bit OK");
            this->_calcTemperature(result[2], result[3]);
            this->_calcHumidity(result[0], result[1]);
            return 1;
        }
        return 0;       
    }

    float getTemperature()
    {
        return this->temperature;
    }

    float getHumidity()
    {
        return this->humidity;
    }
};

#endif
