/**
 * @file sensors.hpp
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief This file contains the base class for all the sensors drivers.
 * @version 0.1
 *
 * @date 2025-10-29
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

constexpr uint32_t SPI_TIMEOUT_MS = 100;
constexpr double G =
    9.8214599692; // https://www.researchgate.net/publication/232822567_Absolute_gravity_values_in_Norway
constexpr double PI = 3.141592653589;

typedef unsigned long ULONG;

/**
 * @brief Base class for all sensors.
 *
 * @param[in] output_data_rate The frequency to read data from a sensor.
 */
template <typename T> class Sensor {
  public:
    Sensor() = default;
    
    virtual uint8_t init();
    virtual uint8_t read(T& data);
    
    virtual ~Sensor() = default;
  protected:
};

#endif