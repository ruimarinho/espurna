// -----------------------------------------------------------------------------
// VL53L1X Sensor over I2C
// Copyright (C) 2017-2019 by Xose Pérez <xose dot perez at gmail dot com>
// -----------------------------------------------------------------------------

#if SENSOR_SUPPORT && VL53L1X_SUPPORT

#pragma once

#include <Arduino.h>
#include "vl53l1x_class.h"

#include "I2CSensor.h"

class VL53L1XSensor : public I2CSensor<> {

    public:

        // ---------------------------------------------------------------------
        // Public
        // ---------------------------------------------------------------------

        VL53L1XSensor() {
            _count = 1;
            _sensor_id = SENSOR_VL53L1X_ID;
            _vl53l1x = new VL53L1X(&Wire, -1, -1);
        }

        ~VL53L1XSensor() {
          delete _vl53l1x;
        }

        // ---------------------------------------------------------------------

        void setDistanceMode(uint8_t mode) {
          if (_mode == mode) return;
          _mode = mode;
          _dirty = true;
        }

        void setMeasurementTimingBudget(uint32_t timing_budget) {
          if (_timing_budget == timing_budget) return;
          _timing_budget = timing_budget;
          _dirty = true;
        }

        void setInterMeasurementPeriod(unsigned int inter_measurement_period) {
          if (_inter_measurement_period == inter_measurement_period) return;
          _inter_measurement_period = inter_measurement_period;
          _dirty = true;
        }

        // ---------------------------------------------------------------------
        // Sensor API
        // ---------------------------------------------------------------------

        void begin() {
          if (!_dirty) {
            return;
          }

          if (_vl53l1x->VL53L1X_SensorInit() != 0) {
            #if SENSOR_DEBUG
              DEBUG_MSG("[VL53L1X] Unable to initialize sensor\n");
            #endif
            return;
          }

          _vl53l1x->VL53L1X_SetDistanceMode(_mode);
          _vl53l1x->VL53L1X_SetInterMeasurementInMs(_inter_measurement_period);
          _vl53l1x->VL53L1X_SetTimingBudgetInMs(_timing_budget);
          _vl53l1x->VL53L1X_StartRanging();

          _ready = true;
          _dirty = false;
        }

        // Descriptive name of the sensor
        String description() {
            char buffer[21];
            snprintf(buffer, sizeof(buffer), "VL53L1X @ I2C (0x%02X)", _address);
            return String(buffer);
        }

        // Descriptive name of the slot # index
        String description(unsigned char index) {
            return description();
        };

        // Type for slot # index
        unsigned char type(unsigned char index) {
            if (index == 0) return MAGNITUDE_DISTANCE;
            return MAGNITUDE_NONE;
        }

        // Pre-read hook (usually to populate registers with up-to-date data)
        void pre() {
          uint8_t dataReady;
          uint8_t rangeStatus;
          uint16_t distance;

          _vl53l1x->VL53L1X_CheckForDataReady(&dataReady);

          if (!(bool)dataReady) {
              return;
          }

          _vl53l1x->VL53L1X_GetRangeStatus(&rangeStatus);
          _vl53l1x->VL53L1X_GetDistance(&distance);
          _vl53l1x->VL53L1X_ClearInterrupt();

          if (rangeStatus > 0) {
            #if SENSOR_DEBUG
              DEBUG_MSG("[VL53L1X] Range error %u\n", rangeStatus);
            #endif
            _error = SENSOR_ERROR_OUT_OF_RANGE;
            return;
          }

          _distance = distance / 1000.00;
          _error = SENSOR_ERROR_OK;
        }

        // Current value for slot # index
        double value(unsigned char index) {
            if (index != 0) return 0;
            return _distance;
        }

    protected:

        VL53L1X *_vl53l1x = NULL;
        uint8_t _mode;
        uint16_t _inter_measurement_period;
        uint16_t _timing_budget;
        double _distance = 0;

};

#endif // SENSOR_SUPPORT && VL53L1X_SUPPORT
