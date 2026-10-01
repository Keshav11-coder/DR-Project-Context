/*
 * mtf02p.h
 *
 *  Created on: Sep 14, 2026
 *      Author: anonymous
 */

#ifndef INC_FC_MTF02P_H_
#define INC_FC_MTF02P_H_

extern "C" {
    #include "mtf02p_HAL.h"
}

#include <FC/types.h>

class MTF02P {
public:
    struct MTF02P_Config_t {
        uint32_t GPIO_PIN;

        int32_t RANGE_MM_MIN;
        int32_t RANGE_MM_MAX;
    };

    struct MTF02P_Diagnostic_t {
    	uint32_t requiredFrames = 0;
    	uint32_t validFrames = 0;
    	uint32_t invalidFrames = 0;

    	bool complete = false;

    	MTF02P_StatusCode_t lastStatus;
    };

    struct MTF02P_Measurement_t {
        MTF02P_StatusCode_t status;

        struct Raw {
            int32_t flowX;
            int32_t flowY;

            int32_t range;
        } raw;

        struct SI {
            MetersPerSecond vX;
            MetersPerSecond vY;

            Meters altitude = Meters{0};
        } si;

        struct Quality {
            uint8_t flowQuality = 0;
            uint8_t rangeQuality = 0;
        } quality;
    };

    MTF02P() : _lastTick() {}

    MTF02P_StatusCode_t initialize(
        MTF02P_Config_t config = {
            .GPIO_PIN = (GPIO_PIN_9 | GPIO_PIN_10),

            .RANGE_MM_MIN = 20,
            .RANGE_MM_MAX = 6000
        }
    );

    MTF02P::MTF02P_Diagnostic_t verify(uint16_t period, uint32_t requiredValidFrames);

    MTF02P_Measurement_t read();

private:
    MTF02P_Data_t _truth;
    MTF02P_Config_t _config;

    uint32_t _lastTick;
    MTF02P_Diagnostic_t _lastDiagnostic = MTF02P_Diagnostic_t {
		.requiredFrames = 0,
		.validFrames = 0,
		.invalidFrames = 0,

		.complete = false,

		.lastStatus = MTF02P_STATUS_OK
	};
    MTF02P_Statistics_t _statistics_start;
    bool _statistics_started = false;

    MTF02P_Measurement_t _lastMeasurement;
};

#endif /* INC_FC_MTF02P_H_ */
