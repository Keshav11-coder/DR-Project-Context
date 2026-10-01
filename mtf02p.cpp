/*
 * mtf02p.cpp
 *
 *  Created on: Sep 14, 2026
 *      Author: anonymous
 */

#include <FC/mtf02p.h>


MTF02P_StatusCode_t MTF02P::initialize(MTF02P_Config_t config)
{
    const MTF02P_StatusCode_t status = MTF02P_Init(config.GPIO_PIN);

    _config = config;

    return status;
}

MTF02P::MTF02P_Diagnostic_t MTF02P::verify(uint16_t period, uint32_t requiredValidFrames)
{
	/* Initial check (initialized or not) */
	if(MTF02P_ReadRawData(&_truth) == MTF02P_ERR_NOT_INITIALIZED)
	{
		MTF02P::MTF02P_Diagnostic_t diagnostic_;
		diagnostic_.requiredFrames = requiredValidFrames;
		diagnostic_.validFrames = 0;
		diagnostic_.invalidFrames = 0;
		diagnostic_.complete = false;
		diagnostic_.lastStatus = MTF02P_ERR_NOT_INITIALIZED;
		return diagnostic_;
	}

	uint32_t now = HAL_GetTick();

	_lastDiagnostic.requiredFrames = requiredValidFrames;

	if (!_statistics_started) {
		_statistics_start = MTF02P_Statistics();
		_statistics_started = true;
		_lastDiagnostic.complete = false;
		_lastDiagnostic.validFrames = 0;
		_lastDiagnostic.invalidFrames = 0;
		_lastTick = now;
	}

	MTF02P_StatusCode_t status = MTF02P_ReadRawData(&_truth);

	if((now - _lastTick) >= period)
	{
		MTF02P_Statistics_t statistics_end = MTF02P_Statistics();

		_lastDiagnostic.validFrames = statistics_end.frames_ok - _statistics_start.frames_ok;
		_lastDiagnostic.invalidFrames = (statistics_end.frames_bad_crc - _statistics_start.frames_bad_crc) +
								   (statistics_end.frames_bad_size - _statistics_start.frames_bad_size);

		// Complete
		_lastDiagnostic.lastStatus = status;
		_lastDiagnostic.complete = true;

		_statistics_started = false;
		_lastTick = now;
	}

	return _lastDiagnostic;
}


MTF02P::MTF02P_Measurement_t MTF02P::read()
{
    // Local container to fetch the freshly parsed packet details
    MTF02P_Data_t freshData;
    const MTF02P_StatusCode_t status = MTF02P_ReadRawData(&freshData);

    // 1. If no fresh frame passed CRC on this tick, return our persistent cache
    if (status == MTF02P_ERR_NO_DATA)
    {
        _lastMeasurement.status = MTF02P_STATUS_OK; // Data is valid, just old
        return _lastMeasurement;
    }

    // 2. If a hard driver error happens, pass it out safely
    if (status != MTF02P_STATUS_OK)
    {
        _lastMeasurement.status = status;
        return _lastMeasurement;
    }

    // 3. A packet was successfully read! Update ONLY the matching subsystem fields
    if (freshData.function == MTF02P_RANGEFINDER)
    {
        _lastMeasurement.raw.range          = freshData.distance;
        _lastMeasurement.quality.rangeQuality = freshData.quality_distance;

        // Recalculate altitude metric
        _lastMeasurement.si.altitude = Meters{ static_cast<float>(freshData.distance) / 1000.0f };
    }
    else if (freshData.function == MTF02P_OPTICAL_FLOW)
    {
//        _lastMeasurement.raw.flowX          = freshData.motion_x;
//        _lastMeasurement.raw.flowY          = freshData.motion_y;
//        _lastMeasurement.quality.flowQuality = freshData.quality_motion;
//
//        // Recalculate planar velocity metrics using the latest known range matrix
//        float currentRangeM = static_cast<float>(_lastMeasurement.raw.range) / 1000.0f;
//
//        _lastMeasurement.si.vX = MetersPerSecond{ static_cast<float>(freshData.motion_x) * currentRangeM / 100.0f };
//        _lastMeasurement.si.vY = MetersPerSecond{ static_cast<float>(freshData.motion_y) * currentRangeM / 100.0f };

		_lastMeasurement.raw.flowX =
				freshData.motion_x;

		_lastMeasurement.raw.flowY =
				freshData.motion_y;

		_lastMeasurement.quality.flowQuality =
				freshData.quality_motion;
    }

    // 4. Ensure we filter out background configuration packet status codes
    _lastMeasurement.status = MTF02P_STATUS_OK;
    return _lastMeasurement;
}
