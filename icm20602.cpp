#include <FC/icm20602.h>

/* =========================
 * Constructor
 * ========================= */
ICM20602::ICM20602() {}

/* =========================
 * Device Detection
 * ========================= */
ICM20602_StatusCode_t ICM20602::detect() {
    ICM20602_StatusCode_t status = ICM20602_Detect();

    if (status != ICM20602_STATUS_OK)
    {
    	_detected = false;
    }

    _detected = true;
    return status;
}

/* =========================
 * Initialization
 * ========================= */
ICM20602_StatusCode_t ICM20602::initialize(ICM20602_Config_t config) {
	_config = config;

	if(!_detected) return ICM20602_ERR_NOT_DETECTED;

    ICM20602_StatusCode_t status = ICM20602_Init(_config.CS_GPIO_PORT, _config.CS_GPIO_PIN);

    if(status != ICM20602_STATUS_OK) return status;

    status = ICM20602_WriteRegister(
        ICM20602_REG_CONFIG,
        static_cast<uint8_t>(_config.DLPF_CFG)
    );

    if(status != ICM20602_STATUS_OK) return status;

    status = ICM20602_WriteRegister(
        ICM20602_REG_GYRO_CONFIG,
        static_cast<uint8_t>(_config.FS_SEL) << 3
    );

    if(status != ICM20602_STATUS_OK) return status;

    status = ICM20602_WriteRegister(
        ICM20602_REG_ACCEL_CONFIG,
        static_cast<uint8_t>(_config.ACCEL_FS_SEL) << 3
    );

    if(status != ICM20602_STATUS_OK) return status;

    status = ICM20602_WriteRegister(
        ICM20602_REG_ACCEL_CONFIG_2,
        static_cast<uint8_t>(_config.ACCEL_DLPF_CFG)
    );

    if(status != ICM20602_STATUS_OK) return status;

    _initialized = true;
    return status;
}

/* =========================
 * Verification
 * ========================= */
ICM20602_StatusCode_t ICM20602::verify() {
    if (!_initialized) return ICM20602_ERR_NOT_INITIALIZED; // Requries detected to also be true

    ICM20602_StatusCode_t status = ICM20602_Verify();

    if(status != ICM20602_STATUS_OK) return status;

    _verified = true;
    return status;
}

/* =========================
 * Calibration
 * ========================= */
ICM20602::ICM20602_Calibration_t ICM20602::calibrate(uint16_t samples, uint32_t ms) {
    if (!_initialized)
    {
    	ICM20602_Calibration_t calibration;
    	calibration.calibrated = false;
    	calibration.lastStatus = ICM20602_ERR_NOT_INITIALIZED;
    	return calibration;
    }

    LSB gyroX{0}, gyroY{0}, gyroZ{0};
    LSB accelX{0}, accelY{0}, accelZ{0};

    for (uint16_t i = 0; i < samples; i++) {
        ICM20602_StatusCode_t status = ICM20602_ReadRawData(&_truth);

        gyroX = gyroX + LSB(_truth.gyro_x);
        gyroY = gyroY + LSB(_truth.gyro_y);
        gyroZ = gyroZ + LSB(_truth.gyro_z);

        accelX = accelX + LSB{_truth.accel_x};
        accelY = accelY + LSB{_truth.accel_y};
        accelZ = accelZ + LSB{_truth.accel_z};

        _lastCalibration.lastStatus = status;
        _lastCalibration.calibrated = false;

        HAL_Delay(ms); // Blocking
    }

    _lastCalibration.gyroOffsetX = gyroX.value() / samples;
    _lastCalibration.gyroOffsetY = gyroY.value() / samples;
    _lastCalibration.gyroOffsetZ = gyroZ.value() / samples;

    _lastCalibration.accelOffsetX = accelX.value() / samples;
    _lastCalibration.accelOffsetY = accelY.value() / samples;
    _lastCalibration.accelOffsetZ = 0.0f;

    _lastCalibration.calibrated = true;
    return _lastCalibration;
}

ICM20602::ICM20602_Calibration_t ICM20602::runtimeCalibrate(uint16_t samples, uint32_t ms) {
	if (!_initialized)
	{
		ICM20602_Calibration_t calibration;
		calibration.calibrated = false;
		calibration.lastStatus = ICM20602_ERR_NOT_INITIALIZED;
		return calibration;
	}

    uint32_t now = HAL_GetTick();

    // If calibrated once, reset state for the next one
    if(!_calibrating)
    {
        lastCalibrationTick = now;
        calibrationSampleStep = 0;

        _gyroX = LSB{0};
        _gyroY = LSB{0};
        _gyroZ = LSB{0};

        _accelX = LSB{0};
        _accelY = LSB{0};
        _accelZ = LSB{0};

        _lastCalibration = ICM20602_Calibration_t{};
        _lastCalibration.lastStatus = ICM20602_STATUS_OK;
        _lastCalibration.calibrated = false;

        _calibrating = true;
    }

    if(calibrationSampleStep < samples) {
    	if((now - lastCalibrationTick) >= ms) {
    		lastCalibrationTick = now;

			ICM20602_StatusCode_t status = ICM20602_ReadRawData(&_truth);

			_gyroX = _gyroX + LSB(_truth.gyro_x);
			_gyroY = _gyroY + LSB(_truth.gyro_y);
			_gyroZ = _gyroZ + LSB(_truth.gyro_z);

			_accelX = _accelX + LSB{_truth.accel_x};
			_accelY = _accelY + LSB{_truth.accel_y};
			_accelZ = _accelZ + LSB{_truth.accel_z};

			_lastCalibration.calibrated = false;
			_lastCalibration.lastStatus = status;

			calibrationSampleStep++;
    	}
    } else {
    	_calibrating = false;

    	_lastCalibration.gyroOffsetX = _gyroX.value() / samples;
    	_lastCalibration.gyroOffsetY = _gyroY.value() / samples;
    	_lastCalibration.gyroOffsetZ = _gyroZ.value() / samples;

    	_lastCalibration.accelOffsetX = _accelX.value() / samples;
    	_lastCalibration.accelOffsetY = _accelY.value() / samples;
    	_lastCalibration.accelOffsetZ = 0.0f;

    	_lastCalibration.calibrated = true;
    }

    return _lastCalibration;
}

/* =========================
 * Read Sensor
 * ========================= */
ICM20602::ICM20602_Measurement_t ICM20602::read() {
    if (!_initialized) // Also potentially means not detected
    {
        return ICM20602_Measurement_t{
        	.status = ICM20602_ERR_NOT_INITIALIZED,

            .raw = ICM20602::ICM20602_Measurement_t::Raw {
                .accelX = LSB{0},
                .accelY = LSB{0},
                .accelZ = LSB{0},
                .gyroX = LSB{0},
                .gyroY = LSB{0},
                .gyroZ = LSB{0},
                .temperature = 0
            },

            .si = ICM20602::ICM20602_Measurement_t::SI {
                .aXRaw = MeterPerSecondSquared{0},
                .aYRaw = MeterPerSecondSquared{0},
                .aZRaw = MeterPerSecondSquared{0},
                .aX = MeterPerSecondSquared{0},
                .aY = MeterPerSecondSquared{0},
                .aZ = MeterPerSecondSquared{0},
                .omegaXRaw = RadiansPerSecond{0},
                .omegaYRaw = RadiansPerSecond{0},
                .omegaZRaw = RadiansPerSecond{0},
                .omegaX = RadiansPerSecond{0},
                .omegaY = RadiansPerSecond{0},
                .omegaZ = RadiansPerSecond{0},
                .temperature = 0
            }
        };
    }

    ICM20602_StatusCode_t status = ICM20602_ReadRawData(&_truth);
    if(status != ICM20602_STATUS_OK) return _lastMeasurement;

    _lastMeasurement = ICM20602_Measurement_t {
    	.status = status,

        .raw = ICM20602::ICM20602_Measurement_t::Raw {
            .accelX = LSB{_truth.accel_x},
            .accelY = LSB{_truth.accel_y},
            .accelZ = LSB{_truth.accel_z},
            .gyroX = LSB{_truth.gyro_x},
            .gyroY = LSB{_truth.gyro_y},
            .gyroZ = LSB{_truth.gyro_z},
            .temperature = _truth.temp
        },

        .si = ICM20602::ICM20602_Measurement_t::SI {
            .aXRaw = LSB{_truth.accel_x}.toMS2(A_FS_SEL_VAL(_config.ACCEL_FS_SEL)),
            .aYRaw = LSB{_truth.accel_y}.toMS2(A_FS_SEL_VAL(_config.ACCEL_FS_SEL)),
            .aZRaw = LSB{_truth.accel_z}.toMS2(A_FS_SEL_VAL(_config.ACCEL_FS_SEL)),

            .aX = LSB{static_cast<int32_t>((static_cast<float>(_truth.accel_x) - _lastCalibration.accelOffsetX))}.toMS2(A_FS_SEL_VAL(_config.ACCEL_FS_SEL)),
            .aY = LSB{static_cast<int32_t>((static_cast<float>(_truth.accel_y) - _lastCalibration.accelOffsetY))}.toMS2(A_FS_SEL_VAL(_config.ACCEL_FS_SEL)),
            .aZ = LSB{static_cast<int32_t>((static_cast<float>(_truth.accel_z) - _lastCalibration.accelOffsetZ))}.toMS2(A_FS_SEL_VAL(_config.ACCEL_FS_SEL)),

            .omegaXRaw = LSB{_truth.gyro_x}.toRadPS(FS_SEL_VAL(_config.FS_SEL)),
            .omegaYRaw = LSB{_truth.gyro_y}.toRadPS(FS_SEL_VAL(_config.FS_SEL)),
            .omegaZRaw = LSB{_truth.gyro_z}.toRadPS(FS_SEL_VAL(_config.FS_SEL)),

            .omegaX = LSB{static_cast<int32_t>((static_cast<float>(_truth.gyro_x) - _lastCalibration.gyroOffsetX))}.toRadPS(FS_SEL_VAL(_config.FS_SEL)),
            .omegaY = LSB{static_cast<int32_t>((static_cast<float>(_truth.gyro_y) - _lastCalibration.gyroOffsetY))}.toRadPS(FS_SEL_VAL(_config.FS_SEL)),
            .omegaZ = LSB{static_cast<int32_t>((static_cast<float>(_truth.gyro_z) - _lastCalibration.gyroOffsetZ))}.toRadPS(FS_SEL_VAL(_config.FS_SEL)),

            .temperature = _truth.temp
        }
    };

    return _lastMeasurement;
}

/* =========================
 * Helpers
 * ========================= */
float ICM20602::FS_SEL_VAL(_FS_SEL fs) {
    switch (fs) {
        case DPS250:  return 131.0f;
        case DPS500:  return 65.5f;
        case DPS1000: return 32.8f;
        case DPS2000: return 16.4f;
        default:      return FS_SEL_VAL(_config.FS_SEL);
    }
}

float ICM20602::A_FS_SEL_VAL(_ACCEL_FS_SEL fs) {
    switch (fs) {
        case G2:  return 16384.0f;
        case G4:  return 8192.0f;
        case G8:  return 4096.0f;
        case G16: return 2048.0f;
        default:  return A_FS_SEL_VAL(_config.ACCEL_FS_SEL);
    }
}
