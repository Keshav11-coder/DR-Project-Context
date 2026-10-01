/*
 * icm20602.h
 *
 *  Created on: Jun 6, 2026
 *      Author: anonymous
 */

#ifndef INC_ICM20602_H_
#define INC_ICM20602_H_

extern "C" {
    #include "icm20602_HAL.h"
}

#include "types.h"

class ICM20602 {
public:
    ICM20602();

    enum _FS_SEL {
        DPS250 = 0b00,
        DPS500 = 0b01,
        DPS1000 = 0b10,
        DPS2000 = 0b11
    };

    enum _ACCEL_FS_SEL {
        G2  = 0b00,
        G4  = 0b01,
        G8  = 0b10,
        G16 = 0b11
    };

    enum _DLPF_CFG {
        FCHOICE_B0 = 0,
        FCHOICE_B1 = 1,
        FCHOICE_B2 = 2,
        FCHOICE_B3 = 3,
        FCHOICE_B4 = 4,
        FCHOICE_B5 = 5,
        FCHOICE_B6 = 6,
        FCHOICE_B7 = 7
    };

    enum _ACCEL_DLPF_CFG {
        ACCEL_FCHOICE_B0 = 0,
        ACCEL_FCHOICE_B1 = 1,
        ACCEL_FCHOICE_B2 = 2,
        ACCEL_FCHOICE_B3 = 3,
        ACCEL_FCHOICE_B4 = 4,
        ACCEL_FCHOICE_B5 = 5,
        ACCEL_FCHOICE_B6 = 6,
        ACCEL_FCHOICE_B7 = 7
    };

    struct ICM20602_Config_t {
        _FS_SEL FS_SEL = DPS500;
        _ACCEL_FS_SEL ACCEL_FS_SEL = G8;

        _DLPF_CFG DLPF_CFG = FCHOICE_B2;
        _ACCEL_DLPF_CFG ACCEL_DLPF_CFG = ACCEL_FCHOICE_B3;

        GPIO_TypeDef* CS_GPIO_PORT = GPIOA;
        uint16_t CS_GPIO_PIN = GPIO_PIN_4;
    };

    struct ICM20602_Measurement_t {
    	ICM20602_StatusCode_t status;

        struct Raw {
            LSB accelX;
            LSB accelY;
            LSB accelZ;

            LSB gyroX;
            LSB gyroY;
            LSB gyroZ;

            int16_t temperature;
        } raw;

        struct SI {
            MeterPerSecondSquared aXRaw;
            MeterPerSecondSquared aYRaw;
            MeterPerSecondSquared aZRaw;

            MeterPerSecondSquared aX;
            MeterPerSecondSquared aY;
            MeterPerSecondSquared aZ;

            RadiansPerSecond omegaXRaw;
            RadiansPerSecond omegaYRaw;
            RadiansPerSecond omegaZRaw;

            RadiansPerSecond omegaX;
            RadiansPerSecond omegaY;
            RadiansPerSecond omegaZ;

            int16_t temperature;
        } si;
    };

    struct ICM20602_Calibration_t {
        float gyroOffsetX{0};
        float gyroOffsetY{0};
        float gyroOffsetZ{0};

        float accelOffsetX{0};
        float accelOffsetY{0};
        float accelOffsetZ{0};

        bool calibrated = false;

        ICM20602_StatusCode_t lastStatus = ICM20602_STATUS_OK;
    };

    ICM20602_StatusCode_t detect();
    ICM20602_StatusCode_t initialize(ICM20602_Config_t config = {
        .FS_SEL = DPS500,
        .ACCEL_FS_SEL = G8,
        .DLPF_CFG = FCHOICE_B2,
        .ACCEL_DLPF_CFG = ACCEL_FCHOICE_B3,

		.CS_GPIO_PORT = GPIOA,
        .CS_GPIO_PIN = GPIO_PIN_4
    });

    ICM20602_StatusCode_t verify();

    ICM20602_Calibration_t calibrate(uint16_t samples, uint32_t ms);
    ICM20602_Calibration_t runtimeCalibrate(uint16_t samples, uint32_t ms);

    ICM20602_Measurement_t read();

private:
    ICM20602_Data_t _truth;
    ICM20602_Config_t _config;

    bool _initialized = false;
    bool _verified = false;
    bool _detected = false;

    ICM20602_Measurement_t _lastMeasurement = ICM20602_Measurement_t{
    	.status = ICM20602_ERR_INVALID_DATA,

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
    };;

    /* Calibration logistics */
    ICM20602_Calibration_t _lastCalibration;
    uint32_t lastCalibrationTick = 0;
    uint16_t calibrationSampleStep = 0;

    bool _calibrating = false;

    LSB _gyroX{0}, _gyroY{0}, _gyroZ{0};
    LSB _accelX{0}, _accelY{0}, _accelZ{0};

    float FS_SEL_VAL(_FS_SEL _fs);
    float A_FS_SEL_VAL(_ACCEL_FS_SEL _a_fs);
};

#endif /* INC_ICM20602_H_ */
