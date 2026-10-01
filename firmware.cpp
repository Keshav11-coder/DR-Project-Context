/*
 * main.cpp
 *
 *  Created on: May 29, 2026
 *      Author: Dmitry (Dave) Camovich
 */

#include "main.h"

/* These are plain C files/variables */
extern "C" {
    void cpp_main(void);

    // C external C global variables
    extern volatile uint8_t rx_flag;

    // Custom C drivers here so C++ can see them
    #include "usbd_cdc_if.h"

    // Standard C library features used by C wrappers
    #include <stdio.h>
}

#include <FC/types.h>

#include <FC/scheduler.h>

#include <FC/mahony.h>

#include <FC/icm20602.h>
#include <FC/mtf02p.h>

//#include "mtf02p_uart_probe.h"

#include <FC/Cascade/orientation_controller.h>
#include <FC/Cascade/rate_controller.h>

#include <indicator.h>

#include "dshot_probe.h"

//#define MTF02P_ENABLED

enum RuntimeMode {
	FAILSAFE,
	BDEBUG,
	RDEBUG,
	IMU_CALIBRATE,
	OFR_VERIFY
};

struct RuntimeContext
{
    RuntimeMode mode;
    Scheduler<7>::SchedulerFault lastFault;
};

struct ControlContext
{
    RateControlRequest rateControlRequest;
    OrientationControlRequest orientationControlRequest;
    Vector3<Body, NewtonMeters> rate_correction;

    /* % TEMPORARY % */
    MTF02P::MTF02P_Measurement_t ofrMeasurement;
    ICM20602::ICM20602_Measurement_t icmMeasurement;
    Microseconds now;
};

class AttitudeEstimationTask : public IScheduledTask
{
public:
	AttitudeEstimationTask(ControlContext& context, ICM20602& imu, MahonyFilter& filter) : context_(context), imu_(imu), filter_(filter) {}

	void update(Microseconds dt) override
	{
		const ICM20602::ICM20602_Measurement_t measurement = imu_.read();

		if(measurement.status != ICM20602_STATUS_OK) // Status is not OK, but keep pushing the last state and deal with the error later in fail safe.
		{
			// Ideally send it to a FailContext, from which an additional FailSafe task reads and handles separately.
			// FailSafe task should have authorization to modify RuntimeContext.

			__ICM20602_Handle_ErrCode(measurement.status);
		}

		context_.rateControlRequest.measured = Vector3<Body, RadiansPerSecond>{
			.x = measurement.si.omegaX,
			.y = measurement.si.omegaY,
			.z = measurement.si.omegaZ
		};

		/* % TEMPORARY % */
		context_.icmMeasurement = measurement;
		context_.now = Microseconds{HAL_GetMicros()};

		context_.orientationControlRequest.measured =
			filter_.update(
				{
					.acceleration = Vector3<Body, MeterPerSecondSquared> {
						.x = measurement.si.aX,
						.y = measurement.si.aY,
						.z = measurement.si.aZ
					},

					.angularVelocity = Vector3<Body, RadiansPerSecond> {
						.x = measurement.si.omegaX,
						.y = measurement.si.omegaY,
						.z = measurement.si.omegaZ
					}
				},
				dt.toSeconds()
			);
	}

private:
	ControlContext& context_;

	ICM20602& imu_;
	MahonyFilter& filter_;
};

class MTF02PObserver : public IScheduledTask
{
public:
	MTF02PObserver(ControlContext& context, MTF02P& ofr) : context_(context), ofr_(ofr) {}

	void update(Microseconds dt) override
	{
		MTF02P::MTF02P_Measurement_t measurement = ofr_.read();

		const MTF02P_StatusCode_t status = measurement.status;

		if(status != MTF02P_STATUS_OK) {
			__MTF02P_Handle_ErrCode(status);
//			context_.ofrMeasurement = MTF02P::MTF02P_Measurement_t{};
			context_.ofrMeasurement.status = status;
			return;
		}

		context_.ofrMeasurement = measurement;
	}

private:
	ControlContext& context_;

	MTF02P& ofr_; // Optical Flow Range Finder
};

class OrientationControlTask : public IScheduledTask
{
public:
	OrientationControlTask(ControlContext& context, const OrientationPIDGains configuration)
						: context_(context),
						  configuration_(configuration),
						  controller_(OrientationController(configuration)) {}

	void update(Microseconds dt) override
	{
		context_.rateControlRequest.target = controller_.update(context_.orientationControlRequest, dt.toSeconds());
	}

private:
	ControlContext& context_;

	const OrientationPIDGains configuration_;
	OrientationController controller_;
};

class RateControlTask : public IScheduledTask
{
public:
	struct RatePIDConfiguration
	{
		PIDGains gainsX;
		PIDGains gainsY;
		PIDGains gainsZ;
	};

	RateControlTask(ControlContext& context, const RatePIDConfiguration configuration) : context_(context), configuration_(configuration)
	{
		controller_ = TorqueRateController(
			PIDCorrectionPolicy(configuration.gainsX),
			PIDCorrectionPolicy(configuration.gainsY),
			PIDCorrectionPolicy(configuration.gainsZ)
		);
	}

	void update(Microseconds dt) override
	{
		const Vector3<Body, NewtonMeters> correction = controller_.update(context_.rateControlRequest, dt.toSeconds());
		context_.rate_correction = correction;
	}

private:
	TorqueRateController controller_ = TorqueRateController(
		PIDCorrectionPolicy(PIDGains{}),
		PIDCorrectionPolicy(PIDGains{}),
		PIDCorrectionPolicy(PIDGains{})
	);

	ControlContext& context_;
	const RatePIDConfiguration configuration_;
};

class LedTask : public IScheduledTask
{
public:
    void update(Microseconds dt) override
    {
    	indicator.update(HAL_GetTick(),
			Indicator::On{ .deltat=Milliseconds{100}, .frequency=1 },
			Indicator::Off{ .deltat=Milliseconds{1900} });
    }

    Indicator indicator;
};

class DebugTask : public IScheduledTask
{
public:
	enum DebugMode {
		LOG_ATT,
		LOG_RAT,
		LOG_RNG,
		LOG_OFL,
		LOG_OFL_RAT
	};

	DebugTask(const ControlContext& context, const DebugMode debugMode) : debugMode_(debugMode), context_(context) {}

    void update(Microseconds dt) override
    {
        Quaternion<World, Body> orientation = context_.orientationControlRequest.measured;
        Quaternion<World, Body> target_o = context_.orientationControlRequest.target;
        Vector3<Body, RadiansPerSecond> corr_o = context_.rateControlRequest.target;

        Vector3<Body, RadiansPerSecond> rate = context_.rateControlRequest.measured;
        Vector3<Body, NewtonMeters> corr_r = context_.rate_correction;

        switch (debugMode_) {
			case LOG_ATT: printf("ATT SP [%7.3f %7.3f %7.3f | w: %7.3f] MEAS [%7.3f %7.3f %7.3f | w: %7.3f] OUT [%7.3f %7.3f %7.3f]\r\n",
							target_o.x, target_o.y, target_o.z, target_o.w,
							orientation.x, orientation.y, orientation.z, orientation.w,
							corr_o.x.value(), corr_o.y.value(), corr_o.z.value()); break;

			case LOG_RAT: printf("RATE SP [%7.3f %7.3f %7.3f] MEAS [%7.3f %7.3f %7.3f] OUT [%7.3f %7.3f %7.3f]\r\n",
							corr_o.x.value(), corr_o.y.value(), corr_o.z.value(),
							rate.x.value(), rate.y.value(), rate.z.value(),
							corr_r.x.value(), corr_r.y.value(), corr_r.z.value()); break;
			case LOG_RNG:
				if(context_.ofrMeasurement.status == MTF02P_ERR_INVALID_RANGE) {
					break; // Ignore bad data for clarity
				}

				printf("RANGE [quality=%3u  distance=%6f m (%s)]\r\n",
                    context_.ofrMeasurement.quality.rangeQuality,
					context_.ofrMeasurement.si.altitude.value(),
                    "OK");

				break;
			case LOG_OFL:
				if(context_.ofrMeasurement.status != MTF02P_STATUS_OK) {
					break; // Ignore bad data for clarity
				}

//				printf("PLANAR FLOW [quality=%3u  vX=%6f m/s vY=%6f m/s (%s)]\r\n",
//					context_.ofrMeasurement.quality.flowQuality,
//					context_.ofrMeasurement.si.vX.value(),
//					context_.ofrMeasurement.si.vY.value(),
//					"OK");

				printf("PLANAR FLOW [quality=%3u  flowX=%ld flowY=%ld (%s)]\r\n",
					context_.ofrMeasurement.quality.flowQuality,
					context_.ofrMeasurement.raw.flowX,
					context_.ofrMeasurement.raw.flowY,
					"OK");

				break;

			case LOG_OFL_RAT:
				if(context_.icmMeasurement.status != ICM20602_STATUS_OK) {
					break; // Ignore bad data for clarity
				}

				if(context_.ofrMeasurement.status != MTF02P_STATUS_OK) {
					break; // Ignore bad data for clarity
				}

				printf("Omega [x=%6f y=%6f] | Flow [x=%ld y=%ld] | t_us:%ld (%s)]\r\n",
					context_.icmMeasurement.si.omegaX.value(),
					context_.icmMeasurement.si.omegaY.value(),
					context_.ofrMeasurement.raw.flowX,
					context_.ofrMeasurement.raw.flowY,
					context_.now.value(),
					"[PCK_OK]");

				break;
        }
    }

private:
    DebugMode debugMode_ = LOG_RNG;
    const ControlContext& context_;
};

void runtimeFaultHandler(
    void* context,
    const Scheduler<7>::SchedulerFault& fault)
{
    RuntimeContext* runtime =
        static_cast<RuntimeContext*>(context);

    runtime->mode = RuntimeMode::FAILSAFE;
    runtime->lastFault = fault;
}

void cpp_main(void) {
	HAL_Delay(5000);

//	dshot_probe();
//	return;

//	mtf02p_uart_probe();
//	return;

	ICM20602 imu;
	MTF02P ofr;

	MahonyFilter filter(Quaternion<World, Body>(), 12.0f, 0.1f);

	Scheduler<7> runtimeSequence;

	ControlContext controlContext;
	controlContext.orientationControlRequest.target = Quaternion<World, Body>();

	RuntimeContext runtimeContext = RuntimeContext{
		.mode = RuntimeMode::BDEBUG
	};

	runtimeSequence.onFault(
		&runtimeContext,
		runtimeFaultHandler);

	AttitudeEstimationTask attitudeEstimationTask(controlContext, imu, filter);

	OrientationControlTask orientationControlTask(controlContext, OrientationPIDGains{ .kP_X=4.5f, .kP_Y=4.5f, .kP_Z=2.8f });

	RateControlTask rateControlTask(controlContext, RateControlTask::RatePIDConfiguration{
		.gainsX = PIDGains{
            .kp = 0.5f,
            .ki = 0.02f,
            .kd = 0.0f,
            .i_limit = 1.0f,
            .output_min = -1.0f,
            .output_max = 1.0f
        },

        .gainsY = PIDGains{
            .kp = 0.5f,
            .ki = 0.02f,
            .kd = 0.0f,
            .i_limit = 1.0f,
            .output_min = -1.0f,
            .output_max = 1.0f
        },

        .gainsZ = PIDGains{
            .kp = 0.5f,
            .ki = 0.02f,
            .kd = 0.0f,
            .i_limit = 1.0f,
            .output_min = -1.0f,
            .output_max = 1.0f
        }
	});

	LedTask ledTask;
	DebugTask debugTask(controlContext, DebugTask::LOG_ATT);

	MTF02PObserver mtf02pObserver(controlContext, ofr);

	imu.detect();
	imu.initialize({
		.FS_SEL = ICM20602::DPS500,
		.ACCEL_FS_SEL = ICM20602::G8,

		.DLPF_CFG = ICM20602::FCHOICE_B2,
		.ACCEL_DLPF_CFG = ICM20602::ACCEL_FCHOICE_B3,

		.CS_GPIO_PORT = GPIOA,
		.CS_GPIO_PIN = GPIO_PIN_4
	});
	const ICM20602_StatusCode_t imuVerifyStatus = imu.verify(); // Check error once, verify() covers all 3: detect, initialize, verify.

	const MTF02P_StatusCode_t ofrInitStatus = ofr.initialize({
		.GPIO_PIN = (GPIO_PIN_9 | GPIO_PIN_10),

		.RANGE_MM_MIN = 20,
		.RANGE_MM_MAX = 6000,
	});

	bool imuCalibrated = false;
	bool ofrVerified = false;

	runtimeSequence.add(attitudeEstimationTask, Microseconds{1000});
	runtimeSequence.add(orientationControlTask, Microseconds{4000});
	runtimeSequence.add(rateControlTask, Microseconds{1000});
	runtimeSequence.add(ledTask, Microseconds{1000}); // Non-blocking task, continuous fine
	runtimeSequence.add(mtf02pObserver, Microseconds{20000});

	runtimeSequence.add(debugTask, Microseconds{20000});

	while(true) {
		static bool failed = false;
		switch(runtimeContext.mode){
			case RDEBUG:
				runtimeSequence.tick();
				break;

			case BDEBUG:
				if(imuVerifyStatus != ICM20602_STATUS_OK){
				    runtimeContext.mode = FAILSAFE;
				    break;
				}

				if(!imuCalibrated){
				    runtimeContext.mode = IMU_CALIBRATE;
				    break;
				}

#if defined(MTF02P_ENABLED)
				if(ofrInitStatus != MTF02P_STATUS_OK) {
					runtimeContext.mode = FAILSAFE;
					break;
				}

				if(!ofrVerified) {
					runtimeContext.mode = OFR_VERIFY;
					break;
				}
#endif

				runtimeContext.mode = RDEBUG;

				ledTask.indicator.update(HAL_GetTick(),
					Indicator::On{ .deltat=Milliseconds{500}, .frequency=1 },
					Indicator::Off{ .deltat=Milliseconds{500} });
				break;

			case IMU_CALIBRATE:
				if(imuCalibrated == false)
				{
					ICM20602::ICM20602_Calibration_t imuCalibration = imu.runtimeCalibrate(1000, 5);
					if(imuCalibration.calibrated) imuCalibrated = true;
				}

				if(imuCalibrated == true) runtimeContext.mode = BDEBUG;

				ledTask.indicator.update(HAL_GetTick(),
					Indicator::On{ .deltat=Milliseconds{250}, .frequency=1 },
					Indicator::Off{ .deltat=Milliseconds{250} });
				break;

			case RuntimeMode::OFR_VERIFY:
			{
				static MTF02P::MTF02P_Diagnostic_t diagnostic;

				if(ofrVerified) {
					runtimeContext.mode = BDEBUG;
					break;
				}

				if(!ofrVerified)
				{
					diagnostic = ofr.verify(500, 20); // During 500ms, does the driver collect adequate OK frames (20), and is that number lower than the invalid frames.

					if(diagnostic.complete) // Diagnostic is valid and has completed
					{
						if(diagnostic.validFrames > diagnostic.invalidFrames) { // Valid outweighs invalid
							ofrVerified = true;
							break;
						} else if (diagnostic.validFrames <= diagnostic.invalidFrames) { // Invalid outweighs valid, go to failsafe
							ofrVerified = false;
							runtimeContext.mode = FAILSAFE;
							break;
						}
					}
				}

				ledTask.indicator.update(HAL_GetTick(),
					Indicator::On{ .deltat=Milliseconds{250}, .frequency=1 },
					Indicator::Off{ .deltat=Milliseconds{250} });
				break;
			}

			case FAILSAFE:
				ledTask.indicator.update(HAL_GetTick(),
					Indicator::On{ .deltat=Milliseconds{100}, .frequency=5 },
					Indicator::Off{ .deltat=Milliseconds{2000} });
				if(!failed) {
					printf(
					    "TASK %d OVERRUN: budget=%lu us runtime=%lu us overrun=%lu us\r\n",
					    runtimeContext.lastFault.task,
					    static_cast<unsigned long>(runtimeContext.lastFault.budget.value()),
					    static_cast<unsigned long>(runtimeContext.lastFault.runtime.value()),
					    static_cast<unsigned long>(runtimeContext.lastFault.overrun.value())
					);

					failed = true;
				}
				break;

			default:
				runtimeContext.mode = FAILSAFE;
				break;
		}
	}
}
