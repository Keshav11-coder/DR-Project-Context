/*
 * velkf.h
 *
 * Velocity Kalman Filter
 *
 * Purpose:
 *     Estimate planar Body-to-World velocity using:
 *
 *         - MTF02P raw optical-flow counts
 *         - MTF02P range measurement
 *         - MTF02P flow quality
 *         - calibrated IMU linear acceleration
 *         - calibrated IMU angular velocity
 *         - Mahony World -> Body attitude
 *
 * Frame convention:
 *
 *     Quaternion<From, To>
 *         transforms vectors From -> To.
 *
 *     Quaternion<Body, World>
 *         therefore represents Body -> World.
 *
 * Project-specific Body-axis convention:
 *
 *     +X = forward pitch direction
 *     +Y = left roll direction
 *     +Z = counter-clockwise yaw direction
 *
 * Optical-flow observations:
 *
 *     Body translation:
 *
 *         +X velocity -> flowX < 0
 *         -X velocity -> flowX > 0
 *
 *         +Y velocity -> flowY < 0
 *         -Y velocity -> flowY > 0
 *
 *     Body rotation:
 *
 *         +omegaX -> flowX rotational contribution > 0
 *         +omegaY -> flowY rotational contribution < 0
 *         omegaZ  -> flow contribution approximately 0
 *
 * Raw MTF02P flow interpretation:
 *
 *     motion_x / motion_y are treated as strongly-supported
 *     degrees*10000 integrated angular-flow measurements.
 *
 *     Therefore:
 *
 *         radians/count =
 *             PI / (180 * 10000)
 *
 * This protocol interpretation remains experimentally unverified.
 *
 * The calibration factors flowScaleX / flowScaleY are therefore
 * deliberately configurable and are NOT hidden inside the filter.
 *
 * No sensor parsing or driver-specific packet handling belongs here.
 */

#ifndef INC_FC_VELKF_H_
#define INC_FC_VELKF_H_

#include "types.h"

class VelocityKalmanFilter
{
public:

    struct Config
    {
        //
        // MTF02P raw-count -> angular-flow conversion.
        //
        // This is the currently strongly-supported protocol
        // interpretation. It is intentionally exposed as a constant
        // rather than treated as a proven physical fact.
        //
        static constexpr float FLOW_RADIANS_PER_COUNT =
            PI_F / (180.0f * 10000.0f);

        //
        // Empirical optical-flow calibration.
        //
        // Initial bench-test value:
        //
        //     1.0
        //
        // These parameters may later become approximately 4-6
        // if bench calibration establishes such a correction.
        //
        float flowScaleX = 1.0f;
        float flowScaleY = 1.0f;

        //
        // Nominal optical-flow integration interval.
        //
        // The current MTF02P interface does not expose a sensor
        // timestamp/integration duration. 20 ms corresponds to
        // the nominal 50 Hz sensor rate and is therefore a
        // configuration parameter, not a measured fact.
        //
        Seconds flowDt = Seconds{0.020f};

        //
        // Optical-flow validity.
        //
        uint8_t minimumFlowQuality = 100;

        float minimumRangeMeters = 0.02f;
        float maximumRangeMeters = 6.0f;

        //
        // Prediction process-noise parameters.
        //
        // Acceleration noise is expressed in m/s^2.
        // Bias random walk is expressed in m/s/sqrt(s).
        //
        float accelerationNoise = 1.0f;
        float velocityBiasNoise = 0.05f;

        //
        // Optical-flow velocity measurement noise.
        //
        float flowMeasurementNoiseX = 0.25f;
        float flowMeasurementNoiseY = 0.25f;

        //
        // World-frame gravity.
        //
        // The current project convention has +Z pointing in the
        // direction of the stationary accelerometer gravity
        // observation, therefore physical gravity is -Z.
        //
        float gravityWorldZ = -GRAVITY_F;

        //
        // Innovation gating.
        //
        // A non-positive value disables innovation rejection.
        //
        // The value is the squared normalized innovation threshold.
        //
        float innovationGate = 9.0f;
    };

    struct Input
    {
        //
        // Calibrated IMU acceleration in Body.
        //
        Vector3<Body, MeterPerSecondSquared> acceleration;

        //
        // Calibrated IMU angular velocity in Body.
        //
        Vector3<Body, RadiansPerSecond> angularVelocity;

        //
        // Mahony's current World -> Body attitude.
        //
        Quaternion<World, Body> attitude;

        //
        // MTF02P raw optical-flow measurements.
        //
        int32_t flowX = 0;
        int32_t flowY = 0;

        //
        // MTF02P range measurement.
        //
        Meters range;

        //
        // MTF02P optical-flow quality.
        //
        uint8_t flowQuality = 0;

        //
        // Prediction timestep.
        //
        Seconds predictionDt;

        //
        // Optical-flow integration timestep.
        //
        // If zero, Config::flowDt is used.
        //
        Seconds flowDt;
    };

    struct State
    {
        float velocityX = 0.0f;
        float velocityY = 0.0f;

        float biasX = 0.0f;
        float biasY = 0.0f;
    };

    struct Diagnostics
    {
        bool predictionValid = false;
        bool flowValid = false;
        bool innovationRejected = false;

        float flowVelocityX = 0.0f;
        float flowVelocityY = 0.0f;

        float innovationX = 0.0f;
        float innovationY = 0.0f;

        float innovationVarianceX = 0.0f;
        float innovationVarianceY = 0.0f;
    };

    VelocityKalmanFilter();
    explicit VelocityKalmanFilter(Config config);

    void reset();

    void reset(
        Vector3<World, MetersPerSecond> velocity
    );

    Vector3<World, MetersPerSecond> update(
        const Input& input
    );

    Vector3<World, MetersPerSecond> velocity() const;

    const State& state() const;

    const Diagnostics& diagnostics() const;

    const Config& config() const;

private:

    static constexpr uint32_t STATE_SIZE = 4;

    enum StateIndex
    {
        VX = 0,
        VY = 1,
        BX = 2,
        BY = 3
    };

    Config _config;

    State _state{};
    float _P[STATE_SIZE][STATE_SIZE]{};

    Diagnostics _diagnostics{};

    void predict(
        const Input& input
    );

    bool makeFlowMeasurement(
        const Input& input,
        float& velocityX,
        float& velocityY,
        float& varianceX,
        float& varianceY
    ) const;

    bool updateMeasurement(
        float measurementX,
        float measurementY,
        float varianceX,
        float varianceY
    );

    void initializeCovariance();

    static float clampPositive(
        float value,
        float minimum
    );
};

#endif /* INC_FC_VELKF_H_ */
