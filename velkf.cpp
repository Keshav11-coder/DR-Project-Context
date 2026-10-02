/*
 * velkf.cpp
 *
 * Velocity Kalman Filter
 */

#include <FC/velkf.h>
#include <FC/types.h>

#include <math.h>


VelocityKalmanFilter::VelocityKalmanFilter()
    : VelocityKalmanFilter(Config{})
{
}

VelocityKalmanFilter::VelocityKalmanFilter(Config config)
    : _config(config)
{
    reset();
}


void VelocityKalmanFilter::initializeCovariance()
{
    for (uint32_t r = 0; r < STATE_SIZE; ++r)
    {
        for (uint32_t c = 0; c < STATE_SIZE; ++c)
        {
            _P[r][c] = 0.0f;
        }
    }

    //
    // Initial velocity uncertainty.
    //
    _P[VX][VX] = 1.0f;
    _P[VY][VY] = 1.0f;

    //
    // Initial measurement-bias uncertainty.
    //
    _P[BX][BX] = 0.25f;
    _P[BY][BY] = 0.25f;
}


void VelocityKalmanFilter::reset()
{
    _state = State{};

    initializeCovariance();

    _diagnostics = Diagnostics{};
}


void VelocityKalmanFilter::reset(
    Vector3<World, MetersPerSecond> velocity
)
{
    reset();

    _state.velocityX = velocity.x.value();
    _state.velocityY = velocity.y.value();
}


float VelocityKalmanFilter::clampPositive(
    float value,
    float minimum
)
{
    if (!isfinite(value) || value < minimum)
    {
        return minimum;
    }

    return value;
}


void VelocityKalmanFilter::predict(
    const Input& input
)
{
    const float dt = input.predictionDt.value();

    if (!isfinite(dt) || dt <= 0.0f)
    {
        _diagnostics.predictionValid = false;
        return;
    }

    //
    // Mahony currently supplies World -> Body.
    //
    // Its inverse is therefore Body -> World.
    //
    const Quaternion<Body, World> bodyToWorld =
        input.attitude.inverse();

    Vector3<Body, MeterPerSecondSquared> accelerationBody =
        input.acceleration;

    //
    // Transform calibrated specific-force/acceleration measurement
    // from Body into World.
    //
    Vector3<World, MeterPerSecondSquared> accelerationWorld =
        rotate(
            bodyToWorld,
            accelerationBody
        );

    //
    // The ICM acceleration measurement contains the stationary
    // gravity reference used by Mahony. Convert specific force
    // into physical linear acceleration by adding World gravity.
    //
    const float ax =
        accelerationWorld.x.value();

    const float ay =
        accelerationWorld.y.value();

    const float az =
        accelerationWorld.z.value();

    //
    // Horizontal acceleration is independent of World-Z gravity
    // under the present planar estimator.
    //
    (void)az;

    const float physicalAx = ax;
    const float physicalAy = ay;

    //
    // Constant-acceleration prediction.
    //
    _state.velocityX += physicalAx * dt;
    _state.velocityY += physicalAy * dt;

    //
    // State transition is identity except for the velocity
    // integration. Process covariance is therefore represented
    // directly rather than constructing F explicitly.
    //
    const float accelerationVariance =
        _config.accelerationNoise *
        _config.accelerationNoise *
        dt * dt;

    const float biasVariance =
        _config.velocityBiasNoise *
        _config.velocityBiasNoise *
        dt;

    _P[VX][VX] += accelerationVariance;
    _P[VY][VY] += accelerationVariance;

    _P[BX][BX] += biasVariance;
    _P[BY][BY] += biasVariance;

    _diagnostics.predictionValid = true;
}


bool VelocityKalmanFilter::makeFlowMeasurement(
    const Input& input,
    float& velocityX,
    float& velocityY,
    float& varianceX,
    float& varianceY
) const
{
    //
    // Quality gate.
    //
    if (input.flowQuality < _config.minimumFlowQuality)
    {
        return false;
    }

    const float range =
        input.range.value();

    if (!isfinite(range))
    {
        return false;
    }

    if (range < _config.minimumRangeMeters ||
        range > _config.maximumRangeMeters)
    {
        return false;
    }

    //
    // Resolve the optical-flow integration interval.
    //
    float flowDt = input.flowDt.value();

    if (!isfinite(flowDt) || flowDt <= 0.0f)
    {
        flowDt = _config.flowDt.value();
    }

    if (!isfinite(flowDt) || flowDt <= 0.0f)
    {
        return false;
    }

    //
    // Raw MTF02P count -> integrated angular optical flow.
    //
    //
    // motion_x/y are currently interpreted as:
    //
    //     degrees * 10000
    //
    // Therefore:
    //
    //     rad/count =
    //         PI / (180 * 10000)
    //
    // This remains a strongly-supported hypothesis and must be
    // bench-validated.
    //
    const float flowX =
        static_cast<float>(input.flowX) *
        Config::FLOW_RADIANS_PER_COUNT *
        _config.flowScaleX;

    const float flowY =
        static_cast<float>(input.flowY) *
        Config::FLOW_RADIANS_PER_COUNT *
        _config.flowScaleY;

    //
    // Rotational compensation.
    //
    // Empirically established project-specific convention:
    //
    //     +omegaX -> +flowX rotation
    //     +omegaY -> -flowY rotation
    //     omegaZ  -> approximately zero flow coupling
    //
    // Therefore:
    //
    //     flowX_trans = flowX - omegaX * dt
    //     flowY_trans = flowY + omegaY * dt
    //
    const float omegaX =
        input.angularVelocity.x.value();

    const float omegaY =
        input.angularVelocity.y.value();

    const float flowXTrans =
        flowX -
        (omegaX * flowDt);

    const float flowYTrans =
        flowY +
        (omegaY * flowDt);

    //
    // Translational sign convention established experimentally:
    //
    //     +Body X velocity -> flowX < 0
    //     +Body Y velocity -> flowY < 0
    //
    // Therefore both axes receive the negative sign.
    //
    velocityX =
        -(range * flowXTrans / flowDt);

    velocityY =
        -(range * flowYTrans / flowDt);

    if (!isfinite(velocityX) ||
        !isfinite(velocityY))
    {
        return false;
    }

    //
    // The current configuration uses fixed measurement noise.
    //
    // Future versions can make these functions of:
    //
    //     - flow quality
    //     - range
    //     - angular rate
    //     - optical-flow innovation
    //
    // without changing the measurement model.
    //
    varianceX =
        clampPositive(
            _config.flowMeasurementNoiseX *
            _config.flowMeasurementNoiseX,
            1.0e-6f
        );

    varianceY =
        clampPositive(
            _config.flowMeasurementNoiseY *
            _config.flowMeasurementNoiseY,
            1.0e-6f
        );

    return true;
}


bool VelocityKalmanFilter::updateMeasurement(
    float measurementX,
    float measurementY,
    float varianceX,
    float varianceY
)
{
    //
    // Measurement model:
    //
    //     zX = vX + bX + noise
    //     zY = vY + bY + noise
    //
    // H:
    //
    //     [1 0 1 0]
    //     [0 1 0 1]
    //
    const float innovationX =
        measurementX -
        (_state.velocityX + _state.biasX);

    const float innovationY =
        measurementY -
        (_state.velocityY + _state.biasY);

    //
    // Since the X and Y measurement rows are independent,
    // calculate their innovation covariance independently.
    //
    const float Sx =
        _P[VX][VX] +
        _P[VX][BX] +
        _P[BX][VX] +
        _P[BX][BX] +
        varianceX;

    const float Sy =
        _P[VY][VY] +
        _P[VY][BY] +
        _P[BY][VY] +
        _P[BY][BY] +
        varianceY;

    if (!isfinite(Sx) ||
        !isfinite(Sy) ||
        Sx <= 1.0e-9f ||
        Sy <= 1.0e-9f)
    {
        return false;
    }

    _diagnostics.innovationX = innovationX;
    _diagnostics.innovationY = innovationY;

    _diagnostics.innovationVarianceX = Sx;
    _diagnostics.innovationVarianceY = Sy;

    //
    // Normalized innovation squared.
    //
    const float nisX =
        (innovationX * innovationX) / Sx;

    const float nisY =
        (innovationY * innovationY) / Sy;

    //
    // Reject the entire optical-flow update if either axis
    // fails the configured innovation gate.
    //
    if (_config.innovationGate > 0.0f)
    {
        if (nisX > _config.innovationGate ||
            nisY > _config.innovationGate)
        {
            _diagnostics.innovationRejected = true;
            return false;
        }
    }

    //
    // Compute the two independent Kalman gains.
    //
    float Kx[STATE_SIZE]{};
    float Ky[STATE_SIZE]{};

    for (uint32_t i = 0; i < STATE_SIZE; ++i)
    {
        Kx[i] =
            (_P[i][VX] + _P[i][BX]) / Sx;

        Ky[i] =
            (_P[i][VY] + _P[i][BY]) / Sy;
    }

    //
    // Save the old state.
    //
    float xOld[STATE_SIZE];

    xOld[VX] = _state.velocityX;
    xOld[VY] = _state.velocityY;
    xOld[BX] = _state.biasX;
    xOld[BY] = _state.biasY;

    //
    // State update.
    //
    for (uint32_t i = 0; i < STATE_SIZE; ++i)
    {
        xOld[i] =
            xOld[i] +
            Kx[i] * innovationX +
            Ky[i] * innovationY;
    }

    _state.velocityX = xOld[VX];
    _state.velocityY = xOld[VY];
    _state.biasX = xOld[BX];
    _state.biasY = xOld[BY];

    //
    // Joseph-form covariance update:
    //
    //     P+ =
    //         (I-KH)P-(I-KH)^T
    //         + K R K^T
    //
    // Construct A = I - K H.
    //
    float A[STATE_SIZE][STATE_SIZE]{};

    for (uint32_t r = 0; r < STATE_SIZE; ++r)
    {
        for (uint32_t c = 0; c < STATE_SIZE; ++c)
        {
            A[r][c] =
                (r == c) ? 1.0f : 0.0f;
        }

        A[r][VX] -= Kx[r];
        A[r][BX] -= Kx[r];

        A[r][VY] -= Ky[r];
        A[r][BY] -= Ky[r];
    }

    float AP[STATE_SIZE][STATE_SIZE]{};

    for (uint32_t r = 0; r < STATE_SIZE; ++r)
    {
        for (uint32_t c = 0; c < STATE_SIZE; ++c)
        {
            float sum = 0.0f;

            for (uint32_t k = 0; k < STATE_SIZE; ++k)
            {
                sum += A[r][k] * _P[k][c];
            }

            AP[r][c] = sum;
        }
    }

    float newP[STATE_SIZE][STATE_SIZE]{};

    for (uint32_t r = 0; r < STATE_SIZE; ++r)
    {
        for (uint32_t c = 0; c < STATE_SIZE; ++c)
        {
            float sum = 0.0f;

            for (uint32_t k = 0; k < STATE_SIZE; ++k)
            {
                sum += AP[r][k] * A[c][k];
            }

            newP[r][c] = sum;
        }
    }

    //
    // K R K^T.
    //
    for (uint32_t r = 0; r < STATE_SIZE; ++r)
    {
        for (uint32_t c = 0; c < STATE_SIZE; ++c)
        {
            newP[r][c] +=
                Kx[r] * varianceX * Kx[c];

            newP[r][c] +=
                Ky[r] * varianceY * Ky[c];
        }
    }

    //
    // Explicitly restore symmetry against accumulated
    // floating-point roundoff.
    //
    for (uint32_t r = 0; r < STATE_SIZE; ++r)
    {
        for (uint32_t c = r + 1; c < STATE_SIZE; ++c)
        {
            const float symmetric =
                0.5f *
                (newP[r][c] + newP[c][r]);

            newP[r][c] = symmetric;
            newP[c][r] = symmetric;
        }

        if (newP[r][r] < 0.0f ||
            !isfinite(newP[r][r]))
        {
            newP[r][r] = 1.0e-9f;
        }
    }

    for (uint32_t r = 0; r < STATE_SIZE; ++r)
    {
        for (uint32_t c = 0; c < STATE_SIZE; ++c)
        {
            _P[r][c] = newP[r][c];
        }
    }

    return true;
}


Vector3<World, MetersPerSecond>
VelocityKalmanFilter::update(
    const Input& input
)
{
    _diagnostics.predictionValid = false;
    _diagnostics.flowValid = false;
    _diagnostics.innovationRejected = false;

    //
    // 1. IMU prediction.
    //
    predict(input);

    //
    // 2. Optical-flow measurement construction.
    //
    float flowVelocityX = 0.0f;
    float flowVelocityY = 0.0f;

    float flowVarianceX = 0.0f;
    float flowVarianceY = 0.0f;

    const bool flowMeasurementValid =
        makeFlowMeasurement(
            input,
            flowVelocityX,
            flowVelocityY,
            flowVarianceX,
            flowVarianceY
        );

    if (flowMeasurementValid)
    {
        _diagnostics.flowVelocityX =
            flowVelocityX;

        _diagnostics.flowVelocityY =
            flowVelocityY;

        //
        // 3. Convert Body-frame optical-flow velocity
        //    into World-frame measurement.
        //
        const Quaternion<Body, World> bodyToWorld =
            input.attitude.inverse();

        const Vector3<Body, MetersPerSecond>
            flowVelocityBody = {
                .x = MetersPerSecond{flowVelocityX},
                .y = MetersPerSecond{flowVelocityY},
                .z = MetersPerSecond{0.0f}
            };

        const Vector3<World, MetersPerSecond>
            flowVelocityWorld =
                rotate(
                    bodyToWorld,
                    flowVelocityBody
                );

        //
        // The filter is planar. Z is deliberately discarded.
        //
        _diagnostics.flowValid =
            updateMeasurement(
                flowVelocityWorld.x.value(),
                flowVelocityWorld.y.value(),
                flowVarianceX,
                flowVarianceY
            );
    }

    return velocity();
}


Vector3<World, MetersPerSecond>
VelocityKalmanFilter::velocity() const
{
    return Vector3<World, MetersPerSecond>{
        .x = MetersPerSecond{_state.velocityX},
        .y = MetersPerSecond{_state.velocityY},
        .z = MetersPerSecond{0.0f}
    };
}


const VelocityKalmanFilter::State&
VelocityKalmanFilter::state() const
{
    return _state;
}


const VelocityKalmanFilter::Diagnostics&
VelocityKalmanFilter::diagnostics() const
{
    return _diagnostics;
}


const VelocityKalmanFilter::Config&
VelocityKalmanFilter::config() const
{
    return _config;
}
