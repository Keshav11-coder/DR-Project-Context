/*
 * mahony.h
 *
 *  Created on: June 4, 2026
 *      Author: anonymous
 */

#ifndef INC_MAHONY_H_
#define INC_MAHONY_H_

#include <math.h>
#include "types.h"

constexpr float GRAVITY_NOMINAL = 9.80665f;

constexpr float GRAVITY_TRUST_MIN = 0.5f * GRAVITY_NOMINAL; // g -> m/s^2
constexpr float GRAVITY_TRUST_MAX = 1.5f * GRAVITY_NOMINAL; // g -> m/s^2

constexpr float GRAVITY_TRUST_MIN_SQR =
    GRAVITY_TRUST_MIN * GRAVITY_TRUST_MIN;

constexpr float GRAVITY_TRUST_MAX_SQR =
    GRAVITY_TRUST_MAX * GRAVITY_TRUST_MAX;

class MahonyFilter
{
public:

    //
    // Quaternion<World, Body> represents:
    //
    //     World -> Body
    //
    // This preserves the physical transformation represented by
    // Quaternion<Body, World> under the previous Quaternion<To, From>
    // convention.
    //

    MahonyFilter(
        Quaternion<World, Body> initialQs,
        float _kp = 30.0,
        float _ki = 0.0
    )
        : qs(initialQs),
          kp(_kp),
          ki(_ki)
    {
    }

    struct InertialPacket
    {
        Vector3<Body, MeterPerSecondSquared> acceleration;
        Vector3<Body, RadiansPerSecond> angularVelocity;
    };

    //
    // Returns the current World -> Body orientation.
    //

    Quaternion<World, Body> update(
        InertialPacket ip = {
            .acceleration = Vector3<Body, MeterPerSecondSquared> {
                .x = MeterPerSecondSquared{0},
                .y = MeterPerSecondSquared{0},
                .z = MeterPerSecondSquared{0}
            },

            .angularVelocity = Vector3<Body, RadiansPerSecond> {
                .x = RadiansPerSecond{0},
                .y = RadiansPerSecond{0},
                .z = RadiansPerSecond{0}
            }
        },
        Seconds dt = Seconds(0.01f)
    )
    {
        float dtf = dt.value();

        if (dtf <= 0.0f)
            return qs;

        float ax = ip.acceleration.x.value();
        float ay = ip.acceleration.y.value();
        float az = ip.acceleration.z.value();

        float gx = ip.angularVelocity.x.value();
        float gy = ip.angularVelocity.y.value();
        float gz = ip.angularVelocity.z.value();

        float rawMagSqr =
            ax * ax +
            ay * ay +
            az * az;

        const bool gravityReferenceValid =
            rawMagSqr >= GRAVITY_TRUST_MIN_SQR &&
            rawMagSqr <= GRAVITY_TRUST_MAX_SQR;

        if (gravityReferenceValid)
        {
            // Normalize accelerometer (assuming the measurement of gravity in the body frame)
            reciprocalNorm = 1.0f / sqrtf(rawMagSqr);

            ax *= reciprocalNorm;
            ay *= reciprocalNorm;
            az *= reciprocalNorm;

            //
            // Estimated direction of gravity in the body frame
            // (factor of 2 divided out).
            //
            // qs represents the World -> Body transformation.
            //

            vx = qs.x * qs.z - qs.w * qs.y;
            vy = qs.w * qs.x + qs.y * qs.z;
            vz = qs.w * qs.w - 0.5f + qs.z * qs.z;

            //
            // Error is cross-product between estimated and measured
            // direction of gravity in Body (half the actual magnitude).
            //

            ex = ay * vz - az * vy;
            ey = az * vx - ax * vz;
            ez = ax * vy - ay * vx;

            //
            // Compute and apply to gyro term with integral feedback.
            //

            if (ki > 0.0)
            {
                ix += ki * ex * dt.value();
                iy += ki * ey * dt.value();
                iz += ki * ez * dt.value();

                gx += ix;
                gy += iy;
                gz += iz;
            }

            //
            // Apply proportional feedback to gyro.
            //

            gx += kp * ex;
            gy += kp * ey;
            gz += kp * ez;
        }

        //
        // Integrate rate of change of quaternion given by gyro.
        //

        const float halfDt = 0.5f * dtf;

        gx *= halfDt;
        gy *= halfDt;
        gz *= halfDt;

        qa = qs.w;
        qb = qs.x;
        qc = qs.y;

        //
        // Add qmult * dt to current orientation.
        //
        // qs remains a World -> Body transformation.
        //

        qs.w += (-qb * gx - qc * gy - qs.z * gz);
        qs.x += ( qa * gx + qc * gz - qs.z * gy);
        qs.y += ( qa * gy - qb * gz + qs.z * gx);
        qs.z += ( qa * gz + qb * gy - qc * gx);

        //
        // Normalize quaternion.
        //

        qs.normalize();

        return qs;
    }

private:

    //
    // World -> Body orientation.
    //
    // Under the canonical Quaternion<From, To> convention:
    //
    //     Quaternion<World, Body>
    //
    // means:
    //
    //     World -> Body
    //

    Quaternion<World, Body> qs;

    float kp = 30.0;
    float ki = 0.0;

    float reciprocalNorm = 1.0f;

    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    float qa = 0.0f, qb = 0.0f, qc = 0.0f;

    float ix = 0, iy = 0, iz = 0;
};

#endif /* INC_MAHONY_H_ */
