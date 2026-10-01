/*
 * orientation_controller.h
 *
 *  Created on: Aug 17, 2026
 *      Author: anonymous
 */

#ifndef INC_FC_CASCADE_ORIENTATION_CONTROLLER_H_
#define INC_FC_CASCADE_ORIENTATION_CONTROLLER_H_

#include "../types.h"

// Quaternion<From, To>
// Transforms vectors from From frame into To frame.
//
// Quaternion<World, Body>
//     World → Body

struct OrientationPIDGains {
	float kP_X = 4.5f;
	float kP_Y = 4.5f;
	float kP_Z = 2.8f;
};

struct OrientationControlRequest
{
	// World → Body attitude.
	Quaternion<World, Body> target;
	Quaternion<World, Body> measured;
};

class OrientationController
{
public:
	using OrientationCorrection = Vector3<Body, RadiansPerSecond>;

    explicit OrientationController(
		OrientationPIDGains gains_)
        : _gains(gains_)
    {}

    Vector3<Body, RadiansPerSecond> update(
        const OrientationControlRequest& request,
        Seconds dt)
    {
        //
        // 1. Fetch current attitude estimation tracking from
        //    Mahony Filter.
        //
        // q_curr represents:
        //
        //     World → Body
        //

        Quaternion<World, Body> q_curr = request.measured;

        //
        // 2. Compute orientation error.
        //
        // q_target:
        //
        //     World → Body
        //
        // q_curr^-1:
        //
        //     Body → World
        //
        // The resulting error is expressed in Body:
        //
        //     Body → Body
        //
        // For unit quaternions, the conjugate is mathematically
        // equivalent to the inverse.
        //

        Quaternion<Body, Body> q_err =
            q_curr.conjugate() * request.target;

        //
        // 3. Resolve double covering to guarantee the shortest
        //    3D angular trajectory.
        //

        if (q_err.w < 0.0f) {
            q_err.x = -q_err.x;
            q_err.y = -q_err.y;
            q_err.z = -q_err.z;
        }

        //
        // 4. Transform orientation error components directly
        //    into target angular body velocities.
        //
        // Factor of 2.0f originates from the small-angle
        // linearization of quaternion errors.
        //

        _correction = Vector3<Body, RadiansPerSecond>{
        	.x = RadiansPerSecond{
        		(2.0f * q_err.x * _gains.kP_X)
        	},

        	.y = RadiansPerSecond{
        		(2.0f * q_err.y * _gains.kP_Y)
        	},

			.z = RadiansPerSecond{
				(2.0f * q_err.z * _gains.kP_Z)
			}
        };

        _request = request;

        return _correction;
    }

    const Quaternion<World, Body>& getTarget() const {
    	return _request.target;
    }

    const Quaternion<World, Body>& getMeasurement() const {
		return _request.measured;
	}

    void reset()
    {
        _correction =
        	Vector3<Body, RadiansPerSecond>{};

        _request =
        	OrientationControlRequest{};
    }

private:
    Vector3<Body, RadiansPerSecond> _correction;
    OrientationControlRequest _request;

    OrientationPIDGains _gains;
};


#endif /* INC_FC_CASCADE_ORIENTATION_CONTROLLER_H_ */
