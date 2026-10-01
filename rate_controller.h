/*
 * rate_controller.h
 *
 *  Created on: Aug 17, 2026
 *      Author: anonymous
 */

#ifndef INC_FC_CASCADE_RATE_CONTROLLER_H_
#define INC_FC_CASCADE_RATE_CONTROLLER_H_

#include "../types.h"
#include "../Policies/pid.h"

struct RateControlRequest
{
	Vector3<Body, RadiansPerSecond> target;
	Vector3<Body, RadiansPerSecond> measured;
};

class TorqueRateController
{
public:
	using TorqueRateCorrection = Vector3<Body, NewtonMeters>;

    explicit TorqueRateController(PIDCorrectionPolicy policy_x_,
								  PIDCorrectionPolicy policy_y_,
								  PIDCorrectionPolicy policy_z_)
        : policy_x(policy_x_), policy_y(policy_y_), policy_z(policy_z_)
    {}

    Vector3<Body, NewtonMeters> update(
        const RateControlRequest& request,
        Seconds dt)
    {
        const float correction_x_ = policy_x.compute(
        		PIDCorrectionRequest<float>{
        			.target=request.target.x.value(),
        			.measured=request.measured.x.value()
        		}, dt);

        const float correction_y_ = policy_y.compute(
        		PIDCorrectionRequest<float>{
        			.target=request.target.y.value(),
        			.measured=request.measured.y.value()
        		}, dt);

        const float correction_z_ = policy_z.compute(
        		PIDCorrectionRequest<float>{
        			.target=request.target.z.value(),
        			.measured=request.measured.z.value()
        		}, dt);

        _correction = Vector3<Body, NewtonMeters>{
        	.x = NewtonMeters{correction_x_},
        	.y = NewtonMeters{correction_y_},
        	.z = NewtonMeters{correction_z_}
        };

        _request = request;

        return _correction;
    }

    const Vector3<Body, RadiansPerSecond>& getTarget() const {
    	return _request.target;
    }

    const Vector3<Body, RadiansPerSecond>& getMeasurement() const {
		return _request.measured;
	}

    void reset()
    {
        policy_x.reset();
        policy_y.reset();
        policy_z.reset();

        _correction = Vector3<Body, NewtonMeters>{};
        _request = RateControlRequest{};
    }

    void reset(const RateControlRequest& request)
	{
		policy_x.reset(request.measured.x.value());
		policy_y.reset(request.measured.y.value());
		policy_z.reset(request.measured.z.value());

		_correction = Vector3<Body, NewtonMeters>{};
		_request = request;
	}

private:
    PIDCorrectionPolicy policy_x;
    PIDCorrectionPolicy policy_y;
    PIDCorrectionPolicy policy_z;

    Vector3<Body, NewtonMeters> _correction;
    RateControlRequest _request;
};

class AccelerationRateController
{
public:
	using AccelerationRateCorrection =
		Vector3<Body, RadiansPerSecondSquared>;

    explicit AccelerationRateController(
		PIDCorrectionPolicy policy_x_,
		PIDCorrectionPolicy policy_y_,
		PIDCorrectionPolicy policy_z_)
        : policy_x(policy_x_),
          policy_y(policy_y_),
          policy_z(policy_z_)
    {}

    Vector3<Body, RadiansPerSecondSquared> update(
        const RateControlRequest& request,
        Seconds dt)
    {
        const float correction_x_ = policy_x.compute(
        		PIDCorrectionRequest<float>{
        			.target=request.target.x.value(),
        			.measured=request.measured.x.value()
        		}, dt);

        const float correction_y_ = policy_y.compute(
        		PIDCorrectionRequest<float>{
        			.target=request.target.y.value(),
        			.measured=request.measured.y.value()
        		}, dt);

        const float correction_z_ = policy_z.compute(
        		PIDCorrectionRequest<float>{
        			.target=request.target.z.value(),
        			.measured=request.measured.z.value()
        		}, dt);

        _correction = Vector3<Body, RadiansPerSecondSquared>{
        	.x = RadiansPerSecondSquared{correction_x_},
        	.y = RadiansPerSecondSquared{correction_y_},
        	.z = RadiansPerSecondSquared{correction_z_}
        };

        _request = request;

        return _correction;
    }

    const Vector3<Body, RadiansPerSecond>& getTarget() const {
    	return _request.target;
    }

    const Vector3<Body, RadiansPerSecond>& getMeasurement() const {
		return _request.measured;
	}

    void reset()
    {
        policy_x.reset();
        policy_y.reset();
        policy_z.reset();

        _correction =
        	Vector3<Body, RadiansPerSecondSquared>{};

        _request = RateControlRequest{};
    }

    void reset(const RateControlRequest& request)
	{
		policy_x.reset(request.measured.x.value());
		policy_y.reset(request.measured.y.value());
		policy_z.reset(request.measured.z.value());

		_correction =
			Vector3<Body, RadiansPerSecondSquared>{};

		_request = request;
	}

private:
    PIDCorrectionPolicy policy_x;
    PIDCorrectionPolicy policy_y;
    PIDCorrectionPolicy policy_z;

    Vector3<Body, RadiansPerSecondSquared> _correction;
    RateControlRequest _request;
};

#endif /* INC_FC_CASCADE_RATE_CONTROLLER_H_ */
