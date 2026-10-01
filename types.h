/*
 * types.h
 *
 *  Created on: Jun 4, 2026
 *      Author: anonymous
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#include <math.h>

#include <stdint.h>

constexpr float PI_F         = 3.14159265358979323846f;
constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
constexpr float RAD_TO_DEG_F = 180.0f / PI_F;
constexpr float GRAVITY_F    = 9.80665f;

struct Radians;
struct Degrees;
struct DegreesPerSecond;
struct RadiansPerSecond;
struct DegreesPerSecondSquared;
struct RadiansPerSecondSquared;
struct MeterPerSecondSquared;
struct MetersPerSecond;
struct Meters;
struct Kilograms;
struct Newton;
struct NewtonMeters;
struct Hertz;
struct Seconds;
struct Milliseconds;
struct Microseconds;

template<typename Derived>
struct Frame
{
};

struct Body  : Frame<Body> {};
struct World : Frame<World> {};

template<typename Derived, typename T = float>
class Unit
{
protected:
    T _value;

public:
    constexpr explicit Unit(T v = T{})
        : _value(v)
    {
    }

    constexpr T value() const
    {
        return _value;
    }

    //
    // Arithmetic
    //

    constexpr Derived operator+(Derived rhs) const
    {
        return Derived(_value + rhs.value());
    }

    constexpr Derived operator-(Derived rhs) const
    {
        return Derived(_value - rhs.value());
    }

    constexpr Derived operator-() const
    {
        return Derived(-_value);
    }

    constexpr Derived operator*(T rhs) const
    {
        return Derived(_value * rhs);
    }

    constexpr Derived operator/(T rhs) const
    {
        return Derived(_value / rhs);
    }

    constexpr T operator/(Derived rhs) const
    {
        return _value / rhs.value();
    }

    //
    // Compound
    //

    constexpr Derived& operator+=(Derived rhs)
    {
        _value += rhs.value();
        return static_cast<Derived&>(*this);
    }

    constexpr Derived& operator-=(Derived rhs)
    {
        _value -= rhs.value();
        return static_cast<Derived&>(*this);
    }

    constexpr Derived& operator*=(T rhs)
    {
        _value *= rhs;
        return static_cast<Derived&>(*this);
    }

    constexpr Derived& operator/=(T rhs)
    {
        _value /= rhs;
        return static_cast<Derived&>(*this);
    }

    //
    // Increment / Decrement
    //

    constexpr Derived& operator++()
    {
        ++_value;
        return static_cast<Derived&>(*this);
    }

    constexpr Derived operator++(int)
    {
        Derived tmp = static_cast<Derived&>(*this);
        ++(*this);
        return tmp;
    }

    constexpr Derived& operator--()
    {
        --_value;
        return static_cast<Derived&>(*this);
    }

    constexpr Derived operator--(int)
    {
        Derived tmp = static_cast<Derived&>(*this);
        --(*this);
        return tmp;
    }

    //
    // Comparisons
    //

    constexpr bool operator==(Derived rhs) const
    {
        return _value == rhs.value();
    }

    constexpr bool operator!=(Derived rhs) const
    {
        return _value != rhs.value();
    }

    constexpr bool operator<(Derived rhs) const
    {
        return _value < rhs.value();
    }

    constexpr bool operator>(Derived rhs) const
    {
        return _value > rhs.value();
    }

    constexpr bool operator<=(Derived rhs) const
    {
        return _value <= rhs.value();
    }

    constexpr bool operator>=(Derived rhs) const
    {
        return _value >= rhs.value();
    }

    //
    // Scalar-left multiply
    //

    friend constexpr Derived operator*(T lhs, Derived rhs)
    {
        return Derived(lhs * rhs.value());
    }
};

struct Degrees : Unit<Degrees>
{
    using Unit::Unit;

    Radians toRadians() const;
};

struct Radians : Unit<Radians>
{
    using Unit::Unit;

    Degrees toDegrees() const;
};

struct DegreesPerSecond : Unit<DegreesPerSecond>
{
    using Unit::Unit;

    RadiansPerSecond toRadiansPerSecond() const;
};

struct RadiansPerSecond : Unit<RadiansPerSecond>
{
    using Unit::Unit;

    DegreesPerSecond toDegreesPerSecond() const;
};

struct DegreesPerSecondSquared : Unit<DegreesPerSecondSquared>
{
    using Unit::Unit;

    RadiansPerSecondSquared toRadiansPerSecondSquared() const;
};

struct RadiansPerSecondSquared : Unit<RadiansPerSecondSquared>
{
    using Unit::Unit;

    DegreesPerSecondSquared toDegreesPerSecondSquared() const;
};

struct MeterPerSecondSquared : Unit<MeterPerSecondSquared>
{
    using Unit::Unit;

    constexpr float toG() const
    {
        return value() / GRAVITY_F;
    }
};

struct Kilograms : Unit<Kilograms>
{
    using Unit::Unit;

    // F = m * a
    constexpr Newton toForce(MeterPerSecondSquared acceleration) const;
};

struct MetersPerSecond : Unit<MetersPerSecond>
{
    using Unit::Unit;

    constexpr float toKilometersPerHour() const
    {
        return value() * 3.6f;
    }
};

struct Meters : Unit<Meters>
{
    using Unit::Unit;

    constexpr float toKilometers() const
    {
        return value() / 1000.0f;
    }

    // τ = F * r
    constexpr NewtonMeters toNewtonMeters(Newton force) const;
};

struct Newton : Unit<Newton>
{
    using Unit::Unit;

    // m = F / a
    constexpr Kilograms toMass(MeterPerSecondSquared acceleration) const;

    // a = F / m
    constexpr MeterPerSecondSquared toAcceleration(Kilograms mass) const;

    // τ = F * r
    constexpr NewtonMeters toNewtonMeters(Meters radius) const;
};

struct NewtonMeters : Unit<NewtonMeters>
{
    using Unit::Unit;

    // F = τ / r
    constexpr Newton toForce(Meters radius) const;

    // r = τ / F
    constexpr Meters toRadius(Newton force) const;
};

inline constexpr Newton Kilograms::toForce(
    MeterPerSecondSquared acceleration
) const
{
    return Newton(value() * acceleration.value());
}

inline constexpr Kilograms Newton::toMass(
    MeterPerSecondSquared acceleration
) const
{
    return Kilograms(value() / acceleration.value());
}

inline constexpr MeterPerSecondSquared Newton::toAcceleration(
    Kilograms mass
) const
{
    return MeterPerSecondSquared(value() / mass.value());
}

inline constexpr NewtonMeters Meters::toNewtonMeters(
    Newton force
) const
{
    return NewtonMeters(value() * force.value());
}

inline constexpr NewtonMeters Newton::toNewtonMeters(
    Meters radius
) const
{
    return NewtonMeters(value() * radius.value());
}

inline constexpr Newton NewtonMeters::toForce(
    Meters radius
) const
{
    return Newton(value() / radius.value());
}

inline constexpr Meters NewtonMeters::toRadius(
    Newton force
) const
{
    return Meters(value() / force.value());
}

struct Seconds : Unit<Seconds>
{
    using Unit::Unit;

    Hertz toHertz() const;
};

struct Milliseconds : Unit<Milliseconds, uint32_t>
{
    using Unit::Unit;

    constexpr Seconds toSeconds() const
    {
    	return Seconds(
			static_cast<float>(value()) / 1000000.0f
		);
    }
};

struct Microseconds : Unit<Microseconds, uint32_t>
{
    using Unit::Unit;

    constexpr Seconds toSeconds() const
    {
        return Seconds(
            static_cast<float>(value()) / 1000000.0f
        );
    }
};

struct Hertz : Unit<Hertz>
{
    using Unit::Unit;

    constexpr Seconds period() const;
};

//
// RAW IMU COUNTS
//

struct LSB : Unit<LSB, int32_t>
{
    using Unit::Unit;

    constexpr DegreesPerSecond toDPS(float sensitivity) const
    {
        return DegreesPerSecond(
            static_cast<float>(value()) / sensitivity
        );
    }

    constexpr RadiansPerSecond toRadPS(float sensitivity) const
    {
        return RadiansPerSecond(
            (static_cast<float>(value()) / sensitivity)
            * DEG_TO_RAD_F
        );
    }

    constexpr float toG(float sensitivity) const
    {
        return static_cast<float>(value()) / sensitivity;
    }

    constexpr MeterPerSecondSquared toMS2(float sensitivity) const
    {
        return MeterPerSecondSquared(
            (static_cast<float>(value()) / sensitivity)
            * GRAVITY_F
        );
    }
};

inline Radians Degrees::toRadians() const
{
    return Radians(value() * DEG_TO_RAD_F);
}

inline Degrees Radians::toDegrees() const
{
    return Degrees(value() * RAD_TO_DEG_F);
}

inline RadiansPerSecond DegreesPerSecond::toRadiansPerSecond() const
{
    return RadiansPerSecond(value() * DEG_TO_RAD_F);
}

inline DegreesPerSecond RadiansPerSecond::toDegreesPerSecond() const
{
    return DegreesPerSecond(value() * RAD_TO_DEG_F);
}

inline RadiansPerSecondSquared DegreesPerSecondSquared::toRadiansPerSecondSquared() const
{
    return RadiansPerSecondSquared(value() * DEG_TO_RAD_F);
}

inline DegreesPerSecondSquared RadiansPerSecondSquared::toDegreesPerSecondSquared() const
{
    return DegreesPerSecondSquared(value() * RAD_TO_DEG_F);
}

inline Hertz Seconds::toHertz() const
{
    return Hertz(1.0f / value());
}

inline constexpr Seconds Hertz::period() const
{
    return Seconds(1.0f / value());
}

template<typename FrameType, typename Quantity>
struct Vector3
{
    Quantity x{};
    Quantity y{};
    Quantity z{};

    constexpr Vector3() = default;

//    constexpr Vector3(
//        Quantity _x,
//        Quantity _y,
//        Quantity _z
//    )
//        : x(_x), y(_y), z(_z)
//    {
//    }

    //
    // Vector arithmetic
    //

    constexpr Vector3 operator+(
        const Vector3& rhs
    ) const
    {
        return Vector3(
            x + rhs.x,
            y + rhs.y,
            z + rhs.z
        );
    }

    constexpr Vector3 operator-(
        const Vector3& rhs
    ) const
    {
        return Vector3(
            x - rhs.x,
            y - rhs.y,
            z - rhs.z
        );
    }

    constexpr Vector3 operator-() const
    {
        return Vector3(
            -x,
            -y,
            -z
        );
    }

    constexpr Vector3 operator*(float scalar) const
    {
        return Vector3(
            x * scalar,
            y * scalar,
            z * scalar
        );
    }

    constexpr Vector3 operator/(float scalar) const
    {
        return Vector3(
            x / scalar,
            y / scalar,
            z / scalar
        );
    }

    //
    // Compound
    //

    constexpr Vector3& operator+=(
        const Vector3& rhs
    )
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;

        return *this;
    }

    constexpr Vector3& operator-=(
        const Vector3& rhs
    )
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;

        return *this;
    }

    constexpr Vector3& operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;

        return *this;
    }

    constexpr Vector3& operator/=(float scalar)
    {
        x /= scalar;
        y /= scalar;
        z /= scalar;

        return *this;
    }

    //
    // Comparisons
    //

    constexpr bool operator==(
        const Vector3& rhs
    ) const
    {
        return x == rhs.x
            && y == rhs.y
            && z == rhs.z;
    }

    constexpr bool operator!=(
        const Vector3& rhs
    ) const
    {
        return !(*this == rhs);
    }

    constexpr Quantity norm() {
    	return Quantity{sqrtf(((x*x)+(y*y)+(z*z)).value())};
    }

    //
    // Scalar-left multiply
    //

    friend constexpr Vector3 operator*(
        float scalar,
        const Vector3& vector
    )
    {
        return vector * scalar;
    }
};

//
// FRAME-AWARE QUATERNION
//
// Quaternion<From, To> represents a rotation/transformation
// that converts vectors expressed in From into vectors expressed
// in To.
//
// Example:
//
//     Quaternion<Body, World>
//
// represents:
//
//     Body -> World
//

template<typename From, typename To>
struct Quaternion
{
    float w{};
    float x{};
    float y{};
    float z{};

    constexpr Quaternion(
        float _w = 1.0f,
        float _x = 0.0f,
        float _y = 0.0f,
        float _z = 0.0f
    )
        : w(_w), x(_x), y(_y), z(_z)
    {
    }

    //
    // Identity
    //
    // Identity is only mathematically meaningful when From == To.
    // The static_assert is intentionally inside the function body
    // so that the frame types are fully instantiated when used.
    //

    static constexpr Quaternion identity()
    {
        return Quaternion(
            1.0f,
            0.0f,
            0.0f,
            0.0f
        );
    }

    //
    // Addition / subtraction
    //
    // Both quaternions must represent the exact same transformation.
    //

    constexpr Quaternion operator+(
        const Quaternion& q
    ) const
    {
        return Quaternion(
            w + q.w,
            x + q.x,
            y + q.y,
            z + q.z
        );
    }

    constexpr Quaternion operator-(
        const Quaternion& q
    ) const
    {
        return Quaternion(
            w - q.w,
            x - q.x,
            y - q.y,
            z - q.z
        );
    }

    //
    // Scalar multiplication
    //

    constexpr Quaternion operator*(float k) const
    {
        return Quaternion(
            w * k,
            x * k,
            y * k,
            z * k
        );
    }

    friend constexpr Quaternion operator*(
        float k,
        const Quaternion& q
    )
    {
        return q * k;
    }

    //
    // Length
    //

    constexpr float length_squared() const
    {
        return
            (w * w) +
            (x * x) +
            (y * y) +
            (z * z);
    }

    float norm() const
    {
        return sqrtf(
            w * w +
            x * x +
            y * y +
            z * z
        );
    }

    //
    // Normalize
    //

    void normalize()
    {
        float l_sq = length_squared();

        if (l_sq < 1e-8f)
        {
            *this = Quaternion(
                1.0f,
                0.0f,
                0.0f,
                0.0f
            );

            return;
        }

        float inv_l = 1.0f / sqrtf(l_sq);

        w *= inv_l;
        x *= inv_l;
        y *= inv_l;
        z *= inv_l;
    }

    //
    // Conjugate
    //
    // The conjugate reverses the transformation direction.
    //
    // Quaternion<From, To>
    //
    //     becomes
    //
    // Quaternion<To, From>
    //
    // representing:
    //
    //     To -> From
    //

    constexpr Quaternion<To, From> conjugate() const
    {
        return Quaternion<To, From>(
            w,
            -x,
            -y,
            -z
        );
    }

    //
    // Inverse
    //
    // For a normalized quaternion this is simply the conjugate.
    // This implementation remains mathematically correct even if
    // the quaternion has not yet been normalized.
    //
    // The inverse reverses the transformation direction:
    //
    //     From -> To
    //
    // becomes:
    //
    //     To -> From
    //

    Quaternion<To, From> inverse() const
    {
        float l_sq = length_squared();

        if (l_sq < 1e-8f)
        {
            return Quaternion<To, From>(
                1.0f,
                0.0f,
                0.0f,
                0.0f
            );
        }

        float inv_l_sq = 1.0f / l_sq;

        return Quaternion<To, From>(
            w * inv_l_sq,
            -x * inv_l_sq,
            -y * inv_l_sq,
            -z * inv_l_sq
        );
    }

    //
    // Axis / angle
    //
    // The resulting quaternion represents a transformation
    // from From -> To.
    //

    static Quaternion from_axis_angle(
        float angle_rad,
        float ax,
        float ay,
        float az
    )
    {
        float magnitude_sq =
            (ax * ax) +
            (ay * ay) +
            (az * az);

        if (magnitude_sq < 1e-8f)
        {
            return Quaternion(
                1.0f,
                0.0f,
                0.0f,
                0.0f
            );
        }

        float inv_mag = 1.0f / sqrtf(magnitude_sq);

        ax *= inv_mag;
        ay *= inv_mag;
        az *= inv_mag;

        float half_angle = angle_rad * 0.5f;
        float sin_half = sinf(half_angle);

        return Quaternion(
            cosf(half_angle),
            ax * sin_half,
            ay * sin_half,
            az * sin_half
        );
    }

    //
    // Raw vector rotation
    //
    // Applies this From -> To rotation to raw vector components.
    //
    // Kept as a low-level operation for embedded efficiency.
    //

    void rotate_vector(
        float& vx,
        float& vy,
        float& vz
    ) const
    {
        float tx =
            2.0f *
            (y * vz - z * vy);

        float ty =
            2.0f *
            (z * vx - x * vz);

        float tz =
            2.0f *
            (x * vy - y * vx);

        vx =
            vx +
            w * tx +
            (y * tz - z * ty);

        vy =
            vy +
            w * ty +
            (z * tx - x * tz);

        vz =
            vz +
            w * tz +
            (x * ty - y * tx);
    }
};

//
// TYPED QUATERNION COMPOSITION
//
// Quaternion<From, Mid> *
// Quaternion<Mid, To>
//
//      =
//
// Quaternion<From, To>
//
// The right-hand quaternion is applied first:
//
//     From -> Mid -> To
//

template<typename From, typename Mid, typename To>
constexpr Quaternion<From, To> operator*(
    const Quaternion<From, Mid>& lhs,
    const Quaternion<Mid, To>& rhs
)
{
    return Quaternion<From, To>(
        lhs.w * rhs.w
            - lhs.x * rhs.x
            - lhs.y * rhs.y
            - lhs.z * rhs.z,

        lhs.w * rhs.x
            + lhs.x * rhs.w
            + lhs.y * rhs.z
            - lhs.z * rhs.y,

        lhs.w * rhs.y
            - lhs.x * rhs.z
            + lhs.y * rhs.w
            + lhs.z * rhs.x,

        lhs.w * rhs.z
            + lhs.x * rhs.y
            - lhs.y * rhs.x
            + lhs.z * rhs.w
    );
}

//
// FRAME-AWARE VECTOR ROTATION
//
// Quaternion<From, To>
// Vector3<From, Quantity>
//
//          ↓
//
// Vector3<To, Quantity>
//
// The quaternion converts the vector:
//
//     From -> To
//

template<typename From, typename To, typename Quantity>
Vector3<To, Quantity> rotate(
    const Quaternion<From, To>& q,
    const Vector3<From, Quantity>& v
)
{
    float x = static_cast<float>(v.x.value());
    float y = static_cast<float>(v.y.value());
    float z = static_cast<float>(v.z.value());

    q.rotate_vector(x, y, z);

    return Vector3<To, Quantity>(
        Quantity(x),
        Quantity(y),
        Quantity(z)
    );
}

#endif /* INC_TYPES_H_ */
