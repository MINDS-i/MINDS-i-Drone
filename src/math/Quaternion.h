#ifndef QUATERNION_H
#define QUATERNION_H
#include "SpatialMath.h"
#include "Vec3.h"
class Vec3;
class Quaternion {
  private:
    float w, x, y, z;

  public:
    Quaternion() : w(1.f), x(0.f), y(0.f), z(0.f) {}
    Quaternion(float W, float X, float Y, float Z) : w(W), x(X), y(Y), z(Z) {}
    explicit Quaternion(const Vec3& euler);
    explicit Quaternion(const Vec3& axis, float angle);
    explicit Quaternion(const Vec3& ref, const Vec3& vec);
    // const methods
    Quaternion inverse() const;
    float dot(const Quaternion& l) const;
    float length() const;
    Vec3 axis() const;
    Vec3 getDerivative(Quaternion l) const;
    Vec3 getDerivative(Quaternion l, float dt) const;
    float getPitch() const;
    float getRoll() const;
    float getYaw() const;
    bool error() const;
    // mutating methods
    void nlerpWith(const Quaternion& l, float percentNew);
    void rotateBy(const Quaternion& l);
    void integrate(const Vec3& rotationalVelocity);
    void preintegrate(const Vec3& rotationalVelocity);
    void normalize();
    // operators
    float& operator[](int x); // this should be avoided
    Quaternion operator~(void) const;
    Quaternion operator-(void) const;
    void operator*=(float s);
    void operator/=(float s);
    void operator*=(const Quaternion& r);
    void operator+=(const Quaternion& r);
    void operator-=(const Quaternion& r);
    friend Quaternion operator*(const Quaternion& l, float s);
    friend Quaternion operator*(float s, const Quaternion& l);
    friend Quaternion operator/(const Quaternion& l, float s);
    friend Quaternion operator+(const Quaternion& l, const Quaternion& r);
    friend Quaternion operator-(const Quaternion& l, const Quaternion& r);
    friend class Vec3;
};
Quaternion operator*(const Quaternion& l, float s);
Quaternion operator*(float s, const Quaternion& l);
Quaternion operator/(const Quaternion& l, float s);
Quaternion operator+(const Quaternion& l, const Quaternion& r);
Quaternion operator-(const Quaternion& l, const Quaternion& r);
#endif
