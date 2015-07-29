#ifndef VEC3_H
#define VEC3_H
#include "Quaternion.h"
#include "SpatialMath.h"
class Quaternion;
class Vec3{
private:
	union {
		struct {
			float x,y,z;
		};
		float data[3];
	};
	//float x,y,z;
public:
	Vec3(): x(0.f), y(0.f), z(0.f) {}
	Vec3(float X, float Y, float Z): x(X), y(Y), z(Z) {}
	//const methods
	float	length() const;
	float	distance(const Vec3& l) const;
	float	dot(const Vec3& l) const;
	bool	error() const;
	//mutating methods
	void	crossWith(const Vec3& l);
	void 	normalize();
	void	lerpWith(const Vec3& l, float percentNew);
	void	rotateBy(const Quaternion& l);
	//operators
	float& operator[] (int index){ return data[index]; }
	void	operator*= (float s);
	void	operator/= (float s);
	void	operator+= (const Vec3& r);
	void	operator-= (const Vec3& r);
	friend class Quaternion;
	friend Vec3 operator * (float s, const Vec3& v);
	friend Vec3 operator * (const Vec3& v, float s);
	friend Vec3 operator / (const Vec3& l, float s);
	friend Vec3 operator + (const Vec3& l, const Vec3& r);
	friend Vec3 operator - (const Vec3& l, const Vec3& r);
};
Vec3	operator * (float s, const Vec3& v);
Vec3	operator * (const Vec3& v, float s);
Vec3	operator / (const Vec3& l, float s);
Vec3	operator + (const Vec3& l, const Vec3& r);
Vec3	operator - (const Vec3& l, const Vec3& r);
#endif
