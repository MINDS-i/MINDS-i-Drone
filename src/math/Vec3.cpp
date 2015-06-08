#include "Vec3.h"
float
Vec3::length() const {
	return sqrt(x*x + y*y + z*z);
}
float
Vec3::distance(const Vec3& l) const {
	return (*this-l).length();
}
float
Vec3::dot(const Vec3& l) const {
	return x*l.x + y*l.y + z*l.z;
}
bool
Vec3::error() const {
	return !(isfinite(x) && isfinite(y) && isfinite(z));
}
void
Vec3::crossWith(const Vec3& l) {
	float X = x;
	float Y = y;
	float Z = z;
	x = Y*l.z - Z*l.y;
	y = Z*l.x - X*l.z;
	z = X*l.y - Y*l.x;
}
void
Vec3::normalize(){
	float inv = 1.f / length();
	x *= inv;
	y *= inv;
	z *= inv;
}
void
Vec3::lerpWith(const Vec3& l, float percentNew){
	float percentOld = 1.f - percentNew;
	x = percentOld * x + percentNew * l.x;
	y = percentOld * y + percentNew * l.y;
	z = percentOld * z + percentNew * l.z;
}
void
Vec3::rotateBy(const Quaternion& q){
	Vec3 t = *this;
	t.crossWith(q.axis());
	t*=2;
	*this += t*q.w;
	t.crossWith(q.axis());
	*this += t;
}
float&
Vec3::operator[] (int index){
	switch(index){
		case 0: return x;
		case 1: return y;
		case 2: return z;
		default: return 0.0;
	}
}
void
Vec3::operator*= (float s){
	x *= s;
	y *= s;
	z *= s;
}
void
Vec3::operator/= (float s){
	x /= s;
	y /= s;
	z /= s;
}
void
Vec3::operator+= (const Vec3& r){
	x += r.x;
	y += r.y;
	z += r.z;
}
void
Vec3::operator-= (const Vec3& r){
	x -= r.x;
	y -= r.y;
	z -= r.z;
}
Vec3
operator * (float s, const Vec3& v){
	return Vec3(v.x*s, v.y*s, v.z*s);
}
Vec3
operator * (const Vec3& v, float s){
	return Vec3(v.x*s, v.y*s, v.z*s);
}
Vec3
operator / (const Vec3& l, float s){
	return Vec3(l.x/s, l.y/s, l.z/s);
}
Vec3
operator + (const Vec3& l, const Vec3& r){
	return Vec3(l.x+r.x, l.y+r.y, l.z+r.z);
}
Vec3
operator - (const Vec3& l, const Vec3& r){
	return Vec3(l.x-r.x, l.y-r.y, l.z-r.z);
}

