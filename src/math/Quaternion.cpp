#include "Quaternion.h"
Quaternion::Quaternion(const Vec3& euler){
	float c1 = cos(euler.x/2.f); //pitch 
	float s1 = sin(euler.x/2.f);
	float c3 = cos(euler.y/2.f); //roll
	float s3 = sin(euler.y/2.f);
	float c2 = cos(euler.z/2.f); //yaw
	float s2 = sin(euler.z/2.f);
	float c1c2 = c1*c2;
	float s1s2 = s1*s2;
	w = c1c2*c3 -  s1s2*s3;
	x = c1c2*s3 +  s1s2*c3;
	y =s1*c2*c3 + c1*s2*s3;
	z =c1*s2*c3 - s1*c2*s3;
}
/*Quaternion::Quaternion(Vec3 reference, const Vec3& signal){
	//is this worth it?
}*/
Quaternion::Quaternion(const Vec3& axis, float angle){
	float s = sin(angle/2.f) / axis.length(); // built in normalization
	w = cos(angle/2.f);
	x = axis.x * s;
	y = axis.y * s;
	z = axis.z * s;
}
Quaternion
Quaternion::conjugate() const {
	return Quaternion(w, -x, -y, -z);
}
float
Quaternion::dot(const Quaternion& l) const {
	return w*l.w + x*l.x + y*l.y + z*l.z;
}
float
Quaternion::length() const {
	return sqrt(w*w + x*x + y*y + z*z);
}
float
Quaternion::distance(const Quaternion& l) const {
	float d = dot(l);
	return acos(2.f*d*d-1.f);
}
Vec3
Quaternion::axis() const {
	return Vec3(x,y,z);
}
Vec3
Quaternion::getEuler() const {
	return Vec3(getPitch(), getRoll(), getYaw());
}
float
Quaternion::getPitch() const {
	return asin  (2 * ((w*y) - (z*x)));
}
float
Quaternion::getRoll() const {
	return atan2 (2 * ((w*x) + (y*z)), 1 - 2 * ((x*x) + (y*y)));
}
float
Quaternion::getYaw() const {
	return atan2 (2 * ((w*z) + (x*y)), 1 - 2 * ((y*y) + (z*z)));
}
bool
Quaternion::error() const {
	return (isnormal(x) && isnormal(y) && isnormal(z) && isnormal(w));
}
void
Quaternion::nlerpWith(const Quaternion& l, float percentNew){
	float percentOld = 1.f - percentNew;
	w = percentOld * w + percentNew * l.w;
	x = percentOld * x + percentNew * l.x;
	y = percentOld * y + percentNew * l.y;
	z = percentOld * z + percentNew * l.z;
	normalize();
}
void
Quaternion::rotateBy(const Quaternion& l){
	float W = w;
	float X = x;
	float Y = y;
	float Z = z;
	w = W*l.w - X*l.x - Y*l.y - Z*l.z;
	x = W*l.x + X*l.w - Y*l.z - Z*l.y;
	y = W*l.y + X*l.z + Y*l.w - Z*l.x;
	z = W*l.z - X*l.y + Y*l.x - Z*l.w;
}
void
Quaternion::rotateByFast(const Vec3& euler){
	//uses small angle formulas implicitly
	//might need to be reduced by a foctor of 2 first
	float wi = (-x*euler.x - y*euler.y - z*euler.z);
	float xi = ( w*euler.x + y*euler.z - z*euler.y);
	float yi = ( w*euler.y - x*euler.z + z*euler.x);
	float zi = ( w*euler.z + x*euler.y - y*euler.x);
	w += wi;
	x += xi;
	y += yi;
	z += zi;
}
void
Quaternion::normalize(){
	float inv = 1/length();
	w *= inv;
	x *= inv;
	y *= inv;
	z *= inv;
}
float& 
Quaternion::operator[] (int index){
	switch(index){
		case 0: return w;
		case 1: return x;
		case 2: return y;
		case 3: return z;
	}
}
Quaternion  
Quaternion::operator ~ (void) const{
	return this->conjugate();
}
void		
Quaternion::operator*= (float s){
	w *= s;
	x *= s;
	y *= s;
	z *= s;
}
void		
Quaternion::operator/= (float s){
	w /= s;
	x /= s;
	y /= s;
	z /= s;
}
void		
Quaternion::operator*= (const Quaternion& r){
	rotateBy(r);
}
void		
Quaternion::operator+= (const Quaternion& r){
	w += r.w;
	x += r.x;
	y += r.y;
	z += r.z;
}
void		
Quaternion::operator-= (const Quaternion& r){
	w -= r.w;
	x -= r.x;
	y -= r.y;
	z -= r.z;
}
Quaternion
operator * (const Quaternion& l, float s){
	return Quaternion(l.w*s, l.x*s, l.y*s, l.z*s);
}
Quaternion
operator * (float s, const Quaternion& l){
	return Quaternion(l.w*s, l.x*s, l.y*s, l.z*s);
}
Quaternion
operator / (const Quaternion& l, float s){
	return Quaternion(l.w/s, l.x/s, l.y/s, l.z/s);
}
Quaternion
operator + (const Quaternion& l, const Quaternion& r){
	return Quaternion(l.w+r.w, l.x+r.x, l.y+r.y, l.z+r.z);
}
Quaternion
operator - (const Quaternion& l, const Quaternion& r){
	return Quaternion(l.w-r.w, l.x-r.x, l.y-r.y, l.z-r.z);
}
