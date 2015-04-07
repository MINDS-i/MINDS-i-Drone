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
Quaternion::Quaternion(const Vec3& axis, float angle){
	float s = sin(angle/2.f) / axis.length(); // built in normalization
	w = cos(angle/2.f);
	x = axis.x * s;
	y = axis.y * s;
	z = axis.z * s;
}
Quaternion::Quaternion(const Vec3& ref, const Vec3& vec){
	float k_cos_theta = ref.dot(vec);
	float k = sqrt(ref.dot(ref)*vec.dot(vec));

	static const float fuzzFactor = .9999;
	if (k_cos_theta/k <= -fuzzFactor) {
	    // 180 degree rotation around any orthogonal vector
	    Vec3 other = (fabs(vec.x) < fuzzFactor) ? Vec3(1,0,0) : Vec3(0,1,0);
	    other.crossWith(vec);
	    //build using axis angle with angle = M_PI
	    other.normalize();
		w = 0.0f;
		x = other.x;
		y = other.y;
		z = other.z;
	}

	Vec3 cross = vec;
	cross.crossWith(ref);
	w = k_cos_theta + k;
	x = cross[0];
	y = cross[1];
	z = cross[2];
	normalize();
}
Quaternion
Quaternion::inverse() const {
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
	float arg = 2.f*d*d-1.f;
	if(fabs(arg)>=0.999999f) return 0;
	return acos(2.f*d*d-1.f);
}
Vec3
Quaternion::axis() const {
	return Vec3(x,y,z);
}
/**
 * The derivative from this to "l" if the time passed was one unit
 * presented in inertial rotations (gyro angles)
 * The angle between (*this) and l must be small
 */
Vec3
Quaternion::getDerivative(Quaternion l) const {
	Quaternion der = l;
	der.rotateBy(~(*this));
	Vec3 rate(der[1],der[2],der[3]);
	rate *= 2;
	return rate;
}
Vec3
Quaternion::getDerivative(Quaternion l, float dt) const {
	Quaternion der = l;
	der.nlerpWith((*this), (1.f-dt));
	der.rotateBy(~(*this));
	Vec3 rate(der[1],der[2],der[3]);
	rate *= 2;
	return rate;
}
float
Quaternion::getPitch() const {
	return asin (2 * ((w*y) - (z*x)));
}
float
Quaternion::getRoll() const {
	return asin (2 * ((y*z) + (x*w)));
}
float
Quaternion::getYaw() const {
	return atan2 (2 * ((z*w) - (x*y)), 1 - 2 * ((z*z) + (y*y)));
}
bool
Quaternion::error() const {
	return !(isfinite(x) && isfinite(y) && isfinite(z) && isfinite(w));
}
void
Quaternion::nlerpWith(const Quaternion& l, float percentNew){
	float percentOld = 1.f - percentNew;
	if(dot(l) < 0.0f) percentNew *= -1; //lerp with the right "polarity"
	w = percentOld * w + percentNew * l.w;
	x = percentOld * x + percentNew * l.x;
	y = percentOld * y + percentNew * l.y;
	z = percentOld * z + percentNew * l.z;
	normalize();
}
void
Quaternion::rotateBy(const Quaternion& l){
	//the hamilton product
	float W = w;
	float X = x;
	float Y = y;
	float Z = z;
	w = W*l.w - X*l.x - Y*l.y - Z*l.z;
	x = W*l.x + X*l.w + Y*l.z - Z*l.y;
	y = W*l.y - X*l.z + Y*l.w + Z*l.x;
	z = W*l.z + X*l.y - Y*l.x + Z*l.w;
}
void
Quaternion::integrate(const Vec3& rotVel){
	//integrates rotational velocities
	/*
	effectivly this:
    Quaternion delta(0,
		    	       (rate[0]),
			           (rate[1]),
			           (rate[2]) );
	delta/=2;
	delta.rotateBy(attitude); //Must be premultiply
	attitude += delta;
	*/
	float W = w/2;
	float X = x/2;
	float Y = y/2;
	float Z = z/2;
	w += -X*rotVel.x - Y*rotVel.y - Z*rotVel.z;
	x +=  W*rotVel.x + Y*rotVel.z - Z*rotVel.y;
	y +=  W*rotVel.y - X*rotVel.z + Z*rotVel.x;
	z +=  W*rotVel.z + X*rotVel.y - Y*rotVel.x;
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
	return this->inverse();
}
Quaternion
Quaternion::operator - (void) const{
	return Quaternion(-w,-x,-y,-z);
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
