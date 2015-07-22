#include "DroneLibs.h"

#include "util/PIDcontroller.h"
#include "util/PIDparameters.h"
#include "math/GreatCircle.h"

class Horizon : public FlightStrategy {
private:
    PIDcontroller  pitchPID, rollPID;
    PIDcontroller  yawPID;
    float          throttle;
    float          pitch, roll, yaw;
    float          velFac, yawFac;
    float          tiltCompLimit;
    ThrottleCurve* tCurve;
public:
    float testPoint[4];
    Horizon(PIDparameters* p, PIDparameters* r, PIDparameters* y)
        : pitchPID(p), rollPID(r), yawPID(y),
          velFac(1), yawFac(1), tiltCompLimit(1) {}
    Horizon(PIDparameters* p, PIDparameters* r, PIDparameters* y, float velocityFactor)
        : pitchPID(p), rollPID(r), yawPID(y),
          velFac(velocityFactor), yawFac(1), tiltCompLimit(1) {}
    Horizon(PIDparameters* p, PIDparameters* r, PIDparameters* y, ThrottleCurve* tc)
        : pitchPID(p), rollPID(r), yawPID(y),
          velFac(1), yawFac(1), tiltCompLimit(1), tCurve(tc) {}
    void update(OrientationEngine& orientation, float (&torques)[4]){
        float pError = velFac * (pitch - orientation.getPitch());
        float rError = velFac * (roll  - orientation.getRoll());
        float yError = yawFac * distanceRadian(orientation.getYaw(), yaw);
        pitchPID.set(pError);
        rollPID.set(rError);
        yawPID.set(yError);
        torques[0] = pitchPID.update(orientation.getPitchRate()*1024.f);//1024 from rad/millisecond
        torques[1] = rollPID.update(orientation.getRollRate()*1024.f);  //to rad/second
        torques[2] = yawPID.update(orientation.getYawRate()*1024.f);
        //torques[3] = throttle;
        Quaternion attitude = orientation.getAttitude();
        Vec3 down(0,0,1);
        down.rotateBy(attitude);
        float corr = max(down[2], tiltCompLimit); //capped at 115%
        torques[3] = throttle/corr;

        testPoint[0] = yaw;
        testPoint[1] = orientation.getYaw();
        testPoint[2] = yError;
        testPoint[3] = torques[2];


    }
    void reset(){
        pitchPID.clearAccumulator();
        rollPID.clearAccumulator();
    }
    void set(float (&setps)[4]){
        set(setps[0], setps[1], setps[2], setps[3]);
    }
    void set(float pitch, float roll, float yaw, float throttle){
        this->pitch    = pitch;
        this->roll     = roll;
        this->yaw      = yaw;
        this->throttle = tCurve->get(throttle);
    }
    float getVelFac(){
        return velFac;
    }
    void setVelFac(float vf){
        velFac = vf;
    }
    float getYawFac(){
        return velFac;
    }
    void setYawFac(float vf){
        velFac = vf;
    }
    float getTiltCompLimit(){
        return tiltCompLimit;
    }
    void setTiltCompLimit(float tcl){
        tiltCompLimit = tcl;
    }
    void setThrottleCurve(ThrottleCurve* tc){
        tCurve = tc;
    }
};
