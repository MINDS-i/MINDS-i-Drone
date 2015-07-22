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
    float          velFac;
    ThrottleCurve* tCurve;
public:
    float testPoint;
    Horizon(PIDparameters* p, PIDparameters* r, PIDparameters* y)
        : pitchPID(p), rollPID(r), yawPID(y), velFac(1) {}
    Horizon(PIDparameters* p, PIDparameters* r, PIDparameters* y, float velocityFactor)
        : pitchPID(p), rollPID(r), yawPID(y), velFac(velocityFactor) {}
    Horizon(PIDparameters* p, PIDparameters* r, PIDparameters* y, ThrottleCurve* tc)
        : pitchPID(p), rollPID(r), yawPID(y), velFac(1), tCurve(tc) {}
    void update(OrientationEngine& orientation, float (&torques)[4]){
        float pError = velFac * (pitch - orientation.getPitch());
        float rError = velFac * (roll  - orientation.getRoll());
        pitchPID.set(pError);
        rollPID.set(rError);
        torques[0] = pitchPID.update(orientation.getPitchRate()*1024.f);//1024 from rad/millisecond
        torques[1] = rollPID.update(orientation.getRollRate()*1024.f);  //to rad/second

        float wrappedYaw = simplifyRadian(yaw, orientation.getYaw());
        torques[2] = yawPID.update(wrappedYaw);

        torques[3] = throttle;
        /*
        Quaternion attitude = orientation.getAttitude();
        Vec3 down(0,0,1);
        down.rotateBy(attitude);
        float corr = max(down[2], 0.85); //capped at 115%
        torques[3] = throttle/corr;
        */
        testPoint = torques[2];
    }
    void reset(){
        pitchPID.clearAccumulator();
        rollPID.clearAccumulator();
    }
    void set(float (&setps)[4]){
        set(setps[0], setps[1], setps[2], setps[3]);
    }
    void set(float pitch, float roll, float yaw, float throttle){
        this->pitch = pitch;
        this->roll = roll;
        this->yaw = yaw;
        this->throttle = tCurve->get(throttle);
        yawPID.set(yaw);
    }
    float getVelFac(){
        return velFac;
    }
    void setVelFac(float vf){
        velFac = vf;
    }
    void setThrottleCurve(ThrottleCurve* tc){
        tCurve = tc;
    }
};
