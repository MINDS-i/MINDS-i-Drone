#include "DroneLibs.h"

#include "util/PIDcontroller.h"
#include "util/PIDparameters.h"

class Horizon : public FlightStrategy {
private:
    PIDcontroller pitchPID, rollPID;
    PIDcontroller yawPID;
    float         throttle;
    float         pitch, roll;
    float         velFac;
public:
    float testPoint;
    Horizon(PIDparameters* p, PIDparameters* r, PIDparameters* y)
        : pitchPID(p), rollPID(r), yawPID(y), velFac(1) {}
    Horizon(PIDparameters* p, PIDparameters* r, PIDparameters* y, float velocityFactor)
        : pitchPID(p), rollPID(r), yawPID(y), velFac(velocityFactor) {}
    void update(OrientationEngine& orientation, float (&torques)[4]){
        Quaternion attitude = orientation.getAttitude();
        float pError = velFac * (pitch - attitude.getPitch());
        float rError = velFac * (roll  - attitude.getRoll());
        pitchPID.set(pError);
        rollPID.set(rError);
        torques[0] = pitchPID.update(orientation.getPitchRate()*1024.f);//1024 from rad/millisecond
        torques[1] = rollPID.update(orientation.getRollRate()*1024.f);  //to rad/second
        torques[2] = yawPID.update(-orientation.getYawRate()*256.f);
        torques[3] = throttle;
        testPoint = torques[2];
    }
    void reset(){
        pitchPID.clearAccumulator();
        rollPID.clearAccumulator();
    }
    void set(float (&setps)[4]){
        set(setps[0], setps[1], setps[2], setps[3]);
    }
    void set(float pitch, float roll, float yawRPM, float throttle){
        this->pitch = pitch;
        this->roll = roll;
        yawPID.set(yawRPM);
        this->throttle = throttle;
    }
    float getVelFac(){
        return velFac;
    }
    void setVelFac(float vf){
        velFac = vf;
    }
};
