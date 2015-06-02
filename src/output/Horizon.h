#include "DroneLibs.h"

#include "util/PIDcontroller.h"
#include "util/PIDparameters.h"

class Horizon : public FlightStrategy {
private:
    PIDcontroller pitchPID, rollPID;
    float         yaw, throttle;
    float         pitch, roll;
    float         velFac;
public:
    Horizon(PIDparameters* p, PIDparameters* r)
        : pitchPID(p), rollPID(r), velFac(1) {}
    Horizon(PIDparameters* p, PIDparameters* r, float velocityFactor)
        : pitchPID(p), rollPID(r), velFac(velocityFactor) {}
    void update(OrientationEngine& orientation, float (&torques)[4]){
        Quaternion attitude = orientation.getAttitude();
        float pError = velFac * (pitch - attitude.getPitch());
        float rError = velFac * (roll  - attitude.getRoll());
        pitchPID.set(pError);
        rollPID.set(rError);
        Vec3 rates = orientation.getRate();
        torques[0] = pitchPID.update(rates[0]*1024.f);//from rad/millisecond
        torques[1] = rollPID.update(rates[1]*1024.f);//to rad/second
        torques[2] = yaw;
        torques[3] = throttle;
    }
    void reset(){
        pitchPID.clearAccumulator();
        rollPID.clearAccumulator();
    }
    void set(float (&setps)[4]){
        set(setps[0], setps[1], setps[2], setps[3]);
    }
    void set(float pitch, float roll, float yawTorque, float throttle){
        this->pitch = pitch;
        this->roll = roll;
        yaw = yawTorque;
        this->throttle = throttle;
    }
    float getVelFac(){
        return velFac;
    }
    void setVelFac(float vf){
        velFac = vf;
    }
};
