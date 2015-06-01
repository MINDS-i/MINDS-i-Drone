#include "DroneLibs.h"

#include "util/PIDcontroller.h"
#include "util/PIDparameters.h"

class Horizon : public FlightStrategy {
private:
    PIDcontroller pitchPID, rollPID;
    float         yaw, throttle;
public:
    Horizon(PIDparameters* p, PIDparameters* r)
        : pitchPID(p), rollPID(r) {}
    void update(OrientationEngine& orientation, float (&torques)[4]){
        Quaternion attitude = orientation.getAttitude();
        torques[0] = pitchPID.update(attitude.getPitch());
        torques[1] = rollPID.update(attitude.getRoll());
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
        pitchPID.set(pitch);
        rollPID.set(roll);
        yaw = yawTorque;
        this->throttle = throttle;
    }
};
