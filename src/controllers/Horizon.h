#include "util/PIDexternaltime.h"
#include "util/PIDparameters.h"
#include "math/GreatCircle.h"
#include "output/FlightStrategy.h"

class Horizon : public FlightStrategy {
private:
    PIDexternaltime pitchPID, pError;
    PIDexternaltime  rollPID, rError;
    PIDexternaltime   yawPID, yError;
    float pitch, roll, yaw, throttle;
    float tiltCompLimit;
public:
    Horizon(PIDparameters* pitchI, PIDparameters* pitchO,
            PIDparameters*  rollI, PIDparameters*  rollO,
            PIDparameters*   yawI, PIDparameters*   yawO ) :
                pitchPID(pitchI), pError(pitchO),
                 rollPID( rollI), rError( rollO),
                  yawPID(  yawI), yError(  yawO) {}
    void update(OrientationEngine& orientation, float ms, float (&torques)[4]){
        //calculate outer loop
        float p = pError.update(orientation.getPitch() - pitch, ms);
        float r = rError.update(orientation.getRoll() - roll, ms);
        float y = yError.update(distanceRadian(yaw,orientation.getYaw()), ms);
        //set inner loops with outer calculations
        pitchPID.set(p);
        rollPID.set(r);
        yawPID.set(y);
        //set torques from inner PID loop calculations
        torques[0] = pitchPID.update(orientation.getPitchRate()*1024.f,ms);//1024 from rad/millisecond
        torques[1] = rollPID.update(orientation.getRollRate()*1024.f,ms);  //to rad/second
        torques[2] = yawPID.update(orientation.getYawRate()*1024.f,ms);
        torques[3] = throttle;
    }
    void reset(){
        pitchPID.clearAccumulator();
        pitchPID.set(0);
        pError.clearAccumulator();
        pError.set(0);
        rollPID.clearAccumulator();
        rollPID.set(0);
        rError.clearAccumulator();
        rError.set(0);
        yawPID.clearAccumulator();
        yawPID.set(0);
        yError.clearAccumulator();
        yError.set(0);
    }
    void set(float (&setps)[4]){
        set(setps[0], setps[1], setps[2], setps[3]);
    }
    void set(float pitch, float roll, float yaw, float throttle){
        this->pitch    = pitch;
        this->roll     = roll;
        this->yaw      = yaw;
        this->throttle = min(throttle, 1.0);
    }
    float getTiltCompLimit(){
        return tiltCompLimit;
    }
    void setTiltCompLimit(float tcl){
        tiltCompLimit = tcl;
    }
};
