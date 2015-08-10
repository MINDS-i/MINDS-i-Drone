#include "DroneLibs.h"

#include "util/PIDcontroller.h"
#include "util/PIDparameters.h"
#include "math/GreatCircle.h"

class Horizon : public FlightStrategy {
private:
    PIDcontroller  pitchPID, pError;
    PIDcontroller   rollPID, rError;
    PIDcontroller    yawPID, yError;
    float          throttle;
    float          pitch, roll, yaw;
    float          velFac, yawFac;
    float          tiltCompLimit;
    bool           standbyOn;
public:
    Horizon(PIDparameters* pitchI, PIDparameters* pitchO,
            PIDparameters*  rollI, PIDparameters*  rollO,
            PIDparameters*   yawI, PIDparameters*   yawO ) :
                pitchPID(pitchI), pError(pitchO),
                 rollPID( rollI), rError( rollO),
                  yawPID(  yawI), yError(  yawO) {}
    void update(OrientationEngine& orientation, float (&torques)[4]){
        if(standbyOn){
            torques[0] = 0;
            torques[1] = 0;
            torques[2] = 0;
            torques[3] = 0;
            return;
        }

        //calculate outer loop
        const float p = pError.update(orientation.getPitch() - pitch          );
        const float r = rError.update(orientation.getRoll()  - roll           );
        const float y = yError.update(distanceRadian(yaw,orientation.getYaw()));
        //set inner loops with outer calculations
        pitchPID.set(p);
        rollPID.set(r);
        yawPID.set(y);
        //set torques from inner PID loop calculations
        torques[0] = pitchPID.update(orientation.getPitchRate()*1024.f);//1024 from rad/millisecond
        torques[1] = rollPID.update(orientation.getRollRate()*1024.f);  //to rad/second
        torques[2] = yawPID.update(orientation.getYawRate()*1024.f);
        torques[3] = throttle;
    }
    void standby(){
        standbyOn = true;
    }
    void activate(){
        standbyOn = false;
        reset();
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
        if(standbyOn) activate();
        set(setps[0], setps[1], setps[2], setps[3]);
    }
    void set(float pitch, float roll, float yaw, float throttle){
        if(standbyOn) activate();
        this->pitch    = pitch;
        this->roll     = roll;
        this->yaw      = yaw;
        this->throttle = throttle;
    }
    float getTiltCompLimit(){
        return tiltCompLimit;
    }
    void setTiltCompLimit(float tcl){
        tiltCompLimit = tcl;
    }
};
