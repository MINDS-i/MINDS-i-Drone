
#include "math/Waypoint.h"
#include "input/APM/LEA6H.h"
#include "util/PIDcontroller.h"
#include "util/PIDparameters.h"

// custom velocity from distance P controller
// pid velocity controller in real world coordinates

// find direction to travel in global frame
// find speed to travel in NS and EW directions
// pid controller target velocities NS EW
// translate output to local pitch/roll targets

// inputs: GPS, craft yaw
// outputs: pitch/roll targets

class PositionHold {
private:
    const float FEET_PER_MILE = 5280.0f;
    Waypoint target;
    PIDcontroller NS, EW;
    // maximum velocity in miles per hour
    float maxVelocity = 4.0;
    // velocity scale in (miles per hour) / (miles from target)
    float velocityScale = FEET_PER_MILE / 5.0;
    // Distance away from the target the quad needs to be to attempt movement
    float triggerDistance = 1.5 / FEET_PER_MILE;
public:
    PositionHold(PIDparameters* parameters):
        NS(parameters), EW(parameters) {}
    // Set the target waypoint to travel to and hover around
    void setTarget(Waypoint wp){
        target = wp;
    }
    // maximum velocity in miles per hour
    void setMaximumVelocityTarget(float mv) {
        maxVelocity = mv;
    }
    // Distance away from the target the quad needs to be to attempt movement
    void setTriggerDistance(float miles){
        triggerDistance = miles;
    }
    // velocity scale in (miles per hour) / (miles from target)
    void setVelocityScale(float vs){
        velocityScale = vs;
    }

    //
    float targetSpeed;
    float direction;
    float course;
    float speed;
    float targetNS;
    float targetEW;
    float speedNS;
    float speedEW;
    float NSoutput;
    float EWoutput;
    //

    struct Result{ float pitch; float roll; };
    Result update(LEA6H& gps, float yaw){
        Waypoint position = gps.getLocation();
        float distance = calcDistance(position, target);

        if(distance < triggerDistance){
            return { 0.0, 0.0 };
        }

        targetSpeed = min(distance*velocityScale, maxVelocity);
        direction = toRad(calcHeading(position, target));
        targetNS = targetSpeed*cos(direction);
        targetEW = targetSpeed*sin(direction);

        speed = gps.getGroundSpeed();
        course = toRad(gps.getCourse());
        speedNS = speed*cos(course);
        speedEW = speed*sin(course);

        NS.set(targetNS);
        EW.set(targetEW);
        NSoutput = NS.update(speedNS);
        EWoutput = EW.update(speedEW);

        float cs = cos(yaw);
        float sn = sin(yaw);
        float pitch = cs*NSoutput - sn*EWoutput;
        float roll  = cs*EWoutput + sn*EWoutput;

        return { pitch, roll };
    }
};
