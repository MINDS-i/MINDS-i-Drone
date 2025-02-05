
#include "input/GPS.h"
#include "math/Waypoint.h"
#include "util/PIDcontroller.h"
#include "util/PIDparameters.h"
#include "util/profile.h"

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
    Waypoint target;
    PIDcontroller NS, EW;
    // maximum velocity in miles per hour
    float maxVelocity = 4.0;
    // velocity scale in (miles per hour) / (miles from target)
    float velocityScale = Units::FEET_PER_MILE / 5.0;

  public:
    PositionHold(PIDparameters* parameters) : NS(parameters), EW(parameters) {}
    // Set the target waypoint to travel to and hover around
    void setTarget(Waypoint wp) { target = wp; }
    // maximum velocity in miles per hour
    void setMaximumVelocityTarget(float mv) { maxVelocity = mv; }
    // velocity scale in (miles per hour) / (miles from target)
    void setVelocityScale(float vs) { velocityScale = vs; }

    //
    float targetSpeed;
    float distance;
    float course;
    float speed;
    float targetNS;
    float targetEW;
    float speedNS;
    float speedEW;
    float NSoutput;
    float EWoutput;
    //

    struct Result {
        float pitch;
        float roll;
    };
    Result update(GPS& gps, float yaw) {
        /**
         * distances are in miles
         * angles are in radians, ccw positive, 0 = north, NED frame
         * time is in hours
         */

        // Recalculate output only when the gps reports new data
        static uint16_t lastIndex = -1;
        static Result output = {0.0, 0.0};
        if (gps.dataIndex() == lastIndex) {
            return output;
        }
        lastIndex = gps.dataIndex();

        Waypoint position = gps.getLocation();

        // target speed to destination
        distance = position.distanceTo(target);
        targetSpeed = min(distance * velocityScale, maxVelocity);

        // target direction and components to destination
        auto targetComponents = position.headingComponents(target);
        float mag = 1.0 / sqrt(sq(targetComponents.x) + sq(targetComponents.y));
        targetNS = targetSpeed * targetComponents.x * mag;
        targetEW = targetSpeed * targetComponents.y * mag;
        if (isnan(targetNS) || isnan(targetEW)) {
            // components end up NaN when exactly on the target location
            targetNS = 0.0;
            targetEW = 0.0;
        }

        // current speed components
        speed = gps.getGroundSpeed();
        course = toRad(gps.getCourse());
        speedNS = speed * cos(course);
        speedEW = speed * sin(course);

        // Update PID loops
        NS.set(targetNS);
        EW.set(targetEW);
        NSoutput = NS.update(speedNS);
        EWoutput = EW.update(speedEW);

        // Rotate output into local frame
        float cs = cos(-yaw);
        float sn = sin(-yaw);
        float pitch = -(+cs * NSoutput - sn * EWoutput); // negative pitch => north
        float roll = (+sn * NSoutput + cs * EWoutput);   // positive roll  => east

        // cache output and return
        output = {pitch, roll};
        return output;
    }
};
