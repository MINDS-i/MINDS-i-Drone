# MINDS-i-Drone
Advanced library for the [MINDS-i][3] Drone platform

To use the library, install it with the arduino library manager or manually add
it to your *sketchbook/libraries* folder.

Setting up your Quadcopter
--------------------------
1. Install your motors in the following configuration:
   ```
     ^ Forward ^
   .->-.   .-<-.
   | 3 |   | 1 |
   .-<-\ ^ /->-.
         XXX
         XXX
   .-<-/   \->-.
   | 2 |   | 4 |
   .->-.   .-<-.
  ```

   | Output Channel | Position      | Direction         |
   |:--------------:|:-------------:|:-----------------:|
   | 1              | Front Right   | Counter Clockwise |
   | 2              | Back Left     | Counter Clockwise |
   | 3              | Front Left    | Clockwise         |
   | 4              | Back Right    | Clockwise         |

2. Plug in your radio in the following configuration:

   | Action   | Input Channel | Receiver  |
   |:--------:|:-------------:|:---------:|
   | Pitch    | 1             | Elevator  |
   | Roll     | 2             | Aileron   |
   | Throttle | 3             | Throttle  |
   | Yaw      | 4             | Rudder    |
   | Mode     | 5             | Gear/Mode |

2. With your APM installed in the quadcopter, upload **CalibrateSensors.ino**
   *(examples/MINDS-i-Drone/CalibrateSensors)*, open
   the serial monitor, and follow the onscreen instructions.
3. Upload **CalibrateESCs.ino** *(examples/MINDS-i-Drone/CalibrateESCs)*, unplug the
   quadcopter, and power it on with a fully charged battery. Wait for the motors
   to finish arming, then unplug the battery.
4. Upload **quadcopter.ino** *(examples/MINDS-i-Drone/quadcopter)*. The drone's settings
   will be set to defaults the first time the quadcopter is run after
   **quadcopter.ino** is uploaded. They are persistent inbetween flights.

+ Never plug the quadcopter into the computer while propellers are installed.

Operating your Quadcopter
-------------------------
1. Turn on your radio transmitter.
2. Install a fully charged battery and set the quadcopter down.
3. Optionally, launch the [MINDS-i Dashboard][1] and connect it using a telemetry radio.
   The dashboard can be used to view, graph, and log live telemetry as well as
   configure the drone's onboard settings and display error messages.
4. To arm, hold the radio throttle stick down and the yaw stick all the way to the right
   for 2 seconds. If the quadcopter refuses to arm, check the error message
   display on the MINDS-i dashboard.
5. The drone will calibrate its inertial sensors for 2 seconds; it must remain
   as still as possible during this time. When it is ready to fly, the motors
   should begin to spin slowly.
6. To disarm, hold the throttle stick down and the yaw stick left.

+ When flying with the Mode/Gear switch down, you will have manual control over
  the quadcopter. With the gear switch up, it will switch to assisted mode.
  In assisted mode the quadcopter will hold its elevation automatically using
  the barometer; hold the throttle at 50% to keep its current elevation, move it
  up to ascend, and move it down to descend. If the "GPS assist" setting is enabled
  (true by default) then assisted mode will also attempt to hold the quadcopter
  in its current position laterally when the right radio stick is centered, giving
  manual control back when the right stick is moved away from its center position.

+ Always take off with the Mode/Gear switch down, and only use it to activate
  assisted flight mode when the quadcopter is stable and hovering

[1]: https://github.com/MINDS-i/Dashboard/releases
[2]: https://github.com/MINDS-i/Drone-Tests
[3]: http://mindsirobotics.com/

## Updates

When pushing updates, make sure to update the version in both [library.properties](library.properties) and [version.h](src/comms/version.h)