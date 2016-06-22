# MINDS-i-Drone
Advanced library for the [MINDS-i][3] Drone platform

To use the library, install it in your arduino *sketchbook/libraries* folder
manually or from within the arduino library manager.

Setting up your Quadcopter
--------------------------
1. Install your motors in the following configuration:
   ```
     ^ Forward ^
   .---.   .---.
   | 3 |   | 1 |
   .---\ ^ /---.
         XXX
         XXX
   .---/   \---.
   | 2 |   | 4 |
   .---.   .---.
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
   (the direction for clockwise rotation on the drone) for 2 seconds. If an error is
   detected, the quadcopter will refuse to arm.
5. The drone will calibrate its inertial sensors for 2 seconds, it must remain
   as still as possible during this time. It will then be ready to fly; the motors
   should begin spinning slowly.
6. To disarm, hold the throttle stick down and the yaw stick left (CCW).

+ To diagnose problems, connect the quadcopter to the [MINDS-i Dashboard][1] and read
  the error messages produced.

[1]: https://github.com/MINDS-i/Dashboard/releases
[2]: https://github.com/MINDS-i/Drone-Tests
[3]: http://mindsirobotics.com/
