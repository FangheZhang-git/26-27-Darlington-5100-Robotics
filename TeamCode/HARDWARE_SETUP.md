# Team 5100 TeleOp

Select **5100 TeleOp** on the Driver Station, then INIT and START.
Driving is robot-centric: forward always follows the robot front, and strafing
is relative to the robot, regardless of its heading. No heading reset is needed.
Forward/backward joystick polarity is inverted from the initial version based
on the physical driving check; strafe and turn inputs are unchanged.
Assign the movement controller
as gamepad 1 and the mechanism controller as gamepad 2 on the Driver Station.

| Driver | Control | Action |
| --- | --- | --- |
| 1 | Left stick | Robot-centric forward/backward and strafe |
| 1 | Right stick X | Turn, with 0.80 sensitivity |
| 2 | Left trigger | Intake forward at fixed 80% while held |
| 2 | Left bumper (hold) | Reverse intake at 60%; overrides left trigger |
| 2 | Right trigger | Shooter at selected flywheel RPM while held (starts at 300 RPM) |
| 2 | D-pad up/down (press) | Increase/decrease target flywheel RPM by 10, limited to 0-900 RPM |
| 2 | D-pad right/left (hold) | Increase/decrease Stick height-gate servo position |
| Either | B / Circle (hold) | Stop all motors and suspend gate adjustment; release resumes controls |
| Driver Station | STOP | End TeleOp and stop all motors |

Triggers act as on/off switches above 10% trigger travel, not power sliders.
Release a trigger to stop that motor. Shooter target resets to 300 wheel RPM
each run; up/down changes by 10 RPM once per new press, clipped to 0-900 RPM.
Holding an arrow does not repeat. Up and down together do nothing. Changing
the target does not start the shooter without holding RT. Either driver's B/Circle
stops all motors and suppresses adjustment; releasing resumes controller commands.

Drive wheel power is normalized and capped at 100%. There is no added joystick
dead zone, turbo, or separate slow mode. Drive and intake use BRAKE; the shooter
coasts when released. Drive and intake use RUN_WITHOUT_ENCODER; the shooter
uses RUN_USING_ENCODER and DcMotorEx.setVelocity() with encoder feedback.
The IMU is informational only. Options is unassigned. Telemetry shows motor
commands, intake percentage, target and measured flywheel RPM, shooter active/
ready status and gate command. Motor directions retain the physically reported
forward/backward, strafe and turn corrections.

## Shooter RPM and indicator

The photographed motor is goBILDA 5203-2402-0001: 6000 RPM at 12 V, 1:1 internal
ratio and 28 encoder counts per output revolution. The 14-tooth motor gear drives
a 28-tooth flywheel gear, so one flywheel revolution equals two motor revolutions:
56 encoder counts. Conversion: encoder ticks/second = wheel RPM * 56 / 60.
300 wheel RPM = 280 ticks/second; 900 wheel RPM = 840 ticks/second.
Signed encoder speed is used; reverse rotation cannot show ready.

Connect the shooter encoder to Control Hub encoder port 1. Configure the RGB
indicator as **Servo**, name **shootLight**, at Control Hub servo port 0.
The goBILDA PWM light uses an explicit 600-2400 us range, 1110 us for red and
1880 us for purple/violet (inside the manufacturer's solid-color boundaries).
It is purple when the shooter is enabled at a positive target, measured speed
is positive, and absolute error is <=100 wheel RPM. Otherwise it is red,
including INIT, zero target, trigger release, and B stop. At target 300, the
inclusive ready interval is 200-400 RPM. No extra settling delay is imposed.
At very low targets the requested +/-100 tolerance is proportionally broad.
STOP requests red, though the SDK may disable servo PWM after OpMode shutdown.

ShooterSettings.java holds conversion, RPM limits and initial PIDF gains.
P=10, I=3, D=0 and F=32767/2800 are starting values, not measured tuning.
The velocity loop adjusts motor effort rather than retaining the old 90% power
cap; 900 wheel RPM is the new command limit. Battery/load variation can still
make a target unreachable or cause transient speed error. Verify encoder sign,
RPM tracking and actual light color on the robot before relying on readiness.
The motor encoder infers wheel speed through the gear ratio; it cannot detect
mechanical slipping or disconnected gears. No automatic feeding is added.

Sources: [motor specifications](https://www.gobilda.com/content/user_manuals/5203-2402-0001_spec_sheet.pdf),
[indicator PWM chart (Product Insight 4)](https://www.gobilda.com/rgb-indicator-light-pwm-controlled/).

Stick is a positional servo controlling the intake opening to reject oversized
balls. It is not a shooter feeder. D-pad right/left adjusts at 0.25 normalized travel per
second and holds the last commanded position on release. Both left/right buttons
held together make no adjustment. No gate command is sent during INIT or START.
The first D-pad command starts near 0.50, because no measured position or known
mechanical endpoints were supplied. The command is limited to 0-1; that is the
servo command range, not a verified mechanical clearance range. Right increases
servo position; whether that physically raises the gate depends on the linkage.
No automatic ball-size detection is implemented because no sensor is specified.

## Existing robot configuration

Use the saved **fiftyoneconfig** configuration shown in the supplied photos.
Those photos show unsaved edits: press Done and Save if they remain unsaved.
Names below are case-sensitive. Java looks up these names; the active Robot
Controller configuration assigns the hub and port.

| Device | Hub | Port | Name |
| --- | --- | --- | --- |
| Right-front drive | Expansion | Motor 0 | `RF` |
| Right-back drive | Expansion | Motor 1 | `RB` |
| Left-front drive | Expansion | Motor 2 | `LF` |
| Left-back drive | Expansion | Motor 3 | `LB` |
| Intake | Control | Motor 0 | `Intake` |
| Shooter | Control | Motor 1 | `Shooter` |
| Height-gate servo | Control | Servo 3 | `Stick` |
| Shooter encoder | Control | Encoder 1 | Uses `Shooter` motor mapping |
| RGB indicator | Control | Servo 0 | `shootLight` |
| IMU | Control | Internal | `imu` |

The photos show GoBILDA 5202/3/4 series motor entries and a standard Servo entry
for Stick. IMU orientation follows the supplied reference: REV logo UP, USB
RIGHT. The old lift/grabber hardware and separate test OpModes are removed.

## Current deployment

The hub runs Robot Controller 12.0. The previously deployed version was compiled
using OnBot Java, preserving that app. This repository builds an SDK 11.2.1 APK
with a different signature; do not install it over the stock 12.0 app. For this
hub, upload RobotHardware.java, RobotTeleOp.java and ShooterSettings.java to OnBot Java and Build
Everything. Select **5100 TeleOp** after the build succeeds. Current source
changes are not on the hub until that upload/build completes.

## Android Studio deployment (separate installation)

The instructions below apply to a compatible Android Studio-managed installation,
not the hub's current stock 12.0 app.

Open this project in Android Studio, connect the powered Control Hub by USB,
select the TeamCode run configuration and the Control Hub device, then Run.
Alternatively install the already-built APK with Android platform tools:

```sh
adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
```

Keep the existing configuration, connect the Driver Station, select **5100
TeleOp**, press INIT, then START. Installation does not start the motors.
The APK is generated by `./gradlew :TeamCode:assembleDebug`; build output is not
committed. A successful build verifies software compilation, not physical motor
directions, gate clearance or shooting performance.
