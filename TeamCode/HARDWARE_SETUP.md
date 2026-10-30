# Team 5100 TeleOp

For the OnBot Java upload, select **5100 RPM TeleOp** on the Driver Station,
then INIT and START. The Android Studio APK calls it **5100 TeleOp**.
OnBot deployment uses package `org.firstinspires.ftc.teamcode.rpm` for all three
Java files and the unique RPM TeleOp name to avoid the installed APK's classes.
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
| 2 | Right trigger | Shooter at selected flywheel RPM while held (starts at 1430 RPM) |
| 2 | D-pad up/down (press) | Increase/decrease target flywheel RPM by 50, limited to 0-2600 RPM |
| 2 | D-pad right/left (hold) | Quickly increase/decrease Stick height-gate servo position |
| Either | B / Circle (hold) | Stop all motors and suspend gate adjustment; release resumes controls |
| Driver Station | STOP | End TeleOp and stop all motors |

Triggers act as on/off switches above 10% trigger travel, not power sliders.
Release a trigger to stop that motor. Shooter target resets to 1430 wheel RPM
each run; up/down changes by 50 RPM once per new press, clipped to 0-2600 RPM.
Holding an arrow does not repeat. Up and down together do nothing. Changing
the target does not start the shooter without holding RT. Either driver's B/Circle
stops all motors and suppresses adjustment; releasing resumes controller commands.

Drive wheel power is normalized and capped at 100%. There is no added joystick
dead zone, turbo, or separate slow mode. Drive and intake use BRAKE; the shooter
coasts when released. Drive and intake use RUN_WITHOUT_ENCODER; the shooter
uses an application encoder-feedback velocity loop. RUN_WITHOUT_ENCODER disables
the hub's internal velocity regulator, but encoder velocity is still read on every
control cycle. Motor effort is calculated from RPM error, not selected by the driver.
The IMU is informational only. Options is unassigned. Telemetry shows motor
commands, intake percentage, target and measured flywheel RPM, shooter active/
ready status and gate command. Motor directions retain the physically reported
forward/backward, strafe and turn corrections.

## Shooter RPM and indicator

The photographed motor is goBILDA 5203-2402-0001: 6000 RPM at 12 V, 1:1 internal
ratio and 28 encoder counts per output revolution. The 14-tooth motor gear drives
a 28-tooth flywheel gear, so one flywheel revolution equals two motor revolutions:
56 encoder counts. Conversion: encoder ticks/second = wheel RPM * 56 / 60.
1430 wheel RPM = 1334.67 ticks/second; 2600 wheel RPM = 2426.67 ticks/second.
Signed encoder speed is used; reverse rotation cannot show ready.

Connect the shooter encoder to Control Hub encoder port 1. Configure the RGB
indicator as **Servo**, name **shootLight**, at Control Hub servo port 0.
The goBILDA PWM light uses an explicit 600-2400 us range, 1110 us for red and
1880 us for purple/violet (inside the manufacturer's solid-color boundaries).
It is purple when the shooter is enabled at a positive target, measured speed
is positive, and absolute error is <=50 wheel RPM. Otherwise it is red,
including INIT, zero target, trigger release, and B stop. At target 1430, the
inclusive ready interval is 1380-1480 RPM. No extra settling delay is imposed.
At very low targets the requested +/-50 tolerance is proportionally broad.
STOP requests red, though the SDK may disable servo PWM after OpMode shutdown.

### Recovery controller

ShooterSettings.java contains a voltage-compensated feedforward estimate and
an asymmetric, continuous piecewise-proportional correction inspired by flywheel
bang-bang control. This replaces the previous hub PIDF loop; the two controllers
are never run together. FLOAT remains enabled, and negative motor power is never
commanded. There is no integral accumulation, fixed recovery pause, or target ramp.

- At or below target minus 50 RPM, command 100% forward effort.
- At target, command nominal holding effort: `0.95 * target / 3000 * 12 / batteryV`.
- Between those points, blend linearly from full effort to holding effort.
- From target to target plus 50 RPM, blend down to zero; above that, coast.
- A zero target, trigger release or B stop commands zero effort.
- Invalid RPM samples command zero. Missing/invalid voltage uses 12 V fallback.

The 50 RPM controller transition band is a separate tuning constant from light
readiness. Feedforward is a nominal estimate, not a measured friction model;
small steady-state error and oscillations are possible. No algorithm guarantees
instant spin-up, zero droop during ball contact, or four identical uninterrupted
shots. If balls remove energy faster than the motor replaces it, mechanical
changes or spacing between balls are required. No automatic feeding is added.
Keep R2 held throughout a burst; releasing it stops the motor.

The same signed encoder sample drives effort, readiness, and displayed RPM.
Battery voltage is sampled every 100 ms. Telemetry/IMU work runs at most 10 Hz,
while shooter control runs every loop. Telemetry shows effort, loop duration,
and the last time spent outside tolerance before first re-entry (including
spin-up). That timer is not a ball counter or a guarantee of settled speed.
The motor encoder infers flywheel speed through the gear ratio and cannot detect
slipping gears. The nominal 2600 RPM command limit remains unchanged.

Validation: controller tests cover startup/recovery effort, voltage compensation,
invalid inputs, clamping, continuity, monotonicity, readiness boundaries and four
successive synthetic RPM dips. These are software tests, not physical testing.
Run from repository root:

```sh
mkdir -p /tmp/ftc-shooter-tests
javac -d /tmp/ftc-shooter-tests TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShooterSettings.java TeamCode/tests/ShooterSettingsTest.java
java -cp /tmp/ftc-shooter-tests ShooterSettingsTest
```

Before calling recovery validated, measure actual target/RPM/effort through a
four-ball burst at the desired spacing, with fresh and partially discharged
batteries. Persistent 100% effort while slow indicates insufficient available
motor output/load margin. Oscillation near target requires widening the controller
transition band or tuning feedforward from measured steady-state effort; do not
widen the purple tolerance to hide error.

Algorithm references: [WPILib asymmetric bang-bang and feedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/bang-bang.html),
[WPILib flywheel tuning and delay limits](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html),
[REV flywheel feedback tuning](https://docs.revrobotics.com/ftc-kickoff-concepts/decode-2025-26/programming-tips-and-tricks).

Sources: [motor specifications](https://www.gobilda.com/content/user_manuals/5203-2402-0001_spec_sheet.pdf),
[indicator PWM chart (Product Insight 4)](https://www.gobilda.com/rgb-indicator-light-pwm-controlled/).

Stick is a positional servo controlling the intake opening to reject oversized
balls. It is not a shooter feeder. Driver 2 D-pad right/left adjusts at 2.0 normalized travel per
second (eight times the old rate, full command range in 0.5 seconds) and holds
the last commanded position on release. Actual rotation speed is limited by
the servo and mechanical load; this setting does not increase its rated speed. Both left/right buttons
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

## Timed autonomous: 5100 Shoot and Back Up

Select **5100 Shoot and Back Up**. INIT initializes hardware with motors stopped;
the Driver Station START button begins the sequence:

| Time after START | Action |
| --- | --- |
| 0–5 seconds | Shooter regulates to the default 1430 wheel RPM; drive and intake stopped. |
| 5–20 seconds | Shooter continues; intake runs in reverse at 80% (power -0.80). |
| Next 1.5 seconds | Shooter and intake commands stop; drive backs up at the default 100% power. |
| Remainder of autonomous | All motor commands remain zero, waiting for TeleOp. |

The shooter feedback loop and red/purple ready indicator run throughout shooting.
Intake starts on the fixed five-second schedule, regardless of readiness. The
flywheel coasts after its power is removed. The gate receives no position command;
position it before the run. STOP interrupts every stage. Backward polarity matches
the physically corrected TeleOp mapping; this is timed open-loop movement, so the
distance depends on battery, traction and load. TeleOp must be selected/started
separately; this autonomous does not start it automatically.

Include `RobotTimedAuto.java` with the three shared source files when deploying
through OnBot Java, using the same package transformation as those files. This
adds a distinct autonomous entry and does not replace the existing Road Runner auto.

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
