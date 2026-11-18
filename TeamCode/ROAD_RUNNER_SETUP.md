# Team 5100: Road Runner two dead wheels

Installed from the official [Road Runner quickstart](https://github.com/acmerobotics/road-runner-quickstart), commit `6e63a7792e9bb6958798bf46fc03d84765b50c51` (retrieved November 17, 2026). License: ROAD_RUNNER_LICENSE.txt.
Road Runner core/actions 1.0.1, FTC utilities 0.1.25, Dashboard 0.5.1.

## Before tuning

1. Open this project in Android Studio, sync Gradle, then build/deploy TeamCode.
2. Encoder names are configured in `TwoDeadWheelLocalizer.java`: **parallel = Expansion Hub port 1 (`RB`)**, **perpendicular = Expansion Hub port 0 (`RF`)**. Plug the dead-wheel cables into those motor encoder sockets and keep the existing motor names in the Robot Controller configuration. These sockets read the dead wheels, not the drive motor rotation.
3. Drive names and directions reuse RobotHardware: LF/LB/RF/RB; IMU is `imu`, logo UP, USB RIGHT. Verify the physical mounting and motor directions; the existing directions are not measured. No RobotHardware instance is constructed by the Road Runner drive, so tuning does not require the intake/servos/shooter.
4. In the Driver Station's `quickstart` group, run `MecanumMotorDirectionDebugger` to check each wheel. Fix directions in RobotHardware so both TeleOp and Road Runner agree.
5. Run `DeadWheelDirectionDebugger`; forward travel must increase parallel ticks, left travel must increase perpendicular ticks. Change the corresponding direction constants in TwoDeadWheelLocalizer. Encoder directions are independent of motor directions.

The drive is already set to `TwoDeadWheelLocalizer`, and the tuning OpModes directly use MecanumDrive. Both pods must have the same distance-per-tick scaling with this stock localizer. Parallel high-rate encoders should use hub port 0 or 3 when practical.

## Tune in order

Follow the official [1.0 tuning guide](https://rr.brott.dev/docs/v1-0/tuning/) for detailed procedures and graph fitting. Do not use the older 0.5 DriveConstants/trajectory-sequence instructions.

| Step | OpMode | Save results in |
| --- | --- | --- |
| 1 | ForwardPushTest | MecanumDrive.PARAMS.inPerTick: measured inches / ticks |
| 2 | ForwardRampLogger | MecanumDrive.PARAMS.kS and kV |
| 3 | LateralRampLogger | MecanumDrive.PARAMS.lateralInPerTick |
| 4 | AngularRampLogger | MecanumDrive.PARAMS.trackWidthTicks; TwoDeadWheelLocalizer.PARAMS.parYTicks and perpXTicks |
| 5 | ManualFeedforwardTuner | Refine kS, kV, kA |
| 6 | ManualFeedbackTuner | axialGain, lateralGain, headingGain; adjust motion constraints to your robot |
| 7 | LocalizationTest and SplineTest | Verify measured pose and trajectory tracking |

Skip LateralPushTest for dead wheels. OTOS-specific tuners are not registered. Unused tank, three-wheel, OTOS, Pinpoint, and drive-encoder localization code has been removed.

On the Control Hub Wi-Fi:

- [Dashboard](http://192.168.43.1:8080/dash)
- [Forward ramp](http://192.168.43.1:8080/tuning/forward-ramp.html)
- [Lateral ramp](http://192.168.43.1:8080/tuning/lateral-ramp.html)
- [Dead-wheel angular ramp](http://192.168.43.1:8080/tuning/dead-wheel-angular-ramp.html)

The upstream numeric defaults are **untuned placeholders**, not robot measurements. Do the push/direction tests before powered ramp tests. Provide room for the selected routine and stop before reaching obstacles. Save Dashboard changes back into the Java Params fields; Dashboard edits are not persistent. Stop/reinitialize an OpMode after changing scale, geometry, or constraints because some values are captured at construction.

## Program autonomous

Edit `RoadRunnerAuto.java`. After tuning, remove `@Disabled` to show **Road Runner Starter Auto**. It drives forward 24 inches, waits, and returns. Road Runner coordinates use inches, radians, +X forward, +Y left, and counterclockwise positive heading. These conventions differ from the existing RobotHardware right/clockwise drive arguments.

Use `drive.actionBuilder(startPose)` with `strafeTo`, `splineTo`, `turn`, and `waitSeconds`, then execute with `Actions.runBlocking`. See [Actions](https://rr.brott.dev/docs/v1-0/actions/) and [builder reference](https://rr.brott.dev/docs/v1-0/builder-ref/). Start pose must match the robot's placement. Do not run other code that commands drive motors or resets the IMU during a trajectory.
