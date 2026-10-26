package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Robot-centric drive, fixed intake and encoder-regulated shooter RPM, and adjustable intake-height gate. */
@TeleOp(name = "5100 TeleOp", group = "5100")
public final class RobotTeleOp extends LinearOpMode {
    private static final double DRIVE_SPEED = 1.0;
    private static final double TURN_SENSITIVITY = 0.80;
    private static final double INTAKE_FORWARD_POWER = 0.80;
    private static final double TRIGGER_THRESHOLD = 0.10;
    private static final double INTAKE_REVERSE_POWER = 0.60;
    // Normalized servo travel per second. Right increases position; left decreases it.
    private static final double STICK_RATE = 0.25;
    private static final double STICK_INITIAL_POSITION = 0.50;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        try {
            telemetry.addLine("5100 ready. Robot-centric: forward follows the robot front.");
            telemetry.addLine("Driver 1: left stick move | right stick turn");
            telemetry.addLine("Driver 2: LT intake 80% | LB reverse 60% | RT shooter (starts at 1500 wheel RPM)");
            telemetry.addLine("Driver 2: up/down shooter +/-50 RPM (0-2600) | left/right gate");
            telemetry.addLine("Either driver: hold B/Circle to stop all motors.");
            telemetry.addLine("Gate stays uncommanded until D-pad input; first command starts near 0.50.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            boolean upWasPressed = gamepad2.dpad_up;
            boolean downWasPressed = gamepad2.dpad_down;
            int shooterRpm = ShooterSettings.INITIAL_RPM;
            // A servo has no actual-position feedback. Do not treat getPosition() as a measurement.
            double stickPosition = STICK_INITIAL_POSITION;
            boolean stickCommanded = false;
            ElapsedTime loopTimer = new ElapsedTime();
            while (opModeIsActive()) {
                // Cap elapsed time so a stalled loop cannot cause a large gate adjustment.
                double dt = Range.clip(loopTimer.seconds(), 0, 0.05);
                loopTimer.reset();
                boolean stopHeld = gamepad1.b || gamepad2.b;
                boolean upPressed = gamepad2.dpad_up;
                boolean downPressed = gamepad2.dpad_down;
                // One adjustment per new press. Opposite buttons together do nothing.
                if (!stopHeld && !(upPressed && downPressed)) {
                    if (upPressed && !upWasPressed) {
                        shooterRpm = Math.min(ShooterSettings.MAX_RPM, shooterRpm + ShooterSettings.STEP_RPM);
                    } else if (downPressed && !downWasPressed) {
                        shooterRpm = Math.max(0, shooterRpm - ShooterSettings.STEP_RPM);
                    }
                }
                upWasPressed = upPressed;
                downWasPressed = downPressed;

                if (stopHeld) {
                    robot.stopAll();
                } else {
                    // No application dead zone: pass proportional SDK stick values directly.
                    // Forward/backward polarity corrected from the physical driving check.
                    robot.driveRobotCentric(gamepad1.left_stick_y, -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * TURN_SENSITIVITY, DRIVE_SPEED);
                    robot.setIntakePower(gamepad2.left_bumper
                            ? -INTAKE_REVERSE_POWER
                            : (gamepad2.left_trigger > TRIGGER_THRESHOLD ? INTAKE_FORWARD_POWER : 0));
                    robot.setShooterRpm(gamepad2.right_trigger > TRIGGER_THRESHOLD ? shooterRpm : 0);
                    int gateDirection = (gamepad2.dpad_right ? 1 : 0) - (gamepad2.dpad_left ? 1 : 0);
                    if (gateDirection != 0) {
                        stickPosition = Range.clip(stickPosition + gateDirection * STICK_RATE * dt, 0, 1);
                        robot.setStickPosition(stickPosition);
                        stickCommanded = true;
                    }
                }

                double measuredRpm = robot.getShooterRpm();
                boolean shooterReady = robot.updateShooterLight(measuredRpm);
                telemetry.addData("Mode", stopHeld ? "Motors stopped (B held)" : "Robot-centric / 100% drive");
                telemetry.addLine("Driver 1: sticks drive | Either B/Circle stops motors");
                telemetry.addData("IMU heading (info only)", "%.1f degrees", Math.toDegrees(robot.headingRadians()));
                telemetry.addData("LF / RF", "%.2f / %.2f", robot.leftFront.getPower(), robot.rightFront.getPower());
                telemetry.addData("LB / RB", "%.2f / %.2f", robot.leftBack.getPower(), robot.rightBack.getPower());
                double intakePower = robot.intake.getPower();
                telemetry.addData("Intake power", "%.0f%% (%s)", Math.abs(intakePower) * 100,
                        intakePower > 0 ? "forward" : intakePower < 0 ? "reverse" : "stopped");
                telemetry.addData("Shooter target (wheel RPM)", shooterRpm);
                telemetry.addData("Shooter measured (wheel RPM)", "%.1f", measuredRpm);
                telemetry.addData("Shooter error (RPM)", "%.1f", shooterRpm - measuredRpm);
                telemetry.addData("Shooter battery (V)", "%.2f", robot.getShooterBatteryVoltage());
                telemetry.addData("Shooter active", !stopHeld && gamepad2.right_trigger > TRIGGER_THRESHOLD && shooterRpm > 0);
                telemetry.addData("Shooter ready", shooterReady ? "PURPLE - within +/-100 RPM" : "RED - not ready");
                telemetry.addData("Gate command", stickCommanded ? String.format(java.util.Locale.US, "%.3f", stickPosition) : "Not positioned (D-pad left/right to adjust)");
                telemetry.addLine("Driver 2: LT intake | LB reverse | RT shooter | Up/down RPM | Left/right gate");
                telemetry.update();
                idle();
            }
        } finally {
            robot.stopAll();
        }
    }
}
