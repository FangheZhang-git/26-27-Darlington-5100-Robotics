package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Robot-centric drive, fixed intake and selectable shooter power, and adjustable intake-height gate. */
@TeleOp(name = "5100 TeleOp", group = "5100")
public final class RobotTeleOp extends LinearOpMode {
    private static final double DRIVE_SPEED = 0.80;
    private static final double TURN_SENSITIVITY = 0.60;
    private static final double INTAKE_FORWARD_POWER = 0.80;
    private static final double TRIGGER_THRESHOLD = 0.10;
    private static final int SHOOTER_INITIAL_PERCENT = 85;
    private static final int SHOOTER_STEP_PERCENT = 10;
    private static final int SHOOTER_MAX_PERCENT = 90;
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
            telemetry.addLine("Driver 2: LT intake 80% | LB reverse 60% | RT shooter (starts at 85%)");
            telemetry.addLine("Driver 2: up/down shooter +/-10 points (0-90%) | left/right gate");
            telemetry.addLine("Either driver: hold B/Circle to stop all motors.");
            telemetry.addLine("Gate stays uncommanded until D-pad input; first command starts near 0.50.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            boolean upWasPressed = gamepad2.dpad_up;
            boolean downWasPressed = gamepad2.dpad_down;
            int shooterPercent = SHOOTER_INITIAL_PERCENT;
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
                        shooterPercent = Math.min(SHOOTER_MAX_PERCENT, shooterPercent + SHOOTER_STEP_PERCENT);
                    } else if (downPressed && !downWasPressed) {
                        shooterPercent = Math.max(0, shooterPercent - SHOOTER_STEP_PERCENT);
                    }
                }
                upWasPressed = upPressed;
                downWasPressed = downPressed;

                if (stopHeld) {
                    robot.stopAll();
                } else {
                    // No application dead zone: pass proportional SDK stick values directly.
                    robot.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x * TURN_SENSITIVITY, DRIVE_SPEED);
                    robot.setIntakePower(gamepad2.left_bumper
                            ? -INTAKE_REVERSE_POWER
                            : (gamepad2.left_trigger > TRIGGER_THRESHOLD ? INTAKE_FORWARD_POWER : 0));
                    robot.setShooterPower(gamepad2.right_trigger > TRIGGER_THRESHOLD ? shooterPercent / 100.0 : 0);
                    int gateDirection = (gamepad2.dpad_right ? 1 : 0) - (gamepad2.dpad_left ? 1 : 0);
                    if (gateDirection != 0) {
                        stickPosition = Range.clip(stickPosition + gateDirection * STICK_RATE * dt, 0, 1);
                        robot.setStickPosition(stickPosition);
                        stickCommanded = true;
                    }
                }

                telemetry.addData("Mode", stopHeld ? "Motors stopped (B held)" : "Robot-centric / 80% drive");
                telemetry.addLine("Driver 1: sticks drive | Either B/Circle stops motors");
                telemetry.addData("IMU heading (info only)", "%.1f degrees", Math.toDegrees(robot.headingRadians()));
                telemetry.addData("LF / RF", "%.2f / %.2f", robot.leftFront.getPower(), robot.rightFront.getPower());
                telemetry.addData("LB / RB", "%.2f / %.2f", robot.leftBack.getPower(), robot.rightBack.getPower());
                double intakePower = robot.intake.getPower();
                telemetry.addData("Intake power", "%.0f%% (%s)", Math.abs(intakePower) * 100,
                        intakePower > 0 ? "forward" : intakePower < 0 ? "reverse" : "stopped");
                telemetry.addData("Shooter power", "%.0f%%", robot.shooter.getPower() * 100);
                telemetry.addData("Shooter setting", "%d%% (0-90%%)", shooterPercent);
                telemetry.addData("Gate command", stickCommanded ? String.format(java.util.Locale.US, "%.3f", stickPosition) : "Not positioned (D-pad left/right to adjust)");
                telemetry.addLine("Driver 2: LT intake | LB reverse | RT shooter | Up/down power | Left/right gate");
                telemetry.update();
                idle();
            }
        } finally {
            robot.stopAll();
        }
    }
}
