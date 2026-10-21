package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Field-centric drive, proportional intake/shooter, and adjustable intake-height gate. */
@TeleOp(name = "5100 TeleOp", group = "5100")
public final class RobotTeleOp extends LinearOpMode {
    private static final double DRIVE_SPEED = 0.80;
    private static final double TURN_SENSITIVITY = 0.60;
    private static final double INTAKE_REVERSE_POWER = 0.60;
    // Normalized servo travel per second. Up increases position; down decreases it.
    private static final double STICK_RATE = 0.25;
    private static final double STICK_INITIAL_POSITION = 0.50;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        try {
            telemetry.addLine("5100 ready. Point robot away from driver before START.");
            telemetry.addLine("Left stick: move | Right stick X: turn | Options: reset heading");
            telemetry.addLine("LT: intake | LB: reverse intake | RT: shooter");
            telemetry.addLine("D-pad up/down: height gate | B: stop all motors while held");
            telemetry.addLine("Gate stays uncommanded until D-pad input; first command starts near 0.50.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            robot.resetHeading();
            boolean optionsWasPressed = gamepad1.options;
            // A servo has no actual-position feedback. Do not treat getPosition() as a measurement.
            double stickPosition = STICK_INITIAL_POSITION;
            boolean stickCommanded = false;
            ElapsedTime loopTimer = new ElapsedTime();
            while (opModeIsActive()) {
                // Cap elapsed time so a stalled loop cannot cause a large gate adjustment.
                double dt = Range.clip(loopTimer.seconds(), 0, 0.05);
                loopTimer.reset();
                boolean optionsPressed = gamepad1.options;
                if (optionsPressed && !optionsWasPressed) robot.resetHeading();
                optionsWasPressed = optionsPressed;

                if (gamepad1.b) {
                    robot.stopAll();
                } else {
                    // No application dead zone: pass proportional SDK stick values directly.
                    robot.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x * TURN_SENSITIVITY, DRIVE_SPEED);
                    robot.setIntakePower(gamepad1.left_bumper
                            ? -INTAKE_REVERSE_POWER : gamepad1.left_trigger);
                    robot.setShooterPower(gamepad1.right_trigger);
                    int gateDirection = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
                    if (gateDirection != 0) {
                        stickPosition = Range.clip(stickPosition + gateDirection * STICK_RATE * dt, 0, 1);
                        robot.setStickPosition(stickPosition);
                        stickCommanded = true;
                    }
                }

                telemetry.addData("Mode", gamepad1.b ? "Motors stopped (B held)" : "Field-centric / 80% drive");
                telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(robot.headingRadians()));
                telemetry.addData("LF / RF", "%.2f / %.2f", robot.leftFront.getPower(), robot.rightFront.getPower());
                telemetry.addData("LB / RB", "%.2f / %.2f", robot.leftBack.getPower(), robot.rightBack.getPower());
                telemetry.addData("Intake / Shooter", "%.2f / %.2f", robot.intake.getPower(), robot.shooter.getPower());
                telemetry.addData("Gate command", stickCommanded ? String.format(java.util.Locale.US, "%.3f", stickPosition) : "Not positioned (D-pad to adjust)");
                telemetry.addLine("LT intake | LB reverse | RT shooter | D-pad gate | Options heading | B stop");
                telemetry.update();
                idle();
            }
        } finally {
            robot.stopAll();
        }
    }
}
