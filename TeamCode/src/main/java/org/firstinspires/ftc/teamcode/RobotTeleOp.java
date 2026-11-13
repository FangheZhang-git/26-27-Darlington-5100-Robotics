package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Robot-centric drive, fixed intake and encoder-regulated shooter RPM, and two-position gate. */
@TeleOp(name = "5100 TeleOp", group = "5100")
public final class RobotTeleOp extends LinearOpMode {
    private static final double DRIVE_SPEED = 1;
    private static final double TURN_SENSITIVITY = 0.80;
    private static final double INTAKE_FORWARD_POWER = 0.80;
    private static final double TRIGGER_THRESHOLD = 0.10;
    private static final double INTAKE_REVERSE_POWER = 0.60;
    private static final double GATE_OPEN_POSITION = 0.554;
    private static final double GATE_CLOSED_POSITION = 0.0;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        try {
            telemetry.addLine("5100 ready. Robot-centric: forward follows the robot front.");
            telemetry.addLine("Driver 1: left stick move | right stick turn");
            telemetry.addLine("Driver 2: LT intake 80% | LB reverse 60% | RT shooter (starts at 1730 wheel RPM)");
            telemetry.addLine("Driver 2: up/down shooter +/-50 RPM (0-2600) | Circle/B open gate | Triangle/Y close gate");
            telemetry.addLine("Driver 1: hold Circle/B to stop all motors.");
            telemetry.addLine("Gate stays uncommanded until Driver 2 presses Circle/B or Triangle/Y.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            boolean upWasPressed = gamepad2.dpad_up;
            boolean downWasPressed = gamepad2.dpad_down;
            int shooterRpm = ShooterSettings.INITIAL_RPM;
            // A servo has no actual-position feedback. Track only the last command.
            double gatePosition = Double.NaN;
            ElapsedTime loopTimer = new ElapsedTime();
            ElapsedTime telemetryTimer = new ElapsedTime();
            telemetry.setMsTransmissionInterval(100);
            while (opModeIsActive()) {
                double loopSeconds = loopTimer.seconds();
                loopTimer.reset();
                boolean stopHeld = gamepad1.b;
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
                    robot.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x * TURN_SENSITIVITY, DRIVE_SPEED);
                    robot.setIntakePower(gamepad2.left_bumper
                            ? -INTAKE_REVERSE_POWER
                            : (gamepad2.left_trigger > TRIGGER_THRESHOLD ? INTAKE_FORWARD_POWER : 0));
                    robot.setShooterRpm(gamepad2.right_trigger > TRIGGER_THRESHOLD ? shooterRpm : 0);
                    // Close wins if both gate buttons are held; otherwise keep the last position.
                    if (gamepad2.y) {
                        gatePosition = GATE_CLOSED_POSITION;
                    } else if (gamepad2.b) {
                        gatePosition = GATE_OPEN_POSITION;
                    }
                    if (!Double.isNaN(gatePosition)) {
                        robot.setStickPosition(gatePosition);
                    }
                }

                double measuredRpm = robot.getShooterRpm();
                boolean shooterReady = robot.updateShooterLight(measuredRpm);
                // Keep slower IMU reads, formatting and transmission out of most control cycles.
                if (telemetryTimer.seconds() >= 0.1) {
                    telemetryTimer.reset();
                    telemetry.addData("Mode", stopHeld ? "Motors stopped (B held)" : "Robot-centric / 100% drive");
                    telemetry.addLine("Driver 1: sticks drive | Circle/B stops motors");
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
                    telemetry.addData("Shooter ready", shooterReady ? "PURPLE - within +/-50 RPM" : "RED - not ready");
                    telemetry.addData("Gate command", Double.isNaN(gatePosition) ? "Not positioned" :
                            (gatePosition == GATE_OPEN_POSITION ? "OPEN (0.554)" : "CLOSED (0.000)"));
                    telemetry.addLine("Driver 2: LT intake | LB reverse | RT shooter | Up/down RPM | Circle/B open | Triangle/Y close");
                    telemetry.addData("Shooter effort", "%.0f%%", robot.getShooterPower() * 100);
                    telemetry.addData("Last return to tolerance (ms)", "%.0f", robot.getLastRecoveryMs());
                    telemetry.addData("Control loop (ms)", "%.1f", loopSeconds * 1000);
                    telemetry.update();
                }
                idle();
            }
        } finally {
            robot.stopAll();
        }
    }
}
