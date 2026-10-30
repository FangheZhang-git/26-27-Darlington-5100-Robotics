package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Timed shooting followed by a short robot-relative retreat. */
@Autonomous(name = "5100 Shoot and Back Up", group = "5100")
public final class RobotTimedAuto extends LinearOpMode {
    private static final double SHOOT_SECONDS = 20.0;
    private static final double INTAKE_START_SECONDS = 5.0;
    private static final double BACK_UP_SECONDS = 1.5;
    private static final double INTAKE_POWER = 0.80;
    private static final double DRIVE_SPEED = 1.0;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        try {
            robot.stopAll();
            telemetry.addLine("INIT: motors stopped. Press START to begin.");
            telemetry.addData("Shooter target (wheel RPM)", ShooterSettings.INITIAL_RPM);
            telemetry.addLine("0-5 s: shooter | 5-20 s: shooter + intake 80%");
            telemetry.addLine("Then back up at 100% for 1.5 s, then remain stopped.");
            telemetry.addLine("Position the height gate before starting; auto does not move it.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            ElapsedTime stageTimer = new ElapsedTime();
            ElapsedTime telemetryTimer = new ElapsedTime();
            telemetry.setMsTransmissionInterval(100);
            while (opModeIsActive()) {
                double elapsed = stageTimer.seconds();
                if (elapsed >= SHOOT_SECONDS) break;
                // Keep updating feedback throughout spin-up AND feeding; do not sleep.
                robot.setShooterRpm(ShooterSettings.INITIAL_RPM);
                double intakePower = elapsed >= INTAKE_START_SECONDS ? INTAKE_POWER : 0;
                robot.setIntakePower(intakePower);
                boolean ready = robot.updateShooterLight(robot.getShooterRpm());
                if (telemetryTimer.seconds() >= 0.1) {
                    telemetryTimer.reset();
                    telemetry.addData("Stage", intakePower == 0 ? "Shooter spin-up" : "Shooting + intake");
                    telemetry.addData("Shooting elapsed (s)", "%.1f / 20.0", elapsed);
                    telemetry.addData("Shooter target (wheel RPM)", ShooterSettings.INITIAL_RPM);
                    telemetry.addData("Shooter measured (wheel RPM)", "%.1f", robot.getShooterRpm());
                    telemetry.addData("Shooter effort", "%.0f%%", robot.getShooterPower() * 100);
                    telemetry.addData("Intake power", "%.0f%%", intakePower * 100);
                    telemetry.addData("Shooter ready", ready ? "PURPLE - within tolerance" : "RED - not ready");
                    telemetry.update();
                }
                idle();
            }

            robot.stopAll();
            if (!opModeIsActive()) return;
            // Positive drive input matches pulling Driver 1's left stick BACKWARD
            // with the robot's physically corrected TeleOp polarity.
            robot.driveRobotCentric(1.0, 0, 0, DRIVE_SPEED);
            stageTimer.reset();
            while (opModeIsActive() && stageTimer.seconds() < BACK_UP_SECONDS) {
                if (telemetryTimer.seconds() >= 0.1) {
                    telemetryTimer.reset();
                    telemetry.addData("Stage", "Backing up at 100%");
                    telemetry.addData("Backing elapsed (s)", "%.1f / 1.5", stageTimer.seconds());
                    telemetry.addData("Shooter effort", "0%%");
                    telemetry.addData("Intake power", "0%%");
                    telemetry.update();
                }
                idle();
            }

            robot.stopAll();
            telemetry.addLine("Auto complete: all motors stopped. Waiting for TeleOp.");
            telemetry.update();
            while (opModeIsActive()) idle();
        } finally {
            // STOP or an exception in any stage shuts off all motor commands.
            robot.stopAll();
        }
    }
}
