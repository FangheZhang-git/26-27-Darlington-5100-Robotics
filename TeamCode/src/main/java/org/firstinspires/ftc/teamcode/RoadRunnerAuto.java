package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Starter path: finish tuning, then remove @Disabled to run. */
@Disabled
@Autonomous(name = "Road Runner Starter Auto", group = "5100")
public final class RoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        telemetry.addLine("Tuned robot required. Path: forward 24 inches, then back.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        try {
            // Inches and radians. +X forward, +Y left, positive heading counterclockwise.
            Actions.runBlocking(drive.actionBuilder(startPose)
                    .strafeTo(new Vector2d(24, 0))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(0, 0))
                    .build());
        } finally {
            drive.leftFront.setPower(0);
            drive.leftBack.setPower(0);
            drive.rightFront.setPower(0);
            drive.rightBack.setPower(0);
        }
    }
}
