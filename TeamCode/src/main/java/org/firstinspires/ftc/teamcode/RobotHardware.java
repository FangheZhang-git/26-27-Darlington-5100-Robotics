package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Team 5100 hardware. Hub/port assignments live in the RC configuration. */
public final class RobotHardware {
    public static final String LEFT_FRONT_NAME = "LF";  // Expansion motor port 2
    public static final String LEFT_BACK_NAME = "LB";   // Expansion motor port 3
    public static final String RIGHT_FRONT_NAME = "RF"; // Expansion motor port 0
    public static final String RIGHT_BACK_NAME = "RB";  // Expansion motor port 1
    public static final String IMU_NAME = "imu";        // Control Hub internal IMU

    public static final String INTAKE_NAME = "Intake";   // Control motor port 0
    public static final String SHOOTER_NAME = "Shooter"; // Control motor port 1
    public static final String STICK_NAME = "Stick";     // Control servo port 3
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // Mirrored direct-drive assumption; physical directions are not yet measured.
    public static final DcMotorSimple.Direction LF_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LB_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RF_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RB_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public final DcMotor leftFront;
    public final DcMotor leftBack;
    public final DcMotor rightFront;
    public final DcMotor rightBack;
    public final IMU imu;
    public final DcMotor intake;
    public final DcMotorEx shooter;
    public final ServoImplEx shootLight;
    private double commandedShooterRpm;
    private final Iterable<VoltageSensor> voltageSensors;
    private long lastVoltageUpdateNanos;
    private double appliedShooterF = ShooterSettings.VELOCITY_F;
    private double shooterBatteryVoltage = Double.NaN;
    // Manufacturer chart: red at 1100 us, violet at 1900 us. Stay inside edges.
    private static final double LIGHT_RED = (1110.0 - 600.0) / 1800.0;
    private static final double LIGHT_PURPLE = (1880.0 - 600.0) / 1800.0;
    public final Servo stick;

    public RobotHardware(HardwareMap hardwareMap) {
        voltageSensors = hardwareMap.voltageSensor;
        leftFront = hardwareMap.get(DcMotor.class, LEFT_FRONT_NAME);
        leftBack = hardwareMap.get(DcMotor.class, LEFT_BACK_NAME);
        rightFront = hardwareMap.get(DcMotor.class, RIGHT_FRONT_NAME);
        rightBack = hardwareMap.get(DcMotor.class, RIGHT_BACK_NAME);
        configure(leftFront, LF_DIRECTION);
        configure(leftBack, LB_DIRECTION);
        configure(rightFront, RF_DIRECTION);
        configure(rightBack, RB_DIRECTION);
        intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shootLight = hardwareMap.get(ServoImplEx.class, "shootLight");
        shootLight.setDirection(Servo.Direction.FORWARD);
        shootLight.setPwmRange(new PwmControl.PwmRange(600, 2400));
        shootLight.setPosition(LIGHT_RED);
        stick = hardwareMap.get(Servo.class, STICK_NAME);
        configure(intake, INTAKE_DIRECTION);
        configure(shooter, SHOOTER_DIRECTION);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(ShooterSettings.VELOCITY_P,
                ShooterSettings.VELOCITY_I, ShooterSettings.VELOCITY_D, ShooterSettings.VELOCITY_F);
        stick.setDirection(Servo.Direction.FORWARD);
        // Do not move the height gate during INIT; the driver positions it manually.
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        if (!imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)))) {
            throw new IllegalStateException("Control Hub IMU initialization failed");
        }
    }

    private static void configure(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Encoder cables are not required for this power-based drive.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double headingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    /** Forward/right are field-relative; positive turn is clockwise. */
    public void driveFieldCentric(double forward, double right, double turn, double speed) {
        double heading = headingRadians();
        double robotRight = right * Math.cos(heading) + forward * Math.sin(heading);
        double robotForward = forward * Math.cos(heading) - right * Math.sin(heading);
        driveRobotCentric(robotForward, robotRight, turn, speed);
    }

    public void driveRobotCentric(double forward, double strafe, double turn, double speed) {
        double lf = forward + strafe + turn;
        double lb = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rb = forward + strafe - turn;
        double denominator = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))));
        double scale = Range.clip(speed, 0, 1) / denominator;
        leftFront.setPower(lf * scale);
        leftBack.setPower(lb * scale);
        rightFront.setPower(rf * scale);
        rightBack.setPower(rb * scale);
    }

    public void setIntakePower(double power) {
        intake.setPower(Range.clip(power, -1, 1));
    }

    public void setShooterRpm(double wheelRpm) {
        if (!Double.isFinite(wheelRpm)) wheelRpm = 0;
        commandedShooterRpm = Range.clip(wheelRpm, 0, ShooterSettings.MAX_RPM);
        if (commandedShooterRpm == 0) {
            shooter.setPower(0); // Disable velocity regulation and let the wheel coast.
        } else {
            maintainShooterVelocity();
        }
    }

    /** Keep the hub's encoder PIDF loop running at the requested wheel speed.
     * Refresh voltage compensation at most four times/second; leave PID gains
     * unchanged and avoid coefficient writes for insignificant voltage changes.
     * This cannot prevent transient drops during spin-up or ball contact.
     */
    private void maintainShooterVelocity() {
        long now = System.nanoTime();
        if (lastVoltageUpdateNanos == 0 || now - lastVoltageUpdateNanos >= 250_000_000L) {
            lastVoltageUpdateNanos = now;
            double voltage = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : voltageSensors) {
                double reading = sensor.getVoltage();
                if (Double.isFinite(reading) && reading >= 7.0 && reading <= 16.0) {
                    voltage = Math.min(voltage, reading);
                }
            }
            shooterBatteryVoltage = Double.isFinite(voltage) ? voltage : Double.NaN;
            double compensatedF = ShooterSettings.voltageCompensatedF(shooterBatteryVoltage);
            if (Math.abs(compensatedF - appliedShooterF) >= 0.1) {
                shooter.setVelocityPIDFCoefficients(ShooterSettings.VELOCITY_P,
                        ShooterSettings.VELOCITY_I, ShooterSettings.VELOCITY_D, compensatedF);
                appliedShooterF = compensatedF;
            }
        }
        shooter.setVelocity(ShooterSettings.rpmToTicksPerSecond(commandedShooterRpm));
    }

    public double getShooterBatteryVoltage() {
        return shooterBatteryVoltage;
    }

    public double getShooterRpm() {
        return ShooterSettings.ticksPerSecondToRpm(shooter.getVelocity());
    }

    public boolean updateShooterLight(double measuredRpm) {
        boolean ready = ShooterSettings.isReady(commandedShooterRpm, measuredRpm);
        shootLight.setPosition(ready ? LIGHT_PURPLE : LIGHT_RED);
        return ready;
    }

    public void setStickPosition(double position) {
        stick.setPosition(Range.clip(position, 0, 1));
    }

    public void stopAll() {
        stopDrive();
        intake.setPower(0);
        setShooterRpm(0);
        shootLight.setPosition(LIGHT_RED);
        // Hold the gate's last position; do not sweep it on STOP.
    }

    public void stopDrive() {
        driveRobotCentric(0, 0, 0, 0);
    }
}
