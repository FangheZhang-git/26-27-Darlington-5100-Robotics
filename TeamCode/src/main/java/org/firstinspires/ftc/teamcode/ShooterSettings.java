package org.firstinspires.ftc.teamcode;

/** All RPM values refer to the flywheel, not the motor shaft. */
public final class ShooterSettings {
    private ShooterSettings() { }
    // goBILDA 5203-2402-0001: 6000 RPM, 1:1 internal gearbox, 28 counts/rev.
    public static final double MOTOR_TICKS_PER_REV = 28.0;
    public static final double MOTOR_REVS_PER_WHEEL_REV = 28.0 / 14.0;
    public static final double TICKS_PER_WHEEL_REV = MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_WHEEL_REV;
    public static final int INITIAL_RPM = 300;
    public static final int STEP_RPM = 50;
    public static final int MAX_RPM = 2600;
    public static final double TOLERANCE_RPM = 100.0;
    // Starting PIDF gains, not physically tuned. F uses nominal 6000 RPM at 12 V.
    public static final double VELOCITY_P = 10.0;
    public static final double VELOCITY_I = 3.0;
    public static final double VELOCITY_D = 0.0;
    public static final double VELOCITY_F = 32767.0 / (6000.0 * MOTOR_TICKS_PER_REV / 60.0);

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_WHEEL_REV / 60.0;
    }

    public static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_WHEEL_REV;
    }

    public static boolean isReady(double commandedRpm, double measuredRpm) {
        // Signed speed rejects reverse rotation; a stopped encoder cannot report ready.
        return commandedRpm > 0 && measuredRpm > 0
                && Double.isFinite(measuredRpm)
                && Math.abs(measuredRpm - commandedRpm) <= TOLERANCE_RPM;
    }
}
