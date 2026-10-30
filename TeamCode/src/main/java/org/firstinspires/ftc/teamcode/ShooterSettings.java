package org.firstinspires.ftc.teamcode;

/** All RPM values refer to the flywheel, not the motor shaft. */
public final class ShooterSettings {
    private ShooterSettings() { }
    // goBILDA 5203-2402-0001: 6000 RPM, 1:1 internal gearbox, 28 counts/rev.
    public static final double MOTOR_TICKS_PER_REV = 28.0;
    public static final double MOTOR_REVS_PER_WHEEL_REV = 28.0 / 14.0;
    public static final double TICKS_PER_WHEEL_REV = MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_WHEEL_REV;
    public static final int INITIAL_RPM = 1430;
    public static final int STEP_RPM = 50;
    public static final int MAX_RPM = 2600;
    public static final double TOLERANCE_RPM = 50.0;
    // Initial, unmeasured tuning: nominal 3000 wheel RPM at 12 V.
    public static final double FREE_WHEEL_RPM = 6000.0 / MOTOR_REVS_PER_WHEEL_REV;
    public static final double FEEDFORWARD_SCALE = 0.95;
    // Independent of the light's tolerance: soften full-on/full-off transitions.
    public static final double RECOVERY_BAND_RPM = 50.0;

    /** Encoder closed-loop control, inspired by asymmetric bang-bang + feedforward.
     * Full forward effort below target-band, coast above target+band, smoothly
     * blended around the target. No reverse braking and no integral windup.
     * Sensor/loop delays and available torque still limit physical performance.
     */
    public static double calculatePower(double targetRpm, double measuredRpm, double voltage) {
        if (!Double.isFinite(targetRpm) || targetRpm <= 0 || !Double.isFinite(measuredRpm)) {
            return 0;
        }
        double target = Math.min(targetRpm, MAX_RPM);
        double battery = Double.isFinite(voltage) && voltage >= 7 && voltage <= 16 ? voltage : 12;
        double feedforward = Math.min(1, FEEDFORWARD_SCALE * target / FREE_WHEEL_RPM * 12 / battery);
        double error = target - measuredRpm;
        if (error >= RECOVERY_BAND_RPM) return 1;
        if (error <= -RECOVERY_BAND_RPM) return 0;
        // Continuous at zero error and at both band boundaries.
        return error >= 0
                ? feedforward + (1 - feedforward) * error / RECOVERY_BAND_RPM
                : feedforward * (1 + error / RECOVERY_BAND_RPM);
    }

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
