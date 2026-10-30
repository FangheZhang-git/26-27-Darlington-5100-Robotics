import org.firstinspires.ftc.teamcode.ShooterSettings;

/** Run with javac/java as documented in HARDWARE_SETUP.md; no robot is driven. */
public final class ShooterSettingsTest {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static double output(double target, double actual, double volts) {
        return ShooterSettings.calculatePower(target, actual, volts);
    }
    public static void main(String[] args) {
        check(ShooterSettings.INITIAL_RPM == 1430, "startup");
        check(ShooterSettings.STEP_RPM == 50 && ShooterSettings.MAX_RPM == 2600, "limits");
        check(Math.abs(ShooterSettings.rpmToTicksPerSecond(1430) - 1334.6666666667) < 1e-6, "conversion");
        check(ShooterSettings.isReady(1430, 1380) && ShooterSettings.isReady(1430, 1480), "inclusive boundaries");
        check(!ShooterSettings.isReady(1430, 1379.9) && !ShooterSettings.isReady(1430, 1480.1), "outside tolerance");
        check(!ShooterSettings.isReady(0, 1) && !ShooterSettings.isReady(1430, Double.NaN), "invalid readiness");
        check(output(1430, 0, 12) == 1, "full effort at startup");
        check(output(1430, 1380, 12) == 1, "full recovery effort");
        check(output(1430, 1480, 12) == 0, "coast at overspeed");
        check(output(1430, 1430, 10) > output(1430, 1430, 14), "voltage compensation");
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1, 0, 6, 17}) {
            check(output(1430, 1430, bad) == output(1430, 1430, 12), "invalid voltage fallback");
        }
        check(output(0, 1430, 12) == 0 && output(-10, 0, 12) == 0, "stop");
        check(output(Double.NaN, 0, 12) == 0 && output(1430, Double.NaN, 12) == 0, "invalid sensor/target");
        check(output(5000, 2600, 12) == output(2600, 2600, 12), "target clamp");
        for (double target : new double[]{10, 1430, 2600}) {
            for (double voltage : new double[]{7, 10, 12, 16}) {
                double previous = 1;
                for (double rpm = -100; rpm <= 3200; rpm += 5) {
                    double power = output(target, rpm, voltage);
                    check(power >= 0 && power <= 1, "bounded forward-only power");
                    check(power <= previous + 1e-12, "effort decreases as RPM rises");
                    previous = power;
                }
            }
        }
        for (double boundary : new double[]{1380, 1430, 1480}) {
            check(Math.abs(output(1430, boundary - 1e-6, 12)
                    - output(1430, boundary + 1e-6, 12)) < 1e-6, "continuous transition");
        }
        // Four consecutive disturbances must each restore full recovery effort,
        // with no accumulated integral or previous-shot state affecting output.
        for (int shot = 0; shot < 4; shot++) {
            check(output(1430, 1430, 12) > 0, "holding effort");
            check(output(1430, 1200, 12) == 1, "shot recovery");
            check(output(1430, 1500, 12) == 0, "overshoot coast");
        }
        System.out.println(checks + " controller checks passed; physical recovery is not tested.");
    }
}
