package frc.lib.math;

import edu.wpi.first.math.MathUtil;

/**
 * Mathematical conversions for swerve calculations
 */
public class Conversions {

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism falconToDegrees
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts degreesToFalcon
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param rpm RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double rpmToFalcon(double rpm, double gearRatio) {
        double motorRPM = rpm * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism falconToDegrees
     */
    public static double falconToMeters(double counts, double gearRatio, double circumference) {
        return counts * circumference / (gearRatio * 2048.0);
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference,
        double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double mpsToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = rpmToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * Normalize angle to between 0 to 360
     *
     * @param goal initial angle
     * @return normalized angle
     */
    public static double reduceTo0_360(double goal) {
        return goal % 360;
    }

    /**
     * Apply a quadratic curve to swerve inputs.
     *
     * @param input Controller input
     * @param deadband Controller deadband
     * @return New curved input.
     */
    public static double applySwerveCurve(double input, double deadband) {
        // For 0.2 Joystick Deadband:
        // f(x)=0.625*x^2+0.5*x+-0.125
        // For 0.1 Joystick Deadband:
        // f(x)=0.778*x^2+0.256*x+-0.033

        double processedInput = 0.0;

        if (Math.abs(input) >= deadband) {
            processedInput =
                (0.778 * Math.pow(Math.abs(input), 2) + 0.256 * Math.abs(input) - 0.033);
            processedInput = Math.copySign(processedInput, input);
        }
        return MathUtil.clamp(processedInput, -1, 1);

    }

}
