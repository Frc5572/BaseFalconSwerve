package frc.lib.math;

/**
 * Mathematical conversions for swerve calculations
 */
public class Conversions {

    /**
     * Convert from motor encoder rotations to Meters
     *
     * @param rotations Rotations of motor encoder
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @param circumference The circumference of the Wheels
     * @return Degrees of Rotation of Mechanism falconToDegrees
     */
    public static double rotationsToMeters(double rotations, double gearRatio,
        double circumference) {
        return rotations * circumference / gearRatio;
    }

    /**
     * Convert from Motor Encoder rotations to degrees
     *
     * @param rotations Rotations of motor encoder
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism falconToDegrees
     */
    public static double rotationToDegrees(double rotations, double gearRatio) {
        return rotations * (360.0 / gearRatio);
    }

    /**
     * Convert from Degrees to rotations of the motor
     *
     * @param degrees
     * @param gearRatio
     * @return Number of rotations of the motor encoder
     */
    public static double degreesToRotation(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    /**
     * Meters per Second to Motor Encoder Rotations
     *
     * @param velocity Current veloctiy in Meters per Second
     * @param circumference Circumference of the wheels
     * @param gearRatio Gear ration between Falcon and Mechanism
     * @return Number of rotations
     */
    public static double mpsToRotations(double velocity, double circumference, double gearRatio) {
        return (velocity / circumference) * gearRatio;
    }

    /**
     * Motor Encoder Rotations to Meters per Second
     *
     * @param rps Motor Encoder Rotations per Second
     * @param circumference Circumference of the wheels
     * @param gearRatio Gear ration between Falcon and Mechanism
     * @return Velocity in Meters per Second
     */
    public static double rpsToMPS(double rps, double circumference, double gearRatio) {
        double wheelRPS = rps / gearRatio;
        double wheelMPS = (wheelRPS * circumference);
        return wheelMPS;
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

}
