package frc.lib.util;

/**
 * Constants file used when creating swerve modules
 */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param driveMotorID ID of the drive motor
     * @param angleMotorID ID of the angle motor
     * @param canCoderID ID of the canCoder
     * @param angleOffset offset of the canCoder angle
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID,
        double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
