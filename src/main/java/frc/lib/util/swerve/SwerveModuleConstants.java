package frc.lib.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Swerve Module Constants wrapper class
 */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     *
     * @param driveMotorID CAN ID of Swerve Module Drive Motor
     * @param angleMotorID CAN ID of Swerve Module Angle Motor
     * @param canCoderID CAN ID of Swerve Module CANCoder
     * @param angleOffset Angle offset of the CANCoder to face the wheel forward
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID,
        Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
