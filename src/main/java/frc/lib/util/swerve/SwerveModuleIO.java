package frc.lib.util.swerve;

import org.littletonrobotics.junction.AutoLog;

/** IO Class for SwerveModule */
public interface SwerveModuleIO {
    /** Inputs Class for SwerveModule */
    @AutoLog
    public static class SwerveModuleInputs {
        public double driveMotorSelectedPosition;
        public double driveMotorSelectedSensorVelocity;
        public double angleMotorSelectedPosition;
        public double absolutePositionAngleEncoder;
        public double[] odometryTimestamps;
        // public double driveMotorTemp;
        // public double angleMotorTemp;
    }

    public void setModNumber(int number);

    public void updateInputs(SwerveModuleInputs inputs);

    public void setDriveMotor(double mps);

    public void setDriveMotorPower(double power);

    public void setAngleMotor(double angle);

    public void setAngleSelectedSensorPosition(double angle);

    public void setPositionAngleMotor(double absolutePosition);

}
