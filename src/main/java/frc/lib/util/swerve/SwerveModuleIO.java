package frc.lib.util.swerve;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;

/** IO Class for SwerveModule */
public interface SwerveModuleIO {

    /** Inputs Class for SwerveModule */
    @AutoLog
    public static class SwerveModuleInputs {
        public double angleEncoderValue;
        public double driveMotorSelectedPosition;
        public double driveMotorSelectedSensorVelocity;
        public double angleMotorSelectedPosition;
        public double driveMotorTemperature;
        public double angleMotorTemperature;
    }

    public default void updateInputs(SwerveModuleInputs inputs) {}

    public default void setDriveMotor(ControlRequest request) {}

    public default void setAngleMotor(ControlRequest request) {}

    public default void setAngleSelectedSensorPosition(double angle) {}

    public default StatusSignal<Double> getAbsolutePositionAngleEncoder() {
        return null;
    }

    public default void setPositionAngleMotor(double absolutePosition) {}

    public default StatusSignal<Double> getVelocityDriveMotor() {
        return null;
    }

    public default StatusSignal<Double> getPositionAngleMotor() {
        return null;
    }

    public default StatusSignal<Double> getPositionDriveMotor() {
        return null;
    }

}
