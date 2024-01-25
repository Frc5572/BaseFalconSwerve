package frc.lib.util.swerve;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.ControlRequest;


public interface SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleInputs {
        public double angleEncoderValue;
        public double driveMotorSelectedPosition;
        public double driveMotorSelectedSensorVelocity;
        public double angleMotorSelectedPosition;
        public double driveMotorTemperature;
        public double angleMotorTemperature;

    }

    public default void updateInputs(SwerveModuleInputs inputs) {
        int i = 0;
    }

    public default void setDriveMotor(ControlRequest request) {};

    public default void setAngleMotor(ControlRequest request) {};

    public default void setAngleSelectedSensorPosition(double angle) {};

    public default void setAngleMotorPosition(ControlRequest request) {};


    public default double getEncoderPosition() {
        return 0.0;
    }

    public default double getVelocityOfMotor() {
        return 0.0;
    }

    public default double getAngleMotor() {
        return 0.0;
    }

    public default double getPositionDriveMotor() {
        return 0.0;
    }


}
