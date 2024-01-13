package frc.lib.util.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Implementation for the real robot
 */
public class SwerveModuleReal implements SwerveModuleIO {

    private TalonFX angleMotor; // Output
    private TalonFX driveMotor; // Output
    private CANCoder angleEncoder; // Input

    private SwerveModuleConstants constants;

    /**
     * Initializer for real swerve module. Configures with defaults.
     */
    public SwerveModuleReal(SwerveModuleConstants constants) {
        this.constants = constants;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(constants.cancoderID, "canivore");
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(constants.angleMotorID, "canivore");
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(constants.driveMotorID, "canivore");
        configDriveMotor();
    }

    /**
     *
     */
    private void resetToAbsolute() {
        double absolutePosition = Conversions
            .degreesToFalcon(Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()).getDegrees()
                - constants.angleOffset, Constants.Swerve.angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    /**
     * Configure the Angle motor CANCoder
     */
    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    /**
     * Configure the Angle motor
     */
    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    /**
     * Configure the Drive motor
     */
    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.angleEncoderValue = angleEncoder.getAbsolutePosition();
        inputs.angleMotorSelectedPosition = angleMotor.getSelectedSensorPosition();
        inputs.driveMotorSelectedPosition = driveMotor.getSelectedSensorPosition();
        inputs.driveMotorSelectedSensorVelocity = driveMotor.getSelectedSensorVelocity();
        inputs.angleMotorTemperature = angleMotor.getTemperature();
        inputs.driveMotorTemperature = driveMotor.getTemperature();
    }

    @Override
    public void setAngleMotor(ControlMode mode, double outputValue) {
        angleMotor.set(mode, outputValue);
    }

    @Override
    public void setAngleSelectedSensorPosition(double angle) {
        angleMotor.setSelectedSensorPosition(angle);
    }

    @Override
    public void setDriveMotor(ControlMode mode, double outputValue) {
        driveMotor.set(mode, outputValue);
    }

    @Override
    public void setDriveMotorWithFF(ControlMode mode, double demand0, DemandType demand1Type,
        double demand1) {
        driveMotor.set(mode, demand0, demand1Type, demand1);
    }

}
