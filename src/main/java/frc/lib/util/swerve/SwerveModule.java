package frc.lib.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.ctre.CTREModuleState;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Base Swerve Module Class. Creates an instance of the swerve module
 */
public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;
    private double lastAngle;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS,
        Constants.Swerve.driveKV, Constants.Swerve.driveKA);
    private DutyCycleOut drivePower = new DutyCycleOut(0);
    private VelocityVoltage driveSpeed = new VelocityVoltage(0);
    private PositionDutyCycle angleMotorPosition = new PositionDutyCycle(0);

    /**
     * Creates an instance of a Swerve Module
     *
     * @param moduleNumber Swerve Module ID. Must be unique
     * @param constants Constants specific to the swerve module
     */
    public SwerveModule(int moduleNumber, frc.lib.util.swerve.SwerveModuleConstants constants) {
        this.moduleNumber = moduleNumber;
        angleOffset = constants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(constants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(constants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(constants.driveMotorID);
        configDriveMotor();



        lastAngle = getState().angle.getDegrees();
    }

    /**
     * Sets the desired state for the swerve module for speed and angle
     *
     * @param desiredState The desired state (speed and angle)
     * @param isOpenLoop Whether to use open or closed loop formula
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        // Custom optimize
        // command, since
        // default WPILib
        // optimize assumes
        // continuous
        // controller which
        // CTRE is
        // not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.setControl(drivePower.withOutput(percentOutput));
        } else {
            double velocity = Conversions.mpsToRotations(desiredState.speedMetersPerSecond,
                Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            driveMotor.setControl(driveSpeed.withVelocity(velocity)
                .withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));

        }

        double angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is
        // less then 1%. Prevents
        // Jittering.
        angleMotor.setControl(angleMotorPosition
            .withPosition(Conversions.degreesToRotation(angle, Constants.Swerve.angleGearRatio)));
        lastAngle = angle;
    }

    /**
     *
     */
    private void resetToAbsolute() {
        double absolutePosition = getCanCoderRaw() / Constants.Swerve.angleGearRatio;
        FeedbackConfigs feedbackConfigs = Robot.ctreConfigs.swerveAngleFXConfig.Feedback
            .withFeedbackRotorOffset(absolutePosition);
        angleMotor.getConfigurator().apply(feedbackConfigs);
    }

    /**
     * Gets the Swerve module position
     *
     * @return Swerve module position
     */
    public SwerveModulePosition getPosition() {



        double position =
            Conversions.rotationsToMeters(driveMotor.getRotorPosition().getValueAsDouble(),
                Constants.Swerve.driveGearRatio, Constants.Swerve.wheelCircumference);

        Rotation2d angle = Rotation2d.fromDegrees(Conversions.rotationToDegrees(
            angleMotor.getRotorPosition().getValueAsDouble(), Constants.Swerve.angleGearRatio));
        return new SwerveModulePosition(position, angle);
    }

    /**
     * Configure the Angle motor CANCoder
     */
    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        MagnetSensorConfigs canCoderConfiguration =
            Robot.ctreConfigs.swerveCanCoderConfig.MagnetSensor;
        canCoderConfiguration.MagnetOffset = angleOffset;
        angleEncoder.getConfigurator().apply(canCoderConfiguration);
    }

    /**
     * Configure the Angle motor
     */
    private void configAngleMotor() {
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    /**
     * Configure the Drive motor
     */
    private void configDriveMotor() {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
    }

    /**
     * Get the rotations of CANCoder from [0, 1)
     *
     * @return rotations of CANCoder from [0, 1)
     */
    public double getCanCoderRaw() {
        return angleEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Get the 2d rotation of the module
     *
     * @return 2d rotation of the module
     */
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(
            Conversions.rotationToDegrees(getCanCoderRaw(), Constants.Swerve.angleGearRatio));
    }

    /**
     * Gets the Swerve module state
     *
     * @return Swerve module state
     */
    public SwerveModuleState getState() {
        double velocity = Conversions.rpsToMPS(driveMotor.getRotorVelocity().getValueAsDouble(),
            Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.rotationToDegrees(
            angleMotor.getRotorPosition().getValueAsDouble(), Constants.Swerve.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

}
