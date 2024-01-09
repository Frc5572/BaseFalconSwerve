package frc.lib.util.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private CANSparkMax angleMotor;
    private TalonFX driveMotor;
    private CANCoder angleEncoder;
    private RelativeEncoder angleEncoderBuiltIn;
    private double lastAngle;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS,
        Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    SparkMaxPIDController angleMotorPIDController;

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
        angleEncoder = new CANCoder(constants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
        angleMotorPIDController = angleMotor.getPIDController();
        angleEncoderBuiltIn = angleMotor.getEncoder();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(constants.driveMotorID);
        configDriveMotor();
        System.out.println("Built In Angle Encoder Setting Position Conversion Factor Status: "
            + angleEncoderBuiltIn
                .setPositionConversionFactor(Constants.Swerve.turningDegreesPerEncoderRevolution));
        System.out.println("Built In Angle Encoder Setting Velocity Conversion Factor Status: "
            + angleEncoderBuiltIn.setVelocityConversionFactor(
                Constants.Swerve.turningDegreesPerEncoderRevolution / 60));

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
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.mpsToFalcon(desiredState.speedMetersPerSecond,
                Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is
        // less then 1%. Prevents
        // Jittering.
        // angleMotor.set(ControlMode.Position,
        // Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio));

        // System.out.println("Angle Motor PID Controller Setting Reference Status: "
        // + angleMotorPIDController.setReference(angle, ControlType.kPosition));
        SmartDashboard.putNumber("Module: " + moduleNumber + "PID Set Angle", angle);
        angleMotorPIDController.setReference(angle, ControlType.kPosition);
        lastAngle = angle;
    }

    // /**
    // *
    // *
    // * @param rotationSpeed Drive motor speed (-1 <= value <= 1)
    // */
    // public void setTurnAngle(double rotationSpeed) {
    // double absolutePosition = Conversions.degreesToFalcon(
    // getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
    // angleMotor.setSelectedSensorPosition(absolutePosition);
    // driveMotor.set(ControlMode.PercentOutput, rotationSpeed);
    // }

    /**
     * Gets the Swerve module position
     *
     * @return Swerve module position
     */
    public SwerveModulePosition getPosition() {
        double position = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
            Constants.Swerve.driveGearRatio, Constants.Swerve.wheelCircumference);

        Rotation2d angle = Rotation2d.fromDegrees(angleEncoderBuiltIn.getPosition());
        return new SwerveModulePosition(position, angle);
    }

    /**
     * Configure the Angle motor CANCoder
     */
    private void configAngleEncoder() {
        System.out.println("Config angle Motor status: " + angleEncoder.configFactoryDefault());
        System.out.println("Angle enccoder config all settings status"
            + angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig));
    }

    /**
     * Configure the Angle motor
     */
    private void configAngleMotor() {
        System.out.println(
            "Angle Motor Restore Factory Defaults Status: " + angleMotor.restoreFactoryDefaults());
        // angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        System.out.println("Angle Motor Set Idle Mode Status: "
            + angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode));
        System.out.println("Angle Motor Set Position PID Wrapping Status: "
            + angleMotorPIDController.setPositionPIDWrappingEnabled(true));
        System.out.println("Angle Motor Set Position PID Wrapping Minimum Input Status: "
            + angleMotorPIDController.setPositionPIDWrappingMinInput(
                Constants.Swerve.angleMotorEncoderPositionPIDMinInput));
        System.out.println("Angle Motor Set Position PID Wrapping Maximum Input Status: "
            + angleMotorPIDController.setPositionPIDWrappingMaxInput(
                Constants.Swerve.angleMotorEncoderPositionPIDMaxInput));
        System.out.println(
            "Angle Motor Set P Status: " + angleMotorPIDController.setP(Constants.Swerve.angleKP));
        System.out.println(
            "Angle Motor Set I Status: " + angleMotorPIDController.setI(Constants.Swerve.angleKI));
        System.out.println(
            "Angle Motor Set D Status: " + angleMotorPIDController.setD(Constants.Swerve.angleKD));
        resetEncoder();
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

    /**
     * Get the 2d rotation of the module
     *
     * @return 2d rotation of the module
     */
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition() - angleOffset);
    }

    /**
     * Gets the Swerve module state
     *
     * @return Swerve module state
     */
    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
            Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(angleEncoderBuiltIn.getPosition());
        return new SwerveModuleState(velocity, angle);
    }

    public REVLibError resetEncoder() {
        return angleEncoderBuiltIn.setPosition(getCanCoder().getDegrees());
    }

}
