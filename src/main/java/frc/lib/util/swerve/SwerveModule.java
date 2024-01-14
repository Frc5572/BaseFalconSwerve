package frc.lib.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.ctre.CTREModuleState;
import frc.robot.Constants;

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
    private TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    private TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    private CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();
    private DutyCycleOut drivePower = new DutyCycleOut(0);
    private VelocityDutyCycle driveSpeed = new VelocityDutyCycle(0).withSlot(0);
    private PositionDutyCycle angleMotorPosition = new PositionDutyCycle(0).withSlot(0);
    // Use Voltage to turn Voltage Compensation on
    // private VelocityVoltage driveSpeed = new VelocityVoltage(0).withSlot(0);
    // private PositionVoltage angleMotorPosition = new PositionVoltage(0).withSlot(0);

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
        angleEncoder = new CANcoder(constants.cancoderID, Constants.Swerve.canBus);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(constants.angleMotorID, Constants.Swerve.canBus);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(constants.driveMotorID, Constants.Swerve.canBus);
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
        // Custom optimize command, since default WPILib optimize assumes
        // continuous controller which CTRE is not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.setControl(drivePower.withOutput(percentOutput));
        } else {
            double velocity = Conversions.mpsToFalconRPS(desiredState.speedMetersPerSecond,
                Constants.Swerve.wheelCircumference, 1);
            driveMotor.setControl(driveSpeed.withVelocity(velocity));
        }

        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees();

        // See Line 163, swerveAngleFXConfig.Feedback.SensorToMechanismRatio
        angleMotor.setControl(
            angleMotorPosition.withPosition(Conversions.degreesToFalconRotations(angle, 1)));
        lastAngle = angle;
    }

    /**
     *
     */
    private void resetToAbsolute() {
        double absolutePosition = getCanCoderRaw() / Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.Feedback.FeedbackRotorOffset = absolutePosition;
        angleMotor.getConfigurator().apply(swerveAngleFXConfig);
    }

    /**
     * Gets the Swerve module position
     *
     * @return Swerve module position
     */
    public SwerveModulePosition getPosition() {
        double position = Conversions.falconRotationsToMechanismMeters(
            driveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference, 1);

        Rotation2d angle = Rotation2d.fromDegrees(Conversions
            .falconRotationsToMechanismDegrees(angleMotor.getPosition().getValueAsDouble(), 1));
        return new SwerveModulePosition(position, angle);
    }

    /**
     * Configure the Angle motor CANCoder
     */
    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        swerveCanCoderConfig.MagnetSensor.SensorDirection =
            Constants.Swerve.canCoderInvert ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange =
            AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.MagnetOffset = angleOffset;
        angleEncoder.getConfigurator().apply(swerveCanCoderConfig);
    }

    /**
     * Configure the Angle motor
     */
    private void configAngleMotor() {
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold =
            Constants.Swerve.anglePeakCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold =
            Constants.Swerve.anglePeakCurrentDuration;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit =
            Constants.Swerve.angleContinuousCurrentLimit;
        swerveAngleFXConfig.Feedback.FeedbackRotorOffset = 0;
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        // swerveAngleFXConfig.Slot0.kS = 0;
        // swerveAngleFXConfig.Slot0.kV = 0;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        // swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        angleMotor.getConfigurator().apply(swerveAngleFXConfig);
        angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    /**
     * Configure the Drive motor
     */
    private void configDriveMotor() {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold =
            Constants.Swerve.drivePeakCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold =
            Constants.Swerve.drivePeakCurrentDuration;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit =
            Constants.Swerve.driveContinuousCurrentLimit;
        swerveDriveFXConfig.Feedback.FeedbackRotorOffset = 0;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
            Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kS = Constants.Swerve.driveKS;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.driveKV;
        swerveDriveFXConfig.Slot0.kA = Constants.Swerve.driveKA;
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;
        driveMotor.getConfigurator().apply(swerveDriveFXConfig);
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
        return Rotation2d.fromDegrees(Conversions
            .falconRotationsToMechanismDegrees(getCanCoderRaw(), Constants.Swerve.angleGearRatio));
    }

    /**
     * Gets the Swerve module state
     *
     * @return Swerve module state
     */
    public SwerveModuleState getState() {
        double velocity = Conversions.falconRPSToMechanismMPS(
            driveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference, 1);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions
            .falconRotationsToMechanismDegrees(angleMotor.getPosition().getValueAsDouble(), 1));
        return new SwerveModuleState(velocity, angle);
    }

}
