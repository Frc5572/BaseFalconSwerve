package frc.lib.util.ctre;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;

/**
 * CTRE config constants
 */
public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    private CurrentLimitsConfigs swerveAngleFXCurrentLimitsConfigs = new CurrentLimitsConfigs();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    private CurrentLimitsConfigs swerveDriveFXCurrentLimitsConfigs = new CurrentLimitsConfigs();
    public CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();
    private MagnetSensorConfigs swerveCanCoderMagnetSensorConfigs = new MagnetSensorConfigs();

    /**
     * CTRE config constants
     */
    public CTREConfigs() {
        /* Swerve Angle Motor Configurations */
        swerveAngleFXCurrentLimitsConfigs.StatorCurrentLimitEnable =
            Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXCurrentLimitsConfigs.SupplyCurrentLimitEnable =
            Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXCurrentLimitsConfigs.StatorCurrentLimit =
            Constants.Swerve.angleContinuousCurrentLimit;
        swerveAngleFXCurrentLimitsConfigs.SupplyCurrentThreshold =
            Constants.Swerve.anglePeakCurrentLimit;
        swerveAngleFXCurrentLimitsConfigs.SupplyTimeThreshold =
            Constants.Swerve.anglePeakCurrentDuration;
        swerveAngleFXCurrentLimitsConfigs.SupplyCurrentLimit =
            Constants.Swerve.anglePeakCurrentLimit;
        swerveAngleFXConfig.CurrentLimits = swerveAngleFXCurrentLimitsConfigs;
        swerveAngleFXConfig.Feedback = new FeedbackConfigs().withFeedbackRotorOffset(0);

        Slot0Configs swerveAngleFXConfigGains = new Slot0Configs();
        swerveAngleFXConfigGains.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfigGains.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfigGains.kD = Constants.Swerve.angleKD;

        /* Swerve Drive Motor Configuration */

        swerveDriveFXCurrentLimitsConfigs.StatorCurrentLimitEnable =
            Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXCurrentLimitsConfigs.SupplyCurrentLimitEnable =
            Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXCurrentLimitsConfigs.StatorCurrentLimit =
            Constants.Swerve.driveContinuousCurrentLimit;
        swerveDriveFXCurrentLimitsConfigs.SupplyCurrentThreshold =
            Constants.Swerve.drivePeakCurrentLimit;
        swerveDriveFXCurrentLimitsConfigs.SupplyTimeThreshold =
            Constants.Swerve.drivePeakCurrentDuration;
        swerveDriveFXCurrentLimitsConfigs.SupplyCurrentLimit =
            Constants.Swerve.drivePeakCurrentLimit;
        swerveDriveFXConfig.CurrentLimits = swerveDriveFXCurrentLimitsConfigs;
        swerveDriveFXConfig.Feedback = new FeedbackConfigs().withFeedbackRotorOffset(0);

        swerveDriveFXConfig.OpenLoopRamps =
            new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(Constants.Swerve.openLoopRamp);
        swerveDriveFXConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs()
            .withTorqueClosedLoopRampPeriod(Constants.Swerve.closedLoopRamp);

        Slot0Configs swerveDriveFXConfigGains = new Slot0Configs();
        swerveDriveFXConfigGains.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfigGains.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfigGains.kD = Constants.Swerve.driveKD;



        /* Swerve CANCoder Configuration */
        swerveCanCoderMagnetSensorConfigs.AbsoluteSensorRange =
            AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderMagnetSensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        swerveCanCoderConfig.MagnetSensor = swerveCanCoderMagnetSensorConfigs;

        // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // swerveCanCoderConfig.initializationStrategy =
        // SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

}
