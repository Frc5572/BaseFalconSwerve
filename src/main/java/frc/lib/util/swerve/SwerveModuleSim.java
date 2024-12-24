package frc.lib.util.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.Constants;

/**
 * Swerve Module Sim
 */
public class SwerveModuleSim implements SwerveModuleIO {
    public int moduleNumber;

    private FlywheelSim driveSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1),
            Constants.Swerve.driveGearRatio, 0.025), DCMotor.getFalcon500(1));

    private double angle;
    private double distance;

    private double driveAppliedVolts = 0.0;
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
    private PIDController driveFeedback = new PIDController(0.5, 0.0, 0.0);

    /**
     * Swerve Module Sim
     */
    public SwerveModuleSim() {}

    @Override
    public void setModNumber(int number) {
        this.moduleNumber = number;
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(LoggedRobot.defaultPeriodSecs);
        double driveSpeed = Units.radiansToRotations(driveSim.getAngularVelocityRadPerSec());
        this.distance += driveSpeed * LoggedRobot.defaultPeriodSecs;
        inputs.driveMotorSelectedPosition = this.distance;
        inputs.driveMotorSelectedSensorVelocity = driveSpeed;

        inputs.angleMotorSelectedPosition = angle;

        inputs.absolutePositionAngleEncoder = angle;
        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    }

    /**
     * Set drive motor in Meter per Sec
     *
     * @param mps Meters per Second
     */
    public void setDriveMotor(double mps) {
        double rpm = Conversions.metersPerSecondToRotationPerSecond(mps,
            Constants.Swerve.wheelCircumference);
        driveFeedback.setSetpoint(rpm);
        double driveFF = driveFeedforward.calculate(MetersPerSecond.of(mps).magnitude());
        SmartDashboard.putNumber("ff/" + moduleNumber, driveFF);
        double volts = driveFeedback.calculate(mps) + driveFF;
        if (rpm == 0) {
            volts = 0;
        }
        SmartDashboard.putNumber("Drive volts/" + moduleNumber, volts);
        setDriveVoltage(volts);
    }

    /**
     * Set Angle for steering motor
     *
     * @param angle Angle to set
     */
    public void setAngleMotor(double angle) {
        this.angle = angle;
    }

    /**
     * Set Drive Voltage
     *
     * @param volts Voltage
     */
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setDriveMotorPower(double power) {
        throw new UnsupportedOperationException("Unimplemented method 'setDriveMotorPower'");
    }

    @Override
    public void setAngleSelectedSensorPosition(double angle) {
        throw new UnsupportedOperationException(
            "Unimplemented method 'setAngleSelectedSensorPosition'");
    }

    @Override
    public void setPositionAngleMotor(double absolutePosition) {
        throw new UnsupportedOperationException("Unimplemented method 'setPositionAngleMotor'");
    }
}
