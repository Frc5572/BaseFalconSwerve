package frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {


    @SuppressWarnings("IOCheck")
    private TalonFX motor = new TalonFX(27);

    @SuppressWarnings("IOCheck")
    private TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    /* angle motor control requests */

    @SuppressWarnings("IOCheck")
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    @SuppressWarnings("IOCheck")
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

    @SuppressWarnings("IOCheck")
    DigitalInput bob = new DigitalInput(9);

    boolean mode = true;

    private LoggedTunableNumber kp = new LoggedTunableNumber("Elevator/kP", 0.0);
    private LoggedTunableNumber ki = new LoggedTunableNumber("Elevator/kI", 0.0);
    private LoggedTunableNumber kd = new LoggedTunableNumber("Elevator/kD", 0.0);
    private LoggedTunableNumber ff = new LoggedTunableNumber("Elevator/ff", 0.0);

    public Elevator() {
        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = kp.getAsDouble();
        swerveAngleFXConfig.Slot0.kI = ki.getAsDouble();
        swerveAngleFXConfig.Slot0.kD = kd.getAsDouble();
        swerveAngleFXConfig.Slot0.kG = ff.getAsDouble();
        swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = 45.75;


        motor.getConfigurator().apply(swerveAngleFXConfig);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("asdfads", bob.get());
        if (bob.get()) {
            motor.setPosition(0);
        }
        if (kp.hasChanged(kp.hashCode()) || ki.hasChanged(ki.hashCode())
            || kd.hasChanged(kd.hashCode()) || ff.hasChanged(ff.hashCode())) {
            swerveAngleFXConfig.Slot0.kP = kp.getAsDouble();
            swerveAngleFXConfig.Slot0.kI = ki.getAsDouble();
            swerveAngleFXConfig.Slot0.kD = kd.getAsDouble();
            swerveAngleFXConfig.Slot0.kG = ff.getAsDouble();
            motor.getConfigurator().apply(swerveAngleFXConfig);
        }
    }

    public Command setPosition(double pos) {
        return Commands.run(() -> motor.setControl(anglePosition.withPosition(pos)), this);
    }

    public Command driveElevator(DoubleSupplier power) {
        return Commands.run(() -> {
            if (mode) {
                double p = MathUtil.applyDeadband(-power.getAsDouble(), .25);
                if (bob.get() && p < 0) {
                    p = 0;
                }
                motor.setControl(driveDutyCycle.withOutput(p));
            } else {
                motor.setControl(anglePosition.withSlot(0).withPosition(0));
            }
        }, this);
    }



}
