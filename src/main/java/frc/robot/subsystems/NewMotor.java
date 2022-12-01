package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem file for the extra motor
 */
public class NewMotor extends SubsystemBase {
    TalonFX extraMotor;

    public NewMotor() {
        extraMotor = new TalonFX(9);
    }

    public void move() {
        extraMotor.set(ControlMode.PercentOutput, .1);
    }

    public void stop() {
        extraMotor.set(ControlMode.PercentOutput, 0);
    }
}
