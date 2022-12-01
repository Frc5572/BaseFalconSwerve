package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewMotor;

/**
 * Moves the new motor on the test bot
 */
public class moveNewMotor extends CommandBase {
    NewMotor s_NewMotor;

    /**
     * Initalizes new motor
     *
     * @param subsystem motor ID
     */
    public moveNewMotor(NewMotor subsystem) {
        this.s_NewMotor = subsystem;
    }

    @Override
    public void initialize() {
        addRequirements(s_NewMotor);
    }

    @Override
    public void execute() {
        s_NewMotor.move();
    }


    @Override
    public void end(boolean interrupted) {
        s_NewMotor.stop();
    }

}
