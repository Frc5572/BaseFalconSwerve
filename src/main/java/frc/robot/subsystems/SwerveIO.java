package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;



public class SwerveIO {

    @AutoLog
    public static class SwerveInputs {
        public float yaw;
        public float roll;
    }

    public default void updateInputs(SwerveInputs inputs) {}

    public default

    SwerveModule SwerveModule

    frc.robot.subsystems.SwerveModule createSwerveModule(int moduleNumber,
        frc.lib.util.swerve.SwerveModuleConstants constants) {
        return new SwerveModule(moduleNumber, constants, new SwerveModuleIO() {});
    }
}
