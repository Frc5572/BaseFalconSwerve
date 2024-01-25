package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.kauailabs.navx.frc.AHRS;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveIO.SwerveInputs;

public class SwerveReal implements SwerveOI {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    // Real Swereve Initializer

    public SwerveReal() {}

    @Override

    public void updateInputs(SwerveInputs inputs) {
        inputs.yaw = gyro.getYaw();
    }

    @Override
    public SwerveModule creatSwerveModule(int moduleNumber, SwerveModuleConstants constants) {
        return new SwerveModule(moduleNumber, constants, new SwerveModuleReal(constants));
    }

}
