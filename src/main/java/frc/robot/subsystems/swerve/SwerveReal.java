package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.kauailabs.navx.frc.AHRS;
import frc.lib.util.swerve.SwerveModule;
import frc.robot.Constants;

/** Real Class for Swerve */
public class SwerveReal implements SwerveIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    /** Real Swerve Initializer */
    public SwerveReal() {

    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yaw = gyro.getYaw();
    }

    @Override
    public SwerveModule createSwerveModule(int moduleNumber, SwerveModuleConstants constants) {
        return new SwerveModule(moduleNumber, constants, new SwerveModuleReal(constants));
    }

}
