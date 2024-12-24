package frc.robot.subsystems.swerve;

import org.photonvision.PhotonCamera;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;

/** Real Class for Swerve */
public class SwerveReal implements SwerveIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);
    private Canandgyro newGyro = new Canandgyro(1);

    private SparkMax neo = new SparkMax(60, MotorType.kBrushless);
    private RelativeEncoder neoEncoder = neo.getEncoder();

    private PhotonCamera camera1 = new PhotonCamera("camera1");

    /** Real Swerve Initializer */
    public SwerveReal() {
        SparkBaseConfig neoConfig = new SparkMaxConfig().inverted(true).idleMode(IdleMode.kCoast);
        neo.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.newyaw = newGyro.getYaw();
        inputs.newpitch = newGyro.getPitch();
        inputs.newroll = newGyro.getRoll();

        inputs.neoPosition = neoEncoder.getPosition();
        inputs.neoVelocity = neoEncoder.getVelocity();

        inputs.camera1Connected = camera1.isConnected();
    }

    public SwerveModule createSwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, angleOffset,
            new SwerveModuleReal(driveMotorID, angleMotorID, cancoderID, angleOffset));
    }

    @Override
    public SwerveModule[] createModules() {
        return new SwerveModule[] {
            createSwerveModule(0, Constants.Swerve.Mod0.driveMotorID,
                Constants.Swerve.Mod0.angleMotorID, Constants.Swerve.Mod0.canCoderID,
                Constants.Swerve.Mod0.angleOffset),
            createSwerveModule(1, Constants.Swerve.Mod1.driveMotorID,
                Constants.Swerve.Mod1.angleMotorID, Constants.Swerve.Mod1.canCoderID,
                Constants.Swerve.Mod1.angleOffset),
            createSwerveModule(2, Constants.Swerve.Mod2.driveMotorID,
                Constants.Swerve.Mod2.angleMotorID, Constants.Swerve.Mod2.canCoderID,
                Constants.Swerve.Mod2.angleOffset),
            createSwerveModule(3, Constants.Swerve.Mod3.driveMotorID,
                Constants.Swerve.Mod3.angleMotorID, Constants.Swerve.Mod3.canCoderID,
                Constants.Swerve.Mod3.angleOffset)};
    }

    public void runNeo(double power) {
        neo.set(power);
    }

}
