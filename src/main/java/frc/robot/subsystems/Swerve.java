package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.swerve.SwerveModule;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveMods;
    public AHRS gyro = new AHRS(Constants.Swerve.navXID);
    private double fieldOffset = gyro.getYaw();
    private boolean hasInitialized = false;

    public Swerve() {
        swerveMods = new SwerveModule[] {new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)};

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(),
            getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative,
        boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                rotation, Rotation2d.fromDegrees(gyro.getYaw() - fieldOffset))
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        setModuleStates(chassisSpeeds);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * Sets swerve module states using Chassis Speeds.
     *
     * @param chassisSpeeds The desired Chassis Speeds
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(swerveModuleStates);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
            new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
            new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }


    /**
     * Resets the gyro field relative driving offset
     */
    public void resetFieldRelativeOffset() {
        // gyro.zeroYaw();
        fieldOffset = getGyroYaw().getDegrees();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder",
                mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle",
                mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                mod.getState().speedMetersPerSecond);
        }
    }

    public void resetInitialized() {
        this.hasInitialized = false;
    }

    /**
     * Sets motors to 0 or inactive.
     *
     * @param isOpenLoop Open or closed loop system
     * @param fieldRelative Whether the movement is relative to the field or absolute
     */
    public void setMotorsZero(boolean isOpenLoop, boolean fieldRelative) {
        System.out.println("Setting Zero!!!!!!");
        setModuleStates(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * New command to set wheels inward.
     */
    public void wheelsIn() {
        swerveMods[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), false);
        swerveMods[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), false);
        swerveMods[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)), false);
        swerveMods[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)),
            false);
        this.setMotorsZero(Constants.Swerve.isOpenLoop, Constants.Swerve.isFieldRelative);
    }
}
