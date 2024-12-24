package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FieldConstants;
import frc.lib.util.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Swerve Subsystem
 */
public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] swerveMods;
    private final Field2d field = new Field2d();
    private double fieldOffset;
    private SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
    private SwerveIO swerveIO;


    /**
     * Gyro
     */
    public enum SwerveGyro {
        NAVX, CANAND
    }

    /**
     * Swerve Subsystem
     */
    public Swerve(SwerveIO swerveIO) {
        this.swerveIO = swerveIO;
        swerveMods = swerveIO.createModules();
        fieldOffset = getGyroYaw().getDegrees();

        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
            getGyroYaw(), getModulePositions(), new Pose2d());

        swerveIO.updateInputs(inputs);

        // AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry,
        // this::getChassisSpeeds,
        // this::setModuleStates, Constants.Swerve.pathFollowerConfig, () -> shouldFlipPath(),
        // this);

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
    }

    /**
     * Tele-Op Drive method
     *
     * @param translation The magnitude in XY
     * @param rotation The magnitude in rotation
     * @param fieldRelative Whether or not field relative
     * @param isOpenLoop Whether or not Open or Closed Loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative,
        boolean isOpenLoop) {
        Robot.profiler.push("swerve.drive()");
        ChassisSpeeds chassisSpeeds =
            new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        if (fieldRelative) {
            chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getFieldRelativeHeading());
        }

        setModuleStates(chassisSpeeds);
        Robot.profiler.pop();
    }

    /**
     * Set Swerve Module States
     *
     * @param desiredStates Array of desired states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        Logger.recordOutput("/Swerve/DesiredStates", desiredStates);
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
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond,
            LoggedRobot.defaultPeriodSecs);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Get current Chassis Speeds
     *
     * @return The current {@link ChassisSpeeds}
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Get Swerve Module States
     *
     * @return Array of Swerve Module States
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Get states based on ABS encoders
     *
     * @return SwerveModuleStates
     */
    @AutoLogOutput(key = "Absolute States")
    public SwerveModuleState[] gModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getStateAbs();
        }
        return states;
    }

    /**
     * Get Swerve Module Positions
     *
     * @return Array of Swerve Module Positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Get Position on field from Odometry
     *
     * @return Pose2d on the field
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * Set the position on the field with given Pose2d
     *
     * @param pose Pose2d to set
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        this.swerveIO.setPose(pose);
    }

    /**
     * Get Rotation of robot from odometry
     *
     * @return Heading of robot relative to the field as {@link Rotation2d}
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get Rotation from the gyro
     *
     * @return Current rotation/yaw of gyro as {@link Rotation2d}
     */
    public Rotation2d getGyroYaw() {
        Rotation2d robotYaw = switch (Constants.Swerve.selectedGyro) {
            case NAVX -> {
                float yaw = inputs.yaw;
                yield (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(-yaw)
                    : Rotation2d.fromDegrees(yaw);

            }
            case CANAND -> {
                double yaw = inputs.newyaw;
                yield Rotation2d.fromRotations(yaw);

            }
            default -> new Rotation2d();
        };
        return robotYaw;
    }

    /**
     * Get Field Relative Heading
     *
     * @return The current field relative heading in {@link Rotation2d}
     */
    public Rotation2d getFieldRelativeHeading() {
        return Rotation2d.fromDegrees(getGyroYaw().getDegrees() - fieldOffset);
    }

    /**
     * Resets the gyro field relative driving offset
     */
    public void resetFieldRelativeOffset() {
        // gyro.zeroYaw();
        fieldOffset = getGyroYaw().getDegrees() + 180;
    }

    @Override
    public void periodic() {
        // Robot.profiler.push("swerve_periodic");
        // Robot.profiler.push("update_inputs");
        swerveIO.updateInputs(inputs);
        // Robot.profiler.swap("update_swerve_mods");
        for (var mod : swerveMods) {
            mod.periodic();
        }
        // Robot.profiler.swap("update_swerve_odometry");
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        // Robot.profiler.swap("process_inputs");
        Logger.processInputs("Swerve", inputs);
        // Robot.profiler.swap("update_shuffleboard");
        // Robot.profiler.push("field");
        field.setRobotPose(getPose());
        // Robot.profiler.swap("apriltag");

        // Robot.profiler.swap("dist-to-speaker");
        SmartDashboard.putNumber("Distance to Speaker",
            FieldConstants.allianceFlip(FieldConstants.Speaker.centerSpeakerOpening)
                .getTranslation().minus(getPose().getTranslation()).getNorm());
        // Robot.profiler.swap("simple");
        SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());
        Logger.recordOutput("/Swerve/ActualStates", getModuleStates());
        // Robot.profiler.pop();
        // Robot.profiler.pop();
        // Robot.profiler.swap("viz");
        // Robot.profiler.pop();
    }

    /**
     * Sets motors to 0 or inactive.
     */
    public void setMotorsZero() {
        System.out.println("Setting Zero!!!!!!");
        setModuleStates(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Make an X pattern with the wheels
     */
    public void wheelsIn() {
        swerveMods[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), false);
        swerveMods[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), false);
        swerveMods[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)), false);
        swerveMods[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)),
            false);
        this.setMotorsZero();
    }

    /**
     * Gets a list containing all 4 swerve module positions
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Determine whether or not to flight the auto path
     *
     * @return True if flip path to Red Alliance, False if Blue
     */
    public static boolean shouldFlipPath() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red;
        }
        return false;
    }

    /**
     * Returns the distance between the speaker and Swerve
     *
     * @return the difference between the pose of speaker and swerve
     */
    public double distanceFromSpeaker() {
        double distance =
            Math.hypot(FieldConstants.Speaker.centerSpeakerOpening.getY() - getPose().getY(),
                FieldConstants.allianceFlip(FieldConstants.Speaker.centerSpeakerOpening).getX()
                    - getPose().getX());
        return distance;
    }

    public Command runNeo(double power) {
        return Commands.runEnd(() -> swerveIO.runNeo(power), () -> swerveIO.runNeo(0), this);
    }
}
