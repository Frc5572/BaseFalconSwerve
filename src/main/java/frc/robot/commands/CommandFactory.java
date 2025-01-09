package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * File to create commands using factories
 */
public class CommandFactory {

    /**
     * Rotate to gamepiece detected via Photon Vision
     *
     * @param swerve Swerve Subsystem
     * @param objectYawSupplier Supplier of Yaw to which to rotate
     * @return Command
     */
    public static Command rotateToGamePiece(Swerve swerve,
        Supplier<Optional<Angle>> objectYawSupplier) {
        HolonomicDriveController holonomicDriveController =
            new HolonomicDriveController(new PIDController(0, 0, 0), new PIDController(0, 0, 0),
                new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP / 2,
                    Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
                    new TrapezoidProfile.Constraints(
                        Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                        Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(1)));
        return new FunctionalCommand(() -> {
        }, () -> {
            Optional<Angle> yaw = objectYawSupplier.get();
            if (yaw.isPresent()) {
                Angle yawAngle = yaw.get();
                ChassisSpeeds chassisSpeeds = holonomicDriveController.calculate(
                    new Pose2d(0, 0, Rotation2d.fromDegrees(yawAngle.in(Degrees))),
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 0, Rotation2d.fromDegrees(0));
                swerve.setModuleStates(chassisSpeeds);
            }
        }, x -> swerve.setMotorsZero(), () -> holonomicDriveController.atReference(), swerve);
    }
}
