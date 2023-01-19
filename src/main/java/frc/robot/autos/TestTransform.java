package frc.robot.autos;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TestTransform extends CommandBase {

    private Swerve swerve;
    private Transform2d transform2d;

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(Constants.SwerveTransformPID.pidXkP, Constants.SwerveTransformPID.pidXkI,
            Constants.SwerveTransformPID.pidXkD),
        new PIDController(Constants.SwerveTransformPID.pidYkP, Constants.SwerveTransformPID.pidYkI,
            Constants.SwerveTransformPID.pidYkD),
        new ProfiledPIDController(Constants.SwerveTransformPID.pidTkP,
            Constants.SwerveTransformPID.pidTkI, Constants.SwerveTransformPID.pidTkD,
            new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.maxAngularVelocity,
                Constants.SwerveTransformPID.maxAngularAcceleration)));
    Pose2d pose2d = new Pose2d();

    public TestTransform(Swerve swerve, Transform2d transform2d) {

        super();
        this.swerve = swerve;
        this.transform2d = transform2d;
        this.addRequirements(swerve);
    }

    @Override
    public void execute() {
        ChassisSpeeds ctrlEffort =
            holonomicDriveController.calculate(swerve.getPose(), pose2d, 0, pose2d.getRotation());
        swerve.drive(new Translation2d(ctrlEffort.vxMetersPerSecond, ctrlEffort.vyMetersPerSecond),
            ctrlEffort.omegaRadiansPerSecond, false, false);
    }

    @Override
    public void initialize() {
        pose2d = swerve.getPose().plus(transform2d);
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}
