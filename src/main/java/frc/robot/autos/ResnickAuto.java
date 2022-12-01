package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ResnickAuto extends frc.robot.commands.AutoBase {
    public ResnickAuto(Swerve swerve) {
        super(swerve);
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", 1, .5);
        PPSwerveControllerCommand resnick = baseSwerveCommand(trajectory);
        PathPlannerState initialState = trajectory.getInitialState();
        addCommands(new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            resnick);
    }

    // PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", 6, 3);
    // PPSwerveControllerCommand resnick = baseSwerveCommand(trajectory);

    // SwerveAutoBuilder a = new SwerveAutoBuilder();


}
