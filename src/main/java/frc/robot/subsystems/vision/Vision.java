package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.AprilTagCamera.AprilTagCameraResult;
import frc.robot.subsystems.vision.AprilTagCamera.Resolution;

public class Vision extends SubsystemBase {
    private static final String VISIBLE_TAGS_LOG_ENTRY = "/VisibleTags";
    private static final String ESTIMATED_POSES_LOG_ENTRY = "/EstimatedPoses";
    private static final String OBJECT_DISTANCE_LOG_ENTRY = "/ObjectDistance";
    private static final String OBJECT_HEADING_LOG_ENTRY = "/ObjectHeading";
    private static final String OBJECT_POSE_LOG_ENTRY = "/ObjectPose";
    private static final String OBJECT_DETECTED_LOG_ENTRY = "/ObjectDetected";

    private static final double INTAKE_YAW_TOLERANCE = 10;

    private AtomicReference<List<AprilTagCameraResult>> m_estimatedRobotPoses;
    private AtomicReference<List<AprilTag>> m_visibleTags;
    private AtomicReference<List<Pose2d>> m_loggedEstimatedPoses;
    private AtomicReference<List<Pose3d>> m_visibleTagPoses;

    private ObjectCamera m_objectCamera;
    private AprilTagCamera[] m_apriltagCameras;
    private Notifier m_cameraNotifier;
    private AprilTagFieldLayout m_fieldLayout;
    private Supplier<Pose2d> m_poseSupplier;
    // private VisionSystemSim m_sim;
    private AprilTagCamera camera1 =
        new AprilTagCamera("camera1", new Transform3d(), Resolution.RES_1280_720, new Rotation2d());
    private ObjectCamera camera2 =
        new ObjectCamera("camera2", new Transform3d(), Resolution.RES_320_240, new Rotation2d());


    public Vision() {
        this.m_apriltagCameras = new AprilTagCamera[] {camera1};
        this.m_objectCamera = camera2;
        this.m_estimatedRobotPoses = new AtomicReference<List<AprilTagCameraResult>>();
        this.m_visibleTags = new AtomicReference<List<AprilTag>>();
        this.m_loggedEstimatedPoses = new AtomicReference<List<Pose2d>>();
        this.m_visibleTagPoses = new AtomicReference<List<Pose3d>>();
        // Load AprilTag field layout
        m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        // PV estimates will always be blue
        m_fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        this.m_cameraNotifier = (RobotBase.isReal()) ? new Notifier(() -> {
            for (var camera : m_apriltagCameras)
                camera.run();
            updateEstimatedGlobalPoses();
        }) : new Notifier(() -> {
            if (m_poseSupplier != null)
                // m_sim.update(m_poseSupplier.get());
                for (var camera : m_apriltagCameras)
                    camera.run();
            updateEstimatedGlobalPoses();
        });
        for (var camera : m_apriltagCameras)
            camera.setPipelineIndex(0);
        // Start camera thread
        m_cameraNotifier.setName(getName());
        m_cameraNotifier.startPeriodic(LoggedRobot.defaultPeriodSecs);
    }

    /**
     * Update currently estimated robot pose from each camera
     */
    private void updateEstimatedGlobalPoses() {
        var apriltagCameraResult = new ArrayList<AprilTagCameraResult>();

        var visibleTags = new ArrayList<AprilTag>();
        var loggedPoses = new ArrayList<Pose2d>();
        var visibleTagPoseList = new ArrayList<Pose3d>();
        for (var camera : m_apriltagCameras) {
            var results = camera.getLatestEstimatedPose();
            if (results == null)
                continue;
            results.forEach(result -> {
                result.estimatedRobotPose.targetsUsed.forEach((photonTrackedTarget) -> {
                    var tag = getTag(photonTrackedTarget.getFiducialId());
                    if (tag.isPresent()) {
                        visibleTags.add(tag.get());
                        visibleTagPoseList.add(tag.get().pose);
                    }
                });
                apriltagCameraResult.add(result);
                loggedPoses.add(result.estimatedRobotPose.estimatedPose.toPose2d());
            });
        }

        // Log visible tags and estimated poses
        m_visibleTagPoses.set(visibleTagPoseList);
        m_loggedEstimatedPoses.set(loggedPoses);

        m_visibleTags.set(visibleTags);
        m_estimatedRobotPoses.set(apriltagCameraResult);
    }

    /**
     * Set pose supplier for simulation
     *
     * @param poseSupplier Pose supplier from drive subsystem
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        m_poseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {
        camera2.periodic();
        List<AprilTagCameraResult> results = getEstimatedGlobalPoses();
        if (results != null) {
            results.stream().sorted(Comparator.comparingDouble(AprilTagCameraResult::timestamp))
                .forEach(RobotState.getInstance()::addVisionObservation);
        }
        // This method will be called once per scheduler run
        // var objectLocation = getObjectLocation();
        // Logger.recordOutput(getName() + OBJECT_DETECTED_LOG_ENTRY,
        // getObjectLocation().isPresent());
        // if (objectLocation.isEmpty())
        // return;
        Logger.recordOutput(getName() + "/shouldIntake", shouldIntake());
        Logger.recordOutput(getName() + "/objectVisible", objectIsVisible());

        // Logger.recordOutput(getName() + OBJECT_POSE_LOG_ENTRY, objectLocation.get());
    }

    /**
     * Get currently estimated robot poses from each camera
     *
     * @return List of estimated poses, the timestamp, and targets used to create the estimate
     */
    public List<AprilTagCameraResult> getEstimatedGlobalPoses() {
        List<Pose3d> tagPoses = m_visibleTagPoses.get();
        List<Pose2d> loggedEstimatedPoses = m_loggedEstimatedPoses.get();

        if (tagPoses != null && !tagPoses.isEmpty()) {
            Logger.recordOutput(getName() + VISIBLE_TAGS_LOG_ENTRY,
                tagPoses.toArray(new Pose3d[0]));
        }
        if (loggedEstimatedPoses != null && !loggedEstimatedPoses.isEmpty()) {
            Logger.recordOutput(getName() + ESTIMATED_POSES_LOG_ENTRY,
                loggedEstimatedPoses.toArray(new Pose2d[0]));
        }

        return m_estimatedRobotPoses.getAndSet(Collections.emptyList());
    }

    /**
     * Get IDs of currently visible tags
     *
     * @return List of IDs of currently visible tags
     */
    public List<AprilTag> getVisibleTags() {
        return m_visibleTags.get();
    }

    public Optional<AprilTag> getTag(int id) {
        return m_fieldLayout.getTags().stream().filter((tag) -> tag.ID == id).findFirst();
    }

    /**
     * Get the position of an object that can be seen by the object camera.
     *
     * @return The position of the object, relative to the field
     */
    // public Optional<Translation2d> getObjectLocation() {

    // Optional<Angle> yaw = m_objectCamera.getYaw();
    // Optional<Distance> distance = m_objectCamera.getDistance();
    // Pose2d pose = m_poseSupplier.get();
    // if (yaw.isEmpty() || distance.isEmpty() || pose == null)
    // return Optional.empty();

    // Logger.recordOutput(getName() + OBJECT_DISTANCE_LOG_ENTRY, distance.get());
    // Logger.recordOutput(getName() + OBJECT_HEADING_LOG_ENTRY, yaw.get());
    // return Optional.of(pose.getTranslation().plus(new Translation2d(
    // // distance.get().in(Units.Meters),
    // 1, Rotation2d
    // .fromRadians(pose.getRotation().getRadians() - yaw.get().in(Units.Radians)))));
    // }

    /**
     * Gets the object heading, relative to the camera.
     *
     * @return the heading
     */
    // public Optional<Angle> getObjectHeading() {
    // Optional<Angle> yaw = m_objectCamera.getYaw();
    // if (yaw.isEmpty())
    // return Optional.empty();
    // return yaw;
    // }

    public boolean shouldIntake() {
        if (!m_objectCamera.objectIsVisible())
            return false;
        double angle = m_objectCamera.getYaw().orElse(Units.Degrees.of(INTAKE_YAW_TOLERANCE))
            .in(Units.Degrees);
        Logger.recordOutput(getName() + "/angle111", angle);
        return Math.abs(angle) < INTAKE_YAW_TOLERANCE;
    }

    public boolean objectIsVisible() {
        return m_objectCamera.objectIsVisible();
    }
}
