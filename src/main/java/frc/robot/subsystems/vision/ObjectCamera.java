package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.vision.AprilTagCamera.Resolution;

/**
 * Camera that looks for rings on the ground
 */
public class ObjectCamera implements AutoCloseable {
    private static final double TARGET_HEIGHT_METERS = 0;
    private static final double MIN_OBJECT_AREA = 0.1;

    private PhotonCamera m_camera;
    private PhotonCameraSim m_cameraSim;
    private Transform3d m_transform;
    private VisionTargetSim m_targetSim;
    private List<PhotonPipelineResult> results = new ArrayList<PhotonPipelineResult>();
    PhotonTrackedTarget bestTarget = null;

    /**
     * Create Object Camera
     *
     * @param name Name of device
     * @param transform Location on robot in meters
     * @param resolution Resolution used by camera
     * @param fovDiag Diagonal FOV of camera
     */
    public ObjectCamera(String name, Transform3d transform, Resolution resolution,
        Rotation2d fovDiag) {
        m_camera = new PhotonCamera(name);
        m_transform = transform;


        // TargetModel targetModel = new TargetModel(0.5, 0.25);
        // Pose3d targetPose = new Pose3d(FieldConstants.fieldLength, FieldConstants.fieldWidth / 2,
        // TARGET_HEIGHT_METERS, new Rotation3d(0, 0, Math.PI));

        // m_targetSim = new VisionTargetSim(targetPose, targetModel);
        // var cameraProperties = SimCameraProperties.PERFECT_90DEG();
        // cameraProperties.setCalibration(resolution.width, resolution.height, fovDiag);
        // m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);

        // Enable wireframe in sim camera stream
        // m_cameraSim.enableDrawWireframe(true);
    }

    public void periodic() {
        results = m_camera.getAllUnreadResults();
    }

    /**
     * Get simulated game object target
     *
     * @return Target sim
     */
    public VisionTargetSim getTargetSim() {
        return m_targetSim;
    }

    /**
     * Get camera sim
     *
     * @return Simulated camera object
     */
    public PhotonCameraSim getCameraSim() {
        return m_cameraSim;
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        Optional<List<PhotonPipelineResult>> results = getLatestResults();
        if (!results.isPresent()
            || results.get().stream().filter(x -> x.hasTargets()).toList().size() == 0)
            return Optional.empty();

        results.get().stream().forEach(result -> {
            List<PhotonTrackedTarget> targets = result.getTargets();

            double bestTargetScore = Double.MAX_VALUE; // lower is better
            for (var target : targets) {
                double avgX = 0, avgY = 0;
                int count = 0;

                for (var corner : target.getMinAreaRectCorners()) {
                    avgX += corner.x;
                    avgY += corner.y;
                    count++;
                }
                avgX /= count;
                avgY /= count;

                // TODO get frame size from photonvision
                double score = Math.hypot(avgX - (1920 / 2), avgY - 1080);

                if (score < bestTargetScore) {
                    bestTarget = target;
                    bestTargetScore = score;
                }
            }
        });
        if (bestTarget == null)
            return Optional.empty();
        return Optional.of(bestTarget);
    }

    /**
     * Get distance to game object
     *
     * @return Distance to object, empty if undetected
     */
    // public Optional<Distance> getDistance() {
    // if (getObjectArea().orElse(0.0) < MIN_OBJECT_AREA)
    // return Optional.empty();

    // Optional<PhotonPipelineResult> result = getLatestResult();
    // if (!result.isPresent() || !result.get().hasTargets())
    // return Optional.empty();

    // double range = PhotonUtils.calculateDistanceToTargetMeters(m_transform.getZ(),
    // TARGET_HEIGHT_METERS, -m_transform.getRotation().getY(),
    // Units.Degrees.of(result.get().getBestTarget().getPitch()).in(Units.Radians));

    // return Optional.of(Units.Meters.of(range));
    // }

    /**
     * Get yaw angle to target
     *
     * @return Yaw angle to target, empty if undetected
     */
    public Optional<Angle> getYaw() {
        Optional<PhotonTrackedTarget> target = getBestTarget();
        if (!target.isPresent()) {
            return Optional.empty();
        }
        if (target.get().getArea() < MIN_OBJECT_AREA)
            return Optional.empty();
        return Optional.of(Units.Degrees.of(target.get().getYaw()));
    }

    /**
     * Get camera to robot transform
     *
     * @return Camera to robot transform
     */
    public Transform3d getTransform() {
        return m_transform;
    }

    // public Optional<Double> getObjectArea() {
    // Optional<PhotonTrackedTarget> target = getBestTarget();
    // if (!target.isPresent()) {
    // return Optional.empty();
    // }
    // return Optional.of(target.get().getArea());
    // }

    public boolean objectIsVisible() {
        Optional<List<PhotonPipelineResult>> result = getLatestResults();
        return result.isPresent() && result.get().stream().anyMatch(x -> x.hasTargets());
    }

    public Optional<List<PhotonPipelineResult>> getLatestResults() {
        if (results.size() > 0) {
            return Optional.of(results);
        }
        return Optional.empty();
    }

    public Optional<PhotonTrackedTarget> getLatestTargets() {
        Optional<List<PhotonPipelineResult>> result = getLatestResults();
        if (!result.isPresent()) {
            return Optional.empty();
        }
        List<PhotonPipelineResult> targets =
            result.get().stream().filter(x -> x.hasTargets()).toList();
        if (targets.size() > 0) {
            return Optional.of(targets.get(0).getBestTarget());
        }
        return Optional.empty();
    }

    @Override
    public void close() {
        m_camera.close();
    }
}
