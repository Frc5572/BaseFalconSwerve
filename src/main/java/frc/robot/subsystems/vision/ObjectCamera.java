// package frc.robot.subsystems.vision;

// import java.util.List;
// import java.util.Optional;
// import org.littletonrobotics.junction.Logger;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.VisionTargetSim;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.units.measure.Units;
// import edu.wpi.first.units.measure.measure.Angle;
// import edu.wpi.first.units.measure.measure.Distance;
// import frc.robot.subsystems.vision.AprilTagCamera.Resolution;

// /**
//  * Camera that looks for rings on the ground
//  */
// public class ObjectCamera implements AutoCloseable {
//     private static final double TARGET_HEIGHT_METERS = 0;
//     private static final double MIN_OBJECT_AREA = 0.1;

//     private PhotonCamera m_camera;
//     private PhotonCameraSim m_cameraSim;
//     private Transform3d m_transform;
//     private VisionTargetSim m_targetSim;
//     Optional<PhotonTrackedTarget> bestTarget = null;
//     boolean hasTargets = false;
//     int counter = 0;

//     /**
//      * Create Object Camera
//      *
//      * @param name Name of device
//      * @param transform Location on robot in meters
//      * @param resolution Resolution used by camera
//      * @param fovDiag Diagonal FOV of camera
//      */
//     public ObjectCamera(String name, Transform3d transform, Resolution resolution,
//         Rotation2d fovDiag) {
//         m_camera = new PhotonCamera(name);
//         m_transform = transform;


//         // TargetModel targetModel = new TargetModel(0.5, 0.25);
//         // Pose3d targetPose = new Pose3d(FieldConstants.fieldLength, FieldConstants.fieldWidth / 2,
//         // TARGET_HEIGHT_METERS, new Rotation3d(0, 0, Math.PI));

//         // m_targetSim = new VisionTargetSim(targetPose, targetModel);
//         // var cameraProperties = SimCameraProperties.PERFECT_90DEG();
//         // cameraProperties.setCalibration(resolution.width, resolution.height, fovDiag);
//         // m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);

//         // Enable wireframe in sim camera stream
//         // m_cameraSim.enableDrawWireframe(true);
//     }

//     public void periodic() {
//         List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
//         Logger.recordOutput("Object/ResultEmpty", results.isEmpty());
//         if (!results.isEmpty()) {
//             PhotonPipelineResult recent = results.get(results.size() - 1);
//             Logger.recordOutput("Object/Result", recent);
//             if (recent.hasTargets()) {
//                 hasTargets = true;
//                 bestTarget = Optional.of(recent.getBestTarget());
//             }
//         }
//     }

//     /**
//      * Get simulated game object target
//      *
//      * @return Target sim
//      */
//     public VisionTargetSim getTargetSim() {
//         return m_targetSim;
//     }

//     /**
//      * Get camera sim
//      *
//      * @return Simulated camera object
//      */
//     public PhotonCameraSim getCameraSim() {
//         return m_cameraSim;
//     }

//     // public Optional<PhotonTrackedTarget> getBestTarget() {
//     // List<PhotonTrackedTarget> targets = m_camera.getLatestResult().getTargets();

//     // PhotonTrackedTarget bestTarget = null;
//     // double bestTargetScore = Double.MAX_VALUE; // lower is better
//     // for (var target : targets) {
//     // double avgX = 0, avgY = 0;
//     // int count = 0;

//     // for (var corner : target.getMinAreaRectCorners()) {
//     // avgX += corner.x;
//     // avgY += corner.y;
//     // count++;
//     // }
//     // avgX /= count;
//     // avgY /= count;

//     // double score = Math.hypot(avgX - (m_resolution.width / 2), avgY - m_resolution.height);


//     // if (score < bestTargetScore) {
//     // bestTarget = target;
//     // bestTargetScore = score;
//     // }
//     // }

//     // if (bestTarget == null)
//     // return Optional.empty();
//     // return Optional.of(bestTarget);
//     // }

//     /**
//      * Get distance to game object
//      *
//      * @return Distance to object, empty if undetected
//      */
//     public Optional<Distance> getDistance() {
//         if (!hasTargets || !bestTarget.isPresent()
//             || getObjectArea().orElse(0.0) < MIN_OBJECT_AREA) {
//             return Optional.empty();
//         }

//         double range = PhotonUtils.calculateDistanceToTargetMeters(m_transform.getZ(),
//             TARGET_HEIGHT_METERS, -m_transform.getRotation().getY(),
//             Units.Degrees.of(bestTarget.get().getPitch()).in(Units.Radians));

//         return Optional.of(Units.Meters.of(range));
//     }

//     /**
//      * Get yaw angle to target
//      *
//      * @return Yaw angle to target, empty if undetected
//      */
//     public Optional<Angle> getYaw() {
//         if (!hasTargets || !bestTarget.isPresent()
//             || getObjectArea().orElse(0.0) < MIN_OBJECT_AREA) {
//             return Optional.empty();
//         }
//         return Optional.of(Units.Degrees.of(bestTarget.get().getYaw()));
//     }

//     /**
//      * Get camera to robot transform
//      *
//      * @return Camera to robot transform
//      */
//     public Transform3d getTransform() {
//         return m_transform;
//     }

//     public Optional<Double> getObjectArea() {
//         if (!hasTargets || !bestTarget.isPresent()) {
//             return Optional.empty();
//         }
//         return Optional.of(bestTarget.get().getArea());
//     }


//     public boolean objectIsVisible() {
//         return hasTargets;
//     }

//     // public Optional<PhotonPipelineResult> getLatestResult() {
//     // if (results.isEmpty()) {
//     // return Optional.empty();
//     // }
//     // return Optional.of(results.get(results.size() - 1));
//     // }

//     @Override
//     public void close() {
//         m_camera.close();
//     }
// }
