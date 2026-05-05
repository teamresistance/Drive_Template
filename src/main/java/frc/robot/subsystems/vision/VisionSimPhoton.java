package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.GeomUtil;
import frc.robot.util.TimestampedVisionUpdate;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSimPhoton implements VisionIOPhoton {

  private static final String VISION_SIM_LABEL = "PhotonVisionSim";
  private static final double FIELD_BORDER_MARGIN = 0.5;

  private static final int CAM_RES_WIDTH = 1280;
  private static final int CAM_RES_HEIGHT = 800;
  private static final double CAM_DIAG_FOV_DEG = 75.0;
  private static final double CAM_AVG_LATENCY_MS = 30.0;
  private static final double CAM_LATENCY_STD_DEV_MS = 5.0;
  private static final double CAM_FPS = 60.0;
  private static final double CAM_CALIB_ERROR_AVG = 0.25;
  private static final double CAM_CALIB_ERROR_STD_DEV = 0.08;

  private static double stdDevScalar = stdDevScalarDefault;

  private static final Pose3d[] CAMERA_POSES = CameraPoses.poses;

  private final VisionSystemSim visionSim;
  private final PhotonCameraSim[] cameraSims;
  private final PhotonCamera[] cameras;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Supplier<Pose2d> poseSupplier = Pose2d::new;
  private List<TimestampedVisionUpdate> visionUpdates;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = x -> {};

  /**
   * Creates a new VisionSim with the specified PhotonCameras. Pass the same camera instances used
   * in {@link VisionRealPhoton} so simulated results feed directly into real subsystem logic.
   *
   * @param cameras The PhotonVision cameras to simulate, in the same order as in VisionSubsystem
   */
  public VisionSimPhoton(PhotonCamera... cameras) {
    register();
    this.cameras = cameras;
    visionSim = new VisionSystemSim(VISION_SIM_LABEL);

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
      visionSim.addAprilTags(aprilTagFieldLayout);
    } catch (IOException e) {
      Logger.recordOutput("Photon/FieldLayoutLoadError", e.getMessage());
    }

    SimCameraProperties camProps = buildCameraProperties();
    cameraSims = new PhotonCameraSim[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      cameraSims[i] = new PhotonCameraSim(cameras[i], camProps);

      // disable by default
      cameraSims[i].enableRawStream(false);
      cameraSims[i].enableProcessedStream(false);

      Transform3d robotToCamera =
          new Transform3d(CAMERA_POSES[i].getTranslation(), CAMERA_POSES[i].getRotation());

      visionSim.addCamera(cameraSims[i], robotToCamera);
    }
  }

  /**
   * Sets the data interfaces for pose estimation integration.
   *
   * @param poseSupplier Supplier that provides the current estimated robot pose
   * @param visionConsumer Consumer that accepts vision measurement updates
   */
  @Override
  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  // MAKE SURE SIM AND REAL METHODS MATCH WHEN CHANGING!
  @Override
  public void periodic() {
    visionSim.update(poseSupplier.get());

    Pose2d currentPose = poseSupplier.get();
    visionUpdates = new ArrayList<>();

    double singleTagAdjustment = 1.0;
    if (Constants.TUNING_MODE) SingleTagAdjustment.updateLoggedTagAdjustments();

    // Loop through all the cameras
    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {
      // Camera-specific variables
      Pose3d cameraPose;
      Pose2d robotPose;
      List<Pose3d> tagPose3ds = new ArrayList<>();

      List<PhotonPipelineResult> unprocessedResults = cameras[instanceIndex].getAllUnreadResults();

      if (unprocessedResults.isEmpty()) continue;

      PhotonPipelineResult unprocessedResult =
          unprocessedResults.get(unprocessedResults.size() - 1);

      Logger.recordOutput(
          LOGGING_KEY_PREFIX + instanceIndex + " Has Targets", unprocessedResult.hasTargets());
      Logger.recordOutput(
          LOGGING_KEY_PREFIX + instanceIndex + "LatencyMS",
          unprocessedResult.metadata.getLatencyMillis());

      Logger.recordOutput(
          "Photon/Raw Camera Data " + instanceIndex,
          SmartDashboard.getRaw(
              "photonvision/" + cameras[instanceIndex].getName() + "/rawBytes", new byte[] {}));

      // Continue if the camera doesn't have any targets
      if (!unprocessedResult.hasTargets()) {
        Logger.recordOutput("Photon/Tags Used " + instanceIndex, 0);
        Logger.recordOutput("Photon/Camera Pose " + instanceIndex, new Pose3d());
        Logger.recordOutput("Photon/Camera" + instanceIndex + "/TagPoses", new Pose3d[0]);
        continue;
      }

      double timestamp = unprocessedResult.getTimestampSeconds();
      Logger.recordOutput(LOGGING_KEY_PREFIX + instanceIndex + " Timestamp", timestamp);

      // multiple tags detected or not
      boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().isPresent();
      Logger.recordOutput("Photon/UsingMultitag " + instanceIndex, shouldUseMultiTag);

      if (shouldUseMultiTag) {
        // If multitag, use directly
        var result = unprocessedResult.getMultiTagResult().get();

        cameraPose = GeomUtil.transform3dToPose3d(result.estimatedPose.best);

        robotPose =
            cameraPose
                .transformBy(GeomUtil.pose3dToTransform3d(CAMERA_POSES[instanceIndex]).inverse())
                .toPose2d();

        // Populate array of tag poses with tags used
        for (int id : result.fiducialIDsUsed) {
          if (aprilTagFieldLayout.getTagPose(id).isPresent()) {
            tagPose3ds.add(aprilTagFieldLayout.getTagPose(id).get());
          }
        }

        Logger.recordOutput(
            "Photon/Camera" + instanceIndex + "/TagPoses", tagPose3ds.toArray(new Pose3d[0]));
        Logger.recordOutput("Photon/Camera Pose " + instanceIndex, cameraPose);
        Logger.recordOutput(
            "VisionSim/Camera" + instanceIndex + "/TagPoses", tagPose3ds.toArray(new Pose3d[0]));
      } else {
        // If not using multitag, disambiguate and then use
        PhotonTrackedTarget target = unprocessedResult.targets.get(0);

        // discard apriltags that don't exist on the field
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isEmpty()) {
          continue;
        }

        Pose3d tagPos = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();

        // transform camera pose by the origin-to-cam transform so the origin is the robot origin
        Pose3d cameraPose0 = tagPos.transformBy(target.getBestCameraToTarget().inverse());
        Pose3d cameraPose1 = tagPos.transformBy(target.getAlternateCameraToTarget().inverse());
        Pose2d robotPose0 =
            cameraPose0
                .transformBy(GeomUtil.pose3dToTransform3d(CAMERA_POSES[instanceIndex]).inverse())
                .toPose2d();
        Pose2d robotPose1 =
            cameraPose1
                .transformBy(GeomUtil.pose3dToTransform3d(CAMERA_POSES[instanceIndex]).inverse())
                .toPose2d();

        double projectionError = target.getPoseAmbiguity();

        // Select a pose using projection error and current rotation
        if (projectionError < 0.15
            || (Math.abs(robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
                < Math.abs(
                    robotPose1.getRotation().minus(currentPose.getRotation()).getRadians()))) {
          cameraPose = cameraPose0;
          robotPose = robotPose0;
        } else {
          cameraPose = cameraPose1;
          robotPose = robotPose1;
        }

        tagPose3ds.add(tagPos);

        singleTagAdjustment = SingleTagAdjustment.getAdjustmentForTag(target.getFiducialId());
        Logger.recordOutput("Photon/Camera Pose " + instanceIndex, cameraPose);
        Logger.recordOutput(
            "Photon/Camera" + instanceIndex + "/TagPoses", tagPose3ds.toArray(new Pose3d[0]));
      }

      if (robotPose == null) {
        continue;
      }

      // Move on to next camera if robot pose is off the field
      if (robotPose.getX() < -FIELD_BORDER_MARGIN
          || robotPose.getX() > aprilTagFieldLayout.getFieldLength() + FIELD_BORDER_MARGIN
          || robotPose.getY() < -FIELD_BORDER_MARGIN
          || robotPose.getY() > aprilTagFieldLayout.getFieldWidth() + FIELD_BORDER_MARGIN) {
        continue;
      }

      // Calculate average distance to tag
      double totalDistance = 0.0;
      for (Pose3d tagPose : tagPose3ds) {
        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
      }

      double avgDistance = totalDistance / tagPose3ds.size();
      double xyStdDev;
      double thetaStdDev;

      //      if (shouldUseMultiTag) {
      //        // use default multitag std dev
      //        xyStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
      //        thetaStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
      //      } else {
      //        // use polynomial model
      //        xyStdDev = XY_STD_DEV_MODEL.predict(avgDistance);
      //        thetaStdDev = THETA_STD_DEV_MODEL.predict(avgDistance);
      //      }
      xyStdDev = XY_STD_DEV_MODEL.predict(avgDistance);
      thetaStdDev = THETA_STD_DEV_MODEL.predict(avgDistance);

      // add results to the vision updates
      if (shouldUseMultiTag) {
        visionUpdates.add(
            new TimestampedVisionUpdate(
                robotPose,
                timestamp,
                VecBuilder.fill(
                    stdDevScalar * MULTITAG_STD_DEV_SCALAR * xyStdDev,
                    stdDevScalar * MULTITAG_STD_DEV_SCALAR * xyStdDev,
                    stdDevScalar * MULTITAG_STD_DEV_SCALAR * thetaStdDev)));
      } else {
        visionUpdates.add(
            new TimestampedVisionUpdate(
                robotPose,
                timestamp,
                VecBuilder.fill(
                    singleTagAdjustment * xyStdDev * stdDevScalar,
                    singleTagAdjustment * xyStdDev * stdDevScalar,
                    singleTagAdjustment * thetaStdDev * stdDevScalar)));

        Logger.recordOutput("VisionData/" + instanceIndex, robotPose);
        Logger.recordOutput("Photon/Tags Used " + instanceIndex, tagPose3ds.size());
      }
    }

    // Apply all vision updates to pose estimator
    visionConsumer.accept(visionUpdates);
  }

  @Override
  public void updateStdDevScalar(double newScalar) {
    stdDevScalar = newScalar;
  }

  /**
   * enables the raw and processed sim camera streams for a specific camera, off by default
   *
   * @param cameraIndex index of the camera
   */
  public void enableStreams(int cameraIndex) {
    cameraSims[cameraIndex].enableRawStream(true);
    cameraSims[cameraIndex].enableProcessedStream(true);
  }

  private static SimCameraProperties buildCameraProperties() {
    SimCameraProperties props = new SimCameraProperties();
    props.setCalibration(CAM_RES_WIDTH, CAM_RES_HEIGHT, Rotation2d.fromDegrees(CAM_DIAG_FOV_DEG));
    props.setCalibError(CAM_CALIB_ERROR_AVG, CAM_CALIB_ERROR_STD_DEV);
    props.setFPS(CAM_FPS);
    props.setAvgLatencyMs(CAM_AVG_LATENCY_MS);
    props.setLatencyStdDevMs(CAM_LATENCY_STD_DEV_MS);
    return props;
  }
}
