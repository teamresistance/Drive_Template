package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.TimestampedVisionUpdate;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  private static String LOGGING_KEY_PREFIX = "Photon/Camera ";

  private static final double FIELD_BORDER_MARGIN = 0.5;
  private static final Pose3d[] CAMERA_POSES = CameraPoses.poses;

  private final PhotonCamera[] cameras;
  /* For shooting vs. path following in auto */
  private static final double STD_DEV_SCALAR_SHOOTING = 1.6;
  private static final double THETA_STD_DEV_COEFFICIENT_SHOOTING = 0.075;
  private static final PolynomialRegression XY_STD_DEV_MODEL =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358, 6.0
          },
          new double[] {0.005, 0.0135, 0.016, 0.028, 0.0815, 2.4, 3.62, 5.7, 5.9, 5.3, 20.0, 25.0},
          2);
  private static final PolynomialRegression THETA_STD_DEV_MODEL =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358, 6
          },
          new double[] {
            0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.089, 2.027, 3.459, 4.629, 6.068, 13.0
          },
          1);

  public final LoggedTunableNumber XY_STD_DEV_THRESHOLD_1 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 1 meter", 0.01);
  public final LoggedTunableNumber XY_STD_DEV_THRESHOLD_2 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 2 meter", 0.01);
  public final LoggedTunableNumber XY_STD_DEV_THRESHOLD_3 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 3 meter", 0.01);
  public final LoggedTunableNumber XY_STD_DEV_THRESHOLD_4 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 4 meter", 0.01);
  public final LoggedTunableNumber XY_STD_DEV_THRESHOLD_5 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 5 meter", 7);
  public final LoggedTunableNumber XY_STD_DEV_THRESHOLD_6 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 6 meter", 10);

  public final LoggedTunableNumber THETA_STD_DEV_THRESHOLD_1 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 1 meter", 0.01);
  public final LoggedTunableNumber THETA_STD_DEV_THRESHOLD_2 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 2 meter", 0.01);
  public final LoggedTunableNumber THETA_STD_DEV_THRESHOLD_3 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 3 meter", 0.01);
  public final LoggedTunableNumber THETA_STD_DEV_THRESHOLD_4 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 4 meter", 0.01);
  public final LoggedTunableNumber THETA_STD_DEV_THRESHOLD_5 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 5 meter", 7);
  public final LoggedTunableNumber THETA_STD_DEV_THRESHOLD_6 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 6 meter", 10);

  public final LoggedTunableNumber MULTITAG_DISTRIBUTION =
      new LoggedTunableNumber("Vision/multitagDistribution", 0.65);

  AprilTagFieldLayout aprilTagFieldLayout;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = x -> {};
  private List<TimestampedVisionUpdate> visionUpdates;
  private Supplier<Pose2d> poseSupplier = Pose2d::new;

  /**
   * Creates a new VisionSubsystem with the specified cameras.
   *
   * @param cameras The PhotonVision cameras to use for AprilTag detection
   * @throws IOException If the field layout cannot be loaded
   */
  public VisionSubsystem(PhotonCamera... cameras) throws IOException {
    this.cameras = cameras;
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      Logger.recordOutput("Vision/FieldLayoutLoadError", e.getMessage());
    }
  }

  /**
   * Sets the data interfaces for pose estimation integration.
   *
   * @param poseSupplier Supplier that provides the current robot pose
   * @param visionConsumer Consumer that accepts vision measurement updates
   */
  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
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
        continue;
      }

      double timestamp = unprocessedResult.getTimestampSeconds();
      Logger.recordOutput(LOGGING_KEY_PREFIX + instanceIndex + " Timestamp", timestamp);

      boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().isPresent();

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

        Logger.recordOutput("Photon/Camera Pose (Multi tag) " + instanceIndex, cameraPose);
      } else {
        // If not using multitag, disambiguate and then use
        PhotonTrackedTarget target = unprocessedResult.targets.get(0);

        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isEmpty()) {
          continue;
        }

        Pose3d tagPos = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();

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
        Logger.recordOutput("Photon/Camera Pose (Single Tag) " + instanceIndex, cameraPose);
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

      if (shouldUseMultiTag) {
        xyStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
        thetaStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
      } else {
        xyStdDev = XY_STD_DEV_MODEL.predict(avgDistance);
        thetaStdDev = THETA_STD_DEV_MODEL.predict(avgDistance);
      }

      if (shouldUseMultiTag) {
        visionUpdates.add(
            new TimestampedVisionUpdate(
                robotPose,
                timestamp,
                VecBuilder.fill(
                    STD_DEV_SCALAR_SHOOTING * THETA_STD_DEV_COEFFICIENT_SHOOTING * xyStdDev,
                    STD_DEV_SCALAR_SHOOTING * THETA_STD_DEV_COEFFICIENT_SHOOTING * xyStdDev,
                    STD_DEV_SCALAR_SHOOTING * THETA_STD_DEV_COEFFICIENT_SHOOTING * thetaStdDev)));
      } else {
        visionUpdates.add(
            new TimestampedVisionUpdate(
                robotPose,
                timestamp,
                VecBuilder.fill(
                    singleTagAdjustment * xyStdDev * STD_DEV_SCALAR_SHOOTING,
                    singleTagAdjustment * xyStdDev * STD_DEV_SCALAR_SHOOTING,
                    singleTagAdjustment * thetaStdDev * STD_DEV_SCALAR_SHOOTING)));

        Logger.recordOutput("VisionData/" + instanceIndex, robotPose);
        Logger.recordOutput("Photon/Tags Used " + instanceIndex, tagPose3ds.size());
      }
    }

    // Apply all vision updates to pose estimator
    visionConsumer.accept(visionUpdates);
  }
}
