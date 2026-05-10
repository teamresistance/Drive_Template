package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.TimestampedVisionUpdate;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Each Limelight is identified by its network-table name (e.g. {@code "limelight-front"}).
 * Camera-to-robot transforms are supplied via {@link CameraPoses#poses} in the same order as the
 * name array passed to the constructor.
 */
public class VisionRealLimelight implements VisionIOLimelight {

  private static double stdDevScalar = STD_DEV_SCALAR_DEFAULT;

  private final String[] cameraNames;
  private static final Pose3d[] CAMERA_POSES = CameraPoses.limelightPoses;

  private AprilTagFieldLayout aprilTagFieldLayout;
  private Supplier<Pose2d> poseSupplier = Pose2d::new;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = x -> {};

  /**
   * @param cameraNames Name of each limelight, in same order as poses from CameraPoses
   */
  public VisionRealLimelight(String... cameraNames) {
    register();
    this.cameraNames = cameraNames;

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
    } catch (IOException e) {
      Logger.recordOutput("Limelight/FieldLayoutLoadError", e.getMessage());
    }

    // Push the camera-to-robot transform into each Limelight so its onboard
    // MegaTag pipeline can compute field-relative poses for us.
    for (int i = 0; i < cameraNames.length; i++) {
      setCameraRobotTransform(cameraNames[i], CAMERA_POSES[i]);
      LimelightHelpers.SetIMUMode(cameraNames[i], 4); // fused IMU and robot yaw
    }
  }

  @Override
  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  // MAKE SURE SIM AND REAL METHODS MATCH WHEN CHANGING!
  @Override
  public void periodic() {
    Pose2d currentPose = poseSupplier.get();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    double singleTagAdjustment = 1.0;
    if (Constants.TUNING_MODE) SingleTagAdjustment.updateLoggedTagAdjustments();

    for (int instanceIndex = 0; instanceIndex < cameraNames.length; instanceIndex++) {
      String name = cameraNames[instanceIndex];

      // push yaw to limelight
      LimelightHelpers.SetRobotOrientation(
          name, currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

      // decide which to use
      PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      PoseEstimate estimate = chooseBestEstimate(mt1, mt2);

      if (estimate == null || estimate.tagCount == 0) {
        Logger.recordOutput(LOGGING_KEY_PREFIX_LL + instanceIndex + " Has Targets", false);
        Logger.recordOutput("Limelight/Tags Used " + instanceIndex, 0);
        Logger.recordOutput("Limelight/Camera Pose " + instanceIndex, new Pose3d());
        Logger.recordOutput("Limelight/Camera" + instanceIndex + "/TagPoses", new Pose3d[0]);
        continue;
      }

      Logger.recordOutput(LOGGING_KEY_PREFIX_LL + instanceIndex + " Has Targets", true);

      boolean shouldUseMultiTag = estimate.tagCount > 1;
      Logger.recordOutput("Limelight/UsingMultitag " + instanceIndex, shouldUseMultiTag);

      double timestamp = estimate.timestampSeconds;
      Logger.recordOutput(LOGGING_KEY_PREFIX_LL + instanceIndex + " Timestamp", timestamp);

      Pose2d robotPose = estimate.pose;
      Logger.recordOutput(LOGGING_KEY_PREFIX_LL + instanceIndex + "LatencyMS", estimate.latency);

      List<Pose3d> tagPose3ds = new ArrayList<>();
      for (var rawFiducial : estimate.rawFiducials) {
        int id = rawFiducial.id;
        if (aprilTagFieldLayout != null && aprilTagFieldLayout.getTagPose(id).isPresent()) {
          tagPose3ds.add(aprilTagFieldLayout.getTagPose(id).get());
        }
      }

      // reconstruct camera pose from robot pose + camera offset
      Pose3d cameraPose =
          new Pose3d(robotPose)
              .transformBy(GeomUtil.pose3dToTransform3d(CAMERA_POSES[instanceIndex]));

      Logger.recordOutput("Limelight/Camera Pose " + instanceIndex, cameraPose);
      Logger.recordOutput(
          "Limelight/Camera" + instanceIndex + "/TagPoses", tagPose3ds.toArray(new Pose3d[0]));

      // ── Field-border rejection ────────────────────────────────────────────
      if (aprilTagFieldLayout != null
          && (robotPose.getX() < -FIELD_BORDER_MARGIN
              || robotPose.getX() > aprilTagFieldLayout.getFieldLength() + FIELD_BORDER_MARGIN
              || robotPose.getY() < -FIELD_BORDER_MARGIN
              || robotPose.getY() > aprilTagFieldLayout.getFieldWidth() + FIELD_BORDER_MARGIN)) {
        continue;
      }

      double totalDistance = 0.0;
      for (Pose3d tagPose : tagPose3ds) {
        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
      }

      // just in case
      double avgDistance =
          tagPose3ds.isEmpty() ? estimate.avgTagDist : totalDistance / tagPose3ds.size();

      double xyStdDev;
      double thetaStdDev;

      if (shouldUseMultiTag) {
        xyStdDev = Math.pow(avgDistance, 2.0) / estimate.tagCount;
        thetaStdDev = Math.pow(avgDistance, 2.0) / estimate.tagCount;
      } else {
        singleTagAdjustment =
            estimate.rawFiducials.length > 0
                ? SingleTagAdjustment.getAdjustmentForTag(estimate.rawFiducials[0].id)
                : 1.0;
        xyStdDev = XY_STD_DEV_MODEL.predict(avgDistance);
        thetaStdDev = THETA_STD_DEV_MODEL.predict(avgDistance);
      }

      // ── Add vision update ─────────────────────────────────────────────────
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
        Logger.recordOutput("Limelight/Tags Used " + instanceIndex, tagPose3ds.size());
      }
    }

    visionConsumer.accept(visionUpdates);
  }

  @Override
  public void updateStdDevScalar(double newScalar) {
    stdDevScalar = newScalar;
  }

  @Override
  public void updateCameraTransform(String name, Pose3d camPose) {
    setCameraRobotTransform(name, camPose);
  }

  /** Gives preference to MT2, uses MT1 if MT2 doesn't have a pose */
  private static PoseEstimate chooseBestEstimate(PoseEstimate mt1, PoseEstimate mt2) {
    if (mt2 != null && mt2.tagCount > 0) return mt2;
    if (mt1 != null && mt1.tagCount > 0) return mt1;
    return null;
  }

  /**
   * Pushes the camera-to-robot transform into the Limelight via {@link
   * LimelightHelpers#setCameraPose_RobotSpace} so the onboard pipeline can account for camera
   * placement when computing field-relative poses.
   *
   * <p>The Limelight expects the offset as (forward, right, up, roll, pitch, yaw) in metres and
   * degrees.
   */
  private static void setCameraRobotTransform(String name, Pose3d cameraPose) {
    Translation3d t = cameraPose.getTranslation();
    Rotation3d r = cameraPose.getRotation();
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        t.getX(),
        t.getY(),
        t.getZ(),
        Math.toDegrees(r.getX()),
        Math.toDegrees(r.getY()),
        Math.toDegrees(r.getZ()));
  }
}
