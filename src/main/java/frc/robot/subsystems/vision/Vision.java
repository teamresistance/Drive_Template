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

public class Vision extends SubsystemBase {
  private static final double fieldBorderMargin = 0.5;
  private static final Pose3d[] cameraPoses = CameraPoses.cameraPoses;

  private final PhotonCamera[] cameras;
  /* For shooting vs. path following in auto */
  //  private double stdDevScalarShooting = 0.65;

  //  private PolynomialRegression xyStdDevModel =
  //      new PolynomialRegression(
  //          new double[] {1, 2, 3, 4, 5, 6}, new double[] {0.01, 0.01, 0.01, 0.01, 7, 10}, 2);
  //  private PolynomialRegression thetaStdDevModel =
  //      new PolynomialRegression(
  //          new double[] {1, 2, 3, 4, 5, 6}, new double[] {0.01, 0.01, 0.01, 0.01, 7, 10}, 2);

  /* For shooting vs. path following in auto */
  private double stdDevScalarShooting = 1.6;
  private final double thetaStdDevCoefficientShooting = 0.075;
  private PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358, 6.0
          },
          new double[] {0.005, 0.0135, 0.016, 0.028, 0.0815, 2.4, 3.62, 5.7, 5.9, 5.3, 20.0, 25.0},
          2);
  private PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358, 6
          },
          new double[] {
            0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.089, 2.027, 3.459, 4.629, 6.068, 13.0
          },
          1);

  public final LoggedTunableNumber xyStdDevThreshold_1 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 1 meter", 0.01);
  public final LoggedTunableNumber xyStdDevThreshold_2 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 2 meter", 0.01);
  public final LoggedTunableNumber xyStdDevThreshold_3 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 3 meter", 0.01);
  public final LoggedTunableNumber xyStdDevThreshold_4 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 4 meter", 0.01);
  public final LoggedTunableNumber xyStdDevThreshold_5 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 5 meter", 7);
  public final LoggedTunableNumber xyStdDevThreshold_6 =
      new LoggedTunableNumber("Vision/xyStdDevThreshold 6 meter", 10);

  public final LoggedTunableNumber thetaStdDevThreshold_1 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 1 meter", 0.01);
  public final LoggedTunableNumber thetaStdDevThreshold_2 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 2 meter", 0.01);
  public final LoggedTunableNumber thetaStdDevThreshold_3 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 3 meter", 0.01);
  public final LoggedTunableNumber thetaStdDevThreshold_4 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 4 meter", 0.01);
  public final LoggedTunableNumber thetaStdDevThreshold_5 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 5 meter", 7);
  public final LoggedTunableNumber thetaStdDevThreshold_6 =
      new LoggedTunableNumber("Vision/thetaStdDevThreshold 6 meter", 10);

  public final LoggedTunableNumber multitagDistrubution =
      new LoggedTunableNumber("Vision/multitagDistrubution", 0.65);

  AprilTagFieldLayout aprilTagFieldLayout;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private List<TimestampedVisionUpdate> visionUpdates;
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();

  public Vision(PhotonCamera... cameras) throws IOException {
    this.cameras = cameras;
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException ignored) {
    }
  }

  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
    Pose2d currentPose = poseSupplier.get();
    visionUpdates = new ArrayList<>();
    //
    //    if (Constants.TUNING_MODE) {
    //      xyStdDevModel =
    //          new PolynomialRegression(
    //              new double[] {1, 2, 3, 4, 5, 6},
    //              new double[] {
    //                xyStdDevThreshold_1.get(),
    //                xyStdDevThreshold_2.get(),
    //                xyStdDevThreshold_3.get(),
    //                xyStdDevThreshold_4.get(),
    //                xyStdDevThreshold_5.get(),
    //                xyStdDevThreshold_6.get()
    //              },
    //              2);
    //
    //      thetaStdDevModel =
    //          new PolynomialRegression(
    //              new double[] {1, 2, 3, 4, 5, 6},
    //              new double[] {
    //                thetaStdDevThreshold_1.get(),
    //                thetaStdDevThreshold_2.get(),
    //                thetaStdDevThreshold_3.get(),
    //                thetaStdDevThreshold_4.get(),
    //                thetaStdDevThreshold_5.get(),
    //                thetaStdDevThreshold_6.get()
    //              },
    //              2);
    //      stdDevScalarShooting = multitagDistrubution.get();
    //    }

    double singleTagAdjustment = 1.0;
    if (Constants.TUNING_MODE) SingleTagAdjustment.updateLoggedTagAdjustments();

    // Loop through all the cameras
    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {
      // Camera-specific variables
      Pose3d cameraPose;
      Pose2d robotPose;
      List<Pose3d> tagPose3ds = new ArrayList<>();

      List<PhotonPipelineResult> unprocessedResults = cameras[instanceIndex].getAllUnreadResults();
      if (unprocessedResults.isEmpty()) return;
      PhotonPipelineResult unprocessedResult =
          unprocessedResults.get(unprocessedResults.size() - 1);

      // if (unprocessedResults.size() > 2) {
      //   unprocessedResults =
      //       unprocessedResults.subList(unprocessedResults.size() - 2,
      // unprocessedResults.size());
      // }

      // Logger.recordOutput(
      //     "Photon/Camera " + instanceIndex + "ResultsLength", unprocessedResults.size());

      // for (PhotonPipelineResult unprocessedResult : unprocessedResults) {
      Logger.recordOutput(
          "Photon/Camera " + instanceIndex + " Has Targets", unprocessedResult.hasTargets());
      Logger.recordOutput(
          "Photon/Camera " + instanceIndex + "LatencyMS",
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
      Logger.recordOutput("Photon/Camera " + instanceIndex + " Timestamp", timestamp);

      boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().isPresent();

      if (shouldUseMultiTag) {
        // If multitag, use directly
        cameraPose =
            GeomUtil.transform3dToPose3d(
                unprocessedResult.getMultiTagResult().get().estimatedPose.best);

        robotPose =
            cameraPose
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();

        // Populate array of tag poses with tags used
        for (int id : unprocessedResult.getMultiTagResult().get().fiducialIDsUsed) {
          tagPose3ds.add(aprilTagFieldLayout.getTagPose(id).get());
        }

        Logger.recordOutput("Photon/Camera Pose (Multi tag) " + instanceIndex, cameraPose);
      } else {
        // If not using multitag, disambiugate and then use
        PhotonTrackedTarget target = unprocessedResult.targets.get(0);

        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isEmpty()) {
          continue;
        }

        Pose3d tagPos = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();

        Pose3d cameraPose0 = tagPos.transformBy(target.getBestCameraToTarget().inverse());
        Pose3d cameraPose1 = tagPos.transformBy(target.getAlternateCameraToTarget().inverse());
        Pose2d robotPose0 =
            cameraPose0
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();
        Pose2d robotPose1 =
            cameraPose1
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();

        double projectionError = target.getPoseAmbiguity();

        // Select a pose using projection error and current rotation
        if (projectionError < 0.15) {
          cameraPose = cameraPose0;
          robotPose = robotPose0;
        } else if (Math.abs(robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
            < Math.abs(robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())) {
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
      if (robotPose.getX() < -fieldBorderMargin
          || robotPose.getX() > aprilTagFieldLayout.getFieldLength() + fieldBorderMargin
          || robotPose.getY() < -fieldBorderMargin
          || robotPose.getY() > aprilTagFieldLayout.getFieldWidth() + fieldBorderMargin) {
        continue;
      }

      // Calculate average distance to tag
      double totalDistance = 0.0;
      for (Pose3d tagPose : tagPose3ds) {
        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
      }
      double avgDistance = totalDistance / tagPose3ds.size();
      double xyStdDev = 0.0;
      double thetaStdDev = 0.0;

      if (shouldUseMultiTag) {
        xyStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
        thetaStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
      } else {
        xyStdDev = xyStdDevModel.predict(avgDistance);
        thetaStdDev = thetaStdDevModel.predict(avgDistance);
      }

      if (shouldUseMultiTag) {
        visionUpdates.add(
            new TimestampedVisionUpdate(
                robotPose,
                timestamp,
                VecBuilder.fill(
                    stdDevScalarShooting * thetaStdDevCoefficientShooting * xyStdDev,
                    stdDevScalarShooting * thetaStdDevCoefficientShooting * xyStdDev,
                    stdDevScalarShooting * thetaStdDevCoefficientShooting * thetaStdDev)));
      } else {
        visionUpdates.add(
            new TimestampedVisionUpdate(
                robotPose,
                timestamp,
                VecBuilder.fill(
                    singleTagAdjustment * xyStdDev * stdDevScalarShooting,
                    singleTagAdjustment * xyStdDev * stdDevScalarShooting,
                    singleTagAdjustment * thetaStdDev * stdDevScalarShooting)));

        Logger.recordOutput("VisionData/" + instanceIndex, robotPose);
        Logger.recordOutput("Photon/Tags Used " + instanceIndex, tagPose3ds.size());
      }
    }

    // Apply all vision updates to pose estimator
    visionConsumer.accept(visionUpdates);
  }
}
