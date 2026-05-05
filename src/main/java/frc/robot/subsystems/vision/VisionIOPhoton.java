package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.TimestampedVisionUpdate;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public interface VisionIOPhoton extends Subsystem {

  PolynomialRegression XY_STD_DEV_MODEL =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358, 6.0
          },
          new double[] {0.005, 0.0135, 0.016, 0.028, 0.0815, 2.4, 3.62, 5.7, 5.9, 5.3, 20.0, 25.0},
          2);

  PolynomialRegression THETA_STD_DEV_MODEL =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358, 6
          },
          new double[] {
            0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.089, 2.027, 3.459, 4.629, 6.068, 13.0
          },
          1);

  double MULTITAG_STD_DEV_SCALAR = 0.075;
  double stdDevScalarDefault = 1.0;

  /**
   * Sets the data interfaces used to integrate vision measurements into the pose estimator.
   *
   * @param poseSupplier Supplier that provides the current estimated robot pose
   * @param visionConsumer Consumer that accepts timestamped vision measurement updates
   */
  void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer);

  /**
   * Updates the global scalar applied to all vision standard deviations. Increase to trust vision
   * less; decrease to trust it more.
   *
   * @param newScalar The new standard deviation scalar
   */
  void updateStdDevScalar(double newScalar);
}
