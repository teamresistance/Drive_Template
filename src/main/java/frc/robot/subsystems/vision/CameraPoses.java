package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CameraPoses {
  public static final Pose3d[] poses =
      new Pose3d[] {
        // Front Left
        new Pose3d(
            new Translation3d(0.256, 0.213, 0.267), // Right camera translation (X, Y, Z)
            new Rotation3d(0.0, Units.degreesToRadians(-12.63), Units.degreesToRadians(-15.0))),
        // Front Right
        new Pose3d(
            new Translation3d(0.268, -0.213, 0.267), // Left camera translation (X, Y, Z)
            new Rotation3d(
                0.0,
                Units.degreesToRadians(-12.63),
                Units.degreesToRadians(45.0))), // in radians btw
        //        // Back Right
        new Pose3d(
            new Translation3d(-0.235, -0.235, 0.267), // Left camera translation (X, Y, Z)
            new Rotation3d(
                0.0,
                Units.degreesToRadians(-12.63),
                Units.degreesToRadians(-45.0 - 90.0))), // in radians btw

        // front_center
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(12.5 - 0.9375),
                Units.inchesToMeters(3.0),
                Units.inchesToMeters(6.5)), // Left camera translation (X, Y, Z)
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-23.0),
                Units.degreesToRadians(-1.0))), // in radians btw

        //         Back Left
        new Pose3d(
            new Translation3d(-0.235, 0.235, 0.267), // Right camera translation (X, Y, Z)
            new Rotation3d(
                0.0, Units.degreesToRadians(-12.63), Units.degreesToRadians(45.0 + 90.0))),
      };
}
