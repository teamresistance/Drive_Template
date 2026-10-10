package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// the front of the robot is the tower side. left side of the robot is the intake.

public class CameraPoses {
  public static final Pose3d[] poses =
      new Pose3d[] {
        // Front Left
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(11.67),
                Units.inchesToMeters(11.46),
                Units.inchesToMeters(10.88)),
            new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(14.6))),
        // Front Right
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(11.67),
                Units.inchesToMeters(-11.46),
                Units.inchesToMeters(10.88)),
            new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(-14.6))),
        // Back Right
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(-11.67),
                Units.inchesToMeters(-11.46),
                Units.inchesToMeters(10.88)),
            new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(14.6 + 180))),
        // Back Left
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(-11.67),
                Units.inchesToMeters(11.46),
                Units.inchesToMeters(10.88)),
            new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(-14.6 + 180))),
      };
  public static final Pose3d[] limelightPoses =
      new Pose3d[] {
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(-3.25),
                Units.inchesToMeters(-27.5 / 2),
                Units.inchesToMeters(18.125)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(10), Units.degreesToRadians(90)))
      };
}
