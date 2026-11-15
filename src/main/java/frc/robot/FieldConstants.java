package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Use this file as an example file that allows for referencing positions on a field
public final class FieldConstants {

  public static final Pose2d APRILTAG_1 =
      new Pose2d(11.878056, 7.424674, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_2 =
      new Pose2d(11.915394, 4.638040, Rotation2d.fromDegrees(90));
  public static final Pose2d APRILTAG_3 =
      new Pose2d(11.311890, 4.390136, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_4 =
      new Pose2d(11.311890, 4.034536, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_5 =
      new Pose2d(11.915394, 3.431286, Rotation2d.fromDegrees(270));
  public static final Pose2d APRILTAG_6 =
      new Pose2d(11.878056, 0.644398, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_7 =
      new Pose2d(11.952986, 0.644398, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_8 =
      new Pose2d(12.270994, 3.431286, Rotation2d.fromDegrees(270));
  public static final Pose2d APRILTAG_9 =
      new Pose2d(12.519152, 3.678936, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_10 =
      new Pose2d(12.519152, 4.034536, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_11 =
      new Pose2d(12.270994, 4.638040, Rotation2d.fromDegrees(90));
  public static final Pose2d APRILTAG_12 =
      new Pose2d(11.952986, 7.424674, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_13 =
      new Pose2d(16.533368, 7.403338, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_14 =
      new Pose2d(16.533368, 6.971538, Rotation2d.fromDegrees(90));
  public static final Pose2d APRILTAG_15 =
      new Pose2d(16.532860, 4.323588, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_16 =
      new Pose2d(16.532860, 3.891788, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_17 =
      new Pose2d(4.663186, 0.644398, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_18 =
      new Pose2d(4.625594, 3.431286, Rotation2d.fromDegrees(270));
  public static final Pose2d APRILTAG_19 =
      new Pose2d(5.229098, 3.678936, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_20 =
      new Pose2d(5.229098, 4.034536, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_21 =
      new Pose2d(4.625594, 4.638040, Rotation2d.fromDegrees(90));
  public static final Pose2d APRILTAG_22 =
      new Pose2d(4.663186, 7.424674, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_23 =
      new Pose2d(4.588256, 7.424674, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_24 =
      new Pose2d(4.269994, 4.638040, Rotation2d.fromDegrees(90));
  public static final Pose2d APRILTAG_25 =
      new Pose2d(4.021836, 4.390136, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_26 =
      new Pose2d(4.021836, 3.678936, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_27 =
      new Pose2d(4.269994, 3.431286, Rotation2d.fromDegrees(270));
  public static final Pose2d APRILTAG_28 =
      new Pose2d(4.269994, 2.923286, Rotation2d.fromDegrees(180));
  public static final Pose2d APRILTAG_29 =
      new Pose2d(0.007620, 0.665988, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_30 =
      new Pose2d(0.007620, 1.097788, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_31 =
      new Pose2d(0.008128, 3.745738, Rotation2d.fromDegrees(0));
  public static final Pose2d APRILTAG_32 =
      new Pose2d(0.008128, 4.177538, Rotation2d.fromDegrees(0));

  public static final Pose2d BLUE_GOAL_CENTER = new Pose2d(4.65, 4.02, Rotation2d.kZero);
  public static final Pose2d RED_GOAL_CENTER = new Pose2d(11.91, 4.02, Rotation2d.kZero);

  // Y+ blue alliance passing target
  public static final Pose2d BLUE_TOP_FERRY_TARGET =
      new Pose2d(APRILTAG_32.getX() + 0, 6.02, Rotation2d.kZero);
  // Y- blue alliance passing target
  public static final Pose2d BLUE_BOTTOM_FERRY_TARGET =
      new Pose2d(APRILTAG_32.getX() + 0, 2.02, Rotation2d.kZero);
  // Y+ red alliance passing target
  public static final Pose2d RED_TOP_FERRY_TARGET =
      new Pose2d(APRILTAG_16.getX() - 0, 6.02, Rotation2d.kZero);
  // Y- red alliance passing target
  public static final Pose2d RED_BOTTOM_FERRY_TARGET =
      new Pose2d(APRILTAG_16.getX() - 0, 2.02, Rotation2d.kZero);

  public static final double BLUE_SHOOTING_ZONE_END = APRILTAG_20.getX() + 0.5;
  public static final double NEUTRAL_ZONE_BLUESIDE = APRILTAG_20.getX() + 0.5;
  public static final double NEUTRAL_ZONE_REDSIDE = APRILTAG_4.getX() - 0.5;
  public static final double RED_SHOOTING_ZONE_START = APRILTAG_4.getX() - 0.5;
  public static final double TOP_BOTTOM_SPLIT_Y = APRILTAG_20.getY();

  public static final double BUMPZONE_START_BLUE = APRILTAG_26.getX();
  public static final double BUMPZONE_END_BLUE = APRILTAG_20.getX();
  public static final double BUMPZONE_START_RED = APRILTAG_4.getX();
  public static final double BUMPZONE_END_RED = APRILTAG_10.getX();

  public static final Pose2d RED_LEFT_CLIMBER_ALIGN =
      new Pose2d(15.46, 3.58, Rotation2d.fromDegrees(0));
  public static final Pose2d RED_RIGHT_CLIMBER_ALIGN =
      new Pose2d(15.49, 5.04, Rotation2d.fromDegrees(-180));
  public static final Pose2d BLUE_LEFT_CLIMBER_ALIGN =
      new Pose2d(1.03, 4.50, Rotation2d.fromDegrees(-180));
  public static final Pose2d BLUE_RIGHT_CLIMBER_ALIGN =
      new Pose2d(1.01, 3.03, Rotation2d.fromDegrees(0));
}
