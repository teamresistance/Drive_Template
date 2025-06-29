package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

// Use this file as an example file that allows for referencing positions on a field
public final class FieldConstants {

  // -------------------------------------------------
  // Blue Alliance Offset Positions
  // -------------------------------------------------

  public static final Pose2d OFFSET_TAG_17 = new Pose2d(3.603, 2.504, Rotation2d.fromDegrees(60));
  public static final Pose2d OFFSET_TAG_18 = new Pose2d(2.728, 3.971, Rotation2d.fromDegrees(0));
  public static final Pose2d OFFSET_TAG_19 = new Pose2d(3.562, 5.523, Rotation2d.fromDegrees(-60));
  public static final Pose2d OFFSET_TAG_20 = new Pose2d(5.322, 5.577, Rotation2d.fromDegrees(-120));
  public static final Pose2d OFFSET_TAG_21 = new Pose2d(6.21, 4.051, Rotation2d.fromDegrees(180));
  public static final Pose2d OFFSET_TAG_22 = new Pose2d(5.415, 2.528, Rotation2d.fromDegrees(120));

  // -------------------------------------------------
  // Red Alliance Offset Positions
  // -------------------------------------------------

  public static final Pose2d OFFSET_TAG_7 = new Pose2d(14.82, 4.08, Rotation2d.fromDegrees(180.0));
  public static final Pose2d OFFSET_TAG_8 = new Pose2d(13.94, 5.54, Rotation2d.fromDegrees(-120));
  public static final Pose2d OFFSET_TAG_9 = new Pose2d(12.132, 5.523, Rotation2d.fromDegrees(-60));
  public static final Pose2d OFFSET_TAG_10 = new Pose2d(11.297, 3.971, Rotation2d.fromDegrees(0));
  public static final Pose2d OFFSET_TAG_11 = new Pose2d(12.225, 2.474, Rotation2d.fromDegrees(60));
  public static final Pose2d OFFSET_TAG_6 = new Pose2d(13.985, 2.528, Rotation2d.fromDegrees(120));

  public static final Pose2d INNER_CAGE_RED =
      new Pose2d(9.786, 3.02, Rotation2d.fromDegrees(90.0)); // deprecated, using CageSelectCmd now
  public static final Pose2d BLUE_RIGHT_HOME_CORNER =
      new Pose2d(0.457, 0.457, Rotation2d.fromDegrees(0.000));
  public static final Pose2d APRILTAG_18 = new Pose2d(3.200, 3.874, Rotation2d.fromDegrees(0.000));
  public static final Pose2d BLUE_A_TREE = new Pose2d(3.200, 4.039, Rotation2d.fromDegrees(0.000));

  // -------------------------------------------------
  // Blue Alliance Side
  // -------------------------------------------------
  public static final Pose2d BLUE_B_TREE = new Pose2d(3.200, 3.708, Rotation2d.fromDegrees(0.000));
  public static final Pose2d APRILTAG_17 = new Pose2d(3.977, 2.833, Rotation2d.fromDegrees(60.000));
  public static final Pose2d BLUE_C_TREE = new Pose2d(3.834, 2.916, Rotation2d.fromDegrees(60.000));
  public static final Pose2d BLUE_D_TREE = new Pose2d(4.120, 2.751, Rotation2d.fromDegrees(60.000));
  public static final Pose2d APRILTAG_22 =
      new Pose2d(5.266, 2.986, Rotation2d.fromDegrees(120.000));
  public static final Pose2d BLUE_E_TREE =
      new Pose2d(5.123, 2.903, Rotation2d.fromDegrees(120.000));
  public static final Pose2d BLUE_F_TREE =
      new Pose2d(5.409, 3.068, Rotation2d.fromDegrees(120.000));
  public static final Pose2d APRILTAG_21 =
      new Pose2d(5.778, 4.178, Rotation2d.fromDegrees(180.000));
  public static final Pose2d BLUE_G_TREE =
      new Pose2d(5.778, 4.013, Rotation2d.fromDegrees(180.000));
  public static final Pose2d BLUE_H_TREE =
      new Pose2d(5.778, 4.343, Rotation2d.fromDegrees(180.000));
  public static final Pose2d APRILTAG_20 =
      new Pose2d(5.002, 5.218, Rotation2d.fromDegrees(-120.000));
  public static final Pose2d BLUE_I_TREE =
      new Pose2d(5.145, 5.136, Rotation2d.fromDegrees(-120.000));
  public static final Pose2d BLUE_J_TREE =
      new Pose2d(4.859, 5.301, Rotation2d.fromDegrees(-120.000));
  public static final Pose2d APRILTAG_19 =
      new Pose2d(3.713, 5.066, Rotation2d.fromDegrees(-60.000));
  public static final Pose2d BLUE_K_TREE =
      new Pose2d(3.856, 5.149, Rotation2d.fromDegrees(-60.000));
  public static final Pose2d BLUE_L_TREE =
      new Pose2d(3.570, 4.984, Rotation2d.fromDegrees(-60.000));
  public static final Pose2d RED_RIGHT_CORNER =
      new Pose2d(17.091, 7.595, Rotation2d.fromDegrees(0.000));
  public static final Pose2d APRILTAG_7 =
      new Pose2d(14.348, 4.178, Rotation2d.fromDegrees(180.000));
  public static final Pose2d RED_A_TREE =
      new Pose2d(14.348, 4.013, Rotation2d.fromDegrees(180.000));

  // -------------------------------------------------
  // Red Alliance Side
  // -------------------------------------------------
  public static final Pose2d RED_B_TREE =
      new Pose2d(14.348, 4.343, Rotation2d.fromDegrees(180.000));
  public static final Pose2d APRILTAG_8 =
      new Pose2d(13.571, 5.218, Rotation2d.fromDegrees(-120.000));
  public static final Pose2d RED_C_TREE =
      new Pose2d(13.714, 5.136, Rotation2d.fromDegrees(-120.000));
  public static final Pose2d RED_D_TREE =
      new Pose2d(13.428, 5.301, Rotation2d.fromDegrees(-120.000));
  public static final Pose2d APRILTAG_9 =
      new Pose2d(12.282, 5.066, Rotation2d.fromDegrees(-60.000));
  public static final Pose2d RED_E_TREE =
      new Pose2d(12.425, 5.149, Rotation2d.fromDegrees(-60.000));
  public static final Pose2d RED_F_TREE =
      new Pose2d(12.139, 4.984, Rotation2d.fromDegrees(-60.000));
  public static final Pose2d APRILTAG_10 = new Pose2d(11.770, 3.874, Rotation2d.fromDegrees(0.000));
  public static final Pose2d RED_G_TREE = new Pose2d(11.770, 4.039, Rotation2d.fromDegrees(0.000));
  public static final Pose2d RED_H_TREE = new Pose2d(11.770, 3.708, Rotation2d.fromDegrees(0.000));
  public static final Pose2d APRILTAG_11 =
      new Pose2d(12.546, 2.833, Rotation2d.fromDegrees(60.000));
  public static final Pose2d RED_I_TREE = new Pose2d(12.403, 2.916, Rotation2d.fromDegrees(60.000));
  public static final Pose2d RED_J_TREE = new Pose2d(12.689, 2.751, Rotation2d.fromDegrees(60.000));
  public static final Pose2d APRILTAG_6 =
      new Pose2d(13.835, 2.986, Rotation2d.fromDegrees(120.000));
  public static final Pose2d RED_K_TREE =
      new Pose2d(13.692, 2.903, Rotation2d.fromDegrees(120.000));
  public static final Pose2d RED_L_TREE =
      new Pose2d(13.978, 3.068, Rotation2d.fromDegrees(120.000));
  public static Optional<DriverStation.Alliance> ally;

  // Prevent instantiation
  private FieldConstants() {
    // Deliberately empty - prevents instantiation of constants class
  }

  public static AllianceTreePlace getAllianceBranchFromBranch(Place place) {
    ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == DriverStation.Alliance.Red) {
        switch (place) {
          case A_TREE:
            return AllianceTreePlace.RED_A_TREE;
          case B_TREE:
            return AllianceTreePlace.RED_B_TREE;
          case C_TREE:
            return AllianceTreePlace.RED_C_TREE;
          case D_TREE:
            return AllianceTreePlace.RED_D_TREE;
          case E_TREE:
            return AllianceTreePlace.RED_E_TREE;
          case F_TREE:
            return AllianceTreePlace.RED_F_TREE;
          case G_TREE:
            return AllianceTreePlace.RED_G_TREE;
          case H_TREE:
            return AllianceTreePlace.RED_H_TREE;
          case I_TREE:
            return AllianceTreePlace.RED_I_TREE;
          case J_TREE:
            return AllianceTreePlace.RED_J_TREE;
          case K_TREE:
            return AllianceTreePlace.RED_K_TREE;
          case L_TREE:
            return AllianceTreePlace.RED_L_TREE;
          default:
            return AllianceTreePlace.RED_A_TREE;
        }
      } else if (ally.get() == DriverStation.Alliance.Blue) {
        switch (place) {
          case A_TREE:
            return AllianceTreePlace.BLUE_A_TREE;
          case B_TREE:
            return AllianceTreePlace.BLUE_B_TREE;
          case C_TREE:
            return AllianceTreePlace.BLUE_C_TREE;
          case D_TREE:
            return AllianceTreePlace.BLUE_D_TREE;
          case E_TREE:
            return AllianceTreePlace.BLUE_E_TREE;
          case F_TREE:
            return AllianceTreePlace.BLUE_F_TREE;
          case G_TREE:
            return AllianceTreePlace.BLUE_G_TREE;
          case H_TREE:
            return AllianceTreePlace.BLUE_H_TREE;
          case I_TREE:
            return AllianceTreePlace.BLUE_I_TREE;
          case J_TREE:
            return AllianceTreePlace.BLUE_J_TREE;
          case K_TREE:
            return AllianceTreePlace.BLUE_K_TREE;
          case L_TREE:
            return AllianceTreePlace.BLUE_L_TREE;
          default:
            return AllianceTreePlace.BLUE_A_TREE;
        }
      }
    }
    return null;
  }

  // -------------------------------------------------

  public static Pose2d getOffsetApriltagFromTree(AllianceTreePlace tree) {
    return switch (tree) {
      case RED_A_TREE -> OFFSET_TAG_7;
      case RED_B_TREE -> OFFSET_TAG_7;
      case RED_C_TREE -> OFFSET_TAG_8;
      case RED_D_TREE -> OFFSET_TAG_8;
      case RED_E_TREE -> OFFSET_TAG_9;
      case RED_F_TREE -> OFFSET_TAG_9;
      case RED_G_TREE -> OFFSET_TAG_10;
      case RED_H_TREE -> OFFSET_TAG_10;
      case RED_I_TREE -> OFFSET_TAG_11;
      case RED_J_TREE -> OFFSET_TAG_11;
      case RED_K_TREE -> OFFSET_TAG_6;
      case RED_L_TREE -> OFFSET_TAG_6;
      case BLUE_A_TREE -> OFFSET_TAG_18;
      case BLUE_B_TREE -> OFFSET_TAG_18;
      case BLUE_C_TREE -> OFFSET_TAG_17;
      case BLUE_D_TREE -> OFFSET_TAG_17;
      case BLUE_E_TREE -> OFFSET_TAG_22;
      case BLUE_F_TREE -> OFFSET_TAG_22;
      case BLUE_G_TREE -> OFFSET_TAG_21;
      case BLUE_H_TREE -> OFFSET_TAG_21;
      case BLUE_I_TREE -> OFFSET_TAG_20;
      case BLUE_J_TREE -> OFFSET_TAG_20;
      case BLUE_K_TREE -> OFFSET_TAG_19;
      case BLUE_L_TREE -> OFFSET_TAG_19;
    };
  }

  public static Pose2d getApriltagFromTree(AllianceTreePlace tree) {
    return switch (tree) {
      case RED_A_TREE -> APRILTAG_7;
      case RED_B_TREE -> APRILTAG_7;
      case RED_C_TREE -> APRILTAG_8;
      case RED_D_TREE -> APRILTAG_8;
      case RED_E_TREE -> APRILTAG_9;
      case RED_F_TREE -> APRILTAG_9;
      case RED_G_TREE -> APRILTAG_10;
      case RED_H_TREE -> APRILTAG_10;
      case RED_I_TREE -> APRILTAG_11;
      case RED_J_TREE -> APRILTAG_11;
      case RED_K_TREE -> APRILTAG_6;
      case RED_L_TREE -> APRILTAG_6;
      case BLUE_A_TREE -> APRILTAG_18;
      case BLUE_B_TREE -> APRILTAG_18;
      case BLUE_C_TREE -> APRILTAG_17;
      case BLUE_D_TREE -> APRILTAG_17;
      case BLUE_E_TREE -> APRILTAG_22;
      case BLUE_F_TREE -> APRILTAG_22;
      case BLUE_G_TREE -> APRILTAG_21;
      case BLUE_H_TREE -> APRILTAG_21;
      case BLUE_I_TREE -> APRILTAG_20;
      case BLUE_J_TREE -> APRILTAG_20;
      case BLUE_K_TREE -> APRILTAG_19;
      case BLUE_L_TREE -> APRILTAG_19;
    };
  }

  // -------------------------------------------------

  public static Pose2d getSetPoint(Place place) {
    Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == DriverStation.Alliance.Red) {
        return switch (place) {
          case A_TREE -> RED_A_TREE;
          case B_TREE -> RED_B_TREE;
          case C_TREE -> RED_C_TREE;
          case D_TREE -> RED_D_TREE;
          case E_TREE -> RED_E_TREE;
          case F_TREE -> RED_F_TREE;
          case G_TREE -> RED_G_TREE;
          case H_TREE -> RED_H_TREE;
          case I_TREE -> RED_I_TREE;
          case J_TREE -> RED_J_TREE;
          case K_TREE -> RED_K_TREE;
          case L_TREE -> RED_L_TREE;
          case LEFT_CORAL_STATION -> new Pose2d();
          case RIGHT_CORAL_STATION -> new Pose2d();
          case ALGAE_STATION -> new Pose2d();
          case LEFT_CAGE -> new Pose2d();
          case RIGHT_CAGE -> new Pose2d();
          case MIDDLE_CAGE -> new Pose2d();
        };
      }
      if (ally.get() == DriverStation.Alliance.Blue) {
        return switch (place) {
          case A_TREE -> BLUE_A_TREE;
          case B_TREE -> BLUE_B_TREE;
          case C_TREE -> BLUE_C_TREE;
          case D_TREE -> BLUE_D_TREE;
          case E_TREE -> BLUE_E_TREE;
          case F_TREE -> BLUE_F_TREE;
          case G_TREE -> BLUE_G_TREE;
          case H_TREE -> BLUE_H_TREE;
          case I_TREE -> BLUE_I_TREE;
          case J_TREE -> BLUE_J_TREE;
          case K_TREE -> BLUE_K_TREE;
          case L_TREE -> BLUE_L_TREE;
          case LEFT_CORAL_STATION -> new Pose2d();
          case RIGHT_CORAL_STATION -> new Pose2d();
          case ALGAE_STATION -> new Pose2d();
          case LEFT_CAGE -> new Pose2d();
          case RIGHT_CAGE -> new Pose2d();
          case MIDDLE_CAGE -> new Pose2d();
        };
      }
    }
    return null;
  }

  public enum AllianceTreePlace {
    RED_A_TREE,
    RED_B_TREE,
    RED_C_TREE,
    RED_D_TREE,
    RED_E_TREE,
    RED_F_TREE,
    RED_G_TREE,
    RED_H_TREE,
    RED_I_TREE,
    RED_J_TREE,
    RED_K_TREE,
    RED_L_TREE,
    BLUE_A_TREE,
    BLUE_B_TREE,
    BLUE_C_TREE,
    BLUE_D_TREE,
    BLUE_E_TREE,
    BLUE_F_TREE,
    BLUE_G_TREE,
    BLUE_H_TREE,
    BLUE_I_TREE,
    BLUE_J_TREE,
    BLUE_K_TREE,
    BLUE_L_TREE,
  }

  public enum Place {
    A_TREE,
    B_TREE,
    C_TREE,
    D_TREE,
    E_TREE,
    F_TREE,
    G_TREE,
    H_TREE,
    I_TREE,
    J_TREE,
    K_TREE,
    L_TREE,
    LEFT_CORAL_STATION,
    RIGHT_CORAL_STATION,
    ALGAE_STATION,
    LEFT_CAGE,
    RIGHT_CAGE,
    MIDDLE_CAGE,
  }
}
