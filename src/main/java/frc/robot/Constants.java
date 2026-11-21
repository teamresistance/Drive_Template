package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

public final class Constants {

  public static boolean TEST_MODE = false;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static final boolean TUNING_MODE = false;
  public static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          5.0,
          Units.degreesToRadians(540),
          Units.degreesToRadians(400));

  public static final Translation2d CENTER_OF_ROTATION =
      new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));

  public static final int LED_CANDLE_ID = 40;
  public static final CANBus LED_CANDLE_BUS = new CANBus("drive");

  public enum LED_MODE {
    DISABLED,
    ENABLED,
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double SLAPDOWN_MOTOR_CONVERSION = 1 / 12.0;
  public static final double ROLLER_MOTOR_CONVERSION = 3;
  public static final int MOTOR_DEVICE_ID = 12;
  public static final double SHOOTER_HOOD_REVS_PER_DEG = (4.0 * (175.0 / 10.0)) / 360.0;
  public static final double TURRET_LIMIT_DEG = 135.0;
  public static final double START_HOOD_ANGLE = 13.0;
  public static final double MIN_SHOOTER_DEG = 13.0;
  public static final double MAX_SHOOTER_DEG = 45.0;
  public static final double SHOOTER_TURRET_REVS_PER_DEG = (5 * 5 * 3.2) / 360.0;
  public static final int SHOOTER_HOOD_ID = 12;
  public static final int SHOOTER_TURRET_IND = 13;
  public static final int SHOOTER_FLYWHEEL_ID = 14;
  public static final int SHOOTER_FLYWHEEL_ID_2 = 15;
  public static final Translation2d SHOOTER_HUB = new Translation2d(0, 0);
}
