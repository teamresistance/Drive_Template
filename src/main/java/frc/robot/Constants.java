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
}
