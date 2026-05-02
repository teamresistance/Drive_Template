package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
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

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final int CANDLE_ID = 40;

  public enum LEDMode {
    RAINBOW,
    ACTIVE,
    INACTIVE,
    AUTO,
    DISABLED,
    CLOSE_TO_NEXT_SHIFT,
    CLOSE_TO_NEXT_SHIFT_US,
    CLOSE_TO_NEXT_SHIFT_NOTUS,
    ENDGAME
  }

  public static final int LED_START_INDEX = 0;
  public static final int LED_END_INDEX = 161;

  public static final RainbowAnimation LED_ANIMATION_RAINBOW =
      new RainbowAnimation(LED_START_INDEX, LED_END_INDEX).withFrameRate(60);
  public static final SolidColor LED_ANIMATION_ACTIVE =
      new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(0, 255, 0));
  public static final SolidColor LED_ANIMATION_INACTIVE =
      new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 0, 0));
  public static final StrobeAnimation LED_ANIMATION_BUMP =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
          .withFrameRate(10)
          .withColor(new RGBWColor(0, 255, 255));
  public static final LarsonAnimation LED_ANIMATION_AUTO =
      new LarsonAnimation(LED_START_INDEX, LED_END_INDEX)
          .withFrameRate(20)
          .withColor(new RGBWColor(255, 150, 0))
          .withSize(20);
  public static final EmptyAnimation LED_ANIMATION_DISABLED_GOOD = new EmptyAnimation(0);
  public static final EmptyAnimation LED_ANIMATION_DISABLED_FINE = new EmptyAnimation(0);
  public static final StrobeAnimation LED_ANIMATION_DISABLED_BAD =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
          .withColor(new RGBWColor(15, 0, 0))
          .withFrameRate(1);
  public static final StrobeAnimation LED_ANIMATION_CLOSE_TO_NEXT_SHIFT =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
          .withColor(new RGBWColor(255, 40, 0))
          .withFrameRate(4);
  public static final StrobeAnimation LED_ANIMATION_CLOSE_TO_NEXT_SHIFT_US =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
          .withColor(new RGBWColor(0, 255, 0))
          .withFrameRate(6);
  public static final StrobeAnimation LED_ANIMATION_CLOSE_TO_NEXT_SHIFT_NOTUS =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
          .withColor(new RGBWColor(255, 0, 0))
          .withFrameRate(6);
  public static final StrobeAnimation LED_ANIMATION_ENDGAME =
      new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
          .withColor(new RGBWColor(255, 255, 0))
          .withFrameRate(4);
}
