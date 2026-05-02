package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  private final List<LEDStream> streams = new ArrayList<>();
  private Constants.LEDMode lastLEDMode = null;
  private final CANdle candle = new CANdle(Constants.CANDLE_ID, new CANBus("drive"));

  /**
   * Initally defines the modeToAnimation HashMap. This makes it easier to modify the code as it
   * condenses one absolutely massive switch statement to a slightly less massive HashMap. The two
   * data types are 1. the LEDModes defined in the Constants file and 2. the light animations
   * possible
   */
  private final Map<Constants.LEDMode, Supplier<com.ctre.phoenix6.controls.ControlRequest>>
      modeToAnimation = new HashMap<>();

  /**
   * Constructor intializes modeToAnimation. Commented out modes reflect the commented out modes in
   * the switch.
   */
  public LEDSubsystem() {

    /**
     * For the disabled mode, we need to check battery voltage to determine which animation to use,
     * so a method reference is used
     */
    modeToAnimation.put(Constants.LEDMode.DISABLED, this::getDisabledAnimation);
    modeToAnimation.put(Constants.LEDMode.RAINBOW, () -> Constants.LED_ANIMATION_RAINBOW);
    modeToAnimation.put(Constants.LEDMode.AUTO, () -> Constants.LED_ANIMATION_AUTO);
    modeToAnimation.put(Constants.LEDMode.ACTIVE, () -> Constants.LED_ANIMATION_ACTIVE);
    modeToAnimation.put(Constants.LEDMode.INACTIVE, () -> Constants.LED_ANIMATION_INACTIVE);
    modeToAnimation.put(
        Constants.LEDMode.CLOSE_TO_NEXT_SHIFT, () -> Constants.LED_ANIMATION_CLOSE_TO_NEXT_SHIFT);
    modeToAnimation.put(
        Constants.LEDMode.CLOSE_TO_NEXT_SHIFT_US,
        () -> Constants.LED_ANIMATION_CLOSE_TO_NEXT_SHIFT_US);
    modeToAnimation.put(
        Constants.LEDMode.CLOSE_TO_NEXT_SHIFT_NOTUS,
        () -> Constants.LED_ANIMATION_CLOSE_TO_NEXT_SHIFT_NOTUS);
    modeToAnimation.put(Constants.LEDMode.ENDGAME, () -> Constants.LED_ANIMATION_ENDGAME);
  }

  // Returns the appropriate disables animation based on battery voltage
  private ControlRequest getDisabledAnimation() {
    double voltage = RobotController.getBatteryVoltage();
    if (voltage >= 12.6) {
      return Constants.LED_ANIMATION_DISABLED_GOOD;
    } else if (voltage > 12.2) {
      return Constants.LED_ANIMATION_DISABLED_FINE;
    } else {
      return Constants.LED_ANIMATION_DISABLED_BAD;
    }
  }

  // Adds an LEDStream to the list of periodically checked streams.
  public void addStream(LEDStream stream) {
    streams.add(stream);
  }

  @Override
  public void periodic() {

    // Find the stream with the highest priority that is currently active
    LEDStream highest =
        streams.stream()
            .filter(LEDStream::isActive)
            .max(Comparator.comparingInt(s -> s.priority))
            .orElse(null);

    if (highest != null) {
      Constants.LEDMode currentMode = highest.getLEDMode();
      // Only apply mode if it changed to avoid repeatedly calling setControl()
      if (lastLEDMode != currentMode) {
        applyMode(currentMode);
        lastLEDMode = currentMode;
      }
    }
  }

  /**
   * Checks if mode is valid and applies the corresponding animation to the CANdle. The status of
   * the method is retured through smart dashboard.
   */
  private void applyMode(Constants.LEDMode ledMode) {
    String ledModeString = ledMode.toString();
    if (modeToAnimation.containsKey(ledMode)) {
      try {
        candle.setControl(modeToAnimation.get(ledMode).get());
        Logger.recordOutput("LEDs/CurrentMode", ledModeString);
      } catch (Exception e) {
        Logger.recordOutput("LEDs/Error", "Failed to apply mode: " + ledModeString);
      }
    } else {
      Logger.recordOutput(
          "LED/Error", "LED mode not configured in modeToAnimation map: " + ledModeString);
    }
  }
}
