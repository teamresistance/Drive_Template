package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// TODO:
//  Convert this to a builder format?
//  Example: new LEDStream().withName(...).withPriority(...).withLEDModeSupplier(...)
public class LEDStream {

  public final String name;
  public final int priority;
  private final Supplier<Constants.LEDMode> ledModeSupplier;
  private final Supplier<Boolean> activeSupplier;

  public boolean useBrightnessSupplier = false;
  public DoubleSupplier brightnessSupplier;

  public boolean useFramerateSupplier = false;
  public DoubleSupplier framerateSupplier;

  private double expirationTime = -1;
  private boolean timedActive = false;

  public LEDStream(
      String name,
      int priority,
      Supplier<Constants.LEDMode> ledModeSupplier,
      Supplier<Boolean> activeSupplier) {

    this.name = name;
    this.priority = priority;
    this.ledModeSupplier = ledModeSupplier;
    this.activeSupplier = activeSupplier;
  }

  // overload for exclusively time based streams, so () -> false doesn't need to exist
  public LEDStream(String name, int priority, Supplier<Constants.LEDMode> ledModeSupplier) {

    this.name = name;
    this.priority = priority;
    this.ledModeSupplier = ledModeSupplier;
    this.activeSupplier = () -> false;
  }

  /**
   * Runs this stream for a fixed number of seconds. This effectively acts as a new condition for
   * the LEDStream, instead of checking the activeSupplier it checks if the timer has expired or not
   */
  public void runForSeconds(double seconds) {
    expirationTime = Timer.getFPGATimestamp() + seconds;
    timedActive = true;
  }

  private boolean isExpired() {
    return timedActive && Timer.getFPGATimestamp() > expirationTime;
  }

  public boolean isActive() {

    // when running on time, active state is based on expired/not
    if (timedActive) {
      if (isExpired()) {
        timedActive = false;
        expirationTime = -1;
        return false;
      }
      return true;
    }

    // otherwise actually use the condition
    return activeSupplier.get();
  }

  public Constants.LEDMode getLEDMode() {
    return ledModeSupplier.get();
  }

  public LEDStream withBrightnessSupplier(DoubleSupplier brightnessSupplier) {
    this.brightnessSupplier = brightnessSupplier;
    useBrightnessSupplier = true;
    return this;
  }

  public LEDStream withFramerateSupplier(DoubleSupplier framerateSupplier) {
    this.framerateSupplier = framerateSupplier;
    useFramerateSupplier = true;
    return this;
  }

  public void forceOff() {
    timedActive = false;
  }
}
