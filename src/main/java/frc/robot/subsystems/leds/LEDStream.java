package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class LEDStream {

  public String name;
  public int priority;
  private Supplier<Constants.LEDMode> ledModeSupplier;
  private Supplier<Boolean> activeSupplier = () -> false;

  public boolean useBrightnessSupplier = false;
  public DoubleSupplier brightnessSupplier;

  public boolean useFramerateSupplier = false;
  public DoubleSupplier framerateSupplier;

  private double expirationTime = -1;
  private boolean timedActive = false;
  private boolean active = false;

  // Builder Methods
  public LEDStream withName(String name) {
    this.name = name;
    return this;
  }

  public LEDStream withPriority(int priority) {
    this.priority = priority;
    return this;
  }

  public LEDStream withLEDModeSupplier(Supplier<Constants.LEDMode> ledModeSupplier) {
    this.ledModeSupplier = ledModeSupplier;
    return this;
  }

  public LEDStream withActiveSupplier(Supplier<Boolean> activeSupplier) {
    this.activeSupplier = activeSupplier;
    return this;
  }

  public LEDStream withBrightnessSupplier(DoubleSupplier brightnessSupplier) {
    this.brightnessSupplier = brightnessSupplier;
    this.useBrightnessSupplier = true;
    return this;
  }

  public LEDStream withFramerateSupplier(DoubleSupplier framerateSupplier) {
    this.framerateSupplier = framerateSupplier;
    this.useFramerateSupplier = true;
    return this;
  }

  /**
   * Runs this stream for a fixed number of seconds. This effectively acts as a new condition for
   * the LEDStream, instead of checking the activeSupplier it checks if the timer has expired or
   * not.
   */
  public void runForSeconds(double seconds) {
    expirationTime = Timer.getFPGATimestamp() + seconds;
    timedActive = true;
  }

  // Runs this stream indefinately.
  public void run() {
    active = true;
  }

  private boolean isExpired() {
    return timedActive && Timer.getFPGATimestamp() > expirationTime;
  }

  public boolean isActive() {

    // When running on time, active state is based on expired/not
    if (timedActive) {
      if (isExpired()) {
        timedActive = false;
        expirationTime = -1;
        return false;
      }
      return true;

      // When running indefinitely, active state is based on the active variable
    } else if (active) {
      return true;
    }

    // Otherwise actually use the condition
    return activeSupplier.get();
  }

  public Constants.LEDMode getLEDMode() {
    return ledModeSupplier.get();
  }

  public void forceOff() {
    timedActive = false;
    active = false;
  }
}
