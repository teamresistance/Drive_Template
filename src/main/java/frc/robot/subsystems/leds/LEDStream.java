package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

  // Creates the builder format.
  private LEDStream(LEDStreamBuilder builder) {
    this.name = builder.name;
    this.priority = builder.priority;
    this.ledModeSupplier = builder.ledModeSupplier;
    this.activeSupplier = builder.activeSupplier;
  }

  public static class LEDStreamBuilder {

    // Mandatory
    private final String name;
    private final int priority;
    private final Supplier<Constants.LEDMode> ledModeSupplier;

    // Optional
    private Supplier<Boolean> activeSupplier = () -> false;

    private DoubleSupplier brightnessSupplier;
    private boolean useBrightnessSupplier = false;

    private DoubleSupplier framerateSupplier;
    private boolean useFramerateSupplier = false;

    // Mandatory
    public LEDStreamBuilder(
        String name, int priority, Supplier<Constants.LEDMode> ledModeSupplier) {
      this.name = name;
      this.priority = priority;
      this.ledModeSupplier = ledModeSupplier;
    }

    // Optional
    public LEDStreamBuilder withActiveSupplier(Supplier<Boolean> activeSupplier) {
      this.activeSupplier = activeSupplier;
      return this;
    }

    public LEDStreamBuilder withBrightnessSupplier(DoubleSupplier brightnessSupplier) {
      this.brightnessSupplier = brightnessSupplier;
      this.useBrightnessSupplier = true;
      return this;
    }

    public LEDStreamBuilder withFramerateSupplier(DoubleSupplier framerateSupplier) {
      this.framerateSupplier = framerateSupplier;
      this.useFramerateSupplier = true;
      return this;
    }

    // Build method to create the LEDStream
    public LEDStream build() {
      return new LEDStream(this);
    }
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

    // When running on time, active state is based on expired/not
    if (timedActive) {
      if (isExpired()) {
        timedActive = false;
        expirationTime = -1;
        return false;
      }
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
  }
}
