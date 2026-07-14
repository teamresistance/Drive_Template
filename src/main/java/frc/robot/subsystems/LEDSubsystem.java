package frc.robot.subsystems;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  // hardware
  private final CANdle candle = new CANdle(Constants.LED_CANDLE_ID, Constants.LED_CANDLE_BUS);
  private final int LED_START_INDEX = 0;
  private final int LED_END_INDEX = 151;

  // tracking
  private Constants.LED_MODE ledMode = Constants.LED_MODE.DISABLED;
  private double lastBrightness = -1;

  // base animations
  private final SolidColor ANIM_DISABLED =
      new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255 / 2, 165 / 2, 0));
  private final RainbowAnimation ANIM_ENABLED =
      new RainbowAnimation(LED_START_INDEX, LED_END_INDEX).withFrameRate(50);

  public void setModeDisabled() {
    if (ledMode == Constants.LED_MODE.DISABLED) return;

    ledMode = Constants.LED_MODE.DISABLED;
    candle.setControl(ANIM_DISABLED);
  }

  public void setModeEnabled(double brightness) {
    if (Math.abs(brightness - lastBrightness) < 0.05) return;
    // if (ledMode == Constants.LED_MODE.ENABLED) return;
    // TODO: This should be uncommented for LED cases unless the animation can change, like here.

    lastBrightness = brightness;
    candle.setControl(new EmptyAnimation(0));
    candle.setControl(ANIM_ENABLED.withBrightness(brightness));
    ledMode = Constants.LED_MODE.ENABLED;
    Logger.recordOutput("LEDS/Enabled/Brightness", brightness);
    Logger.recordOutput("LEDS/Mode", ledMode);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LEDS/Active", ledMode);
  }
}
