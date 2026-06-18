package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.drive.SwerveDriveIO;

public class ContinuousLEDCommand extends Command {

  private final LEDSubsystem leds;
  private final SwerveDriveIO drive;

  /**
   * This command should always be running. All logic for deciding the LED mode should occur within
   * this command - therefore any subsystems that affect LEDs must be passed through this
   * constructor along with the LED subsystem.
   */
  public ContinuousLEDCommand(LEDSubsystem leds, SwerveDriveIO drive) {
    this.leds = leds;
    this.drive = drive;
    addRequirements(leds);
  }

  @Override
  public void execute() {

    // default disabled solid color
    if (DriverStation.isDisabled()) {
      leds.setModeDisabled();
      return;
    }

    // increasingly bright rainbow based on speed
    leds.setModeEnabled(
        MathUtil.clamp(
            drive.getVelocity().getTranslation().getNorm()
                / (TunerConstants.kSpeedAt12Volts.magnitude() / 2),
            0.1,
            1.0));

    // TODO: any other LED animations should be decided in this method body

  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
