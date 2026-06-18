package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.SwerveDriveIO;
import frc.robot.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.Logger;

public class ContinuousVisionStdDevCommand extends Command {

  private final VisionIO[] vision;
  private final SwerveDriveIO drive;

  /**
   * This command should always be running. This is where the logic for applying a standard
   * deviation scalar should go - a mockery of a Kalman filter. Any subsystems used in determining
   * the scalar should also go into this command.
   */
  public ContinuousVisionStdDevCommand(SwerveDriveIO drive, VisionIO... vision) {
    this.drive = drive;
    this.vision = vision;

    // means nothing for other "visions", only needed for the one it defaults on
    for (VisionIO vsn : vision) {
      addRequirements(vsn);
    }
  }

  @Override
  public void execute() {

    double calculatedStdDevScalar = 1;

    // add between 0 and 0.3 to the scalar depending on velocity, this is FILLER, to be tuned or
    // removed
    calculatedStdDevScalar +=
        drive.getVelocity().getTranslation().getNorm()
            / TunerConstants.kSpeedAt12Volts.magnitude()
            * 0.3;

    // TODO: any other logic that affects the scalar should also go into this method body

    Logger.recordOutput("Vision/StdDevScalar", calculatedStdDevScalar);
    for (VisionIO vision : vision) {
      vision.updateStdDevScalar(calculatedStdDevScalar);
    }
  }
}
