package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public interface HoodIO extends Subsystem {
  @Override
  void periodic();

  // Subsystem functionalities
  void setTarget(double degs);

  // Conversion methods
  default double degsToShooterHoodRevs(double degs) {
    return Constants.SHOOTER_HOOD_REVS_PER_DEG * degs;
  }

  default double shooterHoodRevsToDegs(double revs) {
    return revs / Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }
}
