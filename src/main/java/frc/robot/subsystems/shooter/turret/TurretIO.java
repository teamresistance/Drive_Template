package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public interface TurretIO extends Subsystem {
  @Override
  void periodic();

  // Subsystem functionalities
  void setTargetTurretPos(double degs);

  void setTargetFlywheeelSpeed(double rpm);

  // Conversion methods
  default double degsToShooterTurretRevs(double degs) {
    return Constants.SHOOTER_TURRET_REVS_PER_DEG * degs;
  }

  default double shooterTurretRevsToDegs(double revs) {
    return revs / Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }
}
