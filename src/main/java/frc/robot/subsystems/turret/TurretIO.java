package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface TurretIO extends Subsystem {

  @Override
  void periodic();

  double hoodAngle(double deg);

  double turretAngle(double deg);

  void swivel(double deg);

  void setHood(double deg);

  void flywheelVelocity(double velocity);
}
