package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface HoodIO extends Subsystem {

  @Override
  void periodic();

  double angle(double deg);

  void setHood(double deg);
}
