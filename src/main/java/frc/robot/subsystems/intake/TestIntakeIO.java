package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface TestIntakeIO extends Subsystem {

  @Override
  void periodic();

  double slapdownConvert(double deg);

  double rollerConvert(double motorRot);

  void kindaIn();

  void deploy();

  void bringIn();

  void brake();

  boolean withinTolerance(double deg);
}
