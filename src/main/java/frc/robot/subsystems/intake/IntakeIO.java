package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public interface IntakeIO extends Subsystem {
  @Override
  void periodic();

  // Subsystem functionalities
  void deploySlapdown();

  void brakeSlapdown();

  void kindaIn();

  void fullyIn();

  void startRoller();

  void stopRoller();

  // Conversion methods
  default double degreesToSlapMotorRotations(double degrees) {
    return degrees / Constants.SLAPDOWN_ROTATIONS_TO_DEGREES;
  }

  default double slapMotorRotationsToDegrees(double slapMotorRotations) {
    return slapMotorRotations * Constants.SLAPDOWN_ROTATIONS_TO_DEGREES;
  }

  // Tolerance
  boolean withinTolerance(double target);
}
