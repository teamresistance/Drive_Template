package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeReal implements IntakeIO {

  // Slapdown motor (30:1 ratio)
  TalonFX slapdown = new TalonFX(0);

  // Roller motor (3:1 ratio)
  TalonFX roller = new TalonFX(1);

  // Creates a new IntakeReal.
  public IntakeReal() {
    register();

    // PID
    slapdown.setPosition(degreesToSlapMotorRotations(Constants.SLAPDOWN_FULLY_IN));
    var slapdowncfg = new TalonFXConfiguration();
    slapdowncfg.Slot0.kS = 0;
    slapdowncfg.Slot0.kV = 0;
    slapdowncfg.Slot0.kP = 0;
    slapdowncfg.Slot0.kI = 0;
    slapdowncfg.Slot0.kD = 0;
    slapdown.getConfigurator().apply(slapdowncfg);
    var rollercfg = new TalonFXConfiguration();
    rollercfg.Slot0.kS = 0;
    rollercfg.Slot0.kV = 0;
    rollercfg.Slot0.kP = 0;
    rollercfg.Slot0.kI = 0;
    rollercfg.Slot0.kD = 0;
    roller.getConfigurator().apply(slapdowncfg);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    Logger.recordOutput(
        "Intake/Slapdown", slapMotorRotationsToDegrees(slapdown.getPosition().getValueAsDouble()));
    Logger.recordOutput("Intake/Roller", roller.getVelocity().getValueAsDouble());
  }

  // Deploys the robot slapdown into active intake position
  @Override
  public void deploySlapdown() {

    // Sets the intake down to 90 degrees
    slapdown.setControl(
        new PositionVoltage(degreesToSlapMotorRotations(Constants.SLAPDOWN_DEPLOYED)));
  }

  // Stops the motor once deploy() runs
  @Override
  public void brakeSlapdown() {
    slapdown.setControl(new StaticBrake());
  }

  // Lifts the intake and makes it inactive
  @Override
  public void kindaIn() {

    // Lifts the slapdown to 20 degrees
    slapdown.setControl(
        new PositionVoltage(degreesToSlapMotorRotations(Constants.SLAPDOWN_KINDA_IN)));
  }

  // Lifts the intake to its starting position
  @Override
  public void fullyIn() {

    // Stops the roller
    roller.setControl(new CoastOut());

    // Lifts the slapdown to 20 degrees
    slapdown.setControl(
        new PositionVoltage(degreesToSlapMotorRotations(Constants.SLAPDOWN_FULLY_IN)));
  }

  // Starts the roller
  @Override
  public void startRoller() {

    // Runs the roller at 30 RPM
    roller.setControl(new VelocityVoltage(Constants.ROLLER_RPM));
  }

  // Stops the roller
  @Override
  public void stopRoller() {
    roller.setControl(new CoastOut());
  }

  // Returns whether the slapdown is within 0.25 degrees of its target position
  @Override
  public boolean withinTolerance(double target) {
    return Math.abs(slapMotorRotationsToDegrees(slapdown.getPosition().getValueAsDouble()) - target)
        <= Constants.SLAPDOWN_TOLERANCE;
  }
}
