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
  private TalonFX slapdownMotor = new TalonFX(1);
  private TalonFX rollerMotor = new TalonFX(2);

  /** Creates a new Intake. */
  public IntakeReal() {
    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kS = 0.0;
    cfg.Slot0.kV = 0.0;
    cfg.Slot0.kP = 0.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    slapdownMotor.getConfigurator().apply(cfg);

    var cfg1 = new TalonFXConfiguration();

    cfg1.Slot0.kS = 0.0;
    cfg1.Slot0.kV = 0.0;
    cfg1.Slot0.kP = 0.0;
    cfg1.Slot0.kI = 0.0;
    cfg1.Slot0.kD = 0.0;

    rollerMotor.getConfigurator().apply(cfg1);

    slapdownMotor.setPosition(slapDegToRot(-30));
    register();
  }

  @Override
  public double motorRotToRoller(double motorRot) {
    return motorRot * Constants.MOTOR_CONVERSION_TO_ROLLER;
  }

  @Override
  public double slapDegToRot(double deg) {
    return deg * Constants.SLAP_CONVERSION_TO_ROT;
  }

  @Override
  public void out() {
    slapdownMotor.setControl(new PositionVoltage(slapDegToRot(90)));
    rollerMotor.setControl(new VelocityVoltage(motorRotToRoller(30)));
  }

  @Override
  public void middle() {
    slapdownMotor.setControl(new PositionVoltage(slapDegToRot(20)));
    rollerMotor.setControl(new CoastOut());
  }

  @Override
  public void in() {
    slapdownMotor.setControl(new PositionVoltage(slapDegToRot(-30)));
    rollerMotor.setControl(new CoastOut());
  }

  @Override
  public void brake() {
    slapdownMotor.setControl(new StaticBrake());
  }

  @Override
  public boolean withinTolerance(double deg) {
    double pos = slapdownMotor.getPosition().getValueAsDouble();
    return (((pos * 360 / 30) >= deg - 0.25) && ((pos * 360 / 30) <= deg + 0.25));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Intake/Angle", slapdownMotor.getPosition().getValueAsDouble() * 360 / 30);
  }
}
