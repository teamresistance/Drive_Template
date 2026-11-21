package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class TestIntakeReal implements TestIntakeIO {
  private final TalonFX slapdownMotor = new TalonFX(1);
  private final TalonFX rollerMotor = new TalonFX(2);
  private final NeutralOut stop = new NeutralOut();

  public TestIntakeReal() {
    register();
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

    slapdownMotor.setPosition(slapdownConvert(-30));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/WithinTolerance", withinTolerance(0));
  }

  @Override
  public double slapdownConvert(double deg) {
    return deg * Constants.SLAPDOWN_MOTOR_CONVERSION;
  }

  @Override
  public double rollerConvert(double motorRot) {
    return motorRot * Constants.ROLLER_MOTOR_CONVERSION;
  }

  @Override
  public void deploy() {
    slapdownMotor.setControl(new PositionVoltage(slapdownConvert(90)));
    rollerMotor.setControl(new VelocityVoltage(rollerConvert(30)));
  }

  @Override
  public void kindaIn() {
    slapdownMotor.setControl(new PositionVoltage(slapdownConvert(20)));
    rollerMotor.setControl(new CoastOut());
  }

  @Override
  public void bringIn() {
    slapdownMotor.setControl(new PositionVoltage(slapdownConvert(-30)));
    rollerMotor.setControl(new StaticBrake());
  }

  @Override
  public void brake() {
    slapdownMotor.setControl(stop);
  }

  @Override
  public boolean withinTolerance(double deg) {
    double position = slapdownMotor.getPosition().getValueAsDouble();
    return ((position * 360 / 30) >= deg - 0.25) && ((position * 360 / 30) <= deg + 0.25);
  }
}
