package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class TurretReal implements TurretIO {
  private final TalonFX shooterHoodMotor = new TalonFX(Constants.SHOOTER_HOOD_ID);
  private final TalonFX shooterTurretMotor = new TalonFX(Constants.SHOOTER_TURRET_IND);
  private final TalonFX shooterFlywheelMotor1 = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID);
  private final TalonFX shooterFlywheelMotor2 = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID_2);

  public TurretReal() {
    register();
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kS = 0.0;
    cfg.Slot0.kV = 0.0;
    cfg.Slot0.kP = 0.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    shooterHoodMotor.getConfigurator().apply(cfg);

    var cfg1 = new TalonFXConfiguration();
    cfg1.Slot0.kS = 0.0;
    cfg1.Slot0.kV = 0.0;
    cfg1.Slot0.kP = 5.0;
    cfg1.Slot0.kI = 0.0;
    cfg1.Slot0.kD = 0.0;
    shooterTurretMotor.getConfigurator().apply(cfg1);

    var cfg2 = new TalonFXConfiguration();
    cfg2.Slot0.kS = 0.0;
    cfg2.Slot0.kV = 0.0;
    cfg2.Slot0.kP = 0.0;
    cfg2.Slot0.kI = 0.0;
    cfg2.Slot0.kD = 0.0;
    shooterFlywheelMotor1.getConfigurator().apply(cfg2);

    var cfg3 = new TalonFXConfiguration();
    cfg3.Slot0.kS = 0.0;
    cfg3.Slot0.kV = 0.0;
    cfg3.Slot0.kP = 0.0;
    cfg3.Slot0.kI = 0.0;
    cfg3.Slot0.kD = 0.0;
    shooterFlywheelMotor2.getConfigurator().apply(cfg3);

    shooterTurretMotor.setPosition(0);
    shooterHoodMotor.setPosition(Constants.MIN_SHOOTER_DEG * Constants.SHOOTER_HOOD_REVS_PER_DEG);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Hood/Angle",
        shooterHoodMotor.getPosition().getValueAsDouble() / Constants.SHOOTER_HOOD_REVS_PER_DEG);
    Logger.recordOutput(
        "Turret/Angle",
        shooterTurretMotor.getPosition().getValueAsDouble()
            / Constants.SHOOTER_TURRET_REVS_PER_DEG);
    Logger.recordOutput(
        "Flywheel1/Velocity", shooterFlywheelMotor1.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Flywheel2/Velocity", shooterFlywheelMotor2.getVelocity().getValueAsDouble());
  }

  @Override
  public double hoodAngle(double deg) {
    return deg * Constants.SHOOTER_HOOD_REVS_PER_DEG;
  }

  public double turretAngle(double deg) {
    return deg * Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }

  @Override
  public void swivel(double deg) {
    if (Double.isNaN(deg)) {
      return;
    }
    double limit = Math.max(-Constants.TURRET_LIMIT_DEG, Math.min(Constants.TURRET_LIMIT_DEG, deg));
    shooterTurretMotor.setControl(new PositionVoltage(turretAngle(limit)));
  }

  @Override
  public void setHood(double deg) {
    if (Double.isNaN(deg)) {
      return;
    }
    double limit = Math.max(Constants.MIN_SHOOTER_DEG, Math.min(Constants.MAX_SHOOTER_DEG, deg));
    shooterHoodMotor.setControl(new PositionVoltage(hoodAngle(limit)));
  }

  @Override
  public void flywheelVelocity(double velocity) {
    shooterFlywheelMotor1.setControl(new VelocityVoltage(velocity));
    shooterFlywheelMotor2.setControl(
        new Follower(Constants.SHOOTER_FLYWHEEL_ID, MotorAlignmentValue.Aligned));
  }
}
