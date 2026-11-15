// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  private TalonFX turretMotor = new TalonFX(Constants.SHOOTER_TURRET_ID);
  private TalonFX flywheelMotor1 = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID_1);
  private TalonFX flywheelMotor2 = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID_2);

  /** Creates a new TurretReal. */
  public TurretReal() {
    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kS = 0.0;
    cfg.Slot0.kV = 0.0;
    cfg.Slot0.kP = 4.5;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    turretMotor.getConfigurator().apply(cfg);

    var cfg1 = new TalonFXConfiguration();

    cfg1.Slot0.kS = 0.0;
    cfg1.Slot0.kV = 0.0;
    cfg1.Slot0.kP = 0.0;
    cfg1.Slot0.kI = 0.0;
    cfg1.Slot0.kD = 0.0;

    flywheelMotor1.getConfigurator().apply(cfg1);
    flywheelMotor2.getConfigurator().apply(cfg1);
  }

  @Override
  public double degToMotorRevs(double deg) {
    return deg * Constants.SHOOTER_TURRET_REVS_PER_DEG;
  }

  @Override
  public void toDeg(double deg) {
    if (deg > 135) deg = 135;
    if (deg < -135) deg = -135;

    turretMotor.setControl(new PositionVoltage(degToMotorRevs(deg)));
  }

  public void setFlywheelSpeed(double speed) {
    flywheelMotor1.setControl(new VelocityVoltage(speed));
    flywheelMotor2.setControl(
        new Follower(Constants.SHOOTER_FLYWHEEL_ID_1, MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(
        "Turret/Angle",
        turretMotor.getPosition().getValueAsDouble() / Constants.SHOOTER_TURRET_REVS_PER_DEG);
  }
}
