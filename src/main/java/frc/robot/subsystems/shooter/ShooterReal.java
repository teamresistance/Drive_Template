// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

public class ShooterReal implements ShooterIO {
  private TalonFX turretMotor = new TalonFX(1);

  /** Creates a new ThingSubsystem. */
  public ShooterReal() {
    var cfg = new TalonFXConfiguration();

    cfg.Slot0.kS = 0.0;
    cfg.Slot0.kV = 0.0;
    cfg.Slot0.kP = 0.0;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    turretMotor.getConfigurator().apply(cfg);

    register();
  }

  private double targetDeg = 0;

  @Override
  public void setTarget(double deg) {
    turretMotor.setControl(new VelocityVoltage(deg));
    targetDeg = deg;
  }

  @Override
  public double getError() {
    return Math.abs(turretMotor.getPosition().getValueAsDouble() * 360.0 - targetDeg);
  }

  @Override
  public void startMotor() {
    turretMotor.set(1.0);
  }

  @Override
  public void stopMotor() {
    turretMotor.set(0);
  }

  @Override
  public boolean overheating() {
    return turretMotor.getDeviceTemp().getValueAsDouble() > 70;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Overheating/Status", overheating());
  }
}
